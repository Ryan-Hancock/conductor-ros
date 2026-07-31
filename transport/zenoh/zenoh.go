//go:build zenoh

package zenoh

import (
	"crypto/rand"
	"fmt"
	"log/slog"
	"reflect"
	"sync"
	"sync/atomic"
	"time"

	"github.com/BooleanCat/option"
	zgo "github.com/eclipse-zenoh/zenoh-go/zenoh"
	zext "github.com/eclipse-zenoh/zenoh-go/zenoh/zenohext"

	conductor "conductor.dev/conductor"
	"conductor.dev/conductor/cdr"
	"conductor.dev/conductor/transport/rmwzenoh"
)

func init() {
	conductor.RegisterTransport("zenoh", New)
}

// New opens a Zenoh session connected to the router at opts.Endpoint and
// returns a Transport speaking the rmw_zenoh wire format for opts.Domain.
func New(opts conductor.TransportOptions) (conductor.Transport, error) {
	zgo.InitLoggerFromEnvOr("error")
	cfg := zgo.NewConfigDefault()
	if opts.Endpoint != "" {
		if err := cfg.InsertJson5("connect/endpoints", fmt.Sprintf("[%q]", opts.Endpoint)); err != nil {
			return nil, fmt.Errorf("zenoh config: %w", err)
		}
	}
	session, err := zgo.Open(cfg, nil)
	if err != nil {
		return nil, fmt.Errorf("zenoh open (is a router running at %s?): %w", opts.Endpoint, err)
	}
	return &transport{
		session: session,
		zid:     session.ZId().String(),
		domain:  opts.Domain,
		nids:    map[string]int{},
	}, nil
}

type transport struct {
	mu         sync.Mutex
	session    zgo.Session
	zid        string
	domain     int
	nextNID    int
	nextEID    int
	nids       map[string]int
	tokens     []zgo.LivelinessToken
	subs       []zgo.Subscriber
	advSubs    []zext.AdvancedSubscriber
	pubs       []*zgo.Publisher
	advPubs    []*zext.AdvancedPublisher
	queryables []zgo.Queryable
	queriers   []*zgo.Querier
}

// declareToken registers a liveliness token so the entity shows up in the
// ROS graph (ros2 node list, ros2 topic list/info). Callers hold t.mu.
func (t *transport) declareToken(keyexpr string) error {
	ke, err := zgo.NewKeyExpr(keyexpr)
	if err != nil {
		return err
	}
	tok, err := t.session.Liveliness().DeclareToken(ke, nil)
	if err != nil {
		return err
	}
	t.tokens = append(t.tokens, tok)
	return nil
}

func (t *transport) DeclareNode(name string) error {
	t.mu.Lock()
	defer t.mu.Unlock()
	nid := t.nextNID
	t.nextNID++
	t.nids[name] = nid
	eid := t.nextEID
	t.nextEID++
	return t.declareToken(rmwzenoh.NodeToken(t.domain, t.zid, nid, eid, name))
}

func (t *transport) endpointIDs(node string) (nid, eid int) {
	nid = t.nids[node]
	eid = t.nextEID
	t.nextEID++
	return nid, eid
}

func (t *transport) Publisher(spec conductor.TopicSpec) (func(any, conductor.Metadata) error, error) {
	info, ok := conductor.MessageInfoOf(spec.Type)
	if !ok {
		return nil, fmt.Errorf("message type %s is not registered; call conductor.RegisterMessage", spec.Type)
	}
	ke, err := zgo.NewKeyExpr(rmwzenoh.TopicKeyexpr(t.domain, spec.Topic, info.DDSType(), info.Hash))
	if err != nil {
		return nil, err
	}
	// A transient-local publisher keeps a cache its late subscribers can query:
	// that is what durability *is* on this transport, and it is how rmw_zenoh
	// implements it, so a ROS subscriber gets our history and ours gets theirs.
	put := func(payload []byte, att []byte) error { return nil }
	var basePub *zgo.Publisher
	var advPub *zext.AdvancedPublisher
	if spec.QoS.Durability == conductor.TransientLocal {
		opts := &zext.AdvancedPublisherOptions{
			PublisherDetection: true,
			Cache:              option.Some(zext.AdvancedPublisherCacheOptions{MaxSamples: uint(depthOf(spec.QoS))}),
		}
		if spec.QoS.Reliability == conductor.Reliable {
			// Lets a subscriber notice it missed a sample and ask the cache for
			// it. Sporadic keeps the background traffic down, as rmw_zenoh does.
			opts.SampleMissDetection = option.Some(zext.AdvancedPublisherSampleMissDetectionOptions{
				HeartbeatMode: zext.HeartbeatModeSporadic(heartbeatPeriodMs),
			})
		}
		p, err := zext.Ext(&t.session).DeclareAdvancedPublisher(ke, opts)
		if err != nil {
			return nil, err
		}
		advPub = &p
		put = func(payload, att []byte) error {
			return advPub.Put(zgo.NewZBytes(payload), &zext.AdvancedPublisherPutOptions{
				Attachment: option.Some(zgo.NewZBytes(att)),
			})
		}
	} else {
		p, err := t.session.DeclarePublisher(ke, nil)
		if err != nil {
			return nil, err
		}
		basePub = &p
		put = func(payload, att []byte) error {
			return basePub.Put(zgo.NewZBytes(payload), &zgo.PublisherPutOptions{
				Attachement: option.Some(zgo.NewZBytes(att)),
			})
		}
	}

	t.mu.Lock()
	if basePub != nil {
		t.pubs = append(t.pubs, basePub)
	} else {
		t.advPubs = append(t.advPubs, advPub)
	}
	nid, eid := t.endpointIDs(spec.Node)
	err = t.declareToken(rmwzenoh.EndpointToken(t.domain, t.zid, nid, eid,
		rmwzenoh.EntityPublisher, spec.Node, "/"+spec.Topic, info.DDSType(), info.Hash, spec.QoS))
	t.mu.Unlock()
	if err != nil {
		return nil, err
	}

	var gid [16]byte
	if _, err := rand.Read(gid[:]); err != nil {
		return nil, err
	}
	var seq atomic.Int64
	return func(msg any, md conductor.Metadata) error {
		payload, err := cdr.Marshal(msg)
		if err != nil {
			return err
		}
		att := rmwzenoh.EncodeAttachment(seq.Add(1), time.Now().UnixNano(), gid[:])
		if md.Trace.Valid() {
			att = rmwzenoh.AppendTraceContext(att, md.Trace.TraceID, md.Trace.SpanID, md.Trace.Sampled)
		}
		return put(payload, att)
	}, nil
}

// heartbeatPeriodMs is how often a reliable transient-local publisher announces
// its latest sequence number, so a subscriber can notice a gap and ask for what
// it missed. rmw_zenoh uses the same idea; the period only costs traffic when
// nothing else is being published.
const heartbeatPeriodMs = 1000

// depthOf is the history a transient-local endpoint keeps or asks for.
func depthOf(q conductor.QoS) int {
	if q.Depth <= 0 {
		return 1
	}
	return q.Depth
}

func (t *transport) Subscribe(spec conductor.TopicSpec, deliver func(any, conductor.Metadata)) error {
	info, ok := conductor.MessageInfoOf(spec.Type)
	if !ok {
		return fmt.Errorf("message type %s is not registered; call conductor.RegisterMessage", spec.Type)
	}
	ke, err := zgo.NewKeyExpr(rmwzenoh.TopicKeyexpr(t.domain, spec.Topic, info.DDSType(), info.Hash))
	if err != nil {
		return err
	}
	msgType := spec.Type
	topic := spec.Topic
	onSample := zgo.Closure[zgo.Sample]{Call: func(s zgo.Sample) {
		ptr := reflect.New(msgType)
		if err := cdr.Unmarshal(s.Payload().Bytes(), ptr.Interface()); err != nil {
			slog.Warn("conductor/zenoh: dropping undecodable message", "topic", topic, "err", err)
			return
		}
		var md conductor.Metadata
		if att := s.Attachement(); att.IsSome() {
			if traceID, spanID, sampled, ok := rmwzenoh.ExtractTraceContext(att.Unwrap().Bytes()); ok {
				md.Trace = conductor.TraceContext{TraceID: traceID, SpanID: spanID, Sampled: sampled}
			}
		}
		deliver(ptr.Elem().Interface(), md)
	}}

	// A transient-local subscriber asks publishers for what it missed, and
	// keeps asking as late publishers appear — which is how a node that starts
	// after the transform tree was published still receives it.
	var sub zgo.Subscriber
	var advSub *zext.AdvancedSubscriber
	if spec.QoS.Durability == conductor.TransientLocal {
		opts := &zext.AdvancedSubscriberOptions{
			SubscriberDetection: true,
			History: option.Some(zext.AdvancedSubscriberHistoryOptions{
				DetectLatePublishers: true,
				MaxSamples:           uint(depthOf(spec.QoS)),
			}),
		}
		if spec.QoS.Reliability == conductor.Reliable {
			opts.Recovery = option.Some(zext.AdvancedSubscriberRecoveryOptions{
				LastSampleMissDetection: zext.LastSampleMissDetectionModeHeartbeat(),
			})
		}
		s, err := zext.Ext(&t.session).DeclareAdvancedSubscriber(ke, onSample, opts)
		if err != nil {
			return err
		}
		advSub = &s
	} else {
		s, err := t.session.DeclareSubscriber(ke, onSample, nil)
		if err != nil {
			return err
		}
		sub = s
	}

	t.mu.Lock()
	defer t.mu.Unlock()
	if advSub != nil {
		t.advSubs = append(t.advSubs, *advSub)
	} else {
		t.subs = append(t.subs, sub)
	}
	nid, eid := t.endpointIDs(spec.Node)
	return t.declareToken(rmwzenoh.EndpointToken(t.domain, t.zid, nid, eid,
		rmwzenoh.EntitySubscription, spec.Node, "/"+spec.Topic, info.DDSType(), info.Hash, spec.QoS))
}

func (t *transport) Serve(spec conductor.ServiceSpec, handle func(any) (any, error)) error {
	info, ok := conductor.ServiceInfoOf(spec.ReqType, spec.ResType)
	if !ok {
		return fmt.Errorf("service type (%s, %s) is not registered; generate it with conductor msggen or call conductor.RegisterService",
			spec.ReqType, spec.ResType)
	}
	keStr := rmwzenoh.TopicKeyexpr(t.domain, spec.Service, info.DDSType(), info.Hash)
	ke, err := zgo.NewKeyExpr(keStr)
	if err != nil {
		return err
	}
	reqType := spec.ReqType
	service := spec.Service
	// The handler blocks a zenoh callback thread until the node's executor
	// produces the response; queries are loaned, so replying must happen
	// inside the callback.
	q, err := t.session.DeclareQueryable(ke, zgo.Closure[zgo.Query]{Call: func(query zgo.Query) {
		payload := query.Payload()
		if !payload.IsSome() {
			slog.Warn("conductor/zenoh: service query without payload", "service", service)
			return
		}
		reqPtr := reflect.New(reqType)
		if err := cdr.Unmarshal(payload.Unwrap().Bytes(), reqPtr.Interface()); err != nil {
			slog.Warn("conductor/zenoh: dropping undecodable request", "service", service, "err", err)
			return
		}
		var seq int64
		var gid []byte
		if att := query.Attachement(); att.IsSome() {
			if s, _, g, err := rmwzenoh.DecodeAttachment(att.Unwrap().Bytes()); err == nil {
				seq, gid = s, g
			}
		}
		res, err := handle(reqPtr.Elem().Interface())
		if err != nil {
			query.ReplyErr(zgo.NewZBytesFromString(err.Error()), nil)
			return
		}
		out, err := cdr.Marshal(res)
		if err != nil {
			slog.Warn("conductor/zenoh: response marshal failed", "service", service, "err", err)
			query.ReplyErr(zgo.NewZBytesFromString("response serialization failed"), nil)
			return
		}
		att := rmwzenoh.EncodeAttachment(seq, time.Now().UnixNano(), gid)
		if err := query.Reply(ke, zgo.NewZBytes(out), &zgo.QueryReplyOptions{
			Attachement: option.Some(zgo.NewZBytes(att)),
		}); err != nil {
			slog.Warn("conductor/zenoh: reply failed", "service", service, "err", err)
		}
	}}, &zgo.QueryableOptions{Complete: true})
	if err != nil {
		return err
	}

	t.mu.Lock()
	defer t.mu.Unlock()
	t.queryables = append(t.queryables, q)
	nid, eid := t.endpointIDs(spec.Node)
	return t.declareToken(rmwzenoh.EndpointToken(t.domain, t.zid, nid, eid,
		rmwzenoh.EntityService, spec.Node, "/"+spec.Service, info.DDSType(), info.Hash, defaultQoS()))
}

func (t *transport) ServiceClient(spec conductor.ServiceSpec) (func(any, time.Duration) (any, error), error) {
	info, ok := conductor.ServiceInfoOf(spec.ReqType, spec.ResType)
	if !ok {
		return nil, fmt.Errorf("service type (%s, %s) is not registered; generate it with conductor msggen or call conductor.RegisterService",
			spec.ReqType, spec.ResType)
	}
	keStr := rmwzenoh.TopicKeyexpr(t.domain, spec.Service, info.DDSType(), info.Hash)
	ke, err := zgo.NewKeyExpr(keStr)
	if err != nil {
		return nil, err
	}
	// Zenoh queriers carry their own deadline (10s by default), which is
	// shorter than many legitimate calls — an action's get_result blocks for
	// the whole goal. Honour the declared timeout, with headroom so the
	// caller's own timeout fires first and reports the better error.
	// Consolidation must be NONE, matching rmw_zenoh: the default (LATEST)
	// deduplicates replies by key expression, and since every reply to a ROS
	// service arrives on the one service keyexpr, it can swallow the reply
	// outright.
	opts := &zgo.QuerierOptions{
		Target:         option.Some(zgo.QueryTargetAllComplete),
		Consolidataion: option.Some(zgo.NewQueryConsolidataion(zgo.ConsolidationModeNone)),
	}
	if spec.Timeout > 0 {
		opts.TimeoutMs = uint64((spec.Timeout + spec.Timeout/10) / time.Millisecond)
	}
	querier, err := t.session.DeclareQuerier(ke, opts)
	if err != nil {
		return nil, err
	}

	t.mu.Lock()
	t.queriers = append(t.queriers, &querier)
	nid, eid := t.endpointIDs(spec.Node)
	err = t.declareToken(rmwzenoh.EndpointToken(t.domain, t.zid, nid, eid,
		rmwzenoh.EntityClient, spec.Node, "/"+spec.Service, info.DDSType(), info.Hash, defaultQoS()))
	t.mu.Unlock()
	if err != nil {
		return nil, err
	}

	var gid [16]byte
	if _, err := rand.Read(gid[:]); err != nil {
		return nil, err
	}
	var seq atomic.Int64
	resType := spec.ResType
	service := spec.Service
	q := &querier
	return func(req any, timeout time.Duration) (any, error) {
		payload, err := cdr.Marshal(req)
		if err != nil {
			return nil, err
		}
		att := rmwzenoh.EncodeAttachment(seq.Add(1), time.Now().UnixNano(), gid[:])
		replies, err := q.Get("", zgo.NewFifoChannel[zgo.Reply](16), &zgo.QuerierGetOptions{
			Payload:     option.Some(zgo.NewZBytes(payload)),
			Attachement: option.Some(zgo.NewZBytes(att)),
		})
		if err != nil {
			return nil, err
		}
		select {
		case reply, ok := <-replies:
			if !ok {
				return nil, fmt.Errorf("service %q: no server responded", service)
			}
			if errOpt := reply.Err(); errOpt.IsSome() {
				replyErr := errOpt.Unwrap()
				return nil, fmt.Errorf("service %q: %s", service, replyErr.Payload().String())
			}
			sample := reply.Ok().Unwrap()
			resPtr := reflect.New(resType)
			if err := cdr.Unmarshal(sample.Payload().Bytes(), resPtr.Interface()); err != nil {
				return nil, fmt.Errorf("service %q: undecodable response: %w", service, err)
			}
			return resPtr.Elem().Interface(), nil
		case <-time.After(timeout):
			return nil, fmt.Errorf("service %q: no response within %s", service, timeout)
		}
	}, nil
}

func defaultQoS() conductor.QoS {
	q, _ := conductor.QoSProfile("reliable")
	return q
}

func (t *transport) Start() error { return nil }

func (t *transport) Close() error {
	t.mu.Lock()
	defer t.mu.Unlock()
	for _, tok := range t.tokens {
		if err := tok.Undeclare(); err != nil {
			slog.Warn("conductor/zenoh: token undeclare", "err", err)
		}
	}
	for _, s := range t.subs {
		s.Drop()
	}
	for _, s := range t.advSubs {
		s.Drop()
	}
	for _, p := range t.pubs {
		p.Drop()
	}
	for _, p := range t.advPubs {
		p.Drop()
	}
	for _, q := range t.queryables {
		q.Drop()
	}
	for _, q := range t.queriers {
		q.Drop()
	}
	t.session.Drop()
	return nil
}
