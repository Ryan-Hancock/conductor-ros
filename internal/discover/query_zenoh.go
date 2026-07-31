//go:build zenoh

package discover

import (
	"fmt"
	"sort"
	"time"

	zgo "github.com/eclipse-zenoh/zenoh-go/zenoh"

	"conductor.dev/conductor/transport/rmwzenoh"
)

// Query asks the network what is on it. rmw_zenoh advertises every node and
// endpoint as a liveliness token, so the graph is already published; the only
// question is how to read all of it.
//
// It is read the way rmw_zenoh reads it for its own graph cache: a liveliness
// *subscriber* declared with history, which streams the tokens that already
// exist and then any that appear. A one-shot liveliness query looks simpler and
// is not equivalent — it completes when the network says it is done, which on a
// busy graph returned a different subset on every run. A subscriber plus a quiet
// period is both complete and honest about when it stopped listening.
func Query(endpoint string, domain int, timeout time.Duration) (*Graph, error) {
	zgo.InitLoggerFromEnvOr("error")
	cfg := zgo.NewConfigDefault()
	if endpoint != "" {
		if err := cfg.InsertJson5("connect/endpoints", fmt.Sprintf("[%q]", endpoint)); err != nil {
			return nil, fmt.Errorf("zenoh config: %w", err)
		}
	}
	session, err := zgo.Open(cfg, nil)
	if err != nil {
		return nil, fmt.Errorf("zenoh open (is a router running at %s?): %w", endpoint, err)
	}
	defer session.Close(nil)

	// Open returns before the session has reached the router, and a graph read
	// too early is answered by nobody — which reads as an empty graph rather
	// than as an error, and is exactly the kind of quiet half-answer this
	// command exists to remove. So wait to be connected.
	if err := awaitRouter(session, timeout); err != nil {
		return nil, err
	}

	// Scoped to the domain, because two domains on one router are two graphs
	// and merging them would invent peers that cannot hear each other.
	ke, err := zgo.NewKeyExpr(fmt.Sprintf("%s/%d/**", rmwzenoh.AdminSpace, domain))
	if err != nil {
		return nil, err
	}
	// A channel handler hands back the receive side, which is what the collect
	// loop below reads.
	samples := make(chan zgo.Sample, 4096)
	sub, err := session.Liveliness().DeclareSubscriber(ke, zgo.Closure[zgo.Sample]{
		Call: func(s zgo.Sample) {
			select {
			case samples <- s:
			default: // a graph larger than the buffer: the quiet period ends it
			}
		},
	}, &zgo.LivelinessSubscriberOptions{
		History: true, // the tokens that already exist, not just the next change
	})
	if err != nil {
		return nil, fmt.Errorf("liveliness subscriber: %w", err)
	}
	defer sub.Drop()

	g := &Graph{Domain: domain, Endpoint: endpoint}
	tokens := map[string]bool{}
	unreadable := map[string]string{}

	// Collect until the stream goes quiet: the graph is a set of tokens that
	// arrive in a burst, so "nothing new for a while" is the end of it. The
	// overall timeout is the backstop for a graph that keeps changing.
	const quiet = 400 * time.Millisecond
	deadline := time.After(timeout)
	idle := time.NewTimer(quiet)
	defer idle.Stop()

collect:
	for {
		select {
		case sample := <-samples:
			token := sample.KeyExpr().String()
			switch sample.Kind() {
			case zgo.SampleKindDelete:
				// The entity went away while we were looking: it is not part
				// of the graph as it stands.
				delete(tokens, token)
				delete(unreadable, token)
			default:
				tokens[token] = true
			}
			if !idle.Stop() {
				select {
				case <-idle.C:
				default:
				}
			}
			idle.Reset(quiet)
		case <-idle.C:
			break collect
		case <-deadline:
			break collect
		}
	}

	// Sorted so the same graph reads the same way twice: a derived externals
	// block that depends on arrival order would be a poor thing to commit.
	names := make([]string, 0, len(tokens))
	for token := range tokens {
		names = append(names, token)
	}
	sort.Strings(names)
	for _, token := range names {
		e, err := rmwzenoh.ParseToken(token)
		if err != nil {
			// A token this build cannot read is worth reporting, not worth
			// failing over: the graph is someone else's and may carry entity
			// kinds from a newer rmw_zenoh.
			unreadable[token] = err.Error()
			continue
		}
		g.Entities = append(g.Entities, e)
	}
	for token, why := range unreadable {
		g.Unreadable = append(g.Unreadable, Unreadable{Token: token, Err: why})
	}
	return g, nil
}

// awaitRouter waits for the session to be connected to a router (or, on a
// peer-to-peer graph, to any peer). It reports what it was waiting for when it
// gives up, because "no router at that endpoint" and "a graph with nothing on
// it" are different problems with different fixes.
func awaitRouter(session zgo.Session, timeout time.Duration) error {
	deadline := time.Now().Add(timeout)
	for {
		routers, err := session.RoutersZId()
		if err != nil {
			return fmt.Errorf("zenoh session: %w", err)
		}
		if len(routers) > 0 {
			return nil
		}
		peers, err := session.PeersZId()
		if err != nil {
			return fmt.Errorf("zenoh session: %w", err)
		}
		if len(peers) > 0 {
			return nil
		}
		if time.Now().After(deadline) {
			return fmt.Errorf("no zenoh router or peer answered within %s "+
				"(is rmw_zenohd running, and is the endpoint right?)", timeout)
		}
		time.Sleep(50 * time.Millisecond)
	}
}
