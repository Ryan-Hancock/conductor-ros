package conductor

import (
	"context"
	"errors"
	"fmt"
	"log/slog"
	"reflect"
	"sync"
	"sync/atomic"
	"time"
)

// Goal is the handle passed to an action handler: the goal value, feedback
// publishing, and cancellation.
//
// Action handlers run on their own goroutine, one per goal — NOT on the
// node's executor, because goals are long-running. Handlers must therefore
// not touch node state that executor callbacks also use; communicate over
// the Goal handle, channels, or publishers.
type Goal[G, F any] struct {
	goal         G
	ctx          context.Context
	sendFeedback func(F)
}

// Value returns the goal.
func (g *Goal[G, F]) Value() G { return g.goal }

// Context is canceled when the client (or anyone) requests goal
// cancellation. A handler that returns a non-nil error after cancellation
// reports the goal as canceled rather than aborted.
func (g *Goal[G, F]) Context() context.Context { return g.ctx }

// Feedback publishes a feedback message for this goal.
func (g *Goal[G, F]) Feedback(f F) { g.sendFeedback(f) }

// Action declares an action server implementing the ROS 2 action protocol
// (send_goal/cancel_goal/get_result services plus feedback/status topics
// under <name>/_action/). The owning node must define a handler method named
// On<FieldName> with signature func(*Goal[G, F]) (R, error); it runs on a
// dedicated goroutine per goal. Returning an error aborts the goal (or
// completes cancellation if the goal's context was canceled); the result
// returned alongside it is still delivered to the client, which is where
// interfaces like nav2_msgs/action/NavigateToPose keep their error codes.
//
// Tags: action (required) — the action name.
type Action[G, F, R any] struct {
	name  string
	goals atomic.Uint64
}

// Name returns the wired action name (empty before Run).
func (a *Action[G, F, R]) Name() string { return a.name }

func (a *Action[G, F, R]) bind(rt *runtimeState, nr *nodeRuntime, field reflect.StructField, ownerPtr reflect.Value) error {
	name := field.Tag.Get("action")
	if name == "" {
		return errors.New(`missing action tag (e.g. action:"navigate")`)
	}
	m := ownerPtr.MethodByName("On" + field.Name)
	if !m.IsValid() {
		return fmt.Errorf("missing handler method On%s", field.Name)
	}
	h, ok := m.Interface().(func(*Goal[G, F]) (R, error))
	if !ok {
		return fmt.Errorf("On%s must have signature func(*conductor.Goal[G, F]) (R, error)", field.Name)
	}
	info, ok := ActionInfoOf(reflect.TypeFor[G](), reflect.TypeFor[F](), reflect.TypeFor[R]())
	if !ok {
		return fmt.Errorf("action type (%s, %s, %s) is not registered; generate it with conductor msggen or call conductor.RegisterAction",
			reflect.TypeFor[G](), reflect.TypeFor[F](), reflect.TypeFor[R]())
	}
	// Register the concrete generic wire types under this action's identity.
	registerMessageType(reflect.TypeFor[feedbackMessage[F]](), info.FeedbackMessage)
	registerServiceTypes(reflect.TypeFor[sendGoalRequest[G]](), reflect.TypeFor[sendGoalResponse](), info.SendGoal)
	registerServiceTypes(reflect.TypeFor[getResultRequest](), reflect.TypeFor[getResultResponse[R]](), info.GetResult)

	rt.recordProvides(nr.name, name)
	rt.recordEndpoint(Endpoint{Node: nr.name, Kind: EndpointAction, Field: field.Name, Name: name,
		Type: info.Name, count: countOf(a.goals.Load)})
	srv := &actionServer[G, F, R]{
		action:   name,
		handler:  h,
		goals:    map[uuidMsg]*goalRecord[R]{},
		accepted: &a.goals,
	}
	base := name + "/_action/"
	transientQoS, _ := QoSProfile("transient")
	reliableQoS, _ := QoSProfile("reliable")

	statusPub, err := rt.transport.Publisher(TopicSpec{
		Topic: base + "status", QoS: transientQoS,
		Type: reflect.TypeFor[goalStatusArrayMsg](), Node: nr.name,
	})
	if err != nil {
		return err
	}
	feedbackPub, err := rt.transport.Publisher(TopicSpec{
		Topic: base + "feedback", QoS: reliableQoS,
		Type: reflect.TypeFor[feedbackMessage[F]](), Node: nr.name,
	})
	if err != nil {
		return err
	}
	srv.statusPub, srv.feedbackPub = statusPub, feedbackPub

	serveSpecs := []struct {
		suffix   string
		req, res reflect.Type
		handle   func(any) (any, error)
	}{
		{"send_goal", reflect.TypeFor[sendGoalRequest[G]](), reflect.TypeFor[sendGoalResponse](), srv.handleSendGoal},
		{"get_result", reflect.TypeFor[getResultRequest](), reflect.TypeFor[getResultResponse[R]](), srv.handleGetResult},
		{"cancel_goal", reflect.TypeFor[cancelGoalRequest](), reflect.TypeFor[cancelGoalResponse](), srv.handleCancel},
	}
	for _, s := range serveSpecs {
		spec := ServiceSpec{Service: base + s.suffix, ReqType: s.req, ResType: s.res, Node: nr.name}
		if err := rt.transport.Serve(spec, s.handle); err != nil {
			return err
		}
	}
	a.name = name
	return nil
}

type goalRecord[R any] struct {
	info   goalInfoMsg
	status int8
	result R
	done   chan struct{}
	cancel context.CancelFunc
}

func (r *goalRecord[R]) terminal() bool {
	return r.status == goalStatusSucceeded || r.status == goalStatusCanceled || r.status == goalStatusAborted
}

type actionServer[G, F, R any] struct {
	action      string
	handler     func(*Goal[G, F]) (R, error)
	statusPub   func(any, Metadata) error
	feedbackPub func(any, Metadata) error

	mu    sync.Mutex
	goals map[uuidMsg]*goalRecord[R]

	// accepted counts goals for the dashboard's live inventory.
	accepted *atomic.Uint64
}

func (s *actionServer[G, F, R]) handleSendGoal(reqAny any) (any, error) {
	req := reqAny.(sendGoalRequest[G])

	s.mu.Lock()
	if _, exists := s.goals[req.GoalId]; exists {
		s.mu.Unlock()
		return sendGoalResponse{Accepted: false}, nil
	}
	s.accepted.Add(1)
	ctx, cancel := context.WithCancel(context.Background())
	rec := &goalRecord[R]{
		info:   goalInfoMsg{GoalId: req.GoalId, Stamp: time.Now()},
		status: goalStatusExecuting,
		done:   make(chan struct{}),
		cancel: cancel,
	}
	s.goals[req.GoalId] = rec
	s.mu.Unlock()
	s.publishStatus()

	goal := &Goal[G, F]{
		goal: req.Goal,
		ctx:  ctx,
		sendFeedback: func(f F) {
			if err := s.feedbackPub(feedbackMessage[F]{GoalId: req.GoalId, Feedback: f}, Metadata{}); err != nil {
				slog.Warn("conductor: feedback publish failed", "action", s.action, "err", err)
			}
		},
	}
	go func() {
		defer cancel()
		result, err := s.handler(goal)
		s.mu.Lock()
		switch {
		case err != nil && ctx.Err() != nil:
			rec.status = goalStatusCanceled
			rec.result = result // canceled goals report the handler's partial result
		case err != nil:
			// An aborted goal reports its result too. This is not a detail:
			// the interfaces in common use put the reason for failure in the
			// result and nowhere else — nav2_msgs/action/NavigateToPose
			// carries error_code and error_msg — so dropping it would leave a
			// client knowing only that something went wrong.
			rec.status = goalStatusAborted
			rec.result = result
			slog.Warn("conductor: goal aborted", "action", s.action, "err", err)
		default:
			rec.status = goalStatusSucceeded
			rec.result = result
		}
		close(rec.done)
		s.mu.Unlock()
		s.publishStatus()
		// Keep terminal goals queryable for a grace period, then drop them.
		time.AfterFunc(30*time.Second, func() {
			s.mu.Lock()
			delete(s.goals, req.GoalId)
			s.mu.Unlock()
		})
	}()
	return sendGoalResponse{Accepted: true, Stamp: rec.info.Stamp}, nil
}

func (s *actionServer[G, F, R]) handleGetResult(reqAny any) (any, error) {
	req := reqAny.(getResultRequest)
	s.mu.Lock()
	rec, ok := s.goals[req.GoalId]
	s.mu.Unlock()
	if !ok {
		return getResultResponse[R]{}, nil // STATUS_UNKNOWN
	}
	<-rec.done // blocks until the goal reaches a terminal state
	s.mu.Lock()
	defer s.mu.Unlock()
	return getResultResponse[R]{Status: rec.status, Result: rec.result}, nil
}

func (s *actionServer[G, F, R]) handleCancel(reqAny any) (any, error) {
	req := reqAny.(cancelGoalRequest)
	res := cancelGoalResponse{}

	s.mu.Lock()
	zero := req.GoalInfo.GoalId == (uuidMsg{})
	if zero {
		// Zero UUID cancels every active goal (stamp filtering not implemented).
		for _, rec := range s.goals {
			if !rec.terminal() {
				rec.status = goalStatusCanceling
				rec.cancel()
				res.GoalsCanceling = append(res.GoalsCanceling, rec.info)
			}
		}
	} else if rec, ok := s.goals[req.GoalInfo.GoalId]; !ok {
		res.ReturnCode = cancelErrUnknownGoalID
	} else if rec.terminal() {
		res.ReturnCode = cancelErrGoalTerminated
	} else {
		rec.status = goalStatusCanceling
		rec.cancel()
		res.GoalsCanceling = append(res.GoalsCanceling, rec.info)
	}
	s.mu.Unlock()

	if len(res.GoalsCanceling) > 0 {
		s.publishStatus()
	}
	return res, nil
}

func (s *actionServer[G, F, R]) publishStatus() {
	s.mu.Lock()
	arr := goalStatusArrayMsg{StatusList: make([]goalStatusMsg, 0, len(s.goals))}
	for _, rec := range s.goals {
		arr.StatusList = append(arr.StatusList, goalStatusMsg{GoalInfo: rec.info, Status: rec.status})
	}
	s.mu.Unlock()
	if err := s.statusPub(arr, Metadata{}); err != nil {
		slog.Warn("conductor: status publish failed", "action", s.action, "err", err)
	}
}
