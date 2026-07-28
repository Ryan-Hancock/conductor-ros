package conductor

import (
	"crypto/rand"
	"encoding/hex"
	"errors"
	"fmt"
	"log/slog"
	"reflect"
	"sync"
	"time"
)

// ErrGoalRejected is returned by SendGoal when the server refuses the goal.
var ErrGoalRejected = errors.New("conductor: goal rejected by action server")

// GoalStatus mirrors action_msgs/msg/GoalStatus.
type GoalStatus int8

const (
	StatusUnknown   GoalStatus = 0
	StatusAccepted  GoalStatus = 1
	StatusExecuting GoalStatus = 2
	StatusCanceling GoalStatus = 3
	StatusSucceeded GoalStatus = 4
	StatusCanceled  GoalStatus = 5
	StatusAborted   GoalStatus = 6
)

func (s GoalStatus) String() string {
	switch s {
	case StatusAccepted:
		return "ACCEPTED"
	case StatusExecuting:
		return "EXECUTING"
	case StatusCanceling:
		return "CANCELING"
	case StatusSucceeded:
		return "SUCCEEDED"
	case StatusCanceled:
		return "CANCELED"
	case StatusAborted:
		return "ABORTED"
	default:
		return "UNKNOWN"
	}
}

// Succeeded reports whether the goal completed successfully.
func (s GoalStatus) Succeeded() bool { return s == StatusSucceeded }

// sendGoalTimeout bounds the send_goal and cancel_goal calls, which return
// promptly regardless of how long the goal itself runs.
const sendGoalTimeout = 10 * time.Second

// ActionClient calls an action server implementing the ROS 2 action protocol
// — including servers written with rclcpp/rclpy, such as Nav2.
//
// SendGoal blocks only until the server accepts or rejects; the returned
// handle carries feedback and the eventual result. Because those calls
// block, drive an ActionClient from an Action handler's goroutine or one you
// start yourself, never directly from a node's executor callback (a timer or
// subscription handler), which would stall that node.
//
// Tags: action (required), timeout (time.ParseDuration; default 5m) — the
// longest a goal may take before Result gives up.
type ActionClient[G, F, R any] struct {
	name    string
	timeout time.Duration

	sendGoal   func(any, time.Duration) (any, error)
	cancelGoal func(any, time.Duration) (any, error)
	getResult  func(any, time.Duration) (any, error)

	mu      sync.Mutex
	pending map[uuidMsg]chan F
}

// Name returns the wired action name (empty before Run).
func (c *ActionClient[G, F, R]) Name() string { return c.name }

// GoalHandle tracks one accepted goal.
type GoalHandle[F, R any] struct {
	id       uuidMsg
	feedback chan F

	cancel func() error
	result func() (R, GoalStatus, error)

	once      sync.Once
	res       R
	status    GoalStatus
	resultErr error
}

// ID returns the goal's UUID as a hex string, matching the form ros2 CLI
// prints.
func (h *GoalHandle[F, R]) ID() string { return hex.EncodeToString(h.id.Uuid[:]) }

// Feedback returns the goal's feedback stream. It is closed once Result
// returns. Sends are dropped rather than blocking the transport, so a slow
// consumer loses messages instead of stalling the graph; drain it in a
// dedicated goroutine if every message matters.
func (h *GoalHandle[F, R]) Feedback() <-chan F { return h.feedback }

// Cancel asks the server to cancel this goal. It returns once the server has
// acknowledged the request; call Result to wait for the goal to actually
// finish (normally with StatusCanceled).
func (h *GoalHandle[F, R]) Cancel() error { return h.cancel() }

// Result blocks until the goal reaches a terminal state, returning the
// result value and terminal status. err is non-nil only for transport
// failures or timeout — an aborted or canceled goal is reported through
// status, along with whatever partial result the server sent. Result must be
// called (it releases the handle's feedback subscription); calling it more
// than once returns the same values.
func (h *GoalHandle[F, R]) Result() (R, GoalStatus, error) {
	h.once.Do(func() { h.res, h.status, h.resultErr = h.result() })
	return h.res, h.status, h.resultErr
}

// SendGoal sends goal and waits for the server's accept/reject decision.
// A rejected goal returns ErrGoalRejected and a nil handle.
func (c *ActionClient[G, F, R]) SendGoal(goal G) (*GoalHandle[F, R], error) {
	if c.sendGoal == nil {
		panic("conductor: SendGoal on an action client that was not wired by Run")
	}
	var id uuidMsg
	if _, err := rand.Read(id.Uuid[:]); err != nil {
		return nil, err
	}

	// Register for feedback before sending, so nothing is missed between
	// acceptance and the first feedback message.
	fb := make(chan F, 16)
	c.mu.Lock()
	c.pending[id] = fb
	c.mu.Unlock()
	release := func() {
		c.mu.Lock()
		if _, ok := c.pending[id]; ok {
			delete(c.pending, id)
			close(fb)
		}
		c.mu.Unlock()
	}

	resAny, err := c.sendGoal(sendGoalRequest[G]{GoalId: id, Goal: goal}, sendGoalTimeout)
	if err != nil {
		release()
		return nil, fmt.Errorf("action %q: send_goal: %w", c.name, err)
	}
	if !resAny.(sendGoalResponse).Accepted {
		release()
		return nil, ErrGoalRejected
	}

	h := &GoalHandle[F, R]{id: id, feedback: fb}
	h.cancel = func() error {
		if _, err := c.cancelGoal(cancelGoalRequest{GoalInfo: goalInfoMsg{GoalId: id}}, sendGoalTimeout); err != nil {
			return fmt.Errorf("action %q: cancel_goal: %w", c.name, err)
		}
		return nil
	}
	h.result = func() (R, GoalStatus, error) {
		defer release()
		var zero R
		out, err := c.getResult(getResultRequest{GoalId: id}, c.timeout)
		if err != nil {
			return zero, StatusUnknown, fmt.Errorf("action %q: get_result: %w", c.name, err)
		}
		got := out.(getResultResponse[R])
		return got.Result, GoalStatus(got.Status), nil
	}
	return h, nil
}

func (c *ActionClient[G, F, R]) bind(rt *runtimeState, nr *nodeRuntime, field reflect.StructField, ownerPtr reflect.Value) error {
	name := field.Tag.Get("action")
	if name == "" {
		return errors.New(`missing action tag (e.g. action:"navigate")`)
	}
	timeout := 5 * time.Minute
	if tag := field.Tag.Get("timeout"); tag != "" {
		d, err := time.ParseDuration(tag)
		if err != nil || d <= 0 {
			return fmt.Errorf("invalid timeout %q", tag)
		}
		timeout = d
	}
	info, ok := ActionInfoOf(reflect.TypeFor[G](), reflect.TypeFor[F](), reflect.TypeFor[R]())
	if !ok {
		return fmt.Errorf("action type (%s, %s, %s) is not registered; generate it with conductor msggen or call conductor.RegisterAction",
			reflect.TypeFor[G](), reflect.TypeFor[F](), reflect.TypeFor[R]())
	}
	registerMessageType(reflect.TypeFor[feedbackMessage[F]](), info.FeedbackMessage)
	registerServiceTypes(reflect.TypeFor[sendGoalRequest[G]](), reflect.TypeFor[sendGoalResponse](), info.SendGoal)
	registerServiceTypes(reflect.TypeFor[getResultRequest](), reflect.TypeFor[getResultResponse[R]](), info.GetResult)

	c.name = name
	c.timeout = timeout
	c.pending = map[uuidMsg]chan F{}
	rt.recordConsumes(nr.name, name)
	base := name + "/_action/"

	clients := []struct {
		suffix   string
		req, res reflect.Type
		timeout  time.Duration
		dst      *func(any, time.Duration) (any, error)
	}{
		{"send_goal", reflect.TypeFor[sendGoalRequest[G]](), reflect.TypeFor[sendGoalResponse](), sendGoalTimeout, &c.sendGoal},
		{"cancel_goal", reflect.TypeFor[cancelGoalRequest](), reflect.TypeFor[cancelGoalResponse](), sendGoalTimeout, &c.cancelGoal},
		// get_result blocks for the whole goal, so it carries the goal timeout.
		{"get_result", reflect.TypeFor[getResultRequest](), reflect.TypeFor[getResultResponse[R]](), timeout, &c.getResult},
	}
	for _, cl := range clients {
		call, err := rt.transport.ServiceClient(ServiceSpec{
			Service: base + cl.suffix, ReqType: cl.req, ResType: cl.res,
			Node: nr.name, Timeout: cl.timeout,
		})
		if err != nil {
			return err
		}
		*cl.dst = call
	}

	// Feedback is dispatched straight from the transport rather than through
	// the node executor: goals are driven from their caller's goroutine, and
	// routing here would couple feedback delivery to executor availability.
	reliableQoS, _ := QoSProfile("reliable")
	return rt.transport.Subscribe(TopicSpec{
		Topic: base + "feedback", QoS: reliableQoS,
		Type: reflect.TypeFor[feedbackMessage[F]](), Node: nr.name,
	}, func(m any, _ Metadata) {
		msg, ok := m.(feedbackMessage[F])
		if !ok {
			return
		}
		// The send stays under the lock: Result closes the channel while
		// holding it, and sending on a closed channel would panic. Safe
		// because the send is non-blocking.
		c.mu.Lock()
		defer c.mu.Unlock()
		ch := c.pending[msg.GoalId]
		if ch == nil {
			return // feedback for another client's goal, or already finished
		}
		select {
		case ch <- msg.Feedback:
		default:
			slog.Debug("conductor: feedback dropped, consumer too slow", "action", c.name)
		}
	})
}
