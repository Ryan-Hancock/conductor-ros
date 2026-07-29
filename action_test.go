package conductor

import (
	"errors"
	"strings"
	"sync/atomic"
	"testing"
	"time"
)

type countGoal struct{ Target int32 }
type countFeedback struct{ At int32 }
type countResult struct{ Reached int32 }

func init() {
	RegisterAction[countGoal, countFeedback, countResult](ActionInfo{
		Name:            "test_pkg/action/Count",
		SendGoal:        MessageInfo{Name: "test_pkg/action/Count_SendGoal", Hash: "RIHS01_test"},
		GetResult:       MessageInfo{Name: "test_pkg/action/Count_GetResult", Hash: "RIHS01_test"},
		FeedbackMessage: MessageInfo{Name: "test_pkg/action/Count_FeedbackMessage", Hash: "RIHS01_test"},
	})
}

type Counter2 struct {
	Count Action[countGoal, countFeedback, countResult] `action:"count"`
}

func (c *Counter2) OnCount(g *Goal[countGoal, countFeedback]) (countResult, error) {
	var i int32
	for i = 0; i < g.Value().Target; i++ {
		select {
		case <-g.Context().Done():
			return countResult{Reached: i}, g.Context().Err()
		case <-time.After(5 * time.Millisecond):
		}
		g.Feedback(countFeedback{At: i})
	}
	return countResult{Reached: i}, nil
}

// driveAction exercises the full wire protocol through the transport
// interface, exactly as a remote action client would.
func TestActionLifecycle(t *testing.T) {
	a := newTestApp(t, &Counter2{})
	tr := a.rt.transport

	var feedbacks atomic.Int64
	fbSpec := TopicSpec{Topic: "count/_action/feedback", Type: nil, Node: "test"}
	if err := tr.Subscribe(fbSpec, func(m any, _ Metadata) {
		if _, ok := m.(feedbackMessage[countFeedback]); ok {
			feedbacks.Add(1)
		}
	}); err != nil {
		t.Fatal(err)
	}

	sendGoal, err := tr.ServiceClient(ServiceSpec{Service: "count/_action/send_goal", Node: "test"})
	if err != nil {
		t.Fatal(err)
	}
	getResult, err := tr.ServiceClient(ServiceSpec{Service: "count/_action/get_result", Node: "test"})
	if err != nil {
		t.Fatal(err)
	}
	cancelGoal, err := tr.ServiceClient(ServiceSpec{Service: "count/_action/cancel_goal", Node: "test"})
	if err != nil {
		t.Fatal(err)
	}

	id := uuidMsg{Uuid: [16]uint8{1, 2, 3}}
	resp, err := sendGoal(sendGoalRequest[countGoal]{GoalId: id, Goal: countGoal{Target: 5}}, time.Second)
	if err != nil {
		t.Fatal(err)
	}
	if !resp.(sendGoalResponse).Accepted {
		t.Fatal("goal not accepted")
	}

	res, err := getResult(getResultRequest{GoalId: id}, 5*time.Second)
	if err != nil {
		t.Fatal(err)
	}
	rr := res.(getResultResponse[countResult])
	if rr.Status != goalStatusSucceeded || rr.Result.Reached != 5 {
		t.Fatalf("result = %+v", rr)
	}
	if feedbacks.Load() == 0 {
		t.Error("no feedback received")
	}

	// Cancellation path: long goal, cancel mid-flight.
	id2 := uuidMsg{Uuid: [16]uint8{9}}
	if _, err := sendGoal(sendGoalRequest[countGoal]{GoalId: id2, Goal: countGoal{Target: 1000}}, time.Second); err != nil {
		t.Fatal(err)
	}
	time.Sleep(20 * time.Millisecond)
	cres, err := cancelGoal(cancelGoalRequest{GoalInfo: goalInfoMsg{GoalId: id2}}, time.Second)
	if err != nil {
		t.Fatal(err)
	}
	if n := len(cres.(cancelGoalResponse).GoalsCanceling); n != 1 {
		t.Fatalf("goals canceling = %d, want 1", n)
	}
	res2, err := getResult(getResultRequest{GoalId: id2}, 5*time.Second)
	if err != nil {
		t.Fatal(err)
	}
	if got := res2.(getResultResponse[countResult]).Status; got != goalStatusCanceled {
		t.Fatalf("status after cancel = %d, want %d", got, goalStatusCanceled)
	}

	// Unknown goal cancellation.
	cres2, err := cancelGoal(cancelGoalRequest{GoalInfo: goalInfoMsg{GoalId: uuidMsg{Uuid: [16]uint8{7, 7}}}}, time.Second)
	if err != nil {
		t.Fatal(err)
	}
	if code := cres2.(cancelGoalResponse).ReturnCode; code != cancelErrUnknownGoalID {
		t.Fatalf("cancel unknown goal return code = %d", code)
	}
}

type Driver2 struct {
	Count ActionClient[countGoal, countFeedback, countResult] `action:"count" timeout:"10s"`
}

// TestActionClientRoundTrip drives a conductor action server with a
// conductor action client, exercising the same protocol a remote ROS client
// would speak.
func TestActionClientRoundTrip(t *testing.T) {
	drv := &Driver2{}
	newTestApp(t, &Counter2{}, drv)

	h, err := drv.Count.SendGoal(countGoal{Target: 4})
	if err != nil {
		t.Fatal(err)
	}
	if len(h.ID()) != 32 {
		t.Errorf("goal id = %q, want 32 hex chars", h.ID())
	}

	var feedback []int32
	fbDone := make(chan struct{})
	go func() {
		defer close(fbDone)
		for f := range h.Feedback() {
			feedback = append(feedback, f.At)
		}
	}()

	res, status, err := h.Result()
	if err != nil {
		t.Fatal(err)
	}
	if !status.Succeeded() {
		t.Fatalf("status = %s, want SUCCEEDED", status)
	}
	if res.Reached != 4 {
		t.Fatalf("reached = %d, want 4", res.Reached)
	}
	<-fbDone
	if len(feedback) == 0 {
		t.Error("no feedback received")
	}

	// Result is idempotent.
	if _, s2, err := h.Result(); err != nil || s2 != status {
		t.Errorf("second Result() = %s, %v; want %s, nil", s2, err, status)
	}
}

// Refuser2 aborts every goal, reporting how far it got — the shape every
// real interface has, where the reason for failure lives in the result.
type Refuser2 struct {
	Count Action[countGoal, countFeedback, countResult] `action:"refuse"`
}

func (r *Refuser2) OnCount(g *Goal[countGoal, countFeedback]) (countResult, error) {
	return countResult{Reached: 3}, errors.New("cannot go further")
}

type RefusedDriver struct {
	Count ActionClient[countGoal, countFeedback, countResult] `action:"refuse" timeout:"10s"`
}

// An aborted goal delivers the server's result as well as its status.
// nav2_msgs/action/NavigateToPose puts error_code and error_msg there and
// nowhere else, so a client that only learned "aborted" would be told a
// failure happened without being told what it was.
func TestActionAbortCarriesItsResult(t *testing.T) {
	drv := &RefusedDriver{}
	newTestApp(t, &Refuser2{}, drv)

	h, err := drv.Count.SendGoal(countGoal{Target: 10})
	if err != nil {
		t.Fatal(err)
	}
	res, status, err := h.Result()
	if err != nil {
		t.Fatal(err)
	}
	if status != StatusAborted {
		t.Fatalf("status = %s, want ABORTED", status)
	}
	if res.Reached != 3 {
		t.Fatalf("aborted result = %d, want the 3 the handler reported", res.Reached)
	}
}

func TestActionClientCancel(t *testing.T) {
	drv := &Driver2{}
	newTestApp(t, &Counter2{}, drv)

	h, err := drv.Count.SendGoal(countGoal{Target: 10000})
	if err != nil {
		t.Fatal(err)
	}
	time.Sleep(20 * time.Millisecond)
	if err := h.Cancel(); err != nil {
		t.Fatal(err)
	}
	res, status, err := h.Result()
	if err != nil {
		t.Fatal(err)
	}
	if status != StatusCanceled {
		t.Fatalf("status = %s, want CANCELED", status)
	}
	if res.Reached == 0 || res.Reached >= 10000 {
		t.Errorf("partial result = %d, want a partial count", res.Reached)
	}
}

type NoActionInfo struct {
	X Action[ping, ping, ping] `action:"x"`
}

func (n *NoActionInfo) OnX(g *Goal[ping, ping]) (ping, error) { return ping{}, nil }

func TestActionUnregisteredTypeFailsWiring(t *testing.T) {
	_, err := newApp("inproc", TransportOptions{}, "", &NoActionInfo{})
	if err == nil || !strings.Contains(err.Error(), "not registered") {
		t.Fatalf("expected unregistered-action error, got %v", err)
	}
}
