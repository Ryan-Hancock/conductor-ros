package conductor

import (
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
	a, err := newApp("inproc", TransportOptions{}, "", &Counter2{})
	if err != nil {
		t.Fatal(err)
	}
	defer a.stop()
	tr := a.rt.transport

	var feedbacks atomic.Int64
	fbSpec := TopicSpec{Topic: "count/_action/feedback", Type: nil, Node: "test"}
	if err := tr.Subscribe(fbSpec, func(m any) {
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
