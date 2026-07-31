package main

import (
	"reflect"
	"strings"
	"testing"
	"time"

	"conductor.dev/conductor"
	"conductor.dev/conductor/cdr"
	"conductor.dev/conductor/conductortest"
)

// These tests run the manipulation mission against a scripted move_group: a
// probe wired into the app under test, serving the same action, answering the
// way MoveIt does — including its habit of reporting failure twice, once in the
// goal status and once in the result's error code.

// planner is move_group's half of the conversation, under the test's control.
type planner struct {
	Move conductor.Action[MoveGroupGoal, MoveGroupFeedback, MoveGroupResult] `action:"move_action"`

	requests chan MotionPlanRequest // every request received
	outcomes chan int32             // scripted error codes, one per request
}

func newPlanner() *planner {
	return &planner{
		requests: make(chan MotionPlanRequest, 32),
		outcomes: make(chan int32, 32),
	}
}

func (p *planner) OnMove(g *conductor.Goal[MoveGroupGoal, MoveGroupFeedback]) (MoveGroupResult, error) {
	req := g.Value().Request
	p.requests <- req
	select {
	case code := <-p.outcomes:
		if code != MoveItErrorCodes_SUCCESS {
			return MoveGroupResult{
				ErrorCode: MoveItErrorCodes{Val: code, Message: "scripted"},
			}, errScripted
		}
	default:
	}
	return MoveGroupResult{ErrorCode: MoveItErrorCodes{Val: MoveItErrorCodes_SUCCESS}}, nil
}

var errScripted = &plannerError{}

type plannerError struct{}

func (e *plannerError) Error() string { return "scripted planning failure" }

func runCommander(t *testing.T, p *planner, params map[string]string) (*conductortest.App, *Commander) {
	t.Helper()
	semantics, err := conductor.LoadSemantics("groups.json")
	if err != nil || semantics == nil {
		t.Fatalf("loading groups.json: %v", err)
	}
	if params == nil {
		params = map[string]string{}
	}
	if _, ok := params["settle"]; !ok {
		params["settle"] = "1ms"
	}

	commander := &Commander{}
	app := conductortest.RunWith(t, conductortest.Options{
		ManualLifecycle: true,
		Semantics:       semantics,
		Params:          map[string]map[string]string{"commander": params},
	}, commander)
	app.Probe("move_group", p)

	app.Transition("commander", conductor.TransitionConfigure)
	app.Transition("commander", conductor.TransitionActivate)
	return app, commander
}

func await(t *testing.T, ch <-chan MotionPlanRequest, what string) MotionPlanRequest {
	t.Helper()
	select {
	case v := <-ch:
		return v
	case <-time.After(5 * time.Second):
		t.Fatalf("timed out waiting for %s", what)
		return MotionPlanRequest{}
	}
}

// The joint values in a planning request come from the robot's SRDF, not from
// this application: that is the whole point of declaring the group.
func TestNamedConfigurationsComeFromTheSRDF(t *testing.T) {
	p := newPlanner()
	_, commander := runCommander(t, p, nil)

	req := await(t, p.requests, "the first planning request")
	if req.GroupName != "panda_arm" {
		t.Fatalf("group = %q, want the declared panda_arm", req.GroupName)
	}
	if len(req.GoalConstraints) != 1 {
		t.Fatalf("%d goal constraints, want one", len(req.GoalConstraints))
	}

	// panda.srdf's "ready": seven joints, with panda_joint4 at -2.356.
	joints := req.GoalConstraints[0].JointConstraints
	if len(joints) != 7 {
		t.Fatalf("%d joint constraints, want the SRDF's seven", len(joints))
	}
	var found bool
	for _, j := range joints {
		if j.JointName == "panda_joint4" {
			found = true
			if j.Position != -2.356 {
				t.Errorf("panda_joint4 = %v, want the SRDF's -2.356", j.Position)
			}
		}
	}
	if !found {
		t.Errorf("no constraint for panda_joint4: %+v", joints)
	}

	// And the same values are what the application reads, without a number of
	// its own anywhere.
	ready, err := commander.Arm.State("ready")
	if err != nil {
		t.Fatal(err)
	}
	if len(ready.Positions) != 7 || ready.Positions[3] != -2.356 {
		t.Errorf("ready = %+v", ready)
	}
}

// The gripper is a planning group like the arm, with its own configurations.
func TestTheHandIsAGroupToo(t *testing.T) {
	p := newPlanner()
	_, commander := runCommander(t, p, nil)

	await(t, p.requests, "the ready plan")
	await(t, p.requests, "the reach plan")
	grasp := await(t, p.requests, "the grasp plan")
	if grasp.GroupName != "hand" {
		t.Fatalf("group = %q, want hand", grasp.GroupName)
	}
	closed, err := commander.Hand.State("close")
	if err != nil {
		t.Fatal(err)
	}
	if len(grasp.GoalConstraints[0].JointConstraints) != len(closed.JointNames) {
		t.Errorf("%d constraints for %d joints", len(grasp.GoalConstraints[0].JointConstraints), len(closed.JointNames))
	}
}

// MoveIt reports a failed plan in the result's error code, and an application
// that only checked the goal status would carry on as though the arm had moved.
func TestAFailedPlanTakesTheRecoveryBranch(t *testing.T) {
	p := newPlanner()
	p.outcomes <- MoveItErrorCodes_SUCCESS         // ready
	p.outcomes <- MoveItErrorCodes_PLANNING_FAILED // reach
	app, _ := runCommander(t, p, nil)

	await(t, p.requests, "the ready plan")
	await(t, p.requests, "the reach plan")

	// Recovery plans to "transport", which is a configuration the SRDF names.
	recovery := await(t, p.requests, "the recovery plan")
	if recovery.GoalConstraints[0].Name != "transport" {
		t.Fatalf("recovery planned to %q, want the SRDF's transport configuration",
			recovery.GoalConstraints[0].Name)
	}
	if status, step := app.Mission("commander"); status != conductor.MissionRunning {
		t.Errorf("mission %s at %q, want it still running", status, step)
	}
}

// A group the SRDF does not declare cannot be wired at all: the failure is at
// startup, naming what the robot does have.
func TestAnUndeclaredGroupFailsToWire(t *testing.T) {
	semantics, err := conductor.LoadSemantics("groups.json")
	if err != nil {
		t.Fatal(err)
	}
	var probe struct {
		Arm conductor.Group `group:"panda_manipulator"`
	}
	app, err := conductor.NewTestApp(conductor.TestOptions{
		ManualLifecycle: true, Semantics: semantics,
	}, &Commander{})
	if err != nil {
		t.Fatal(err)
	}
	defer app.Close()

	err = app.BindProbe("bad", &probe)
	if err == nil {
		t.Fatal("a group that is not in the SRDF was wired")
	}
	for _, want := range []string{"panda_manipulator", "panda_arm"} {
		if !strings.Contains(err.Error(), want) {
			t.Errorf("error does not mention %q: %v", want, err)
		}
	}
}

// moveit_msgs/action/MoveGroup is the largest nested message in common ROS use:
// arrays of structs, arrays of arrays, fixed-size arrays, signed blobs, strings,
// times and durations. Encoding and decoding it is the codec's stress test, and
// it is here rather than in cdr's own tests because this is where the types are.
func TestMoveGroupRoundTripsThroughCDR(t *testing.T) {
	want := fullGoal()
	b, err := cdr.Marshal(want)
	if err != nil {
		t.Fatalf("marshalling a MoveGroup goal: %v", err)
	}
	if len(b) < 2000 {
		t.Fatalf("encoded goal is %d bytes; the fixture is not exercising much", len(b))
	}

	var got MoveGroupGoal
	if err := cdr.Unmarshal(b, &got); err != nil {
		t.Fatalf("unmarshalling: %v", err)
	}
	// compare rather than reflect.DeepEqual: a time.Time carries a monotonic
	// reading and a location that a ROS timestamp does not.
	compare(t, "", reflect.ValueOf(want), reflect.ValueOf(got))
}

// compare walks two values and reports the fields that differ, because
// DeepEqual on a message this size says only "no".
func compare(t *testing.T, path string, a, b reflect.Value) {
	t.Helper()
	if a.Type() == reflect.TypeOf(time.Time{}) {
		if !a.Interface().(time.Time).Equal(b.Interface().(time.Time)) {
			t.Errorf("%s: %v != %v", path, a.Interface(), b.Interface())
		}
		return
	}
	switch a.Kind() {
	case reflect.Struct:
		for i := 0; i < a.NumField(); i++ {
			if !a.Type().Field(i).IsExported() {
				continue
			}
			compare(t, path+"."+a.Type().Field(i).Name, a.Field(i), b.Field(i))
		}
	case reflect.Slice, reflect.Array:
		if a.Len() != b.Len() {
			t.Errorf("%s: len %d != %d", path, a.Len(), b.Len())
			return
		}
		for i := 0; i < a.Len(); i++ {
			compare(t, path+"["+itoa(i)+"]", a.Index(i), b.Index(i))
		}
	default:
		if !reflect.DeepEqual(a.Interface(), b.Interface()) {
			t.Errorf("%s: %v != %v", path, a.Interface(), b.Interface())
		}
	}
}

func itoa(i int) string {
	if i < 10 {
		return string(rune('0' + i))
	}
	return string(rune('0'+i/10)) + string(rune('0'+i%10))
}
