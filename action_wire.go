package conductor

import (
	"reflect"
	"time"
)

// Wire types for the ROS 2 action protocol (unique_identifier_msgs and
// action_msgs, plus the per-action generic wrappers rosidl derives). They are
// hand-written rather than msggen-generated because the runtime's action
// machinery needs them and generated packages depend on this package; the
// hardcoded hashes are verified against the installed distro in
// action_wire_test.go.

type uuidMsg struct {
	Uuid [16]uint8
}

type goalInfoMsg struct {
	GoalId uuidMsg
	Stamp  time.Time
}

type goalStatusMsg struct {
	GoalInfo goalInfoMsg
	Status   int8
}

type goalStatusArrayMsg struct {
	StatusList []goalStatusMsg
}

type cancelGoalRequest struct {
	GoalInfo goalInfoMsg
}

type cancelGoalResponse struct {
	ReturnCode     int8
	GoalsCanceling []goalInfoMsg
}

// Per-action wrappers; the type parameter instantiations are registered at
// bind time with the hashes carried by the action's ActionInfo.
type sendGoalRequest[G any] struct {
	GoalId uuidMsg
	Goal   G
}

type sendGoalResponse struct {
	Accepted bool
	Stamp    time.Time
}

type getResultRequest struct {
	GoalId uuidMsg
}

type getResultResponse[R any] struct {
	Status int8
	Result R
}

type feedbackMessage[F any] struct {
	GoalId   uuidMsg
	Feedback F
}

// action_msgs/msg/GoalStatus status codes.
const (
	goalStatusExecuting int8 = 2
	goalStatusCanceling int8 = 3
	goalStatusSucceeded int8 = 4
	goalStatusCanceled  int8 = 5
	goalStatusAborted   int8 = 6
)

// action_msgs/srv/CancelGoal return codes.
const (
	cancelErrNone           int8 = 0
	cancelErrUnknownGoalID  int8 = 2
	cancelErrGoalTerminated int8 = 3
)

// RIHS01 hashes for the fixed action wire types, from the ROS 2 Lyrical type
// description files (verified in action_wire_test.go).
const (
	goalStatusArrayHash = "RIHS01_6c1684b00f177d37438febe6e709fc4e2b0d4248dca4854946f9ed8b30cda83e"
	cancelGoalHash      = "RIHS01_573d8b0a534451d7bc2ac8c5ffde8ac14b8593b7001175d0cd6516dcbeb8689a"
)

func init() {
	RegisterMessage[goalStatusArrayMsg]("action_msgs/msg/GoalStatusArray", goalStatusArrayHash)
	RegisterService[cancelGoalRequest, cancelGoalResponse]("action_msgs/srv/CancelGoal", cancelGoalHash)
}

// ActionInfo identifies an action type: its name and the wire-level
// identities of its derived send_goal/get_result services and feedback
// message. conductor msggen emits RegisterAction calls from .action files.
type ActionInfo struct {
	Name            string
	SendGoal        MessageInfo
	GetResult       MessageInfo
	FeedbackMessage MessageInfo
}

type actionKey struct {
	goal, feedback, result reflect.Type
}

var actionRegistry = map[actionKey]ActionInfo{}

// RegisterAction associates the (Goal, Feedback, Result) Go type triple with
// its action identity.
func RegisterAction[G, F, R any](info ActionInfo) {
	msgMu.Lock()
	defer msgMu.Unlock()
	actionRegistry[actionKey{reflect.TypeFor[G](), reflect.TypeFor[F](), reflect.TypeFor[R]()}] = info
}

// ActionInfoOf looks up the registered info for a goal/feedback/result type
// triple.
func ActionInfoOf(goal, feedback, result reflect.Type) (ActionInfo, bool) {
	msgMu.RLock()
	defer msgMu.RUnlock()
	info, ok := actionRegistry[actionKey{goal, feedback, result}]
	return info, ok
}

// registerMessageType and registerServiceTypes register concrete generic
// instantiations (feedbackMessage[F], sendGoalRequest[G], ...) at bind time.
func registerMessageType(t reflect.Type, info MessageInfo) {
	msgMu.Lock()
	defer msgMu.Unlock()
	msgRegistry[t] = info
}

func registerServiceTypes(req, res reflect.Type, info MessageInfo) {
	msgMu.Lock()
	defer msgMu.Unlock()
	svcRegistry[serviceKey{req, res}] = info
}
