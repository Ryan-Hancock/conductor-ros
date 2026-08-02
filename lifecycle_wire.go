package conductor

import "time"

// Wire types for the ROS 2 managed-node protocol (lifecycle_msgs). Like the
// action wire types they are hand-written here rather than generated,
// because the runtime needs them and generated packages import this one;
// their hashes are checked against the installed distro in
// lifecycle_test.go.

type stateMsg struct {
	Id    uint8
	Label string
}

type transitionMsg struct {
	Id    uint8
	Label string
}

type transitionDescriptionMsg struct {
	Transition transitionMsg
	StartState stateMsg
	GoalState  stateMsg
}

type transitionEventMsg struct {
	Stamp      time.Time
	Transition transitionMsg
	StartState stateMsg
	GoalState  stateMsg
}

type changeStateRequest struct {
	Transition transitionMsg
}

type changeStateResponse struct {
	Success bool
}

type getStateRequest struct{ StructureNeedsAtLeastOneMember uint8 }

type getStateResponse struct {
	CurrentState stateMsg
}

type getAvailableStatesRequest struct{ StructureNeedsAtLeastOneMember uint8 }

type getAvailableStatesResponse struct {
	AvailableStates []stateMsg
}

type getAvailableTransitionsRequest struct{ StructureNeedsAtLeastOneMember uint8 }

type getAvailableTransitionsResponse struct {
	AvailableTransitions []transitionDescriptionMsg
}

// RIHS01 hashes from the ROS 2 Lyrical type descriptions.
const (
	transitionEventHash         = "RIHS01_78b19e2be41a797fc009c96bda68944182edabec7852cccd928a40bc06f6f9d7"
	changeStateHash             = "RIHS01_356fe34f0475a43acf54542013af4167b0e729f77ea22ffb045c6ad8e20668e5"
	getStateHash                = "RIHS01_800a0a5aae599782b02932de0caf563f6dc4e7e94b794eadde075ba2cbef9795"
	getAvailableStatesHash      = "RIHS01_00a07d79d2207d71e81a8cbc1880e5d924cc16d4688ea8e8e06e443dc8f8aa1d"
	getAvailableTransitionsHash = "RIHS01_59b7ecefce0982a8a844b9f2c4f14764c1c4543cc55e72924e2aa4adad83e9bc"
)

func init() {
	RegisterMessage[transitionEventMsg]("lifecycle_msgs/msg/TransitionEvent", transitionEventHash)
	RegisterService[changeStateRequest, changeStateResponse]("lifecycle_msgs/srv/ChangeState", changeStateHash)
	RegisterService[getStateRequest, getStateResponse]("lifecycle_msgs/srv/GetState", getStateHash)
	RegisterService[getAvailableStatesRequest, getAvailableStatesResponse]("lifecycle_msgs/srv/GetAvailableStates", getAvailableStatesHash)
	RegisterService[getAvailableTransitionsRequest, getAvailableTransitionsResponse]("lifecycle_msgs/srv/GetAvailableTransitions", getAvailableTransitionsHash)
}

// State is a managed node's primary lifecycle state (lifecycle_msgs/msg/State).
type State uint8

const (
	StateUnknown      State = 0
	StateUnconfigured State = 1
	StateInactive     State = 2
	StateActive       State = 3
	StateFinalized    State = 4

	stateConfiguring     State = 10
	stateCleaningUp      State = 11
	stateShuttingDown    State = 12
	stateActivating      State = 13
	stateDeactivating    State = 14
	stateErrorProcessing State = 15
)

func (s State) String() string {
	switch s {
	case StateUnconfigured:
		return "unconfigured"
	case StateInactive:
		return "inactive"
	case StateActive:
		return "active"
	case StateFinalized:
		return "finalized"
	case stateConfiguring:
		return "configuring"
	case stateCleaningUp:
		return "cleaningup"
	case stateShuttingDown:
		return "shuttingdown"
	case stateActivating:
		return "activating"
	case stateDeactivating:
		return "deactivating"
	case stateErrorProcessing:
		return "errorprocessing"
	default:
		return "unknown"
	}
}

func (s State) msg() stateMsg { return stateMsg{Id: uint8(s), Label: s.String()} }

// Transition identifies a lifecycle transition (lifecycle_msgs/msg/Transition).
type Transition uint8

const (
	TransitionCreate               Transition = 0
	TransitionConfigure            Transition = 1
	TransitionCleanup              Transition = 2
	TransitionActivate             Transition = 3
	TransitionDeactivate           Transition = 4
	TransitionUnconfiguredShutdown Transition = 5
	TransitionInactiveShutdown     Transition = 6
	TransitionActiveShutdown       Transition = 7
	TransitionDestroy              Transition = 8
)

func (t Transition) String() string {
	switch t {
	case TransitionCreate:
		return "create"
	case TransitionConfigure:
		return "configure"
	case TransitionCleanup:
		return "cleanup"
	case TransitionActivate:
		return "activate"
	case TransitionDeactivate:
		return "deactivate"
	case TransitionUnconfiguredShutdown, TransitionInactiveShutdown, TransitionActiveShutdown:
		return "shutdown"
	case TransitionDestroy:
		return "destroy"
	default:
		return "unknown"
	}
}

func (t Transition) msg() transitionMsg { return transitionMsg{Id: uint8(t), Label: t.String()} }

// transitionTable maps (start state, transition) to the goal primary state,
// per the ROS 2 managed-node design. Transitions absent from the table are
// rejected.
var transitionTable = map[struct {
	from State
	t    Transition
}]State{
	{StateUnconfigured, TransitionConfigure}:            StateInactive,
	{StateUnconfigured, TransitionUnconfiguredShutdown}: StateFinalized,
	{StateInactive, TransitionActivate}:                 StateActive,
	{StateInactive, TransitionCleanup}:                  StateUnconfigured,
	{StateInactive, TransitionInactiveShutdown}:         StateFinalized,
	{StateActive, TransitionDeactivate}:                 StateInactive,
	{StateActive, TransitionActiveShutdown}:             StateFinalized,
}

// intermediateState is the transition state entered while a transition runs.
var intermediateState = map[Transition]State{
	TransitionConfigure:            stateConfiguring,
	TransitionCleanup:              stateCleaningUp,
	TransitionActivate:             stateActivating,
	TransitionDeactivate:           stateDeactivating,
	TransitionUnconfiguredShutdown: stateShuttingDown,
	TransitionInactiveShutdown:     stateShuttingDown,
	TransitionActiveShutdown:       stateShuttingDown,
}

// shutdownFor returns the shutdown transition valid from a primary state.
// resultOf is the state a node lands in when a transition succeeds. Only the
// four transitions an application drives are listed: the error and shutdown
// paths end wherever the node's own handlers take it, and guessing there
// would be worse than admitting we do not know.
func resultOf(t Transition) (State, bool) {
	switch t {
	case TransitionConfigure:
		return StateInactive, true
	case TransitionActivate:
		return StateActive, true
	case TransitionDeactivate:
		return StateInactive, true
	case TransitionCleanup:
		return StateUnconfigured, true
	}
	return StateUnknown, false
}

func shutdownFor(s State) (Transition, bool) {
	switch s {
	case StateUnconfigured:
		return TransitionUnconfiguredShutdown, true
	case StateInactive:
		return TransitionInactiveShutdown, true
	case StateActive:
		return TransitionActiveShutdown, true
	}
	return 0, false
}
