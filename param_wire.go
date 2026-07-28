package conductor

import "time"

// Wire types for the ROS 2 parameter protocol (rcl_interfaces), hand-written
// here for the same reason as the lifecycle and action wire types: the
// runtime needs them and generated packages import this one. Hashes are
// checked against the installed distro in param_test.go.

// ParameterType values from rcl_interfaces/msg/ParameterType.
const (
	paramTypeNotSet       uint8 = 0
	paramTypeBool         uint8 = 1
	paramTypeInteger      uint8 = 2
	paramTypeDouble       uint8 = 3
	paramTypeString       uint8 = 4
	paramTypeByteArray    uint8 = 5
	paramTypeBoolArray    uint8 = 6
	paramTypeIntegerArray uint8 = 7
	paramTypeDoubleArray  uint8 = 8
	paramTypeStringArray  uint8 = 9
)

type parameterValueMsg struct {
	Type              uint8
	BoolValue         bool
	IntegerValue      int64
	DoubleValue       float64
	StringValue       string
	ByteArrayValue    []uint8
	BoolArrayValue    []bool
	IntegerArrayValue []int64
	DoubleArrayValue  []float64
	StringArrayValue  []string
}

type parameterMsg struct {
	Name  string
	Value parameterValueMsg
}

type floatingPointRangeMsg struct {
	FromValue float64
	ToValue   float64
	Step      float64
}

type integerRangeMsg struct {
	FromValue int64
	ToValue   int64
	Step      uint64
}

type parameterDescriptorMsg struct {
	Name                  string
	Type                  uint8
	Description           string
	AdditionalConstraints string
	ReadOnly              bool
	DynamicTyping         bool
	FloatingPointRange    []floatingPointRangeMsg
	IntegerRange          []integerRangeMsg
}

type setParametersResultMsg struct {
	Successful bool
	Reason     string
}

type listParametersResultMsg struct {
	Names    []string
	Prefixes []string
}

type getParametersRequest struct {
	Names []string
}

type getParametersResponse struct {
	Values []parameterValueMsg
}

type setParametersRequest struct {
	Parameters []parameterMsg
}

type setParametersResponse struct {
	Results []setParametersResultMsg
}

// SetParametersAtomically shares the request shape with SetParameters but
// answers with a single result, so it needs its own response type.
type setParametersAtomicallyResponse struct {
	Result setParametersResultMsg
}

type listParametersRequest struct {
	Prefixes []string
	Depth    uint64
}

type listParametersResponse struct {
	Result listParametersResultMsg
}

type describeParametersRequest struct {
	Names []string
}

type describeParametersResponse struct {
	Descriptors []parameterDescriptorMsg
}

type getParameterTypesRequest struct {
	Names []string
}

type getParameterTypesResponse struct {
	Types []uint8
}

type parameterEventMsg struct {
	Stamp             time.Time
	Node              string
	NewParameters     []parameterMsg
	ChangedParameters []parameterMsg
	DeletedParameters []parameterMsg
}

// RIHS01 hashes from the ROS 2 Lyrical type descriptions.
const (
	parameterEventHash          = "RIHS01_043e627780fcad87a22d225bc2a037361dba713fca6a6b9f4b869a5aa0393204"
	getParametersHash           = "RIHS01_bf9803d5c74cf989a5de3e0c2e99444599a627c7ff75f97b8c05b01003675cbc"
	setParametersHash           = "RIHS01_56eed9a67e169f9cb6c1f987bc88f868c14a8fc9f743a263bc734c154015d7e0"
	setParametersAtomicallyHash = "RIHS01_0e192ef259c07fc3c07a13191d27002222e65e00ccec653ca05e856f79285fcd"
	listParametersHash          = "RIHS01_3e6062bfbb27bfb8730d4cef2558221f51a11646d78e7bb30a1e83afac3aad9d"
	describeParametersHash      = "RIHS01_845b484d71eb0673dae682f2e3ba3c4851a65a3dcfb97bddd82c5b57e91e4cff"
	getParameterTypesHash       = "RIHS01_da199c878688b3e530bdfe3ca8f74cb9fa0c303101e980a9e8f260e25e1c80ca"
)

func init() {
	RegisterMessage[parameterEventMsg]("rcl_interfaces/msg/ParameterEvent", parameterEventHash)
	RegisterService[getParametersRequest, getParametersResponse]("rcl_interfaces/srv/GetParameters", getParametersHash)
	RegisterService[setParametersRequest, setParametersResponse]("rcl_interfaces/srv/SetParameters", setParametersHash)
	RegisterService[setParametersRequest, setParametersAtomicallyResponse]("rcl_interfaces/srv/SetParametersAtomically", setParametersAtomicallyHash)
	RegisterService[listParametersRequest, listParametersResponse]("rcl_interfaces/srv/ListParameters", listParametersHash)
	RegisterService[describeParametersRequest, describeParametersResponse]("rcl_interfaces/srv/DescribeParameters", describeParametersHash)
	RegisterService[getParameterTypesRequest, getParameterTypesResponse]("rcl_interfaces/srv/GetParameterTypes", getParameterTypesHash)
}
