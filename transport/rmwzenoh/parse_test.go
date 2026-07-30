package rmwzenoh

import (
	"testing"

	conductor "conductor.dev/conductor"
)

// The token this parses is the one captured from live rmw_zenoh traffic in
// the golden above: a `ros2 topic pub /chatter` publisher, written by C++,
// read here. Reading someone else's graph is the whole point, so the test
// that matters is the one whose input conductor did not write.
func TestParseCapturedPublisherToken(t *testing.T) {
	token := "@ros2_lv/0/" + cliZid + "/0/4/MP/%/%/" + cliNode +
		"/%chatter/" + chatterDDS + "/" + chatterHash + "/::,10:,:,:,,"

	e, err := ParseToken(token)
	if err != nil {
		t.Fatal(err)
	}
	if e.Kind != EntityPublisher {
		t.Errorf("kind = %q, want %q", e.Kind, EntityPublisher)
	}
	if e.Domain != 0 || e.ZID != cliZid || e.NID != 0 || e.EID != 4 {
		t.Errorf("identity = %d/%s/%d/%d", e.Domain, e.ZID, e.NID, e.EID)
	}
	if e.Node != cliNode {
		t.Errorf("node = %q, want %q", e.Node, cliNode)
	}
	if e.Topic != "/chatter" {
		t.Errorf("topic = %q, want /chatter", e.Topic)
	}
	if e.Type != "std_msgs/msg/String" {
		t.Errorf("type = %q, want std_msgs/msg/String", e.Type)
	}
	if e.DDSType != chatterDDS {
		t.Errorf("dds type = %q, want %q", e.DDSType, chatterDDS)
	}
	if e.Hash != chatterHash {
		t.Errorf("hash = %q, want %q", e.Hash, chatterHash)
	}
	if e.QoS.Name != "reliable" || e.QoS.Depth != 10 {
		t.Errorf("qos = %q depth %d, want reliable depth 10", e.QoS.Name, e.QoS.Depth)
	}
	if e.Enclave != "/" || e.Namespace != "/" {
		t.Errorf("enclave/namespace = %q/%q, want //", e.Enclave, e.Namespace)
	}
}

func TestParseNodeToken(t *testing.T) {
	e, err := ParseToken(NodeToken(0, cliZid, 2, 3, "safety_monitor"))
	if err != nil {
		t.Fatal(err)
	}
	if e.Kind != EntityNode || e.Node != "safety_monitor" || e.NID != 2 || e.EID != 3 {
		t.Fatalf("parsed %+v", e)
	}
	if e.Topic != "" {
		t.Errorf("node token carries a topic %q", e.Topic)
	}
}

// Every token this package can write, it can read: the QoS profile, the
// namespaced topic and the type all survive the round trip.
func TestTokenRoundTrip(t *testing.T) {
	for _, profile := range []string{"reliable", "sensor", "transient"} {
		q, _ := conductor.QoSProfile(profile)
		for _, kind := range []string{EntityPublisher, EntitySubscription, EntityService, EntityClient} {
			token := EndpointToken(3, cliZid, 1, 7, kind, "navigator",
				"/patrol_1/cmd_vel", "geometry_msgs::msg::dds_::Twist_", "RIHS01_abc", q)
			e, err := ParseToken(token)
			if err != nil {
				t.Fatalf("%s %s: %v", kind, profile, err)
			}
			if e.Kind != kind {
				t.Errorf("%s: kind = %q", kind, e.Kind)
			}
			if e.Domain != 3 || e.NID != 1 || e.EID != 7 || e.Node != "navigator" {
				t.Errorf("%s: identity %+v", kind, e)
			}
			if e.Topic != "/patrol_1/cmd_vel" {
				t.Errorf("%s: topic = %q, want the namespaced name back", kind, e.Topic)
			}
			if e.Type != "geometry_msgs/msg/Twist" {
				t.Errorf("%s: type = %q", kind, e.Type)
			}
			if e.QoS.Name != profile {
				t.Errorf("%s: qos = %q, want %q", kind, e.QoS.Name, profile)
			}
			if e.QoS.Reliability != q.Reliability || e.QoS.Durability != q.Durability || e.QoS.Depth != q.Depth {
				t.Errorf("%s %s: qos %+v, want %+v", kind, profile, e.QoS, q)
			}
		}
	}
}

// A profile conductor cannot name is reported as unnamed rather than rounded
// to the nearest one: pretending a peer offered "reliable" when it offered
// something else is how a QoS mismatch becomes silence.
func TestParseQoSKeyexprOutsideOurProfiles(t *testing.T) {
	q, err := ParseQoSKeyexpr("2:1:,7:,:,:,,")
	if err != nil {
		t.Fatal(err)
	}
	if q.Reliability != conductor.BestEffort || q.Durability != conductor.TransientLocal || q.Depth != 7 {
		t.Fatalf("parsed %+v", q)
	}
	if q.Name != "" {
		t.Errorf("named %q, but conductor has no best-effort transient-local profile", q.Name)
	}
}

func TestParseTokenRejectsRubbish(t *testing.T) {
	cases := map[string]string{
		"not a token":        "some/other/keyexpr",
		"short node token":   "@ros2_lv/0/zid/0/0/NN/%/%",
		"unknown kind":       "@ros2_lv/0/zid/0/0/XX/%/%/n/%t/pkg::msg::dds_::T_/RIHS01_x/::,10:,:,:,,",
		"endpoint too short": "@ros2_lv/0/zid/0/0/MP/%/%/node",
		"bad domain":         "@ros2_lv/x/zid/0/0/NN/%/%/node",
		"bad qos":            "@ros2_lv/0/zid/0/0/MP/%/%/n/%t/pkg::msg::dds_::T_/RIHS01_x/9::,10:,:,:,,",
	}
	for name, token := range cases {
		if _, err := ParseToken(token); err == nil {
			t.Errorf("%s: parsed %q without complaint", name, token)
		}
	}
}

func TestTypeFromDDS(t *testing.T) {
	cases := map[string]string{
		"std_msgs::msg::dds_::String_":                  "std_msgs/msg/String",
		"nav2_msgs::action::dds_::NavigateToPose_Goal_": "nav2_msgs/action/NavigateToPose_Goal",
		"std_srvs::srv::dds_::SetBool_Request_":         "std_srvs/srv/SetBool_Request",
		"something_odd":                                 "something_odd",
	}
	for dds, want := range cases {
		if got := TypeFromDDS(dds); got != want {
			t.Errorf("TypeFromDDS(%q) = %q, want %q", dds, got, want)
		}
	}
}
