package msggen

import (
	"encoding/json"
	"os"
	"path/filepath"
	"strings"
	"testing"
)

func testResolver(t *testing.T) *Resolver {
	t.Helper()
	return NewResolver([]string{filepath.Join("testdata", "share")})
}

// Expected hashes are the RIHS01 values shipped in the ROS 2 Lyrical type
// description files — the same values verified against live rmw_zenoh
// traffic. Reproducing them from .msg sources alone proves the whole
// parse -> describe -> hash pipeline.
func TestKnownHashes(t *testing.T) {
	r := testResolver(t)
	want := map[string]string{
		"std_msgs/msg/Empty":            "RIHS01_20b625256f32d5dbc0d04fee44f43c41e51c70d3502f84b4a08e7a9c26a96312",
		"std_msgs/msg/Bool":             "RIHS01_feb91e995ff9ebd09c0cb3d2aed18b11077585839fb5db80193b62d74528f6c9",
		"std_msgs/msg/String":           "RIHS01_df668c740482bbd48fb39d76a70dfd4bd59db1288021743503259e948f6b1a18",
		"std_msgs/msg/Header":           "RIHS01_f49fb3ae2cf070f793645ff749683ac6b06203e41c891e17701b1cb597ce6a01",
		"geometry_msgs/msg/Vector3":     "RIHS01_cc12fe83e4c02719f1ce8070bfd14aecd40f75a96696a67a2a1f37f7dbb0765d",
		"geometry_msgs/msg/Twist":       "RIHS01_9c45bf16fe0983d80e3cfe750d6835843d265a9a6c46bd2e609fcddde6fb8d2a",
		"geometry_msgs/msg/PoseStamped": "RIHS01_10f3786d7d40fd2b54367835614bff85d4ad3b5dab62bf8bca0cc232d73b4cd8",
	}
	for name, wantHash := range want {
		td, err := r.Describe(name)
		if err != nil {
			t.Fatalf("%s: %v", name, err)
		}
		if got := td.Hash(); got != wantHash {
			t.Errorf("%s:\n got %s\nwant %s", name, got, wantHash)
		}
	}
}

func TestParseFeatures(t *testing.T) {
	src := `
# A test message.
uint8 MODE_IDLE=0
uint8 MODE_ACTIVE=1
string NAME = "quoted # not a comment"

std_msgs/Header header
Header shorthand          # bare Header means std_msgs/Header
geometry_msgs/Point[] path
float64[36] covariance
int32[<=5] window
string<=10 label
uint8 mode
float64 speed 1.5         # field with default
SamePkgType nested
`
	m, err := ParseMessage("test_pkg", "Fancy", []byte(src))
	if err != nil {
		t.Fatal(err)
	}
	if len(m.Constants) != 3 {
		t.Fatalf("constants: %+v", m.Constants)
	}
	if m.Constants[2].Value != `"quoted # not a comment"` {
		t.Errorf("string constant value = %q", m.Constants[2].Value)
	}
	fields := map[string]Field{}
	for _, f := range m.Fields {
		fields[f.Name] = f
	}
	checks := []struct {
		name string
		want FieldType
	}{
		{"header", FieldType{Nested: "std_msgs/msg/Header"}},
		{"shorthand", FieldType{Nested: "std_msgs/msg/Header"}},
		{"path", FieldType{Nested: "geometry_msgs/msg/Point", Kind: UnboundedSeq}},
		{"covariance", FieldType{Builtin: "float64", Kind: Array, Capacity: 36}},
		{"window", FieldType{Builtin: "int32", Kind: BoundedSeq, Capacity: 5}},
		{"label", FieldType{Builtin: "string", StrCapacity: 10}},
		{"mode", FieldType{Builtin: "uint8"}},
		{"speed", FieldType{Builtin: "float64"}},
		{"nested", FieldType{Nested: "test_pkg/msg/SamePkgType"}},
	}
	for _, c := range checks {
		f, ok := fields[c.name]
		if !ok {
			t.Errorf("missing field %s", c.name)
			continue
		}
		if f.Type != c.want {
			t.Errorf("%s: got %+v, want %+v", c.name, f.Type, c.want)
		}
	}
	if fields["speed"].Default != "1.5" {
		t.Errorf("speed default = %q", fields["speed"].Default)
	}

	// type_id spot checks against the FieldType constant table.
	ids := map[string]int{
		"covariance": 11 + offsetArray,    // DOUBLE_ARRAY = 59
		"window":     6 + offsetBounded,   // INT32_BOUNDED_SEQUENCE = 102
		"path":       1 + offsetUnbounded, // NESTED_TYPE_UNBOUNDED_SEQUENCE = 145
		"label":      idBoundedString,     // BOUNDED_STRING = 21
	}
	for name, want := range ids {
		if got := typeID(fields[name].Type); got != want {
			t.Errorf("typeID(%s) = %d, want %d", name, got, want)
		}
	}
}

// Service hashes cover the synthesized service-level description (request +
// response + event members); reproducing rosidl's exact values proves the
// whole .srv pipeline including the _Event synthesis.
func TestKnownServiceHashes(t *testing.T) {
	r := testResolver(t)
	want := map[string]string{
		"std_srvs/srv/SetBool":          "RIHS01_abe9e4bb6b41b40e6789712c00ec8871923e089af3f667a79992a428cff2da0a",
		"std_srvs/srv/SetBool_Request":  "RIHS01_c62fbb99d94e1b25e8ef9e109f9581956bb1b3361a45a4e5810c36a90d29932e",
		"std_srvs/srv/SetBool_Response": "RIHS01_d0814e7f7b4880ab77e9c57426c7aa1562ab69f11eef8e2e968812f9cbd0b059",
		"std_srvs/srv/SetBool_Event":    "RIHS01_3c4c20015afb4303eafd347b1d6a786f171a89c814726961a9593ef10df878cf",
		"std_srvs/srv/Trigger":          "RIHS01_eeff2cd6fa5ad9d27cdf4dec64818317839b62f212a91e6b5304b634b2062c5f",
		// Trigger has an empty request: placeholder-field handling on the wire.
		"std_srvs/srv/Trigger_Request": "RIHS01_d010825374ce8918e72bfd826c82603e60f45419e932ea976f807b74a863a199",
	}
	for name, wantHash := range want {
		td, err := r.Describe(name)
		if err != nil {
			t.Fatalf("%s: %v", name, err)
		}
		if got := td.Hash(); got != wantHash {
			t.Errorf("%s:\n got %s\nwant %s", name, got, wantHash)
		}
	}
}

func TestGenerateService(t *testing.T) {
	r := testResolver(t)
	src, err := Generate(r, []string{"std_srvs/srv/SetBool"}, "srvs")
	if err != nil {
		t.Fatal(err)
	}
	out := string(src)
	for _, want := range []string{
		"type SetBoolRequest struct {",
		"type SetBoolResponse struct {",
		"//ros:type std_srvs/srv/SetBool_Request",
		"conductor.RegisterService[SetBoolRequest, SetBoolResponse](\"std_srvs/srv/SetBool\",\n\t\t\"RIHS01_abe9e4bb6b41b40e6789712c00ec8871923e089af3f667a79992a428cff2da0a\")",
	} {
		if !strings.Contains(out, want) {
			t.Errorf("generated code missing %q\n---\n%s", want, out)
		}
	}
	if strings.Contains(out, "Event") {
		t.Error("event message must not be generated as a struct")
	}
}

// Action hashes cover the six-member action description plus all derived
// services and messages; matching rosidl's values proves the .action
// synthesis (SendGoal/GetResult/FeedbackMessage) end to end.
func TestKnownActionHashes(t *testing.T) {
	r := testResolver(t)
	want := map[string]string{
		"example_interfaces/action/Fibonacci":                 "RIHS01_9508051da1ea4658de144b09bd0690ff3de52104683d847aed764d2915906f51",
		"example_interfaces/action/Fibonacci_Goal":            "RIHS01_226cb437e4355dcd3e914f930382a3b0cc1da81545bd319ed554e95a03255f51",
		"example_interfaces/action/Fibonacci_SendGoal":        "RIHS01_d1a57fb2a4afe8c21e34fb10db206f16ce6729b28531141472df92277c55b557",
		"example_interfaces/action/Fibonacci_GetResult":       "RIHS01_1b0de0d5d29dc955d92f546706568428632771db13ec84c15ec1c1a59f424a57",
		"example_interfaces/action/Fibonacci_FeedbackMessage": "RIHS01_c1de71afd52e49a89c53d8262366884185bc0a02f78ce051c4e46b0a7fe59bb2",
	}
	for name, wantHash := range want {
		td, err := r.Describe(name)
		if err != nil {
			t.Fatalf("%s: %v", name, err)
		}
		if got := td.Hash(); got != wantHash {
			t.Errorf("%s:\n got %s\nwant %s", name, got, wantHash)
		}
	}
}

func TestEmptyMessageGetsPlaceholder(t *testing.T) {
	m, err := ParseMessage("p", "Nothing", []byte("# only comments\nuint8 ONLY_CONST=1\n"))
	if err != nil {
		t.Fatal(err)
	}
	if len(m.Fields) != 1 || m.Fields[0].Name != "structure_needs_at_least_one_member" {
		t.Fatalf("fields = %+v", m.Fields)
	}
}

func TestGenerate(t *testing.T) {
	r := testResolver(t)
	src, err := Generate(r, []string{"geometry_msgs/msg/PoseStamped"}, "rosmsgs")
	if err != nil {
		t.Fatal(err)
	}
	out := string(src)
	for _, want := range []string{
		"package rosmsgs",
		"//ros:type geometry_msgs/msg/PoseStamped",
		"type PoseStamped struct {",
		"time.Time", // builtin_interfaces/Time mapped to Go time
		"conductor.RegisterMessage[PoseStamped](\"geometry_msgs/msg/PoseStamped\",\n\t\t\"RIHS01_10f3786d7d40fd2b54367835614bff85d4ad3b5dab62bf8bca0cc232d73b4cd8\")",
	} {
		if !strings.Contains(out, want) {
			t.Errorf("generated code missing %q\n---\n%s", want, out)
		}
	}
	if strings.Contains(out, "builtin_interfaces") {
		t.Error("builtin_interfaces types must not be generated as structs")
	}
}

// TestDistroCorpus validates the hasher against every message type of an
// installed ROS distro (skipped when none is present). This is the strong
// guarantee that custom .msg hashing matches rosidl exactly.
func TestDistroCorpus(t *testing.T) {
	share := "/opt/ros/lyrical/share"
	if _, err := os.Stat(share); err != nil {
		t.Skip("no ROS distro installed")
	}
	r := NewResolver([]string{share})
	msgFiles, err := filepath.Glob(filepath.Join(share, "*", "msg", "*.msg"))
	if err != nil {
		t.Fatal(err)
	}
	srvFiles, err := filepath.Glob(filepath.Join(share, "*", "srv", "*.srv"))
	if err != nil {
		t.Fatal(err)
	}
	actFiles, err := filepath.Glob(filepath.Join(share, "*", "action", "*.action"))
	if err != nil {
		t.Fatal(err)
	}
	checked, skipped := 0, 0
	for _, actFile := range actFiles {
		jsonFile := strings.TrimSuffix(actFile, ".action") + ".json"
		raw, err := os.ReadFile(jsonFile)
		if err != nil {
			continue
		}
		src, err := os.ReadFile(actFile)
		if err != nil {
			t.Fatal(err)
		}
		if strings.Contains(string(src), "wstring") {
			skipped++
			continue
		}
		var ref struct {
			TypeHashes []struct {
				TypeName   string `json:"type_name"`
				HashString string `json:"hash_string"`
			} `json:"type_hashes"`
		}
		if err := json.Unmarshal(raw, &ref); err != nil {
			t.Fatalf("%s: %v", jsonFile, err)
		}
		pkg := filepath.Base(filepath.Dir(filepath.Dir(actFile)))
		prefix := pkg + "/action/"
		for _, h := range ref.TypeHashes {
			if !strings.HasPrefix(h.TypeName, prefix) {
				continue
			}
			td, err := r.Describe(h.TypeName)
			if err != nil {
				t.Errorf("%s: %v", h.TypeName, err)
				continue
			}
			if got := td.Hash(); got != h.HashString {
				t.Errorf("%s:\n got %s\nwant %s", h.TypeName, got, h.HashString)
			}
			checked++
		}
	}
	for _, srvFile := range srvFiles {
		jsonFile := strings.TrimSuffix(srvFile, ".srv") + ".json"
		raw, err := os.ReadFile(jsonFile)
		if err != nil {
			continue
		}
		src, err := os.ReadFile(srvFile)
		if err != nil {
			t.Fatal(err)
		}
		if strings.Contains(string(src), "wstring") {
			skipped++
			continue
		}
		var ref struct {
			TypeHashes []struct {
				TypeName   string `json:"type_name"`
				HashString string `json:"hash_string"`
			} `json:"type_hashes"`
		}
		if err := json.Unmarshal(raw, &ref); err != nil {
			t.Fatalf("%s: %v", jsonFile, err)
		}
		pkg := filepath.Base(filepath.Dir(filepath.Dir(srvFile)))
		prefix := pkg + "/srv/"
		for _, h := range ref.TypeHashes {
			if !strings.HasPrefix(h.TypeName, prefix) {
				continue
			}
			td, err := r.Describe(h.TypeName)
			if err != nil {
				t.Errorf("%s: %v", h.TypeName, err)
				continue
			}
			if got := td.Hash(); got != h.HashString {
				t.Errorf("%s:\n got %s\nwant %s", h.TypeName, got, h.HashString)
			}
			checked++
		}
	}
	for _, msgFile := range msgFiles {
		jsonFile := strings.TrimSuffix(msgFile, ".msg") + ".json"
		raw, err := os.ReadFile(jsonFile)
		if err != nil {
			continue // no reference hash available
		}
		src, err := os.ReadFile(msgFile)
		if err != nil {
			t.Fatal(err)
		}
		if strings.Contains(string(src), "wstring") {
			skipped++ // documented v0 limitation
			continue
		}
		var ref struct {
			TypeHashes []struct {
				TypeName   string `json:"type_name"`
				HashString string `json:"hash_string"`
			} `json:"type_hashes"`
		}
		if err := json.Unmarshal(raw, &ref); err != nil {
			t.Fatalf("%s: %v", jsonFile, err)
		}
		pkg := filepath.Base(filepath.Dir(filepath.Dir(msgFile)))
		name := strings.TrimSuffix(filepath.Base(msgFile), ".msg")
		full := pkg + "/msg/" + name

		td, err := r.Describe(full)
		if err != nil {
			// Referenced type may itself use unsupported features.
			if strings.Contains(err.Error(), "wstring") {
				skipped++
				continue
			}
			t.Errorf("%s: %v", full, err)
			continue
		}
		for _, h := range ref.TypeHashes {
			if h.TypeName == full {
				if got := td.Hash(); got != h.HashString {
					t.Errorf("%s:\n got %s\nwant %s", full, got, h.HashString)
				}
				checked++
			}
		}
	}
	t.Logf("corpus: %d hashes verified, %d skipped (wstring)", checked, skipped)
	if checked < 100 {
		t.Errorf("corpus suspiciously small: only %d types checked", checked)
	}
}
