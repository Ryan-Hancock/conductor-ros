package rmwzenoh

import (
	"encoding/hex"
	"testing"

	conductor "conductor.dev/conductor"
)

// Golden values in this file are samples captured from live rmw_zenoh 0.10
// traffic (ROS 2 Lyrical, ros2 topic pub /chatter std_msgs/msg/String).

const (
	chatterDDS  = "std_msgs::msg::dds_::String_"
	chatterHash = "RIHS01_df668c740482bbd48fb39d76a70dfd4bd59db1288021743503259e948f6b1a18"
	cliZid      = "cb6e75cf8194a4d81370b618e0a4ed07"
	cliNode     = "_ros2cli_413596"
)

func reliableDepth(depth int) conductor.QoS {
	q, _ := conductor.QoSProfile("reliable")
	q.Depth = depth
	return q
}

func TestTopicKeyexpr(t *testing.T) {
	got := TopicKeyexpr(0, "/chatter", chatterDDS, chatterHash)
	want := "0/chatter/" + chatterDDS + "/" + chatterHash
	if got != want {
		t.Fatalf("got  %s\nwant %s", got, want)
	}
	if TopicKeyexpr(0, "chatter", chatterDDS, chatterHash) != want {
		t.Fatal("leading slash must not change the keyexpr")
	}
}

func TestNodeToken(t *testing.T) {
	got := NodeToken(0, cliZid, 0, 0, cliNode)
	want := "@ros2_lv/0/" + cliZid + "/0/0/NN/%/%/" + cliNode
	if got != want {
		t.Fatalf("got  %s\nwant %s", got, want)
	}
}

func TestPublisherToken(t *testing.T) {
	got := EndpointToken(0, cliZid, 0, 4, EntityPublisher, cliNode,
		"/chatter", chatterDDS, chatterHash, reliableDepth(10))
	want := "@ros2_lv/0/" + cliZid + "/0/4/MP/%/%/" + cliNode +
		"/%chatter/" + chatterDDS + "/" + chatterHash + "/::,10:,:,:,,"
	if got != want {
		t.Fatalf("got  %s\nwant %s", got, want)
	}
}

func TestQoSKeyexpr(t *testing.T) {
	cases := map[string]string{
		"reliable":  "::,10:,:,:,,",
		"sensor":    "2::,5:,:,:,,",
		"transient": ":1:,1:,:,:,,",
	}
	for profile, want := range cases {
		q, ok := conductor.QoSProfile(profile)
		if !ok {
			t.Fatalf("unknown profile %s", profile)
		}
		if got := QoSKeyexpr(q); got != want {
			t.Errorf("QoSKeyexpr(%s) = %q, want %q", profile, got, want)
		}
	}
}

func TestEncodeAttachment(t *testing.T) {
	gid, _ := hex.DecodeString("1236acd8c8370ad4e6ed96c021a8b85e")
	got := EncodeAttachment(0x1b, 0x18c67c893ac5a140, gid)
	want := "1b0000000000000040a1c53a897cc618101236acd8c8370ad4e6ed96c021a8b85e"
	if hex.EncodeToString(got) != want {
		t.Fatalf("got  %s\nwant %s", hex.EncodeToString(got), want)
	}
}
