package cdr

import (
	"encoding/hex"
	"reflect"
	"testing"
	"time"

	"conductor.dev/conductor/msgs"
)

// Golden vectors generated with rclpy.serialization.serialize_message on
// ROS 2 Lyrical (Fast CDR), so a pass here means byte-compatibility with
// real ROS traffic.
func TestGoldenVectors(t *testing.T) {
	cases := []struct {
		name string
		msg  any
		hex  string
	}{
		{
			"bool_true",
			msgs.Bool{Data: true},
			"0001000001000000",
		},
		{
			"string_hello",
			msgs.String{Data: "hello conductor"},
			"000100001000000068656c6c6f20636f6e647563746f7200",
		},
		{
			"twist",
			msgs.Twist{Linear: msgs.Vector3{X: 1.5, Y: -2.25, Z: 0.125}, Angular: msgs.Vector3{Z: 3.0}},
			"00010000000000000000f83f00000000000002c0000000000000c03f000000000000000000000000000000000000000000000840",
		},
		{
			"posestamped",
			msgs.PoseStamped{
				Header: msgs.Header{Stamp: time.Unix(7, 13), FrameID: "map"},
				Pose: msgs.Pose{
					Position:    msgs.Point{X: 1, Y: 2, Z: 3},
					Orientation: msgs.Quaternion{W: 1},
				},
			},
			"00010000070000000d000000040000006d617000000000000000f03f00000000000000400000000000000840000000000000000000000000000000000000000000000000000000000000f03f",
		},
	}
	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			got, err := Marshal(c.msg)
			if err != nil {
				t.Fatal(err)
			}
			if hex.EncodeToString(got) != c.hex {
				t.Fatalf("Marshal mismatch\n got %s\nwant %s", hex.EncodeToString(got), c.hex)
			}

			// Round-trip: decode the golden bytes and re-encode.
			raw, _ := hex.DecodeString(c.hex)
			out := reflect.New(reflect.TypeOf(c.msg))
			if err := Unmarshal(raw, out.Interface()); err != nil {
				t.Fatal(err)
			}
			again, err := Marshal(out.Elem().Interface())
			if err != nil {
				t.Fatal(err)
			}
			if hex.EncodeToString(again) != c.hex {
				t.Fatalf("round-trip mismatch\n got %s\nwant %s", hex.EncodeToString(again), c.hex)
			}
		})
	}
}

type allPrims struct {
	B   bool
	I8  int8
	U8  uint8
	I16 int16
	U16 uint16
	I32 int32
	U32 uint32
	I64 int64
	U64 uint64
	F32 float32
	F64 float64
	S   string
	Seq []int32
	Arr [3]uint8
}

func TestRoundTripAllPrimitives(t *testing.T) {
	in := allPrims{
		B: true, I8: -8, U8: 8, I16: -16, U16: 16, I32: -32, U32: 32,
		I64: -64, U64: 64, F32: 0.5, F64: -2.5, S: "hi",
		Seq: []int32{1, 2, 3}, Arr: [3]uint8{9, 8, 7},
	}
	b, err := Marshal(in)
	if err != nil {
		t.Fatal(err)
	}
	var out allPrims
	if err := Unmarshal(b, &out); err != nil {
		t.Fatal(err)
	}
	if !reflect.DeepEqual(in, out) {
		t.Fatalf("round trip: got %+v, want %+v", out, in)
	}
}

func TestUnmarshalTruncated(t *testing.T) {
	var out msgs.Twist
	if err := Unmarshal([]byte{0x00, 0x01, 0x00, 0x00, 0x01}, &out); err == nil {
		t.Fatal("expected truncation error")
	}
	if err := Unmarshal([]byte{0x00}, &out); err == nil {
		t.Fatal("expected short-header error")
	}
}
