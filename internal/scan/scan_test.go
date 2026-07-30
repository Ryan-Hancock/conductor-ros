package scan

import (
	"path/filepath"
	"testing"
)

func TestScanApp(t *testing.T) {
	app, err := ScanApp(filepath.Join("testdata", "sampleapp"))
	if err != nil {
		t.Fatal(err)
	}

	if app.Name != "sample" {
		t.Errorf("Name = %q, want sample", app.Name)
	}
	if len(app.Nodes) != 2 {
		t.Fatalf("got %d nodes, want 2", len(app.Nodes))
	}

	monitor, sensor := app.Nodes[0], app.Nodes[1]
	if monitor.Name != "monitor" || sensor.Name != "sensor" {
		t.Fatalf("node names = %q, %q; want monitor, sensor", monitor.Name, sensor.Name)
	}

	if len(sensor.Pubs) != 1 || sensor.Pubs[0].Topic != "battery" || sensor.Pubs[0].QoS != "sensor" {
		t.Errorf("sensor pubs = %+v", sensor.Pubs)
	}
	if sensor.Pubs[0].GoType != "main.Battery" {
		t.Errorf("sensor pub GoType = %q, want main.Battery (bare identifiers get package-qualified)", sensor.Pubs[0].GoType)
	}
	if len(sensor.Timers) != 1 || sensor.Timers[0].Rate != "10hz" {
		t.Errorf("sensor timers = %+v", sensor.Timers)
	}

	if len(monitor.Subs) != 3 {
		t.Fatalf("monitor subs = %+v", monitor.Subs)
	}
	if monitor.Subs[1].GoType != "msgs.Bool" {
		t.Errorf("estop GoType = %q, want msgs.Bool", monitor.Subs[1].GoType)
	}
	if len(monitor.Params) != 1 || monitor.Params[0].Name != "low_voltage" || monitor.Params[0].Default != "11.1" {
		t.Errorf("monitor params = %+v", monitor.Params)
	}
	if _, ok := monitor.Methods["OnBatt"]; !ok {
		t.Error("monitor methods missing OnBatt")
	}

	if got := app.Messages["main.Battery"]; got != "sample_msgs/msg/Battery" {
		t.Errorf("Messages[main.Battery] = %q", got)
	}
	if got := app.Messages["msgs.Bool"]; got != "std_msgs/msg/Bool" {
		t.Errorf("Messages[msgs.Bool] = %q (module-wide //ros:type scan should find the msgs package)", got)
	}

	if len(app.Externals) != 1 || app.Externals[0].Topic != "estop" {
		t.Errorf("externals = %+v", app.Externals)
	}
}

// Missions, frames and the calls written in code all come out of the same
// syntactic pass.
func TestScanMissionsAndFrames(t *testing.T) {
	app, err := ScanApp(filepath.Join("testdata", "missionapp"))
	if err != nil {
		t.Fatal(err)
	}

	var courier, perception *Node
	for _, n := range app.Nodes {
		switch n.Name {
		case "courier":
			courier = n
		case "perception":
			perception = n
		}
	}
	if courier == nil || perception == nil {
		t.Fatalf("nodes = %+v, want courier and perception", app.Nodes)
	}

	if len(courier.Missions) != 1 || courier.Missions[0].Start != "pickup" || courier.Missions[0].Name != "trip" {
		t.Fatalf("missions = %+v", courier.Missions)
	}
	if len(courier.Steps) != 5 {
		t.Fatalf("%d steps, want 5: %+v", len(courier.Steps), courier.Steps)
	}
	transit := courier.Steps[1]
	if transit.Name != "transit" || transit.Next != "dropof" || transit.Fail != "recharge" || transit.Timeout != "2m" {
		t.Errorf("transit = %+v", transit)
	}

	// Task.Goto with a literal argument is recorded against the method it is
	// written in, which is what ties a transition to its step.
	var gotos []Call
	for _, c := range courier.Calls {
		if c.Method == "Goto" {
			gotos = append(gotos, c)
		}
	}
	if len(gotos) != 1 || gotos[0].In != "OnTransit" || gotos[0].Args[0] != "nowhere" {
		t.Errorf("goto calls = %+v", gotos)
	}

	if perception.TF == nil || perception.TF.Field != "TF" {
		t.Errorf("perception TF = %+v", perception.TF)
	}
	if perception.Pubs[0].Frame != "camera" {
		t.Errorf("frame tag = %q, want camera", perception.Pubs[0].Frame)
	}
	var lookups []Call
	for _, c := range perception.Calls {
		if c.Method == "Lookup" {
			lookups = append(lookups, c)
		}
	}
	if len(lookups) != 1 || lookups[0].Recv != "p.TF" || lookups[0].Args[1] != "laser" {
		t.Errorf("lookup calls = %+v", lookups)
	}

	if app.Frames == nil || app.FramesFile != "frames.json" {
		t.Fatalf("frames = %+v, file %q", app.Frames, app.FramesFile)
	}
	if len(app.Frames.Fixed()) != 1 || len(app.Frames.Transforms) != 3 {
		t.Errorf("frames = %+v", app.Frames.Transforms)
	}
	if !app.Stamped["main.Cloud"] || app.Stamped["main.Reading"] {
		t.Errorf("stamped = %+v; Cloud has a Header, Reading does not", app.Stamped)
	}
}
