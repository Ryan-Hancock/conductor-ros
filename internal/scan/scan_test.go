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
