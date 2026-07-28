package conductor

import (
	"fmt"
	"log/slog"
	"net/http"
	"sort"
	"strings"
	"sync"
	"sync/atomic"
	"time"
)

// Metrics are collected for every node and endpoint with no user code, and
// exposed in Prometheus text format. There is no ROS-native metrics story at
// all — teams bolt on their own — so conductor ships one: message counts,
// callback latency, service and goal outcomes, and node lifecycle state.

type counterKey struct {
	name   string // metric name
	labels string // pre-rendered label set
}

type histogram struct {
	count atomic.Uint64
	sumNS atomic.Uint64
}

var (
	metricsMu  sync.RWMutex
	counters   = map[counterKey]*atomic.Uint64{}
	gauges     = map[counterKey]*atomic.Int64{}
	histograms = map[counterKey]*histogram{}
)

func labelString(pairs ...string) string {
	if len(pairs) == 0 {
		return ""
	}
	var b strings.Builder
	for i := 0; i+1 < len(pairs); i += 2 {
		if b.Len() > 0 {
			b.WriteByte(',')
		}
		fmt.Fprintf(&b, "%s=%q", pairs[i], pairs[i+1])
	}
	return b.String()
}

func counter(name string, labels ...string) *atomic.Uint64 {
	key := counterKey{name, labelString(labels...)}
	metricsMu.RLock()
	c, ok := counters[key]
	metricsMu.RUnlock()
	if ok {
		return c
	}
	metricsMu.Lock()
	defer metricsMu.Unlock()
	if c, ok = counters[key]; ok {
		return c
	}
	c = &atomic.Uint64{}
	counters[key] = c
	return c
}

func gauge(name string, labels ...string) *atomic.Int64 {
	key := counterKey{name, labelString(labels...)}
	metricsMu.RLock()
	g, ok := gauges[key]
	metricsMu.RUnlock()
	if ok {
		return g
	}
	metricsMu.Lock()
	defer metricsMu.Unlock()
	if g, ok = gauges[key]; ok {
		return g
	}
	g = &atomic.Int64{}
	gauges[key] = g
	return g
}

func observe(name string, d time.Duration, labels ...string) {
	key := counterKey{name, labelString(labels...)}
	metricsMu.RLock()
	h, ok := histograms[key]
	metricsMu.RUnlock()
	if !ok {
		metricsMu.Lock()
		if h, ok = histograms[key]; !ok {
			h = &histogram{}
			histograms[key] = h
		}
		metricsMu.Unlock()
	}
	h.count.Add(1)
	h.sumNS.Add(uint64(d.Nanoseconds()))
}

// MetricsText renders all collected metrics in Prometheus exposition format.
func MetricsText() string {
	metricsMu.RLock()
	defer metricsMu.RUnlock()

	var lines []string
	render := func(name, labels, value string) {
		if labels == "" {
			lines = append(lines, fmt.Sprintf("%s %s", name, value))
			return
		}
		lines = append(lines, fmt.Sprintf("%s{%s} %s", name, labels, value))
	}
	for key, c := range counters {
		render(key.name, key.labels, fmt.Sprint(c.Load()))
	}
	for key, g := range gauges {
		render(key.name, key.labels, fmt.Sprint(g.Load()))
	}
	for key, h := range histograms {
		render(key.name+"_count", key.labels, fmt.Sprint(h.count.Load()))
		render(key.name+"_sum_seconds", key.labels, fmt.Sprintf("%g", float64(h.sumNS.Load())/1e9))
	}
	sort.Strings(lines)
	return strings.Join(lines, "\n") + "\n"
}

// serveMetrics starts an HTTP server exposing /metrics. It returns the
// server so the caller can shut it down.
func serveMetrics(addr string) *http.Server {
	mux := http.NewServeMux()
	mux.HandleFunc("/metrics", func(w http.ResponseWriter, r *http.Request) {
		w.Header().Set("Content-Type", "text/plain; version=0.0.4")
		fmt.Fprint(w, MetricsText())
	})
	srv := &http.Server{Addr: addr, Handler: mux, ReadHeaderTimeout: 5 * time.Second}
	go func() {
		if err := srv.ListenAndServe(); err != nil && err != http.ErrServerClosed {
			slog.Error("conductor: metrics server", "err", err)
		}
	}()
	slog.Info("conductor: metrics available", "addr", "http://"+addr+"/metrics")
	return srv
}
