package conductor

import "sort"

// Conductor derives bringup order from the application graph instead of
// making developers hand-write it. A node depends on whatever it consumes:
// the publisher of a topic it subscribes to, the server of a service or
// action it calls. Dependencies are activated first, so a node is only ever
// activated once the things it needs are already running — the ordering
// teams normally encode by hand in launch files and wait_for_service loops.

// nodeDeps records what one node provides and consumes, collected while its
// fields are bound.
type nodeDeps struct {
	provides map[string]bool // topics published, services/actions served
	consumes map[string]bool // topics subscribed, services/actions called
}

func newNodeDeps() *nodeDeps {
	return &nodeDeps{provides: map[string]bool{}, consumes: map[string]bool{}}
}

func (rt *runtimeState) recordProvides(node, name string) {
	rt.depsFor(node).provides[name] = true
}

func (rt *runtimeState) recordConsumes(node, name string) {
	rt.depsFor(node).consumes[name] = true
}

func (rt *runtimeState) depsFor(node string) *nodeDeps {
	if rt.deps == nil {
		rt.deps = map[string]*nodeDeps{}
	}
	d, ok := rt.deps[node]
	if !ok {
		d = newNodeDeps()
		rt.deps[node] = d
	}
	return d
}

// BringupOrder returns node names ordered so that every node follows the
// nodes it depends on. Cycles are expected in robotics (feedback loops), so
// nodes left in a cycle are appended in declaration order rather than
// treated as an error; cycles is the set of node names involved.
func BringupOrder(nodes []string, deps map[string]*nodeDeps) (order []string, cycles []string) {
	// providers[name] is the set of nodes providing an endpoint.
	providers := map[string][]string{}
	for _, n := range nodes {
		d := deps[n]
		if d == nil {
			continue
		}
		for name := range d.provides {
			providers[name] = append(providers[name], n)
		}
	}

	// edges[n] is the set of nodes n depends on.
	edges := map[string]map[string]bool{}
	indegree := map[string]int{}
	for _, n := range nodes {
		edges[n] = map[string]bool{}
		indegree[n] = 0
	}
	for _, n := range nodes {
		d := deps[n]
		if d == nil {
			continue
		}
		for name := range d.consumes {
			for _, provider := range providers[name] {
				if provider == n || edges[n][provider] {
					continue // self-dependency or already recorded
				}
				edges[n][provider] = true
				indegree[n]++
			}
		}
	}

	// Kahn's algorithm, taking ready nodes in declaration order for stable
	// output.
	position := map[string]int{}
	for i, n := range nodes {
		position[n] = i
	}
	var ready []string
	for _, n := range nodes {
		if indegree[n] == 0 {
			ready = append(ready, n)
		}
	}
	sort.Slice(ready, func(i, j int) bool { return position[ready[i]] < position[ready[j]] })

	done := map[string]bool{}
	for len(ready) > 0 {
		n := ready[0]
		ready = ready[1:]
		order = append(order, n)
		done[n] = true
		var unlocked []string
		for _, other := range nodes {
			if done[other] || !edges[other][n] {
				continue
			}
			indegree[other]--
			if indegree[other] == 0 {
				unlocked = append(unlocked, other)
			}
		}
		sort.Slice(unlocked, func(i, j int) bool { return position[unlocked[i]] < position[unlocked[j]] })
		ready = append(ready, unlocked...)
	}

	for _, n := range nodes {
		if !done[n] {
			cycles = append(cycles, n)
			order = append(order, n)
		}
	}
	return order, cycles
}
