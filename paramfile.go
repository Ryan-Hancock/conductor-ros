package conductor

import (
	"fmt"
	"os"
	"sort"
	"strconv"
	"strings"
)

// Parameter files use the ROS 2 layout:
//
//	node_name:
//	  ros__parameters:
//	    max_speed: 1.5
//	    frame_id: "base_link"
//	    gains: [1.0, 2.0]
//	/**:                 # wildcard: applies to every node
//	  ros__parameters:
//	    use_sim_time: true
//
// Rather than take a YAML dependency, conductor parses exactly this shape
// and reports anything else as an error. The subset is deliberately small:
// two levels of nesting and scalar or inline-list values.

// paramFile maps node name -> parameter name -> raw value text.
type paramFile map[string]map[string]string

// paramWildcard is the node name matching every node, as in ROS.
const paramWildcard = "/**"

// parseParamFile parses ROS-style parameter YAML.
func parseParamFile(path string, src []byte) (paramFile, error) {
	out := paramFile{}
	var node string
	inParams := false

	for i, raw := range strings.Split(string(src), "\n") {
		lineNo := i + 1
		line := stripYAMLComment(raw)
		if strings.TrimSpace(line) == "" {
			continue
		}
		if strings.Contains(line, "\t") {
			return nil, fmt.Errorf("%s:%d: tabs are not valid YAML indentation", path, lineNo)
		}
		indent := len(line) - len(strings.TrimLeft(line, " "))
		content := strings.TrimSpace(line)

		key, value, hasColon := strings.Cut(content, ":")
		if !hasColon {
			return nil, fmt.Errorf("%s:%d: expected \"key: value\", got %q", path, lineNo, content)
		}
		key = strings.TrimSpace(strings.Trim(key, `"`))
		value = strings.TrimSpace(value)

		switch {
		case indent == 0:
			if value != "" {
				return nil, fmt.Errorf("%s:%d: node %q must be followed by ros__parameters, not a value", path, lineNo, key)
			}
			node = key
			inParams = false
			if _, ok := out[node]; !ok {
				out[node] = map[string]string{}
			}
		case node == "":
			return nil, fmt.Errorf("%s:%d: parameter %q appears before any node name", path, lineNo, key)
		case key == "ros__parameters":
			inParams = true
		case !inParams:
			return nil, fmt.Errorf("%s:%d: %q must appear under a ros__parameters block", path, lineNo, key)
		case value == "":
			return nil, fmt.Errorf("%s:%d: parameter %q has no value (nested parameter groups are not supported)", path, lineNo, key)
		default:
			out[node][key] = value
		}
	}
	return out, nil
}

// stripYAMLComment removes a trailing # comment outside quotes.
func stripYAMLComment(line string) string {
	var quote byte
	for i := 0; i < len(line); i++ {
		c := line[i]
		switch {
		case quote != 0:
			if c == quote {
				quote = 0
			}
		case c == '"' || c == '\'':
			quote = c
		case c == '#':
			return line[:i]
		}
	}
	return line
}

// LoadParamFiles reads parameter files in order; later files win. Values are
// returned per node, with wildcard entries merged into every node that
// appears, and kept separately so nodes not named in any file still get
// them.
func LoadParamFiles(paths ...string) (paramFile, error) {
	merged := paramFile{}
	for _, path := range paths {
		src, err := os.ReadFile(path)
		if err != nil {
			return nil, err
		}
		pf, err := parseParamFile(path, src)
		if err != nil {
			return nil, err
		}
		for node, params := range pf {
			if merged[node] == nil {
				merged[node] = map[string]string{}
			}
			for k, v := range params {
				merged[node][k] = v
			}
		}
	}
	return merged, nil
}

// forNode returns the parameter values that apply to a node: wildcard
// entries first, then the node's own, which win.
func (pf paramFile) forNode(node string) map[string]string {
	out := map[string]string{}
	for k, v := range pf[paramWildcard] {
		out[k] = v
	}
	for k, v := range pf[node] {
		out[k] = v
	}
	return out
}

// parseYAMLScalar converts a raw YAML scalar to the Go type the parameter
// was declared with, so a file cannot change a parameter's type.
func parseYAMLScalar(dst any, raw string) error {
	raw = strings.TrimSpace(raw)
	// Inline lists are handled by the caller; strip quotes for scalars.
	if len(raw) >= 2 && (raw[0] == '"' || raw[0] == '\'') && raw[len(raw)-1] == raw[0] {
		raw = raw[1 : len(raw)-1]
	}
	return parseInto(dst, raw)
}

// sortedNodes returns the node names in a parameter file, for stable output.
func (pf paramFile) sortedNodes() []string {
	names := make([]string, 0, len(pf))
	for n := range pf {
		names = append(names, n)
	}
	sort.Strings(names)
	return names
}

// formatYAMLValue renders a Go parameter value the way a ROS parameter file
// expects it, used when generating params.yaml.
func formatYAMLValue(v any) string {
	switch t := v.(type) {
	case string:
		return strconv.Quote(t)
	case bool:
		return strconv.FormatBool(t)
	case int:
		return strconv.Itoa(t)
	case int64:
		return strconv.FormatInt(t, 10)
	case float64:
		return strconv.FormatFloat(t, 'g', -1, 64)
	default:
		return fmt.Sprint(v)
	}
}
