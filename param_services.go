package conductor

import (
	"log/slog"
	"reflect"
	"sort"
	"strings"
	"time"
)

// Each node exposes the standard ROS 2 parameter services, so `ros2 param
// list/get/set/describe` works against conductor nodes like any other, and
// publishes /parameter_events on change.

func bindParamServices(rt *runtimeState, nr *nodeRuntime) error {
	reliableQoS, _ := QoSProfile("reliable")
	eventPub, err := rt.transport.Publisher(TopicSpec{
		Topic: "parameter_events", QoS: reliableQoS,
		Type: reflect.TypeFor[parameterEventMsg](), Node: nr.name,
	})
	if err != nil {
		return err
	}

	byName := func() map[string]*paramHandle {
		m := make(map[string]*paramHandle, len(nr.params))
		for _, h := range nr.params {
			m[h.name] = h
		}
		return m
	}

	serve := func(suffix string, req, res reflect.Type, handle func(any) (any, error)) error {
		return rt.transport.Serve(ServiceSpec{
			Service: nr.name + "/" + suffix, ReqType: req, ResType: res, Node: nr.name,
		}, handle)
	}

	if err := serve("list_parameters",
		reflect.TypeFor[listParametersRequest](), reflect.TypeFor[listParametersResponse](),
		func(reqAny any) (any, error) {
			req := reqAny.(listParametersRequest)
			var names []string
			for _, h := range nr.params {
				if len(req.Prefixes) > 0 && !hasAnyPrefix(h.name, req.Prefixes) {
					continue
				}
				names = append(names, h.name)
			}
			sort.Strings(names)
			return listParametersResponse{Result: listParametersResultMsg{Names: names}}, nil
		}); err != nil {
		return err
	}

	if err := serve("get_parameters",
		reflect.TypeFor[getParametersRequest](), reflect.TypeFor[getParametersResponse](),
		func(reqAny any) (any, error) {
			req := reqAny.(getParametersRequest)
			params := byName()
			values := make([]parameterValueMsg, 0, len(req.Names))
			for _, name := range req.Names {
				if h, ok := params[name]; ok {
					values = append(values, h.value())
				} else {
					values = append(values, parameterValueMsg{Type: paramTypeNotSet})
				}
			}
			return getParametersResponse{Values: values}, nil
		}); err != nil {
		return err
	}

	if err := serve("get_parameter_types",
		reflect.TypeFor[getParameterTypesRequest](), reflect.TypeFor[getParameterTypesResponse](),
		func(reqAny any) (any, error) {
			req := reqAny.(getParameterTypesRequest)
			params := byName()
			types := make([]uint8, 0, len(req.Names))
			for _, name := range req.Names {
				if h, ok := params[name]; ok {
					types = append(types, h.rosType())
				} else {
					types = append(types, paramTypeNotSet)
				}
			}
			return getParameterTypesResponse{Types: types}, nil
		}); err != nil {
		return err
	}

	if err := serve("describe_parameters",
		reflect.TypeFor[describeParametersRequest](), reflect.TypeFor[describeParametersResponse](),
		func(reqAny any) (any, error) {
			req := reqAny.(describeParametersRequest)
			params := byName()
			out := make([]parameterDescriptorMsg, 0, len(req.Names))
			for _, name := range req.Names {
				d := parameterDescriptorMsg{Name: name}
				if h, ok := params[name]; ok {
					d.Type = h.rosType()
					d.Description = "declared by conductor as " + h.typeOf.String()
				}
				out = append(out, d)
			}
			return describeParametersResponse{Descriptors: out}, nil
		}); err != nil {
		return err
	}

	// apply sets one parameter, returning the per-parameter result.
	apply := func(params map[string]*paramHandle, p parameterMsg) (setParametersResultMsg, *parameterMsg) {
		h, ok := params[p.Name]
		if !ok {
			return setParametersResultMsg{Reason: "no such parameter: " + p.Name}, nil
		}
		if err := h.setFromValue(p.Value); err != nil {
			return setParametersResultMsg{Reason: err.Error()}, nil
		}
		slog.Info("conductor: parameter updated", "node", nr.name, "parameter", p.Name)
		counter("conductor_parameter_updates_total", "node", nr.name, "parameter", p.Name).Add(1)
		return setParametersResultMsg{Successful: true}, &parameterMsg{Name: p.Name, Value: h.value()}
	}

	announce := func(changed []parameterMsg) {
		if len(changed) == 0 {
			return
		}
		if err := eventPub(parameterEventMsg{
			Stamp:             time.Now(),
			Node:              "/" + nr.name,
			ChangedParameters: changed,
		}, Metadata{}); err != nil {
			slog.Warn("conductor: parameter event publish failed", "node", nr.name, "err", err)
		}
	}

	if err := serve("set_parameters",
		reflect.TypeFor[setParametersRequest](), reflect.TypeFor[setParametersResponse](),
		func(reqAny any) (any, error) {
			req := reqAny.(setParametersRequest)
			params := byName()
			results := make([]setParametersResultMsg, 0, len(req.Parameters))
			var changed []parameterMsg
			for _, p := range req.Parameters {
				res, applied := apply(params, p)
				results = append(results, res)
				if applied != nil {
					changed = append(changed, *applied)
				}
			}
			announce(changed)
			return setParametersResponse{Results: results}, nil
		}); err != nil {
		return err
	}

	// Atomically means all-or-nothing, so validate every parameter before
	// applying any of them.
	return serve("set_parameters_atomically",
		reflect.TypeFor[setParametersRequest](), reflect.TypeFor[setParametersAtomicallyResponse](),
		func(reqAny any) (any, error) {
			req := reqAny.(setParametersRequest)
			params := byName()
			for _, p := range req.Parameters {
				h, ok := params[p.Name]
				if !ok {
					return setParametersAtomicallyResponse{Result: setParametersResultMsg{
						Reason: "no such parameter: " + p.Name,
					}}, nil
				}
				if h.rosType() != p.Value.Type {
					return setParametersAtomicallyResponse{Result: setParametersResultMsg{
						Reason: "wrong type for parameter " + p.Name,
					}}, nil
				}
			}
			var changed []parameterMsg
			for _, p := range req.Parameters {
				if _, applied := apply(params, p); applied != nil {
					changed = append(changed, *applied)
				}
			}
			announce(changed)
			return setParametersAtomicallyResponse{Result: setParametersResultMsg{Successful: true}}, nil
		})
}

func hasAnyPrefix(name string, prefixes []string) bool {
	for _, p := range prefixes {
		if strings.HasPrefix(name, p) {
			return true
		}
	}
	return false
}
