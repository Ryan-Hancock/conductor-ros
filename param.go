package conductor

import (
	"fmt"
	"reflect"
	"strconv"
	"strings"
	"sync"
	"time"
)

// Param declares a node parameter. Supported types: string, float64, int,
// bool, time.Duration.
//
// Values resolve in order: the default tag, then any parameter files given
// to -params (or selected by -env), then runtime updates through the ROS
// parameter services. Get is safe to call from any goroutine.
//
// Tags: name (default: snake_case of the field name), default (parsed into T).
type Param[T any] struct {
	name string

	mu  sync.RWMutex
	val T
}

// Get returns the current parameter value.
func (p *Param[T]) Get() T {
	p.mu.RLock()
	defer p.mu.RUnlock()
	return p.val
}

// Name returns the resolved parameter name (empty before Run).
func (p *Param[T]) Name() string { return p.name }

// set replaces the value; used by file loading and the parameter services.
func (p *Param[T]) set(v T) {
	p.mu.Lock()
	p.val = v
	p.mu.Unlock()
}

func (p *Param[T]) bind(rt *runtimeState, nr *nodeRuntime, field reflect.StructField, ownerPtr reflect.Value) error {
	name := field.Tag.Get("name")
	if name == "" {
		name = snakeCase(field.Name)
	}
	p.name = name

	if def := field.Tag.Get("default"); def != "" {
		var v T
		if err := parseInto(&v, def); err != nil {
			return fmt.Errorf("default %q: %w", def, err)
		}
		p.set(v)
	}

	// Register with the node so the parameter services can see and change
	// it without knowing its static type.
	h := &paramHandle{
		name: name,
		get:  func() any { return p.Get() },
		set: func(raw string) error {
			var v T
			if err := parseYAMLScalar(&v, raw); err != nil {
				return err
			}
			p.set(v)
			return nil
		},
		typeOf: reflect.TypeFor[T](),
	}
	nr.params = append(nr.params, h)

	// Apply any value supplied by a parameter file.
	if raw, ok := rt.paramValues[nr.name][name]; ok {
		if err := h.set(raw); err != nil {
			return fmt.Errorf("parameter %q from file: %w", name, err)
		}
	}
	return nil
}

// paramHandle is the type-erased view of a Param the parameter services use.
type paramHandle struct {
	name   string
	get    func() any
	set    func(raw string) error
	typeOf reflect.Type
}

// rosType maps the Go parameter type to a rcl_interfaces ParameterType.
func (h *paramHandle) rosType() uint8 {
	switch h.typeOf.Kind() {
	case reflect.Bool:
		return paramTypeBool
	case reflect.Int, reflect.Int64:
		// time.Duration is an int64; ROS has no duration parameter type, so
		// it is exposed as an integer (nanoseconds).
		return paramTypeInteger
	case reflect.Float64:
		return paramTypeDouble
	case reflect.String:
		return paramTypeString
	default:
		return paramTypeNotSet
	}
}

// value renders the parameter as a rcl_interfaces ParameterValue.
func (h *paramHandle) value() parameterValueMsg {
	v := parameterValueMsg{Type: h.rosType()}
	switch got := h.get().(type) {
	case bool:
		v.BoolValue = got
	case int:
		v.IntegerValue = int64(got)
	case int64:
		v.IntegerValue = got
	case time.Duration:
		v.IntegerValue = int64(got)
	case float64:
		v.DoubleValue = got
	case string:
		v.StringValue = got
	}
	return v
}

// setFromValue applies a ParameterValue, rejecting a type change (ROS
// requires matching types unless dynamic typing is declared).
func (h *paramHandle) setFromValue(v parameterValueMsg) error {
	want := h.rosType()
	if v.Type != want {
		return fmt.Errorf("parameter %q is type %d, got %d", h.name, want, v.Type)
	}
	switch want {
	case paramTypeBool:
		return h.set(strconv.FormatBool(v.BoolValue))
	case paramTypeInteger:
		if h.typeOf == reflect.TypeFor[time.Duration]() {
			return h.set(time.Duration(v.IntegerValue).String())
		}
		return h.set(strconv.FormatInt(v.IntegerValue, 10))
	case paramTypeDouble:
		return h.set(strconv.FormatFloat(v.DoubleValue, 'g', -1, 64))
	case paramTypeString:
		return h.set(v.StringValue)
	}
	return fmt.Errorf("parameter %q has an unsupported type", h.name)
}

func parseInto(dst any, s string) error {
	switch v := dst.(type) {
	case *string:
		*v = s
	case *float64:
		f, err := strconv.ParseFloat(s, 64)
		if err != nil {
			return err
		}
		*v = f
	case *int:
		i, err := strconv.Atoi(s)
		if err != nil {
			return err
		}
		*v = i
	case *bool:
		b, err := strconv.ParseBool(s)
		if err != nil {
			return err
		}
		*v = b
	case *time.Duration:
		d, err := time.ParseDuration(s)
		if err != nil {
			// Parameter files and the ROS services carry durations as
			// integer nanoseconds.
			if n, nerr := strconv.ParseInt(strings.TrimSpace(s), 10, 64); nerr == nil {
				*v = time.Duration(n)
				return nil
			}
			return err
		}
		*v = d
	default:
		return fmt.Errorf("unsupported param type %T", dst)
	}
	return nil
}
