package conductor

import (
	"fmt"
	"reflect"
	"strconv"
	"time"
)

// Param declares a node parameter. Supported types: string, float64, int,
// bool, time.Duration.
//
// Tags: name (default: snake_case of the field name), default (parsed into T).
// v0.1 resolves defaults only; per-environment overrides and dynamic updates
// are on the roadmap (see DESIGN.md).
type Param[T any] struct {
	name string
	val  T
}

// Get returns the resolved parameter value.
func (p *Param[T]) Get() T { return p.val }

// Name returns the resolved parameter name (empty before Run).
func (p *Param[T]) Name() string { return p.name }

func (p *Param[T]) bind(rt *runtimeState, nr *nodeRuntime, field reflect.StructField, ownerPtr reflect.Value) error {
	name := field.Tag.Get("name")
	if name == "" {
		name = snakeCase(field.Name)
	}
	p.name = name
	def := field.Tag.Get("default")
	if def == "" {
		return nil
	}
	if err := parseInto(&p.val, def); err != nil {
		return fmt.Errorf("default %q: %w", def, err)
	}
	return nil
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
			return err
		}
		*v = d
	default:
		return fmt.Errorf("unsupported param type %T", dst)
	}
	return nil
}
