// Package cdr implements OMG CDR (XCDR1, little-endian) serialization for
// conductor message structs — the payload format ROS 2 middlewares exchange.
// Marshal prepends the 4-byte RTPS encapsulation header {0x00, 0x01, 0x00,
// 0x00} (CDR_LE) and pads the result to a 4-byte boundary, byte-compatible
// with Fast CDR as used by rclpy/rclcpp (verified against
// rclpy.serialization golden vectors in cdr_test.go).
//
// Mapping: Go bool/int8..int64/uint8..uint64/float32/float64/string map to
// their IDL counterparts; slices are IDL sequences; arrays are IDL arrays;
// structs serialize exported fields in declaration order. time.Time and
// time.Duration map to builtin_interfaces Time/Duration {int32 sec, uint32
// nanosec}. Alignment is relative to the start of the body (after the
// encapsulation header).
package cdr

import (
	"encoding/binary"
	"errors"
	"fmt"
	"math"
	"reflect"
	"time"
)

var encapsulationLE = []byte{0x00, 0x01, 0x00, 0x00}

var (
	timeType     = reflect.TypeOf(time.Time{})
	durationType = reflect.TypeOf(time.Duration(0))
)

// Marshal serializes v (a message struct or pointer to one).
func Marshal(v any) ([]byte, error) {
	rv := reflect.ValueOf(v)
	if rv.Kind() == reflect.Pointer {
		rv = rv.Elem()
	}
	e := &encoder{}
	if err := e.value(rv); err != nil {
		return nil, err
	}
	e.align(4)
	out := make([]byte, 0, len(e.buf)+4)
	out = append(out, encapsulationLE...)
	return append(out, e.buf...), nil
}

// Unmarshal deserializes data (including encapsulation header) into out,
// which must be a non-nil pointer to a message struct.
func Unmarshal(data []byte, out any) error {
	rv := reflect.ValueOf(out)
	if rv.Kind() != reflect.Pointer || rv.IsNil() {
		return errors.New("cdr: Unmarshal target must be a non-nil pointer")
	}
	if len(data) < 4 {
		return errors.New("cdr: payload shorter than encapsulation header")
	}
	if data[0] != 0x00 || data[1] != 0x01 {
		return fmt.Errorf("cdr: unsupported encapsulation % x (only CDR_LE)", data[:2])
	}
	d := &decoder{buf: data[4:]}
	return d.value(rv.Elem())
}

type encoder struct {
	buf []byte
}

func (e *encoder) align(n int) {
	for len(e.buf)%n != 0 {
		e.buf = append(e.buf, 0)
	}
}

func (e *encoder) u16(v uint16) {
	e.align(2)
	e.buf = binary.LittleEndian.AppendUint16(e.buf, v)
}

func (e *encoder) u32(v uint32) {
	e.align(4)
	e.buf = binary.LittleEndian.AppendUint32(e.buf, v)
}

func (e *encoder) u64(v uint64) {
	e.align(8)
	e.buf = binary.LittleEndian.AppendUint64(e.buf, v)
}

func (e *encoder) value(v reflect.Value) error {
	switch v.Type() {
	case timeType:
		t := v.Interface().(time.Time)
		var sec int64
		var nsec int
		if !t.IsZero() {
			sec, nsec = t.Unix(), t.Nanosecond()
		}
		e.u32(uint32(int32(sec)))
		e.u32(uint32(nsec))
		return nil
	case durationType:
		d := v.Interface().(time.Duration)
		e.u32(uint32(int32(d / time.Second)))
		e.u32(uint32(d % time.Second))
		return nil
	}
	switch v.Kind() {
	case reflect.Bool:
		b := byte(0)
		if v.Bool() {
			b = 1
		}
		e.buf = append(e.buf, b)
	case reflect.Int8:
		e.buf = append(e.buf, byte(v.Int()))
	case reflect.Uint8:
		e.buf = append(e.buf, byte(v.Uint()))
	case reflect.Int16:
		e.u16(uint16(v.Int()))
	case reflect.Uint16:
		e.u16(uint16(v.Uint()))
	case reflect.Int32:
		e.u32(uint32(v.Int()))
	case reflect.Uint32:
		e.u32(uint32(v.Uint()))
	case reflect.Int64:
		e.u64(uint64(v.Int()))
	case reflect.Uint64:
		e.u64(v.Uint())
	case reflect.Float32:
		e.u32(math.Float32bits(float32(v.Float())))
	case reflect.Float64:
		e.u64(math.Float64bits(v.Float()))
	case reflect.String:
		s := v.String()
		e.u32(uint32(len(s) + 1))
		e.buf = append(e.buf, s...)
		e.buf = append(e.buf, 0)
	case reflect.Slice:
		e.u32(uint32(v.Len()))
		for i := 0; i < v.Len(); i++ {
			if err := e.value(v.Index(i)); err != nil {
				return err
			}
		}
	case reflect.Array:
		for i := 0; i < v.Len(); i++ {
			if err := e.value(v.Index(i)); err != nil {
				return err
			}
		}
	case reflect.Struct:
		t := v.Type()
		for i := 0; i < t.NumField(); i++ {
			if !t.Field(i).IsExported() {
				continue
			}
			if err := e.value(v.Field(i)); err != nil {
				return err
			}
		}
	default:
		return fmt.Errorf("cdr: unsupported type %s", v.Type())
	}
	return nil
}

type decoder struct {
	buf []byte
	off int
}

func (d *decoder) alignAndNeed(align, n int) error {
	for d.off%align != 0 {
		d.off++
	}
	if d.off+n > len(d.buf) {
		return errors.New("cdr: payload truncated")
	}
	return nil
}

func (d *decoder) u16() (uint16, error) {
	if err := d.alignAndNeed(2, 2); err != nil {
		return 0, err
	}
	v := binary.LittleEndian.Uint16(d.buf[d.off:])
	d.off += 2
	return v, nil
}

func (d *decoder) u32() (uint32, error) {
	if err := d.alignAndNeed(4, 4); err != nil {
		return 0, err
	}
	v := binary.LittleEndian.Uint32(d.buf[d.off:])
	d.off += 4
	return v, nil
}

func (d *decoder) u64() (uint64, error) {
	if err := d.alignAndNeed(8, 8); err != nil {
		return 0, err
	}
	v := binary.LittleEndian.Uint64(d.buf[d.off:])
	d.off += 8
	return v, nil
}

func (d *decoder) byte() (byte, error) {
	if err := d.alignAndNeed(1, 1); err != nil {
		return 0, err
	}
	b := d.buf[d.off]
	d.off++
	return b, nil
}

func (d *decoder) value(v reflect.Value) error {
	switch v.Type() {
	case timeType:
		sec, err := d.u32()
		if err != nil {
			return err
		}
		nsec, err := d.u32()
		if err != nil {
			return err
		}
		v.Set(reflect.ValueOf(time.Unix(int64(int32(sec)), int64(nsec))))
		return nil
	case durationType:
		sec, err := d.u32()
		if err != nil {
			return err
		}
		nsec, err := d.u32()
		if err != nil {
			return err
		}
		v.SetInt(int64(int32(sec))*int64(time.Second) + int64(nsec))
		return nil
	}
	switch v.Kind() {
	case reflect.Bool:
		b, err := d.byte()
		if err != nil {
			return err
		}
		v.SetBool(b != 0)
	case reflect.Int8:
		b, err := d.byte()
		if err != nil {
			return err
		}
		v.SetInt(int64(int8(b)))
	case reflect.Uint8:
		b, err := d.byte()
		if err != nil {
			return err
		}
		v.SetUint(uint64(b))
	case reflect.Int16:
		n, err := d.u16()
		if err != nil {
			return err
		}
		v.SetInt(int64(int16(n)))
	case reflect.Uint16:
		n, err := d.u16()
		if err != nil {
			return err
		}
		v.SetUint(uint64(n))
	case reflect.Int32:
		n, err := d.u32()
		if err != nil {
			return err
		}
		v.SetInt(int64(int32(n)))
	case reflect.Uint32:
		n, err := d.u32()
		if err != nil {
			return err
		}
		v.SetUint(uint64(n))
	case reflect.Int64:
		n, err := d.u64()
		if err != nil {
			return err
		}
		v.SetInt(int64(n))
	case reflect.Uint64:
		n, err := d.u64()
		if err != nil {
			return err
		}
		v.SetUint(n)
	case reflect.Float32:
		n, err := d.u32()
		if err != nil {
			return err
		}
		v.SetFloat(float64(math.Float32frombits(n)))
	case reflect.Float64:
		n, err := d.u64()
		if err != nil {
			return err
		}
		v.SetFloat(math.Float64frombits(n))
	case reflect.String:
		n, err := d.u32()
		if err != nil {
			return err
		}
		if err := d.alignAndNeed(1, int(n)); err != nil {
			return err
		}
		b := d.buf[d.off : d.off+int(n)]
		d.off += int(n)
		if len(b) > 0 && b[len(b)-1] == 0 {
			b = b[:len(b)-1]
		}
		v.SetString(string(b))
	case reflect.Slice:
		n, err := d.u32()
		if err != nil {
			return err
		}
		if int(n) > len(d.buf)-d.off {
			return errors.New("cdr: sequence length exceeds payload")
		}
		s := reflect.MakeSlice(v.Type(), int(n), int(n))
		for i := 0; i < int(n); i++ {
			if err := d.value(s.Index(i)); err != nil {
				return err
			}
		}
		v.Set(s)
	case reflect.Array:
		for i := 0; i < v.Len(); i++ {
			if err := d.value(v.Index(i)); err != nil {
				return err
			}
		}
	case reflect.Struct:
		t := v.Type()
		for i := 0; i < t.NumField(); i++ {
			if !t.Field(i).IsExported() {
				continue
			}
			if err := d.value(v.Field(i)); err != nil {
				return err
			}
		}
	default:
		return fmt.Errorf("cdr: unsupported type %s", v.Type())
	}
	return nil
}
