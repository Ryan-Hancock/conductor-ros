package msggen

import (
	"bytes"
	"crypto/sha256"
	"encoding/json"
	"fmt"
)

// REP-2011 type hashing (RIHS01): sha256 over a canonical JSON rendering of
// the type description, replicating rosidl_generator_type_description
// exactly — insertion-ordered keys, Python's (', ', ': ') separators,
// default values excluded, referenced types sorted by name. Validated
// against the installed ROS distro's precomputed hashes in msggen_test.go.

// FieldType IDs from type_description_interfaces/msg/FieldType.msg. Scalar
// base IDs; containers add a fixed offset.
var builtinTypeIDs = map[string]int{
	"int8": 2, "uint8": 3, "int16": 4, "uint16": 5,
	"int32": 6, "uint32": 7, "int64": 8, "uint64": 9,
	"float32": 10, "float64": 11,
	"bool": 15, "byte": 16, "string": 17, "wstring": 18,
}

const (
	idNested        = 1
	idBoundedString = 21
	offsetArray     = 48
	offsetBounded   = 96
	offsetUnbounded = 144
)

func typeID(ft FieldType) int {
	var base int
	switch {
	case ft.Nested != "":
		base = idNested
	case ft.StrCapacity > 0:
		base = idBoundedString
	default:
		base = builtinTypeIDs[ft.Builtin]
	}
	switch ft.Kind {
	case Array:
		base += offsetArray
	case BoundedSeq:
		base += offsetBounded
	case UnboundedSeq:
		base += offsetUnbounded
	}
	return base
}

// TypeDescription is a fully resolved type: the type itself plus every
// transitively referenced type, sorted by name.
type TypeDescription struct {
	Individual Individual
	Referenced []Individual
}

// Individual describes one type's fields in hashable form.
type Individual struct {
	TypeName string
	Fields   []DescField
}

type DescField struct {
	Name           string
	TypeID         int
	Capacity       int
	StringCapacity int
	NestedTypeName string
}

func individualOf(m *Message) Individual {
	ind := Individual{TypeName: m.FullName()}
	for _, f := range m.Fields {
		ind.Fields = append(ind.Fields, DescField{
			Name:           f.Name,
			TypeID:         typeID(f.Type),
			Capacity:       f.Type.Capacity,
			StringCapacity: f.Type.StrCapacity,
			NestedTypeName: f.Type.Nested,
		})
	}
	return ind
}

// Hash computes the RIHS01 hash string for the description.
func (td TypeDescription) Hash() string {
	sum := sha256.Sum256(td.hashableJSON())
	return fmt.Sprintf("RIHS01_%x", sum)
}

func (td TypeDescription) hashableJSON() []byte {
	var b bytes.Buffer
	b.WriteString(`{"type_description": `)
	writeIndividual(&b, td.Individual)
	b.WriteString(`, "referenced_type_descriptions": [`)
	for i, r := range td.Referenced {
		if i > 0 {
			b.WriteString(", ")
		}
		writeIndividual(&b, r)
	}
	b.WriteString(`]}`)
	return b.Bytes()
}

func writeIndividual(b *bytes.Buffer, ind Individual) {
	b.WriteString(`{"type_name": `)
	b.Write(jsonString(ind.TypeName))
	b.WriteString(`, "fields": [`)
	for i, f := range ind.Fields {
		if i > 0 {
			b.WriteString(", ")
		}
		fmt.Fprintf(b, `{"name": %s, "type": {"type_id": %d, "capacity": %d, "string_capacity": %d, "nested_type_name": %s}}`,
			jsonString(f.Name), f.TypeID, f.Capacity, f.StringCapacity, jsonString(f.NestedTypeName))
	}
	b.WriteString(`]}`)
}

func jsonString(s string) []byte {
	out, err := json.Marshal(s)
	if err != nil {
		panic(err) // strings cannot fail to marshal
	}
	return out
}
