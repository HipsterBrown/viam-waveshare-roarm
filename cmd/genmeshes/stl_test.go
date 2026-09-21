package main

import (
	"bytes"
	"encoding/binary"
	"math"
	"testing"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/spatialmath"
)

// unitCubeSTL is a binary STL of the 12-triangle cube spanning (0..10) mm.
func unitCubeSTL() []byte {
	var b bytes.Buffer
	b.Write(make([]byte, 80))
	tris := cubeTriangles(10)
	binary.Write(&b, binary.LittleEndian, uint32(len(tris)))
	for _, t := range tris {
		binary.Write(&b, binary.LittleEndian, [3]float32{0, 0, 0})
		for _, p := range t {
			binary.Write(&b, binary.LittleEndian, [3]float32{float32(p.X), float32(p.Y), float32(p.Z)})
		}
		binary.Write(&b, binary.LittleEndian, uint16(0))
	}
	return b.Bytes()
}

func cubeTriangles(s float64) [][3]r3.Vector {
	v := func(x, y, z float64) r3.Vector { return r3.Vector{X: x * s, Y: y * s, Z: z * s} }
	q := func(a, b, c, d r3.Vector) [][3]r3.Vector { return [][3]r3.Vector{{a, b, c}, {a, c, d}} }
	var t [][3]r3.Vector
	t = append(t, q(v(0, 0, 0), v(1, 0, 0), v(1, 1, 0), v(0, 1, 0))...)
	t = append(t, q(v(0, 0, 1), v(1, 0, 1), v(1, 1, 1), v(0, 1, 1))...)
	t = append(t, q(v(0, 0, 0), v(1, 0, 0), v(1, 0, 1), v(0, 0, 1))...)
	t = append(t, q(v(0, 1, 0), v(1, 1, 0), v(1, 1, 1), v(0, 1, 1))...)
	t = append(t, q(v(0, 0, 0), v(0, 1, 0), v(0, 1, 1), v(0, 0, 1))...)
	t = append(t, q(v(1, 0, 0), v(1, 1, 0), v(1, 1, 1), v(1, 0, 1))...)
	return t
}

func TestReadSTL(t *testing.T) {
	tris, err := readSTL(unitCubeSTL())
	if err != nil || len(tris) != 12 {
		t.Fatalf("tris %d err %v", len(tris), err)
	}
	lo, hi := aabb(tris)
	if lo != (r3.Vector{}) || hi != (r3.Vector{X: 10, Y: 10, Z: 10}) {
		t.Fatalf("aabb %v %v", lo, hi)
	}
}

func TestTransformAndCentre(t *testing.T) {
	tris, _ := readSTL(unitCubeSTL())
	moved := transformTris(tris, spatialmath.NewPoseFromPoint(r3.Vector{X: 100}))
	lo, hi := aabb(moved)
	c := lo.Add(hi).Mul(0.5)
	if c != (r3.Vector{X: 105, Y: 5, Z: 5}) {
		t.Fatalf("centre %v", c)
	}
	centred := translateTris(moved, c.Mul(-1))
	lo, hi = aabb(centred)
	if math.Abs(lo.X+5) > 1e-9 || math.Abs(hi.X-5) > 1e-9 {
		t.Fatalf("not centred: %v %v", lo, hi)
	}
}

func TestWeld(t *testing.T) {
	tris, _ := readSTL(unitCubeSTL())
	pos, idx := weld(tris)
	if len(pos) != 8 || len(idx) != 36 {
		t.Fatalf("weld: %d vertices, %d indices; want 8 and 36", len(pos), len(idx))
	}
}
