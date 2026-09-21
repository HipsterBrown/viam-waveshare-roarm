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

func TestConvexHullOfCubeCorners(t *testing.T) {
	var pts []r3.Vector
	for _, x := range []float64{0, 10} {
		for _, y := range []float64{0, 10} {
			for _, z := range []float64{0, 10} {
				pts = append(pts, r3.Vector{X: x, Y: y, Z: z})
			}
		}
	}
	pts = append(pts, r3.Vector{X: 5, Y: 5, Z: 5}) // interior point must not appear
	hull := convexHull(pts)
	if len(hull) != 12 {
		t.Fatalf("cube hull has %d triangles, want 12", len(hull))
	}
	if n, _ := coverage(pts, [][][3]r3.Vector{hull}, 1e-9); n != 0 {
		t.Fatalf("%d points outside their own hull", n)
	}
	if convexHull([]r3.Vector{{}, {X: 1}, {X: 2}, {X: 3}}) != nil {
		t.Fatal("collinear points must be degenerate")
	}
}

func TestSlabHullsEncloseTheMesh(t *testing.T) {
	tris, _ := readSTL(unitCubeSTL())
	pieces := slabHulls(tris, 4)
	pos, _ := weld(tris)
	if n, gap := coverage(pos, pieces, 0.5); n != 0 {
		t.Fatalf("%d vertices outside the envelope (worst %.2f mm)", n, gap)
	}
	for _, p := range pieces {
		if len(p) > 92 { // a 26-face polytope has at most 48 vertices, 2V-4 triangles
			t.Fatalf("a slab polytope has %d triangles, over the 26-direction bound", len(p))
		}
	}
	// A cube's bounding polytope is the cube itself (plus the 0.05 mm margin).
	lo, hi := aabb(flatten(slabHulls(tris, 1)))
	if hi.Sub(lo).Sub(r3.Vector{X: 10.1, Y: 10.1, Z: 10.1}).Norm() > 1e-6 {
		t.Fatalf("cube polytope extent %v", hi.Sub(lo))
	}
}
