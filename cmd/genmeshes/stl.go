package main

import (
	"encoding/binary"
	"fmt"
	"math"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/spatialmath"
)

// readSTL parses a binary STL (80-byte header, uint32 count, 50-byte
// records) in whatever units the file uses; Waveshare's are millimetres.
// rdk's own STL loader is not used because it assumes metres and scales by 1000.
func readSTL(data []byte) ([][3]r3.Vector, error) {
	if len(data) < 84 {
		return nil, fmt.Errorf("STL too short (%d bytes)", len(data))
	}
	n := int(binary.LittleEndian.Uint32(data[80:84]))
	if want := 84 + 50*n; len(data) < want {
		return nil, fmt.Errorf("STL truncated: %d triangles need %d bytes, have %d (ASCII STL is not supported)", n, want, len(data))
	}
	tris := make([][3]r3.Vector, n)
	for i := range tris {
		off := 84 + 50*i + 12 // skip the normal
		for k := 0; k < 3; k++ {
			tris[i][k] = r3.Vector{
				X: float64(math.Float32frombits(binary.LittleEndian.Uint32(data[off:]))),
				Y: float64(math.Float32frombits(binary.LittleEndian.Uint32(data[off+4:]))),
				Z: float64(math.Float32frombits(binary.LittleEndian.Uint32(data[off+8:]))),
			}
			off += 12
		}
	}
	return tris, nil
}

// transformTris applies pose to every vertex.
func transformTris(tris [][3]r3.Vector, pose spatialmath.Pose) [][3]r3.Vector {
	out := make([][3]r3.Vector, len(tris))
	for i, t := range tris {
		for k, p := range t {
			out[i][k] = spatialmath.Compose(pose, spatialmath.NewPoseFromPoint(p)).Point()
		}
	}
	return out
}

func translateTris(tris [][3]r3.Vector, d r3.Vector) [][3]r3.Vector {
	out := make([][3]r3.Vector, len(tris))
	for i, t := range tris {
		for k, p := range t {
			out[i][k] = p.Add(d)
		}
	}
	return out
}

func aabb(tris [][3]r3.Vector) (lo, hi r3.Vector) {
	lo = r3.Vector{X: math.Inf(1), Y: math.Inf(1), Z: math.Inf(1)}
	hi = lo.Mul(-1)
	for _, t := range tris {
		for _, p := range t {
			lo = r3.Vector{X: math.Min(lo.X, p.X), Y: math.Min(lo.Y, p.Y), Z: math.Min(lo.Z, p.Z)}
			hi = r3.Vector{X: math.Max(hi.X, p.X), Y: math.Max(hi.Y, p.Y), Z: math.Max(hi.Z, p.Z)}
		}
	}
	return lo, hi
}

// weld merges vertices that are identical at float32 precision and returns
// an indexed triangle list, which is what the GLB carries.
func weld(tris [][3]r3.Vector) (positions []r3.Vector, indices []uint32) {
	seen := map[[3]float32]uint32{}
	for _, t := range tris {
		for _, p := range t {
			key := [3]float32{float32(p.X), float32(p.Y), float32(p.Z)}
			i, ok := seen[key]
			if !ok {
				i = uint32(len(positions))
				seen[key] = i
				positions = append(positions, r3.Vector{X: float64(key[0]), Y: float64(key[1]), Z: float64(key[2])})
			}
			indices = append(indices, i)
		}
	}
	return positions, indices
}

// toMesh builds an rdk mesh (millimetres in memory) for decimation and PLY output.
func toMesh(tris [][3]r3.Vector, label string) *spatialmath.Mesh {
	ts := make([]*spatialmath.Triangle, len(tris))
	for i, t := range tris {
		ts[i] = spatialmath.NewTriangle(t[0], t[1], t[2])
	}
	return spatialmath.NewMesh(spatialmath.NewZeroPose(), ts, label)
}

// boxTris is the 12-triangle surface of the box [lo, hi], wound outward.
func boxTris(lo, hi r3.Vector) [][3]r3.Vector {
	v := func(x, y, z bool) r3.Vector {
		p := lo
		if x {
			p.X = hi.X
		}
		if y {
			p.Y = hi.Y
		}
		if z {
			p.Z = hi.Z
		}
		return p
	}
	q := func(a, b, c, d r3.Vector) [][3]r3.Vector { return [][3]r3.Vector{{a, b, c}, {a, c, d}} }
	var t [][3]r3.Vector
	t = append(t, q(v(false, false, false), v(false, true, false), v(true, true, false), v(true, false, false))...) // -Z
	t = append(t, q(v(false, false, true), v(true, false, true), v(true, true, true), v(false, true, true))...)     // +Z
	t = append(t, q(v(false, false, false), v(true, false, false), v(true, false, true), v(false, false, true))...) // -Y
	t = append(t, q(v(false, true, false), v(false, true, true), v(true, true, true), v(true, true, false))...)     // +Y
	t = append(t, q(v(false, false, false), v(false, false, true), v(false, true, true), v(false, true, false))...) // -X
	t = append(t, q(v(true, false, false), v(true, true, false), v(true, true, true), v(true, false, true))...)     // +X
	return t
}
