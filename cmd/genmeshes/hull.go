package main

import (
	"math"

	"github.com/golang/geo/r3"
)

// convexHull returns the convex hull of pts as outward-wound triangles, or nil
// when the points are degenerate (fewer than four non-coplanar points).
// Incremental construction: start from an extreme tetrahedron, then for each
// remaining point outside the current hull delete the faces it can see and
// fan new faces from the horizon edges to the point. Sizes here are a few
// thousand points per slab, where O(points x faces) is fine.
func convexHull(pts []r3.Vector) [][3]r3.Vector {
	const eps = 1e-6
	seen := map[[3]float64]bool{}
	var p []r3.Vector
	for _, v := range pts {
		k := [3]float64{v.X, v.Y, v.Z}
		if !seen[k] {
			seen[k] = true
			p = append(p, v)
		}
	}
	if len(p) < 4 {
		return nil
	}

	// Extreme tetrahedron: x extremes, then the point farthest from that
	// line, then the point farthest from that plane.
	i0, i1 := 0, 0
	for i, v := range p {
		if v.X < p[i0].X {
			i0 = i
		}
		if v.X > p[i1].X {
			i1 = i
		}
	}
	if i0 == i1 {
		return nil
	}
	line := p[i1].Sub(p[i0])
	i2, best := -1, eps
	for i, v := range p {
		if d := line.Cross(v.Sub(p[i0])).Norm() / line.Norm(); d > best {
			best, i2 = d, i
		}
	}
	if i2 < 0 {
		return nil
	}
	plane := line.Cross(p[i2].Sub(p[i0])).Normalize()
	i3, best := -1, eps
	for i, v := range p {
		if d := math.Abs(plane.Dot(v.Sub(p[i0]))); d > best {
			best, i3 = d, i
		}
	}
	if i3 < 0 {
		return nil
	}
	inside := p[i0].Add(p[i1]).Add(p[i2]).Add(p[i3]).Mul(0.25)

	type face struct {
		a, b, c int
		n       r3.Vector
		off     float64
	}
	mk := func(a, b, c int) face {
		n := p[b].Sub(p[a]).Cross(p[c].Sub(p[a])).Normalize()
		if n.Dot(inside.Sub(p[a])) > 0 {
			b, c = c, b
			n = n.Mul(-1)
		}
		return face{a, b, c, n, n.Dot(p[a])}
	}
	faces := []face{mk(i0, i1, i2), mk(i0, i1, i3), mk(i0, i2, i3), mk(i1, i2, i3)}

	for i := range p {
		if i == i0 || i == i1 || i == i2 || i == i3 {
			continue
		}
		var keep []face
		visible := map[[2]int]bool{}
		for _, f := range faces {
			if f.n.Dot(p[i])-f.off > eps {
				visible[[2]int{f.a, f.b}] = true
				visible[[2]int{f.b, f.c}] = true
				visible[[2]int{f.c, f.a}] = true
			} else {
				keep = append(keep, f)
			}
		}
		if len(visible) == 0 {
			continue
		}
		// Adjacent faces share an edge in opposite directions, so a visible
		// face's edge is on the horizon when its reverse is not also visible.
		for e := range visible {
			if !visible[[2]int{e[1], e[0]}] {
				keep = append(keep, mk(e[0], e[1], i))
			}
		}
		faces = keep
	}

	out := make([][3]r3.Vector, 0, len(faces))
	for _, f := range faces {
		out = append(out, [3]r3.Vector{p[f.a], p[f.b], p[f.c]})
	}
	return out
}

// dopDirs are the 26 face normals of the bounding polytope: the 6 axes, the
// 12 edge diagonals and the 8 corner diagonals of a cube, unit length.
var dopDirs = func() []r3.Vector {
	var out []r3.Vector
	for x := -1.0; x <= 1; x++ {
		for y := -1.0; y <= 1; y++ {
			for z := -1.0; z <= 1; z++ {
				if x == 0 && y == 0 && z == 0 {
					continue
				}
				out = append(out, r3.Vector{X: x, Y: y, Z: z}.Normalize())
			}
		}
	}
	return out
}()

// boundingPolytope is the tightest convex polytope with faces along dopDirs
// that contains pts, pushed out by margin, as outward-wound triangles. Its
// face count is bounded by the 26 directions whatever the CAD's tessellation,
// so it needs no simplification; it chamfers the corners and edges of the
// bounding box wherever the mesh does not reach them. Vertices come from
// intersecting every triple of face planes and keeping the intersections that
// satisfy all 26 constraints.
func boundingPolytope(pts []r3.Vector, margin float64) [][3]r3.Vector {
	if len(pts) == 0 {
		return nil
	}
	sup := make([]float64, len(dopDirs))
	for k, d := range dopDirs {
		sup[k] = math.Inf(-1)
		for _, p := range pts {
			sup[k] = math.Max(sup[k], d.Dot(p))
		}
		sup[k] += margin
	}
	const eps = 1e-6
	var verts []r3.Vector
	for i := 0; i < len(dopDirs); i++ {
		for j := i + 1; j < len(dopDirs); j++ {
			for k := j + 1; k < len(dopDirs); k++ {
				a, b, c := dopDirs[i], dopDirs[j], dopDirs[k]
				det := a.Dot(b.Cross(c))
				if math.Abs(det) < 1e-9 {
					continue
				}
				// Cramer's rule for a.x=sup[i], b.x=sup[j], c.x=sup[k].
				x := b.Cross(c).Mul(sup[i]).Add(c.Cross(a).Mul(sup[j])).Add(a.Cross(b).Mul(sup[k])).Mul(1 / det)
				ok := true
				for m, d := range dopDirs {
					if d.Dot(x) > sup[m]+eps {
						ok = false
						break
					}
				}
				if ok {
					verts = append(verts, x)
				}
			}
		}
	}
	return convexHull(verts)
}

// slabHulls is the collision decimation: the mesh is cut into n slabs along
// its longest axis and each slab becomes the bounding polytope of every
// triangle touching it. Each triangle lies inside the polytope of every slab
// it touches, so the union encloses the mesh by construction. A degenerate
// slab falls back to its bounding box. Pieces are returned separately so
// coverage can be checked per convex piece.
func slabHulls(tris [][3]r3.Vector, n int) [][][3]r3.Vector {
	const margin = 0.05
	lo, hi := aabb(tris)
	ext := hi.Sub(lo)
	axis := 0
	if ext.Y > ext.X {
		axis = 1
	}
	if ext.Z > []float64{ext.X, ext.Y, ext.Z}[axis] {
		axis = 2
	}
	get := func(v r3.Vector) float64 { return []float64{v.X, v.Y, v.Z}[axis] }
	start, width := get(lo), get(ext)/float64(n)
	slab := func(x float64) int {
		i := int((x - start) / width)
		return max(0, min(n-1, i))
	}
	members := make([][]r3.Vector, n)
	for _, t := range tris {
		a, b := n, -1
		for _, p := range t {
			i := slab(get(p))
			a, b = min(a, i), max(b, i)
		}
		for i := a; i <= b; i++ {
			members[i] = append(members[i], t[0], t[1], t[2])
		}
	}
	var pieces [][][3]r3.Vector
	for _, pts := range members {
		if len(pts) == 0 {
			continue
		}
		hull := boundingPolytope(pts, margin)
		if hull == nil {
			slo, shi := aabb(triplets(pts))
			m := r3.Vector{X: margin, Y: margin, Z: margin}
			hull = boxTris(slo.Sub(m), shi.Add(m))
		}
		pieces = append(pieces, hull)
	}
	return pieces
}

// triplets regroups a flat vertex list (3 per triangle) into triangles.
func triplets(pts []r3.Vector) [][3]r3.Vector {
	out := make([][3]r3.Vector, 0, len(pts)/3)
	for i := 0; i+2 < len(pts); i += 3 {
		out = append(out, [3]r3.Vector{pts[i], pts[i+1], pts[i+2]})
	}
	return out
}

// flatten concatenates pieces into one triangle list.
func flatten(pieces [][][3]r3.Vector) [][3]r3.Vector {
	var out [][3]r3.Vector
	for _, p := range pieces {
		out = append(out, p...)
	}
	return out
}

// coverage returns how many points lie outside every convex piece by more
// than tol (mm) and the worst such distance. The envelope is correct by
// construction; this is the guard that keeps it so.
func coverage(points []r3.Vector, pieces [][][3]r3.Vector, tol float64) (outside int, worst float64) {
	for _, p := range points {
		best := math.Inf(1)
		for _, piece := range pieces {
			d := 0.0
			for _, f := range piece {
				n := f[1].Sub(f[0]).Cross(f[2].Sub(f[0]))
				if n.Norm() == 0 {
					continue
				}
				d = math.Max(d, n.Normalize().Dot(p.Sub(f[0])))
			}
			best = math.Min(best, d)
		}
		if best > tol {
			outside++
			worst = math.Max(worst, best)
		}
	}
	return outside, worst
}
