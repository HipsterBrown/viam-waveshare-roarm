package geometry

import (
	"encoding/binary"
	"encoding/json"
	"math"
	"testing"

	"github.com/golang/geo/r3"
	commonpb "go.viam.com/api/common/v1"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
)

var zeros5 = []referenceframe.Input{0, 0, 0, 0, 0}

// homePose is the arm extended: elbow at 90 degrees, everything else zero.
var homePose = []referenceframe.Input{0, 0, math.Pi / 2, 0, 0}

func stripGeometry(t *testing.T, raw []byte) referenceframe.ModelConfigJSON {
	t.Helper()
	var cfg referenceframe.ModelConfigJSON
	if err := json.Unmarshal(raw, &cfg); err != nil {
		t.Fatal(err)
	}
	for i := range cfg.Links {
		cfg.Links[i].Geometry = nil
	}
	return cfg
}

func TestBoxAndMeshModelsShareTheChain(t *testing.T) {
	a, b := stripGeometry(t, boxModelJSON), stripGeometry(t, meshModelJSON)
	ja, _ := json.Marshal(a)
	jb, _ := json.Marshal(b)
	if string(ja) != string(jb) {
		t.Fatal("roarm_m3.json and roarm_m3_mesh.json differ in something other than geometry")
	}
	for _, c := range []string{CollisionBox, CollisionMesh} {
		m, err := ArmModel(c, "arm")
		if err != nil {
			t.Fatalf("%s: %v", c, err)
		}
		if len(m.DoF()) != 5 {
			t.Fatalf("%s: %d DoF", c, len(m.DoF()))
		}
	}
}

// Every arm link's collision mesh, placed at its geometry pose, has the same
// axis-aligned extent as the box in the box model (the box IS the mesh AABB;
// a convex hull keeps the point set's AABB). Catches a mesh on the wrong link
// or decimation drift.
func TestCollisionMeshesMatchTheBoxes(t *testing.T) {
	var boxCfg, meshCfg referenceframe.ModelConfigJSON
	if err := json.Unmarshal(boxModelJSON, &boxCfg); err != nil {
		t.Fatal(err)
	}
	if err := json.Unmarshal(meshModelJSON, &meshCfg); err != nil {
		t.Fatal(err)
	}
	boxes := map[string]*spatialmath.GeometryConfig{}
	for _, l := range boxCfg.Links {
		if l.Geometry != nil {
			boxes[l.ID] = l.Geometry
		}
	}
	checked := 0
	for _, l := range meshCfg.Links {
		if l.Geometry == nil {
			continue
		}
		box, ok := boxes[l.ID]
		if !ok {
			t.Fatalf("mesh model has geometry on %s but the box model does not", l.ID)
		}
		if box.TranslationOffset != l.Geometry.TranslationOffset {
			t.Fatalf("%s: geometry pose differs between box and mesh files (%v vs %v)", l.ID, box.TranslationOffset, l.Geometry.TranslationOffset)
		}
		mesh, err := spatialmath.NewMeshFromProto(spatialmath.NewZeroPose(),
			&commonpb.Mesh{ContentType: l.Geometry.MeshContentType, Mesh: l.Geometry.MeshData}, l.ID)
		if err != nil {
			t.Fatalf("%s: %v", l.ID, err)
		}
		lo, hi := meshAABB(mesh)
		size := hi.Sub(lo)
		for axis, pair := range [][2]float64{{size.X, box.X}, {size.Y, box.Y}, {size.Z, box.Z}} {
			if math.Abs(pair[0]-pair[1]) > 2 {
				t.Fatalf("%s axis %d: mesh extent %.1f vs box %.1f", l.ID, axis, pair[0], pair[1])
			}
		}
		if centre := lo.Add(hi).Mul(0.5); centre.Norm() > 2 {
			t.Fatalf("%s: mesh is not centred on its geometry pose (centre offset %v)", l.ID, centre)
		}
		if bv, mv := box.X*box.Y*box.Z, size.X*size.Y*size.Z; bv > 1.3*mv {
			t.Fatalf("%s: box volume %.0f is more than 1.3x the mesh AABB %.0f", l.ID, bv, mv)
		}
		checked++
	}
	if checked != 6 {
		t.Fatalf("checked %d links, want 6", checked)
	}
}

func TestMeshModelSurvivesTheModuleBoundary(t *testing.T) {
	m, err := ArmModel(CollisionMesh, "arm")
	if err != nil {
		t.Fatal(err)
	}
	resp := referenceframe.KinematicModelToProtobuf(m)
	if resp.Format != commonpb.KinematicsFileFormat_KINEMATICS_FILE_FORMAT_SVA {
		t.Fatalf("format %v", resp.Format)
	}
	back, err := referenceframe.UnmarshalModelJSON(resp.KinematicsData, "arm")
	if err != nil {
		t.Fatal(err)
	}
	gif, err := back.Geometries(zeros5)
	if err != nil {
		t.Fatal(err)
	}
	meshes := 0
	for _, g := range gif.Geometries() {
		if _, ok := g.(*spatialmath.Mesh); ok {
			meshes++
		}
	}
	if meshes != 6 {
		t.Fatalf("%d mesh geometries after the round trip, want 6", meshes)
	}
}

func TestArmMeshesAreValidGLBsUnderBudget(t *testing.T) {
	meshes := ArmMeshes()
	if len(meshes) != 6 {
		t.Fatalf("%d GLBs, want 6", len(meshes))
	}
	total := len(boxModelJSON) + len(meshModelJSON)
	for name, m := range meshes {
		if m.ContentType != "model/gltf-binary" {
			t.Fatalf("%s content type %q", name, m.ContentType)
		}
		if string(m.Mesh[:4]) != "glTF" {
			t.Fatalf("%s: not a GLB", name)
		}
		total += len(m.Mesh)
	}
	for _, f := range []string{"gripper_jaw.ply", "gripper_jaw_collision.ply"} {
		b, err := meshFS.ReadFile("meshes/" + f)
		if err != nil {
			t.Fatal(err)
		}
		total += len(b)
	}
	t.Logf("embedded geometry total: %d bytes", total)
	if total >= 1_000_000 {
		t.Fatalf("embedded geometry is %d bytes; budget is under 1,000,000", total)
	}
}

func TestGripperModelsAndJaw(t *testing.T) {
	for _, c := range []string{CollisionBox, CollisionMesh} {
		m, err := GripperModel(c, "g")
		if err != nil {
			t.Fatalf("%s: %v", c, err)
		}
		if len(m.DoF()) != 0 {
			t.Fatalf("%s: gripper model must be 0-DoF", c)
		}
		p, _ := m.Transform([]referenceframe.Input{})
		if math.Abs(p.Point().Z-GripperMountToTCPMM) > 0.01 {
			t.Fatalf("%s: tcp at %v", c, p.Point())
		}
		if referenceframe.KinematicModelToProtobuf(m).Format != commonpb.KinematicsFileFormat_KINEMATICS_FILE_FORMAT_SVA {
			t.Fatalf("%s: does not ship as SVA", c)
		}
	}
	closed, err := GripperMeshes(GripperJointLimits[0])
	if err != nil || len(closed) != 1 {
		t.Fatalf("closed: %v %d", err, len(closed))
	}
	open, err := GripperMeshes(GripperJointLimits[1])
	if err != nil {
		t.Fatal(err)
	}
	if closed[0].Pose().Point().Sub(open[0].Pose().Point()).Norm() < 5 {
		t.Fatal("opening the jaw did not move the mesh")
	}
	// The hinge axis is +Y in the tool frame (spec 3.1 step 7).
	if math.Abs(JawAxis.Y-1) > 1e-6 {
		t.Fatalf("JawAxis %v, want +Y", JawAxis)
	}
	// The jaw's collision hull has the box's extent, like the arm links.
	ply, _ := meshFS.ReadFile("meshes/gripper_jaw_collision.ply")
	hull, err := spatialmath.NewMeshFromProto(spatialmath.NewZeroPose(), &commonpb.Mesh{ContentType: "ply", Mesh: ply}, "jaw")
	if err != nil {
		t.Fatal(err)
	}
	lo, hi := meshAABB(hull)
	if size := hi.Sub(lo); math.Abs(size.X-JawBox.X) > 2 || math.Abs(size.Y-JawBox.Y) > 2 || math.Abs(size.Z-JawBox.Z) > 2 {
		t.Fatalf("jaw hull extent %v vs JawBox %v", size, JawBox)
	}
}

func meshAABB(m *spatialmath.Mesh) (lo, hi r3.Vector) {
	lo = r3.Vector{X: math.Inf(1), Y: math.Inf(1), Z: math.Inf(1)}
	hi = lo.Mul(-1)
	for _, tri := range m.Triangles() {
		for _, p := range tri.Points() {
			lo = r3.Vector{X: math.Min(lo.X, p.X), Y: math.Min(lo.Y, p.Y), Z: math.Min(lo.Z, p.Z)}
			hi = r3.Vector{X: math.Max(hi.X, p.X), Y: math.Max(hi.Y, p.Y), Z: math.Max(hi.Z, p.Z)}
		}
	}
	return lo, hi
}

// The JSON limits are the module's only joint limits (rdk's arm client
// validates against them too), so they must match the URDF's mechanical
// limits and nothing else may widen them.
func TestModelJointLimitsAreTheURDFLimits(t *testing.T) {
	model, err := ArmModel(CollisionBox, "roarm_m3")
	if err != nil {
		t.Fatal(err)
	}
	wantDeg := [][2]float64{{-180, 180}, {-90, 90}, {-57.3, 169.0}, {-90, 90}, {-180, 180}}
	for i, l := range model.DoF() {
		if math.Abs(l.Min*180/math.Pi-wantDeg[i][0]) > 0.1 || math.Abs(l.Max*180/math.Pi-wantDeg[i][1]) > 0.1 {
			t.Fatalf("joint %d limits %.1f..%.1f deg, want %v", i+1, l.Min*180/math.Pi, l.Max*180/math.Pi, wantDeg[i])
		}
	}
}

// modelWithoutTool parses roarm_m3.json with the tool link removed, so the
// tool offset can be measured against the bare chain.
func modelWithoutTool(t *testing.T) referenceframe.Model {
	t.Helper()
	m := &referenceframe.ModelConfigJSON{}
	if err := json.Unmarshal(boxModelJSON, m); err != nil {
		t.Fatal(err)
	}
	links := m.Links[:0]
	for _, l := range m.Links {
		if l.ID != "tool" {
			links = append(links, l)
		}
	}
	m.Links = links
	model, err := m.ParseConfig("bare")
	if err != nil {
		t.Fatal(err)
	}
	return model
}

// The tool frame sits ToolOffsetMM beyond the link5 (wrist-roll) frame,
// further from the base than link5 itself (i.e. along +Z of the roll frame,
// not into link4). The bare model's leaf is link5. If this fails after a JSON
// edit, the tool translation has the wrong sign or is on the wrong axis.
func TestToolFrameIsTheGripperMount(t *testing.T) {
	withTool, err := ArmModel(CollisionBox, "roarm_m3")
	if err != nil {
		t.Fatal(err)
	}
	bare := modelWithoutTool(t)
	pTool, err := withTool.Transform(homePose)
	if err != nil {
		t.Fatal(err)
	}
	pJoint, err := bare.Transform(homePose)
	if err != nil {
		t.Fatal(err)
	}
	d := pTool.Point().Sub(pJoint.Point()).Norm()
	if math.Abs(d-ToolOffsetMM) > 0.01 {
		t.Fatalf("tool is %.3f mm from link5, want %.3f", d, ToolOffsetMM)
	}
	if pTool.Point().Norm() <= pJoint.Point().Norm() {
		t.Fatalf("tool (%v) is not further from the base than link5 (%v): translation sign is wrong", pTool.Point(), pJoint.Point())
	}
}

// The whole chain, pinned to Waveshare's URDF: at the zero pose the tool sits
// ToolOffsetMM along link5's +Z from link5's world origin (45.147, 0, 557.113).
// cmd/genmeshes prints the same world positions when it regenerates
// roarm_m3.json; if this drifts, the two have diverged.
func TestChainMatchesURDF(t *testing.T) {
	model, err := ArmModel(CollisionBox, "roarm_m3")
	if err != nil {
		t.Fatal(err)
	}
	p, err := model.Transform(make([]referenceframe.Input, len(model.DoF())))
	if err != nil {
		t.Fatal(err)
	}
	want := r3.Vector{X: 45.148, Y: 0, Z: 609.148}
	if p.Point().Sub(want).Norm() > 0.1 {
		t.Fatalf("tool at zero pose is %v, want %v", p.Point(), want)
	}
}

// glbPositions decodes the POSITION accessor of one of our GLBs (accessor 0,
// bufferView 0 at byte offset 0 of the single buffer; see cmd/genmeshes/glb.go).
func glbPositions(t *testing.T, glb []byte) []r3.Vector {
	t.Helper()
	jsonLen := int(binary.LittleEndian.Uint32(glb[12:16]))
	var doc struct {
		Accessors []struct{ Count int } `json:"accessors"`
	}
	if err := json.Unmarshal(glb[20:20+jsonLen], &doc); err != nil {
		t.Fatal(err)
	}
	bin := glb[20+jsonLen+8:]
	out := make([]r3.Vector, doc.Accessors[0].Count)
	for i := range out {
		off := i * 12
		out[i] = r3.Vector{
			X: float64(math.Float32frombits(binary.LittleEndian.Uint32(bin[off:]))),
			Y: float64(math.Float32frombits(binary.LittleEndian.Uint32(bin[off+4:]))),
			Z: float64(math.Float32frombits(binary.LittleEndian.Uint32(bin[off+8:]))),
		}
	}
	return out
}

// Every vertex of a link's visual mesh lies inside that link's collision
// envelope. This is the guard against the hole rdk's hull decimator left in
// link2 (a shaft with no vertices in its middle): the AABB test above cannot
// see a missing middle, this can.
func TestCollisionEnvelopesEncloseTheVisualMeshes(t *testing.T) {
	var meshCfg referenceframe.ModelConfigJSON
	if err := json.Unmarshal(meshModelJSON, &meshCfg); err != nil {
		t.Fatal(err)
	}
	glbs := ArmMeshes()
	for _, l := range meshCfg.Links {
		if l.Geometry == nil {
			continue
		}
		env, err := spatialmath.NewMeshFromProto(spatialmath.NewZeroPose(),
			&commonpb.Mesh{ContentType: l.Geometry.MeshContentType, Mesh: l.Geometry.MeshData}, l.ID)
		if err != nil {
			t.Fatal(err)
		}
		// The envelope is a union of axis-aligned boxes, 12 triangles each.
		tris := env.Triangles()
		if len(tris)%12 != 0 {
			t.Fatalf("%s: %d triangles is not a whole number of boxes", l.ID, len(tris))
		}
		type box struct{ lo, hi r3.Vector }
		var boxes []box
		for i := 0; i < len(tris); i += 12 {
			b := box{lo: r3.Vector{X: math.Inf(1), Y: math.Inf(1), Z: math.Inf(1)}}
			b.hi = b.lo.Mul(-1)
			for _, tr := range tris[i : i+12] {
				for _, p := range tr.Points() {
					b.lo = r3.Vector{X: math.Min(b.lo.X, p.X), Y: math.Min(b.lo.Y, p.Y), Z: math.Min(b.lo.Z, p.Z)}
					b.hi = r3.Vector{X: math.Max(b.hi.X, p.X), Y: math.Max(b.hi.Y, p.Y), Z: math.Max(b.hi.Z, p.Z)}
				}
			}
			boxes = append(boxes, b)
		}
		const tol = 0.5 // mm; PLY metres are written with 6 decimals
		outside := 0
		for _, p := range glbPositions(t, glbs[l.ID].Mesh) {
			in := false
			for _, b := range boxes {
				if p.X >= b.lo.X-tol && p.X <= b.hi.X+tol && p.Y >= b.lo.Y-tol && p.Y <= b.hi.Y+tol && p.Z >= b.lo.Z-tol && p.Z <= b.hi.Z+tol {
					in = true
					break
				}
			}
			if !in {
				outside++
			}
		}
		if outside > 0 {
			t.Fatalf("%s: %d visual-mesh vertices lie outside the collision envelope", l.ID, outside)
		}
	}
}
