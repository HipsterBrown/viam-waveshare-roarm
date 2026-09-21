// Command genmeshes regenerates every geometry artifact the module embeds from
// Waveshare's roarm_m3 URDF and STL meshes. Run by hand; outputs are committed.
//
//	go run ./cmd/genmeshes --roarm-ws /path/to/roarm_ws
package main

import (
	"encoding/json"
	"flag"
	"fmt"
	"log"
	"os"
	"path/filepath"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
)

func main() {
	ws := flag.String("roarm-ws", "", "path to a checkout of github.com/waveshareteam/roarm_ws (branch ros2-humble)")
	out := flag.String("out", "internal/geometry", "output directory (the module's geometry package)")
	collisionTriangles := flag.Int("collision-triangles", 200, "triangle budget per collision envelope (12 per slab)")
	flag.Parse()
	if *ws == "" {
		flag.Usage()
		os.Exit(2)
	}
	desc := filepath.Join(*ws, "src", "roarm_main", "roarm_description")
	joints, err := readURDF(filepath.Join(desc, "urdf", "roarm_m3", "roarm_m3.xacro"))
	if err != nil {
		log.Fatal(err)
	}
	world := zeroPoseWorld(joints)
	for _, l := range append(armLinks, "gripper_link", "hand_tcp") {
		fmt.Printf("%-13s world %s\n", l, mm(world[l].Point()))
	}

	meshDir := filepath.Join(desc, "meshes", "roarm_m3")
	outMeshes := filepath.Join(*out, "meshes")
	if err := os.MkdirAll(outMeshes, 0o755); err != nil {
		log.Fatal(err)
	}
	old, _ := existingBoxes(filepath.Join(*out, "roarm_m3.json"))

	boxes := map[string]linkGeometry{}
	meshes := map[string]linkGeometry{}
	fmt.Printf("\n%-10s %8s %8s %8s %8s  %s\n", "link", "tris", "env", "glb", "ply", "box centre / size (mm)   [old]")
	for _, link := range armLinks {
		tris, err := readSTLFile(filepath.Join(meshDir, link+".stl"))
		if err != nil {
			log.Fatal(err)
		}
		// Align into the SVA geometry frame: the link's parent (input) frame,
		// which at the zero pose is the URDF link frame itself for every arm
		// link except base_link (whose parent is world, 70.1 mm below).
		parentWorld := geometryParentWorld(world, link)
		aligned := transformTris(tris, spatialmath.Compose(spatialmath.PoseInverse(parentWorld), world[link]))
		lo, hi := aabb(aligned)
		c := lo.Add(hi).Mul(0.5)
		centred := translateTris(aligned, c.Mul(-1))

		pos, idx := weld(centred)
		glb := writeGLB(link, pos, idx)
		if err := os.WriteFile(filepath.Join(outMeshes, link+".glb"), glb, 0o644); err != nil {
			log.Fatal(err)
		}
		boxesForLink := slabBoxes(centred, *collisionTriangles/12)
		if n, gap := coverage(pos, boxesForLink, coverageTolMM); n > 0 {
			log.Fatalf("%s: collision envelope leaves %d of %d vertices outside it (worst %.1f mm)", link, n, len(pos), gap)
		}
		envelope := toMesh(boxesTris(boxesForLink), link)
		// The collision PLY travels inline in roarm_m3_mesh.json; no standalone file.
		ply := envelope.TrianglesToPLYBytes(false)
		boxes[link] = linkGeometry{Center: c, Size: hi.Sub(lo), Label: link}
		meshes[link] = linkGeometry{Center: c, PLY: ply, Label: link}
		o := old[link]
		fmt.Printf("%-10s %8d %8d %8d %8d  %s / %s   [%s / %s]\n", link, len(tris), len(envelope.Triangles()), len(glb), len(ply),
			mm(c), mm(hi.Sub(lo)), mm(o.Center), mm(o.Size))
	}

	for name, geoms := range map[string]map[string]linkGeometry{"roarm_m3.json": boxes, "roarm_m3_mesh.json": meshes} {
		js, err := emitModel(joints, geoms, "roarm_m3")
		if err != nil {
			log.Fatal(err)
		}
		if err := os.WriteFile(filepath.Join(*out, name), append(js, '\n'), 0o644); err != nil {
			log.Fatal(err)
		}
		fmt.Println("wrote", filepath.Join(*out, name))
	}

	if err := emitJaw(joints, world, meshDir, *out, *collisionTriangles); err != nil {
		log.Fatal(err)
	}
}

// existingBoxes reads the committed file's box geometries by link id so the
// report can show what a regeneration changed.
func existingBoxes(path string) (map[string]linkGeometry, error) {
	raw, err := os.ReadFile(path)
	if err != nil {
		return nil, err
	}
	var cfg referenceframe.ModelConfigJSON
	if err := json.Unmarshal(raw, &cfg); err != nil {
		return nil, err
	}
	geoms := map[string]linkGeometry{}
	for _, l := range cfg.Links {
		if l.Geometry == nil {
			continue
		}
		geoms[l.ID] = linkGeometry{
			Center: l.Geometry.TranslationOffset,
			Size:   r3.Vector{X: l.Geometry.X, Y: l.Geometry.Y, Z: l.Geometry.Z},
			Label:  l.ID,
		}
	}
	return geoms, nil
}

// coverageTolMM is how far a source vertex may poke out of its collision envelope
// before the generator refuses to ship it (PLY rounding is ~0.001 mm).
const coverageTolMM = 0.5

func readSTLFile(path string) ([][3]r3.Vector, error) {
	raw, err := os.ReadFile(path)
	if err != nil {
		return nil, err
	}
	return readSTL(raw)
}

// geometryParentWorld is the world pose of the frame an SVA link's geometry
// is expressed in: the link's parent. For base_link that is world; for every
// other arm link it is the incoming joint's frame, which at the zero pose
// coincides with the URDF link frame.
func geometryParentWorld(world map[string]spatialmath.Pose, link string) spatialmath.Pose {
	if link == "base_link" {
		return world["world"]
	}
	return world[link]
}
