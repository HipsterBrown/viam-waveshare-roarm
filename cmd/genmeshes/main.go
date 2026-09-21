// Command genmeshes regenerates every geometry artifact the module embeds from
// Waveshare's roarm_m3 URDF and STL meshes. Run by hand; outputs are committed.
//
//	go run ./cmd/genmeshes --roarm-ws /path/to/roarm_ws
package main

import (
	"flag"
	"fmt"
	"log"
	"os"
	"path/filepath"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/spatialmath"
)

func main() {
	ws := flag.String("roarm-ws", "", "path to a checkout of github.com/waveshareteam/roarm_ws (branch ros2-humble)")
	out := flag.String("out", "internal/geometry", "output directory (the module's geometry package)")
	slabs := flag.Int("slabs", 6, "bounding-polytope slabs per arm-link collision envelope along its longest axis")
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
	boxes := map[string]linkGeometry{}
	meshes := map[string]linkGeometry{}
	fmt.Printf("\n%-10s %8s %8s %8s %8s  %s\n", "link", "tris", "env", "glb", "ply", "box centre / size (mm)")
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
		pieces := slabHulls(centred, *slabs)
		if n, gap := coverage(pos, pieces, coverageTolMM); n > 0 {
			log.Fatalf("%s: collision envelope leaves %d of %d vertices outside it (worst %.1f mm)", link, n, len(pos), gap)
		}
		envelope := toMesh(flatten(pieces), link)
		// The collision PLY travels inline in roarm_m3_mesh.json; no standalone file.
		ply := envelope.TrianglesToPLYBytes(false)
		boxes[link] = linkGeometry{Center: c, Size: hi.Sub(lo)}
		meshes[link] = linkGeometry{Center: c, PLY: ply}
		fmt.Printf("%-10s %8d %8d %8d %8d  %s / %s\n", link, len(tris), len(envelope.Triangles()), len(glb), len(ply),
			mm(c), mm(hi.Sub(lo)))
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

	if err := emitJaw(joints, world, meshDir, *out, jawSlabs); err != nil {
		log.Fatal(err)
	}
}

// coverageTolMM is how far a source vertex may poke out of its collision
// envelope before the generator refuses to ship it (PLY rounding is ~0.001 mm).
const coverageTolMM = 0.5

// jawSlabs is the jaw envelope's slab count; the jaw is 78 mm long, so fewer
// slabs than an arm link keep its PLY small without coarsening it much.
const jawSlabs = 4

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
