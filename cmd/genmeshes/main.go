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
)

func main() {
	ws := flag.String("roarm-ws", "", "path to a checkout of github.com/waveshareteam/roarm_ws (branch ros2-humble)")
	out := flag.String("out", "internal/geometry", "output directory (the module's geometry package)")
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
		fmt.Printf("%-13s world %v\n", l, round1(world[l].Point()))
	}

	// Task 2: carry the existing boxes over unchanged. Task 3 replaces this
	// with boxes and meshes derived from the STLs.
	geoms, err := existingBoxes(filepath.Join(*out, "roarm_m3.json"))
	if err != nil {
		log.Fatal(err)
	}
	boxJSON, err := emitModel(joints, geoms, "roarm_m3")
	if err != nil {
		log.Fatal(err)
	}
	if err := os.WriteFile(filepath.Join(*out, "roarm_m3.json"), append(boxJSON, '\n'), 0o644); err != nil {
		log.Fatal(err)
	}
	fmt.Println("wrote", filepath.Join(*out, "roarm_m3.json"))
}

// existingBoxes reads the current file's box geometries by link id so Task 2
// can regenerate the kinematics without touching collision geometry yet.
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
