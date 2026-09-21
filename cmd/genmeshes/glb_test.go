package main

import (
	"encoding/binary"
	"encoding/json"
	"testing"

	"github.com/golang/geo/r3"
)

func TestWriteGLB(t *testing.T) {
	tris, _ := readSTL(unitCubeSTL())
	pos, idx := weld(tris)
	glb := writeGLB("link1", pos, idx)
	if string(glb[:4]) != "glTF" || binary.LittleEndian.Uint32(glb[4:8]) != 2 {
		t.Fatalf("bad header %q", glb[:8])
	}
	if int(binary.LittleEndian.Uint32(glb[8:12])) != len(glb) {
		t.Fatal("total length field mismatch")
	}
	jsonLen := int(binary.LittleEndian.Uint32(glb[12:16]))
	var doc struct {
		Nodes []struct {
			Name  string    `json:"name"`
			Scale []float64 `json:"scale"`
		} `json:"nodes"`
		Accessors []struct {
			Count int       `json:"count"`
			Max   []float64 `json:"max"`
		} `json:"accessors"`
		Buffers []struct {
			ByteLength int `json:"byteLength"`
		} `json:"buffers"`
	}
	if err := json.Unmarshal(glb[20:20+jsonLen], &doc); err != nil {
		t.Fatal(err)
	}
	if doc.Nodes[0].Name != "link1" || doc.Nodes[0].Scale[0] != 0.001 {
		t.Fatalf("node %+v", doc.Nodes[0])
	}
	if doc.Accessors[0].Count != 8 || doc.Accessors[1].Count != 36 || doc.Accessors[0].Max[0] != 10 {
		t.Fatalf("accessors %+v", doc.Accessors)
	}
	if doc.Buffers[0].ByteLength != 8*12+36*4 {
		t.Fatalf("buffer length %d", doc.Buffers[0].ByteLength)
	}
	_ = r3.Vector{}
}
