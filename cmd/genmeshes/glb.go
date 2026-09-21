package main

import (
	"bytes"
	"encoding/binary"
	"encoding/json"
	"math"

	"github.com/golang/geo/r3"
)

// writeGLB emits a minimal glTF 2.0 binary: one node named `name` carrying
// one mesh with POSITION (float32, millimetres) and uint32 indices, under a
// 0.001 node scale so the scene reads metres. No normals or materials; the
// viewer shades flat. The node scale mirrors rdk's embedded fake-arm GLBs.
func writeGLB(name string, positions []r3.Vector, indices []uint32) []byte {
	var bin bytes.Buffer
	lo := r3.Vector{X: math.Inf(1), Y: math.Inf(1), Z: math.Inf(1)}
	hi := lo.Mul(-1)
	for _, p := range positions {
		binary.Write(&bin, binary.LittleEndian, [3]float32{float32(p.X), float32(p.Y), float32(p.Z)})
		lo = r3.Vector{X: math.Min(lo.X, p.X), Y: math.Min(lo.Y, p.Y), Z: math.Min(lo.Z, p.Z)}
		hi = r3.Vector{X: math.Max(hi.X, p.X), Y: math.Max(hi.Y, p.Y), Z: math.Max(hi.Z, p.Z)}
	}
	posLen := bin.Len()
	for _, i := range indices {
		binary.Write(&bin, binary.LittleEndian, i)
	}
	idxLen := bin.Len() - posLen

	doc := map[string]any{
		"asset":  map[string]any{"version": "2.0", "generator": "waveshareroarm/cmd/genmeshes"},
		"scene":  0,
		"scenes": []any{map[string]any{"nodes": []int{0}}},
		"nodes":  []any{map[string]any{"name": name, "mesh": 0, "scale": []float64{0.001, 0.001, 0.001}}},
		"meshes": []any{map[string]any{"name": name, "primitives": []any{
			map[string]any{"attributes": map[string]int{"POSITION": 0}, "indices": 1, "mode": 4},
		}}},
		"accessors": []any{
			map[string]any{"bufferView": 0, "componentType": 5126, "count": len(positions), "type": "VEC3",
				"min": []float64{lo.X, lo.Y, lo.Z}, "max": []float64{hi.X, hi.Y, hi.Z}},
			map[string]any{"bufferView": 1, "componentType": 5125, "count": len(indices), "type": "SCALAR"},
		},
		"bufferViews": []any{
			map[string]any{"buffer": 0, "byteOffset": 0, "byteLength": posLen, "target": 34962},
			map[string]any{"buffer": 0, "byteOffset": posLen, "byteLength": idxLen, "target": 34963},
		},
		"buffers": []any{map[string]any{"byteLength": bin.Len()}},
	}
	js, _ := json.Marshal(doc)
	for len(js)%4 != 0 {
		js = append(js, ' ')
	}
	binBytes := bin.Bytes()
	for len(binBytes)%4 != 0 {
		binBytes = append(binBytes, 0)
	}

	var out bytes.Buffer
	total := 12 + 8 + len(js) + 8 + len(binBytes)
	binary.Write(&out, binary.LittleEndian, uint32(0x46546C67)) // "glTF"
	binary.Write(&out, binary.LittleEndian, uint32(2))
	binary.Write(&out, binary.LittleEndian, uint32(total))
	binary.Write(&out, binary.LittleEndian, uint32(len(js)))
	binary.Write(&out, binary.LittleEndian, uint32(0x4E4F534A)) // "JSON"
	out.Write(js)
	binary.Write(&out, binary.LittleEndian, uint32(len(binBytes)))
	binary.Write(&out, binary.LittleEndian, uint32(0x004E4942)) // "BIN\0"
	out.Write(binBytes)
	return out.Bytes()
}
