package main

import (
	"encoding/xml"
	"fmt"
	"os"
	"strconv"
	"strings"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/spatialmath"
)

const xacroNS = "http://www.ros.org/wiki/xacro"

// urdfJoint is one <joint>: origin in millimetres, rpy in radians (URDF
// fixed-axis roll/pitch/yaw), axis in the child frame, limits in radians.
type urdfJoint struct {
	Name, Type, Parent, Child string
	XYZ, RPY, Axis            r3.Vector
	Lower, Upper              float64
}

type xmlOrigin struct {
	XYZ string `xml:"xyz,attr"`
	RPY string `xml:"rpy,attr"`
}

type xmlJoint struct {
	Name   string    `xml:"name,attr"`
	Type   string    `xml:"type,attr"`
	Origin xmlOrigin `xml:"origin"`
	Parent struct {
		Link string `xml:"link,attr"`
	} `xml:"parent"`
	Child struct {
		Link string `xml:"link,attr"`
	} `xml:"child"`
	Axis struct {
		XYZ string `xml:"xyz,attr"`
	} `xml:"axis"`
	Limit struct {
		Lower float64 `xml:"lower,attr"`
		Upper float64 `xml:"upper,attr"`
	} `xml:"limit"`
}

// readURDF parses Waveshare's roarm_m3.xacro. The file is plain URDF apart
// from <xacro:include> lines (ignored) and $(find ...) inside mesh paths
// (irrelevant here; meshes are located by name). Any other xacro element,
// or a $( outside a mesh filename, is an error so a future revision that
// starts using macros cannot be silently misread.
func readURDF(path string) ([]urdfJoint, error) {
	raw, err := os.ReadFile(path)
	if err != nil {
		return nil, err
	}
	for i, line := range strings.Split(string(raw), "\n") {
		if strings.Contains(line, "$(") && !strings.Contains(line, "<mesh filename") && !strings.Contains(line, "xacro:include") {
			return nil, fmt.Errorf("%s:%d: unhandled xacro substitution: %s", path, i+1, strings.TrimSpace(line))
		}
	}
	dec := xml.NewDecoder(strings.NewReader(string(raw)))
	var joints []urdfJoint
	for {
		tok, err := dec.Token()
		if err != nil {
			break
		}
		se, ok := tok.(xml.StartElement)
		if !ok {
			continue
		}
		if se.Name.Space == xacroNS {
			if se.Name.Local != "include" {
				return nil, fmt.Errorf("unhandled xacro element <xacro:%s>", se.Name.Local)
			}
			if err := dec.Skip(); err != nil {
				return nil, err
			}
			continue
		}
		if se.Name.Local != "joint" {
			continue
		}
		var xj xmlJoint
		if err := dec.DecodeElement(&xj, &se); err != nil {
			return nil, fmt.Errorf("joint: %w", err)
		}
		xyz, err := vec3(xj.Origin.XYZ, 0)
		if err != nil {
			return nil, fmt.Errorf("joint %s origin xyz: %w", xj.Name, err)
		}
		rpy, err := vec3(xj.Origin.RPY, 0)
		if err != nil {
			return nil, fmt.Errorf("joint %s origin rpy: %w", xj.Name, err)
		}
		axis, err := vec3(xj.Axis.XYZ, 1)
		if err != nil {
			return nil, fmt.Errorf("joint %s axis: %w", xj.Name, err)
		}
		joints = append(joints, urdfJoint{
			Name: xj.Name, Type: xj.Type, Parent: xj.Parent.Link, Child: xj.Child.Link,
			XYZ: xyz.Mul(1000), RPY: rpy, Axis: axis,
			Lower: xj.Limit.Lower, Upper: xj.Limit.Upper,
		})
	}
	if len(joints) == 0 {
		return nil, fmt.Errorf("%s: no joints found", path)
	}
	return joints, nil
}

// vec3 parses "x y z"; an empty string is (0,0,defaultZ), which covers a
// missing <origin> (identity) and a missing <axis> (URDF default z).
func vec3(s string, defaultZ float64) (r3.Vector, error) {
	f := strings.Fields(s)
	if len(f) == 0 {
		return r3.Vector{Z: defaultZ}, nil
	}
	if len(f) != 3 {
		return r3.Vector{}, fmt.Errorf("want 3 numbers, got %q", s)
	}
	var v [3]float64
	for i, x := range f {
		n, err := strconv.ParseFloat(x, 64)
		if err != nil {
			return r3.Vector{}, err
		}
		v[i] = n
	}
	return r3.Vector{X: v[0], Y: v[1], Z: v[2]}, nil
}

// originPose is a joint's <origin> as a pose (mm, URDF fixed-axis rpy).
func originPose(j urdfJoint) spatialmath.Pose {
	return spatialmath.NewPose(j.XYZ, &spatialmath.EulerAngles{Roll: j.RPY.X, Pitch: j.RPY.Y, Yaw: j.RPY.Z})
}

// zeroPoseWorld returns every link's world pose at the zero pose, walking
// from "world" through the joints in whatever order they appear.
func zeroPoseWorld(joints []urdfJoint) map[string]spatialmath.Pose {
	world := map[string]spatialmath.Pose{"world": spatialmath.NewZeroPose()}
	for progress := true; progress; {
		progress = false
		for _, j := range joints {
			p, ok := world[j.Parent]
			if !ok {
				continue
			}
			if _, done := world[j.Child]; done {
				continue
			}
			world[j.Child] = spatialmath.Compose(p, originPose(j))
			progress = true
		}
	}
	return world
}

// jointByChild and jointByParent index the chain for the emitter.
func jointByChild(joints []urdfJoint, child string) (urdfJoint, bool) {
	for _, j := range joints {
		if j.Child == child {
			return j, true
		}
	}
	return urdfJoint{}, false
}

func jointByParent(joints []urdfJoint, parent string) (urdfJoint, bool) {
	for _, j := range joints {
		if j.Parent == parent && j.Type == "revolute" {
			return j, true
		}
	}
	return urdfJoint{}, false
}
