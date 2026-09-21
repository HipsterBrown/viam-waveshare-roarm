package main

import (
	"math"
	"os"
	"path/filepath"
	"testing"
)

const fixture = `<?xml version="1.0" ?>
<robot name="roarm_m3" xmlns:xacro="http://www.ros.org/wiki/xacro">
<xacro:include filename="$(find roarm_description)/urdf/roarm_m3/materials.xacro" />
<link name="world"></link>
<link name="base_link"><visual><geometry><mesh filename="file://$(find roarm_description)/meshes/roarm_m3/base_link.stl" scale="0.001 0.001 0.001"/></geometry><material name="silver"/></visual></link>
<link name="link1"></link>
<link name="link2"></link>
<joint name="world_to_base_link" type="fixed">
  <origin xyz="0.0 0.0 0.0701" rpy="0 0 0"/>
  <parent link="world"/><child link="base_link"/>
</joint>
<joint name="base_link_to_link1" type="revolute">
  <origin xyz="0.0 0.0 0.0" rpy="0 0 0"/>
  <parent link="base_link"/><child link="link1"/>
  <axis xyz="0.0 0.0 1.0"/>
  <limit upper="3.1416" lower="-3.1416" effort="100" velocity="100"/>
</joint>
<joint name="link1_to_link2" type="revolute">
  <origin xyz="0.0 0.0 0.051959" rpy="-1.5708 -1.5708 0"/>
  <parent link="link1"/><child link="link2"/>
  <axis xyz="-0.0 0 1.0"/>
  <limit upper="1.5708" lower="-1.5708" effort="100" velocity="100"/>
</joint>
</robot>`

func writeFixture(t *testing.T, body string) string {
	t.Helper()
	p := filepath.Join(t.TempDir(), "roarm_m3.xacro")
	if err := os.WriteFile(p, []byte(body), 0o644); err != nil {
		t.Fatal(err)
	}
	return p
}

func TestReadURDF_JointsAndMillimetres(t *testing.T) {
	joints, err := readURDF(writeFixture(t, fixture))
	if err != nil {
		t.Fatal(err)
	}
	if len(joints) != 3 {
		t.Fatalf("want 3 joints, got %d", len(joints))
	}
	j := joints[2]
	if j.Name != "link1_to_link2" || j.Parent != "link1" || j.Child != "link2" || j.Type != "revolute" {
		t.Fatalf("joint fields: %+v", j)
	}
	if math.Abs(j.XYZ.Z-51.959) > 1e-9 || math.Abs(j.RPY.X+1.5708) > 1e-9 || math.Abs(j.RPY.Y+1.5708) > 1e-9 {
		t.Fatalf("origin not converted to mm / rpy kept: %+v", j)
	}
	if j.Axis.Z != 1 || j.Lower != -1.5708 || j.Upper != 1.5708 {
		t.Fatalf("axis/limits: %+v", j)
	}
}

func TestReadURDF_RejectsUnknownXacro(t *testing.T) {
	bad := fixture[:len(fixture)-len("</robot>")] + `<xacro:property name="x" value="1"/></robot>`
	if _, err := readURDF(writeFixture(t, bad)); err == nil {
		t.Fatal("expected an error for an unhandled xacro element")
	}
}

// World poses at the zero pose, walked from "world". Values are from the real
// xacro: base_link at z 70.1, link1 at the same place (zero joint origin),
// link2 51.959 higher.
func TestZeroPoseWorld(t *testing.T) {
	joints, err := readURDF(writeFixture(t, fixture))
	if err != nil {
		t.Fatal(err)
	}
	world := zeroPoseWorld(joints)
	if got := world["base_link"].Point(); math.Abs(got.Z-70.1) > 1e-6 {
		t.Fatalf("base_link at %v", got)
	}
	if got := world["link2"].Point(); math.Abs(got.Z-122.059) > 1e-6 || math.Abs(got.X) > 1e-6 {
		t.Fatalf("link2 at %v, want (0,0,122.059)", got)
	}
	// link2's frame is rotated by rpy (-pi/2, -pi/2, 0): its +Z axis (the third
	// COLUMN of the rotation matrix) points along world +Y.
	zAxis := world["link2"].Orientation().RotationMatrix().Col(2)
	if math.Abs(zAxis.Y-1) > 1e-6 {
		t.Fatalf("link2 +Z in world = %v, want +Y", zAxis)
	}
}
