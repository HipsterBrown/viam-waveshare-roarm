package geometry

import (
	"os/exec"
	"strings"
	"testing"
)

// The layout rules from the spec: internal packages never import components,
// and the gripper and simulated components never import the hardware arm.
func TestImportGraph(t *testing.T) {
	deps := func(pkg string) string {
		out, err := exec.Command("go", "list", "-deps", "waveshareroarm/"+pkg).CombinedOutput()
		if err != nil {
			t.Fatalf("go list %s: %v\n%s", pkg, err, out)
		}
		return string(out)
	}
	for _, pkg := range []string{"internal/roarm", "internal/geometry", "internal/testfake"} {
		if strings.Contains(deps(pkg), "waveshareroarm/components/") {
			t.Fatalf("%s imports a components package", pkg)
		}
	}
	for _, pkg := range []string{"components/gripper", "components/simulated"} {
		if strings.Contains(deps(pkg), "waveshareroarm/components/arm") {
			t.Fatalf("%s imports components/arm", pkg)
		}
	}
}
