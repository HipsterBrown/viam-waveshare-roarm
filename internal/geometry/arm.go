package geometry

import (
	_ "embed"
	"encoding/json"

	"github.com/pkg/errors"
	"go.viam.com/rdk/referenceframe"
)

//go:embed roarm_m3.json
var roarmModelJson []byte

// ToolOffsetMM is the distance from the wrist-roll axis to the gripper mount
// along the roll axis, from Waveshare's RoArm-M3 URDF (link5_to_gripper_link
// origin z = 0.052035 m). The "tool" link in roarm_m3.json carries this value;
// bench task B5 confirms it against the physical arm.
const ToolOffsetMM = 52.0

// ArmModel parses the embedded RoArm-M3 kinematics into a referenceframe
// model named name.
func ArmModel(name string) (referenceframe.Model, error) {
	m := &referenceframe.ModelConfigJSON{
		OriginalFile: &referenceframe.ModelFile{
			Bytes:     roarmModelJson,
			Extension: "json",
		},
	}
	err := json.Unmarshal(roarmModelJson, m)
	if err != nil {
		return nil, errors.Wrap(err, "failed to unmarshal json file")
	}

	return m.ParseConfig(name)
}
