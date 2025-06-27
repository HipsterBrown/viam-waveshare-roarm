package main

import (
	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/components/gripper"
	"go.viam.com/rdk/module"
	"go.viam.com/rdk/resource"
	"waveshareroarm"
)

func main() {
	module.ModularMain(
		resource.APIModel{API: arm.API, Model: waveshareroarm.RoArmM3},
		resource.APIModel{API: gripper.API, Model: waveshareroarm.RoArmM3Gripper},
	)
}
