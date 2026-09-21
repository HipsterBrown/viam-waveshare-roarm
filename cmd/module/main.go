package main

import (
	rdkarm "go.viam.com/rdk/components/arm"
	rdkgripper "go.viam.com/rdk/components/gripper"
	"go.viam.com/rdk/module"
	"go.viam.com/rdk/resource"

	"waveshareroarm/components/arm"
	"waveshareroarm/components/gripper"
)

func main() {
	module.ModularMain(
		resource.APIModel{API: rdkarm.API, Model: arm.Model},
		resource.APIModel{API: rdkgripper.API, Model: gripper.Model},
	)
}
