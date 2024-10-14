/*
# Code for handling the kinematics of deltesian robots
#
# Copyright (C) 2022  Fabrice Gallet <tircown@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

*/
package deltesian

import (
	"k3c/project"
	"math"
)

// Slow moves once the ratio of tower to XY movement exceeds SLOW_RATIO
const SLOW_RATIO = 3.

// Minimum angle with the horizontal for the arm to not exceed - in degrees
const MIN_ANGLE = 5.

type DeltesianKinematics struct {
	Rails                   []*project.PrinterRail
	Arm_x                   []float64
	Arm2                    []float64
	Limits                  [][]float64
	SlowX2, verySlowX2      float64
	MaxZVelocity, maxZAccel float64
	AxesMin, axesMax        [][]float64
	HomedAxis               []bool
	Abs_endstop             []float64
	Home_z                  []float64
	Slow_x2                 *float64
	Very_slow_x2            *float64
}

func NewDeltesianKinematics(toolhead interface{}, config project.ConfigWrapper) *DeltesianKinematics {
	self := &DeltesianKinematics{}
	// self.rails = [None] * 3
	var stepper_configs []*project.ConfigWrapper
	for _, s := range []string{"left", "right", "y"} {
		stepper_configs = append(stepper_configs, config.Getsection("stepper_"+s))
	}
	self.Rails[0] = project.NewPrinterRail(stepper_configs[0], false, nil, false)
	def_pos_es := self.Rails[0].Get_homing_info().Position_endstop
	self.Rails[1] = project.NewPrinterRail(stepper_configs[1], false, def_pos_es, false)
	self.Rails[2] = project.LookupMultiRail(stepper_configs[2], true, nil, false)
	var arm_x []float64
	self.Arm_x = arm_x
	arm_x[0] = stepper_configs[0].Getfloat("arm_x_length", nil, 0, 0, 0., 0, true)
	arm_x[1] = stepper_configs[1].Getfloat("arm_x_length", arm_x[0], 0, 0, 0., 0, true)
	arm2 := []float64{math.Pow(arm_x[0], 2), math.Pow(arm_x[1], 2)}

	var arm []float64
	arm[0] = stepper_configs[0].Getfloat("arm_length", nil, 0, 0, arm_x[0], 0, true)
	arm[1] = stepper_configs[1].Getfloat("arm_length", arm[0], 0, 0, arm_x[1], 0, true)

	self.Arm2 = arm2
	self.Rails[0].Setup_itersolve("deltesian_stepper_alloc", arm2[0], -arm_x[0])
	self.Rails[1].Setup_itersolve("deltesian_stepper_alloc", arm2[1], arm_x[1])
	self.Rails[2].Setup_itersolve("cartesian_stepper_alloc", "y")
	for _, s := range self.Get_steppers() {
		s.Set_trapq(toolhead.Get_trapq())
		toolhead.Register_step_generator(s.Generate_steps)
	}
	config.Get_printer().Register_event_handler("stepper_enable:motor_off", self.Motor_off)
	self.Limits = [][]float64{{1.0, -1.0}, {1.0, -1.0}, {1.0, -1.0}}
	// X axis limits
	min_angle := config.Getfloat("min_angle", MIN_ANGLE, 0., 90., nil, nil, true)
	cos_angle := math.Cos(min_angle / 180 * math.Pi)
	x_kin_min := math.Ceil(-math.Min(arm_x[0], cos_angle*arm[1]-arm_x[1]))
	x_kin_max := math.Floor(math.Min(arm_x[1], cos_angle*arm[0]-arm_x[0]))
	x_kin_range := math.Min(x_kin_max-x_kin_min, math.Min(x_kin_max*2, -x_kin_min*2))
	print_width := config.Getfloat("print_width", nil, 0., x_kin_range, nil, nil, true)
	if print_width != nil {
		self.Limits[0] = []float64{-print_width * 0.5, print_width * 0.5}
	} else {
		self.Limits[0] = []float64{x_kin_min, x_kin_max}
	}
	// Y axis limits
	self.Limits[1] = self.Rails[2].Get_range()
	// Z axis limits
	var pmax []float64
	for _, r := range self.Rails[:2] {
		pmax = append(pmax, r.Get_homing_info().Position_endstop)
	}
	length := int(math.Min(float64(len(pmax)), math.Min(float64(len(arm2)), float64(len(arm_x)))))
	self.Abs_endstop = []float64{}
	for i := 0; i < length; i++ {
		p := pmax[i]
		a2 := pmax[i]
		ax := pmax[i]
		self.Abs_endstop = append(self.Abs_endstop, p+math.Sqrt(a2-math.Pow(ax, 2)))
	}

	self.Home_z = self.Actuator_to_cartesian(self.Abs_endstop)[1]
	z_max := math.Min(self.Pillars_z_max(self.Limits[0][0]),
		math.Min(self.Pillars_z_max(self.Limits[0][1]), self.Pillars_z_max(self.Limits[0][2])))
	z_min := config.Getfloat("minimum_z_position", 0, z_max)
	self.Limits[2] = []float64{z_min, z_max}
	// Limit the speed/accel if is is at the extreme values of the x axis
	slow_ratio := config.Getfloat("slow_ratio", SLOW_RATIO, 0., nil, nil, true)
	self.Slow_x2, self.Very_slow_x2 = nil, nil
	if slow_ratio > 0 {
		sr2 := math.Pow(slow_ratio, 2)
	}
	return self
}
