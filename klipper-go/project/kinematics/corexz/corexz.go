/*
# Code for handling the kinematics of corexz robots
#
# Copyright (C) 2020  Maks Zolin <mzolin@vorondesign.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
*/
package corexz

import (
	"k3c/project"
	"k3c/project/homing"
	"math"
	"sort"
	"strings"
)

type CoreXZKinematics struct {
	Rails          []project.PrinterRail
	Max_z_velocity float64
	Max_z_accel    float64
	Limits         [][]float64
	Axes_min       []float64
	Axes_max       []float64
}

func NewCoreXZKinematics(toolhead interface{}, config *project.ConfigWrapper) *CoreXZKinematics {
	// Setup axis rails
	self := &CoreXZKinematics{}
	rails := []*project.PrinterRail{
		project.NewPrinterRail(config.Getsection("stepper_x"), true, nil, false),
		project.NewPrinterRail(config.Getsection("stepper_y"), true, nil, false),
		project.NewPrinterRail(config.Getsection("stepper_z"), true, nil, false),
	}
	rails[0].Get_endstops()[0][0].Add_stepper(rails[2].Get_steppers()[0])
	rails[2].Get_endstops()[0][0].Add_stepper(rails[0].Get_steppers()[0])
	rails[0].Setup_itersolve("corexz_stepper_alloc", '+')
	rails[1].Setup_itersolve("cartesian_stepper_alloc", 'y')
	rails[2].Setup_itersolve("corexz_stepper_alloc", '-')
	for _, s := range Get_steppers() {
		s.Set_trapq(toolhead.Get_trapq())
		toolhead.Register_step_generator(s.Generate_steps)
	}
	config.Get_printer().Register_event_handler("stepper_enable:motor_off", self.Motor_off)
	// Setup boundary checks
	maxVelocity, maxAccel := toolhead.Get_max_velocity()
	self.Max_z_velocity = config.Getfloat(
		"max_z_velocity", maxVelocity, 0, maxVelocity, 0., 0, true)
	self.Max_z_accel = config.Getfloat(
		"max_z_accel", maxAccel, 0, maxAccel, 0., 0, true)

	self.Limits = [][]float64{{1.0, -1.0}, {1.0, -1.0}, {1.0, -1.0}}
	var ranges [][]float64
	for _, r := range self.Rails {
		position_min, position_max := r.Get_range()
		ranges = append(ranges, []float64{position_min, position_max})
	}
	self.Axes_min = []float64{ranges[0][0], ranges[1][0], ranges[2][0], 0}
	self.Axes_max = []float64{ranges[0][1], ranges[1][1], ranges[2][1], 0}
	return self
}
func (self *CoreXZKinematics) Get_steppers() {
	steppers := []*project.MCU_stepper{}
	for _, rail := range self.Rails {
		for _, s := range rail.Get_steppers() {
			steppers = append(steppers, s)
		}
	}
}
func (self *CoreXZKinematics) Calc_position(stepper_positions map[string]float64) []float64 {
	pos := make([]float64, 3)
	for i, rail := range self.Rails {
		pos[i] = stepper_positions[rail.Get_name()]
	}
	return []float64{0.5 * (pos[0] + pos[2]), pos[1], 0.5 * (pos[0] - pos[2])}
}
func (self *CoreXZKinematics) Set_position(newpos [3]float64, homingAxes []int) {
	for i, rail := range self.Rails {
		rail.Set_position(newpos)
		if sort.SearchInts(homingAxes, i) != len(homingAxes) {
			a, b := rail.Get_range()
			self.Limits[i] = []float64{a, b}
		}
	}
}
func (self *CoreXZKinematics) note_z_not_homed() {
	// Helper for Safe Z Home
	self.Limits[2] = []float64{1.0, -1.0}
}
func (self *CoreXZKinematics) Home(homing_state homing.Homing) {
	// Each axis is homed independently and in order
	for _, axis := range homing_state.Get_axes() {
		rail := self.Rails[axis]
		// Determine movement
		position_min, position_max := rail.Get_range()
		hi := rail.Get_homing_info()
		homepos := []float64{0, 0, 0, 0}
		homepos[axis] = hi.PositionEndstop

		forcepos := make([]float64, len(homepos))
		copy(forcepos, homepos)
		if hi.PositiveDir {
			forcepos[axis] -= 1.5 * (hi.PositionEndstop - position_min)
		} else {
			forcepos[axis] += 1.5 * (position_max - hi.PositionEndstop)
		}
		// Perform homing
		homing_state.Home_rails([]interface{}{rail}, forcepos, homepos)
	}
}
func (self *CoreXZKinematics) Motor_off(print_time float64) {
	self.Limits = [][]float64{{1.0, -1.0}, {1.0, -1.0}, {1.0, -1.0}}
}

func (self *CoreXZKinematics) Check_endstops(move project.Move) error {
	endPos := move.End_pos
	for _, i := range []int{0, 1, 2} {
		if move.Axes_d[i] != 0 && (endPos[i] < self.Limits[i][0] || endPos[i] > self.Limits[i][1]) {
			if self.Limits[i][0] > self.Limits[i][1] {
				return move.Move_error("Must home axis first")
			}
			return move.Move_error("Move out of range")
		}
	}
	return nil
}
func (self *CoreXZKinematics) Check_move(move project.Move) {
	limits := self.Limits
	xpos := move.End_pos[0]
	ypos := move.End_pos[1]
	if xpos < limits[0][0] || xpos > limits[0][1] ||
		ypos < limits[1][0] || ypos > limits[1][1] {
		self.Check_endstops(move)
	}
	if move.Axes_d[2] != 0 {
		// Normal XY move - use defaults
		return
	}
	// Move with Z - update velocity and accel for slower Z axis
	self.Check_endstops(move)
	zRatio := move.Move_d / math.Abs(move.Axes_d[2])
	move.Limit_speed(
		self.Max_z_accel*zRatio,
		self.Max_z_accel*zRatio,
	)
}
func (self *CoreXZKinematics) get_status(eventtime int) map[string]interface{} {
	var axes []string
	arr := []string{"x", "y", "z"}
	for i, item := range self.Limits {
		a := arr[i]
		l := item[0]
		h := item[1]
		if l <= h {
			axes = append(axes, a)
		}
	}
	return map[string]interface{}{
		"homed_axes":   strings.Join(axes, ""),
		"axis_minimum": self.Axes_min,
		"axis_maximum": self.Axes_max,
	}
}
func Load_kinematics(toolhead *project.Toolhead, config *project.ConfigWrapper) *CoreXZKinematics {
	return NewCoreXZKinematics(toolhead, config)
}
