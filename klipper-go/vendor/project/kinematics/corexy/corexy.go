/*
 Code for handling the kinematics of corexy robots

 Copyright (C) 2017-2021  Kevin O"Connor <kevin@koconnor.net>

 This file may be distributed under the terms of the GNU GPLv3 license.
*/
package corexy

import (
	"k3c/project"
	"math"
	"sort"
	"strings"
)

type CoreXYKinematics struct {
	Rails          []*project.PrinterRail
	Max_z_velocity float64
	Max_z_accel    float64
	Limits         [][]float64
	Axes_min       []float64
	Axes_max       []float64
}

//初始化
func NewCoreXYKinematics(toolhead interface{}, config project.ConfigWrapper) *CoreXYKinematics {
	self := &CoreXYKinematics{}
	// Setup axis rails
	rails := []*project.PrinterRail{}
	for _, n := range []string{"x", "y", "z"} {
		rails = append(rails,
			project.LookupMultiRail(config.Getsection("stepper_"+n), false, nil, false))
	}
	self.Rails = rails
	for s := range self.Rails[1].Get_steppers() {
		self.Rails[0].Get_endstops()[0].Front().Value.(*project.MCU_endstop).Add_stepper(s)
	}
	for s := range self.Rails[0].Get_steppers() {
		self.Rails[1].Get_endstops()[0].Front().Value.(*project.MCU_endstop).Add_stepper(s)
	}
	self.Rails[0].Setup_itersolve("corexy_stepper_alloc", "+")
	self.Rails[1].Setup_itersolve("corexy_stepper_alloc", "-")
	self.Rails[2].Setup_itersolve("cartesian_stepper_alloc", "z")
	for s := range self.Get_steppers() {
		s.Set_trapq(toolhead.Get_trapq())
		toolhead.Register_step_generator(s.generate_steps)
	}
	config.Get_printer().Register_event_handler("stepper_enable:motor_off", self.MotorOff)

	// Setup boundary checks
	maxVelocity, maxAccel := toolhead.Get_max_velocity()
	self.Max_z_velocity = config.Getfloat("max_z_velocity", maxVelocity, nil, maxVelocity, 0, nil, true)
	self.Max_z_accel = config.Getfloat("max_z_accel", maxAccel, nil, maxAccel, 0, nil, true)
	self.Limits = [][]float64{{1.0, -1.0}, {1.0, -1.0}, {1.0, -1.0}}
	ranges := [][]float64{}
	for _, r := range self.Rails {
		position_min, position_max := r.Get_range()
		ranges = append(ranges, []float64{position_min, position_max})
	}
	self.Axes_min = toolhead.Coord(ranges[0][0], ranges[1][0], ranges[2][0], 0)
	self.Axes_max = toolhead.Coord(ranges[0][1], ranges[1][1], ranges[2][1], 0)
	return self
}

func (self *CoreXYKinematics) Get_steppers() []interface{} {
	return []interface{}{self.Rails[0].Get_steppers(), self.Rails[1].Get_steppers(), self.Rails[2].Get_steppers()}
}
func (self *CoreXYKinematics) Calc_position(stepper_positions map[string]float64) []float64 {
	pos := []float64{}
	for _, rail := range self.Rails {
		pos = append(pos, stepper_positions[rail.Get_name()])
	}
	return []float64{0.5 * (pos[0] + pos[1]), 0.5 * (pos[0] - pos[1]), pos[2]}
}
func (self *CoreXYKinematics) Set_position(newpos []float64, homing_axes []int) {
	for i, rail := range self.Rails {
		rail.Set_position(newpos)
		if sort.SearchInts(homing_axes, i) != len(homing_axes) {
			self.Limits[i] = rail.Get_range()
		}
	}
}
func (self *CoreXYKinematics) Note_z_not_homed() {
	// Helper for Safe Z Home
	self.Limits[2] = []float64{1.0, -1.0}
}
func (self *CoreXYKinematics) Home(homing_state homing.Homing) {
	// Each axis is homed independently and in order
	for _, axis := range homing_state.Get_axes() {
		rail := self.Rails[axis]
		// Determine movement
		position_min, position_max := rail.Get_range()
		hi := rail.Get_homing_info()
		var homepos []float64
		homepos[axis] = hi.Position_endstop
		forcepos := make([]float64, len(homepos))
		copy(forcepos, homepos)
		if hi.Positive_dir {
			forcepos[axis] -= 1.5 * (hi.Position_endstop - position_min)
		} else {
			forcepos[axis] += 1.5 * (position_max - hi.Position_endstop)
		}
		// Perform homing
		homing_state.Home_rails([]interface{}{rail}, forcepos, homepos)
	}
}
func (self *CoreXYKinematics) Motor_off(print_time float64) {
	self.Limits = [][]float64{{1.0, -1.0}, {1.0, -1.0}, {1.0, -1.0}}
}
func (self *CoreXYKinematics) Check_endstops(move interface{}) error {
	end_pos := move.End_pos
	for i := 0; i < 3; i++ {
		if move.Axes_d[i] != nil &&
			(end_pos[i] < self.Limits[i][0] ||
				end_pos[i] > self.Limits[i][1]) {
			if self.Limits[i][0] > self.Limits[i][1] {
				return move.Move_error("Must home axis first")
			}
			return move.Move_error()
		}
	}
	return nil
}
func (self *CoreXYKinematics) Check_move(move interface{}) {
	limits := self.Limits
	xpos := move.End_pos[0]
	ypos := move.End_pos[1]
	if xpos < limits[0][0] || xpos > limits[0][1] ||
		ypos < limits[1][0] || ypos > limits[1][1] {
		self.Check_endstops(move)
	}
	if move.Axes_d[2] == nil {
		// Normal XY move - use defaults
		return
	}
	// Move with Z - update velocity and accel for slower Z axis
	self.Check_endstops(move)
	z_ratio := move.Move_d / math.Abs(move.Axes_d[2])
	move.Limit_speed(self.Max_z_velocity*z_ratio, self.Max_z_accel*z_ratio)
}
func (self *CoreXYKinematics) Get_status(eventtime float64) map[string]interface{} {
	var axes []string
	str_arr := []string{"x", "y", "z"}
	for i := 0; i < 3; i++ {
		l := self.Limits[i][0]
		h := self.Limits[i][1]
		if l <= h {
			axes = append(axes, str_arr[i])
		}
	}
	return map[string]interface{}{
		"homed_axes":   strings.Join(axes, ""),
		"axis_minimum": self.Axes_min,
		"axis_maximum": self.Axes_max,
	}
}
func Load_kinematics(toolhead interface{}, config project.ConfigWrapper) *CoreXYKinematics {
	return NewCoreXYKinematics(toolhead, config)
}
