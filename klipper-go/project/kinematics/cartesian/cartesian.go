/*
Code for handling the kinematics of cartesian robots

Copyright (C) 2016-2021  Kevin O"Connor <kevin@koconnor.net>

This file may be distributed under the terms of the GNU GPLv3 license.
*/
package cartesian

import (
	"k3c/common/value"
	"k3c/project"
	"k3c/project/homing"
	"math"
	"sort"
	"strings"
)

type CartKinematics struct {
	Printer             *project.Printer
	Dual_carriage_axis  *int
	Dual_carriage_rails []interface{}
	Rails               []interface{}
	Max_z_velocity      float64
	Max_z_accel         float64
	Limits              [][]float64
	Axes_min            []float64
	Axes_max            []float64
}

func NewCartKinematics(toolhead interface{}, config project.ConfigWrapper) *CartKinematics {
	self := &CartKinematics{}
	self.Printer = config.Get_printer()
	// Setup axis rails
	self.Dual_carriage_axis = nil
	self.Dual_carriage_rails = []interface{}{}
	rails := []interface{}{}
	str_arr := []string{"x", "y", "z"}
	for _, n := range str_arr {
		rails = append(rails, stepper.LookupMultiRail(config.Getsection("stepper_"+n)))
	}
	self.Rails = rails
	length := len(self.Rails)
	if length < len(str_arr) {
		length = len(str_arr)
	}
	for i := 0; i < length; i++ {
		rail := self.Rails[i]
		axis := str_arr[i]
		rail.Setup_itersolve("cartesian_stepper_alloc", []byte(axis))
	}
	for _, s := range self.Get_steppers() {
		s.Set_trapq(toolhead.Get_trapq())
		toolhead.Register_step_generator(s.Generate_steps)
	}
	self.Printer.Register_event_handler("stepper_enable:motor_off",
		self.Motor_off)
	// Setup boundary checks
	max_velocity, max_accel := toolhead.Get_max_velocity()
	self.Max_z_velocity = config.Getfloat("max_z_velocity", max_velocity, nil, max_velocity, 0., nil, true)
	self.Max_z_accel = config.Getfloat("max_z_accel", max_accel, nil, max_accel, 0., nil, true)
	self.Limits = [][]float64{{1.0, -1.0}, {1.0, -1.0}, {1.0, -1.0}}
	var ranges [][]float64
	for _, r := range self.Rails {
		position_min, position_max := r.Get_range()
		ranges = append(ranges, []float64{position_min, position_max})
	}
	var ranges_r0_arr []float64
	var ranges_r1_arr []float64
	for _, r := range ranges {
		ranges_r0_arr = append(ranges_r0_arr, r[0])
		ranges_r1_arr = append(ranges_r1_arr, r[1])
	}
	self.Axes_min = toolhead.Coord(ranges_r0_arr[0], ranges_r0_arr[1], ranges_r0_arr[2], 0)
	self.Axes_max = toolhead.Coord(ranges_r1_arr[0], ranges_r1_arr[1], ranges_r1_arr[2], 0)
	// Check for dual carriage support
	if config.Has_section("dual_carriage") {
		dc_config := config.Getsection("dual_carriage")
		dc_axis := dc_config.Getchoice("axis", map[string]string{"x": "x", "y": "y"})
		dual_carriage_axis := map[string]int{"x": 0, "y": 1}[dc_axis]
		self.Dual_carriage_axis = &dual_carriage_axis
		dc_rail := stepper.LookupMultiRail(dc_config)
		dc_rail.Setup_itersolve("cartesian_stepper_alloc", []byte(dc_axis))
		for _, s := range dc_rail.Get_steppers() {
			toolhead.Register_step_generator(s.Generate_steps)
		}
		self.Dual_carriage_rails = []interface{}{self.Rails[*self.Dual_carriage_axis], dc_rail}
		gocode, err := self.Printer.Lookup_object("gcode", nil)
		if err != nil {
			value.StaticValue.Error.Println(err)
		}
		gocode.(project.GCodeDispatch).Register_command("SET_DUAL_CARRIAGE", self.Cmd_SET_DUAL_CARRIAGE, false, cmd_SET_DUAL_CARRIAGE_help)
	}
	return self
}
func (self *CartKinematics) Get_steppers() []interface{} {
	rails := self.Rails
	var steppers []interface{}
	if self.Dual_carriage_axis != nil {
		dca := *self.Dual_carriage_axis
		rails = append(rails[:dca], append(self.Dual_carriage_rails, rails[dca+1:]))
		for _, rail := range rails {
			for _, s := range rail.Get_steppers() {
				steppers = append(steppers, s)
			}
		}
	}
	return steppers
}
func (self *CartKinematics) calc_position(stepper_positions map[string]float64) []float64 {
	var position_arr []float64
	for _, rail := range self.Rails {
		position_arr = append(position_arr, stepper_positions[rail.Get_name()])
	}
	return position_arr
}
func (self *CartKinematics) Set_position(newpos []float64, homing_axes []int) {
	for i, rail := range self.Rails {
		rail.Set_position(newpos)
		if sort.SearchInts(homing_axes, i) != len(homing_axes) {
			self.Limits[i] = rail.Get_range()
		}
	}
}
func (self *CartKinematics) Note_z_not_homed() {
	// Helper for Safe Z Home
	self.Limits[2] = []float64{1.0, -1.0}
}
func (self *CartKinematics) Home_axis(homing_state homing.Homing, axis int, rail interface{}) {
	// Determine movement
	position_min, position_max := rail.Get_range()
	hi := rail.Get_homing_info()
	homepos := []float64{}
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
func (self *CartKinematics) Home(homing_state homing.Homing) {
	// Each axis is homed independently and in order
	for _, axis := range homing_state.Get_axes() {
		if axis == *self.Dual_carriage_axis {
			dc1 := self.Dual_carriage_rails[0]
			dc2 := self.Dual_carriage_rails[1]
			altc := self.Rails[axis] == dc2
			self.Activate_carriage(0)
			self.Home_axis(homing_state, axis, dc1)
			self.Activate_carriage(1)
			self.Home_axis(homing_state, axis, dc2)
			self.Activate_carriage(altc)
		} else {
			self.Home_axis(homing_state, axis, self.Rails[axis])
		}
	}
}
func (self *CartKinematics) Motor_off(print_time float64) {
	self.Limits = [][]float64{{1.0, -1.0}, {1.0, -1.0}, {1.0, -1.0}}
}
func (self *CartKinematics) Check_endstops(move interface{}) error {
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

func (self *CartKinematics) check_move(move interface{}) {
	limits := self.Limits
	xpos := move.End_pos[0]
	ypos := move.End_pos[1]
	if xpos < limits[0][0] || xpos > limits[0][1] ||
		ypos < limits[1][0] || ypos > limits[1][1] {
		err := self.Check_endstops(move)
		if err != nil {
			value.StaticValue.Error.Println(err)
		}
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
func (self *CartKinematics) Get_status(eventtime float64) map[string]interface{} {
	str_arr := []string{"x", "y", "z"}
	var axes []string
	for i, str := range str_arr {
		l := self.Limits[i][0]
		h := self.Limits[i][1]
		if l <= h {
			axes = append(axes, str)
		}
	}
	return map[string]interface{}{
		"homed_axes":   strings.Join(axes, ""),
		"axis_minimum": self.Axes_min,
		"axis_maximum": self.Axes_max,
	}
}

// Dual carriage support
func (self *CartKinematics) Activate_carriage(carriage int) {
	toolhead, err := self.Printer.Lookup_object("toolhead", nil)
	if err != nil {
		value.StaticValue.Error.Println(err)
	}
	toolhead.Flush_step_generation()
	dc_rail := self.Dual_carriage_rails[carriage]
	dc_axis := self.Dual_carriage_axis
	self.Rails[*dc_axis].Set_trapq(nil)
	dc_rail.Set_trapq(toolhead.Get_trapq())
	self.Rails[*dc_axis] = dc_rail
	pos := toolhead.Get_position()
	pos[dc_axis] = dc_rail.Get_commanded_position()
	toolhead.Set_position(pos)
	if self.Limits[*dc_axis][0] <= self.Limits[*dc_axis][1] {
		self.Limits[*dc_axis] = dc_rail.Get_range()
	}
}

const cmd_SET_DUAL_CARRIAGE_help = "Set which carriage is active"

func (self *CartKinematics) Cmd_SET_DUAL_CARRIAGE(gcmd project.GCodeCommand) {
	minval := 0.
	maxval := 1.
	carriage := gcmd.Get_int("CARRIAGE", nil, &minval, &maxval)
	self.Activate_carriage(carriage)
}
func Load_kinematics(toolhead interface{}, config project.ConfigWrapper) *CartKinematics {
	return NewCartKinematics(toolhead, config)
}
