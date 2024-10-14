// Package hybrid_corexy Code for handling the kinematics of hybrid-corexy robots
//
// Copyright (C) 2021  Fabrice Gallet <tircown@gmail.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
package hybrid_corexy

import (
	"k3c/project"
	"k3c/project/kinematics/idex_modes"
	"math"
	"sort"
	"strings"
)

// The hybrid-corexy kinematic is also known as Markforged kinematics

type HybridCoreXYKinematics struct {
	Printer        *project.Printer
	Rails          []*project.PrinterRail
	Axes_min       []float64
	Axes_max       []float64
	Dc_module      interface{}
	Max_z_velocity float64
	Max_z_accel    float64
	Limits         [][]float64
}

func NewHybridCoreXYKinematics(toolhead *project.Toolhead, config *project.ConfigWrapper) *HybridCoreXYKinematics {
	self := &HybridCoreXYKinematics{}
	self.Printer = config.Get_printer()
	// printer_config := config.Getsection("printer")
	// itersolve parameters
	self.Rails = []*project.PrinterRail{
		project.NewPrinterRail(config.Getsection("stepper_x"), true, 0, true),
		project.LookupMultiRail(config.Getsection("stepper_y"), true, nil, true),
		project.LookupMultiRail(config.Getsection("stepper_z"), true, nil, true),
	}
	self.Rails[1].Get_endstops()[0][0].Add_stepper(self.Rails[0].Get_steppers()[0])
	self.Rails[0].Setup_itersolve("corexy_stepper_alloc", '-')
	self.Rails[1].Setup_itersolve("cartesian_stepper_alloc", 'y')
	self.Rails[2].Setup_itersolve("cartesian_stepper_alloc", 'z')
	var ranges [][]float64
	for _, r := range self.Rails {
		a, b := r.Get_range()
		ranges = append(ranges, []float64{a, b})
	}
	self.Axes_min = []float64{ranges[0][0], ranges[1][0], ranges[2][0], 0}
	self.Axes_max = []float64{ranges[0][1], ranges[1][1], ranges[2][1], 0}
	if config.Has_section("dual_carriage") {
		dc_config := config.Getsection("dual_carriage")
		// dummy for cartesian config users
		dc_config.Getchoice("axis", map[interface{}]string{"x": "x"}, "x", true)
		// setup second dual carriage rail
		self.Rails = append(self.Rails, project.NewPrinterRail(dc_config, true, nil, true))
		self.Rails[1].Get_endstops()[0][0].Add_stepper(self.Rails[3].Get_steppers()[0])
		self.Rails[3].Setup_itersolve("cartesian_stepper_alloc", 'y')
		dcRail0 := idex_modes.DualCarriagesRail(self.Printer, self.Rails[0], 0, true,
			"corexy_stepper_alloc", "-",
			"cartesian_reverse_stepper_alloc", 'y',
		)
		dcRail1 := idex_modes.DualCarriagesRail(self.Printer, self.Rails[3], 0, false,
			"corexy_stepper_alloc", "+",
			"cartesian_stepper_alloc", 'y',
		)
		self.Dc_module = idex_modes.DualCarriages(self.Printer, dcRail0, dcRail1, 0)
	}
	for _, s := range self.Get_steppers() {
		s.Set_trapq(toolhead.Get_trapq())
		toolhead.Register_step_generator(s.Generate_steps)
	}
	self.Printer.Register_event_handler("stepper_enable:motor_off", self.Motor_off)
	// Setup boundary checks
	maxVelocity, maxAccel := toolhead.Get_max_velocity()
	self.Max_z_velocity = config.Getfloat("max_z_velocity", maxVelocity, 0, maxVelocity, 0, 0, true)
	self.Max_z_accel = config.Getfloat("max_z_accel", maxAccel, 0, maxAccel, 0, 0, true)
	self.Limits = [][]float64{{1.0, -1.0}, {1.0, -1.0}, {1.0, -1.0}}
	return self
}
func (self *HybridCoreXYKinematics) Get_steppers() []*project.MCU_stepper {
	var arr []*project.MCU_stepper
	for _, rail := range self.Rails {
		for _, s := range rail.Get_steppers() {
			arr = append(arr, s)
		}
	}
	return arr
}

func (self *HybridCoreXYKinematics) Calc_position(stepperPositions map[string]float64) []float64 {
	pos := []float64{}
	for _, rail := range self.Rails {
		pos = append(pos, stepperPositions[rail.Get_name(false)])
	}
	if self.Dc_module != nil && "CARRIAGE_1" == self.Dc_module.Get_status()["active_carriage"] {
		return []float64{pos[0] - pos[1], pos[1], pos[2]}
	} else {
		return []float64{pos[0] + pos[1], pos[1], pos[2]}
	}
}
func (self *HybridCoreXYKinematics) Update_limits(i int, _range []float64) {
	self.Limits[i] = _range
}
func (self *HybridCoreXYKinematics) Override_Rail(i int, rail *project.PrinterRail) {
	self.Rails[i] = rail
}

func (self *HybridCoreXYKinematics) Set_Position(newpos []float64, homingAxes []int) {
	for i, rail := range self.Rails {
		rail.Set_position(newpos)
		if sort.SearchInts(homingAxes, i) != len(homingAxes) {
			val1, val2 := rail.Get_range()
			self.Limits[i] = []float64{val1, val2}
		}
	}
}
func (self *HybridCoreXYKinematics) Note_z_not_homed() {
	// Helper for Safe Z Home
	self.Limits[2] = []float64{1.0, -1.0}
}
func (self *HybridCoreXYKinematics) Home_axis(homing_state interface{}, axis int, rail *project.PrinterRail) {
	position_min, position_max := rail.Get_range()
	hi := rail.Get_homing_info()
	homepos := []float64{0, 0, 0, 0}
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
func (self *HybridCoreXYKinematics) home(homing_state interface{}) {
	for _, axis := range homing_state.Get_axes() {
		if self.Dc_module != nil && axis == 0 {
			self.Dc_module.Save_idex_state()
			for _, i := range []int{0, 1} {
				self.Dc_module.Toggle_active_dc_rail(i)
				self.Home_axis(homing_state, axis, self.Rails[0])
			}
			self.Dc_module.Restore_idex_state()
		} else {
			self.Home_axis(homing_state, axis, self.Rails[axis])
		}
	}
}
func (self *HybridCoreXYKinematics) Motor_off(print_time float64) {
	self.Limits = [][]float64{{1.0, -1.0}, {1.0, -1.0}, {1.0, -1.0}}
}

func (self *HybridCoreXYKinematics) Check_endstops(move project.Move) error {
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
func (self *HybridCoreXYKinematics) Check_move(move project.Move) {
	limits := self.Limits
	xpos, ypos := move.End_pos[0], move.End_pos[1]
	if xpos < limits[0][0] || xpos > limits[0][1] || ypos < limits[1][0] || ypos > limits[1][1] {
		self.Check_endstops(move)
	}
	if move.Axes_d[2] == 0 {
		//  Normal XY move - use defaults
		return
	}
	// Move with Z - update velocity and accel for slower Z axis
	self.Check_endstops(move)
	zRatio := move.Move_d / math.Abs(move.Axes_d[2])
	move.Limit_speed(self.Max_z_velocity*zRatio, self.Max_z_accel*zRatio)
}
func (self *HybridCoreXYKinematics) Get_status(eventtime float64) map[string]interface{} {
	var axes []string
	str := []string{"x", "y", "z"}
	for i, lh := range self.Limits {
		a := str[i]
		if lh[0] <= lh[1] {
			axes = append(axes, a)
		}
	}
	return map[string]interface{}{
		"homed_axes":   strings.Join(axes, ""),
		"axis_minimum": self.Axes_min,
		"axis_maximum": self.Axes_max,
	}
}

func Load_kinematics(toolhead *project.Toolhead, config *project.ConfigWrapper) *HybridCoreXYKinematics {
	return NewHybridCoreXYKinematics(toolhead, config)
}
