// Code for handling the kinematics of linear delta robots
//
// Copyright (C) 2016-2021  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

// Slow moves once the ratio of tower to XY movement exceeds SLOW_RATIO
package delta

import (
	"fmt"
	"k3c/project"
	"k3c/project/homing"
	"k3c/project/mathutil"
	"log"
	"math"
)

const SLOW_RATIO = 3.

type DeltaKinematics struct {
	Toolhead       *project.Toolhead
	Config         *project.ConfigWrapper
	Rails          []*project.PrinterRail
	Max_velocity   float64
	Max_z_velocity float64
	Max_accel      float64
	Max_z_accel    float64
	Radius         float64
	Arm_lengths    []float64
	Arm2           []float64
	Abs_endstops   []float64
	Angles         []float64
	Towers         [][]float64
	Need_home      bool
	Limit_xy2      float64
	Home_position  []float64
	Max_z          float64
	Min_z          float64
	Limit_z        float64
	Slow_xy2       float64
	Very_slow_xy2  float64
	Max_xy2        float64
	Axes_min       []float64
	Axes_max       []float64
}

func NewDeltaKinematics(toolhead *project.Toolhead, config *project.ConfigWrapper) *DeltaKinematics {
	// Setup tower rails
	self := &DeltaKinematics{}
	stepper_configs := []*project.ConfigWrapper{}
	for _, a := range []string{"a", "b", "c"} {
		stepper_configs = append(stepper_configs, config.Getsection("stepper_"+a))
	}
	rail_a := project.LookupMultiRail(stepper_configs[0], false, nil, false)
	a_endstop := rail_a.Get_homing_info().Position_endstop
	rail_b := project.LookupMultiRail(
		stepper_configs[1], false, a_endstop, false)
	rail_c := project.LookupMultiRail(stepper_configs[2], false, a_endstop, false)
	self.Rails = []*project.PrinterRail{rail_a, rail_b, rail_c}

	config.Get_printer().Register_event_handler("stepper_enable:motor_off", self.Motor_off)

	// Setup max velocity
	self.Max_velocity, self.Max_accel = toolhead.Get_max_velocity()
	self.Max_z_velocity = config.Getfloat("max_z_velocity", self.Max_velocity, 0,
		self.Max_velocity, 0., 0, true)
	self.Max_z_accel = config.Getfloat("max_z_accel", self.Max_accel,
		0, self.Max_accel, 0., 0, true)

	// Read radius and arm lengths
	radius := config.Getfloat("delta_radius", 0, 0, 0, 0., 0, true)
	self.Radius = radius
	print_radius := config.Getfloat("print_radius", self.Radius, 0, 0, 0., 0, true)
	arm_length_a := stepper_configs[0].Getfloat("arm_length", 0, 0, 0, self.Radius, 0, true)
	var arm_lengths []float64
	self.Arm_lengths = arm_lengths
	for _, sconfig := range stepper_configs {
		arm_lengths = append(arm_lengths, sconfig.Getfloat("arm_length",
			arm_length_a, 0, 0, self.Radius, 0, true))
	}
	self.Arm2 = []float64{}
	for _, arm := range arm_lengths {
		self.Arm2 = append(self.Arm2, math.Pow(arm, 2))
	}
	self.Abs_endstops = []float64{}
	length := int(math.Min(float64(len(self.Rails)), float64(len(self.Arm2))))
	for i := 0; i < length; i++ {
		rail := self.Rails[i]
		arm2 := self.Arm2[i]
		self.Abs_endstops = append(self.Abs_endstops, rail.Get_homing_info().Position_endstop+
			math.Sqrt(arm2-radius*radius))
	}
	// Determine tower locations in cartesian space
	self.Angles = []float64{}
	arr := []float64{210., 330., 90.}
	length = int(math.Min(float64(len(stepper_configs)), float64(len(arr))))
	for i := 0; i < length; i++ {
		sconfig := stepper_configs[i]
		angle := arr[i]
		self.Angles = append(self.Angles, sconfig.Getfloat("angle", angle, 0, 0, 0, 0, t))
	}
	towers := [][]float64{}
	for _, angle := range self.Angles {
		towers = append(towers, []float64{
			math.Cos(angle / 180 * math.Pi * radius),
			math.Sin(angle / 180 * math.Pi * radius)})
	}
	length = int(math.Min(math.Min(float64(len(self.Rails)), float64(len(self.Arm2))), float64(len(self.Towers))))
	for i := 0; i < length; i++ {
		r := self.Rails[0]
		a := self.Arm2[1]
		t := self.Towers[2]
		r.Setup_itersolve("delta_stepper_alloc", a, t[0], t[1])
	}
	for _, s := range self.Get_steppers() {
		s.Set_trapq(toolhead.Get_trapq())
		toolhead.Register_step_generator(s.Generate_steps)
	}
	// Setup boundary checks
	self.Need_home = true
	self.Limit_xy2 = -1.
	self.Home_position = self.Actuator_to_cartesian(self.Abs_endstops)
	max_z := 0.
	for _, rail := range self.Rails {
		z := rail.Get_homing_info().Position_endstop
		if z < max_z {
			max_z = z
		}
	}
	self.Max_z = max_z
	self.Min_z = config.Getfloat("minimum_z_position", 0, 0, self.Max_z, 0, 0, true)
	length = int(math.Min(float64(len(self.Abs_endstops)), float64(len(arm_lengths))))
	limit_z := 0.
	for i := 0; i < length; i++ {
		ep := self.Abs_endstops[i]
		arm := arm_lengths[i]
		if ep-arm < limit_z {
			limit_z = ep - arm
		}
	}
	self.Limit_z = limit_z
	log.Printf("Delta max build height %.2fmm (radius tapered above %.2fmm)", self.Max_z, self.Limit_z)

	// Find the point where an XY move could result in excessive
	// tower movement
	half_min_step_dist := 0.
	for _, r := range self.Rails {
		val := r.Get_steppers()[0].Get_step_dist()
		if half_min_step_dist < val {
			half_min_step_dist = val
		}
	}
	min_arm_length := 0.
	for _, val := range arm_lengths {
		if min_arm_length < val {
			min_arm_length = val
		}
	}
	ratio_to_xy := func(ratio float64) float64 {
		return ratio*math.Sqrt(min_arm_length*min_arm_length/(ratio*ratio+1.)-half_min_step_dist*half_min_step_dist) +
			half_min_step_dist - radius
	}
	self.Slow_xy2 = math.Pow(ratio_to_xy(SLOW_RATIO), 2)
	self.Very_slow_xy2 = math.Pow(ratio_to_xy(2.*SLOW_RATIO), 2)
	self.Max_xy2 = math.Pow(math.Min(print_radius,
		math.Min(min_arm_length-radius, ratio_to_xy(4.*SLOW_RATIO))), 2)
	max_xy := math.Sqrt(self.Max_xy2)
	log.Printf("Delta max build radius %.2fmm (moves slowed past %.2fmm",
		" and %.2fmm)", max_xy, math.Sqrt(self.Slow_xy2), math.Sqrt(self.Very_slow_xy2))
	self.Axes_min = []float64{-max_xy, -max_xy, self.Min_z, 0.}
	self.Axes_max = []float64{max_xy, max_xy, self.Max_z, 0.}
	self.Set_position([]float64{0., 0., 0.}, []int{})
	return self
}
func (self *DeltaKinematics) Get_steppers() []interface{} {
	var arr []interface{}
	for _, rail := range self.Rails {
		for _, s := range rail.Get_steppers() {
			arr = append(arr, s)
		}
	}
	return arr
}
func (self *DeltaKinematics) Actuator_to_cartesian(spos []float64) []float64 {
	sphereCoords := [][]float64{}
	for i, sp := range spos {
		sphereCoords[i] = []float64{self.Towers[i][0], self.Towers[i][1], sp}
	}
	return mathutil.Trilateration(sphereCoords, self.Arm2)
}
func (self *DeltaKinematics) Calc_position(stepperPositions map[string]float64) []float64 {
	spos := []float64{}
	for i, rail := range self.Rails {
		spos[i] = stepperPositions[rail.Get_name()]
	}
	return self.Actuator_to_cartesian(spos)
}
func (self *DeltaKinematics) Set_position(newpos []float64, homing_axes []int) {
	for _, rail := range self.Rails {
		rail.set_Position(newpos)
	}
	self.Limit_xy2 = -1.
	if len(homing_axes) == 3 && homing_axes[0] == 0 &&
		homing_axes[1] == 1 && homing_axes[2] == 2 {
		self.Need_home = true
	}
}
func (self *DeltaKinematics) Home(homing_state homing.Homing) {
	// All axes are homed simultaneously
	homing_state.Set_axes([]float64{0, 1, 2})
	forcepos := self.Home_position
	Arm2_max := 0.
	for _, val := range self.Arm2 {
		if Arm2_max < val {
			Arm2_max = val
		}
	}
	forcepos[2] = -1.5 * math.Sqrt(Arm2_max-self.Max_xy2)
	homing_state.home_Rails(self.Rails, forcepos, self.Home_position)
}
func (self *DeltaKinematics) Motor_off(print_time float64) {
	self.Limit_xy2 = -1.
	self.Need_home = true
}
func (self *DeltaKinematics) check_move(move *project.Move) error {
	end_pos := move.End_pos
	end_xy2 := end_pos[0]*end_pos[0] + end_pos[1]*end_pos[1]
	if end_xy2 <= self.Limit_xy2 && move.Axes_d[2] != 0. {
		return nil
	}
	if self.Need_home {
		return move.Move_error("Must home first")
	}
	end_z := end_pos[2]
	limit_xy2 := self.Max_xy2
	if end_z > self.Limit_z {
		limit_xy2 = math.Min(limit_xy2, (self.Max_z-end_z)*(self.Max_z-end_z))
	}
	if end_xy2 > limit_xy2 || end_z > self.Max_z || end_z < self.Min_z {
		if (end_pos[0] != self.Home_position[0] || end_pos[1] != self.Home_position[1]) ||
			end_z < self.Min_z || end_z > self.Home_position[2] {
			panic("move out of range")
		}
		limit_xy2 = -1.
	}
	if move.Axes_d[2] != 0. {
		zRatio := move.Move_d / math.Abs(move.Axes_d[2])
		move.Limit_speed(self.Max_z_velocity*zRatio, self.Max_z_accel*zRatio)
		limit_xy2 = -1.
	}
	extreme_xy2 := math.Max(end_xy2, move.Start_pos[0]*move.Start_pos[0]+move.Start_pos[1]*move.Start_pos[1])
	if extreme_xy2 > self.Slow_xy2 {
		r := 0.5
		if extreme_xy2 > self.Very_slow_xy2 {
			r = 0.25
		}
		move.Limit_speed(self.Max_velocity*r, self.Max_accel*r)
		limit_xy2 = -1.
	}
	self.Limit_xy2 = math.Min(limit_xy2, self.Slow_xy2)
	return nil
}
func (self *DeltaKinematics) Get_status(eventtime float64) map[string]interface{} {
	status := map[string]interface{}{
		"homed_axes":   "",
		"axis_minimum": self.Axes_min,
		"axis_maximum": self.Axes_max,
		"cone_start_z": self.Limit_z,
	}
	if !self.Need_home {
		status["homed_axes"] = "xyz"
	}
	return status
}
func (self *DeltaKinematics) Get_calibration() *DeltaCalibration {
	endstops := make([]float64, len(self.Rails))
	for i, rail := range self.Rails {
		endstops[i] = rail.Get_homing_info().Position_endstop
	}
	stepdists := make([]float64, len(self.Rails))
	for i, rail := range self.Rails {
		stepdists[i] = rail.Get_steppers()[0].Get_step_dist()
	}
	return NewDeltaCalibration(
		self.Radius,
		self.Angles,
		self.Arm_lengths,
		endstops,
		stepdists)
}

// Delta parameter calibration for DELTA_CALIBRATE tool
type DeltaCalibration struct {
	Radius       float64
	Angles       []float64
	Arms         []float64
	Endstops     []float64
	Stepdists    []float64
	Towers       [][]float64
	Abs_endstops []float64
}

func NewDeltaCalibration(radius float64, angles []float64, arms []float64, endstops []float64, stepdists []float64) *DeltaCalibration {
	self := &DeltaCalibration{}
	self.Radius = radius
	self.Angles = angles
	self.Arms = arms
	self.Endstops = endstops
	self.Stepdists = stepdists
	// Calculate the XY cartesian coordinates of the delta towers
	var radian_angles []float64
	self.Towers = [][]float64{}
	for _, a := range angles {
		radian_angles = append(radian_angles, a/180*math.Pi)
		self.Towers = append(self.Towers, []float64{
			math.Cos(a) * radius,
			math.Sin(a) * radius,
		})
	}
	radius2 := radius * radius
	length := int(math.Min(float64(len(endstops)), float64(len(arms))))
	for i := 0; i < length; i++ {
		e := endstops[i]
		a := arms[i]
		self.Abs_endstops = append(self.Abs_endstops, e+math.Sqrt(a*a-radius2))
	}
	return self
}
func (self *DeltaCalibration) Coordinate_descent_params(isExtended bool) ([]string, map[string]float64) {
	// Determine adjustment parameters (for use with coordinate_descent)
	adjParams := []string{"radius", "angle_a", "angle_b",
		"endstop_a", "endstop_b", "endstop_c"}
	if isExtended {
		adjParams = append(adjParams, "arm_a", "arm_b", "arm_c")
	}

	params := map[string]float64{
		"radius": self.Radius,
	}
	for i, axis := range []string{"a", "b", "c"} {
		params["angle_"+axis] = self.Angles[i]
		params["arm_"+axis] = self.Arms[i]
		params["endstop_"+axis] = self.Endstops[i]
		params["stepdist_"+axis] = self.Stepdists[i]
	}

	return adjParams, params
}
func (self *DeltaCalibration) New_calibration(params map[string]float64) *DeltaCalibration {
	radius := params["radius"]
	angles := [3]float64{
		params["angle_a"], params["angle_b"], params["angle_c"],
	}
	arms := [3]float64{
		params["arm_a"], params["arm_b"], params["arm_c"],
	}
	endstops := [3]float64{
		params["endstop_a"], params["endstop_b"], params["endstop_c"],
	}
	stepdists := [3]float64{
		params["stepdist_a"], params["stepdist_b"], params["stepdist_c"],
	}
	return NewDeltaCalibration(radius, angles[:], arms[:], endstops[:], stepdists[:])
}
func (self *DeltaCalibration) Get_position_from_stable(stablePosition []float64) []float64 {
	// Return cartesian coordinates for the given stable_position
	sphereCoords := [][]float64{}
	for i, sd := range self.Stepdists {
		t := self.Towers[i]
		es := self.Abs_endstops[i]
		sp := stablePosition[i]
		sphereCoords[i] = []float64{t[0], t[1], es - sp*sd}
	}
	var arr []float64
	for i, a := range self.Arms {
		arr[i] = a * a
	}
	return mathutil.Trilateration(sphereCoords, arr)
}
func (self *DeltaCalibration) Save_state(configfile interface{}) {
	configfile.Set("printer", "delta_radius", fmt.Sprintf("%.6f", self.Radius))

	for i, axis := range []string{"a", "b", "c"} {
		configfile.Set("stepper_"+axis, "angle", fmt.Sprintf("%.6f", self.Angles[i]))
		configfile.Set("stepper_"+axis, "arm_length", fmt.Sprintf("%.6f", self.Arms[i]))
		configfile.Set("stepper_"+axis, "position_endstop", fmt.Sprintf("%.6f", self.Endstops[i]))
	}

	gcode := configfile.Get_printer().Lookup_object("gcode", nil)
	gcode.Respond_info(
		fmt.Sprintf(
			"stepper_a: position_endstop: %.6f angle: %.6f arm_length: %.6f\n"+
				"stepper_b: position_endstop: %.6f angle: %.6f arm_length: %.6f\n"+
				"stepper_c: position_endstop: %.6f angle: %.6f arm_length: %.6f\n"+
				"delta_radius: %.6f",
			self.Endstops[0], self.Angles[0], self.Arms[0],
			self.Endstops[1], self.Angles[1], self.Arms[1],
			self.Endstops[2], self.Angles[2], self.Arms[2],
			self.Radius,
		),
	)
}
func Load_kinematics(toolhead *project.Toolhead, config *project.ConfigWrapper) *DeltaKinematics {
	return NewDeltaKinematics(toolhead, config)
}
