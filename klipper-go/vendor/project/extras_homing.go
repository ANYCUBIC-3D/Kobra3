// Helper code for implementing homing operations
//
// Copyright (C) 2016-2021  Kevin O"Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
package project

import (
	"container/list"
	"fmt"
	"k3c/common/constants"
	"k3c/common/utils/collections"
	"k3c/common/utils/object"

	//"k3c/common/value"
	"log"
	"math"
	"reflect"
	"strings"
	//"k3c/project/config"
	//"math"
	//"strings"
	//"k3c/project"
)

const (
	HOMING_START_DELAY   = 0.001
	ENDSTOP_SAMPLE_TIME  = .000015
	ENDSTOP_SAMPLE_COUNT = 1
)

// 0 未归零 1归零中 2归零完成
var is_homing = 0

// Return a completion that completes when all completions in a list complete
func Multi_complete(Printer *Printer, completions []*ReactorCompletion) *ReactorCompletion {
	if len(completions) == 1 {
		return completions[0]
	}
	// Build completion that waits for all completions
	var reactor = Printer.Get_reactor()
	var lambda = func(interface{}) interface{} {
		for _, c := range completions {
			c.Wait(constants.NEVER, nil)
		}
		return nil
	}
	var cp = reactor.Register_callback(lambda, constants.NOW)

	// If any completion indicates an error, then exit main completion early

	for _, c := range completions {
		var lambda = func(interface{}) interface{} {
			if c.Wait(constants.NEVER, nil) != nil {
				cp.Complete(1)
			} else {
				return nil
			}
			return nil
		}
		reactor.Register_callback(lambda, constants.NOW)
	}
	return cp
}

// Tracking of stepper positions during a homing/probing move
type StepperPosition struct {
	Stepper      *MCU_stepper
	Endstop_name string
	Stepper_name string
	Start_pos    int
	Halt_pos     int
	Trig_pos     int
}

func NewStepperPosition(stepper *MCU_stepper, endstop_name string) *StepperPosition {
	var self = &StepperPosition{}
	self.Stepper = stepper
	self.Endstop_name = endstop_name
	self.Stepper_name = stepper.Get_name(false)
	self.Start_pos = stepper.Get_mcu_position()
	self.Halt_pos, self.Trig_pos = 0, 0
	return self
}

func (self *StepperPosition) Note_home_end(trigger_time float64) {
	self.Halt_pos = self.Stepper.Get_mcu_position()
	self.Trig_pos = self.Stepper.Get_past_mcu_position(trigger_time)
}

// Implementation of homing/probing moves
type HomingMove struct {
	Printer           *Printer
	Endstops          []list.List
	Toolhead          *Toolhead
	Stepper_positions []*StepperPosition
}

func NewHomingMove(printer *Printer, endstops []list.List, toolhead interface{}) *HomingMove {
	var self = HomingMove{}
	self.Printer = printer
	self.Endstops = endstops
	if toolhead == nil {
		//var err error
		toolhead = printer.Lookup_object("toolhead", object.Sentinel{})
		//if err != nil {
		//	log.Print(err.Error())
		//}
	}
	self.Toolhead = toolhead.(*Toolhead)
	self.Stepper_positions = []*StepperPosition{}
	return &self
}

func (self *HomingMove) Get_mcu_endstops() []interface{} {
	_es := []interface{}{}
	for _, es := range self.Endstops {
		_es = append(_es, es)
	}
	return _es
}

func (self *HomingMove) Calc_endstop_rate(mcu_endstop interface{}, movepos []float64, speed float64) float64 {
	var startpos = self.Toolhead.Get_position()

	var axes_d []float64
	for i, f := range startpos {
		axes_d = append(axes_d, movepos[i]-f)
	}

	var sum float64
	for _, d := range axes_d[:3] {
		sum += d * d
	}

	var move_d = math.Sqrt(sum)
	var move_t = move_d / speed

	var max = 0.0
	if _, ok := mcu_endstop.(*ProbeEndstopWrapper); ok {
		for _, s := range mcu_endstop.(*ProbeEndstopWrapper).Get_steppers() {
			var min = math.Abs(s.(*MCU_stepper).Calc_position_from_coord(startpos)-s.(*MCU_stepper).Calc_position_from_coord(movepos)) / s.(*MCU_stepper).Get_step_dist()
			max = math.Max(min, max)
		}
	} else {
		for _, s := range mcu_endstop.(*MCU_endstop).Get_steppers() {
			var min = math.Abs(s.(*MCU_stepper).Calc_position_from_coord(startpos)-s.(*MCU_stepper).Calc_position_from_coord(movepos)) / s.(*MCU_stepper).Get_step_dist()
			max = math.Max(min, max)
		}
	}

	var max_steps = max
	if max_steps < 0. {
		return .001
	}
	return move_t / max_steps
}

func (self *HomingMove) Calc_toolhead_pos(kin_spos1 map[string]float64, offsets map[string]float64) []float64 {
	// kin_spos = dict(kin_spos)
	kin_spos := make(map[string]float64, len(kin_spos1))

	for key, value := range kin_spos1 {
		kin_spos[key] = value
	}

	var kin = self.Toolhead.Get_kinematics().(IKinematics)
	for _, stepper := range kin.Get_steppers() {
		var sname = stepper.(*MCU_stepper).Get_name(false)
		//value.StaticValue.Debug.Printf("offsets[sname]:%v stepper.(*MCU_stepper).Get_step_dist():%v offsets[sname] * stepper.(*MCU_stepper).Get_step_dist() %f \n", offsets[sname], stepper.(*MCU_stepper).Get_step_dist(), offsets[sname]*stepper.(*MCU_stepper).Get_step_dist())
		kin_spos[sname] += offsets[sname] * stepper.(*MCU_stepper).Get_step_dist()
	}
	var thpos = self.Toolhead.Get_position()
	spos := []float64{}
	spos = append(spos, kin.Calc_position(kin_spos)[:3]...)
	spos = append(spos, thpos[3:]...)
	return spos
}

func (self *HomingMove) Homing_move(movepos []float64, speed float64, probe_pos bool,
	triggered bool, check_triggered bool) interface{} {
	// Notify start of homing/probing move
	self.Printer.Send_event("homing:homing_move_begin", []interface{}{self})
	// Note start location
	self.Toolhead.Flush_step_generation()
	kin := self.Toolhead.Get_kinematics().(IKinematics)

	kin_spos := map[string]float64{}
	arr := kin.Get_steppers()
	for _, s := range arr {
		num := s.(*MCU_stepper).Get_commanded_position()
		key := s.(*MCU_stepper).Get_name(false)
		kin_spos[key] = num
	}
	//log.Printf("Homing_move self.Endstops:%v", self.Endstops)
	for _, l := range self.Endstops {
		name := l.Back().Value.(string)
		if _, ok := l.Front().Value.(*ProbeEndstopWrapper); ok {
			es := l.Front().Value.(*ProbeEndstopWrapper)
			for _, s := range es.Get_steppers() {
				self.Stepper_positions = append(self.Stepper_positions, NewStepperPosition(s.(*MCU_stepper), name))
			}
		} else if _, ok := l.Front().Value.(*MCU_endstop); ok {
			es := l.Front().Value.(*MCU_endstop)
			for _, s := range es.Get_steppers() {
				self.Stepper_positions = append(self.Stepper_positions, NewStepperPosition(s.(*MCU_stepper), name))
			}
		}
	}
	//log.Printf("Homing_move self.Stepper_positions:%v", self.Stepper_positions)

	// Start endstop checking
	var print_time = self.Toolhead.Get_last_move_time()
	var endstop_triggers = []*ReactorCompletion{}
	for _, l := range self.Endstops {
		if _, ok := l.Front().Value.(*ProbeEndstopWrapper); ok {
			mcu_endstop := l.Front().Value.(*ProbeEndstopWrapper)
			var rest_time = self.Calc_endstop_rate(mcu_endstop, movepos, speed)
			_triggered := 0
			if triggered {
				_triggered = 1
			} else {
				_triggered = 0
			}
			var wait = mcu_endstop.Home_start.(func(float64, float64, int64, float64, int64) interface{})(print_time, ENDSTOP_SAMPLE_TIME,
				ENDSTOP_SAMPLE_COUNT, rest_time,
				int64(_triggered))
			endstop_triggers = append(endstop_triggers, wait.(*ReactorCompletion))
		} else {
			mcu_endstop := l.Front().Value.(*MCU_endstop)
			var rest_time = self.Calc_endstop_rate(mcu_endstop, movepos, speed)
			var _triggered int64 = 0
			if triggered {
				_triggered = 1
			} else {
				_triggered = 0
			}
			var wait = mcu_endstop.Home_start(print_time, ENDSTOP_SAMPLE_TIME,
				ENDSTOP_SAMPLE_COUNT, rest_time,
				_triggered)
			endstop_triggers = append(endstop_triggers, wait.(*ReactorCompletion))
		}

	}
	var all_endstop_trigger = Multi_complete(self.Printer, endstop_triggers)
	self.Toolhead.Dwell(HOMING_START_DELAY)
	// Issue move
	var error1 string = ""
	// try:
	err := self.Toolhead.Drip_move(movepos, speed, all_endstop_trigger)
	if err != nil {
		error1 = fmt.Sprintf("Error during homing move: %s", err.Error())
	}
	// Wait for endstops to trigger
	var trigger_times = map[string]float64{}
	var trigger_time float64
	var move_end_print_time = self.Toolhead.Get_last_move_time()
	for _, l := range self.Endstops {
		if _, ok := l.Front().Value.(*MCU_endstop); ok {
			mcu_endstop := l.Front().Value.(*MCU_endstop)
			trigger_time = mcu_endstop.Home_wait(move_end_print_time)
		} else {
			mcu_endstop := l.Front().Value.(*ProbeEndstopWrapper)
			trigger_time = mcu_endstop.Home_wait.(func(float64) float64)(move_end_print_time)
		}
		name := l.Back().Value.(string)
		//value.StaticValue.Debug.Printf("trigger_times:%f \n", trigger_time)
		if trigger_time > 0. {
			trigger_times[name] = trigger_time
		} else if trigger_time < 0. && error1 == "" {
			error1 = fmt.Sprintf("Communication timeout during homing %s", name)
		} else if check_triggered && error1 == "" {
			error1 = fmt.Sprintf("No trigger on %s after full movement", name)
		}
	}
	// Determine stepper halt positions
	self.Toolhead.Flush_step_generation()
	for _, sp := range self.Stepper_positions {
		var tt = move_end_print_time
		if _, ok := trigger_times[sp.Endstop_name]; ok {
			tt = trigger_times[sp.Endstop_name]
		}

		sp.Note_home_end(tt)
	}
	haltpos, trigpos := []float64{}, []float64{}
	if probe_pos {

		halt_steps := map[string]float64{}
		for _, sp := range self.Stepper_positions {
			halt_steps[sp.Stepper_name] = float64(sp.Halt_pos - sp.Start_pos)
		}

		trig_steps := map[string]float64{}
		for _, sp := range self.Stepper_positions {
			trig_steps[sp.Stepper_name] = float64(sp.Trig_pos - sp.Start_pos)
		}
		//log.Printf("homing_move halt_steps:%v trig_steps:%v \n", halt_steps, trig_steps)
		haltpos = self.Calc_toolhead_pos(kin_spos, trig_steps)
		trigpos = haltpos
		//log.Printf("homing_move haltpos:%v\n", haltpos)
		if reflect.DeepEqual(trig_steps, halt_steps) == false {
			haltpos = self.Calc_toolhead_pos(kin_spos, halt_steps)
			log.Printf("homing_move else haltpos:%v\n", haltpos)
		}
	} else {
		for _, v := range movepos {
			haltpos = append(haltpos, v)
			trigpos = append(trigpos, v)
		}
		over_steps := map[string]float64{}
		for _, sp := range self.Stepper_positions {
			over_steps[sp.Stepper_name] = float64(sp.Halt_pos - sp.Trig_pos)
		}
		var values bool = false
		for _, v := range over_steps {
			if v > 0. || v < 0. {
				values = true
			}
		}
		if values {
			self.Toolhead.Set_position(movepos, []int{})
			halt_kin_spos := map[string]float64{}
			for _, s := range kin.Get_steppers() {
				halt_kin_spos[s.(*MCU_stepper).Get_name(false)] = s.(*MCU_stepper).Get_commanded_position()
			}
			haltpos = self.Calc_toolhead_pos(halt_kin_spos, over_steps)
		}
	}

	self.Toolhead.Set_position(haltpos, []int{})
	// Signal homing/probing move complete
	_, err1 := self.Printer.Send_event("homing:homing_move_end", []interface{}{self})
	if err1 != nil {
		error1 = err1.Error()
	}
	if error1 != "" {
		panic(error1)
	}
	return trigpos
}

func (self *HomingMove) Check_no_movement() string {
	if self.Printer.Get_start_args()["debuginput"] != "" {
		return ""
	}
	for _, sp := range self.Stepper_positions {
		if sp.Start_pos == sp.Trig_pos {
			return sp.Endstop_name
		}
	}
	return ""
}

// State tracking of homing requests
type Homing struct {
	Printer         *Printer
	Toolhead        *Toolhead
	Changed_axes    []int
	Trigger_mcu_pos map[string]float64
	Adjust_pos      map[string]float64
}

func NewHoming(printer *Printer) *Homing {
	var self = Homing{}
	self.Printer = printer
	toolhead := printer.Lookup_object("toolhead", object.Sentinel{})
	//if err != nil {
	//	log.Print(err.Error())
	//} else {
	self.Toolhead = toolhead.(*Toolhead)
	//}
	self.Changed_axes = []int{}
	self.Trigger_mcu_pos = map[string]float64{}
	self.Adjust_pos = map[string]float64{}
	return &self
}

func (self *Homing) Set_axes(axes []int) {
	self.Changed_axes = axes
}

func (self *Homing) Get_axes() []int {
	return self.Changed_axes
}

func (self *Homing) Get_trigger_position(stepper_name string) float64 {
	return self.Trigger_mcu_pos[stepper_name]
}

func (self *Homing) Set_stepper_adjustment(stepper_name string, adjustment float64) {
	self.Adjust_pos[stepper_name] = adjustment
}

func (self *Homing) Fill_coord(coord []interface{}) []float64 {
	// Fill in any None entries in "coord" with current toolhead position
	position := self.Toolhead.Get_position()
	thcoord := make([]float64, len(position))
	copy(thcoord, position)
	for i := 0; i < len(coord); i++ {
		if coord[i] != nil {
			thcoord[i] = coord[i].(float64)
		}
	}
	return thcoord
}

func (self *Homing) Set_homed_position(pos []float64) {
	self.Toolhead.Set_position(self.Fill_coord(collections.FloatInterface(pos)), []int{})
}

func (self *Homing) Home_rails(rails []*PrinterRail, forcepos []interface{}, movepos []interface{}, hasZ bool) {
	// Notify of upcoming homing operation
	self.Printer.Send_event("homing:home_rails_begin", []interface{}{self, rails})
	// Alter kinematics class to think printer is at forcepos
	var homing_axes []int
	for axis := 0; axis < 3; axis++ {
		if forcepos[axis] != nil {
			homing_axes = append(homing_axes, axis)
		}
	}
	var startpos = self.Fill_coord(forcepos)
	var homepos = self.Fill_coord(movepos)
	self.Toolhead.Set_position(startpos, homing_axes)
	// Perform first home
	endstops := []list.List{}
	for _, rail := range rails {
		for _, es := range rail.Get_endstops() {
			endstops = append(endstops, es)
		}
	}
	var hi = rails[0].Get_homing_info()
	hmove := NewHomingMove(self.Printer, endstops, nil)
	if hasZ {
		self.Printer.Send_event("leviq3:fan_close", nil)
	}
	self.Printer.Send_event("homing:sgthrs_calibration_start", []interface{}{rails[0].Get_name(false)})

	hmove.Homing_move(homepos, hi.Speed, false, true, true)
	//value.StaticValue.Debug.Printf("homePos: %v \n", pos)
	// Perform second home
	if hi.Retract_dist > 0 {
		// Retract
		startpos = self.Fill_coord(forcepos)
		homepos = self.Fill_coord(movepos)
		var axes_d []float64
		for i, hp := range homepos {
			axes_d = append(axes_d, hp-startpos[i])
		}
		var sum float64
		for _, d := range axes_d[:3] {
			sum += d * d
		}

		var move_d = math.Sqrt(sum)

		var retract_r = math.Min(1., hi.Retract_dist/move_d)
		var retractpos []float64
		for i, hp := range homepos {
			retractpos = append(retractpos, hp-axes_d[i]*retract_r)
		}
		self.Printer.Send_event("homing:check_ola_olb_begin", []interface{}{rails[0].Get_name(false)})
		self.Toolhead.Move(retractpos, hi.Retract_speed)
		self.Toolhead.Flush_step_generation()
		self.Toolhead.Move(homepos, hi.Second_homing_speed)
		self.Toolhead.Wait_moves()
		self.Printer.Send_event("homing:check_ola_olb_end", []interface{}{rails[0].Get_name(false)})
	}
	// Signal home operation complete
	self.Toolhead.Flush_step_generation()
	for _, sp := range hmove.Stepper_positions {
		self.Trigger_mcu_pos[sp.Stepper_name] = float64(sp.Trig_pos)
	}

	self.Toolhead.Set_position(homepos, []int{})

	self.Adjust_pos = map[string]float64{}
	self.Printer.Send_event("homing:home_rails_end", []interface{}{self, rails})

	var values bool = false
	for _, v := range self.Adjust_pos {
		if v < 0. || v > 0. {
			values = true
		}
	}

	if values {
		// Apply any homing offsets
		var kin = self.Toolhead.Get_kinematics().(IKinematics)
		homepos = self.Toolhead.Get_position()
		kin_spos := map[string]float64{}
		for _, s := range kin.Get_steppers() {
			if s.(*MCU_stepper).Get_name(false) == "" {
				kin_spos[s.(*MCU_stepper).Get_name(false)] = s.(*MCU_stepper).Get_commanded_position() + 0.0
			} else {
				kin_spos[s.(*MCU_stepper).Get_name(false)] = s.(*MCU_stepper).Get_commanded_position() + self.Adjust_pos[s.(*MCU_stepper).Get_name(false)]
			}
		}
		newpos := kin.Calc_position(kin_spos)
		for _, axis := range homing_axes {
			homepos[axis] = newpos[axis]
		}
		self.Toolhead.Set_position(homepos, []int{})
	}
}

type PrinterHoming struct {
	Printer *Printer
	config  *ConfigWrapper
}

func NewPrinterHoming(config *ConfigWrapper) *PrinterHoming {
	var self = PrinterHoming{}
	self.config = config
	self.Printer = config.Get_printer()
	// Register g-code commands
	gcode := self.Printer.Lookup_object("gcode", object.Sentinel{})
	//if err != nil {
	//	log.Print(err.Error())
	//}
	gcode.(*GCodeDispatch).Register_command("G28", self.Cmd_G28, false, "")
	return &self
}

func (self *PrinterHoming) Manual_home(toolhead interface{}, endstops []list.List, pos []float64, speed float64,
	triggered bool, check_triggered bool) {
	hmove := NewHomingMove(self.Printer, endstops, toolhead)
	// try:
	hmove.Homing_move(pos, speed, false, triggered, check_triggered)
	//     except self.Printer.command_error:
	//         if self.Printer.is_shutdown():
	//             raise self.Printer.command_error(
	//                 "Homing failed due to printer shutdown")
	//         raise
}

func (self *PrinterHoming) Probing_move(mcu_probe interface{}, pos []float64, speed float64) []float64 {
	endstops := []list.List{}
	endstop := list.List{}
	endstop.PushBack(mcu_probe)
	endstop.PushBack("probe")
	endstops = append(endstops, endstop)
	//log.Printf("Probing_move pos:%v speed:%v\n", pos, speed)
	hmove := NewHomingMove(self.Printer, endstops, nil)
	// try:
	epos := hmove.Homing_move(pos, speed, true, true, true)
	// except self.Printer.command_error:
	//     if self.Printer.is_shutdown():
	//         raise self.Printer.command_error(
	//             "Probing failed due to printer shutdown")
	//     raise
	if hmove.Check_no_movement() != "" {
		panic("Probe triggered prior to movement")
	}
	return epos.([]float64)
}

func (self *PrinterHoming) Cmd_G28(argv interface{}) error {
	is_homing = 1
	defer func() {
		if err := recover(); err != nil {
			is_homing = 0
			panic(err)
		}
	}()
	gcmd := argv.(*GCodeCommand)

	// Move to origin
	axes := []int{}
	for pos, axis := range strings.Split("X Y Z", " ") {
		if gcmd.Has(axis) {
			axes = append(axes, pos)
		}
	}

	if len(axes) == 0 {
		axes = []int{0, 1, 2}
	}
	homing_state := NewHoming(self.Printer)
	homing_state.Set_axes(axes)
	toolhead := self.Printer.Lookup_object("toolhead", object.Sentinel{}) //.get_kinematics()
	//if err != nil {
	//	log.Print(err.Error())
	//}
	kin := toolhead.(*Toolhead).Get_kinematics().(IKinematics)
	//kin.Home(homing_state)
	//log.Print(kin)
	//CatchPanic()
	/* gcode := self.Printer.Lookup_object("gcode", object.Sentinel{}).(*GCodeDispatch)
	gcode.Run_script_from_command("M400") */
	self.tryCatchhome1(kin, homing_state)
	is_homing = 2
	return nil
}

func (self *PrinterHoming) tryCatchhome1(kin IKinematics, homing_state *Homing) {
	defer func() {
		//处理panic
		if err := recover(); err != nil {
			_, ok := err.(error)
			if ok {
				if self.Printer.Is_shutdown() {
					panic("Homing failed due to printer shutdown")
				}
				motor := self.Printer.Lookup_object("stepper_enable", object.Sentinel{})
				//self.printer.lookup_object('stepper_enable').motor_off()
				motor.(*PrinterStepperEnable).Motor_off()
			}
			panic(err)
		}
	}()
	kin.Home(homing_state)
}

func Load_config_homing(config *ConfigWrapper) interface{} {
	return NewPrinterHoming(config)
}
