package idex_modes

import (
	"k3c/common/value"
	"k3c/project"
)

// Support for duplication and mirroring modes for IDEX printers
//
// Copyright (C) 2021  Fabrice Gallet <tircown@gmail.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

type DualCarriages struct {
	Printer     *project.Printer
	Axis        int
	Dc          []DualCarriagesRail
	Saved_state map[string]interface{}
}

func NewDualCarriages(printer *project.Printer, rail_0, rail_1 project.PrinterRail, axis int) *DualCarriages {
	self := &DualCarriages{}
	self.Printer = printer
	self.Axis = axis
	self.Dc = []project.PrinterRail{rail_0, rail_1}
	self.Printer.Add_object("dual_carriage", self.Dc)
	gcode_obj, err := self.Printer.Lookup_object("gcode", nil)
	if err != nil {
		value.StaticValue.Error.Println(err)
	}
	gcode := gcode_obj.(project.GCodeDispatch)
	gcode.Register_command("SET_DUAL_CARRIAGE", self.Cmd_SET_DUAL_CARRIAGE,
		false,
		cmd_SET_DUAL_CARRIAGE_help)
	return self
}
func (self *DualCarriages) ToggleActiveDcRail(index int) {
	toolhead_obj, err := self.Printer.Lookup_object("toolhead", nil)
	if err != nil {
		value.StaticValue.Error.Println(err)
	}
	toolhead := toolhead_obj.(project.Toolhead)
	toolhead.Flush_step_generation()
	pos := toolhead.Get_position()
	kin := toolhead.Get_kinematics()
	for i, dc := range self.Dc {
		dc_rail := self.Get_rail()
		if i != index {
			dc.Inactivate(pos)
			kin.Override_rail(3, dc_rail)
		} else if dc.Is_active() == false {
			newpos := append(pos[:dc.Axis], dc.Axis_position)
			dc.Activate(newpos)
			kin.Override_rail(dc.Axis, dc_rail)
			toolhead.Set_position(newpos, []int{})
			kin.Update_limits(dc.Axis, dc_rail.Get_range())
		}
	}
}
func (self *DualCarriages) GetStatus(eventtime float64) map[string]interface{} {
	dc0 := self.Dc[0]
	// dc1 := self.Dc[1]
	if dc0.Is_active() == true {
		return map[string]interface{}{
			"mode":            "FULL_CONTROL",
			"active_carriage": "CARRIAGE_0",
		}
	} else {
		return map[string]interface{}{
			"mode":            "FULL_CONTROL",
			"active_carriage": "CARRIAGE_1",
		}
	}
}

func (self *DualCarriages) Save_idex_state() {
	dc0 := self.Dc[0]
	dc1 := self.Dc[1]
	var mode, active_carriage string
	if dc0.Is_active() == true {
		mode, active_carriage = "FULL_CONTROL", "CARRIAGE_0"
	} else {
		mode, active_carriage = "FULL_CONTROL", "CARRIAGE_1"
	}
	self.Saved_state = map[string]interface{}{
		"mode":            mode,
		"active_carriage": active_carriage,
		"axis_positions":  []float64{dc0.Axis_position, dc1.Axis_position},
	}
}
func (self *DualCarriages) Restore_idex_state() {
	if self.Saved_state != nil {
		// set carriage 0 active
		if self.Saved_state["active_carriage"] == "CARRIAGE_0" && !self.Dc[0].Is_active() {
			self.Toggle_active_dc_rail(0)
		}
		// set carriage 1 active
		if self.Saved_state["active_carriage"] == "CARRIAGE_1" && !self.Dc[1].Is_active() {
			self.Toggle_active_dc_rail(1)
		}
	}
}

const cmd_SET_DUAL_CARRIAGE_help = "Set which carriage is active"

func (self *DualCarriages) Cmd_SET_DUAL_CARRIAGE(gcmd *project.GCodeCommand) {
	zero := 0.
	one := 1.
	index := gcmd.Get_int("CARRIAGE", nil, "", &zero, &one, &zero, &zero)
	if !(self.Dc[0].Is_active() == self.Dc[1].Is_active() == true) && !self.Dc[index].Is_active() {
		self.Toggle_active_dc_rail(index)
	}
}

const (
	ACTIVE   = 1
	INACTIVE = 2
)

type DualCarriagesRail struct {
	Printer                *project.Printer
	Rail                   *project.PrinterRail
	Axis                   int
	Status                 int
	Stepper_alloc_active   []int
	Stepper_alloc_inactive []int
	Axis_position          int
}

func NewDualCarriagesRail(printer *project.Printer, rail *project.PrinterRail, axis int, active bool, stepper_alloc_active []int, stepper_alloc_inactive []int) *DualCarriagesRail {
	self := &DualCarriagesRail{}
	self.Printer = printer
	self.Rail = rail
	self.Axis = axis
	if active {
		self.Status = ACTIVE
	} else {
		self.Status = INACTIVE
	}
	self.Stepper_alloc_active = stepper_alloc_active
	self.Stepper_alloc_inactive = stepper_alloc_inactive
	self.Axis_position = -1
	return self
}

func (self *DualCarriagesRail) Stepper_alloc(position []float64, active bool) {
	toolhead_obj, err := self.Printer.Lookup_object("toolhead", nil)
	if err != nil {
		value.StaticValue.Error.Println(err)
	}
	toolhead := toolhead_obj.(project.Toolhead)
	self.Axis_position = int(position[self.Axis])
	self.Rail.Set_trapq(0)
	if active {
		self.Status = ACTIVE
		if self.Stepper_alloc_active != nil {
			self.Rail.Setup_itersolve("", self.Stepper_alloc_active)
			self.Rail.Set_position(position)
			self.Rail.Set_trapq(toolhead.Get_trapq().(int))
		}
	} else {
		self.Status = INACTIVE
		if self.Stepper_alloc_inactive != nil {
			self.Rail.Setup_itersolve("", self.Stepper_alloc_inactive)
			self.Rail.Set_position(position)
			self.Rail.Set_trapq(toolhead.Get_trapq().(int))
		}
	}
}
func (self *DualCarriagesRail) GetRail() *project.PrinterRail {
	return self.Rail
}

func (self *DualCarriagesRail) IsActive() bool {
	return self.Status == ACTIVE
}

func (self *DualCarriagesRail) Activate(position []float64) {
	self.Stepper_alloc(position, true)
}

func (self *DualCarriagesRail) Inactivate(position []float64) {
	self.Stepper_alloc(position, false)
}
