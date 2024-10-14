package project

import (
	"fmt"
	kerror "k3c/common/errors"
	"k3c/common/utils/object"
	"k3c/common/value"
	"strconv"
)

type LeviQ3Helper struct {
	printer        *Printer
	reactor        IReactor
	gcode          *GCodeDispatch
	config         *ConfigWrapper
	bed_temp       float64
	extru_temp     float64
	extru_end_temp float64
	to_z_up_speed  float64
	z_up           float64
}

func NewLeviQ3Helper(config *ConfigWrapper) *LeviQ3Helper {
	l := LeviQ3Helper{}
	l.printer = config.Get_printer()
	l.reactor = l.printer.Get_reactor()
	l.config = config
	gcode := l.printer.Lookup_object("gcode", object.Sentinel{})
	l.bed_temp = config.Getfloat("bed_temp", 60.0, 0, 0, 0, 0, true)
	l.extru_temp = config.Getfloat("extru_temp", 190.0, 0, 0, 0, 0, true)
	l.extru_end_temp = config.Getfloat("extru_end_temp", 140.0, 0, 0, 0, 0, true)
	l.to_z_up_speed = config.Getfloat("to_z_up_speed", 15.0, 0, 0, 0, 0, true)
	l.z_up = config.Getfloat("z_up", 15.0, 0, 0, 0, 0, true)
	l.gcode = gcode.(*GCodeDispatch)
	l.printer.Register_event_handler("leviq3:wait", l.leviq3_wait)
	l.gcode.Register_command("LEVIQ3", l.CMD_LEVIQ3, false, l.CMD_LEVIQ3_HELP())
	l.gcode.Register_command("LEVIQ_SET_ZERO", l.leviq_set_zero, false, l.CMD_LEVIQ3_HELP())
	l.gcode.Register_command("LEVIQ3_PREHEATING", l.CMD_LEVIQ3_PREHEATING, false, l.CMD_LEVIQ3_PREHEATING_HELP())
	l.gcode.Register_command("LEVIQ3_WIPING", l.CMD_LEVIQ3_WIPING, false, l.CMD_LEVIQ3_WIPING_HELP())
	l.gcode.Register_command("LEVIQ2_PROBE", l.CMD_LEVIQ3_PROBE, false, l.CMD_LEVIQ3_PROBE_HELP())
	return &l
}

func (l *LeviQ3Helper) leviq3_wait([]interface{}) error {
	var toolhead = l.printer.Lookup_object("toolhead", object.Sentinel{}).(*Toolhead)
	toolhead.Wait_moves()
	toolhead.Dwell(0.25)
	return nil
}

func (l *LeviQ3Helper) leviq_set_zero(arg interface{}) error {
	gCodeMove := l.printer.Lookup_object("gcode_move", object.Sentinel{}).(*GCodeMove)
	gCodeMove.base_position[2] = 0.0
	gCodeMove.homing_position[2] = 0.0
	return nil
}

func (l *LeviQ3Helper) CMD_LEVIQ3(arg interface{}) error {
	gcmd := arg.(*GCodeCommand)
	err := l.CMD_LEVIQ3_PREHEATING(arg)
	if err != nil {
		return err
	}

	err = l.CMD_LEVIQ3_WIPING(arg)
	if err != nil {
		return err
	}

	err = l.CMD_LEVIQ3_PROBE(arg)
	if err != nil {
		return err
	}
	sync := gcmd.Get("SYNC", "disable", value.None, nil, nil, nil, nil)
	if sync == "enable" {
		gcode := MustLookupGcode(l.printer)
		gcode.Request_restart("restart")
	}
	return nil
}

func (l *LeviQ3Helper) CMD_LEVIQ3_PREHEATING(arg interface{}) error {

	gcmd := arg.(*GCodeCommand)
	if u, ok := gcmd.Params["S"]; ok {
		if uFloat, err := strconv.ParseFloat(u, 64); err == nil {
			l.extru_temp = uFloat
		}
	}
	if d, ok := gcmd.Params["E"]; ok {
		if dFloat, err := strconv.ParseFloat(d, 64); err == nil {
			l.extru_end_temp = dFloat
		}
	}
	l.leviq_set_zero(nil)
	l.gcode.Run_script_from_command(fmt.Sprintf("M140 S%f", l.bed_temp))
	l.gcode.Run_script_from_command(fmt.Sprintf("M104 S%f", l.extru_temp))
	toolhead := l.printer.Lookup_object("toolhead", object.Sentinel{}).(*Toolhead)
	erro := func() (err error) {
		defer func() {
			if r := recover(); r != nil {
				value.StaticValue.Error.Println(r)
				panic(r)
			}
		}()
		// G28
		l.gcode.Run_script_from_command("HOME_XY")
		toolhead.Wait_moves()
		toolhead.Dwell(1.0)
		return nil
	}()
	if erro != nil {
		return erro
	}
	l.gcode.Run_script_from_command(fmt.Sprintf("M109 S%f", l.extru_temp))
	extruder_heater_check := l.printer.Lookup_object("verify_heater extruder", object.Sentinel{}).(*HeaterCheck)
	heater_bed_heater_check := l.printer.Lookup_object("verify_heater heater_bed", object.Sentinel{}).(*HeaterCheck)
	if heater_bed_heater_check.err {
		return kerror.HeaterBedHeatingError
	}
	extruder := l.printer.Lookup_object("extruder", object.Sentinel{}).(*PrinterExtruder)
	tempMap := extruder.Get_status(l.reactor.Monotonic())
	if tempMap["temperature"].(float64) < l.extru_temp {
		l.gcode.Run_script_from_command(fmt.Sprintf("M109 S%f", l.extru_temp))
	}
	if extruder_heater_check.err {
		return kerror.ExtruderHeatingError
	}
	return nil
}

func (l *LeviQ3Helper) CMD_LEVIQ3_WIPING(arg interface{}) error {

	l.gcode.Run_script_from_command("BED_MESH_CLEAR")
	l.gcode.Run_script_from_command(fmt.Sprintf("M104 S%f", l.extru_end_temp))
	toolhead := l.printer.Lookup_object("toolhead", object.Sentinel{}).(*Toolhead)
	l.gcode.Run_script_from_command("G9112")
	toolhead.Wait_moves()
	toolhead.Dwell(2.0)
	l.gcode.Run_script_from_command("M400")
	l.gcode.Run_script_from_command("M107")
	l.gcode.Run_script_from_command("G28 Z")
	toolhead.Wait_moves()
	toolhead.Dwell(1.0)
	return nil
}

func (l *LeviQ3Helper) CMD_LEVIQ3_PROBE(arg interface{}) error {

	erro := func() (err error) {
		defer func() {
			if r := recover(); r != nil {
				value.StaticValue.Error.Println(r)
				err = kerror.AutomaticLevelingFailedError
			}
		}()

		l.gcode.Run_script_from_command("BED_MESH_CALIBRATE")
		return nil
	}()
	if erro != nil {
		handErrorLeviq3(l)
		return erro
	}
	l.gcode.Run_script_from_command("m106 S0")
	l.gcode.Run_script_from_command("M190 S0")
	l.gcode.Run_script_from_command(fmt.Sprintf("G1 Z%f F%f", l.z_up, l.to_z_up_speed*60.0))
	l.gcode.Run_script_from_command("M400")
	l.gcode.Run_script_from_command("HOME_XY")
	l.gcode.Run_script_from_command("M109 S0")
	return nil
}

func handErrorLeviq3(l *LeviQ3Helper) {
	gcode_move_obj := l.printer.Lookup_object("gcode_move", object.Sentinel{})
	gcode_move := gcode_move_obj.(*GCodeMove)
	for i := 0; i < 3; i++ {
		gcode_move.base_position[i] = 0.0
		gcode_move.homing_position[i] = 0.0
	}
	l.gcode.Run_script_from_command("M190 S0")
	l.gcode.Run_script_from_command("M109 S0")
	l.gcode.Run_script_from_command("M107")
	l.gcode.Run_script_from_command("M84")
}

func (l *LeviQ3Helper) CMD_LEVIQ3_HELP() string {
	return "Calculates the z_offset automatically"
}

func (l *LeviQ3Helper) CMD_LEVIQ3_PREHEATING_HELP() string {
	return "Automatic leveling function preheating"
}

func (l *LeviQ3Helper) CMD_LEVIQ3_WIPING_HELP() string {
	return "Automatic leveling function wiping"
}

func (l *LeviQ3Helper) CMD_LEVIQ3_PROBE_HELP() string {
	return "Automatic leveling function probe"
}

func Load_config_LeviQ3(config *ConfigWrapper) interface{} {
	return NewLeviQ3Helper(config)
}
