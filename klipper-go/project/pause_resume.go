package project

import (
	"fmt"
	"k3c/common/utils/object"
	"k3c/common/value"
	"log"
	"os"
	"strconv"
)

/*
# Pause/Resume functionality with position capture/restore
#
# Copyright (C) 2019  Eric Callahan <arksine.code@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

*/

type PauseResume struct {
	Printer            *Printer
	Gcode              *GCodeDispatch
	Recover_velocity   float64
	V_sd               *VirtualSD
	Is_paused          bool
	Sd_paused          bool
	Pause_command_sent bool
	config             *ConfigWrapper
	pause_position     []float64
	extru_pull         float64
	z_up               float64
	extru_load         float64
	extru_unload       float64
	hasM600            bool
	// m600 params
	params_l float64
	params_u float64
	params_b int64
	bee_path string
}

func NewPauseResume(config *ConfigWrapper) *PauseResume {
	self := &PauseResume{}
	self.config = config
	self.Printer = config.Get_printer()
	self.Gcode = self.Printer.Lookup_object("gcode", object.Sentinel{}).(*GCodeDispatch)
	self.Recover_velocity = config.Getfloat("recover_velocity", 50., 0., 0., 0., 0., true)
	self.pause_position = config.Getfloatlist("pause_position", []float64{200.0, 200.0}, ",", 2, false)
	self.z_up = config.Getfloat("z_up", 10., 0., 0., 0., 0., true)
	self.extru_pull = config.Getfloat("extru_pull", 2., 0., 0., 0., 0., true)
	self.extru_load = config.Getfloat("extru_load", 50.0, 0., 0., 0., 0., true)
	self.extru_unload = config.Getfloat("extru_unload", 2., 0., 0., 0., 0., true)
	self.bee_path = config.Get("bee_path", "", false).(string)
	self.V_sd = nil
	self.Is_paused = false
	self.Sd_paused = false
	self.Pause_command_sent = false

	self.Printer.Register_event_handler("project:connect", self.Handle_connect)

	self.Gcode.Register_command("PAUSE", self.Cmd_PAUSE, false, Cmd_PAUSE_help)
	self.Gcode.Register_command("RESUME", self.Cmd_RESUME, false, Cmd_RESUME_help)
	self.Gcode.Register_command("CLEAR_PAUSE", self.Cmd_CLEAR_PAUSE, false, Cmd_CLEAR_PAUSE_help)
	self.Gcode.Register_command("CANCEL_PRINT", self.Cmd_CANCEL_PRINT, false, Cmd_CANCEL_PRINT_help)
	self.Gcode.Register_command("M600", self.Cmd_M600, false, Cmd_CANCEL_PRINT_help)

	webhooks := self.Printer.Load_object(config, "webhooks", object.Sentinel{}).(*WebHooks)
	webhooks.Register_endpoint("pause_resume/cancel", self.Handle_cancel_request)
	webhooks.Register_endpoint("pause_resume/pause", self.Handle_pause_request)
	webhooks.Register_endpoint("pause_resume/resume", self.Handle_resume_request)
	return self
}
func (self *PauseResume) Handle_connect(args []interface{}) error {
	self.V_sd = self.Printer.Lookup_object("virtual_sdcard", value.None).(*VirtualSD)
	return nil
}
func (self *PauseResume) Handle_cancel_request(web_request *WebRequest) (interface{}, error) {
	self.Printer.Send_event("printer_macro:canel", nil)
	self.Gcode.Run_script_from_command("TURN_OFF_HEATERS")
	self.Gcode.Run_script("CANCEL_PRINT")
	self.Printer.Send_event("printer_macro:reset", nil)
	return nil, nil
}
func (self *PauseResume) Handle_pause_request(web_request *WebRequest) (interface{}, error) {
	self.Gcode.Run_script("PAUSE")
	return nil, nil
}
func (self *PauseResume) Handle_resume_request(web_request *WebRequest) (interface{}, error) {
	self.Gcode.Run_script("RESUME")
	return nil, nil
}
func (self *PauseResume) Get_status(eventtime float64) map[string]interface{} {
	return map[string]interface{}{
		"is_paused": self.Is_paused,
	}
}
func (self *PauseResume) Is_sd_active() bool {
	return self.V_sd != nil && self.V_sd.Is_active()
}
func (self *PauseResume) Send_pause_command() {
	/*
		# This sends the appropriate pause command from an event.  Note
		# the difference between pause_command_sent and is_paused, the
		# module isn't officially paused until the PAUSE gcode executes.
	*/
	if !self.Pause_command_sent {
		if self.Is_sd_active() {
			// Printing from virtual sd, run pause command
			self.Sd_paused = true
			self.V_sd.Do_pause()
		} else {
			self.Sd_paused = false
			self.Gcode.Respond_info("action:paused", true)
		}
		self.Pause_command_sent = true
	}
}

const Cmd_PAUSE_help = "Pauses the current print"

func (self *PauseResume) Cmd_PAUSE(gcmd interface{}) error {
	if self.Is_paused {
		gcmd.(*GCodeCommand).Respond_info("Print already paused", true)
		return nil
	}
	printStats := self.Printer.Lookup_object("print_stats", object.Sentinel{}).(*PrintStats)
	// 升温中不执行坐标保存和移动逻辑
	self.Send_pause_command()
	self.Gcode.Run_script_from_command("SAVE_GCODE_STATE NAME=PAUSE_STATE")
	self.Is_paused = true
	if is_homing == 2 {
		self.Gcode.Run_script_from_command("G91")
		self.Gcode.Run_script_from_command(fmt.Sprintf("G1 E-%f F1800", self.extru_pull))

		self.Gcode.Run_script_from_command(fmt.Sprintf("G1 Z%f F1800", self.z_up))
		self.Gcode.Run_script_from_command("G90")
		if len(self.pause_position) > 0 {
			self.Gcode.Run_script_from_command(fmt.Sprintf("G1 X%f Y%f F3600", self.pause_position[0], self.pause_position[1]))
		}
	}
	// self.Printer.reactor.Pause(self.Printer.reactor.Monotonic() + 5.0)
	printStats.state = Pause_

	return nil
}
func (self *PauseResume) Send_resume_command() {
	if self.Sd_paused {
		// Printing from virtual sd, run pause command
		self.V_sd.Do_resume()
		self.Sd_paused = false
	} else {
		self.Gcode.Respond_info("action:resumed", true)
	}
	self.Pause_command_sent = false
}

const Cmd_RESUME_help = "Resumes the print from a pause"

func (self *PauseResume) Cmd_RESUME(gcmd interface{}) error {
	if !self.Is_paused {
		gcmd.(*GCodeCommand).Respond_info("Print is not paused, resume aborted", true)
		return nil
	}
	self.resume_callback()
	zero := 0.
	velocity := gcmd.(*GCodeCommand).Get_float("VELOCITY", self.Recover_velocity, &zero, &zero, &zero, &zero)
	log.Println("Cmd_RESUME")
	toolhead := self.Printer.Lookup_object("toolhead", object.Sentinel{}).(*Toolhead)
	move := NewMove(toolhead, toolhead.Commanded_pos, []float64{0.0, 0.0, 0.0, 0.0}, 5.0)
	if move.Is_kinematic_move {
		err := func() (err1 interface{}) {
			defer func() {
				if e := recover(); e != nil {
					err1 = e
				}
			}()
			toolhead.Kin.Check_move(move)
			return nil
		}()
		if err == nil {
			self.Gcode.Run_script_from_command(fmt.Sprintf("RESTORE_GCODE_STATE NAME=PAUSE_STATE MOVE=1 MOVE_SPEED=%.4f", velocity))
		}
	} else {
		self.Gcode.Run_script_from_command(fmt.Sprintf("RESTORE_GCODE_STATE NAME=PAUSE_STATE MOVE=1 MOVE_SPEED=%.4f", velocity))
	}
	self.Send_resume_command()
	self.Is_paused = false
	return nil
}

const Cmd_CLEAR_PAUSE_help = "Clears the current paused state without resuming the print"

func (self *PauseResume) Cmd_CLEAR_PAUSE(gcmd interface{}) error {
	self.Is_paused = false
	self.Pause_command_sent = false
	return nil
}

const Cmd_CANCEL_PRINT_help = "Cancel the current print"

func (self *PauseResume) Cmd_CANCEL_PRINT(gcmd interface{}) error {
	if self.Is_sd_active() || self.Sd_paused {
		self.V_sd.Do_cancel()
	} else {
		gcmd.(*GCodeCommand).Respond_info("action:cancel", true)
	}
	self.Cmd_CLEAR_PAUSE(gcmd)
	// 取消断电续打记录
	virtualSD := self.Printer.Lookup_object("virtual_sdcard", object.Sentinel{}).(*VirtualSD)
	virtualSD.Cancel_record()
	return nil
}

var CMD_M600_HELP = "The M600 command initiates the filament change procedure."

func (p *PauseResume) Cmd_M600(gcmd interface{}) error {
	if p.Is_paused {
		gcmd.(*GCodeCommand).Respond_info("Print already paused", true)
		return nil
	}
	printStats := p.Printer.Lookup_object("print_stats", object.Sentinel{}).(*PrintStats)
	// 升温中不执行坐标保存和移动逻辑
	p.Send_pause_command()
	p.Gcode.Run_script_from_command("SAVE_GCODE_STATE NAME=PAUSE_STATE")
	p.Is_paused = true
	params := gcmd.(*GCodeCommand).Get_command_parameters()

	params_x, params_y, params_z, params_e := handParams(params, p)

	if is_homing == 2 {
		p.Gcode.Run_script_from_command("G91")
		p.Gcode.Run_script_from_command(fmt.Sprintf("G1 E-%f F1800", params_e))

		p.Gcode.Run_script_from_command(fmt.Sprintf("G1 Z%f F1800", params_z))
		p.Gcode.Run_script_from_command("G90")

		p.Gcode.Run_script_from_command(fmt.Sprintf("G1 X%f Y%f F3600", params_x, params_y))
		p.Gcode.Run_script_from_command("G91")
		p.Gcode.Run_script_from_command(fmt.Sprintf("G1 E-%f F1800", p.params_u))
		p.Gcode.Run_script_from_command("G90")

	}
	// self.Printer.reactor.Pause(self.Printer.reactor.Monotonic() + 5.0)
	printStats.state = Pause_
	p.hasM600 = true
	for i := 0; i < int(p.params_b) && p.bee_path != ""; i++ {
		// 蜂鸣器 "echo 1 > /sys/class/pwm/pwmchip0/pwm0/enable && usleep 50000 && echo 0 > /sys/class/pwm/pwmchip0/pwm0/enable"
		// "/sys/class/pwm/pwmchip0/pwm0/enable"
		beeper, err := os.OpenFile(p.bee_path, os.O_WRONLY, 0666)
		if err != nil {
			value.StaticValue.Debug.Println(err)
			return nil
		}
		defer beeper.Close()
		_, err = beeper.WriteString("1")
		if err != nil {
			value.StaticValue.Debug.Println(err)
		}
		p.Printer.reactor.Pause(p.Printer.reactor.Monotonic() + 0.05)
		_, err = beeper.WriteString("0")
		if err != nil {
			value.StaticValue.Debug.Println(err)
		}
		p.Printer.reactor.Pause(p.Printer.reactor.Monotonic() + 0.01)
	}
	return nil
}

func handParams(params map[string]string, p *PauseResume) (float64, float64, float64, float64) {
	params_x := 0.0
	if x, err := strconv.ParseFloat(params["X"], 64); err == nil {
		params_x = x
	} else {
		params_x = p.pause_position[0]
	}

	params_y := 0.0
	if y, err := strconv.ParseFloat(params["Y"], 64); err == nil {
		params_y = y
	} else {
		params_y = p.pause_position[1]
	}

	params_z := 0.0
	if z, err := strconv.ParseFloat(params["Z"], 64); err == nil {
		params_z = z
	} else {
		params_z = p.z_up
	}

	params_e := 0.0
	if e, err := strconv.ParseFloat(params["E"], 64); err == nil {
		params_e = e
	} else {
		params_e = p.extru_pull
	}

	if l, err := strconv.ParseFloat(params["L"], 64); err == nil {
		p.params_l = l
	} else {
		p.params_l = p.extru_load
	}

	if u, err := strconv.ParseFloat(params["U"], 64); err == nil {
		p.params_u = u
	} else {
		p.params_u = p.extru_unload
	}
	if b, err := strconv.ParseInt(params["B"], 10, 64); err == nil {
		p.params_b = b
	} else {
		p.params_b = 1
	}
	return params_x, params_y, params_z, params_e
}

func (p *PauseResume) resume_callback() {
	if p.hasM600 {
		p.Gcode.Run_script_from_command("G91")
		p.Gcode.Run_script_from_command(fmt.Sprintf("G1 E%f F180", p.params_l))
		p.Gcode.Run_script_from_command("G1 E2 F1800")
		p.Gcode.Run_script_from_command("G90")
	}
	p.hasM600 = false

	p.V_sd.Do_pre_resume()
}

func Load_config_pause_resume(config *ConfigWrapper) interface{} {
	return NewPauseResume(config)
}
