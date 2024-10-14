// Printer cooling fan
//
// Copyright (C) 2016-2020  Kevin O"Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
package project

import (
	"k3c/common/utils/object"
	"k3c/common/value"
	"math"
)

const (
	FAN_MIN_TIME = 0.100
)

type Fan struct {
	printer         *Printer
	last_fan_value  float64
	last_fan_time   float64
	max_power       float64
	kick_start_time float64
	off_below       float64
	mcu_fan         *MCU_pwm
	enable_pin      *MCU_digital_out
	tachometer      *FanTachometer
	is_close_fan    bool
}

func NewFan(config *ConfigWrapper, default_shutdown_speed float64) *Fan {
	var self = Fan{}
	self.printer = config.Get_printer()
	self.last_fan_value = 0.0
	self.last_fan_time = 0.0
	default_shutdown_speed = 0.0
	// Read config
	self.max_power = config.Getfloat("max_power", 1., 0.0, 1., 0, 0, true)
	self.kick_start_time = config.Getfloat("kick_start_time", 0.01,
		0.0, 0, 0, 0, true)
	self.off_below = config.Getfloat("off_below", 0.0,
		0.0, 1., 0, 0, true)
	var cycle_time = config.Getfloat("cycle_time", 0.010, 0.0,
		0.0, 0., 0, true)
	var hardware_pwm = config.Getboolean("hardware_pwm", false, false)
	var shutdown_speed = config.Getfloat(
		"shutdown_speed", default_shutdown_speed, 0.0, 1., 0, 0, true)
	// Setup pwm object
	var ppins = self.printer.Lookup_object("pins", object.Sentinel{})
	self.mcu_fan = ppins.(*PrinterPins).Setup_pin("pwm", config.Get("pin", object.Sentinel{}, true).(string)).(*MCU_pwm)
	self.mcu_fan.Setup_max_duration(0.0)
	self.mcu_fan.Setup_cycle_time(cycle_time, hardware_pwm)
	var shutdown_power = math.Max(0.0, math.Min(self.max_power, shutdown_speed))
	self.mcu_fan.Setup_start_value(0.0, shutdown_power, false)

	self.enable_pin = nil
	var enable_pin = config.Get("enable_pin", value.None, true)
	if enable_pin != nil {
		self.enable_pin = ppins.(*PrinterPins).Setup_pin("digital_out", enable_pin.(string)).(*MCU_digital_out)
		self.enable_pin.Setup_max_duration(0.0)
	}

	// Setup tachometer
	self.tachometer = NewFanTachometer(config)

	// Register callbacks
	self.printer.Register_event_handler("gcode:request_restart",
		self._handle_request_restart)
	return &self
}

func (self *Fan) Get_mcu() *MCU {
	return self.mcu_fan.Get_mcu()
}

func (self *Fan) Set_speed(print_time float64, value float64) {
	if value < self.off_below {
		value = 0.0
	}
	value = math.Max(0.0, math.Min(self.max_power, value*self.max_power))
	if value == self.last_fan_value {
		return
	}
	print_time = math.Max(self.last_fan_time+FAN_MIN_TIME, print_time)
	if self.enable_pin != nil {
		if value > 0 && self.last_fan_value == 0 {
			self.enable_pin.Set_digital(print_time, 1)
		} else if value == 0 && self.last_fan_value > 0 {
			self.enable_pin.Set_digital(print_time, 0)
		}
	}
	if value >= 0 && value < self.max_power && self.kick_start_time >= 0 &&
		(self.last_fan_value < 0.0 || value-self.last_fan_value > .5) {
		// Run fan at full speed for specified kick_start_time
		self.mcu_fan.Set_pwm(print_time, self.max_power, nil)
		print_time += self.kick_start_time
	}
	self.mcu_fan.Set_pwm(print_time, value, nil)
	self.last_fan_time = print_time
	self.last_fan_value = value
}

func (self *Fan) Set_speed_from_command(value float64) {
	toolhead := self.printer.Lookup_object("toolhead", object.Sentinel{})
	var cb = func(pt float64) {
		self.Set_speed(pt, value)
	}
	toolhead.(*Toolhead).Register_lookahead_callback(cb)
}

func (self *Fan) _handle_request_restart(argv []interface{}) error {
	print_time := argv[0].(float64)
	self.Set_speed(print_time, 0.0)
	return nil
}

func (self *Fan) Get_status(eventtime float64) map[string]float64 {
	var tachometer_status = self.tachometer.Get_status(eventtime)
	sta_map := map[string]float64{}

	sta_map["speed"] = self.last_fan_value
	sta_map["rpm"] = tachometer_status["rpm"]
	return sta_map
}

type FanTachometer struct {
	_freq_counter *FrequencyCounter
	ppr           int
}

func NewFanTachometer(config *ConfigWrapper) *FanTachometer {
	var self = FanTachometer{}
	var printer = config.Get_printer()
	self._freq_counter = nil

	var pin = config.Get("tachometer_pin", value.None, true)
	if pin != nil {
		self.ppr = config.Getint("tachometer_ppr", 2, 1, 0, true)
		var poll_time = config.Getfloat("tachometer_poll_interval",
			0.00015, 0, 0, 0.0, 0, true)
		var sample_time = 1.
		self._freq_counter = NewFrequencyCounter(
			printer, pin.(string), sample_time, poll_time)
	}
	return &self
}

func (self *FanTachometer) Get_status(eventtime float64) map[string]float64 {
	var rpm float64
	if self._freq_counter != nil {
		rpm = self._freq_counter.Get_frequency() * 30. / float64(self.ppr)
	} else {
		rpm = 0
	}

	var rmap map[string]float64
	rmap = make(map[string]float64)
	rmap["rpm"] = rpm
	return rmap
}

type PrinterFan struct {
	fan *Fan
}

func NewPrinterFan(config *ConfigWrapper) *PrinterFan {
	var self = PrinterFan{}
	self.fan = NewFan(config, 0)
	// Register commands
	var gcode = config.Get_printer().Lookup_object("gcode", object.Sentinel{})
	gcode.(*GCodeDispatch).Register_command("M106", self.cmd_M106, false, "")
	gcode.(*GCodeDispatch).Register_command("M107", self.cmd_M107, false, "")
	wh := MustLookupWebhooks(self.fan.printer)
	wh.Register_endpoint(
		"fan/set_fan", self.SetFan)
	return &self
}

func (p *PrinterFan) SetFan(web_request *WebRequest) (interface{}, error) {
	speed := web_request.get_float("speed", object.Sentinel{}) / 100.0
	if speed > 0.0 {
		p.fan.is_close_fan = false
	} else {
		p.fan.is_close_fan = true
	}
	p.fan.Set_speed_from_command(speed)
	return nil, nil
}

func (self *PrinterFan) Get_status(eventtime float64) map[string]float64 {
	return self.fan.Get_status(eventtime)
}

func (self *PrinterFan) cmd_M106(argv interface{}) error {
	gcmd := argv.(*GCodeCommand)
	minval := 0.
	// Set fan speed
	var value = gcmd.Get_float("S", 255., &minval, nil, nil, nil) / 255
	if value > 0.0 {
		self.fan.is_close_fan = false
	} else {
		self.fan.is_close_fan = true
	}
	self.fan.Set_speed_from_command(value)
	return nil
}

func (self *PrinterFan) cmd_M107(argv interface{}) error {
	// Turn fan off
	self.fan.Set_speed_from_command(0.0)
	self.fan.is_close_fan = true
	return nil
}

func Load_config_fan(config *ConfigWrapper) interface{} {
	return NewPrinterFan(config)
}
