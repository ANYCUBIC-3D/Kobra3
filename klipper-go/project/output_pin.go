package project

import (
	"fmt"
	"k3c/common/constants"
	"k3c/common/utils/object"
	"math"
	"strings"
)

const (
	//PIN_MIN_TIME = 0.100
	RESEND_HOST_TIME  = 0.300 + 0.100
	MAX_SCHEDULE_TIME = 5.0
)

type PrinterOutputPin struct {
	printer            *Printer
	is_pwm             bool
	mcu_pin            interface{}
	scale              float64
	last_cycle_time    float64
	last_print_time    float64
	reactor            IReactor
	resend_timer       *ReactorTimer
	resend_interval    float64
	last_value         float64
	shutdown_value     float64
	default_cycle_time float64
}

const cmd_SET_PIN_help = "Set the value of an output pin"

func NewPrinterOutputPin(config *ConfigWrapper) *PrinterOutputPin {
	self := &PrinterOutputPin{}
	self.printer = config.Get_printer()
	ppins := self.printer.Lookup_object("pins", object.Sentinel{}).(*PrinterPins) // Adjust the type assertion as needed
	self.is_pwm = config.Getboolean("pwm", false, true)

	if self.is_pwm {
		self.mcu_pin = ppins.Setup_pin("pwm", config.Get("pin", "", true).(string))
		cycle_time := config.Getfloat("cycle_time", 0.100, 0.0, MAX_SCHEDULE_TIME, 0., 0., true)
		hardware_pwm := config.Getboolean("hardware_pwm", false, true)

		self.mcu_pin.(*MCU_pwm).Setup_cycle_time(cycle_time, hardware_pwm)
		self.scale = config.Getfloat("scale", 1.0, 0.0, 0, 0, 0, true)
		self.last_cycle_time = cycle_time
		self.last_value = cycle_time
	} else {
		self.mcu_pin = ppins.Setup_pin("digital_out", config.Get("pin", "", true).(string))
		self.scale = 1.0
		self.last_cycle_time, self.default_cycle_time = 0., 0.
	}
	self.last_print_time = 0.
	static_value := config.Getfloat("static_value", math.NaN(),
		0., self.scale, 0, 0, true)
	self.reactor = self.printer.Get_reactor()
	self.resend_timer = nil
	self.resend_interval = 0.
	if math.IsNaN(static_value) == false {
		self.last_value = static_value / self.scale
		if _, ok := self.mcu_pin.(*MCU_pwm); ok {
			self.mcu_pin.(*MCU_pwm).Setup_max_duration(0.)
			self.mcu_pin.(*MCU_pwm).Setup_start_value(
				self.last_value, self.last_value, true)
		} else if _, ok := self.mcu_pin.(*MCU_digital_out); ok {
			self.mcu_pin.(*MCU_digital_out).Setup_max_duration(0.)
			self.mcu_pin.(*MCU_digital_out).Setup_start_value(
				self.last_value, self.last_value, true)
		}
	} else {
		max_mcu_duration := config.Getfloat("maximum_mcu_duration", 0.,
			0.500, MAX_SCHEDULE_TIME, 0, 0, true)

		if max_mcu_duration > 0 {
			self.resend_interval = max_mcu_duration - RESEND_HOST_TIME
		}

		self.last_value = config.Getfloat(
			"value", 0., 0., self.scale, 0, 0, true) / self.scale
		self.shutdown_value = config.Getfloat(
			"shutdown_value", 0., 0., self.scale, 0, 0, true) / self.scale
		if _, ok := self.mcu_pin.(*MCU_pwm); ok {
			self.mcu_pin.(*MCU_pwm).Setup_max_duration(max_mcu_duration)
			self.mcu_pin.(*MCU_pwm).Setup_start_value(
				self.last_value, self.shutdown_value, false)
		} else if _, ok := self.mcu_pin.(*MCU_digital_out); ok {
			self.mcu_pin.(*MCU_digital_out).Setup_max_duration(max_mcu_duration)
			self.mcu_pin.(*MCU_digital_out).Setup_start_value(
				self.last_value, self.shutdown_value, false)
		}

		pin_name := strings.Split(config.Get_name(), " ")[1]
		gcode := self.printer.Lookup_object("gcode", object.Sentinel{}).(*GCodeDispatch)
		gcode.Register_mux_command("SET_PIN", "PIN", pin_name, self.cmd_SET_PIN,
			cmd_SET_PIN_help)
	}

	return self
}

func (self *PrinterOutputPin) Get_status(eventTime float64) map[string]float64 {
	return map[string]float64{"value": self.last_value}
}

func (self *PrinterOutputPin) _set_pin(print_time float64, value float64, cycle_time float64, is_resend bool) {
	if value == self.last_value && cycle_time == self.last_cycle_time {
		if !is_resend {
			return
		}
	}
	print_time = math.Max(print_time, self.last_print_time+PIN_MIN_TIME)
	if self.is_pwm {
		self.mcu_pin.(*MCU_pwm).Set_pwm(print_time, value, &cycle_time)
	} else {
		self.mcu_pin.(*MCU_digital_out).Set_digital(print_time, int(value))
	}
	self.last_value = value
	self.last_cycle_time = cycle_time
	self.last_print_time = print_time
	if self.resend_interval > 0 && self.resend_timer == nil {
		self.resend_timer = self.reactor.Register_timer(self._resend_current_val, constants.NOW)
	}
}

func (self *PrinterOutputPin) cmd_SET_PIN(argv interface{}) error {
	gcmd := argv.(*GCodeCommand)
	zero := 0.0
	value := gcmd.Get_float("VALUE", 0.0, &zero, &self.scale, nil, nil)
	value /= self.scale
	cycle_time := gcmd.Get_float("CYCLE_TIME", self.last_cycle_time, nil, nil, nil, nil)
	if !self.is_pwm && (value != 0.0 && value != 1.0) {
		return fmt.Errorf("Invalid pin value")
	}

	toolhead := self.printer.Lookup_object("toolhead", object.Sentinel{}).(*Toolhead) // Adjust the type assertion as needed
	toolhead.Register_lookahead_callback(func(print_time float64) {
		self._set_pin(print_time, value, cycle_time, false)
	})

	return nil
}

func (self *PrinterOutputPin) _resend_current_val(eventTime float64) float64 {
	if self.last_value == self.shutdown_value {
		self.reactor.Unregister_timer(self.resend_timer)
		self.resend_timer = nil
		return constants.NEVER
	}

	systime := self.reactor.Monotonic()
	print_time := 0.0
	if _, ok := self.mcu_pin.(*MCU_pwm); ok {
		print_time = self.mcu_pin.(*MCU_pwm).Get_mcu().Estimated_print_time(systime)
	} else if _, ok := self.mcu_pin.(*MCU_digital_out); ok {
		print_time = self.mcu_pin.(*MCU_digital_out).Get_mcu().Estimated_print_time(systime)
	}

	timeDiff := (self.last_print_time + self.resend_interval) - print_time
	if timeDiff > 0.0 {
		// Reschedule for resend time
		return systime + timeDiff
	}
	self._set_pin(print_time+PIN_MIN_TIME, self.last_value, self.last_cycle_time, true)
	return systime + self.resend_interval
}

func Load_config_prefix_OutputPin(config *ConfigWrapper) interface{} {
	return NewPrinterOutputPin(config)
}
