package project

import (
	"k3c/common/utils/object"
)

const LED_OUTPIN_MIN_TIME = 0.100

type LedDigitalOut struct {
	printer    *Printer
	gcode      *GCodeDispatch
	led_pin1   *MCU_digital_out
	led_pin2   *MCU_digital_out
	last_value int
	reactor    IReactor
}

func NewLedDigitalOut(config *ConfigWrapper) *LedDigitalOut {
	self := &LedDigitalOut{}
	self.printer = config.Get_printer()
	self.reactor = self.printer.Get_reactor()
	self.gcode = self.printer.Lookup_object("gcode", object.Sentinel{}).(*GCodeDispatch)
	ppins := self.printer.Lookup_object("pins", object.Sentinel{}).(*PrinterPins)
	self.last_value = config.Getint("value", 1, 0, 0, true)
	self.led_pin1 = ppins.Setup_pin("digital_out", config.Get("pin1", "", true).(string)).(*MCU_digital_out)
	self.led_pin2 = ppins.Setup_pin("digital_out", config.Get("pin2", "", true).(string)).(*MCU_digital_out)
	self.led_pin1.Setup_max_duration(0.)
	self.led_pin2.Setup_max_duration(0.)
	self.gcode.Register_command("M1011", self.Cmd_M1011, false, "")

	wh := MustLookupWebhooks(self.printer)
	wh.Register_endpoint(
		"led/set_led", self.SetLed)
	return self
}

func (self *LedDigitalOut) SetLed(web_request *WebRequest) (interface{}, error) {

	led_status := int(web_request.get_float("S", object.Sentinel{}))
	if 0 == led_status {
		self._set_pin1(led_status)
		self._set_pin2(led_status)
	} else if 1 == led_status {
		self._set_pin1(led_status)
		self._set_pin2(led_status)
	}
	return nil, nil
}

func (self *LedDigitalOut) _set_pin1(value int) {
	curtime := self.reactor.Monotonic()
	est_time := self.led_pin1.Get_mcu().Estimated_print_time(curtime)
	next_cmd_time := est_time + OUTPIN_MIN_TIME
	self.led_pin1.Set_digital(next_cmd_time, value)
}

func (self *LedDigitalOut) _set_pin2(value int) {
	curtime := self.reactor.Monotonic()
	est_time := self.led_pin2.Get_mcu().Estimated_print_time(curtime)
	next_cmd_time := est_time + OUTPIN_MIN_TIME
	self.led_pin2.Set_digital(next_cmd_time, value)
}

func (self *LedDigitalOut) Cmd_M1011(argv interface{}) error {
	gcmd := argv.(*GCodeCommand)
	led_status := gcmd.Get_int("S", 0, nil, nil)
	if 0 == led_status {
		self._set_pin1(led_status)
		self._set_pin2(led_status)
	} else if 1 == led_status {
		self._set_pin1(led_status)
		self._set_pin2(led_status)
	}
	return nil
}

func Load_config_prefix_LedOutPut(config *ConfigWrapper) interface{} {
	return NewLedDigitalOut(config)
}
