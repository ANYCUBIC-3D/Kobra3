package project

import (
	"fmt"
	"k3c/common/constants"
	"k3c/common/utils/object"
	"k3c/common/value"
	"math"
	"strings"
)

const HINT_THERMAL = `
See the 'verify_heater' section in docs/Config_Reference.md
for the parameters that control this check.
`

type HeaterCheck struct {
	printer            *Printer
	heater_name        string
	heater             *Heater
	hysteresis         float64
	max_error          float64
	heating_gain       float64
	check_gain_time    float64
	approaching_target bool
	starting_approach  bool
	last_target        float64
	goal_temp          float64
	error              float64
	goal_systime       float64
	check_timer        *ReactorTimer
	err                bool
}

func NewHeaterCheck(config *ConfigWrapper) *HeaterCheck {
	self := new(HeaterCheck)
	self.printer = config.Get_printer()
	self.printer.Register_event_handler("project:connect", self.handle_connect)
	self.printer.Register_event_handler("project:shutdown", self.handle_shutdown)
	self.heater_name = strings.Split(config.Get_name(), " ")[1] // assert len >=2
	self.heater = nil
	self.hysteresis = config.Getfloat("hysteresis", 5.0, 0, 0, 0, 0, true)
	self.max_error = config.Getfloat("max_error", 120.0, 0, 0, 0, 0, true)
	self.heating_gain = config.Getfloat("heating_gain", 2.0, 0, 0, 0, 0, true)
	default_gain_time := 20.0
	if self.heater_name == "heater_bed" {
		default_gain_time = 60.0
	}
	self.check_gain_time = config.Getfloat("check_gain_time", default_gain_time, 1.0, 0, 0, 0, true)
	self.approaching_target = false
	self.starting_approach = false

	self.last_target = 0
	self.goal_temp = 0
	self.error = 0

	// self.goal_systime = self.printer.get_reactor().NEVER
	self.goal_systime = constants.NEVER
	self.check_timer = nil
	self.err = false
	wh := MustLookupWebhooks(self.printer)
	wh.Register_endpoint("heater_check/check_error", self.handle_heater_check)
	return self
}

func (self *HeaterCheck) handle_connect([]interface{}) error {
	if _, ok := self.printer.Get_start_args()["debugoutput"]; ok {
		//  Disable verify_heater if outputting to a debug file
		return nil
	}

	pheaters := self.printer.Lookup_object("heaters", object.Sentinel{})
	//if err != nil {
	//	value.StaticValue.Error.Println("printer Lookup_object heaters error: ", err)
	//	return fmt.Errorf("printer Lookup_object heaters error: %w", err)
	//}

	pheater, ok := pheaters.(*PrinterHeaters)
	if !ok {
		value.StaticValue.Error.Printf("pheaters type is %T not *PrinterHeaters\n", pheater)
		return fmt.Errorf("pheaters type is %T not *PrinterHeaters", pheater)
	}
	self.heater = pheater.Lookup_heater(self.heater_name)
	value.StaticValue.Debug.Printf("Starting heater checks for %s", self.heater_name)
	reactor := self.printer.Get_reactor()
	self.check_timer = reactor.Register_timer(self.check_event, constants.NOW)
	return nil
}

func (self *HeaterCheck) handle_shutdown([]interface{}) error {
	if self.check_timer != nil {
		reactor := self.printer.Get_reactor()
		reactor.Update_timer(self.check_timer, constants.NEVER)
	}
	return nil
}

func (self *HeaterCheck) check_event(eventtime float64) float64 {
	temp, target := self.heater.Get_temp(eventtime)
	if temp >= target-self.hysteresis || target <= 0.0 {
		// Temperature near target - reset checks
		if self.approaching_target && target > 0.0 {
			value.StaticValue.Debug.Printf("Heater %s within range of %.3f", self.heater_name, target)
		}
		self.approaching_target = false
		self.starting_approach = false
		if temp <= target+self.hysteresis {
			self.error = 0.0
		}
		self.last_target = target
		return eventtime + 1.0
	}

	self.error += (target - self.hysteresis) - temp
	if !self.approaching_target {
		if target != self.last_target {
			// Target changed - reset checks
			value.StaticValue.Debug.Printf("Heater %s approaching new target of %.3f", self.heater_name, target)
			self.approaching_target = true
			self.starting_approach = true
			self.goal_temp = temp + self.heating_gain
			self.goal_systime = eventtime + self.check_gain_time
		} else if self.error >= self.max_error {
			// Failure due to inability to maintain target temperature
			return self.heater_fault()
		}
	} else if temp >= self.goal_temp {
		//  Temperature approaching target - reset checks
		self.starting_approach = false
		self.error = 0.0
		self.goal_temp = temp + self.heating_gain
		self.goal_systime = eventtime + self.check_gain_time
	} else if eventtime >= self.goal_systime {
		// Temperature is no longer approaching target
		self.approaching_target = false
		value.StaticValue.Debug.Printf("Heater %s no longer approaching target %.3f",
			self.heater_name, target)
	} else if self.starting_approach {
		self.goal_temp = math.Min(self.goal_temp, temp+self.heating_gain)
	}
	self.last_target = target
	return eventtime + 1.0
}

func (self *HeaterCheck) heater_fault() float64 {
	self.err = true
	msg := fmt.Sprintf("Heater %s not heating at expected rate", self.heater_name)
	value.StaticValue.Error.Println(msg)
	self.printer.Invoke_shutdown(msg + HINT_THERMAL)
	gcode := self.printer.Lookup_object("gcode", object.Sentinel{}).(*GCodeDispatch)
	gcode.Respond_info(msg, false)
	return constants.NEVER
}

func (self *HeaterCheck) handle_heater_check(web_request *WebRequest) (interface{}, error) {
	web_request.Send(self.Get_status(0))
	return nil, nil
}

func (self *HeaterCheck) Get_status(eventtime float64) map[string]interface{} {
	err_status := 0
	if self.err {
		err_status = 1
	}
	return map[string]interface{}{
		"name":       self.heater_name,
		"err_status": err_status,
	}
}
func (self *HeaterCheck) Stats(eventtime float64) (bool, string) {
	return false, fmt.Sprintf("HeaterCheck:heater_name=%s error_count=%f hysteresis=%v goal_systime=%f last_target=%f goal_temp=%f",
		self.heater_name, self.error, self.hysteresis, self.goal_systime, self.last_target, self.goal_temp)
}
func Load_config_verify_heater(config *ConfigWrapper) interface{} {
	return NewHeaterCheck(config)
}
