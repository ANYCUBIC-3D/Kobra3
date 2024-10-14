package project

import (
	"fmt"
	"k3c/common/constants"
	"k3c/common/utils/object"
	"k3c/common/value"
	"log"
	"strings"
)

type RunoutHelper struct {
	name              string
	printer           *Printer
	reactor           IReactor
	gcode             *GCodeDispatch
	runout_pause      bool
	runout_gcode      interface{}
	insert_gcode      interface{}
	pause_delay       float64
	event_delay       float64
	min_event_systime float64
	filament_present  int
	sensor_enabled    bool
}

func NewRunoutHelper(config *ConfigWrapper) *RunoutHelper {
	self := RunoutHelper{}
	self.name = strings.Split(config.Get_name(), " ")[len(strings.Split(config.Get_name(), " "))-1]
	self.printer = config.Get_printer()
	self.reactor = self.printer.Get_reactor()
	gcode := self.printer.Lookup_object("gcode", object.Sentinel{})
	self.gcode = gcode.(*GCodeDispatch)
	// Read config
	self.runout_pause = config.Getboolean("pause_on_runout", true, true)
	if self.runout_pause {
		self.printer.Load_object(config, "pause_resume", object.Sentinel{})
	}
	self.runout_gcode, self.insert_gcode = nil, nil
	gcode_macro := self.printer.Load_object(config, "gcode_macro_1", object.Sentinel{}).(*PrinterGCodeMacro)
	if self.runout_pause || config.Get("runout_gcode", nil, true) != nil {
		self.runout_gcode = gcode_macro.Load_template(config, "runout_gcode", "")
	}
	if config.Get("insert_gcode", nil, true) != nil {
		self.insert_gcode = gcode_macro.Load_template(config, "insert_gcode", "")
	}
	self.pause_delay = config.Getfloat("pause_delay", 0.5, 0, 0, 0.0, 0, true)
	self.event_delay = config.Getfloat("event_delay", 3.0, 0, 0, 0.0, 0, true)
	// Internal state
	self.min_event_systime = constants.NEVER
	self.filament_present = 0
	self.sensor_enabled = true
	// Register commands and event handlers
	self.printer.Register_event_handler("project:ready", self._handle_ready)
	self.gcode.Register_mux_command(
		"QUERY_FILAMENT_SENSOR", "SENSOR", self.name,
		self.Cmd_QUERY_FILAMENT_SENSOR,
		self.Cmd_QUERY_FILAMENT_SENSOR_help())
	self.gcode.Register_mux_command(
		"SET_FILAMENT_SENSOR", "SENSOR", self.name,
		self.Cmd_SET_FILAMENT_SENSOR,
		self.Cmd_SET_FILAMENT_SENSOR_help())
	wh := MustLookupWebhooks(self.printer)
	wh.Register_endpoint("filament/query", self.QUERY_FILAMENT_SENSOR)
	wh.Register_endpoint("filament/set", self.SET_FILAMENT_SENSOR)
	wh.Register_endpoint("filament/set_sensor", self.set_filament_sensor)

	return &self
}

func (self *RunoutHelper) _handle_ready([]interface{}) error {
	self.min_event_systime = self.reactor.Monotonic() + 2.
	return nil
}

func (self *RunoutHelper) _runout_event_handler(aa interface{}) interface{} {
	eventTime := aa.(float64)
	// Pausing from inside an event requires that the pause portion
	// of pause_resume execute immediately.
	pause_prefix := ""
	if self.runout_pause {
		//pauseResume := self.printer.Lookup_object("pause_resume", object.Sentinel{})
		//pauseResume.(*PauseResume).Send_pause_command()
		pause_prefix = "PAUSE\n"
		self.printer.Get_reactor().Pause(eventTime + self.pause_delay)
	}
	self._exec_gcode(pause_prefix, self.runout_gcode)
	return nil
}

func (self *RunoutHelper) _insert_event_handler(interface{}) interface{} {
	self._exec_gcode("", self.insert_gcode)
	return nil
}

func (self *RunoutHelper) _exec_gcode(prefix string, template interface{}) {

	script, err := template.(*TemplateWrapper).Render(nil)
	if err != nil {
		value.StaticValue.Error.Println("Script running error")
	}
	self.gcode.Run_script(prefix + script + "\nM400")

	self.min_event_systime = self.reactor.Monotonic() + self.event_delay
}

func (self *RunoutHelper) Note_filament_present(is_filament_present int) {
	if is_filament_present == self.filament_present {
		return
	}
	self.filament_present = is_filament_present
	eventtime := self.reactor.Monotonic()
	if eventtime < self.min_event_systime || !self.sensor_enabled {
		// do not process during the initialization time, duplicates,
		// during the event delay time, while an event is running, or
		// when the sensor is disabled
		return
	}
	// Determine "printing" status
	is_printing := false
	idle_timeout := self.printer.Lookup_object("idle_timeout", nil)
	if idle_timeout != nil {
		is_printing = idle_timeout.(*IdleTimeout).Get_status(eventtime)["state"] == "Printing"
	} else {
		print_stats := self.printer.Lookup_object("print_stats", object.Sentinel{})
		is_printing = print_stats.(*PrintStats).state == Printing_
	}
	// Perform filament action associated with status change (if any)
	if is_filament_present == 1 {
		if !is_printing && self.insert_gcode != nil {
			// insert detected
			self.min_event_systime = constants.NEVER
			log.Printf("Filament Sensor %s: insert event detected, Time %.2f", self.name, eventtime)
			self.reactor.Register_callback(self._insert_event_handler, constants.NOW)
		}
	} else if is_printing && self.runout_gcode != nil {
		// runout detected
		self.min_event_systime = constants.NEVER
		log.Printf("Filament Sensor %s: runout event detected, Time %.2f", self.name, eventtime)
		self.reactor.Register_callback(self._runout_event_handler, constants.NOW)
	}
}

func (self *RunoutHelper) Get_status(eventtime float64) map[string]interface{} {
	var sensorEnabled int
	if self.sensor_enabled {
		sensorEnabled = 1
	}
	return map[string]interface{}{
		"name":              self.name,
		"filament_detected": self.filament_present,
		"enabled":           sensorEnabled,
	}
}

func (self *RunoutHelper) Cmd_QUERY_FILAMENT_SENSOR_help() string {
	return "Query the status of the Filament Sensor"
}

func (self *RunoutHelper) Cmd_QUERY_FILAMENT_SENSOR(argv interface{}) error {
	gcmd := argv.(*GCodeCommand)

	var msg string
	if self.filament_present == 1 {
		msg = fmt.Sprintf("Filament Sensor %s: filament detected", self.name)
	} else {
		msg = fmt.Sprintf("Filament Sensor %s: filament not detected", self.name)
	}
	gcmd.Respond_info(msg, true)
	return nil
}

func (self *RunoutHelper) QUERY_FILAMENT_SENSOR(web_request *WebRequest) (interface{}, error) {

	var msg string
	self.name = web_request.get_str("SENSOR", object.Sentinel{})
	if self.filament_present == 1 {
		msg = fmt.Sprintf("Filament Sensor %s: filament detected", self.name)
	} else {
		msg = fmt.Sprintf("Filament Sensor %s: filament not detected", self.name)
	}
	web_request.Send(msg)
	return nil, nil
}

func (self *RunoutHelper) Cmd_SET_FILAMENT_SENSOR_help() string {
	return "Sets the filament sensor on/off"
}

func (self *RunoutHelper) Cmd_SET_FILAMENT_SENSOR(argv interface{}) error {
	gcmd := argv.(*GCodeCommand)
	self.sensor_enabled = gcmd.Get_int("ENABLE", 1, nil, nil) > 0
	return nil
}

func (self *RunoutHelper) SET_FILAMENT_SENSOR(web_request *WebRequest) (interface{}, error) {
	self.name = web_request.get_str("SENSOR", object.Sentinel{})
	self.sensor_enabled = web_request.get_float("ENABLE", object.Sentinel{}) > 0
	return nil, nil
}

func (self *RunoutHelper) set_filament_sensor(web_request *WebRequest) (interface{}, error) {
	self.sensor_enabled = web_request.get_float("enable", 0.0) > 0
	web_request.Send(nil)
	return nil, nil
}

type SwitchSensor struct {
	runout_helper *RunoutHelper
	Get_status    func(float64) map[string]interface{}
}

func NewSwitchSensor(config *ConfigWrapper) *SwitchSensor {
	self := SwitchSensor{}
	printer := config.Get_printer()
	buttons := printer.Load_object(config, "buttons", object.Sentinel{})
	switch_pin := config.Get("switch_pin", nil, true).(string)
	buttons.(*PrinterButtons).Register_buttons([]string{switch_pin}, self._button_handler)
	self.runout_helper = NewRunoutHelper(config)
	self.Get_status = self.runout_helper.Get_status
	return &self
}

func (self *SwitchSensor) _button_handler(eventtime float64, state int) {
	self.runout_helper.Note_filament_present(state)
}

func Load_config_SwitchSensor(config *ConfigWrapper) interface{} {
	return NewSwitchSensor(config)
}
