// Support for a heated bed
//
// Copyright (C) 2018-2019  Kevin O"Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
package project

import (
	"k3c/common/utils/object"
)

type PrinterHeaterBed struct {
	Printer    *Printer
	Heater     *Heater
	Get_status func(eventtime float64) map[string]float64
	Stats      func(eventtime float64) (bool, string)
}

func NewPrinterHeaterBed(config *ConfigWrapper) *PrinterHeaterBed {
	var self = &PrinterHeaterBed{}
	self.Printer = config.Get_printer()
	var pheaters = self.Printer.Load_object(config, "heaters", object.Sentinel{}).(*PrinterHeaters)
	self.Heater = pheaters.Setup_heater(config, "B")
	self.Get_status = self.Heater.Get_status
	self.Stats = self.Heater.Stats
	// Register commands
	var gcode = self.Printer.Lookup_object("gcode", object.Sentinel{})
	//if err != nil {
	//	value.StaticValue.Error.Println(err)
	//}
	gcode.(*GCodeDispatch).Register_command("M140", self.Cmd_M140, false, "")
	gcode.(*GCodeDispatch).Register_command("M190", self.Cmd_M190, false, "")

	wh := MustLookupWebhooks(self.Printer)
	wh.Register_endpoint(
		"temp/heaterbed_temp", self.setHeaterBedTemp)
	return self
}

func (p *PrinterHeaterBed) setHeaterBedTemp(web_request *WebRequest) (interface{}, error) {
	temp := web_request.get_float("temp", object.Sentinel{})
	p.Heater.Set_temp(temp)
	return nil, nil
}

func (self *PrinterHeaterBed) Cmd_M140(argv interface{}) error {
	gcmd := argv.(*GCodeCommand)
	return self.setTemperature(gcmd, false)
}

func (self *PrinterHeaterBed) Cmd_M190(argv interface{}) error {
	gcmd := argv.(*GCodeCommand)
	return self.setTemperature(gcmd, true)
}

func (self *PrinterHeaterBed) setTemperature(gcmd *GCodeCommand, wait bool) error {
	// Set Bed Temperature
	var temp = gcmd.Get_float("S", 0., nil, nil, nil, nil)
	var pheaters = self.Printer.Lookup_object("heaters", object.Sentinel{})
	//if err != nil {
	//	value.StaticValue.Error.Println(err)
	//}
	return pheaters.(*PrinterHeaters).Set_temperature(self.Heater, temp, wait, 0)
}

type IPrinterHeaterBed interface {
	load_config(config *ConfigWrapper) *PrinterHeaterBed
}

func Load_config_Heater_bed(config *ConfigWrapper) interface{} {
	return NewPrinterHeaterBed(config)
}
