package project

import (
	"fmt"
	"k3c/common/utils/object"
	"k3c/common/value"
	"strconv"
	"strings"
)

type MarcoException struct {
	msg string
}

func (e *MarcoException) Error() string {
	return e.msg
}

type PrinterMarco struct {
	printer    *Printer
	gcode      *GCodeDispatch
	cancelFlag bool
}

func NewPrinterMarco(config *ConfigWrapper) interface{} {
	p := &PrinterMarco{}
	p.printer = config.printer
	p.gcode = p.printer.Lookup_object("gcode", object.Sentinel{}).(*GCodeDispatch)
	p.gcode.Register_command("G9111", p.Cmd_G9111, false, "")
	p.gcode.Register_command("G9112", p.Cmd_G9112, false, "")

	p.printer.Register_event_handler("printer_macro:canel", p.CancelEvent)
	p.printer.Register_event_handler("printer_macro:reset", p.ResetEvent)
	return p
}

func (p *PrinterMarco) Cmd_G9111(arg interface{}) error {
	defer func() {
		p.cancelFlag = false
		if err := recover(); err != nil {
			if e, ok := err.(*MarcoException); ok {
				value.StaticValue.Debug.Println(e.msg)
			} else {
				panic(err)
			}
		}
	}()
	gcmd := arg.(*GCodeCommand)
	params := getKeyValueParams(gcmd.Get_commandline())
	extruderTemp := getParam(params, "EXTRUDERTEMP", 220)
	bedTemp := getParam(params, "BEDTEMP", 60)
	leviq3 := p.printer.Lookup_object("leviQ3", object.Sentinel{}).(*LeviQ3Helper)
	wipeTemp := getParam(params, "WIPETEMP", leviq3.extru_temp)

	p.runGcode(fmt.Sprintf("M104 S%f I1", wipeTemp))
	p.runGcode(fmt.Sprintf("M140 S%f I1", bedTemp))
	p.runGcode("HOME_XY")
	p.runGcode(fmt.Sprintf("M109 S%f I1", wipeTemp))
	p.runGcode("G9112")
	p.runGcode("MOVE_HEAT_POS")
	p.runGcode(fmt.Sprintf("M109 S%f I1", extruderTemp))
	p.runGcode("UNDERLINE")

	return nil
}

func (p *PrinterMarco) Cmd_G9112(arg interface{}) error {
	defer func() {
		p.cancelFlag = false
	}()
	p.runGcode("G90")
	p.runGcode("G28 W")
	p.runGcode("M106 S255")
	p.runGcode("WIPE_NOZZLE")
	return nil
}

func (p *PrinterMarco) CancelEvent(arg []interface{}) error {
	p.cancelFlag = true
	return nil
}

func (p *PrinterMarco) ResetEvent(arg []interface{}) error {
	p.cancelFlag = false
	return nil
}

func (p *PrinterMarco) runGcode(cmd string) {
	if p.cancelFlag {
		panic(&MarcoException{"Instruction cancellation"})
	}
	p.gcode.Run_script_from_command(cmd)
}

func getKeyValueParams(cmdLine string) map[string]float64 {
	params := make(map[string]float64)
	arr := strings.Split(cmdLine, " ")
	for i := 1; i < len(arr); i++ {
		line := strings.Trim(arr[i], " ")
		if line != "" {
			strs := strings.Split(line, "=")
			k := strings.ToUpper(strings.Trim(strs[0], " "))
			v, err := strconv.ParseFloat(strings.Trim(strs[1], " "), 64)
			if err != nil {
				panic(err)
			}
			params[k] = v
		}
	}
	return params
}

func getParam(params map[string]float64, key string, defalut float64) float64 {
	value := defalut
	if v, ok := params[key]; ok {
		value = v
	}
	return value
}

func Load_config_printer_marco(config *ConfigWrapper) interface{} {
	return NewPrinterMarco(config)
}
