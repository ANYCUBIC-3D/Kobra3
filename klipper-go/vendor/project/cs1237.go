package project

import (
	"fmt"
	"k3c/common/errors"
	"k3c/common/utils/cast"
	"k3c/common/utils/maths"
	"k3c/common/utils/object"
	"k3c/common/value"
	"k3c/project/chelper"
)

const (
	SELF_CHECK_STATE = 1 << iota
	SCRATCH_STATE
	BLOCK_FILAMENT_STATE
	HEAD_BLOCK_STATE
	SELF_CHECK_ERR       = SELF_CHECK_STATE | 0x80
	SCRATCH_STATE_ERR    = SCRATCH_STATE | 0x80
	BLOCK_FILAMENT_ERR   = BLOCK_FILAMENT_STATE | 0x80
	HEAD_BLOCK_STATE_ERR = HEAD_BLOCK_STATE | 0x80
)

type cs1237 struct {
	dout_pin                   interface{}
	sclk_pin                   interface{}
	register                   int
	sensitivity                int
	samples_count              int
	data_bit                   int
	mcu                        *MCU
	oid                        int
	trdispatch                 interface{}
	trsyncs                    []*MCU_trsync
	reset_cs_cmd               *CommandWrapper
	cs_report_cmd              *CommandWrapper
	query_cs_diff              *CommandWrapper
	checkself_cs1237           *CommandWrapper
	printer                    *Printer
	adc_value                  int64
	raw_value                  int64
	sensor_state               int64
	checkself_flag             int64
	report                     bool
	query_comple               *ReactorCompletion
	head_block_sensitivity     int
	scratch_sensitivity        int
	self_check_sensitivity     int
	block_filament_sensitivity int
	boot_up_checkself          *CommandQueryWrapper
}

func NewCs1237(config *ConfigWrapper) *cs1237 {
	c := &cs1237{}
	c.register = config.Getint("register", object.Sentinel{}, 0, 0, true)
	c.sensitivity = config.Getint("sensitivity", object.Sentinel{}, 0, 0, true)
	c.data_bit = config.Getint("data_bit", object.Sentinel{}, 0, 0, true)
	c.samples_count = config.Getint("samples_count", object.Sentinel{}, 0, 0, true)
	c.head_block_sensitivity = config.Getint("head_block_sensitivity", object.Sentinel{}, -400000, -300000, true)
	c.scratch_sensitivity = config.Getint("scratch_sensitivity", object.Sentinel{}, -200000, -100000, true)
	c.self_check_sensitivity = config.Getint("self_check_sensitivity", object.Sentinel{}, -500, -400, true)
	c.block_filament_sensitivity = config.Getint("block_filament_sensitivity", object.Sentinel{}, -10000, -3000, true)

	ppins := MustLookupPins(config.Get_printer())
	dout_pin := cast.ToString(config.Get("dout_pin", object.Sentinel{}, true))
	dout_pin_params := ppins.Lookup_pin(dout_pin, true, true, nil)
	c.dout_pin = dout_pin_params["pin"]

	sclk_pin := cast.ToString(config.Get("sclk_pin", object.Sentinel{}, true))
	sclk_pin_params := ppins.Lookup_pin(sclk_pin, true, true, nil)

	c.sclk_pin = sclk_pin_params["pin"]
	c.mcu = sclk_pin_params["chip"].(*MCU)
	c.trdispatch = chelper.Trdispatch_alloc()
	c.trsyncs = []*MCU_trsync{NewMCU_trsync(c.mcu, c.trdispatch)}
	c.printer = config.Get_printer()
	c.mcu.Register_config_callback(c.Build_config)
	gcode := c.printer.Lookup_object("gcode", object.Sentinel{}).(*GCodeDispatch)
	gcode.Register_command("G9121", c.Cmd_G9121, false, "")
	gcode.Register_command("G9122", c.Cmd_G9122, false, "")
	gcode.Register_command("G9123", c.Cmd_G9123, false, "")
	gcode.Register_command("CS1237_DUMP", c.Cmd_CS1237_DUMP, false, "debug CS1237 DUMP")
	c.printer.Register_event_handler("cs1237:self_check",
		c.cs1237_self_check)
	//c.printer.Register_event_handler("project:ready", c._handle_ready)
	return c
}

func (c *cs1237) Build_config() {
	c.oid = c.mcu.Create_oid()
	value.StaticValue.Debug.Printf("oid : %d oid_count: %d ", c.oid, c.mcu._oid_count)
	// 发送应变片参数
	c.mcu.Add_config_cmd(fmt.Sprintf("config_cs1237 oid=%d dout_pin=%s sclk_pin=%s register=%d sensitivity=%d samples_count=%d data_bit=%d",
		c.oid, c.dout_pin, c.sclk_pin, c.register, c.sensitivity, c.samples_count, c.data_bit), false, false)
	c.reset_cs_cmd, _ = c.mcu.Lookup_command("reset_cs1237 oid=%c count=%c", c.trsyncs[0].Get_command_queue())
	c.cs_report_cmd, _ = c.mcu.Lookup_command("start_cs1237_report oid=%c enable=%c ticks=%c print_state=%c sensitivity=%c", c.trsyncs[0].Get_command_queue())

	//c.query_cmd, _ = c.mcu.Lookup_command("query_cs1237_adc oid=%c", c.trsyncs[0].Get_command_queue())
	c.mcu.Register_response(c.cs1237_query_handle, "cs1237_state", c.oid)

	c.query_cs_diff, _ = c.mcu.Lookup_command("query_cs1237_diff oid=%c", c.trsyncs[0].Get_command_queue())
	c.mcu.Register_response(c.cs1237_diff_handle, "cs1237_diff", c.oid)

	c.checkself_cs1237, _ = c.mcu.Lookup_command("checkself_cs1237 oid=%c write=%c", c.trsyncs[0].Get_command_queue())
	c.mcu.Register_response(c.checkself_cs1237_handle, "cs1237_checkself_flag", c.oid)

	c.boot_up_checkself = c.mcu.Lookup_query_command(
		"start_selfcheck_task oid=%c",
		"selfcheck_state oid=%c state=%c",
		c.oid, c.trsyncs[0].Get_command_queue(), false)
}

func (c *cs1237) _handle_ready([]interface{}) error {
	params := c.boot_up_checkself.Send([]int64{int64(c.oid)}, 0, 0)
	if params != nil {
		state := params.(map[string]interface{})["state"].(int)
		if state == 0 {
			c.printer.Invoke_shutdown("cs1237 boot up checkself failed")
		}
	}
	return nil
}

func (c *cs1237) Cmd_CS1237_DUMP(arg interface{}) error {
	gcmd, ok := arg.(*GCodeCommand)
	if !ok {
		return nil
	}
	min_i := 0
	max_i := 1
	e := gcmd.Get_int("E", 0.0, &min_i, &max_i)
	min := 0.1
	max := 1.0
	t := gcmd.Get_float("T", 1, &min, &max, nil, nil)
	min_i = SELF_CHECK_STATE
	max_i = HEAD_BLOCK_STATE
	s := gcmd.Get_int("S", SELF_CHECK_STATE, &min_i, &max_i)
	sensitivity := 0
	switch s {
	case SELF_CHECK_STATE:
		sensitivity = c.self_check_sensitivity
	case SCRATCH_STATE:
		sensitivity = c.scratch_sensitivity
	case BLOCK_FILAMENT_STATE:
		sensitivity = c.block_filament_sensitivity
	case HEAD_BLOCK_STATE:
		sensitivity = c.head_block_sensitivity
	}
	if e > 0. {
		c.cs1237_check_start(t, int64(s), int64(sensitivity), 0)
	} else {
		c.cs1237_check_stop(int64(s))
	}
	return nil
}

func (c *cs1237) Cmd_G9121(arg interface{}) error {
	gcmd, ok := arg.(*GCodeCommand)
	if !ok {
		return nil
	}

	min_i := 0
	max_i := 1
	e := gcmd.Get_int("E", 0.0, &min_i, &max_i)
	c.report = gcmd.Get_int("R", 0, &min_i, &max_i) > 0
	min := 0.1
	max := 1.0
	t := gcmd.Get_float("T", 1, &min, &max, nil, nil)
	if e > 0. {
		c.cs1237_check_start(t, SCRATCH_STATE, int64(c.scratch_sensitivity), 0)
	} else {
		c.cs1237_check_stop(SCRATCH_STATE)
	}
	return nil
}

func (c *cs1237) Cmd_G9122(arg interface{}) error {
	gcmd, ok := arg.(*GCodeCommand)
	if !ok {
		return nil
	}

	min_i := 0
	max_i := 1
	e := gcmd.Get_int("E", 0.0, &min_i, &max_i)
	c.report = gcmd.Get_int("R", 0, &min_i, &max_i) > 0
	min := 0.1
	max := 1.0
	t := gcmd.Get_float("T", 1, &min, &max, nil, nil)
	if e > 0. {
		c.cs1237_check_start(t, SELF_CHECK_STATE, int64(c.self_check_sensitivity), 0)
	} else {
		c.cs1237_check_stop(SELF_CHECK_STATE)
	}
	return nil
}

func (c *cs1237) Cmd_G9123(arg interface{}) error {
	defer func() {
		c.query_comple = nil
	}()
	gcmd, ok := arg.(*GCodeCommand)
	if !ok {
		return nil
	}
	zero := 0
	three := 3
	w := gcmd.Get_int("W", 0, &zero, &three)
	if c.query_comple == nil {
		c.query_comple = c.printer.reactor.Completion()
	}
	c.checkself_cs1237.Send([]int64{int64(c.oid), int64(w)}, 0, 0)

	timeout := c.printer.reactor.Monotonic() + 2
	//}
	params := c.query_comple.Wait(timeout, nil)
	if params != nil {
		c.checkself_flag = params.(map[string]interface{})["flag"].(int64)
	}
	return nil
}

func (c *cs1237) cs1237_check_start(check_period float64, check_type int64,
	sensitivity int64, delay_send_time float64) {
	tick := c.mcu.Seconds_to_clock(check_period)
	delay_t := c.mcu.Seconds_to_clock(delay_send_time)
	c.cs_report_cmd.Send([]int64{int64(c.oid), int64(1), int64(tick),
		int64(check_type), int64(sensitivity)}, delay_t, 0)
}

func (c *cs1237) cs1237_check_stop(check_type int64) {
	c.cs_report_cmd.Send([]int64{int64(c.oid), int64(0), int64(0),
		int64(check_type), int64(0)}, 0, 0)
}

func (c *cs1237) cs1237_query_handle(params map[string]interface{}) error {
	c.adc_value = maths.Int64_conversion(params["adc"].(int64))
	c.raw_value = maths.Int64_conversion(params["raw"].(int64))
	c.sensor_state = params["state"].(int64)
	value.StaticValue.Debug.Print("params ", params, c.raw_value, c.adc_value)
	switch c.sensor_state {
	case SCRATCH_STATE_ERR:
		value.StaticValue.Debug.Print("SCRATCH_STATE_ERR")
		c.printer.Send_event("cs1237:scratch_notice", nil)
	case BLOCK_FILAMENT_ERR:
		value.StaticValue.Debug.Print("BLOCK_FILAMENT_ERR")
		c.printer.Send_event("cs1237:block_filament_notice", nil)
	case HEAD_BLOCK_STATE_ERR:
		value.StaticValue.Debug.Print("HEAD_BLOCK_STATE_ERR")
		c.printer.Send_event("cs1237:head_block_notice", nil)
	}
	return nil
}

func (c *cs1237) cs1237_diff() (int64, int64, error) {
	defer func() {
		c.query_comple = nil
	}()
	if c.query_comple == nil {
		c.query_comple = c.printer.reactor.Completion()
	}
	c.query_cs_diff.Send([]int64{int64(c.oid)}, 0, 0)
	//if timeout < constants.NEVER {
	timeout := c.printer.reactor.Monotonic() + 2
	//}
	params := c.query_comple.Wait(timeout, nil)
	if params != nil {
		_params := params.(map[string]interface{})
		return _params["diff"].(int64), _params["raw"].(int64), nil
	} else {
		value.StaticValue.Debug.Print("cs1237_query response timeout")
		return -1, -1, fmt.Errorf("cs1237_query response timeout")
	}

	return 0, 0, nil
}

func (c *cs1237) cs1237_diff_handle(params map[string]interface{}) error {
	params["raw"] = maths.Int64_conversion(params["raw"].(int64))
	//value.StaticValue.Debug.Print(params)
	tc := c.query_comple
	if (tc) != nil {
		c.printer.reactor.Async_complete(tc, params)
	}
	return nil
}

func (c *cs1237) checkself_cs1237_handle(params map[string]interface{}) error {
	gcode := c.printer.Lookup_object("gcode", object.Sentinel{}).(*GCodeDispatch)
	c.checkself_flag = maths.Int64_conversion(params["flag"].(int64))
	value.StaticValue.Debug.Print(params)
	gcode.Respond_info(fmt.Sprintf("NozzleTestFlag:%d", c.checkself_flag), true)
	tc := c.query_comple
	if (tc) != nil {
		c.printer.reactor.Async_complete(tc, params)
	}
	return nil
}

func (c *cs1237) Get_status(eventtime float64) map[string]interface{} {
	resp := make(map[string]interface{})
	if !c.report {
		return resp
	}
	resp["adc"] = c.adc_value
	resp["raw"] = c.raw_value
	resp["state"] = c.sensor_state

	return resp
}

func (c *cs1237) Reset_cs(num int) {
	count := 3
	if num > 0 {
		count = num
	}
	c.reset_cs_cmd.Send([]int64{int64(c.oid), int64(count)}, 0, 0)
	//value.StaticValue.Debug.Println("reset_cs_cmd send done")
	var toolhead = c.printer.Lookup_object("toolhead", object.Sentinel{}).(*Toolhead)
	toolhead.Dwell(0.1)
}

func (c *cs1237) cs1237_self_check_resonances() {
	gcode := MustLookupGcode(c.printer)
	toolhead := MustLookupToolhead(c.printer)
	pos := toolhead.Get_position()
	s := fmt.Sprintf("SPECIALIZED_TEST_RESONANCES axis=x FREQ_START=45 FREQ_END=55 HZ_PER_SEC=10 POINT=%f,%f,%f", pos[0], pos[1], pos[2])
	gcode.Run_script_from_command(s)
}

func (c *cs1237) cs1237_self_check(argv []interface{}) error {

	c.cs1237_check_start(1, SELF_CHECK_STATE, int64(c.self_check_sensitivity), 0)
	c.cs1237_self_check_resonances()
	toolhead := MustLookupToolhead(c.printer)
	toolhead.Wait_moves()
	c.printer.reactor.Pause(c.printer.reactor.Monotonic() + 3.0)
	c.cs1237_check_stop(SELF_CHECK_STATE)
	if c.sensor_state == SELF_CHECK_ERR {
		return errors.PressuareZAxesError
	}

	return nil
}
func (c *cs1237) Stats(eventtime float64) (bool, string) {
	return false, fmt.Sprintf("cs1237:adc_value=%d raw_value=%d sensor_state=%d", c.adc_value, c.raw_value, c.sensor_state)

}
func Load_config_cs1237(config *ConfigWrapper) interface{} {
	return NewCs1237(config)
}
