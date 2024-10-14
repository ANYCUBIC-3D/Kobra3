package project

import (
	"container/list"
	"errors"
	"fmt"
	"k3c/common/constants"
	"k3c/common/utils/cast"
	"k3c/common/utils/collections"
	"k3c/common/utils/maths"
	"k3c/common/utils/object"
	"k3c/common/utils/str"
	"k3c/common/utils/sys"
	"k3c/common/value"
	"math"
	"math/bits"
	"sort"
	"strings"
)

// Return the position of the first bit set in a mask
func ffs(mask int64) int {
	return bits.TrailingZeros64(uint64(mask))
}

type FieldHelper struct {
	all_fields        map[string]map[string]int64
	signed_fields     map[string]int
	field_formatters  map[string]func(interface{}) string
	registers         map[string]interface{}
	field_to_register map[string]string
}

func NewFieldHelper(all_fields map[string]map[string]int64, signed_fields []string, field_formatters map[string]func(interface{}) string,
	registers *map[string]interface{}) *FieldHelper {
	self := new(FieldHelper)
	self.all_fields = all_fields
	self.signed_fields = make(map[string]int)
	for _, field := range signed_fields {
		self.signed_fields[field] = 1
	}

	self.field_formatters = field_formatters
	self.registers = make(map[string]interface{})
	if registers != nil {
		for k, v := range *registers {
			self.registers[k] = v
		}
	}

	self.field_to_register = make(map[string]string)
	for r, fields := range self.all_fields {
		for f := range fields {
			self.field_to_register[f] = r
		}
	}
	return self
}

func (self *FieldHelper) Lookup_register(field_name string, _default interface{}) interface{} {
	if val, ok := self.field_to_register[field_name]; ok {
		return val
	}
	return _default
}

/*
*

	def get_field(self, field_name, reg_value=None, reg_name=None):
	    # Returns value of the register field
	    if reg_name is None:
	        reg_name = self.field_to_register[field_name]
	    if reg_value is None:
	        reg_value = self.registers.get(reg_name, 0)
	    mask = self.all_fields[reg_name][field_name]
	    field_value = (reg_value & mask) >> ffs(mask)
	    if field_name in self.signed_fields and ((reg_value & mask)<<1) > mask:
	        field_value -= (1 << field_value.bit_length())
	    return field_value
*/
func (self *FieldHelper) Get_field(field_name string, reg_value interface{}, _reg_name *string) int64 {
	// Returns value of the register field
	reg_name := cast.String(_reg_name)
	if value.IsNone(_reg_name) {
		reg_name = self.field_to_register[field_name]
	}

	if value.IsNone(reg_value) {
		reg_value = self.registers[reg_name]
	}

	mask := self.all_fields[reg_name][field_name]
	field_value := (cast.ToInt64(reg_value) & mask) >> ffs(mask)
	if _, ok := self.signed_fields[field_name]; ok && ((cast.ToInt64(reg_value)&mask)<<1) > mask {
		field_value -= 1 << bits.Len(uint(field_value))
	}

	return field_value
}

/*
*

def set_field(self, field_name, field_value, reg_value=None, reg_name=None):

	# Returns register value with field bits filled with supplied value
	if reg_name is None:
	    reg_name = self.field_to_register[field_name]
	if reg_value is None:
	    reg_value = self.registers.get(reg_name, 0)
	mask = self.all_fields[reg_name][field_name]
	new_value = (reg_value & ~mask) | ((field_value << ffs(mask)) & mask)
	self.registers[reg_name] = new_value
	return new_value
*/
func (self *FieldHelper) Set_field(field_name string, field_value interface{}, reg_value interface{}, reg_name interface{}) int64 {
	// Returns register value with field bits filled with supplied value
	//reg_name := cast.String(_reg_name)
	if value.IsNone(reg_name) {
		reg_name = self.field_to_register[field_name]
	}

	if value.IsNone(reg_value) {
		reg_value = self.registers[reg_name.(string)]
	}
	mask := self.all_fields[reg_name.(string)][field_name]
	new_value := (cast.ToInt64(reg_value) & ^mask) | ((cast.ToInt64(field_value) << ffs(mask)) & mask)
	self.registers[reg_name.(string)] = new_value
	return new_value
}

/**

  def set_config_field(self, config, field_name, default):
      # Allow a field to be set from the config file
      config_name = "driver_" + field_name.upper()
      reg_name = self.field_to_register[field_name]
      mask = self.all_fields[reg_name][field_name]
      maxval = mask >> ffs(mask)
      if maxval == 1:
          val = config.getboolean(config_name, default)
      elif field_name in self.signed_fields:
          val = config.getint(config_name, default,
                              minval=-(maxval//2 + 1), maxval=maxval//2)
      else:
          val = config.getint(config_name, default, minval=0, maxval=maxval)
      return self.set_field(field_name, val)
*/

func (self *FieldHelper) Set_config_field(config *ConfigWrapper, field_name string, _default interface{}) int64 {
	// Allow a field to be set from the config file
	config_name := "driver_" + strings.ToUpper(field_name)
	reg_name := self.field_to_register[field_name]
	mask := self.all_fields[reg_name][field_name]
	maxval := mask >> ffs(mask)

	var val interface{}
	if maxval == 1 {
		_val := config.Getboolean(config_name, _default, true)
		if _val {
			val = 1
		} else {
			val = 0
		}

	} else if _, ok := self.signed_fields[field_name]; ok {
		if _, ok := _default.(int); ok {
			val = config.Getint(config_name, _default, -(maths.FloorDiv(cast.ForceInt(maxval), 2) + 1), maths.FloorDiv(cast.ForceInt(maxval), 2), true)
		} else {
			val = config.Getint64(config_name, _default, -1, maxval, true)
		}
	} else {
		if _, ok := _default.(int); ok {
			val = config.Getint(config_name, _default, 0, cast.ForceInt(maxval), true)
		} else {
			val = config.Getint64(config_name, _default, 0, maxval, true)

		}
	}
	return self.Set_field(field_name, val, nil, nil)
}

/**

  def pretty_format(self, reg_name, reg_value):
      # Provide a string description of a register
      reg_fields = self.all_fields.get(reg_name, {})
      reg_fields = sorted([(mask, name) for name, mask in reg_fields.items()])
      fields = []
      for mask, field_name in reg_fields:
          field_value = self.get_field(field_name, reg_value, reg_name)
          sval = self.field_formatters.get(field_name, str)(field_value)
          if sval and sval != "0":
              fields.append(" %s=%s" % (field_name, sval))
      return "%-11s %08x%s" % (reg_name + ":", reg_value, "".join(fields))

*/

func (self *FieldHelper) pretty_format(reg_name string, reg_value interface{}) string {
	// Provide a string description of a register
	reg_fields := self.all_fields[reg_name]

	keys := str.MapStringKeys(reg_fields)
	sort.Strings(keys)
	fields := make([]string, 0, len(keys))
	for _, field_name := range keys {
		field_value := self.Get_field(field_name, reg_value, cast.StringP(reg_name))
		if self.field_formatters[field_name] == nil {
			value.StaticValue.Debug.Printf("pretty_format nil error: %v, %v, %v", reg_name, field_name, field_value)
			continue
		}
		sval := self.field_formatters[field_name](field_value)
		if len(sval) != 0 && sval != "0" {
			fields = append(fields, fmt.Sprintf(" %s=%s", field_name, sval))
		}
	}

	return fmt.Sprintf("%-11s %08x%s", reg_name+":", reg_value, strings.Join(fields, ""))
}

/**

  def get_reg_fields(self, reg_name, reg_value):
      # Provide fields found in a register
      reg_fields = self.all_fields.get(reg_name, {})
      return {field_name: self.get_field(field_name, reg_value, reg_name)
              for field_name, mask in reg_fields.items()}

*/

func (self *FieldHelper) Get_reg_fields(reg_name string, reg_value interface{}) map[string]int64 {
	// Provide fields found in a register
	reg_fields := self.all_fields[reg_name]
	var regFields = make(map[string]int64)
	for field_name := range reg_fields {
		regFields[field_name] = self.Get_field(field_name, reg_value, cast.StringP(reg_name))
	}
	return regFields
}

/*
######################################################################
# Periodic error checking
######################################################################
*/

type TMCErrorCheck struct {
	printer              *Printer
	stepper_name         string
	mcu_tmc              IMCU_TMC
	fields               *FieldHelper
	check_timer          *ReactorTimer
	last_drv_status      interface{}
	last_status          map[string]interface{}
	gstat_reg_info       []interface{}
	clear_gstat          bool
	irun_field           string
	drv_status_reg_info  []interface{}
	drv_check_timer      *ReactorTimer
	drv_ola_olb_reg_info []interface{}
}

type IMCU_TMC interface {
	Get_fields() *FieldHelper
	Get_register(string) (int64, error)
	Set_register(string, int64, *float64) error
}

func NewTMCErrorCheck(config *ConfigWrapper, mcu_tmc IMCU_TMC) *TMCErrorCheck {
	self := new(TMCErrorCheck)
	self.printer = config.Get_printer()
	name_parts := strings.Split(config.Get_name(), " ")
	self.stepper_name = strings.Join(name_parts[1:], " ")
	self.mcu_tmc = mcu_tmc
	self.fields = mcu_tmc.Get_fields()
	self.check_timer = nil
	self.last_drv_status = nil

	self.last_status = nil
	// Setup for GSTAT query
	_reg_name := self.fields.Lookup_register("drv_err", nil)
	if !value.IsNone(_reg_name) {
		self.gstat_reg_info = []interface{}{0, _reg_name, int64(0xffffffff), int64(0xffffffff), 0}
	} else {
		self.gstat_reg_info = nil
	}

	self.clear_gstat = true

	// Setup for DRV_STATUS query
	self.irun_field = "irun"
	reg_name := "DRV_STATUS"

	var (
		mask           int64 = 0
		err_mask       int64 = 0
		cs_actual_mask int64 = 0
	)

	if name_parts[0] == "tmc2130" {
		// TMC2130 driver quirks
		self.clear_gstat = false
		cs_actual_mask = self.fields.all_fields[reg_name]["cs_actual"]
	} else if name_parts[0] == "tmc2660" {
		// TMC2660 driver quirks
		self.irun_field = "cs"
		reg_name = "READRSP@RDSEL2"
		cs_actual_mask = self.fields.all_fields[reg_name]["se"]
	}

	err_fields := []string{"ot", "s2ga", "s2gb", "s2vsa", "s2vsb"}
	warn_fields := []string{"otpw", "t120", "t143", "t150", "t157"}

	for _, f := range str.MergeSlice(err_fields, warn_fields) {
		if _, ok := self.fields.all_fields[reg_name]; ok {
			mask |= self.fields.all_fields[reg_name][f]
			if collections.Contains(err_fields, f) {
				err_mask |= self.fields.all_fields[reg_name][f]
			}
		}
	}

	self.drv_status_reg_info = []interface{}{0, reg_name, mask, err_mask, cs_actual_mask}

	drv_err_fields := []string{"ola", "ola"}
	mask = 0
	for _, f := range drv_err_fields {
		if _, ok := self.fields.all_fields[reg_name]; ok {
			mask |= self.fields.all_fields[reg_name][f]
		}
	}
	self.drv_ola_olb_reg_info = []interface{}{0, reg_name, mask, mask, cs_actual_mask}
	return self
}

/**

  def _query_register(self, reg_info, try_clear=False):
      last_value, reg_name, mask, err_mask, cs_actual_mask = reg_info
      cleared_flags = 0
      count = 0
      while 1:
          try:
              val = self.mcu_tmc.get_register(reg_name)
          except self.printer.command_error as e:
              count += 1
              if count < 3 and str(e).startswith("Unable to read tmc uart"):
                  # Allow more retries on a TMC UART read error
                  reactor = self.printer.get_reactor()
                  reactor.pause(reactor.monotonic() + 0.050)
                  continue
              raise
          if val & mask != last_value & mask:
              fmt = self.fields.pretty_format(reg_name, val)
              logging.info("TMC '%s' reports %s", self.stepper_name, fmt)
          reg_info[0] = last_value = val
          if not val & err_mask:
              if not cs_actual_mask or val & cs_actual_mask:
                  break
              irun = self.fields.get_field(self.irun_field)
              if self.check_timer is None or irun < 4:
                  break
              if (self.irun_field == "irun"
                  and not self.fields.get_field("ihold")):
                  break
              # CS_ACTUAL field of zero - indicates a driver reset
          count += 1
          if count >= 3:
              fmt = self.fields.pretty_format(reg_name, val)
              raise self.printer.command_error("TMC '%s' reports error: %s"
                                               % (self.stepper_name, fmt))
          if try_clear and val & err_mask:
              try_clear = False
              cleared_flags |= val & err_mask
              self.mcu_tmc.set_register(reg_name, val & err_mask)
      return cleared_flags

*/

var printerCommandError = errors.New("printer command error")

func (self *TMCErrorCheck) _query_register(reg_info []interface{}, try_clear bool) interface{} {
	var (
		last_value           = cast.ToInt64(reg_info[0])
		reg_name             = cast.ToString(reg_info[1])
		mask                 = cast.ToInt64(reg_info[2])
		err_mask             = cast.ToInt64(reg_info[3])
		cs_actual_mask       = cast.ToInt64(reg_info[4])
		cleared_flags  int64 = 0
		count                = 0
	)

	for {
		val, err := self.mcu_tmc.Get_register(reg_name)
		if err != nil {
			count += 1
			if count < 3 && strings.HasPrefix(err.Error(), "Unable to read tmc uart") {
				// Allow more retries on a TMC UART read error
				reactor := self.printer.Get_reactor()
				reactor.Pause(reactor.Monotonic() + 0.050)
				continue
			}
			return err
		}

		if val&mask != last_value&mask { //  @todo val & mask != last_value & mask 优先级确定
			fmtStr := self.fields.pretty_format(reg_name, val)
			value.StaticValue.Debug.Printf("TMC %s reports %s\n", self.stepper_name, fmtStr)
		}

		reg_info[0] = val
		last_value = val
		cs_actual_mask = cs_actual_mask
		if (val & err_mask) == 0 { // @todo not val & err_mask
			if cs_actual_mask == 0 || (val&cs_actual_mask) != 0 {
				break
			}

			irun := self.fields.Get_field(self.irun_field, nil, nil)
			if value.IsNone(self.check_timer) || irun < 4 {
				break
			}

			if self.irun_field == "irun" && self.fields.Get_field("ihold", nil, nil) == 0 {
				break
			}
		}

		count += 1
		if count >= 3 {
			fmtStr := self.fields.pretty_format(reg_name, val)
			return fmt.Errorf("TMC %s reports error: %s", self.stepper_name, fmtStr)
		}

		if try_clear && (val&err_mask) != 0 { // @todo try_clear and val & err_mask:
			try_clear = false
			cleared_flags |= val & err_mask
			self.mcu_tmc.Set_register(reg_name, val&err_mask, nil)
		}
	}

	return cleared_flags
}

/**

  def _do_periodic_check(self, eventtime):
      try:
          self._query_register(self.drv_status_reg_info)
          if self.gstat_reg_info is not None:
              self._query_register(self.gstat_reg_info)
      except self.printer.command_error as e:
          self.printer.invoke_shutdown(str(e))
          return self.printer.get_reactor().NEVER
      return eventtime + 1.
*/

func (self *TMCErrorCheck) _do_periodic_check(eventtime float64) float64 {
	defer sys.CatchPanic()
	result := self._query_register(self.drv_status_reg_info, false)
	if _, ok := result.(error); ok {
		//self.printer.Invoke_shutdown(result.(error).Error())
		self.printer.reactor.Unregister_timer(self.check_timer)
		self.check_timer = nil
		value.StaticValue.Error.Print(result)

		return constants.NEVER
	}

	if !value.IsNone(self.gstat_reg_info) {
		result = self._query_register(self.gstat_reg_info, false)
		if _, ok := result.(error); ok {
			//self.printer.Invoke_shutdown(result.(error).Error())
			self.printer.reactor.Unregister_timer(self.check_timer)
			self.check_timer = nil
			value.StaticValue.Error.Print(result)
			return constants.NEVER
		}
	}
	return eventtime + 1
}

func (self *TMCErrorCheck) drv_do_periodic_check(eventtime float64) float64 {
	defer sys.CatchPanic()
	result := self._query_register(self.drv_ola_olb_reg_info, false)
	if _, ok := result.(error); ok {
		self.printer.reactor.Unregister_timer(self.drv_check_timer)
		self.drv_check_timer = nil
		value.StaticValue.Error.Print(result)
		self.printer.Invoke_shutdown(result.(error).Error())
		return constants.NEVER
	}

	return eventtime + 1
}

func (self *TMCErrorCheck) drv_stop_checks() {
	if value.IsNone(self.drv_check_timer) {
		return
	}

	self.printer.reactor.Update_timer(self.drv_check_timer, constants.NEVER)
	//self.check_timer = nil
}

func (self *TMCErrorCheck) drv_start_checks() {
	reactor := self.printer.Get_reactor()
	curtime := reactor.Monotonic()
	self.drv_check_timer = reactor.Register_timer(self.drv_do_periodic_check,
		curtime+1.)
}

/**

  def stop_checks(self):
      if self.check_timer is None:
          return
      self.printer.get_reactor().unregister_timer(self.check_timer)
      self.check_timer = None
*/

func (self *TMCErrorCheck) Stop_checks() {
	if value.IsNone(self.check_timer) {
		return
	}
	self.printer.Get_reactor().Unregister_timer(self.check_timer)
	self.check_timer = nil
}

/**

    def start_checks(self):
        if self.check_timer is not None:
            self.stop_checks()
        cleared_flags = 0
        self._query_register(self.drv_status_reg_info)
        if self.gstat_reg_info is not None:
            cleared_flags = self._query_register(self.gstat_reg_info,
                                                 try_clear=self.clear_gstat)
        reactor = self.printer.get_reactor()
        curtime = reactor.monotonic()
        self.check_timer = reactor.register_timer(self._do_periodic_check,
                                                  curtime + 1.)
        if cleared_flags:
            reset_mask = self.fields.all_fields["GSTAT"]["reset"]
            if cleared_flags & reset_mask:
                return True
        return False

**/

func (self *TMCErrorCheck) Start_checks() bool {
	if value.IsNotNone(self.check_timer) {
		self.Stop_checks()
	}
	var cleared_flags int64 = 0
	result := self._query_register(self.drv_status_reg_info, false)
	if _, ok := result.(error); ok {
		self.printer.Invoke_shutdown(result.(error).Error())
	}

	if value.IsNotNone(self.gstat_reg_info) {
		cleared_flags = cast.ToInt64(self._query_register(self.gstat_reg_info, self.clear_gstat))
	}

	reactor := self.printer.Get_reactor()
	curtime := reactor.Monotonic()
	self.check_timer = reactor.Register_timer(self._do_periodic_check,
		curtime+1.)

	if cleared_flags != 0 {
		reset_mask := self.fields.all_fields["GSTAT"]["reset"]
		if (cleared_flags & reset_mask) != 0 {
			return true
		}
	}

	return false
}

/**

  def get_status(self, eventtime=None):
      if self.check_timer is None:
          return {'drv_status': None}
      last_value, reg_name = self.drv_status_reg_info[:2]
      if last_value != self.last_drv_status:
          self.last_drv_status = last_value
          fields = self.fields.get_reg_fields(reg_name, last_value)
          fields = {n: v for n, v in fields.items() if v}
          self.last_status = {'drv_status': fields}
      return self.last_status
*/

func (self *TMCErrorCheck) Get_status(eventtime float64) map[string]interface{} {
	if value.IsNone(self.check_timer) {
		return map[string]interface{}{"drv_status": nil}
	}

	var (
		last_value = cast.ToInt(self.drv_status_reg_info[0])
		reg_name   = cast.ToString(self.drv_status_reg_info[1])
	)

	if last_value != cast.ToInt(self.last_drv_status) {
		self.last_drv_status = last_value
		fieldstemp := self.fields.Get_reg_fields(reg_name, last_value)
		fields := make(map[string]int64)
		for k, v := range fieldstemp {
			if v != 0 {
				fields[k] = v
			}
		}

		self.last_status = map[string]interface{}{"drv_status": fields}
	}
	return self.last_status
}

/**
######################################################################
# G-Code command helpers
######################################################################
**/

type TMCCommandHelper struct {
	printer          *Printer
	stepper_name     string
	name             string
	mcu_tmc          IMCU_TMC
	current_helper   ICurrentHelper
	echeck_helper    *TMCErrorCheck
	fields           *FieldHelper
	read_registers   interface{}
	read_translate   interface{}
	toff             interface{}
	mcu_phase_offset *int
	stepper          *MCU_stepper
	stepper_enable   *PrinterStepperEnable
	sg_results       []int
	tsteps           []int
	sg_flush_timer   *ReactorTimer
	sgthrs           int64
	dedge            bool
	initial_check    bool
}

const (
	cmd_INIT_TMC_help          = "Initialize TMC stepper driver registers"
	cmd_SET_TMC_FIELD_help     = "Set a register field of a TMC driver"
	cmd_SET_TMC_CURRENT_help   = "Set the current of a TMC driver"
	cmd_DUMP_TMC_help          = "Read and display TMC stepper driver registers"
	cmd_OPEN_STEALTHCHOP_help  = "Open STEALTHCHOP"
	cmd_CLOSE_STEALTHCHOP_help = "Close STEALTHCHOP"
)

type ICurrentHelper interface {
	Get_current() []float64
	Set_current(float64, float64, float64)
}

func NewTMCCommandHelper(config *ConfigWrapper, mcu_tmc IMCU_TMC, current_helper ICurrentHelper) *TMCCommandHelper {
	self := new(TMCCommandHelper)
	self.printer = config.Get_printer()
	name_parts := strings.Split(config.Get_name(), " ")
	self.stepper_name = strings.Join(name_parts[1:], " ")
	self.name = str.LastName(config.Get_name())
	self.mcu_tmc = mcu_tmc
	self.current_helper = current_helper
	self.echeck_helper = NewTMCErrorCheck(config, mcu_tmc)
	self.fields = mcu_tmc.Get_fields()
	self.read_registers = nil
	self.read_translate = nil
	self.toff = nil
	self.mcu_phase_offset = nil
	self.stepper = nil
	self.stepper_enable = self.printer.Load_object(config, "stepper_enable", object.Sentinel{}).(*PrinterStepperEnable)
	self.printer.Register_event_handler("stepper:sync_mcu_position",
		self._handle_sync_mcu_pos)
	self.printer.Register_event_handler("stepper:set_sdir_inverted",
		self._handle_sync_mcu_pos)
	self.printer.Register_event_handler("project:mcu_identify",
		self._handle_mcu_identify)
	self.printer.Register_event_handler("project:connect",
		self._handle_connect)
	self.printer.Register_event_handler("homing:check_ola_olb_begin",
		self._handle_check_ola_olb_begin)
	self.printer.Register_event_handler("homing:check_ola_olb_end",
		self._handle_check_ola_olb_end)

	// Set microstep config options
	TMCMicrostepHelper(config, mcu_tmc)

	// Register commands
	gcode := MustLookupGcode(self.printer)
	gcode.Register_mux_command("SET_TMC_FIELD", "STEPPER", self.name,
		self.Cmd_SET_TMC_FIELD,
		cmd_SET_TMC_FIELD_help)
	gcode.Register_mux_command("INIT_TMC", "STEPPER", self.name,
		self.Cmd_INIT_TMC,
		cmd_INIT_TMC_help)
	gcode.Register_mux_command("SET_TMC_CURRENT", "STEPPER", self.name,
		self.Cmd_SET_TMC_CURRENT,
		cmd_SET_TMC_CURRENT_help)

	gcode.Register_mux_command("OPEN_STEALTHCHOP", "STEPPER", self.name,
		self.Cmd_OPEN_STEALTHCHOP,
		cmd_OPEN_STEALTHCHOP_help)

	gcode.Register_mux_command("CLOSE_STEALTHCHOP", "STEPPER", self.name,
		self.Cmd_CLOSE_STEALTHCHOP,
		cmd_CLOSE_STEALTHCHOP_help)

	self.dedge = config.Getboolean("driver_DEDGE", true, true)
	self.initial_check = true
	return self
}

func (self *TMCCommandHelper) Cmd_OPEN_STEALTHCHOP(argv interface{}) error {
	coolthrs := self.fields.Get_field("tcoolthrs", value.None, nil)
	reg := self.fields.Lookup_register("en_pwm_mode", value.None)
	var val int64
	if value.IsNone(reg) {
		// On "stallguard4" drivers, "stealthchop" must be enabled
		val = self.fields.Set_field("en_spreadcycle", 0, value.None, nil)
		value.StaticValue.Debug.Print("OPEN_STEALTHCHOP", val, self.fields.Get_field("en_spreadcycle", value.None, nil))
	} else {
		// On earlier drivers, "stealthchop" must be disabled
		val = self.fields.Set_field("en_pwm_mode", 0, value.None, nil)
	}
	self.mcu_tmc.Set_register("GCONF", val, nil)
	if coolthrs == 0 {
		tc_val := self.fields.Set_field("tcoolthrs", 0xfffff, value.None, nil)
		self.mcu_tmc.Set_register("TCOOLTHRS", tc_val, nil)
	}
	return nil
}

func (self *TMCCommandHelper) Cmd_CLOSE_STEALTHCHOP(argv interface{}) error {
	var val int64
	reg := self.fields.Lookup_register("en_pwm_mode", value.None)
	if value.IsNone(reg) {
		val = self.fields.Set_field("en_spreadcycle", 1, value.None, nil)
		value.StaticValue.Debug.Print("CLOSE_STEALTHCHOP", val, self.fields.Get_field("en_spreadcycle", value.None, nil))
	} else {
		val = self.fields.Set_field("en_pwm_mode", 1, value.None, nil)
	}
	self.mcu_tmc.Set_register("GCONF", val, nil)
	tc_val := self.fields.Set_field("tcoolthrs", 0, value.None, nil)
	self.mcu_tmc.Set_register("TCOOLTHRS", tc_val, nil)
	return nil
}

func (self *TMCCommandHelper) Get_en_spreadcycle() bool {
	return self.fields.Get_field("en_spreadcycle", value.None, nil) == 1
}

/**

def _init_registers(self, print_time=None):
       # Send registers
       for reg_name, val in self.fields.registers.items():
           self.mcu_tmc.set_register(reg_name, val, print_time)
*/

func (self *TMCCommandHelper) _init_registers(print_time *float64) {
	for reg_name, val := range self.fields.registers {
		self.mcu_tmc.Set_register(reg_name, cast.ToInt64(val), print_time)
	}
}

/**

  def cmd_INIT_TMC(self, gcmd):
      logging.info("INIT_TMC %s", self.name)
      print_time = self.printer.lookup_object('toolhead').get_last_move_time()
      self._init_registers(print_time)
*/

func (self *TMCCommandHelper) Cmd_INIT_TMC(argv interface{}) error {
	value.StaticValue.Debug.Printf("INIT_TMC %s", self.name)
	print_time := MustLookupToolhead(self.printer).Get_last_move_time()
	self._init_registers(cast.Float64P(print_time))
	return nil
}

/**

  def cmd_SET_TMC_FIELD(self, gcmd):
      field_name = gcmd.get('FIELD').lower()
      reg_name = self.fields.lookup_register(field_name, None)
      if reg_name is None:
          raise gcmd.error("Unknown field name '%s'" % (field_name,))
      value = gcmd.get_int('VALUE')
      reg_val = self.fields.set_field(field_name, value)
      print_time = self.printer.lookup_object('toolhead').get_last_move_time()
      self.mcu_tmc.set_register(reg_name, reg_val, print_time)
*/

func (self *TMCCommandHelper) Cmd_SET_TMC_FIELD(argv interface{}) error {
	gcmd := argv.(*GCodeCommand)
	field_name := strings.ToLower(gcmd.Get("FIELD", object.Sentinel{}, "", nil, nil, nil, nil))
	reg_name := self.fields.Lookup_register(field_name, nil)

	if value.IsNone(reg_name) {
		return fmt.Errorf("Unknown field name '%s'", field_name)
	}
	value := gcmd.Get_int("VALUE", 0, nil, nil)
	reg_val := self.fields.Set_field(field_name, value, nil, nil)
	//print_time := MustLookupToolhead(self.printer).Get_last_move_time()
	self.mcu_tmc.Set_register(cast.ToString(reg_name), reg_val, nil)
	return nil
}

/*
*

	def cmd_SET_TMC_CURRENT(self, gcmd):
	    ch = self.current_helper
	    prev_cur, prev_hold_cur, req_hold_cur, max_cur = ch.get_current()
	    run_current = gcmd.get_float('CURRENT', None, minval=0., maxval=max_cur)
	    hold_current = gcmd.get_float('HOLDCURRENT', None,
	                                  above=0., maxval=max_cur)
	    if run_current is not None or hold_current is not None:
	        if run_current is None:
	            run_current = prev_cur
	        if hold_current is None:
	            hold_current = req_hold_cur
	        toolhead = self.printer.lookup_object('toolhead')
	        print_time = toolhead.get_last_move_time()
	        ch.set_current(run_current, hold_current, print_time)
	        prev_cur, prev_hold_cur, req_hold_cur, max_cur = ch.get_current()
	    # Report values
	    if prev_hold_cur is None:
	        gcmd.respond_info("Run Current: %0.2fA" % (prev_cur,))
	    else:
	        gcmd.respond_info("Run Current: %0.2fA Hold Current: %0.2fA"
	                          % (prev_cur, prev_hold_cur)
*/
func (self *TMCCommandHelper) Cmd_SET_TMC_CURRENT(argv interface{}) error {
	ch := self.current_helper
	current := ch.Get_current()

	var (
		prev_cur      = current[0] // pointer value
		prev_hold_cur = current[1]
		req_hold_cur  = current[2]
		max_cur       = current[3]
	)

	gcmd := argv.(*GCodeCommand)
	_run_current := gcmd.Get_floatP("CURRENT", nil, cast.Float64P(0.), cast.Float64P(max_cur), nil, nil)
	_hold_current := gcmd.Get_floatP("HOLDCURRENT", nil, nil, cast.Float64P(max_cur),
		cast.Float64P(0.), nil)
	run_current := cast.Float64(_run_current)
	hold_current := cast.Float64(_hold_current)

	if value.IsNotNone(_run_current) || value.IsNotNone(_hold_current) {
		if value.IsNone(_run_current) {
			run_current = prev_cur
		}

		if value.IsNone(_hold_current) {
			hold_current = req_hold_cur
		}

		toolhead := MustLookupToolhead(self.printer)
		print_time := toolhead.Get_last_move_time()
		ch.Set_current(run_current, hold_current, print_time)

		current = ch.Get_current() // fetch again bad bad bad

		prev_cur = current[0]
		prev_hold_cur = current[1]
		req_hold_cur = current[2]
		max_cur = current[3]
	}

	if prev_hold_cur == -1 { // -1 as none @todo 考虑优化处理
		gcmd.Respond_info(fmt.Sprintf("Run Current: %0.2fA", prev_cur), false)
	} else {
		gcmd.Respond_info(fmt.Sprintf("Run Current: %0.2fA Hold Current: %0.2fA", prev_cur, prev_hold_cur), false)
	}
	return nil
}

/*
*

	def _get_phases(self):
	      return (256 >> self.fields.get_field("mres")) * 4
*/
func (self *TMCCommandHelper) _get_phases() int {
	shift := self.fields.Get_field("mres", nil, nil)
	return (256 >> shift) * 4
}

/*
*

	def get_phase_offset(self):
	    return self.mcu_phase_offset, self._get_phases()
	def _query_phase(self):
	    field_name = "mscnt"
	    if self.fields.lookup_register(field_name, None) is None:
	        # TMC2660 uses MSTEP
	        field_name = "mstep"
	    reg = self.mcu_tmc.get_register(self.fields.lookup_register(field_name))
	    return self.fields.get_field(field_name, reg)
*/
func (self *TMCCommandHelper) Get_phase_offset() (*int, int) { // @todo
	return self.mcu_phase_offset, self._get_phases()
}

func (self *TMCCommandHelper) _query_phase() int64 {
	field_name := "mscnt"
	if value.IsNone(self.fields.Lookup_register(field_name, nil)) {
		// TMC2660 uses MSTEP
		field_name = "mstep"
	}

	reg, _ := self.mcu_tmc.Get_register(cast.ToString(self.fields.Lookup_register(field_name, "")))
	return self.fields.Get_field(field_name, reg, nil)
}

/**

  def _handle_sync_mcu_pos(self, stepper):
      if stepper.get_name() != self.stepper_name:
          return
      try:
          driver_phase = self._query_phase()
      except self.printer.command_error as e:
          logging.info("Unable to obtain tmc %s phase", self.stepper_name)
          self.mcu_phase_offset = None
          enable_line = self.stepper_enable.lookup_enable(self.stepper_name)
          if enable_line.is_motor_enabled():
              raise
          return
      if not stepper.get_dir_inverted()[0]:
          driver_phase = 1023 - driver_phase
      phases = self._get_phases()
      phase = int(float(driver_phase) / 1024 * phases + .5) % phases
      moff = (phase - stepper.get_mcu_position()) % phases
      if self.mcu_phase_offset is not None and self.mcu_phase_offset != moff:
          logging.warning("Stepper %s phase change (was %d now %d)",
                          self.stepper_name, self.mcu_phase_offset, moff)
      self.mcu_phase_offset = moff

*/

func (self *TMCCommandHelper) _handle_sync_mcu_pos(argv []interface{}) error {
	stepper := argv[0].(*MCU_stepper)
	if stepper.Get_name(false) != self.stepper_name {
		return nil
	}

	defer func() {
		if r := recover(); r != nil {
			value.StaticValue.Error.Printf("Unable to obtain tmc %s phase", self.stepper_name)
			self.mcu_phase_offset = nil
			enable_line, err := self.stepper_enable.Lookup_enable(self.stepper_name)
			if err == nil {
				if enable_line.Is_motor_enabled() {
					value.StaticValue.Error.Panicf(fmt.Sprintf("TMCCommandHelper _handle_sync_mcu_pos %v", r))
				}
			} else {
				msg := fmt.Sprintf("TMCCommandHelper _handle_sync_mcu_pos %v, error: %v", r, err)
				value.StaticValue.Error.Printf(msg)
			}
		}
	}()

	driver_phase := self._query_phase() // @todo python会捕获此处异常，但貌似不会发生异常
	ret0, _ := stepper.Get_dir_inverted()
	if ret0 != 0 {
		driver_phase = 1023 - driver_phase
	}

	phases := self._get_phases()
	// HACK: Python的“%”运算符的运算结果与golang的不同，需要进行修正
	phase := maths.PyMod(int(float64(driver_phase)/1024*float64(phases)+.5), phases)
	moff := maths.PyMod(phase-stepper.Get_mcu_position(), phases)

	if value.IsNotNone(self.mcu_phase_offset) && cast.Int(self.mcu_phase_offset) != moff {
		value.StaticValue.Debug.Printf("Stepper %s phase change (was %d now %d)",
			self.stepper_name, self.mcu_phase_offset, moff)
	}
	self.mcu_phase_offset = &moff
	return nil
}

func (self *TMCCommandHelper) _do_enable(print_time *float64) {
	defer func() {
		if r := recover(); r != nil {
			value.StaticValue.Error.Printf("TMCCommandHelper->_do_enable panic: %v %v", r, self.stepper_name)
			//self.printer.Invoke_shutdown(r)
		}
	}()
	if value.IsNotNone(self.toff) {
		// Shared enable via comms handling
		self.fields.Set_field("toff", self.toff, nil, nil)
	}

	self._init_registers(nil)
	did_reset := self.echeck_helper.Start_checks()
	if did_reset {
		self.mcu_phase_offset = nil
	}

	// Calculate phase offset
	if value.IsNotNone(self.mcu_phase_offset) {
		return
	}
	gcode := MustLookupGcode(self.printer)

	mtx := gcode.Get_mutex()
	mtx.Lock()
	defer mtx.Unlock()
	if value.IsNotNone(self.mcu_phase_offset) {
		return
	}

	//value.StaticValue.Debug.Printf("Pausing toolhead to calculate %s phase offset",
	//	self.stepper_name)
	MustLookupToolhead(self.printer).Wait_moves()
	self._handle_sync_mcu_pos([]interface{}{self.stepper})
}

func (self *TMCCommandHelper) _do_disable(print_time *float64) {
	defer func() {
		if r := recover(); r != nil {
			value.StaticValue.Error.Printf("TMCCommandHelper->_do_disable panic: %v", r)
			self.printer.Invoke_shutdown(r)
		}
	}()

	if value.IsNotNone(self.toff) {
		val := self.fields.Set_field("toff", 0, nil, nil)
		reg_name := cast.ToString(self.fields.Lookup_register("toff", ""))
		self.mcu_tmc.Set_register(reg_name, val, print_time)
	}
}

func (self *TMCCommandHelper) _handle_mcu_identify(argv []interface{}) error {
	// Lookup stepper object
	force_move := MustLookupForceMove(self.printer)
	self.stepper = force_move.Lookup_stepper(self.stepper_name)
	if self.dedge {
		// Note pulse duration and step_both_edge optimizations available
		self.stepper.Setup_default_pulse_duration(.000000100, true)
	} else {
		self.stepper.Setup_default_pulse_duration(nil, false)
	}
	return nil
}

func (self *TMCCommandHelper) _handle_stepper_enable(print_time float64, is_enable bool) {
	var cb func(interface{}) interface{}
	if is_enable {
		cb = func(ev interface{}) interface{} {
			self._do_enable(&print_time)
			return nil
		}
	} else {
		cb = func(ev interface{}) interface{} {
			self._do_disable(&print_time)
			return nil
		}
	}
	self.printer.Get_reactor().Register_callback(cb, 0) // @todo 默认值是多少？
}

func (self *TMCCommandHelper) _handle_connect(argv []interface{}) (err error) {
	// Check if using step on both edges optimization
	_, step_both_edge := self.stepper.Get_pulse_duration()
	if step_both_edge {
		self.fields.Set_field("dedge", 1, nil, nil)
	}

	// Check for soft stepper enable/disable
	enable_line, _ := self.stepper_enable.Lookup_enable(self.stepper_name)
	enable_line.Register_state_callback(self._handle_stepper_enable)
	if !enable_line.Has_dedicated_enable() {
		self.toff = self.fields.Get_field("toff", nil, nil)
		self.fields.Set_field("toff", 0, nil, nil)
		value.StaticValue.Debug.Printf("Enabling TMC virtual enable for '%s'", self.stepper_name)
	}

	// Send init
	//defer func() {
	//	if r := recover(); r != nil {
	//		value.StaticValue.Error.Printf("TMC %s failed to init: %v", self.name, r)
	//		err = fmt.Errorf("TMC %s failed to init: %v", self.name, r)
	//	}
	//}()
	self._init_registers(nil)
	return err
}

func (self *TMCCommandHelper) _handle_check_ola_olb_begin(argv []interface{}) (err error) {
	if self.initial_check == false {
		return nil
	}
	name := argv[0].(string)
	if name == self.stepper_name {
		val := self.fields.Set_field("en_spreadcycle", 1, value.None, nil)
		self.mcu_tmc.Set_register("GCONF", val, nil)
		self.echeck_helper.drv_start_checks()
	}
	return nil
}

func (self *TMCCommandHelper) _handle_check_ola_olb_end(argv []interface{}) (err error) {
	if self.initial_check == false {
		return nil
	} else {
		self.initial_check = false
	}
	name := argv[0].(string)
	if name == self.stepper_name {
		val := self.fields.Set_field("en_spreadcycle", 0, value.None, nil)
		self.mcu_tmc.Set_register("GCONF", val, nil)
		self.echeck_helper.drv_stop_checks()
	}
	return nil
}

/**

  # get_status information export
  def get_status(self, eventtime=None):
      cpos = None
      if self.stepper is not None and self.mcu_phase_offset is not None:
          cpos = self.stepper.mcu_to_commanded_position(self.mcu_phase_offset)
      current = self.current_helper.get_current()
      res = {'mcu_phase_offset': self.mcu_phase_offset,
             'phase_offset_position': cpos,
             'run_current': current[0],
             'hold_current': current[1]}
      res.update(self.echeck_helper.get_status(eventtime))
      return res
*/

func (self *TMCCommandHelper) Get_status(eventtime float64) map[string]interface{} {
	var cpos interface{}
	if value.IsNotNone(self.stepper) && value.IsNotNone(self.mcu_phase_offset) {
		cpos = self.stepper.Mcu_to_commanded_position(cast.Int(self.mcu_phase_offset))
	}
	current := self.current_helper.Get_current()
	res := map[string]interface{}{
		"mcu_phase_offset":      self.mcu_phase_offset,
		"phase_offset_position": cpos,
		"run_current":           current[0],
		"hold_current":          current[1],
	}

	for k, v := range self.echeck_helper.Get_status(eventtime) {
		res[k] = v
	}
	return res
}

/**

  # DUMP_TMC support
  def setup_register_dump(self, read_registers, read_translate=None):
      self.read_registers = read_registers
      self.read_translate = read_translate
      gcode = self.printer.lookup_object("gcode")
      gcode.register_mux_command("DUMP_TMC", "STEPPER", self.name,
                                 self.cmd_DUMP_TMC,
                                 desc=self.cmd_DUMP_TMC_help)
*/

func (self *TMCCommandHelper) Setup_register_dump(read_registers, read_translate interface{}) {
	self.read_registers = read_registers
	self.read_translate = read_translate
	gcode := MustLookupGcode(self.printer)
	gcode.Register_mux_command("DUMP_TMC", "STEPPER", self.name, self.Cmd_DUMP_TMC, cmd_DUMP_TMC_help)
}

/**
    def cmd_DUMP_TMC(self, gcmd):
        logging.info("DUMP_TMC %s", self.name)
        print_time = self.printer.lookup_object('toolhead').get_last_move_time()
        gcmd.respond_info("========== Write-only registers ==========")
        for reg_name, val in self.fields.registers.items():
            if reg_name not in self.read_registers:
                gcmd.respond_info(self.fields.pretty_format(reg_name, val))
        gcmd.respond_info("========== Queried registers ==========")
        for reg_name in self.read_registers:
            val = self.mcu_tmc.get_register(reg_name)
            if self.read_translate is not None:
                reg_name, val = self.read_translate(reg_name, val)
            gcmd.respond_info(self.fields.pretty_format(reg_name, val))
**/

func (self *TMCCommandHelper) Cmd_DUMP_TMC(argv interface{}) error {
	value.StaticValue.Debug.Printf("DUMP_TMC %s", self.name)
	print_time := MustLookupToolhead(self.printer).Get_last_move_time()
	_ = print_time

	gcmd := argv.(*GCodeCommand)
	gcmd.Respond_info("========== Write-only registers ==========", false)

	for reg_name, val := range self.fields.registers {
		if !collections.Contains(self.read_registers.([]string), reg_name) {
			gcmd.Respond_info(self.fields.pretty_format(reg_name, val), false)
		}
	}
	gcmd.Respond_info("========== Queried registers ==========", false)
	for _, reg_name := range self.read_registers.([]string) {
		val, _ := self.mcu_tmc.Get_register(reg_name)

		if value.IsNotNone(self.read_translate) {
			//	reg_name, val = self.read_translate(reg_name, val) @todo 待确定read_translate类型之后再处理
		}

		gcmd.Respond_info(self.fields.pretty_format(reg_name, val), true)
	}
	return nil
}

/**
######################################################################
# TMC virtual pins
######################################################################
*/

// Helper class for "sensorless homing"

type TMCVirtualPinHelper struct {
	printer        *Printer
	mcu_tmc        IMCU_TMC
	fields         *FieldHelper
	diag_pin       interface{}
	diag_pin_field interface{}
	mcu_endstop    interface{}
	en_pwm         bool
	pwmthrs        int64
	coolthrs       int64
}

/**


  def __init__(self, config, mcu_tmc):
      self.printer = config.get_printer()
      self.mcu_tmc = mcu_tmc
      self.fields = mcu_tmc.get_fields()
      if self.fields.lookup_register('diag0_stall') is not None:
          if config.get('diag0_pin', None) is not None:
              self.diag_pin = config.get('diag0_pin')
              self.diag_pin_field = 'diag0_stall'
          else:
              self.diag_pin = config.get('diag1_pin', None)
              self.diag_pin_field = 'diag1_stall'
      else:
          self.diag_pin = config.get('diag_pin', None)
          self.diag_pin_field = None
      self.mcu_endstop = None
      self.en_pwm = False
      self.pwmthrs = 0
      # Register virtual_endstop pin
      name_parts = config.get_name().split()
      ppins = self.printer.lookup_object("pins")
      ppins.register_chip("%s_%s" % (name_parts[0], name_parts[-1]), self)
*/

func NewTMCVirtualPinHelper(config *ConfigWrapper, mcu_tmc IMCU_TMC) *TMCVirtualPinHelper {
	self := new(TMCVirtualPinHelper)
	self.printer = config.Get_printer()
	self.mcu_tmc = mcu_tmc
	self.fields = mcu_tmc.Get_fields()

	if value.IsNotNone(self.fields.Lookup_register("diag0_stall", nil)) {
		if value.IsNotNone(config.Get("diag0_pin", value.None, true)) {
			self.diag_pin = config.Get("diag0_pin", value.None, true)
			self.diag_pin_field = "diag0_stall"
		} else {
			self.diag_pin = config.Get("diag1_pin", value.None, true)
			self.diag_pin_field = "diag1_stall"
		}
	} else {
		self.diag_pin = config.Get("diag_pin", value.None, true)
		self.diag_pin_field = value.None
	}

	self.mcu_endstop = nil
	self.en_pwm = false
	self.pwmthrs = 0
	self.coolthrs = 0
	// Register virtual_endstop pin
	name_parts := strings.Split(config.Get_name(), " ")
	//ppins = self.printer.lookup_object("pins")
	//ppins.register_chip("%s_%s" % (name_parts[0], name_parts[-1]), self)
	ppins := MustLookupPins(self.printer)
	ppins.Register_chip(fmt.Sprintf("%s_%s", name_parts[0], name_parts[len(name_parts)-1]), self)
	return self
}

/**

  def setup_pin(self, pin_type, pin_params):
      # Validate pin
      ppins = self.printer.lookup_object('pins')
      if pin_type != 'endstop' or pin_params['pin'] != 'virtual_endstop':
          raise ppins.error("tmc virtual endstop only useful as endstop")
      if pin_params['invert'] or pin_params['pullup']:
          raise ppins.error("Can not pullup/invert tmc virtual pin")
      if self.diag_pin is None:
          raise ppins.error("tmc virtual endstop requires diag pin config")
      # Setup for sensorless homing
      reg = self.fields.lookup_register("en_pwm_mode", None)
      if reg is None:
          self.en_pwm = not self.fields.get_field("en_spreadcycle")
          self.pwmthrs = self.fields.get_field("tpwmthrs")
      else:
          self.en_pwm = self.fields.get_field("en_pwm_mode")
          self.pwmthrs = 0
      self.printer.register_event_handler("homing:homing_move_begin",
                                          self.handle_homing_move_begin)
      self.printer.register_event_handler("homing:homing_move_end",
                                          self.handle_homing_move_end)
      self.mcu_endstop = ppins.setup_pin('endstop', self.diag_pin)
      return self.mcu_endstop
*/

func (self *TMCVirtualPinHelper) Setup_pin(pin_type string, pin_params map[string]interface{}) interface{} {
	// Validate pin
	ppins := MustLookupPins(self.printer)
	if pin_type != "endstop" || pin_params["pin"] != "virtual_endstop" {
		return errors.New("tmc virtual endstop only useful as endstop")
	}

	if pin_params["invert"] == true || pin_params["pullup"] == true {
		return errors.New("Can not pullup/invert tmc virtual pin")
	}

	if value.IsNone(self.diag_pin) {
		return errors.New("tmc virtual endstop requires diag pin config")
	}

	// Setup for sensorless homing
	reg := self.fields.Lookup_register("en_pwm_mode", value.None)
	if value.IsNone(reg) {
		if self.fields.Get_field("en_spreadcycle", value.None, nil) == 1 {
			self.en_pwm = true
		} else {
			self.en_pwm = false
		}
		// self.en_pwm = not self.fields.Get_field("en_spreadcycle", value.None, )
		self.pwmthrs = self.fields.Get_field("tpwmthrs", value.None, nil)
	} else {
		self.en_pwm = cast.ToBool(self.fields.Get_field("en_pwm_mode", value.None, nil))
		self.pwmthrs = 0
	}

	self.printer.Register_event_handler("homing:homing_move_begin",
		self.handle_homing_move_begin)
	self.printer.Register_event_handler("homing:homing_move_end",
		self.handle_homing_move_end)
	self.printer.Register_event_handler("homing:homing_begin",
		self.handle_homing_begin)
	self.printer.Register_event_handler("homing:homing_end",
		self.handle_homing_end)
	self.mcu_endstop = ppins.Setup_pin("endstop", cast.ToString(self.diag_pin))
	return self.mcu_endstop
}

func (self *TMCVirtualPinHelper) handle_homing_move_begin(argv []interface{}) error {
	hmove := argv[0].(*HomingMove)
	var in bool

	for _, e := range hmove.Get_mcu_endstops() {
		es := e.(list.List)
		if _, ok := es.Front().Value.(*MCU_endstop); ok {
			if self.mcu_endstop == es.Front().Value.(*MCU_endstop) {
				in = true
				break
			}
		} else if _, ok := es.Front().Value.(*ProbeEndstopWrapper); ok {
			if self.mcu_endstop == es.Front().Value.(*ProbeEndstopWrapper) {
				in = true
				break
			}
		}
	}

	if !in {
		return nil
	}

	//reset := self.fields.Set_field("reset", 1, value.None, nil)
	//self.mcu_tmc.Set_register("GSTAT", reset, nil)
	tc_val := self.fields.Set_field("tcoolthrs", 0, value.None, nil)
	self.mcu_tmc.Set_register("TCOOLTHRS", tc_val, nil)

	self.pwmthrs = self.fields.Get_field("tpwmthrs", value.None, nil)
	self.coolthrs = self.fields.Get_field("tcoolthrs", value.None, nil)

	reg := self.fields.Lookup_register("en_pwm_mode", value.None)
	var val int64

	if value.IsNone(reg) {
		// On "stallguard4" drivers, "stealthchop" must be enabled
		self.en_pwm = self.fields.Get_field("en_spreadcycle", value.None, nil) == 0
		tp_val := self.fields.Set_field("tpwmthrs", 0, value.None, nil)
		self.mcu_tmc.Set_register("TPWMTHRS", tp_val, nil)
		val = self.fields.Set_field("en_spreadcycle", 0, value.None, nil)
	} else {
		// On earlier drivers, "stealthchop" must be disabled
		self.en_pwm = self.fields.Get_field("en_pwm_mode", value.None, nil) == 0
		self.fields.Set_field("en_pwm_mode", 0, value.None, nil)
		val = self.fields.Set_field(cast.ToString(self.diag_pin_field), 1, value.None, nil)
	}

	self.mcu_tmc.Set_register("GCONF", val, nil)
	if self.coolthrs == 0 {
		tc_val := self.fields.Set_field("tcoolthrs", 500, value.None, nil)
		self.mcu_tmc.Set_register("TCOOLTHRS", tc_val, nil)
	}

	return nil
}

func (self *TMCVirtualPinHelper) handle_homing_move_end(argv []interface{}) error {
	hmove := argv[0].(*HomingMove)
	var in bool

	for _, e := range hmove.Get_mcu_endstops() {
		es := e.(list.List)
		if _, ok := es.Front().Value.(*MCU_endstop); ok {
			if self.mcu_endstop == es.Front().Value.(*MCU_endstop) {
				in = true
				break
			}
		} else if _, ok := es.Front().Value.(*ProbeEndstopWrapper); ok {
			if self.mcu_endstop == es.Front().Value.(*ProbeEndstopWrapper) {
				in = true
				break
			}
		}
	}

	if !in {
		return nil
	}

	//var val int64
	//reg := self.fields.Lookup_register("en_pwm_mode", value.None)
	//if value.IsNone(reg) {
	//tp_val := self.fields.Set_field("tpwmthrs", self.pwmthrs, value.None, nil)
	//self.mcu_tmc.Set_register("TPWMTHRS", tp_val, cast.Float64P(0))
	//val = self.fields.Set_field("en_spreadcycle", self.en_pwm, value.None, nil)
	//} else {
	//	self.fields.Set_field("en_pwm_mode", self.en_pwm, value.None, nil)
	//val = self.fields.Set_field(cast.ToString(self.diag_pin_field), 0, value.None, nil)
	//}

	//self.mcu_tmc.Set_register("GCONF", val, cast.Float64P(0))
	tc_val := self.fields.Set_field("tcoolthrs", 0, value.None, nil)
	self.mcu_tmc.Set_register("TCOOLTHRS", tc_val, nil)
	return nil
}

func (self *TMCVirtualPinHelper) handle_homing_begin(argv []interface{}) error {
	self.pwmthrs = self.fields.Get_field("tpwmthrs", value.None, nil)
	self.coolthrs = self.fields.Get_field("tcoolthrs", value.None, nil)

	reg := self.fields.Lookup_register("en_pwm_mode", value.None)
	var val int64

	if value.IsNone(reg) {
		// On "stallguard4" drivers, "stealthchop" must be enabled
		self.en_pwm = self.fields.Get_field("en_spreadcycle", value.None, nil) == 0
		tp_val := self.fields.Set_field("tpwmthrs", 0, value.None, nil)
		self.mcu_tmc.Set_register("TPWMTHRS", tp_val, nil)
		val = self.fields.Set_field("en_spreadcycle", 0, value.None, nil)
	} else {
		// On earlier drivers, "stealthchop" must be disabled
		self.en_pwm = self.fields.Get_field("en_pwm_mode", value.None, nil) == 0
		self.fields.Set_field("en_pwm_mode", 0, value.None, nil)
		val = self.fields.Set_field(cast.ToString(self.diag_pin_field), 1, value.None, nil)
	}

	self.mcu_tmc.Set_register("GCONF", val, nil)
	if self.coolthrs == 0 {
		tc_val := self.fields.Set_field("tcoolthrs", 0xfffff, value.None, nil)
		self.mcu_tmc.Set_register("TCOOLTHRS", tc_val, nil)
	}

	return nil
}

func (self *TMCVirtualPinHelper) handle_homing_end(argv []interface{}) error {

	var val int64
	reg := self.fields.Lookup_register("en_pwm_mode", value.None)
	if value.IsNone(reg) {
		tp_val := self.fields.Set_field("tpwmthrs", self.pwmthrs, value.None, nil)
		self.mcu_tmc.Set_register("TPWMTHRS", tp_val, nil)
		val = self.fields.Set_field("en_spreadcycle", self.en_pwm, value.None, nil)
	} else {
		self.fields.Set_field("en_pwm_mode", self.en_pwm, value.None, nil)
		val = self.fields.Set_field(cast.ToString(self.diag_pin_field), 0, value.None, nil)
	}

	self.mcu_tmc.Set_register("GCONF", val, nil)
	tc_val := self.fields.Set_field("tcoolthrs", 0, value.None, nil)
	self.mcu_tmc.Set_register("TCOOLTHRS", tc_val, nil)

	return nil
}

/**
######################################################################
# Config reading helpers
######################################################################
*/

// Helper to initialize the wave table from config or defaults
func TMCWaveTableHelper(config *ConfigWrapper, mcu_tmc IMCU_TMC) {
	set_config_field := mcu_tmc.Get_fields().Set_config_field
	set_config_field(config, "mslut0", int64(0xAAAAB554))
	set_config_field(config, "mslut1", int64(0x4A9554AA))
	set_config_field(config, "mslut2", int64(0x24492929))
	set_config_field(config, "mslut3", int64(0x10104222))
	set_config_field(config, "mslut4", int64(0xFBFFFFFF))
	set_config_field(config, "mslut5", int64(0xB5BB777D))
	set_config_field(config, "mslut6", int64(0x49295556))
	set_config_field(config, "mslut7", int64(0x00404222))
	set_config_field(config, "w0", 2)
	set_config_field(config, "w1", 1)
	set_config_field(config, "w2", 1)
	set_config_field(config, "w3", 1)
	set_config_field(config, "x1", 128)
	set_config_field(config, "x2", 255)
	set_config_field(config, "x3", 255)
	set_config_field(config, "start_sin", 0)
	set_config_field(config, "start_sin90", 247)
}

// Helper to configure and query the microstep settings
func TMCMicrostepHelper(config *ConfigWrapper, mcu_tmc IMCU_TMC) error {
	fields := mcu_tmc.Get_fields()
	stepper_name := strings.Join(strings.Split(config.Get_name(), " ")[1:], " ")
	if !config.Has_section(stepper_name) {
		return fmt.Errorf("Could not find config section '[%s]' required by tmc driver", stepper_name)
	}

	stepper_config := config.Getsection(stepper_name)
	ms_config := config.Getsection(stepper_name)

	if value.IsNone(stepper_config.Get("microsteps", value.None, false)) &&
		value.IsNotNone(config.Get("microsteps", value.None, false)) {
		// Older config format with microsteps in tmc config section
		ms_config = config
	}

	steps := map[interface{}]interface{}{256: 0, 128: 1, 64: 2, 32: 3, 16: 4, 8: 5, 4: 6, 2: 7, 1: 8}
	mres := ms_config.Getchoice("microsteps", steps, nil, true)
	fields.Set_field("mres", mres, value.None, nil)
	fields.Set_field("intpol", config.Getboolean("interpolate", true, true), value.None, nil)
	return nil
}

func TMCtstepHelper(step_dist float64, mres int, tmc_freq, velocity float64) int {
	if velocity > 0. {
		shift := 1 << mres
		step_dist_256 := step_dist / float64(shift)
		threshold := int(tmc_freq*step_dist_256/velocity + .5)
		return maths.Max(0, maths.Min(0xfffff, threshold))
	} else {
		return 0xfffff
	}

}

// Helper to configure "stealthchop" mode
func TMCStealthchopHelper(config *ConfigWrapper, mcu_tmc IMCU_TMC, tmc_freq float64) {
	fields := mcu_tmc.Get_fields()
	en_pwm_mode := false
	velocity := config.Getfloat("stealthchop_threshold", math.NaN(), 0., 0, 0, 0, true)
	tpwmthrs := 0xfffff

	if math.IsNaN(velocity) == false {
		en_pwm_mode = true
		stepper_name := strings.Join(strings.Split(config.Get_name(), " ")[1:], " ")
		sconfig := config.Getsection(stepper_name)
		rotation_dist, steps_per_rotation := Parse_step_distance(sconfig, nil, true)
		step_dist := rotation_dist / float64(steps_per_rotation)
		mres := fields.Get_field("mres", value.None, nil)
		tpwmthrs = TMCtstepHelper(step_dist, int(mres), tmc_freq, velocity)
	}
	fields.Set_field("tpwmthrs", tpwmthrs, value.None, nil)

	reg := fields.Lookup_register("en_pwm_mode", value.None)
	if value.IsNone(reg) == false {
		fields.Set_field("en_pwm_mode", en_pwm_mode, value.None, nil)
	} else {
		// TMC2208 uses en_spreadCycle
		fields.Set_field("en_spreadcycle", !en_pwm_mode, value.None, nil)
	}
}
