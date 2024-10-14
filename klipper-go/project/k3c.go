package project

import (
	"errors"
	"flag"
	"fmt"
	"k3c/common/constants"
	"k3c/common/pprof"
	"k3c/common/utils/object"
	"k3c/common/utils/sys"
	"k3c/common/value"
	"k3c/project/util"
	"log"
	"runtime/debug"

	"os"
	"runtime"
	"strings"
	"time"
)

const (
	message_ready   = "Printer is ready"
	message_startup = "Printer is not ready\nThe project host software is attempting to connect.Please\nretry in a few moments."

	message_restart = "Once the underlying issue is corrected, use the \"RESTART\"\ncommand to reload the config and restart the host software.\nPrinter is halted"

	message_protocol_error1 = "This is frequently caused by running an older version of the\nfirmware on the MCU(s).Fix by recompiling and flashing the\nfirmware."

	message_protocol_error2 = "Once the underlying issue is corrected, use the \"RESTART\"\ncommand to reload the config and restart the host software."

	message_mcu_connect_error = "Once the underlying issue is corrected, use the\n\"FIRMWARE_RESTART\" command to reset the firmware, reload the\nconfig, and restart the host software.\nError configuring printer"

	message_shutdown = "Once the underlying issue is corrected, use the\n\"FIRMWARE_RESTART\" command to reset the firmware, reload the\nconfig, and restart the host software.Printer is shutdown"
	INFO             = 20
	DEBUG            = 10
)

type Printer struct {
	config_error      *Config_error
	Command_error     *CommandError
	Bglogger          *os.File
	Start_args        map[string]interface{}
	reactor           IReactor
	state_message     string
	in_shutdown_state bool
	run_result        string
	event_handlers    map[string][]func([]interface{}) error
	objects           map[string]interface{}
	Module            map[string]interface{}
	device_type       string
	print_size        string
}

func NewPrinter(main_reactor IReactor, bglogger *os.File, start_args map[string]interface{}) *Printer {
	self := Printer{Start_args: map[string]interface{}{}}
	self.config_error = &Config_error{}
	self.Command_error = &CommandError{}

	self.Bglogger = bglogger
	self.Start_args = start_args
	self.reactor = main_reactor
	self.reactor.Register_callback(self._connect, constants.NOW)
	self.state_message = message_startup
	self.in_shutdown_state = false
	self.run_result = ""
	self.event_handlers = map[string][]func([]interface{}) error{}
	self.objects = make(map[string]interface{})
	// Init printer components that must be setup prior to config
	for _, m := range []func(*Printer){Add_early_printer_objects1,
		Add_early_printer_objects_webhooks} {
		m(&self)
	}
	self.Module = LoadMainModule()
	return &self
}
func (self *Printer) Get_start_args() map[string]interface{} {
	return self.Start_args
}
func (self *Printer) Get_reactor() IReactor {
	return self.reactor
}
func (self *Printer) get_state_message() (string, string) {
	var category string
	if self.state_message == message_ready {
		category = "ready"
	} else if self.state_message == message_startup {
		category = "startup"
	} else if self.in_shutdown_state {
		category = "shutdown"
	} else {
		category = "error"
	}
	return self.state_message, category
}
func (self *Printer) Is_shutdown() bool {
	return self.in_shutdown_state
}
func (self *Printer) _set_state(msg string) {
	if self.state_message == message_ready || self.state_message == message_startup {
		self.state_message = msg
	}
	if msg != message_ready && self.Start_args["debuginput"] != nil {
		self.Request_exit("error_exit")
	}
}
func (self *Printer) Add_object(name string, obj interface{}) error {
	_, ok := self.objects[name]
	if ok {
		return errors.New(strings.Join([]string{"Printer object '", name, "' already created"}, ""))
	}
	self.objects[name] = obj
	return nil
}
func (self *Printer) Lookup_object(name string, default1 interface{}) interface{} {
	_, ok := self.objects[name]
	if ok {
		return self.objects[name]
	}
	if _, ok := default1.(object.Sentinel); ok {
		panic(strings.Join([]string{"Unknown config object '", name, "' "}, ""))
	}
	return default1
}
func (self *Printer) Lookup_objects(module string) []interface{} {
	mods := []interface{}{}
	if module == "" {
		for _, v := range self.objects {
			mods = append(mods, v)
		}
		return mods
	}
	prefix := module + " "
	for k, v := range self.objects {
		mod := map[string]interface{}{}
		if strings.HasPrefix(k, prefix) {
			mod[k] = v
			mods = append(mods, mod)
		}
	}
	obj, ok := self.objects[module]
	if ok {
		mod := map[string]interface{}{}
		mod[module] = obj
		mods = append(mods, mod)
	}
	return mods
}

func (self *Printer) load_object1(config *ConfigWrapper, section string) interface{} {
	for k, object := range self.objects {
		if k == section {
			return object
		}
	}

	if strings.HasSuffix(section, " default") {
		if _, ok := self.Module[strings.Split(section, " ")[0]]; !ok {
			value.StaticValue.Error.Printf("%s depend on %s, should loaded before", section, strings.Split(section, " ")[0])
		}
		value.StaticValue.Debug.Printf("%s only as config, don't use for load object", section)
		return self.Module[strings.Split(section, " ")[0]]
	}

	init_func, ok := self.Module[section]
	if ok {
		s := config.Getsection(section)
		self.objects[section] = init_func.(func(*ConfigWrapper) interface{})(s)
	} else {
		init_func, ok = self.Module[strings.Split(section, " ")[0]]
		if ok {
			s := config.Getsection(section)
			self.objects[section] = init_func.(func(*ConfigWrapper) interface{})(s)
		}
	}
	return self.objects[section]
}

func (self *Printer) reload_object(config *ConfigWrapper, section string) interface{} {
	module_parts := strings.Split(section, " ")
	_section := section
	if strings.HasPrefix(section, "gcode_macro") && len(module_parts) > 1 {
		_section = "gcode_macro_1"
	}
	init_func, ok := self.Module[_section]
	if ok {
		s := config.Getsection(section)
		self.objects[section] = init_func.(func(*ConfigWrapper) interface{})(s)
	} else {
		init_func, ok = self.Module[strings.Split(_section, " ")[0]]
		if ok {
			s := config.Getsection(_section)
			self.objects[section] = init_func.(func(*ConfigWrapper) interface{})(s)
		}
	}
	return self.objects[section]
}

func (self *Printer) Load_object(config *ConfigWrapper, section string, default1 interface{}) interface{} {
	obj := self.load_object1(config, section)
	if obj == nil {
		if _, ok := default1.(*object.Sentinel); ok {
			self.config_error.E = fmt.Sprintf("Unable to load module '%s'", section)
			//todo
			//panic(self.config_error)
			log.Print("moudle as ", section, " is not support")
		} else {
			if _, ok := default1.(object.Sentinel); ok {
				self.config_error.E = fmt.Sprintf("Unable to load module '%s'", section)
				//todo
				//panic(self.config_error)
				log.Print("moudle as ", section, " is not support")
			} else {
				return default1
			}

		}
	}
	return obj
}

func (self *Printer) _read_config() {
	pconfig := NewPrinterConfig(self)
	self.objects["configfile"] = pconfig

	config := pconfig.Read_main_config()
	self.device_type = config.Get("device_type", "", false).(string)
	self.print_size = config.Get("print_size", "", false).(string)
	if self.Bglogger != nil {
		pconfig.Log_config(*config)
	}
	// Create printer components
	for _, m := range []func(*ConfigWrapper){Add_printer_objects_pins,
		Add_printer_objects_mcu} {
		m(config)
	}

	for _, section_config := range config.Get_prefix_sections("") {
		self.Load_object(config, section_config.Get_name(), value.None)
	}
	for _, m := range []func(*ConfigWrapper){Add_printer_objects_toolhead, Load_config_tuning_tower} {
		m(config)
	}
	// Validate that there are no undefined parameters in the config file
	pconfig.Check_unused_options(config)
}
func (self *Printer) _build_protocol_error_message(e interface{}) string {
	host_version := self.Start_args["software_version"]
	msg_update := []string{}
	msg_updated := []string{}
	for _, m := range self.Lookup_objects("mcu") {
		mcu := m.(map[string]interface{})["mcu"].(*MCU)
		mcu_name := m.(map[string]interface{})["mcu_name"]
		mcu_version := mcu.Get_status(0)["mcu_version"]
		if mcu_version != host_version {
			msg_update = append(msg_update, fmt.Sprintf("%s: Current version %s", strings.TrimSpace(mcu_name.(string)), mcu_version))
		} else {
			msg_updated = append(msg_updated, fmt.Sprintf("%s: Current version %s", strings.TrimSpace(mcu_name.(string)), mcu_version))
		}
	}
	if len(msg_update) == 0 {
		msg_update = append(msg_update, "<none>")
	}
	if len(msg_updated) == 0 {
		msg_updated = append(msg_updated, "<none>")
	}
	msg := "MCU Protocol error"
	return strings.Join([]string{msg, "\n"}, "")
}
func (self *Printer) _connect(eventtime interface{}) interface{} {
	self.tryCatchConnect1()
	self.tryCatchConnect2()
	return nil
}
func (self *Printer) tryCatchConnect1() {
	defer func() {
		if err := recover(); err != nil {
			_, ok1 := err.(*PinError)
			_, ok2 := err.(*Config_error)
			if ok1 || ok2 {
				log.Println("Config error", err, string(debug.Stack()))
				self._set_state(fmt.Sprintf("%s\n%s", err, message_restart))
				return
			}
			_, ok11 := err.(*MsgprotoError)
			if ok11 {
				log.Print("Protocol error", string(debug.Stack()))
				self._set_state(self._build_protocol_error_message(err))
				util.Dump_mcu_build()
				return
			}
			e, ok22 := err.(*erro)
			if ok22 {
				log.Print("MCU error during connect", string(debug.Stack()))
				self._set_state(fmt.Sprintf("%s%s", e.err, message_mcu_connect_error))
				util.Dump_mcu_build()
				return
			}
			log.Printf("Unhandled exception during connect: %v, debug stack:\n%s\n", err, string(debug.Stack()))
			self._set_state(fmt.Sprintf("Internal error during connect: %s\n%s", err, message_restart))
			return
		}
	}()

	self._read_config()
	self.Send_event("project:mcu_identify", nil)
	cbs := self.event_handlers["project:connect"]
	for _, cb := range cbs {
		if self.state_message != message_startup {
			return
		}
		err := cb(nil)
		if err != nil {
			log.Println("Config error: ", err)
			self._set_state(fmt.Sprintf("%s\n%s", err.Error(), message_restart))
			return
		}
	}
}
func (self *Printer) tryCatchConnect2() {
	defer func() {
		if err := recover(); err != nil {
			log.Println("Unhandled exception during ready callback", err)
			self.Invoke_shutdown(fmt.Sprintf("Internal error during ready callback: %s", err))
			return
		}
	}()
	self._set_state(message_ready)
	cbs := self.event_handlers["project:ready"]
	for _, cb := range cbs {
		if self.state_message != message_ready {
			return
		}
		err := cb(nil)
		if err != nil {
			log.Println("Unhandled exception during ready callback")
			self._set_state(fmt.Sprintf("Internal error during ready callback:%s", err.Error()))
			return
		}
	}
}
func (self *Printer) Run() string {
	systime := float64(time.Now().UnixNano()) / 1000000000
	monotime := self.reactor.Monotonic()
	log.Printf("Start printer at %s (%.1f %.1f)",
		time.Now().String(), systime, monotime)
	// Enter main reactor loop

	err := self.reactor.Run()
	if err != nil {
		msg := "Unhandled exception during run"
		log.Println(msg)
		self.reactor.Register_callback(self.Invoke_shutdown, constants.NOW)
		err = self.reactor.Run()
		if err != nil {
			log.Println("Repeat unhandled exception during run")
			self.run_result = "error_exit"
		}
	}
	run_result := self.run_result
	if run_result == "firmware_restart" {
		_, err := self.Send_event("project:firmware_restart", nil)
		if err != nil {
			log.Println("Unhandled exception during post run")
			return run_result
		}
	}

	_, err = self.Send_event("project:disconnect", nil)
	if err != nil {
		log.Println("Unhandled exception during post run")
		return run_result
	}
	return run_result

}
func (self *Printer) Set_rollover_info(name, info string, isLog bool) {
	if isLog {
		log.Println(info)
	}
}
func (self *Printer) Invoke_shutdown(msg interface{}) interface{} {
	if self.in_shutdown_state {
		return nil
	}
	log.Printf("Transition to shutdown state: %s", msg)
	self.in_shutdown_state = true
	self._set_state(strings.Join([]string{msg.(string), message_shutdown}, ""))
	for _, cb := range self.event_handlers["project:shutdown"] {

		err := cb(nil)
		if err != nil {
			log.Println("Exception during shutdown handler")
		}
		value.StaticValue.Debug.Printf("Reactor garbage collection: %v", self.reactor.Get_gc_stats())
	}
	return nil
}

func (self *Printer) invoke_async_shutdown(msg string) {
	_func := func(argv interface{}) interface{} {
		self.Invoke_shutdown(msg)
		return nil
	}
	self.reactor.Register_async_callback(_func, constants.NOW)
}
func (self *Printer) Register_event_handler(event string, callback func([]interface{}) error) {
	list, ok := self.event_handlers[event]
	if !ok {
		list = []func([]interface{}) error{}
	}
	self.event_handlers[event] = append(list, callback)
}
func (self *Printer) Send_event(event string, params []interface{}) ([]interface{}, error) {
	ret := []interface{}{}
	cbs, ok := self.event_handlers[event]
	if ok {
		for i := 0; i < len(cbs); i++ {
			cb := cbs[i]
			ret = append(ret, (cb)(params))
		}
	}
	return ret, nil
}
func (self *Printer) Request_exit(result string) {
	if self.run_result == "" {
		self.run_result = result
	}
	self.reactor.End()
}
func ModuleK3C() {

}

type OptionParser struct {
	Debuginput  string
	Inputtty    string
	Apiserver   string
	Logfile     string
	Verbose     bool
	Debugoutput string
	Dictionary  string
	Import_test bool
}

type K3C struct {
}

func NewK3C() *K3C {
	K3C := K3C{}

	return &K3C
}

func (self K3C) Main() {
	usage := "[options] <your config file>"
	flag.Usage = func() {
		fmt.Println(os.Args[0], usage)
	}
	options := OptionParser{}
	flag.StringVar(&options.Debuginput, "i", "", "read commands from file instead of from tty port")
	flag.StringVar(&options.Inputtty, "I", "/tmp/printer", "input tty name (default is /tmp/printer)")
	flag.StringVar(&options.Apiserver, "a", "/tmp/unix_uds", "api server unix domain socket filename")
	flag.StringVar(&options.Logfile, "l", "", "write log to file instead of stderr")
	flag.BoolVar(&options.Verbose, "v", true, "enable debug messages")
	flag.StringVar(&options.Debugoutput, "o", "", "write output to file instead of to serial port")
	flag.StringVar(&options.Dictionary, "d", "", "file to read for mcu protocol dictionary")
	flag.BoolVar(&options.Import_test, "import-test", false, "perform an import module test")
	flag.Parse()
	args := flag.Args()
	if options.Import_test {
		import_test()
	}
	start_args := map[string]interface{}{}
	if len(args) != 1 {
		flag.Usage()
		start_args["config_file"] = "./printer.cfg"
	} else {
		start_args["config_file"] = args[0]
	}

	start_args["apiserver"] = options.Apiserver
	start_args["start_reason"] = "startup"
	debuglevel := INFO
	if options.Verbose {
		debuglevel = DEBUG
	}
	if options.Debuginput != "" {
		start_args["debuginput"] = options.Debuginput
		debuginput, err := os.OpenFile(options.Debuginput, os.O_RDONLY, 0644)
		if err != nil {
			value.StaticValue.Error.Print(err.Error())
			os.Exit(3)
		}
		start_args["gcode_fd"] = debuginput.Fd()
	}

	if options.Debugoutput != "" {
		start_args["debugoutput"] = options.Debugoutput
	}
	var bglogger *os.File
	if options.Logfile != "" {
		start_args["log_file"] = options.Logfile
		logFile, err := os.OpenFile(options.Logfile, os.O_CREATE|os.O_WRONLY|os.O_APPEND, 0644)
		if err != nil {
			log.Panic("打开日志文件异常")
		}
		bglogger = logFile

	} else {
		bglogger = nil
		if debuglevel != DEBUG {
			value.StaticValue.Debug = log.New(nil, "", log.LstdFlags|log.Lshortfile|log.Lmsgprefix)
		}

	}
	log.Print("Starting K3C...")
	start_args["software_version"] = sys.GetSoftwareVersion()
	start_args["cpu_info"] = sys.GetCpuInfo()
	var versions string
	if bglogger != nil {
		versions = strings.Join([]string{"\n", "Args: ", strings.Join(flag.Args(), ""), "Git version: ", start_args["software_version"].(string),
			"CPU: ", start_args["cpu_info"].(string),
			"Go: "}, "")
		log.Println(versions)
	} else if options.Debugoutput == "" {
		log.Println("No log file specified! Severe timing issues may result!")
	}
	runtime.GC()

	if value.StaticValue.Config.PProf.Enable {
		value.StaticValue.Debug.Print("pprof enable")
		pprof.Run(value.StaticValue.Config.PProf.Addr, value.StaticValue.Config.PProf.Prefix)
	}
	var res string
	for {
		if bglogger != nil {
			log.Println(versions)
		}
		main_reactor := NewEPollReactor(true)
		printer := NewPrinter(main_reactor, bglogger, start_args)
		res = printer.Run()
		if res == "exit" || res == "error_exit" {
			break
		}
		time.Sleep(time.Second)
		log.Println("Restarting printer")
		start_args["start_reason"] = res
	}
	if bglogger != nil {
		bglogger.Close()
	}
	if res == "error_exit" {
		os.Exit(-1)
	}
	os.Exit(0)
}

func import_test() {
	os.Exit(0)
}
