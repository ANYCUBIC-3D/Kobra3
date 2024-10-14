package project

import (
	"bytes"
	"errors"
	"fmt"
	"io"
	"io/ioutil"
	"k3c/common/collections"
	"k3c/common/configparser"
	"k3c/common/utils/cast"
	"k3c/common/utils/file"
	"k3c/common/utils/object"
	"k3c/common/value"
	"os"
	"path/filepath"
	"reflect"
	"sort"
	"strconv"
	"syscall"
	"time"

	"log"
	"regexp"
	"strings"
)

const (
	AUTOSAVE_HEADER = `
#*# <---------------------- SAVE_CONFIG ---------------------->
#*# DO NOT EDIT THIS BLOCK OR BELOW. The contents are auto-generated.
#*#
`
)

type Config_error struct {
	E string
}
type ConfigWrapper struct {
	printer         *Printer
	fileconfig      *configparser.RawConfigParser
	access_tracking map[string]interface{}
	Section         string
}

func NewConfigWrapper(printer *Printer, fileconfig *configparser.RawConfigParser, access_tracking map[string]interface{}, section string) *ConfigWrapper {
	self := ConfigWrapper{}
	self.printer = printer
	self.fileconfig = fileconfig
	self.access_tracking = access_tracking
	self.Section = section

	return &self
}

func (self *ConfigWrapper) Fileconfig() *configparser.RawConfigParser {
	return self.fileconfig
}
func (self *ConfigWrapper) Get_printer() *Printer {
	return self.printer
}
func (self *ConfigWrapper) Get_name() string {
	return self.Section
}

// func (self *ConfigWrapper) _get_wrapper(parser, option string, default1, minval, maxval,
//
//		above, below string, note_valid bool) {
//
//		if not self.fileconfig.has_option(self.Section, option) {
//			if sentinel == nil {
//				if note_valid != nil {
//					acc_id := (self.Section.lower(), option.lower())
//					self.access_tracking[acc_id] = default1
//					return default
//	}
//
//		}
//		log.Printf("Option '%s' in section '%s' must be specified", option, self.Section)
//	}
//	v := parser(self.Section, option)
//	var n float64
//	if v == "" {
//		var err error
//		n, err = strconv.ParseFloat(v.(string), 64)
//		if err != nil {
//			log.Printf("Unable to parse option '%s' in section '%s'", option, self.Section)
//
//		}
//	}
//	if v == 0 {
//			n=
//	}
//
//	if note_valid {
//		acc_id := strings.Join([]string{strings.ToLower(self.Section), strings.ToLower(option)}, ":")
//		self.access_tracking[acc_id] = default1
//	}
//
//	if minval != nil && n < minval.(float64) {
//		log.Printf("Option '%s' in section '%s' must have minimum of %s", option, self.Section, minval)
//	}
//	if maxval != nil && n > maxval.(float64) {
//		log.Printf("Option '%s' in section '%s' must have maximum of %s", option, self.Section, maxval)
//	}
//	if above != nil && n <= above.(float64) {
//		log.Printf("Option '%s' in section '%s' must be above %s", option, self.Section, above)
//	}
//	if below != nil && n >= below.(float64) {
//		log.Printf("Option '%s' in section '%s' must be below %s", option, self.Section, below)
//	}
//	return v
//
// }
func (self *ConfigWrapper) Get(option string, default1 interface{}, note_valid bool) interface{} {
	if !self.fileconfig.Has_option(self.Section, option) {
		if object.IsNotSentinel(default1) {
			if note_valid && default1 != nil {
				acc_id := strings.Join([]string{strings.ToLower(self.Section), strings.ToLower(option)}, ":")
				self.access_tracking[acc_id] = default1
			}
			return default1
		}
		panic(fmt.Sprintf("Option '%s' in section '%s' must be specified", option, self.Section))
	}
	v := self.fileconfig.Get(self.Section, option)
	if note_valid {
		key := fmt.Sprintf("%s:%s", strings.ToLower(self.Section), strings.ToLower(option))
		self.access_tracking[key] = v
	}
	return v
}

func (self *ConfigWrapper) Getint(option string, default1 interface{}, minval, maxval int,
	note_valid bool) int {
	if !self.fileconfig.Has_option(self.Section, option) {
		if object.IsNotSentinel(default1) {
			if note_valid && default1 != nil {
				acc_id := strings.Join([]string{strings.ToLower(self.Section), strings.ToLower(option)}, ":")
				self.access_tracking[acc_id] = default1
			}
			ret, ok := default1.(int)
			if ok {
				return ret
			} else {
				return 0
			}
		}
		panic(fmt.Sprintf("Option '%s' in section '%s' must be specified", option, self.Section))
	}
	v := self.fileconfig.Getint(self.Section, option)
	n := v.(int)
	if minval != 0 && n < minval {
		panic(fmt.Errorf("Option '%s' in section '%s' must have minimum of %d", option, self.Section, minval))
	}
	if maxval != 0 && n > maxval {
		panic(fmt.Errorf("Option '%s' in section '%s' must have maximum of %d", option, self.Section, maxval))
	}
	return n
}
func (self *ConfigWrapper) Getint64(option string, default1 interface{}, minval, maxval int64,
	note_valid bool) int64 {
	if !self.fileconfig.Has_option(self.Section, option) {
		if object.IsNotSentinel(default1) {
			if note_valid && default1 != nil {
				acc_id := strings.Join([]string{strings.ToLower(self.Section), strings.ToLower(option)}, ":")
				self.access_tracking[acc_id] = default1
			}
			ret, ok := default1.(int64)
			if ok {
				return ret
			} else {
				return 0
			}
		}
		panic(fmt.Sprintf("Option '%s' in section '%s' must be specified", option, self.Section))
	}
	v := self.fileconfig.Getint(self.Section, option)
	n := v.(int64)
	if minval != 0 && n < minval {
		panic(fmt.Errorf("Option '%s' in section '%s' must have minimum of %d", option, self.Section, minval))
		return minval
	}
	if maxval != 0 && n > maxval {
		panic(fmt.Errorf("Option '%s' in section '%s' must have maximum of %d", option, self.Section, maxval))
		return maxval
	}
	return n
}

func (self *ConfigWrapper) GetintNone(option string, def interface{}, minval, maxval int,
	note_valid bool) interface{} {
	if !self.fileconfig.Has_option(self.Section, option) {
		if object.IsNotSentinel(def) {
			if note_valid && def != nil {
				acc_id := strings.Join([]string{strings.ToLower(self.Section), strings.ToLower(option)}, ":")
				self.access_tracking[acc_id] = def
			}
			return def
		}
		panic(fmt.Sprintf("Option '%s' in section '%s' must be specified", option, self.Section))
	}
	v := self.fileconfig.Getint(self.Section, option)
	n := cast.ToInt(v)
	if minval != 0 && n < minval {
		log.Printf("Option '%s' in section '%s' must have minimum of %d", option, self.Section, minval)
	}
	if maxval != 0 && n > maxval {
		log.Printf("Option '%s' in section '%s' must have maximum of %d", option, self.Section, maxval)
	}
	return n
}

func (self *ConfigWrapper) Getfloat(option string, default1 interface{}, minval, maxval,
	above, below float64, note_valid bool) float64 {

	if !self.fileconfig.Has_option(self.Section, option) {
		if object.IsNotSentinel(default1) {
			if note_valid && default1 != nil {
				acc_id := strings.Join([]string{strings.ToLower(self.Section), strings.ToLower(option)}, ":")
				self.access_tracking[acc_id] = default1
			}
			ret, ok := default1.(float64)
			if ok {
				return ret
			} else {
				return 0
			}
		}
		panic(fmt.Sprintf("Option '%s' in section '%s' must be specified", option, self.Section))
	}
	v := self.fileconfig.Getfloat(self.Section, option)
	n, ok := v.(float64)
	if !ok {
		panic(fmt.Sprintf("Unable to parse option '%s' in section '%s'", option, self.Section))
	}
	if note_valid {
		acc_id := strings.Join([]string{strings.ToLower(self.Section), strings.ToLower(option)}, ":")
		self.access_tracking[acc_id] = v
	}
	if minval != 0 && n < minval {
		panic(fmt.Sprintf("Option '%s' in section '%s' must have minimum of %f", option, self.Section, minval))
	}
	if maxval != 0 && n > maxval {
		panic(fmt.Sprintf("Option '%s' in section '%s' must have maximum of %f", option, self.Section, maxval))
	}
	if above != 0 && n <= above {
		panic(fmt.Sprintf("Option '%s' in section '%s' must be above %f", option, self.Section, above))
	}
	if below != 0 && n >= below {
		panic(fmt.Sprintf("Option '%s' in section '%s' must be below %f", option, self.Section, below))
	}

	return n
}

// GetfloatNone 如果返回nil，表明option不存在
func (self *ConfigWrapper) GetfloatNone(option string, default1 interface{}, minval, maxval,
	above, below float64, note_valid bool) interface{} {

	if !self.fileconfig.Has_option(self.Section, option) {
		if object.IsNotSentinel(default1) {
			if note_valid && default1 != nil {
				acc_id := strings.Join([]string{strings.ToLower(self.Section), strings.ToLower(option)}, ":")
				self.access_tracking[acc_id] = default1
			}
			return default1
		}
		panic(fmt.Sprintf("Option '%s' in section '%s' must be specified", option, self.Section))
	}
	v := self.fileconfig.Getfloat64None(self.Section, option)
	if v == nil {
		return nil
	}
	if note_valid {
		acc_id := strings.Join([]string{strings.ToLower(self.Section), strings.ToLower(option)}, ":")
		self.access_tracking[acc_id] = v
	}
	n := cast.ToFloat64(v)

	if minval != 0 && n < minval {
		log.Printf("Option '%s' in section '%s' must have minimum of %f", option, self.Section, minval)
	}
	if maxval != 0 && n > maxval {
		log.Printf("Option '%s' in section '%s' must have maximum of %f", option, self.Section, maxval)
	}
	if above != 0 && n <= above {
		log.Printf("Option '%s' in section '%s' must be above %f", option, self.Section, above)
	}
	if below != 0 && n >= below {
		log.Printf("Option '%s' in section '%s' must be below %f", option, self.Section, below)
	}
	return v
}

func (self *ConfigWrapper) Getboolean(option string, default1 interface{}, note_valid bool) bool {
	if !self.fileconfig.Has_option(self.Section, option) {
		if object.IsNotSentinel(default1) {
			if note_valid && default1 != nil {
				acc_id := strings.Join([]string{strings.ToLower(self.Section), strings.ToLower(option)}, ":")
				self.access_tracking[acc_id] = default1
			}
			ret, ok := default1.(bool)
			if ok {
				return ret
			} else {
				return false
			}
		}
		panic(fmt.Sprintf("Option '%s' in section '%s' must be specified", option, self.Section))
	}
	v := self.fileconfig.Getboolean(self.Section, option)
	return v.(bool)
}

func (self *ConfigWrapper) Getchoice(option string, choices map[interface{}]interface{}, default1 interface{}, note_valid bool) interface{} {
	if !self.fileconfig.Has_option(self.Section, option) {
		if object.IsNotSentinel(default1) {
			if note_valid && default1 != nil {
				acc_id := strings.Join([]string{strings.ToLower(self.Section), strings.ToLower(option)}, ":")
				self.access_tracking[acc_id] = default1
			}
			ret, ok := default1.(string)
			if ok {
				return ret
			} else {
				return ""
			}
		}
		panic(fmt.Sprintf("Option '%s' in section '%s' must be specified", option, self.Section))
	}
	var c interface{}
	for k, _ := range choices {
		if reflect.TypeOf(k).Kind() == reflect.Int {
			c = self.Getint(option, default1, 0, 0, true)
		} else {
			c = self.Get(option, default1, true)
		}
	}
	ret, ok := choices[c]
	if !ok {
		log.Printf("Choice '%s' for option '%s' in section '%s' is not a valid choice", c, option, self.Section)
	}
	return ret

}
func (self *ConfigWrapper) Getlists(option string, default1 interface{}, seps []string, count int, kind reflect.Kind, note_valid bool) interface{} {
	if !self.fileconfig.Has_option(self.Section, option) {
		if object.IsNotSentinel(default1) {
			if note_valid && default1 != nil {
				acc_id := strings.Join([]string{strings.ToLower(self.Section), strings.ToLower(option)}, ":")
				self.access_tracking[acc_id] = default1
			}
			return default1
		}
		panic(fmt.Sprintf("Option '%s' in section '%s' must be specified", option, self.Section))
	}
	if len(seps) == 2 && seps[0] == "," && seps[1] == "\n" {
		ret := [][]interface{}{}
		value := self.fileconfig.Get(self.Section, option)
		str, ok := value.(string)
		if ok {
			str = strings.TrimSpace(str)
			//str = strings.ReplaceAll(str, "", "")
			strs := strings.Split(str, "\n")
			for _, s := range strs {
				valArr := strings.Split(s, ",")
				item := []interface{}{}
				for _, valStr := range valArr {
					var val interface{}
					switch kind {
					case reflect.Int:
						val, _ = strconv.Atoi(strings.TrimSpace(valStr))
					case reflect.Float64:
						val, _ = strconv.ParseFloat(strings.TrimSpace(valStr), 64)
					}
					item = append(item, val)
				}
				ret = append(ret, item)
			}
		}
		return ret
	} else {
		ret := []interface{}{}
		value := self.fileconfig.Get(self.Section, option)
		for _, sep := range seps {
			str, ok := value.(string)
			if ok {
				str = strings.TrimSpace(str)
				str = strings.ReplaceAll(str, "\t", "")
				strs := strings.Split(str, sep)
				var val interface{}
				for _, s := range strs {
					switch kind {
					case reflect.Int:
						val, _ = strconv.Atoi(s)
					case reflect.Float64:
						val, _ = strconv.ParseFloat(s, 64)
					}
					ret = append(ret, val)
				}
			}
		}
		return ret
	}
}

func fcparser(section, option string) {
	//	return lparser(self.fileconfig.get(section, option), len(seps)-1)
	//}
	//return self._get_wrapper(fcparser, option, default,note_valid = note_valid)
}
func (self *ConfigWrapper) getlist(option string, default1 interface{}, sep string, count int, note_valid bool) interface{} {
	if !self.fileconfig.Has_option(self.Section, option) {
		if object.IsNotSentinel(default1) {
			if note_valid && default1 != nil {
				acc_id := strings.Join([]string{strings.ToLower(self.Section), strings.ToLower(option)}, ":")
				self.access_tracking[acc_id] = default1
			}
			return default1
		}
		panic(fmt.Sprintf("Option '%s' in section '%s' must be specified", option, self.Section))
	}
	ret := []interface{}{}

	value := self.fileconfig.Get(self.Section, option)
	str, ok := value.(string)
	if ok {
		strs := strings.Split(str, sep)
		for _, s := range strs {
			ret = append(ret, s)
		}
	}
	for i := 0; i < count-len(ret); i++ {
		ret = append(ret, 0)
	}
	return ret
}
func (self *ConfigWrapper) Getintlist(option string, default1 interface{}, sep string, count int,
	note_valid bool) []int {
	if !self.fileconfig.Has_option(self.Section, option) {
		if object.IsNotSentinel(default1) {
			if note_valid && default1 != nil {
				acc_id := strings.Join([]string{strings.ToLower(self.Section), strings.ToLower(option)}, ":")
				self.access_tracking[acc_id] = default1
			}
			ret, ok := default1.([]int)
			if ok {
				return ret
			} else {
				return nil
			}
		}
		panic(fmt.Sprintf("Option '%s' in section '%s' must be specified", option, self.Section))
	}
	ret := []int{}

	value := self.fileconfig.Get(self.Section, option)
	str, ok := value.(string)
	if ok {
		strs := strings.Split(str, sep)
		for _, s := range strs {
			retI, err := strconv.Atoi(strings.TrimSpace(s))
			if err != nil {
				log.Print(err.Error())
			} else {
				ret = append(ret, retI)
			}
		}
	}
	for i := 0; i < count-len(ret); i++ {
		ret = append(ret, 0)
	}
	return ret
}
func (self *ConfigWrapper) Getfloatlist(option string, default1 interface{}, sep string, count int,
	note_valid bool) []float64 {
	if !self.fileconfig.Has_option(self.Section, option) {
		if object.IsNotSentinel(default1) {
			if note_valid && default1 != nil {
				acc_id := strings.Join([]string{strings.ToLower(self.Section), strings.ToLower(option)}, ":")
				self.access_tracking[acc_id] = default1
			}
			ret, ok := default1.([]float64)
			if ok {
				return ret
			} else {
				return nil
			}
		}
		panic(fmt.Sprintf("Option '%s' in section '%s' must be specified", option, self.Section))
	}
	ret := []float64{}

	value := self.fileconfig.Get(self.Section, option)
	str, ok := value.(string)
	if ok {
		strs := strings.Split(str, sep)
		for _, s := range strs {
			retI, err := strconv.ParseFloat(strings.TrimSpace(s), 64)
			if err != nil {
				log.Print(err.Error())
			} else {
				ret = append(ret, retI)
			}
		}
	}
	for i := 0; i < count-len(ret); i++ {
		ret = append(ret, 0)
	}
	return ret
}
func (self *ConfigWrapper) Getsection(section string) *ConfigWrapper {
	return &ConfigWrapper{printer: self.printer, fileconfig: self.fileconfig,
		access_tracking: self.access_tracking, Section: section}
}
func (self *ConfigWrapper) Has_section(section string) bool {

	return self.fileconfig.Has_section(section)
}
func (self *ConfigWrapper) Get_prefix_sections(prefix string) []*ConfigWrapper {
	configs := []*ConfigWrapper{}
	for _, s := range self.fileconfig.Sections() {
		if strings.HasPrefix(s, prefix) {
			configs = append(configs, self.Getsection(s))
		}
	}
	return configs
}

func (self *ConfigWrapper) Get_prefix_options(prefix string) []string {
	options, _ := self.fileconfig.Options(self.Section)
	prefixOpts := []string{}

	for o := range options {
		if prefix == "variable_" {
			if strings.HasPrefix(o, prefix) {
				prefixOpts = append(prefixOpts, o)
			}
		} else {

		}
	}

	return prefixOpts
}

func (self *ConfigWrapper) Deprecate(option, value string) {
	if !self.fileconfig.Has_option(self.Section, option) {
		return
	}
	msg := ""
	if value == "" {
		msg = fmt.Sprintf("Option '%s' in section '%s' is deprecated.", option, self.Section)
	} else {
		msg = fmt.Sprintf("Value '%s' in option '%s' in section '%s' is deprecated.", value, option, self.Section)
	}
	pconfig := self.printer.Lookup_object("configfile", object.Sentinel{}).(PrinterConfig)
	pconfig.Deprecate(self.Section, option, value, msg)
}

/** <---------------------- SAVE_CONFIG ---------------------->
#*# DO NOT EDIT THIS BLOCK OR BELOW. The contents are auto-generated.
**/

type PrinterConfig struct {
	printer             *Printer
	autosave            *ConfigWrapper
	deprecated          map[string]interface{}
	status_raw_config   map[string]interface{}
	status_save_pending map[string]interface{}
	status_settings     map[string]interface{}

	status_warnings     []interface{}
	save_config_pending bool
	comment_r           *regexp.Regexp
	value_r             *regexp.Regexp
}

func NewPrinterConfig(printer *Printer) *PrinterConfig {
	self := PrinterConfig{}
	self.comment_r = regexp.MustCompile("[#;].*$")
	self.value_r = regexp.MustCompile("[^A-Za-z0-9_].*$")
	self.printer = printer
	self.autosave = nil
	self.deprecated = map[string]interface{}{}
	self.status_raw_config = map[string]interface{}{}
	self.status_save_pending = map[string]interface{}{}

	self.status_settings = map[string]interface{}{}

	self.status_warnings = []interface{}{}
	self.save_config_pending = false
	gcode1 := self.printer.Lookup_object("gcode", object.Sentinel{})
	//if err != nil {
	//	log.Println(err.Error())
	//}
	//log.Println(gcode1)
	gcode1.(*GCodeDispatch).Register_command("SAVE_CONFIG", self.cmd_SAVE_CONFIG, false, cmd_SAVE_CONFIG_help)
	wh := MustLookupWebhooks(self.printer)
	wh.Register_endpoint("config/save_config", self.save_config)
	return &self
}
func (self *PrinterConfig) Get_printer() *Printer {
	return self.printer
}
func (self *PrinterConfig) _read_config_file(filename string) (string, error) {
	bs, err := file.GetBytes(filename)
	if err != nil {
		log.Printf("Unable to open config file %s", filename)
		return "", err
	}
	data := string(bs)
	return strings.ReplaceAll(data, "\r\n", "\n"), nil
}

func (self *PrinterConfig) _find_autosave_data(data string) (string, string) {
	regular_data := data
	autosave_data := ""
	pos := strings.Index(data, AUTOSAVE_HEADER)
	if pos >= 0 {
		regular_data = data[:pos]
		autosave_data = strings.TrimSpace(data[pos+len(AUTOSAVE_HEADER):])
	}
	// Check for errors and strip line prefixes
	if strings.Index(regular_data, "\n#*# ") != -1 {
		value.StaticValue.Debug.Printf("Can't read autosave from config file - autosave state corrupted")
		return data, ""
	}
	out := []string{""}
	lines := strings.Split(autosave_data, "\n")
	for _, line := range lines {

		if (!strings.HasPrefix(line, "#*#") ||
			(len(line) >= 4 && !strings.HasPrefix(line, "#*# "))) && autosave_data == "" {

			log.Println("Can't read autosave from config file - modifications after header")
			return data, ""
		}
		if len(line) > 4 { //处理数组越界
			out = append(out, line[4:])
		} else {
			out = append(out, "")
		}
	}
	out = append(out, "")
	return regular_data, strings.Join(out, "\n")
}

// comment_r = re.compile('[#;].*$')   已在初始化方法中处理了
// value_r = re.compile('[^A-Za-z0-9_].*$')
func (self *PrinterConfig) _strip_duplicates(data string, config *ConfigWrapper) string {
	fileconfig := config.Fileconfig()
	//Comment out fields in 'data' that are defined in 'config'
	lines := strings.Split(data, "\n")
	section := ""
	is_dup_field := false
	for lineno, line := range lines {
		//log.Println(lineno, line)
		pruned_line := strings.Split(line, "#")[0] //去掉注释
		if pruned_line == "" {
			continue
		}
		if pruned_line[0] == ' ' || pruned_line[0] == '\t' {
			if is_dup_field {
				lines[lineno] = "#" + line
			}
			continue
		}
		is_dup_field = false
		pruned_line = strings.TrimSpace(pruned_line)
		if pruned_line[0] == '[' {
			section = pruned_line[1 : len(pruned_line)-1]
			continue
		}
		field := strings.TrimSpace(strings.Split(pruned_line, "=")[0]) //取字段
		if fileconfig.Has_option(section, field) {
			is_dup_field = true
			lines[lineno] = "#" + line
		}
	}
	return strings.Join(lines, "\n")
}

func (self *PrinterConfig) _parse_config_buffer(buffer []string, filename string, fileconfig *configparser.RawConfigParser) {
	if buffer == nil || len(buffer) == 0 {
		return
	}
	fileconfig.Readfp(strings.NewReader(strings.Join(buffer, "\n")), filename)
}

// func (self *PrinterConfig) _resolve_include(source_filename, include_spec, fileconfig,
//
//		visited string) {
//		dirname = os.path.dirname(source_filename)
//		include_spec = include_spec.strip()
//		include_glob = os.path.join(dirname, include_spec)
//		include_filenames = glob.glob(include_glob)
//		if not include_filenames
//		and
//		not
//		glob.has_magic(include_glob)
//		{
//			// Empty set is OK if wildcard but not for direct file reference
//			raise
//			error("Include file '%s' does not exist" % (include_glob, ))
//		}
//		include_filenames.sort()
//		for include_filename
//			in
//		include_filenames {
//			include_data = self._read_config_file(include_filename)
//			self._parse_config(include_data, include_filename, fileconfig,
//				visited)
//		}
//		return include_filenames
//	}
func (self *PrinterConfig) _parse_config(data, filename string, fileconfig *configparser.RawConfigParser, visited map[string]string) {
	path, err := filepath.Abs(filename)
	if err != nil {
		log.Print(err.Error())
	}
	_, ok := visited[path]
	if ok {
		panic(fmt.Sprintf("Recursive include of config file '%s'", filename))
	}
	visited[path] = ""
	lines := strings.Split(data, "\n")
	//	 Buffer lines between includes and parse as a unit so that overrides
	//	 in includes apply linearly as they do within a single file
	buffer := []string{}
	for _, line := range lines {
		// Strip trailing comment
		pos := strings.Index(line, "#")
		if pos >= 0 {
			line = line[:pos]
		}
		// Process include or buffer line
		header := line
		if len(header) != 0 && strings.HasPrefix(header, "include ") {
			//			self._parse_config_buffer(buffer, filename, fileconfig)
			//			include_spec = header[8:].strip()
			//			self._resolve_include(filename, include_spec, fileconfig,
			//				visited)
		} else {
			buffer = append(buffer, line)
		}
	}
	self._parse_config_buffer(buffer, filename, fileconfig)
	//	visited.remove(path)
}
func (self *PrinterConfig) _build_config_wrapper(data, filename string) *ConfigWrapper {
	//if sys.version_info.major >= 3{
	//fileconfig = configparser.RawConfigParser(
	//strict = False, inline_comment_prefixes = (';', '#'))
	//}else{
	fileconfig := configparser.NewRawConfigParser()
	//}
	self._parse_config(data, filename, fileconfig, map[string]string{})
	return NewConfigWrapper(self.printer, fileconfig, map[string]interface{}{}, "printer")
}

//	func (self *PrinterConfig)  _build_config_string( config ConfigWrapper) {
//		sfile = io.StringIO()
//		config.fileconfig.write(sfile)
//		return sfile.getvalue().strip()
//	}
func (self *PrinterConfig) Read_config(filename string) *ConfigWrapper {
	data, _ := self._read_config_file(filename)
	return self._build_config_wrapper(data,
		filename)
}
func (self *PrinterConfig) Read_main_config() *ConfigWrapper {
	filename := self.printer.Get_start_args()["config_file"]
	data, err := self._read_config_file(filename.(string))
	if err != nil {
		panic("use mutable config _read_config_file: " + err.Error())
	}
	regular_data, autosave_data0 := self._find_autosave_data(data)
	path := filepath.Dir(self.printer.Get_start_args()["config_file"].(string))
	mutablecfg, err := configparser.NewMutableConfig(filepath.Join(path, configparser.MutableConfigFileName), configparser.EncryptNone, false)
	if err != nil {
		panic("use mutable config mutable new error: " + err.Error())
	}
	autosave_data := mutablecfg.ToInIString()
	if autosave_data == "" {
		autosave_data = autosave_data0
	}
	//regular_config := self._build_config_wrapper(regular_data, filename.(string))
	//autosave_data = self._strip_duplicates(autosave_data, regular_config)
	self.autosave = self._build_config_wrapper(autosave_data, filename.(string))
	cfg := self._build_config_wrapper(regular_data+"\n"+autosave_data, filename.(string))
	return cfg
}

func (self *PrinterConfig) Check_unused_options(config *ConfigWrapper) {
	/*fileconfig := config.fileconfig
	objects := self.printer.Lookup_objects("")
	// Determine all the fields that have been accessed
	access_tracking := config.access_tracking
	for _,section := range self.autosave.fileconfig.Sections() {
		options, err := self.autosave.fileconfig.Options(section)
		if err != nil {
			panic(err)
		}
		for _, option := range options {
			key := fmt.Sprintf("%s:%s", strings.ToLower(section), strings.ToLower(option))
			access_tracking[key] = 1
		}
	}
	// Validate that there are no undefined parameters in the config file
	valid_sections := make(map[string]int)
	for s, _ := range access_tracking {
		valid_sections[s] = 1
	}
	for _, section_name := range fileconfig.Sections() {
		section := strings.ToLower(section_name)
		_, ok := valid_sections[section]
		ok1 := false
		for _, item := range objects {
			val := item.(map[string]interface{})
			if _, ok2 := val[section]; ok2 {
				ok1 = true
				break
			}
		}
		if !ok && !ok1 {
			panic(fmt.Sprintf("Section '%s' is not a valid config section", section))
		}
		options, err := fileconfig.Options(section_name)
		if err != nil {
			panic(err)
		}
		for _, option := range options {
			option = strings.ToLower(option)
			key := fmt.Sprintf("%s:%s", section, option)
			if _, ok3 := access_tracking[key]; !ok3 {
				panic(fmt.Sprintf("Option '%s' is not valid in section '%s'", option, section))
			}
		}
	}*/
	// Setup get_status()
	self._build_status(config)
}
func (self *PrinterConfig) Log_config(config ConfigWrapper) {
	//lines = ["===== Config file =====",
	//self._build_config_string(config),
	//"======================="]
	//self.printer.set_rollover_info("config", "\n".join(lines))
}

// // Status reporting
func (self *PrinterConfig) Deprecate(section, option, value, msg string) {
	key := fmt.Sprintf("%s:%s:%s", section, option, value)
	self.deprecated[key] = msg
}

func (self *PrinterConfig) _build_status(config *ConfigWrapper) {
	self.status_raw_config = make(map[string]interface{})
	for _, section := range config.Get_prefix_sections("") {
		section_status := make(map[string]interface{})
		self.status_raw_config[section.Get_name()] = section_status
		for _, option := range section.Get_prefix_options("") {
			section_status[option] = section.Get(option, object.Sentinel{}, false)
		}
	}
	self.status_settings = make(map[string]interface{})
	for k, val := range config.access_tracking {
		section := strings.Split(k, ":")[0]
		option := strings.Split(k, ":")[1]
		item, ok := self.status_settings[section]
		if ok {
			item.(map[string]interface{})[option] = val
		} else {
			item = make(map[string]interface{})
			item.(map[string]interface{})[option] = val
			self.status_settings[section] = item
		}
	}
	self.status_warnings = []interface{}{}
	for k, msg := range self.deprecated {
		section := strings.Split(k, ":")[0]
		option := strings.Split(k, ":")[1]
		val := strings.Split(k, ":")[2]
		res := make(map[string]string)
		if val == "" {
			res["type"] = "deprecated_option"
		} else {
			res["type"] = "deprecated_value"
			res["value"] = "value"
		}
		res["message"] = msg.(string)
		res["section"] = section
		res["option"] = option
		self.status_warnings = append(self.status_warnings, res)
	}
}

func (self *PrinterConfig) Get_status(eventtime float64) map[string]interface{} {
	ret := map[string]interface{}{}

	ret["config"] = self.status_raw_config
	ret["settings"] = self.status_settings
	ret["warnings"] = self.status_warnings
	ret["save_config_pending"] = self.save_config_pending
	ret["save_config_pending_items"] = self.status_save_pending

	return ret

}

// Set Autosave functions
func (self *PrinterConfig) Set(section, option, val string) {
	if !self.autosave.fileconfig.Has_section(section) {
		self.autosave.fileconfig.Add_section(section)
	}

	self.autosave.fileconfig.Set(section, option, val)
	pending := self.status_save_pending
	if _, ok := pending[section]; !ok || value.IsNone(pending[section]) {
		pending[section] = make(map[string]interface{})
	}

	opts := pending[section].(map[string]interface{})
	opts[option] = val
	pending[section] = opts
	self.status_save_pending = pending
	self.save_config_pending = true
	value.StaticValue.Debug.Printf("save_config: set [%s] %s = %s", section, option, val)
}

func (self *PrinterConfig) Remove_section(section string) {
	if self.autosave.fileconfig.Has_section(section) {
		self.autosave.fileconfig.Remove_section(section)
		delete(self.status_save_pending, section)
		self.save_config_pending = true
	} else if _, ok := self.status_save_pending[section]; ok {
		delete(self.status_save_pending, section)
		self.save_config_pending = true
	}
}

func (self *PrinterConfig) _disallow_include_conflicts(regular_data, cfgname string, gcode interface{}) error {
	config := self._build_config_wrapper(regular_data, cfgname)
	for _, section := range self.autosave.fileconfig.Sections() {
		options, err := self.autosave.fileconfig.Options(section)
		if err != nil {
			log.Println(err.Error())
			continue
		}
		for _, option := range options {
			if config.fileconfig.Has_option(section, option) {
				msg := fmt.Sprintf("SAVE_CONFIG section '%s' option '%s' conflicts with included value", section, option)
				return errors.New(msg)
			}
			//
		}
	}
	return nil
}

const cmd_SAVE_CONFIG_help = "Overwrite config file and restart"

func (self *PrinterConfig) cmd_SAVE_CONFIG(argv interface{}) error {
	value.StaticValue.Debug.Print("start save config")
	if self.autosave.fileconfig.Sections() == nil {
		value.StaticValue.Debug.Print("skip save config because autosave sections is nil")
		return nil
	}
	gcode := MustLookupGcode(self.printer)
	// Create string containing autosave data
	autosave_data_temp := self._build_config_string(self.autosave)
	autosave_databytes, _ := ioutil.ReadAll(autosave_data_temp)

	l := strings.Split(strings.TrimRight(string(autosave_databytes), "\n"), "\n")
	for i := 0; i < len(l); i++ {
		l[i] = strings.Join([]string{"#*# ", l[i]}, "")
	}
	autosave_data := strings.Join(l, "\n")
	// Read in and validate current config file
	cfgname := self.printer.Get_start_args()["config_file"].(string)

	data, err := self._read_config_file(cfgname)
	regular_data, _ := self._find_autosave_data(data)
	self._build_config_wrapper(regular_data, cfgname)
	if err != nil {
		msg := "Unable to parse existing config on SAVE_CONFIG"
		log.Println(msg)
		return errors.New(msg)
	}
	regular_data = self._strip_duplicates(regular_data, self.autosave)
	self._disallow_include_conflicts(regular_data, cfgname, gcode)
	data = strings.TrimSpace(regular_data) + "\n" + AUTOSAVE_HEADER + "\n" + autosave_data + "\n"
	if !useMutableConfig() {
		// Determine filenames
		datestr := time.Now().Format("-20060102_150405")
		backup_name := cfgname + datestr
		temp_name := cfgname + "_autosave"

		if strings.HasSuffix(cfgname, ".cfg") {
			backup_name = cfgname[:len(cfgname)-4] + datestr + ".cfg"
			temp_name = cfgname[:len(cfgname)-4] + "_autosave.cfg"
		}
		// Create new config file with temporary name and swap with main config
		value.StaticValue.Debug.Printf("SAVE_CONFIG to '%s' (backup in '%s')",
			cfgname, backup_name)
		err1 := ioutil.WriteFile(temp_name, []byte(data), 0666)
		if err1 != nil {
			msg := "Unable to write config file during SAVE_CONFIG"
			log.Println(msg)
			return err1
		}
		os.Rename(cfgname, backup_name)
		os.Rename(temp_name, cfgname)
	} else {
		//log.Println("SaveAndBackup")
		mutablecfg := self._build_mutable_config(self.autosave)
		if err = mutablecfg.SaveAndBackup(); err != nil {
			value.StaticValue.Error.Printf("mutablecfg SaveAndBackup error: " + err.Error())
		} else {
			syscall.Sync()
		}

		// Read in and validate current config file
		// Determine filenames
		// Create new config file with temporary name and swap with main config
		shouldSave, err := self.save_default_data(gcode)
		if err != nil {
			value.StaticValue.Error.Printf("cfg SaveAndBackup error: " + err.Error())
		} else if shouldSave {
			syscall.Sync()
		}
	}

	self.printer.Send_event("project:config_saved", []interface{}{
		self.Read_main_config(),
	})
	if !useMutableConfig() {
		// Request a restart
		gcode.Request_restart("restart")
	}
	return nil
}

func (self *PrinterConfig) save_default_data(gcode *GCodeDispatch) (bool, error) {

	default_data := collections.NewSortedMap()

	if v, ok := self.autosave.fileconfig.Config.Get("leviQ3 default"); ok {
		default_data.Insert("leviQ3 default", v)
	}
	self.autosave.fileconfig.Config = default_data
	// 没有需要保存的数据
	if len(self.autosave.fileconfig.Config.Keys()) == 0 {
		return false, nil
	}
	autosave_data_temp := self._build_config_string(self.autosave)
	autosave_databytes, _ := ioutil.ReadAll(autosave_data_temp)

	l := strings.Split(strings.TrimRight(string(autosave_databytes), "\n"), "\n")
	for i := 0; i < len(l); i++ {
		l[i] = strings.Join([]string{"#*# ", l[i]}, "")
	}
	autosave_data := strings.Join(l, "\n")

	cfgname := self.printer.Get_start_args()["config_file"].(string)

	data, err := self._read_config_file(cfgname)
	regular_data, _ := self._find_autosave_data(data)
	self._build_config_wrapper(regular_data, cfgname)
	if err != nil {
		msg := "Unable to parse existing config on SAVE_CONFIG"
		log.Println(msg)
		return false, errors.New(msg)
	}
	regular_data = self._strip_duplicates(regular_data, self.autosave)
	self._disallow_include_conflicts(regular_data, cfgname, gcode)
	data = strings.TrimSpace(regular_data) + "\n" + AUTOSAVE_HEADER + "\n" + autosave_data + "\n"
	//value.StaticValue.Debug.Print(data)

	datestr := time.Now().Format("-20060102_150405")
	backup_name := cfgname + datestr
	temp_name := cfgname + "_autosave"

	if strings.HasSuffix(cfgname, ".cfg") {
		backup_name = cfgname[:len(cfgname)-4] + datestr + ".cfg"
		temp_name = cfgname[:len(cfgname)-4] + "_autosave.cfg"
	}

	value.StaticValue.Debug.Printf("SAVE_CONFIG to '%s' (backup in '%s')",
		cfgname, backup_name)
	err1 := ioutil.WriteFile(temp_name, []byte(data), 0666)
	if err1 != nil {
		msg := "Unable to write config file during SAVE_CONFIG"
		log.Println(msg)
		return false, err1
	}
	os.Rename(cfgname, backup_name)
	os.Rename(temp_name, cfgname)
	syscall.Sync()
	cleanOldMainConfigBackupFile(cfgname, 3)
	return true, nil
}

func (self *PrinterConfig) copy(dst *PrinterConfig) {
	*dst = *self
}

func (self *PrinterConfig) save_config(web_request *WebRequest) (interface{}, error) {
	if self.autosave.fileconfig.Sections() == nil {
		return nil, nil
	}
	gcode := MustLookupGcode(self.printer)
	/* 	gcode := MustLookupGcode(self.printer)
	   	// Create string containing autosave data
	   	autosave_data_temp := self._build_config_string(self.autosave)
	   	autosave_databytes, _ := ioutil.ReadAll(autosave_data_temp)

	   	l := strings.Split(strings.TrimRight(string(autosave_databytes), "\n"), "\n")
	   	for i := 0; i < len(l); i++ {
	   		l[i] = strings.Join([]string{"#*# ", l[i]}, "")
	   	}
	   	autosave_data := strings.Join(l, "\n")
	   	// Read in and validate current config file
	   	cfgname := self.printer.Get_start_args()["config_file"].(string)

	   	data, err := self._read_config_file(cfgname)
	   	regular_data, _ := self._find_autosave_data(data)
	   	self._build_config_wrapper(regular_data, cfgname)
	   	if err != nil {
	   		msg := "Unable to parse existing config on SAVE_CONFIG"
	   		log.Println(msg)
	   		panic(msg)
	   	}
	   	regular_data = self._strip_duplicates(regular_data, self.autosave)
	   	self._disallow_include_conflicts(regular_data, cfgname, gcode)
	   	data = strings.TrimSpace(regular_data) + "\n" + AUTOSAVE_HEADER + "\n" + autosave_data + "\n"
	   	// Determine filenames
	   	datestr := time.Now().Format("-20060102_150405")
	   	backup_name := cfgname + datestr
	   	temp_name := cfgname + "_autosave"

	   	if strings.HasSuffix(cfgname, ".cfg") {
	   		backup_name = cfgname[:len(cfgname)-4] + datestr + ".cfg"
	   		temp_name = cfgname[:len(cfgname)-4] + "_autosave.cfg"
	   	}
	   	// Create new config file with temporary name and swap with main config
	   	value.StaticValue.Debug.Printf("SAVE_CONFIG to '%s' (backup in '%s')",
	   		cfgname, backup_name)
	   	err1 := ioutil.WriteFile(temp_name, []byte(data), 0666)
	   	if err1 != nil {
	   		msg := "Unable to write config file during SAVE_CONFIG"
	   		log.Println(msg)
	   		panic(err1)
	   	}
	   	os.Rename(cfgname, backup_name)
	   	os.Rename(temp_name, cfgname) */
	mutablecfg := self._build_mutable_config(self.autosave)
	if err := mutablecfg.SaveAndBackup(); err != nil {
		value.StaticValue.Error.Printf("mutablecfg SaveAndBackup error: " + err.Error())
	} else {
		syscall.Sync()
	}

	shouldSave, err := self.save_default_data(gcode)
	if err != nil {
		value.StaticValue.Error.Printf("cfg SaveAndBackup error: " + err.Error())
	} else if shouldSave {
		syscall.Sync()
	}

	self.printer.Send_event("project:config_saved", []interface{}{
		self.Read_main_config(),
	})

	if !useMutableConfig() {
		// Request a restart
		gcode.Request_restart("restart")
	}
	return nil, nil
}

func (self *PrinterConfig) _build_config_string(config *ConfigWrapper) io.Reader {
	buf := bytes.NewBuffer(nil)
	config.fileconfig.Write(buf)
	return buf
}

func (self *PrinterConfig) _build_mutable_config(config *ConfigWrapper) *configparser.MutableConfig {
	path := filepath.Dir(self.printer.Get_start_args()["config_file"].(string))
	file := filepath.Join(path, configparser.MutableConfigFileName)
	cfg, err := configparser.NewMutableConfig(file, configparser.EncryptNone, true)
	if err != nil {
		panic("_build_mutable_config new mutable error: " + err.Error())
	}
	for section, options := range config.fileconfig.Config.Map() {
		sec := cfg.AddSectionIfNotExist(section)
		for name, val := range options.(map[string]string) {
			sec.Set(name, val)
		}
	}
	return cfg
}

func useMutableConfig() bool {
	if os.Getenv("USE_MUTABLE_CONFIG") == "" {
		return true
	}
	if os.Getenv("USE_MUTABLE_CONFIG") == "1" {
		return true
	}
	return false
}

func cleanOldMainConfigBackupFile(filename string, remain int) {
	dir := filepath.Dir(filename)
	basename := filepath.Base(filename)
	suffixName := basename
	if pos := strings.LastIndex(basename, "."); pos > -1 {
		suffixName = basename[0:pos]
	}

	matchs, _ := filepath.Glob(filepath.Join(dir, suffixName+`-*_*.cfg`))
	if len(matchs) <= remain {
		return
	}

	sort.Strings(matchs)
	for _, file := range matchs[0 : len(matchs)-remain] {
		os.Remove(file)
	}
}
