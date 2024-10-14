/*
# Virtual sdcard support (print files directly from a host g-code file)
#
# Copyright (C) 2018  Kevin O"Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
*/
package project

import (
	"bufio"
	"encoding/json"
	"errors"
	"fmt"
	"io"
	"io/fs"
	"io/ioutil"
	"k3c/common/constants"
	kerror "k3c/common/errors"
	"k3c/common/utils/maths"
	"k3c/common/utils/object"

	//"k3c/common/utils/reflects"
	"k3c/common/value"
	"k3c/project/util"
	"log"
	"math"
	"os"
	"os/user"
	"path"
	"path/filepath"
	"regexp"
	"runtime"
	"runtime/debug"
	"sort"
	"strconv"
	"strings"
	"time"
)

var VALID_GCODE_EXTS = []string{"gcode", "g", "gco"}

const PRUSASLICER = "PrusaSlicer"
const ANYCUBICSLICER = "AnycubicSlicer"
const ORCA = "OrcaSlicer"
const BAMBUSTUDIO = "BambuStudio"
const CURA_STEAMENGINE = "Cura_SteamEngine"

var SupportedSlicer = []string{PRUSASLICER, ANYCUBICSLICER, ORCA, BAMBUSTUDIO, CURA_STEAMENGINE}

// 2%
const UNIT_MIN = 0.02

// 8%
const UNIT_MAX = 1

// 10M
const FILE_LIMIT = 1024 * 1024 * 10
const WAIT_TIME = time.Millisecond * 1000

const printInfoRecordFileNum = 10

type Colour struct {
	R int `json:"r"`
	G int `json:"g"`
	B int `json:"b"`
}

type VirtualSD struct {
	Printer                 *Printer
	Sdcard_dirname          string
	Current_file            *os.File
	File_size               int64
	Print_stats             *PrintStats
	Reactor                 IReactor
	Must_pause_work         bool
	Cmd_from_sd             bool
	Next_file_position      int
	Work_timer              *ReactorTimer
	On_error_gcode          interface{}
	Gcode                   *GCodeDispatch
	layerCount              int
	currLayer               int
	totalTime               int
	config                  *ConfigWrapper
	sd_type                 string
	state_path              string
	bm                      *runtime.MemStats
	IsPrint                 bool
	IsStartPrintCmd         bool
	File_position           int
	PrintInfo               *PrintInfo
	recordCount             int
	layerHeight             float64
	filamentType            string
	travelSpeed             float64
	fillDensity             float64
	supportMaterialAuto     int
	filamentRetractLength   float64
	bedTemperature          float64
	extruderTemperature     float64
	perimeters              int
	perimeterExtrusionWidth float64
	perimeterSpeed          float64
	brimType                string
	printerSettingsId       string
	extruderColour          []Colour
	filamentColour          []Colour
	enableContinuePrint     int
	gcodeWhiteList          []string
	flushMultiplier         float64
	flushVolumesMatrix      []int
	flushVolumesVector      []int
	gcodeSlicer             string
	modelSizeX              float64
	modelSizeY              float64
	modelSizeZ              float64
	timelapseType           int
	retractLift             float64

	zAxisTracker *ZAxisTracker
}

var fristM600 = true

type PrintInfo struct {
	FilamentTotalUses      string                   `json:"filament_total_uses"`
	FilamentUsed           float64                  `json:"filament_used"`
	HotbedTemp             float64                  `json:"hotbed_temp"`
	ExtruderTemp           float64                  `json:"extur_temp"`
	Axis_state             map[string]*Saved_states `json:"axis_state"`
	File_position          int                      `json:"file_position"`
	Filepath               string                   `json:"file_path"`
	Last_print_duration    float64                  `json:"last_print_duration"`
	Fan_state              float64                  `json:"fan_state"`
	Progress               float64                  `json:"progress"`
	FilamentSwitch         int                      `json:"filamentSwitch"`
	Cancel                 bool                     `json:"cancel"`
	TotalTime              int                      `json:"total_time"`
	LayerCount             int                      `json:"total_layer"`
	CurrLayer              int                      `json:"currLayer"`
	Accel                  *float64                 `json:"accel"`
	Restore_total_duration float64                  `json:"total_duration"`

	FristLayer            bool
	FristLayerAccel       float64
	FristLayerSpeed       float64
	FristLayerSpeedFactor float64

	UseACE bool `json:"use_ace"`
	TIndex int  `json:"t_index"`
}

func (self *VirtualSD) get_vaild_path(sd_type string) string {
	current_user, err := user.Current()
	if err != nil {
		value.StaticValue.Error.Println(err)
	}

	var sd_split []string
	if sd_type == "api.cfg" {
		path := value.StaticValue.Config.Sys.PrinterPath
		self.sd_type = "api.cfg"
		if path == "" {
			path = self.config.Get("path", object.Sentinel{}, true).(string)
			self.sd_type = "project.cfg"
		}
		sd_split = strings.Split(path, "/")
	} else {
		sd := self.config.Get("path", object.Sentinel{}, true)
		self.sd_type = "project.cfg"
		if sd == nil {
			sd = value.StaticValue.Config.Sys.PrinterPath
			self.sd_type = "api.cfg"
		}
		sd_split = strings.Split(sd.(string), "/")
	}

	Sdcard_dirname := []string{}
	for _, s := range sd_split {
		if s == "~" {
			Sdcard_dirname = append(Sdcard_dirname, current_user.HomeDir)
		} else if s != "" {
			Sdcard_dirname = append(Sdcard_dirname, string(os.PathSeparator)+s)
		}
	}
	return strings.Join(Sdcard_dirname, "")
}

type ZAxisTracker struct {
	filename       string
	virtual_sdcard *VirtualSD
	layer_height   float64
	last_z         float64
}

func NewZAxisTracker(virtual_sdcard *VirtualSD) *ZAxisTracker {
	tracker := &ZAxisTracker{
		virtual_sdcard: virtual_sdcard,
		layer_height:   0,
		last_z:         0,
	}
	tracker.filename = path.Join(value.InitValue().Config.Sys.StatePath, "last_z")

	virtual_sdcard.Printer.Register_event_handler("gcode_move:move", tracker.update_z)

	clearFn := func(i []interface{}) error {
		tracker.layer_height = 0
		tracker.clear()
		return nil
	}

	virtual_sdcard.Printer.Register_event_handler("virtual_sdcard:start_print", clearFn)
	virtual_sdcard.Printer.Register_event_handler("virtual_sdcard:cancel_print", clearFn)
	virtual_sdcard.Printer.Register_event_handler("virtual_sdcard:finish_print", clearFn)

	return tracker
}

func (zat *ZAxisTracker) get_z() (float64, error) {
	filename := path.Join(value.InitValue().Config.Sys.StatePath, "last_z")
	_, err := os.Stat(filename)
	if err != nil {
		return 0., err
	}

	f, err := os.Open(filename)
	if err != nil {
		return 0., err
	}
	defer f.Close()

	bytes, err := io.ReadAll(f)
	if err != nil {
		return 0., err
	}

	z_offset, err := strconv.ParseFloat(string(bytes), 64)
	if err != nil {
		return 0., err
	}
	fmt.Println("z_offset: ", z_offset)
	return z_offset, nil
}

func (zat *ZAxisTracker) clear() {
	_, err := os.Stat(zat.filename)
	if err != nil {
		return
	}
	os.Remove(zat.filename)
}

func (zat *ZAxisTracker) update_layer_height(z float64) {
	zat.layer_height = z
}

func (self *ZAxisTracker) update_z(args []interface{}) error {
	if !self.virtual_sdcard.Print_stats.printing {
		return nil
	}

	gmove := self.virtual_sdcard.Printer.Lookup_object("gcode_move", object.Sentinel{}).(*GCodeMove)

	// pos := args[0].([]float64)
	z_pos := gmove.last_position[2]
	gcode_pos_z := gmove._get_gcode_position()[2]

	retractLift := self.virtual_sdcard.retractLift
	if retractLift <= 0. {
		retractLift = 0.25
	}

	if math.Abs(gcode_pos_z-self.layer_height) <= retractLift {
		self.clear()
		return nil
	}

	if z_pos == self.last_z {
		return nil
	}
	self.last_z = z_pos

	go func() {
		tempname := self.filename + ".1"
		f, _ := os.OpenFile(tempname, os.O_WRONLY|os.O_CREATE|os.O_TRUNC, 0644)
		f.Write([]byte(fmt.Sprintf("%f", self.last_z)))
		f.Sync()
		f.Close()

		os.Rename(tempname, self.filename)
	}()

	// toolhead := self.virtual_sdcard.Printer.Lookup_object("toolhead", object.Sentinel{}).(*Toolhead)

	// speed := args[1].(float64)
	// log.Printf("moved: pos: %v, speed: %f, toolhead pos: %v", pos, speed, toolhead.Get_position())

	// bts, _ := json.Marshal(gmove.Get_status(self.virtual_sdcard.Reactor.Monotonic()))
	// log.Println("gcode_move status: ", string(bts))

	return nil
}

func NewVirtualSD(config *ConfigWrapper) *VirtualSD {
	self := &VirtualSD{}
	self.config = config
	self.Printer = config.Get_printer()
	self.Printer.Register_event_handler("project:shutdown",
		self.Handle_shutdown)
	self.Sdcard_dirname = value.InitValue().Config.Sys.PrinterPath
	self.PrintInfo = &PrintInfo{}
	_, err := os.Stat(self.Sdcard_dirname)
	if err != nil && err == io.EOF {
		os.MkdirAll(self.Sdcard_dirname, 0666)
	}
	self.state_path = path.Join(value.InitValue().Config.Sys.StatePath, "record")
	self.recordCount = 1
	_, err = os.Stat(value.InitValue().Config.Sys.StatePath)
	if err != nil && err == io.EOF {
		os.MkdirAll(value.InitValue().Config.Sys.StatePath, 0666)
	}
	self.Current_file = nil
	self.File_size = 0
	// Print Stat Tracking
	self.Print_stats = MustLoadPrintStats(config)
	// Work timer
	self.Reactor = self.Printer.Get_reactor()
	self.Must_pause_work = false
	self.Cmd_from_sd = false
	self.IsStartPrintCmd = false
	self.Next_file_position = 0
	self.Work_timer = nil
	self.enableContinuePrint = config.Getint("enable_continue_print", 0, 0, 1, true)
	// Error handling
	gcode_macro := self.Printer.Load_object(config, "gcode_macro_1", object.Sentinel{}).(*PrinterGCodeMacro)
	self.On_error_gcode = gcode_macro.Load_template(
		config, "on_error_gcode", "")
	// Register commands
	gcode_object := self.Printer.Lookup_object("gcode", object.Sentinel{})
	self.Gcode = gcode_object.(*GCodeDispatch)
	self.Gcode.Register_command("SDCARD_RESET_FILE", self.Cmd_SDCARD_RESET_FILE,
		false, cmd_SDCARD_RESET_FILE_help)
	self.Gcode.Register_command("SDCARD_PRINT_FILE", self.Cmd_SDCARD_PRINT_FILE,
		false, cmd_SDCARD_PRINT_FILE_help)
	self.Gcode.Register_command("SDCARD_RESUME_PRINT", self.Cmd_RESTORE_POWER_OFF_STATE,
		false, cmd_SDCARD_PRINT_FILE_help)
	self.Gcode.Register_command("SET_RESTORE_POWER_OFF", self.Cmd_SET_RESTORE_POWER_OFF,
		false, cmd_SDCARD_PRINT_FILE_help)

	self.zAxisTracker = NewZAxisTracker(self)

	wh := MustLookupWebhooks(self.Printer)
	wh.Register_endpoint(
		"print/query_resume_print", self.query_resume_print)
	self.bm = new(runtime.MemStats)
	//注册Gcode白名单
	self.register_whitelist("SNAPSHOT")
	self.register_whitelist("SET_VELOCITY_LIMIT")
	self.register_whitelist("SET_PRESSURE_ADVANCE")
	self.register_whitelist("SAVE_GCODE_STATE")
	self.register_whitelist("RESTORE_GCODE_STATE")
	self.register_whitelist("TUNING_TOWER")
	self.register_whitelist("SHAPER_CALIBRATE")
	self.register_whitelist("EXCLUDE_OBJECT_DEFINE")
	self.register_whitelist("EXCLUDE_OBJECT_START")
	self.register_whitelist("EXCLUDE_OBJECT_END")
	self.register_whitelist("EXCLUDE_OBJECT")
	return self
}

type QueryResumePrintInfo struct {
	HasResume      int     `json:"hasResume"`
	Gcode          string  `json:"gcode"`
	GcodePath      string  `json:"gcodePath"`
	Reason         string  `json:"reason"`
	FanSpeed       float64 `json:"fanSpeed"`
	FilamentSwitch int     `json:"filamentSwitch"`
	PrintSpeedMode int     `json:"printSpeedMode"`
	Progress       float64 `json:"progress"`
	PrintTime      float64 `json:"printTime"`
	ZOffset        float64 `json:"zOffset"`
	TotalTime      int     `json:"total_time"`
	LayerCount     int     `json:"total_layer"`
	CurrLayer      int     `json:"current_layer"`
}

func (v *VirtualSD) query_resume_print(web_request *WebRequest) (interface{}, error) {

	var result QueryResumePrintInfo
	var printInfo *PrintInfo
	file_position := 0
	for i := 1; i <= printInfoRecordFileNum; i++ {
		// File_position
		state_str, err := os.ReadFile(fmt.Sprintf(v.state_path+".%d", i))
		if err != nil {
			value.StaticValue.Error.Println(err)
			continue
		}
		var sta PrintInfo
		err = json.Unmarshal(state_str, &sta)
		if err != nil {
			value.StaticValue.Error.Println(err)
			continue
		}
		// record cancel
		if sta.Cancel {
			break
		} else if sta.File_position >= file_position {
			file_position = sta.File_position
			printInfo = &sta
		}
	}
	if printInfo == nil {
		result.HasResume = 0
		result.Gcode = ""
		result.GcodePath = ""
		result.Reason = "Print record does not exist"
		web_request.Send(result)
		return nil, nil
	}

	result.HasResume = 1
	arr := strings.Split(printInfo.Filepath, string(os.PathSeparator))
	result.Gcode = arr[len(arr)-1]
	result.GcodePath = printInfo.Filepath
	result.PrintSpeedMode = printInfo.Axis_state["POWER_OFF"].Speed_Mode
	result.FanSpeed = printInfo.Fan_state
	result.FilamentSwitch = printInfo.FilamentSwitch
	result.PrintTime = printInfo.Last_print_duration
	result.Progress = printInfo.Progress
	result.TotalTime = printInfo.TotalTime
	result.LayerCount = printInfo.LayerCount
	result.CurrLayer = printInfo.CurrLayer
	web_request.Send(result)
	return nil, nil
}

func (self *VirtualSD) Handle_shutdown([]interface{}) error {
	if self.Work_timer != nil {
		self.Must_pause_work = true
	}
	readpos := math.Max(float64(self.File_position)-1024, 0)
	readcount := float64(self.File_position) - readpos
	self.Current_file.Seek(int64(readpos)+128, 0)
	data := make([]byte, int64(readcount+128))
	_, err := self.Current_file.Read(data)
	if err != nil {
		value.StaticValue.Error.Println("virtual_sdcard shutdown read")
	}
	log.Printf("Virtual sdcard (%f): %s\nUpcoming (%d): %s",
		readpos, string(data[:int(readcount)]),
		self.File_position, string(data[int(readcount):]))
	return nil
}

func (self *VirtualSD) Stats(eventtime float64) (bool, string) {
	if self.Work_timer != nil {
		return false, ""
	}
	return true, fmt.Sprintf("sd_pos=%d", self.File_position)
}

type flistNode struct {
	r_path string
	size   int64
}

func (self *VirtualSD) Get_file_list(check_subdirs bool) []flistNode {
	if check_subdirs {
		flist := []flistNode{}
		filestrlist := []string{}
		fileMap := make(map[string]*flistNode)
		filepath.Walk(self.Sdcard_dirname, func(path string, info fs.FileInfo, err error) error {
			if err != nil {
				value.StaticValue.Error.Println(err)
				return nil
			}

			if info.IsDir() {
				return nil
			}
			name := info.Name()
			ext := name[strings.LastIndex(name, ".")+1:]
			if sort.SearchStrings(VALID_GCODE_EXTS, ext) == len(VALID_GCODE_EXTS) {
				return nil
			}
			full_path := path + string(os.PathSeparator) + name
			r_path := full_path[len(self.Sdcard_dirname)+1:]
			size := info.Size()
			fileMap[full_path] = &flistNode{strings.ToLower(r_path), size}
			filestrlist = append(filestrlist, strings.ToLower(r_path))
			return nil
		})
		sort.Strings(filestrlist)
		for _, item := range filestrlist {
			val := fileMap[item]
			if val == nil {
				continue
			}
			flist = append(flist, *val)
		}
		return flist
	} else {
		dname := self.Sdcard_dirname
		dirs, err := ioutil.ReadDir(dname)
		if err != nil {
			value.StaticValue.Error.Printf("virtual_sdcard get_file_list")
			panic("Unable to get file list")
		}
		flist := []flistNode{}
		filenames := make(map[string]fs.FileInfo)
		filestrlist := []string{}
		for _, dir := range dirs {
			filenames[strings.ToLower(dir.Name())] = dir
			filestrlist = append(filestrlist, strings.ToLower(dir.Name()))
		}
		sort.Strings(filestrlist)
		for _, item := range filestrlist {
			fname := filenames[item]
			if fname == nil {
				continue
			}
			if strings.HasPrefix(fname.Name(), ".") == false && fname.IsDir() == false {
				flist = append(flist, flistNode{fname.Name(), fname.Size()})
			}
		}
		return flist
	}
}
func (self *VirtualSD) Get_status(eventtime float64) map[string]interface{} {
	return map[string]interface{}{
		"file_path":                 self.File_path(),                 // strings
		"progress":                  maths.Round(self.Progress(), 4),  // float64
		"is_active":                 self.Is_active(),                 // bool
		"file_position":             self.File_position,               // strings
		"file_size":                 self.File_size,                   // int
		"total_layer":               self.layerCount,                  // int
		"current_layer":             self.currLayer,                   // int
		"total_time":                self.totalTime,                   // int
		"filament_used":             self.PrintInfo.FilamentTotalUses, // string
		"layer_height":              self.layerHeight,
		"filament_type":             self.filamentType,
		"travel_speed":              self.travelSpeed,
		"fill_density":              self.fillDensity,
		"support_material_auto":     self.supportMaterialAuto,
		"filament_retract_length":   self.filamentRetractLength,
		"timelapse_type":            self.timelapseType,
		"bed_temperature":           self.bedTemperature,
		"extruder_temperature":      self.extruderTemperature,
		"perimeters":                self.perimeters,
		"perimeter_extrusion_width": self.perimeterExtrusionWidth,
		"perimeter_speed":           self.perimeterSpeed,
		"brim_type":                 self.brimType,
		"printer_settings_id":       self.printerSettingsId,
		"is_homing":                 is_homing,
		"extruder_colour":           self.extruderColour,
		"model_size_x":              self.modelSizeX,
		"model_size_y":              self.modelSizeY,
		"model_size_z":              self.modelSizeZ,
	}
}
func (self *VirtualSD) File_path() string {
	if self.Current_file != nil {
		return self.Current_file.Name()
	}

	return self.config.fileconfig.Get("virtual_sdcard", "path").(string)
}
func (self *VirtualSD) Progress() float64 {
	if self.File_size != 0 {
		return float64(self.File_position) / float64(self.File_size)
	} else {
		return 0.
	}
}
func (self *VirtualSD) Is_active() bool {
	return self.Work_timer != nil
}
func (self *VirtualSD) Do_pause() {
	if self.Work_timer != nil {
		self.Must_pause_work = true
		self.Printer.Send_event("virtual_sdcard:pause_print", nil)
		for self.Work_timer != nil && !self.Cmd_from_sd {
			self.Reactor.Pause(self.Reactor.Monotonic() + .001)
		}
	}
}

func (self *VirtualSD) Do_pre_resume() error {
	self.Printer.Send_event("virtual_sdcard:pre_resume_print", nil)
	return nil
}

func (self *VirtualSD) Do_resume() error {
	if self.Work_timer != nil {
		return errors.New("SD busy")
	}
	self.Must_pause_work = false
	self.Work_timer = self.Reactor.Register_timer(
		self.Work_handler, constants.NOW)
	self.Printer.Send_event("virtual_sdcard:resume_print", nil)
	return nil
}

func (self *VirtualSD) Do_power_off_resume() error {
	if self.Work_timer != nil {
		return errors.New("SD busy")
	}
	self.Must_pause_work = false
	self.Work_timer = self.Reactor.Register_timer(
		self.Work_resume_handler, constants.NOW)
	self.Printer.Send_event("virtual_sdcard:resume_print", nil)
	return nil
}

func (self *VirtualSD) Do_cancel() {
	self.Printer.Send_event("virtual_sdcard:cancel_print", nil)
	if self.Current_file != nil {
		self.Do_pause()
		self.Current_file.Close()
		self.Current_file = nil
		defer self.Print_stats.Note_cancel()
		self.handle_print_cancel()
	}
	self.File_position = 0.
	self.File_size = 0.
}

// G-Code commands
func (self *VirtualSD) Cmd_error(argv interface{}) error {
	return errors.New("SD write not supported")
}
func (self *VirtualSD) Reset_file() {
	if self.Current_file != nil {
		self.Do_pause()
		self.Current_file.Close()
		self.Current_file = nil
	}
	self.File_position = 0.
	self.File_size = 0.
	self.totalTime = 0
	self.layerCount = 0
	self.currLayer = 0
	self.layerHeight = 0
	self.filamentType = ""
	self.travelSpeed = -1
	self.fillDensity = -1
	self.supportMaterialAuto = -1
	self.filamentRetractLength = -1
	self.retractLift = -1
	self.timelapseType = 0
	self.bedTemperature = -1
	self.extruderTemperature = -1
	self.perimeters = -1
	self.perimeterExtrusionWidth = -1
	self.perimeterSpeed = -1
	self.brimType = ""
	self.printerSettingsId = ""
	self.extruderColour = []Colour{}
	self.filamentColour = []Colour{}
	self.gcodeSlicer = ""
	self.flushMultiplier = 0.
	self.flushVolumesMatrix = []int{}
	self.flushVolumesVector = []int{}
	self.modelSizeX = 0.0
	self.modelSizeY = 0.0
	self.modelSizeZ = 0.0
	self.Print_stats.Reset()
	self.Printer.Send_event("virtual_sdcard:reset_file", nil)
}

const cmd_SDCARD_RESET_FILE_help = "Clears a loaded SD File. Stops the print " +
	"if necessary"

func (self *VirtualSD) Cmd_SDCARD_RESET_FILE(argv interface{}) error {
	if self.Cmd_from_sd {
		return errors.New(
			"SDCARD_RESET_FILE cannot be run from the sdcard")
	}
	self.Reset_file()
	return nil
}

const cmd_SDCARD_PRINT_FILE_help = "Loads a SD file and starts the print.  May " +
	"include files in subdirectories."

const cmd_RESTORE_POWER_OFF_STATE = "Power back on and continue printing" +
	"EABLE 0 continue 1 cancel"

func (self *VirtualSD) Cmd_SDCARD_PRINT_FILE(argv interface{}) error {
	if self.IsPrint {
		return errors.New("printing")
	}
	gcmd := argv.(*GCodeCommand)
	if self.Work_timer != nil {
		return errors.New("SD busy")
	}
	self.Reset_file()
	filename := gcmd.Get("FILENAME", object.Sentinel{}, "", nil, nil, nil, nil)
	err := self.Load_file(gcmd, filename, true)
	if err != nil {
		//value.StaticValue.Error.Println(sys.GetGID(),err)
		//gcmd.Respond_raw(fmt.Sprintf("SD printing byte %d/%d", 0, 0))
		return err
	}
	self.getCodeInfo(self.Current_file.Name())
	self.IsStartPrintCmd = false
	err = self.Do_resume()
	if err != nil {
		value.StaticValue.Error.Println(err)
	}

	return nil
}
func (self *VirtualSD) Cmd_M20(argv interface{}) error {
	gcmd := argv.(*GCodeCommand)
	// List SD card
	files := self.Get_file_list(false)
	gcmd.Respond_raw("Begin file list")
	for _, item := range files {
		gcmd.Respond_raw(fmt.Sprintf("%s %d", item.r_path, item.size))
	}
	gcmd.Respond_raw("End file list")
	return nil
}
func (self *VirtualSD) Cmd_M21(argv interface{}) error {
	gcmd := argv.(*GCodeCommand)
	// Initialize SD card
	gcmd.Respond_raw("SD card ok")
	return nil
}
func (self *VirtualSD) Cmd_M23(argv interface{}) error {
	gcmd := argv.(*GCodeCommand)
	// Select SD file
	if self.Work_timer != nil {
		return errors.New("SD busy")
	}
	self.Reset_file()
	filename := strings.TrimSpace(gcmd.Get_raw_command_parameters())
	if strings.HasPrefix(filename, "/") {
		filename = filename[1:]
	}
	self.Load_file(gcmd, filename, true)
	return nil
}
func (self *VirtualSD) Load_file(gcmd *GCodeCommand, filename string, check_subdirs bool) error {

	filename = strings.Trim(filename, "\"")
	value.StaticValue.Debug.Println("Load gcode file: ", filename)
	if !strings.HasSuffix(filename, ".gcode") {
		return errors.New("Gcode file extension incorrect")
	}

	var file string
	if strings.Index(filename, "/") != -1 {
		file = filepath.Join("/", filename)
	} else {
		file = filepath.Join(self.Sdcard_dirname, filename)
	}
	f, err := os.Stat(file)
	if err != nil {
		return fmt.Errorf("Unable to open file: %s", file)
	}
	fsize := f.Size()
	if fsize == 0 {
		return errors.New("Gcode file is empty")
	}
	gcmd.Respond_raw(fmt.Sprintf("File opened:%s Size:%d", strings.Trim(filename, "\""), fsize))
	gcmd.Respond_raw("File selected")
	_file, _ := os.Open(file)
	if self.Current_file != nil {
		self.Current_file.Close()
		self.Current_file = nil
		//value.StaticValue.Error.Println(sys.GetGID(),"self.Current_file",self.Current_file,"Load_file")
	}
	self.Current_file = _file
	self.File_position = 0
	self.File_size = fsize
	self.Print_stats.Set_current_file(strings.Trim(filename, "\""))
	return nil
}
func (self *VirtualSD) Cmd_M24(argv interface{}) error {
	//gcmd := argv.(*GCodeCommand)
	// Start/resume SD print
	err := self.Do_resume()
	if err != nil {
		value.StaticValue.Error.Println(err)
	}
	return err
}
func (self *VirtualSD) Cmd_M25(argv interface{}) error {
	//gcmd := argv.(*GCodeCommand)
	// Pause SD print
	self.Do_pause()
	return nil
}
func (self *VirtualSD) Cmd_M26(argv interface{}) error {
	gcmd := argv.(*GCodeCommand)
	// Set SD position
	if self.Work_timer != nil {
		panic("SD busy")
	}
	minval := 0
	pos := gcmd.Get_int("S", nil, &minval, &minval)
	self.File_position = pos
	return nil
}
func (self *VirtualSD) Cmd_M27(argv interface{}) error {
	gcmd := argv.(*GCodeCommand)
	// Report SD print status
	if self.Current_file != nil {
		gcmd.Respond_raw("Not SD printing.")
		return nil
	}
	gcmd.Respond_raw(fmt.Sprintf("SD printing byte %d/%d", self.File_position, self.File_size))
	return nil
}
func (self *VirtualSD) Get_file_position() int {
	return self.Next_file_position
}
func (self *VirtualSD) Set_file_position(pos int) {
	self.Next_file_position = pos
}
func (self *VirtualSD) Is_cmd_from_sd() bool {
	return self.Cmd_from_sd
}

func (v *VirtualSD) SAVE_POWER_OFF_STATE(accel *float64) error {
	//gcmd := argv.(*GCodeCommand)
	gcode_move := v.Printer.Lookup_object("gcode_move", object.Sentinel{}).(*GCodeMove)
	fan := v.Printer.Lookup_object("fan", object.Sentinel{}).(*PrinterFan)
	extruder := v.Printer.Lookup_object("extruder", object.Sentinel{}).(*PrinterExtruder)
	heatbed := v.Printer.Lookup_object("heater_bed", object.Sentinel{}).(*PrinterHeaterBed)
	//self.Gcode.Run_script("SAVE_GCODE_STATE")
	//gcm := gcode_move.(*GCodeMove).Saved_states
	v.Gcode.Run_script_from_command("SAVE_GCODE_STATE NAME=POWER_OFF")
	//gcode_move.Lock.RLock()
	sta := &PrintInfo{}
	//defer gcode_move.Lock.RLocker().Unlock()
	sta.Axis_state = gcode_move.Saved_states
	// sta.Axis_state["POWER_OFF"].Last_position[2] += 0.1
	sta.File_position = v.Get_file_position()
	sta.Filepath = v.File_path()
	if accel != nil {
		sta.Accel = accel
	}
	sta.Fan_state = fan.Get_status(v.Printer.reactor.Monotonic())["speed"]
	sta.HotbedTemp = heatbed.Get_status(v.Printer.reactor.Monotonic())["target"]
	sta.ExtruderTemp = extruder.Get_status(v.Printer.reactor.Monotonic())["target"].(float64)
	sta.FilamentTotalUses = v.PrintInfo.FilamentTotalUses
	sta.FilamentUsed = v.Print_stats.filament_used
	sta.Progress = v.Progress()
	sta.TotalTime = v.totalTime
	sta.LayerCount = v.layerCount
	sta.CurrLayer = v.currLayer
	sta.Restore_total_duration = v.Print_stats.Get_status(v.Printer.Get_reactor().Monotonic())["total_duration"].(float64)
	sta.Last_print_duration = sta.Restore_total_duration

	bytes, err := json.Marshal(sta)
	if err != nil {
		value.StaticValue.Debug.Println(err)
	}
	// value.StaticValue.Debug.Println(string(bytes))
	recordFile := fmt.Sprintf(v.state_path+".%d", v.recordCount)
	if v.recordCount == printInfoRecordFileNum {
		v.recordCount = 1
	} else {
		v.recordCount++
	}
	return os.WriteFile(recordFile, bytes, 0666)
}

func (v *VirtualSD) Cancel_record() error {
	gcode_move := v.Printer.Lookup_object("gcode_move", object.Sentinel{}).(*GCodeMove)
	v.Gcode.Run_script_from_command("SAVE_GCODE_STATE NAME=POWER_OFF")
	gcode_move.Lock.RLock()
	defer gcode_move.Lock.RLocker().Unlock()
	v.Print_stats.Restore_total_duration = 0.
	for i := 0; i < printInfoRecordFileNum; i++ {
		recordFile := v.state_path + "." + strconv.Itoa(i+1)
		os.Remove(recordFile)
	}
	v.PrintInfo = &PrintInfo{}
	return nil
}

func (self *VirtualSD) Cmd_RESTORE_POWER_OFF_STATE(argv interface{}) error {
	if self.enableContinuePrint == 0 {
		return errors.New("The power outage function has been turned off.")
	}
	if self.IsPrint {
		return errors.New("printing")
	}
	zero := 0
	enable := argv.(*GCodeCommand).Get_int("ENABLE", nil, &zero, nil)
	if enable != 1 {
		self.Cancel_record()
		return nil
	}
	var printInfo *PrintInfo
	file_position := 0
	for i := 1; i <= 10; i++ {
		// File_position
		state_str, err := os.ReadFile(fmt.Sprintf(self.state_path+".%d", i))
		if err != nil {
			value.StaticValue.Error.Println(err)
			continue
		}
		var sta PrintInfo
		err = json.Unmarshal(state_str, &sta)
		if err != nil {
			value.StaticValue.Error.Println(err)
			continue
		}
		// record cancel
		if sta.Cancel {
			break
		} else if sta.File_position >= file_position {
			file_position = sta.File_position
			printInfo = &sta
		}
	}

	if printInfo == nil {
		return kerror.PrintRecordIsNotExistError
	}

	self.getCodeInfo(printInfo.Filepath)

	self.File_position = printInfo.File_position
	Sdcard_dirname := printInfo.Filepath[:strings.LastIndex(printInfo.Filepath, "/")]
	// Sdcard_dirname := "/" + strings.Join(pathArr[:len(pathArr)-1], "/")
	self.Sdcard_dirname = Sdcard_dirname
	self.Next_file_position = printInfo.File_position
	self.PrintInfo = printInfo
	_file, err := os.Open(self.PrintInfo.Filepath)
	if err != nil {
		return kerror.GcodeIsNotExistError
	}
	//defer _file.Close()
	fs, err := _file.Stat()
	if err != nil {
		return kerror.GcodeIsNotExistError
	}
	_file.Seek(int64(self.Next_file_position), 0)
	self.Current_file = _file
	self.File_size = fs.Size()
	self.currLayer = printInfo.CurrLayer
	fileArr := strings.Split(self.PrintInfo.Filepath, string(os.PathSeparator))
	filename := fileArr[len(fileArr)-1]
	self.Print_stats.Set_current_file(filename)
	self.Print_stats.Restore_total_duration = printInfo.Restore_total_duration
	self.Print_stats.state = Printing_
	self.Do_power_off_resume()
	return nil
}

func (v *VirtualSD) Cmd_SET_RESTORE_POWER_OFF(arg interface{}) error {
	gcmd := arg.(*GCodeCommand)
	min := 0
	max := 1
	enable := gcmd.Get_int("ENABLE", 0, &min, &max)
	v.enableContinuePrint = enable
	return nil
}

func (v *VirtualSD) Work_resume_handler(eventtime float64) float64 {

	// 升温，防止喷嘴冷却后粘住模型
	v.Gcode.Run_script_from_command("M104 S185")
	v.Gcode.Run_script_from_command(fmt.Sprintf("M190 S%f", v.PrintInfo.HotbedTemp))
	// 归零
	v.Gcode.Run_script_from_command("G90")
	v.Gcode.Run_script_from_command("OPEN_STEALTHCHOP STEPPER=stepper_x")
	v.Gcode.Run_script_from_command("OPEN_STEALTHCHOP STEPPER=stepper_y")
	v.Gcode.Run_script_from_command("HOME_XY")
	v.Gcode.Run_script_from_command("CLOSE_STEALTHCHOP STEPPER=stepper_x")
	v.Gcode.Run_script_from_command("CLOSE_STEALTHCHOP STEPPER=stepper_y")
	v.Gcode.Run_script_from_command(fmt.Sprintf("M109 S%f", v.PrintInfo.ExtruderTemp))

	toolhead := v.Printer.Lookup_object("toolhead", object.Sentinel{}).(*Toolhead)
	gcode_move := v.Printer.Lookup_object("gcode_move", object.Sentinel{}).(*GCodeMove)

	// 恢复Z轴实际坐标
	pos := toolhead.Get_position()
	last_z, err := v.zAxisTracker.get_z()
	if v.PrintInfo.UseACE && err == nil {
		v.zAxisTracker.clear()
		// 获取当前位置的Z轴BedMesh偏移
		toolhead.Set_position([]float64{pos[0], pos[1], 0., pos[3]}, []int{2})
		offset_z := gcode_move.last_position[2]

		pos[2] += (last_z - offset_z)
	} else {
		pos[2] += v.PrintInfo.Axis_state["POWER_OFF"].Last_position[2]
	}
	toolhead.Set_position(pos, []int{2})

	// 断电续打首层降速
	v.PrintInfo.FristLayer = true
	v.PrintInfo.FristLayerSpeed = v.PrintInfo.Axis_state["POWER_OFF"].Speed
	v.PrintInfo.FristLayerSpeedFactor = v.PrintInfo.Axis_state["POWER_OFF"].Speed_factor
	v.Gcode.Run_script_from_command("M220 S50")
	if v.PrintInfo.Accel != nil {
		v.Gcode.Run_script_from_command(fmt.Sprintf("M204 S%f", *v.PrintInfo.Accel/2))
		v.PrintInfo.FristLayerAccel = *v.PrintInfo.Accel
	} else {
		v.Gcode.Run_script_from_command(fmt.Sprintf("M204 S%f", toolhead.Max_accel/2))
		v.PrintInfo.FristLayerAccel = toolhead.Max_accel
	}

	if !v.PrintInfo.UseACE {
		v.Gcode.Run_script_from_command("G92 E0")
		v.Gcode.Run_script_from_command("G1 E100 F300")
		v.Gcode.Run_script_from_command("M400")
		v.Gcode.Run_script_from_command("M106 S255")
		v.Gcode.Run_script_from_command("G4 P3000")
		v.Gcode.Run_script_from_command("G90")
		v.Gcode.Run_script_from_command(fmt.Sprintf("M106 S%f", v.PrintInfo.Fan_state*255))
	}

	toolhead.Wait_moves()

	// 重置E轴坐标
	// pos = toolhead.Get_position()
	// pos[3] = v.PrintInfo.Axis_state["POWER_OFF"].Last_position[3]
	// toolhead.Set_position(pos, []int{3})

	// 移动到断电前的位置
	// toolhead.Manual_move([]interface{}{last_pos[0], last_pos[1], nil}, 200)
	// toolhead.Manual_move([]interface{}{nil, nil, last_pos[2]}, 8)

	// 重置E轴坐标
	v.PrintInfo.Axis_state["POWER_OFF"].Last_position[3] = toolhead.Get_position()[3]

	gcode_move.Saved_states = v.PrintInfo.Axis_state

	v.Gcode.Run_script_from_command(fmt.Sprintf("RESTORE_GCODE_STATE NAME=POWER_OFF MOVE=1 MOVE_SPEED=%.4f", 50.))
	v.Print_stats.Note_start()
	v.Print_stats.state = Printing_
	v.Print_stats.printing = true
	v.IsStartPrintCmd = true
	return v.Work_handler(eventtime)
}

func (self *VirtualSD) handle_print_exit() {
	value.StaticValue.Debug.Print("do handle_print_exit")
	self.PrintInfo.FristLayer = false
	self.Sdcard_dirname = value.InitValue().Config.Sys.PrinterPath
	self.Gcode.Run_script_from_command("G92 E0")
	self.Gcode.Run_script_from_command("TURN_OFF_HEATERS")
	self.Gcode.Run_script_from_command("M107")
	self.Gcode.Run_script_from_command("OPEN_STEALTHCHOP STEPPER=stepper_x")
	self.Gcode.Run_script_from_command("OPEN_STEALTHCHOP STEPPER=stepper_y")
	self.Gcode.Run_script_from_command("M900 K0.02")
	toolhead := MustLookupToolhead(self.Printer)
	self.Gcode.Run_script_from_command(fmt.Sprint("M204 S", toolhead.Limit_Max_accel))
	self.Gcode.Run_script_from_command("G90")
	// 取消断电续打记录
	virtualSD := self.Printer.Lookup_object("virtual_sdcard", object.Sentinel{}).(*VirtualSD)
	virtualSD.Cancel_record()
}

func (self *VirtualSD) handle_print_cancel() {
	value.StaticValue.Debug.Print("do handle_print_cancel")
	self.PrintInfo.FristLayer = false
	self.Sdcard_dirname = value.InitValue().Config.Sys.PrinterPath
	self.Gcode.Run_script_from_command("G92 E0")
	self.Gcode.Run_script_from_command("TURN_OFF_HEATERS")
	self.Gcode.Run_script_from_command("M107")
	self.Gcode.Run_script_from_command("HOME_XY")
	self.Gcode.Run_script_from_command("M84")
	self.Gcode.Run_script_from_command("OPEN_STEALTHCHOP STEPPER=stepper_x")
	self.Gcode.Run_script_from_command("OPEN_STEALTHCHOP STEPPER=stepper_y")
	self.Gcode.Run_script_from_command("M900 K0.02")
	toolhead := MustLookupToolhead(self.Printer)
	self.Gcode.Run_script_from_command(fmt.Sprint("M204 S", toolhead.Limit_Max_accel))
	self.Gcode.Run_script_from_command("G90")
	// 取消断电续打记录
	virtualSD := self.Printer.Lookup_object("virtual_sdcard", object.Sentinel{}).(*VirtualSD)
	virtualSD.Cancel_record()
}

// Background work timer
func (self *VirtualSD) Work_handler(eventtime float64) float64 {
	defer func() {
		self.IsPrint = false
	}()
	log.Printf("Starting SD card print (position %d)", self.File_position)
	self.Reactor.Unregister_timer(self.Work_timer)
	_, err := self.Current_file.Seek(int64(self.File_position), 0)
	if err != nil {
		//value.StaticValue.Error.Println(sys.GetGID(),"self.Current_file",self.Current_file,"self.File_position",self.File_position)
		value.StaticValue.Error.Println("virtual_sdcard seek", err.Error())
		self.Work_timer = nil
		if self.Current_file != nil {
			self.Current_file.Close()
			self.Current_file = nil
		}
		return constants.NEVER
	}

	print_stats := self.Printer.Lookup_object("print_stats", object.Sentinel{}).(*PrintStats)
	// 设置打印状态为heating
	if print_stats.state == Standby_ || print_stats.printing == false {
		print_stats.state = Heating_
		self.Print_stats.Note_start()
	} else if print_stats.state != Printing_ {
		print_stats.state = Printing_
		self.Print_stats.Note_start()
	}
	//self.Print_stats.Note_start()
	gcode_mutex := self.Gcode.Get_mutex()
	partial_input := ""
	lines := []string{}
	error_message := ""

	num := 0
	var accel *float64
	extruder_heater_check := self.Printer.Lookup_object("verify_heater extruder", object.Sentinel{}).(*HeaterCheck)
	heater_bed_heater_check := self.Printer.Lookup_object("verify_heater heater_bed", object.Sentinel{}).(*HeaterCheck)
	for !self.Must_pause_work {
		if len(lines) <= 0 {
			// Read more data
			data := make([]byte, 8192)
			l, err := self.Current_file.Read(data)
			if err != nil && err != io.EOF {

				value.StaticValue.Error.Println("virtual_sdcard", err)
				if self.Current_file != nil {
					self.Current_file.Close()
					self.Current_file = nil
				}
				self.Gcode.Respond_raw("Read printing file error")
				error_message = "Printing file error"
				break
			}
			if l <= 0 {
				// End of file
				self.Current_file.Close()
				self.Current_file = nil
				log.Println("Finished SD card print")
				self.Gcode.Respond_raw("Done printing file")
				break
			}
			lines = strings.Split(string(data), "\n")
			lines[0] = partial_input + lines[0]
			partial_input = lines[len(lines)-1]
			lines = append([]string{}, lines[:len(lines)-1]...)
			lines = append([]string{}, util.Reverse(lines)...)
			self.Reactor.Pause(constants.NOW)
			continue
		}
		// Pause if any other request is pending in the gcode class
		if gcode_mutex.Test() {
			self.Reactor.Pause(self.Reactor.Monotonic() + 0.100)
			continue
		}
		// Dispatch command
		self.Cmd_from_sd = true
		line := lines[len(lines)-1]
		lines = append([]string{}, lines[:len(lines)-1]...)
		next_file_position := self.File_position + len(line) + 1
		line = strings.TrimRight(line, "\r")
		self.Next_file_position = next_file_position
		// value.StaticValue.Debug.Println(line)
		// if line == "" {
		// continue
		// }

		if strings.Contains(line, "M600") && fristM600 {
			fristM600 = false
		} else if strings.Contains(line, "M600") {
			fristM600 = true
			continue
		}
		if strings.Contains(line, "M204") {
			params := strings.Split(line, " ")[1]
			if params != "" {
				accel_value, err := strconv.ParseFloat(params[1:], 64)
				if err == nil {
					accel = &accel_value
				}
			}
		}
		num++

		runtime.ReadMemStats(self.bm)
		//log.Println(time.Now().UnixMilli(), self.bm.Sys/1000000, num, line)
		// 获取当前层号 cura
		if strings.HasPrefix(line, ";LAYER:") {
			self.Printer.Send_event("virtual_sdcard:change_layer", nil)

			self.currLayer += 1
			if self.enableContinuePrint == 1 && self.currLayer > 1 {
				self.SAVE_POWER_OFF_STATE(accel)

				// 保存层高
				gmove := self.Printer.Lookup_object("gcode_move", object.Sentinel{}).(*GCodeMove)
				self.zAxisTracker.clear()
				self.zAxisTracker.update_layer_height(gmove.last_position[2])
			}
			if self.PrintInfo.FristLayer {
				self.PrintInfo.FristLayer = false
				// M220
				gmove := self.Printer.Lookup_object("gcode_move", object.Sentinel{}).(*GCodeMove)
				gmove.M220(self.PrintInfo.FristLayerSpeedFactor)
				// M204
				toolhead := self.Printer.Lookup_object("toolhead", object.Sentinel{}).(*Toolhead)
				toolhead.M204(self.PrintInfo.FristLayerAccel)
			}
			// 获取当前层号 PrusaSlicer
		} else if strings.Contains(line, ";LAYER_CHANGE") {
			self.Printer.Send_event("virtual_sdcard:change_layer", nil)

			self.currLayer += 1
			if self.enableContinuePrint == 1 && self.currLayer > 1 {
				self.SAVE_POWER_OFF_STATE(accel)
			}
			if self.PrintInfo.FristLayer {
				self.PrintInfo.FristLayer = false
				// M220
				gmove := self.Printer.Lookup_object("gcode_move", object.Sentinel{}).(*GCodeMove)
				gmove.M220(self.PrintInfo.FristLayerSpeedFactor)
				// M204
				toolhead := self.Printer.Lookup_object("toolhead", object.Sentinel{}).(*Toolhead)
				toolhead.M204(self.PrintInfo.FristLayerAccel)
			}
		} else if strings.Contains(line, ";Z:") { // Prusa&Orca: 保存层高
			if self.enableContinuePrint == 1 {
				layer_height, err := strconv.ParseFloat(strings.Split(line, ":")[1], 64)
				if err == nil {
					self.zAxisTracker.clear()
					self.zAxisTracker.update_layer_height(layer_height)
				}
			}
		}
		isBreak := false

		self.tryCatchWork_handler1(line, &error_message, &isBreak)
		if isBreak {
			if self.Current_file != nil {
				self.Current_file.Close()
				self.Current_file = nil
			}
			value.StaticValue.Error.Print(error_message)
			break
		}
		self.Cmd_from_sd = false
		self.File_position = self.Next_file_position
		// Do we need to skip around?
		if self.Next_file_position != next_file_position {
			value.StaticValue.Debug.Println("seek to position: ", self.File_position)
			_, err := self.Current_file.Seek(int64(self.File_position), 0)
			if err != nil {
				value.StaticValue.Error.Println("virtual_sdcard seek")
				self.Work_timer = nil
				if self.Current_file != nil {
					self.Current_file.Close()
					self.Current_file = nil
				}
				return constants.NEVER
			}
			lines = []string{}
			partial_input = ""
		}
		if heater_bed_heater_check.err {
			error_message = kerror.HeaterBedHeatingError.Message
			return constants.NEVER
		}
		if extruder_heater_check.err {
			error_message = kerror.ExtruderHeatingError.Message
			return constants.NEVER
		}
	}

	//log.Printf("Exiting SD card print (position %d)", self.File_position)
	self.Work_timer = nil
	self.Cmd_from_sd = false
	if error_message != "" {
		self.Cancel_record()
		defer self.Print_stats.Note_error(error_message)
		self.Printer.Send_event("virtual_sdcard:cancel_print", nil)
		self.handle_print_cancel()
		fristM600 = true
	} else if self.Current_file != nil {
		self.Print_stats.Note_pause()
	} else {
		self.Cancel_record()
		defer self.Print_stats.Note_complete()
		self.Printer.Send_event("virtual_sdcard:finish_print", nil)
		self.handle_print_exit()
		fristM600 = true
	}
	return constants.NEVER
}

var timePatterns = []struct {
	re         *regexp.Regexp
	multiplier int
}{{
	regexp.MustCompile(`(\d+)d`),
	24 * 60 * 60,
}, {
	regexp.MustCompile(`(\d+)h`),
	60 * 60,
}, {
	regexp.MustCompile(`(\d+)m`),
	60,
}, {
	regexp.MustCompile(`(\d+)s`),
	1,
},
}

func parseGcodeSlicer(line string) string {
	for _, slicer := range SupportedSlicer {
		if strings.Contains(line, slicer) {
			return slicer
		}
	}

	tag := "; generated by "
	if strings.Contains(line, tag) {
		first := strings.IndexAny(line, tag) + len(tag)
		last := first + strings.IndexAny(line[first:], " ")
		return strings.TrimSpace(line[first:last])
	}
	return ""
}

/*
获取打印总层数
获取打印总时间
*/
func (self *VirtualSD) getCodeInfo(filePath string) {
	file, err := os.Open(filePath)
	if err != nil {
		value.StaticValue.Error.Println(err)
		return
	}
	defer file.Close()
	var end int64
	var curr = 0
	fis, err := file.Stat()
	if err != nil {
		value.StaticValue.Error.Println(err)
		return
	}
	size := fis.Size()
	r := bufio.NewReader(file)
	// line, _ := r.ReadString('\n')
	i := 0
	index, flag := 0, 0
	for ; i <= 30; i++ {
		line, _ := r.ReadString('\n')

		if slicer := parseGcodeSlicer(line); slicer != "" {
			self.gcodeSlicer = slicer
		}

		// PrusaSlicer
		if strings.Contains(line, PRUSASLICER) || strings.Contains(line, ANYCUBICSLICER) || strings.Contains(line, ORCA) {
			if size >= FILE_LIMIT {
				index = -int(float64(size) * UNIT_MIN)
			} else {
				index = -int(float64(size) * UNIT_MAX)
			}
			end = size
			// file.Seek(int64(curr), io.SeekEnd)
			flag = io.SeekEnd
			break
			// cura
		} else if strings.Contains(line, CURA_STEAMENGINE) || strings.Contains(line, BAMBUSTUDIO) {
			if size >= FILE_LIMIT {
				end = int64(float64(size) * UNIT_MIN)
			} else {
				end = int64(float64(size) * UNIT_MAX)
			}
			// file.Seek(0, 0)
			break
		}

	}
	// unknow slicer
	if i > 30 {
		return
	}
	file.Seek(0, 0)
	// 从指定位置读取文件
	file.Seek(int64(index), flag)
	r = bufio.NewReader(file)
	for {
		line, err := r.ReadString('\n')
		if err != nil {
			break
		}
		curr += len([]byte(line))
		// PrusaSlicer 获取层号
		if strings.HasPrefix(line, "; AFTER_LAYER_CHANGE") {
			numStr := strings.Split(line, " ")[2]
			self.layerCount, err = strconv.Atoi(numStr)
			if err != nil {
				value.StaticValue.Error.Println(err)
			}
			// prusa的层号从0开始数
			self.layerCount += 1
			continue
		}
		// PrusaSlicer 获取打印时间
		if strings.HasPrefix(line, "; estimated printing time") {
			line = strings.Split(line, "=")[1]
			var sum int
			for _, pattern := range timePatterns {
				if matched := pattern.re.FindStringSubmatch(line); len(matched) == 2 {
					val, _ := strconv.Atoi(matched[1])
					sum += val * pattern.multiplier
				}
			}
			// 计算打印时间
			self.totalTime = sum
			continue
		}
		// PrusaSlicer 获取所需材料 获取打印时间
		if strings.HasPrefix(line, "; filament used [cm3]") {
			arr := strings.Split(line, "=")
			self.PrintInfo.FilamentTotalUses = strings.TrimSpace(arr[1]) + "m"
			continue
		}

		// cura 获取层号
		if strings.HasPrefix(line, ";LAYER_COUNT:") {
			arr := strings.Split(line, ":")
			self.layerCount, _ = strconv.Atoi(strings.TrimSpace(arr[len(arr)-1]))
			continue
		}
		// cura 获取时间
		if strings.HasPrefix(line, ";TIME:") {
			arr := strings.Split(line, ":")
			self.totalTime, _ = strconv.Atoi(strings.TrimSpace(arr[len(arr)-1]))
			continue
		}
		// cura 获取所需材料
		if strings.HasPrefix(line, ";Filament used:") {
			arr := strings.Split(line, ":")
			self.PrintInfo.FilamentTotalUses = strings.TrimSpace(arr[1])
			continue
		}
		// 获取打印热床温度
		if strings.HasPrefix(line, "M190 ") {
			tempStr := strings.Split(line, " ")[1]
			if HotbedTemp, err := strconv.ParseFloat(strings.TrimSpace(tempStr[1:]), 64); err == nil && HotbedTemp > 0 {
				self.PrintInfo.HotbedTemp = HotbedTemp
			}
			continue
		}
		// 获取打印挤出机温度
		if strings.HasPrefix(line, "M109 ") {
			tempStr := strings.Split(line, " ")[1]
			if ExtruderTemp, err := strconv.ParseFloat(strings.TrimSpace(tempStr[1:]), 64); err == nil && ExtruderTemp > 0 {
				self.PrintInfo.ExtruderTemp = ExtruderTemp
			}
			continue
		}

		// 层高 - Cura
		if strings.HasPrefix(line, ";Layer height:") {
			fields := strings.Split(line, ":")
			self.layerHeight, _ = strconv.ParseFloat(strings.TrimSpace(fields[len(fields)-1]), 64)
			continue
		}

		// 层高 - Prusa
		if strings.HasPrefix(line, "; layer_height =") {
			fields := strings.Split(line, "=")
			self.layerHeight, _ = strconv.ParseFloat(strings.TrimSpace(fields[len(fields)-1]), 64)
			continue
		}

		// 线材类型 - Prusa/Anycubic
		if strings.HasPrefix(line, "; filament_type =") || strings.HasPrefix(line, ";Filament type =") {
			fields := strings.Split(line, "=")
			self.filamentType = strings.TrimSpace(fields[len(fields)-1])
			continue
		}

		// 线材类型 - IceSL
		if strings.HasPrefix(line, "; filament_type :") {
			fields := strings.Split(line, ":")
			self.filamentType = strings.TrimSpace(fields[len(fields)-1])
			continue
		}

		// 层数 - Anycubic
		if strings.HasPrefix(line, "; total_layers =") {
			fields := strings.Split(line, "=")
			self.layerCount, _ = strconv.Atoi(strings.TrimSpace(fields[len(fields)-1]))
			continue
		}

		// 巡航速度 - PrusaSlicer/Anycubic
		// ; travel_speed = 300
		if strings.HasPrefix(line, "; travel_speed =") {
			fields := strings.Split(line, "=")
			self.travelSpeed, _ = strconv.ParseFloat(strings.TrimSpace(fields[len(fields)-1]), 64)
			continue
		}

		//  填充密度 - PrusaSlicer/Anycubic
		// ; fill_density = 10%
		if strings.HasPrefix(line, "; fill_density =") {
			fields := strings.Split(line, "=")
			v := strings.TrimSpace(fields[len(fields)-1])
			v = strings.TrimRight(v, "%")
			self.fillDensity, _ = strconv.ParseFloat(v, 64)
			continue
		}

		// 自动支撑 - PrusaSlicer/Anycubic
		// ; support_material_auto = 1
		if strings.HasPrefix(line, "; support_material_auto =") {
			fields := strings.Split(line, "=")
			self.supportMaterialAuto, _ = strconv.Atoi(strings.TrimSpace(fields[len(fields)-1]))
			continue
		}

		// 回抽长度 - PrusaSlicer
		// ; retract_length = 2
		// ; retract_length = 2,2,2,2
		if strings.HasPrefix(line, "; retract_length =") {
			fields := strings.Split(line, "=")
			len := strings.TrimSpace(fields[len(fields)-1])
			if strings.Contains(len, ",") {
				len = strings.Split(len, ",")[0]
			}
			self.filamentRetractLength, _ = strconv.ParseFloat(len, 64)
			continue
		}

		// 回抽长度 - Anycubic
		// ; filament_retract_length = 0
		if strings.HasPrefix(line, "; filament_retract_length =") {
			fields := strings.Split(line, "=")
			self.filamentRetractLength, _ = strconv.ParseFloat(strings.TrimSpace(fields[len(fields)-1]), 64)
			continue
		}

		// 回抽长度 - OrcaSlicer
		// ; retract_length = 1.5
		if strings.HasPrefix(line, "; retraction_length =") {
			fields := strings.Split(line, "=")
			self.filamentRetractLength, _ = strconv.ParseFloat(strings.TrimSpace(fields[len(fields)-1]), 64)
			continue
		}

		// 回抽抬升 - PrusaSlicer
		// ; retract_lift = 0.25
		// ; retract_lift = 0.25,0.25,0.25,0.25
		if strings.HasPrefix(line, "; retract_lift =") {
			fields := strings.Split(line, "=")
			lift := strings.TrimSpace(fields[len(fields)-1])
			if strings.Contains(lift, ",") {
				lift = strings.Split(lift, ",")[0]
			}
			self.retractLift, _ = strconv.ParseFloat(lift, 64)
			continue
		}

		// 回抽抬升 - OrcaSlicer
		// ; z_hop = 0.25
		if strings.HasPrefix(line, "; z_hop =") {
			fields := strings.Split(line, "=")
			self.retractLift, _ = strconv.ParseFloat(strings.TrimSpace(fields[len(fields)-1]), 64)
			continue
		}

		// 延时摄影模式 - OrcaSlicer
		// ; timelapse_type = 1
		if strings.HasPrefix(line, "; timelapse_type =") {
			fields := strings.Split(line, "=")
			self.timelapseType, _ = strconv.Atoi(strings.TrimSpace(fields[len(fields)-1]))
			continue
		}

		// 热床温度 - PrusaSlicer/Anycubic
		// ; bed_temperature = 60
		if strings.HasPrefix(line, "; bed_temperature =") || strings.HasPrefix(line, "; first_layer_bed_temperature =") {
			fields := strings.Split(line, "=")
			self.bedTemperature, _ = strconv.ParseFloat(strings.TrimSpace(fields[len(fields)-1]), 64)
			self.PrintInfo.HotbedTemp = self.bedTemperature
			continue
		}
		// 挤出机温度 - PrusaSlicer/Anycubic
		// ; temperature = 215
		if strings.HasPrefix(line, "; nozzle_temperature =") || strings.HasPrefix(line, "; first_layer_temperature =") {
			fields := strings.Split(line, "=")
			self.extruderTemperature, _ = strconv.ParseFloat(strings.TrimSpace(fields[len(fields)-1]), 64)
			self.PrintInfo.ExtruderTemp = self.extruderTemperature
			continue
		}

		// 轮廓数量 - PrusaSlicer/Anycubic
		// ; perimeters = 2
		if strings.HasPrefix(line, "; perimeters =") {
			fields := strings.Split(line, "=")
			self.perimeters, _ = strconv.Atoi(strings.TrimSpace(fields[len(fields)-1]))
			continue
		}

		// 轮廓挤出宽度 - PrusaSlicer/Anycubic
		// ; perimeter_extrusion_width = 0.45
		if strings.HasPrefix(line, "; perimeter_extrusion_width =") {
			fields := strings.Split(line, "=")
			self.perimeterExtrusionWidth, _ = strconv.ParseFloat(strings.TrimSpace(fields[len(fields)-1]), 64)
			continue
		}

		// 轮廓速度 - PrusaSlicer/Anycubic
		// ; perimeter_speed = 300
		if strings.HasPrefix(line, "; perimeter_speed =") {
			fields := strings.Split(line, "=")
			self.perimeterSpeed, _ = strconv.ParseFloat(strings.TrimSpace(fields[len(fields)-1]), 64)
			continue
		}

		// 打印平台附着 - PrusaSlicer/Anycubic
		// ; brim_type = outer_only
		if strings.HasPrefix(line, "; brim_type =") {
			fields := strings.Split(line, "=")
			self.brimType = strings.TrimSpace(fields[len(fields)-1])
			continue
		}

		// 打印机显示名称 - PrusaSlicer/Anycubic
		//; printer_settings_id = Anycubic Kobra 2 Pro
		if strings.HasPrefix(line, "; printer_settings_id") {
			fields := strings.Split(line, "=")
			self.printerSettingsId = strings.TrimSpace(fields[len(fields)-1])
			continue
		}

		// 料盒多色打印 - PrusaSlicer/Anycubic
		// T0/T1/T2...
		line = strings.TrimSpace(line)
		if strings.HasPrefix(line, "; extruder_colour") {
			fields := strings.Split(line, "=")
			value := strings.Trim(fields[len(fields)-1], " \"")
			if len(value) > 0 {
				for _, colour := range strings.Split(value, ";") {
					if len(colour) > 0 {
						r, _ := strconv.ParseInt(string(colour[1:3]), 16, 0)
						g, _ := strconv.ParseInt(string(colour[3:5]), 16, 0)
						b, _ := strconv.ParseInt(string(colour[5:7]), 16, 0)
						self.extruderColour = append(self.extruderColour, Colour{R: int(r), G: int(g), B: int(b)})
					}
				}
			}
			continue
		}

		if strings.HasPrefix(line, "; filament_colour") {
			fields := strings.Split(line, "=")
			value := strings.Trim(fields[len(fields)-1], " \"")
			if len(value) > 0 {
				for _, colour := range strings.Split(value, ";") {
					if len(colour) > 0 {
						r, _ := strconv.ParseInt(string(colour[1:3]), 16, 0)
						g, _ := strconv.ParseInt(string(colour[3:5]), 16, 0)
						b, _ := strconv.ParseInt(string(colour[5:7]), 16, 0)
						self.filamentColour = append(self.filamentColour, Colour{R: int(r), G: int(g), B: int(b)})
					}
				}
			}
			continue
		}

		if strings.HasPrefix(line, "; flush_multiplier =") {
			fields := strings.Split(line, "=")
			self.flushMultiplier, _ = strconv.ParseFloat(strings.TrimSpace(fields[len(fields)-1]), 64)
			continue
		}

		if strings.HasPrefix(line, "; flush_volumes_matrix") {
			fields := strings.Split(line, "=")
			matrix := strings.Trim(fields[len(fields)-1], " ")
			if len(matrix) > 0 {
				for _, m_str := range strings.Split(matrix, ",") {
					m, _ := strconv.ParseInt(m_str, 10, 0)
					self.flushVolumesMatrix = append(self.flushVolumesMatrix, int(m))
				}
			}
			continue
		}

		if strings.HasPrefix(line, "; flush_volumes_vector") {
			fields := strings.Split(line, "=")
			vector := strings.Trim(fields[len(fields)-1], " ")
			if len(vector) > 0 {
				for _, v_str := range strings.Split(vector, ",") {
					v, _ := strconv.ParseInt(v_str, 10, 0)
					self.flushVolumesVector = append(self.flushVolumesVector, int(v))
				}
			}
			continue
		}

		// 模型尺寸 - Anycubic
		// ; model_size = 3.99548,3.5,1.69282
		if strings.HasPrefix(line, "; model_size = ") {
			fields := strings.Split(line, "=")
			modelSizes := strings.Split(strings.TrimSpace(fields[len(fields)-1]), ",")
			if len(modelSizes) == 3 {
				self.modelSizeX, _ = strconv.ParseFloat(modelSizes[0], 64)
				self.modelSizeY, _ = strconv.ParseFloat(modelSizes[1], 64)
				self.modelSizeZ, _ = strconv.ParseFloat(modelSizes[2], 64)
			}
			continue
		}

		if int64(curr) >= end {
			break
		}
	}
}

func (self *VirtualSD) register_whitelist(name string) {
	self.gcodeWhiteList = append(self.gcodeWhiteList, name)
}

func (self *VirtualSD) in_whitelist(gcmd string) bool {
	for _, name := range self.gcodeWhiteList {
		if strings.HasPrefix(gcmd, name) {
			return true
		}
	}
	return false
}

func (self *VirtualSD) tryCatchWork_handler1(line string, error_message *string, isBread *bool) {
	defer func() {
		if err := recover(); err != nil {
			if err == "exit" {
				panic(err)
			}
			//捕获处理catch CommandError
			e, ok1 := err.(*CommandError)
			if ok1 {
				*error_message = e.E
				self.tryCatchWork_handler11()
				//*isBread = true
				//return
			} else if e1, ok1 := err.(string); ok1 {
				*error_message = e1
			} else if e2, ok2 := err.(*kerror.Error); ok2 {
				*error_message = e2.Error()
			}

			value.StaticValue.Error.Println("virtual_sdcard dispatch", line, err, string(debug.Stack()))
			if *error_message == "" {
				self.Gcode.Respond_raw("print file error")
			} else {
				self.Gcode.Respond_raw(*error_message)
			}

			*isBread = true
			return
		}
	}()
	*isBread = false
	lStr := strings.ToUpper(strings.TrimSpace(line))
	if strings.HasPrefix(lStr, "M") || strings.HasPrefix(lStr, "G") || strings.HasPrefix(lStr, "T") || self.in_whitelist(lStr) {
		// HACK: Gcode文件识别到M117命令时才将状态设置为打印中
		if strings.Contains(line, "M117") {
			//设置状态为打印中
			if !self.IsStartPrintCmd {
				print_stats := self.Printer.Lookup_object("print_stats", object.Sentinel{}).(*PrintStats)
				print_stats.state = Printing_
				print_stats.printing = true
				self.Printer.Send_event("virtual_sdcard:start_print", nil)
				self.Cancel_record() // 开始新的打印，清除旧的断电续打记录
				self.IsStartPrintCmd = true
			}
			self.Gcode.Run_script(line)
		} else {
			self.Gcode.Run_script(line)
		}
	}
}
func (self *VirtualSD) tryCatchWork_handler11() {
	defer func() {
		if err := recover(); err != nil {
			if err == "exit" {
				panic(err)
			}
			//捕获处理catch CommandError
			value.StaticValue.Error.Println("virtual_sdcard on_error", err, string(debug.Stack()))
		}
	}()
	s, _ := self.On_error_gcode.(*TemplateWrapper).Render(nil)
	lStr := strings.TrimSpace(s)
	if strings.HasPrefix(lStr, "M") || strings.HasPrefix(lStr, "m") || strings.HasPrefix(lStr, "G") || strings.HasPrefix(lStr, "g") {
		self.Gcode.Run_script(s)
	}
}
func Load_config_virtual_sdcard(config *ConfigWrapper) interface{} {
	return NewVirtualSD(config)
}
