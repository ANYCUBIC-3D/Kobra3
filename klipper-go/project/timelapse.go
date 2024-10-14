package project

import (
	"fmt"
	"k3c/common/utils/object"
	"k3c/common/value"
	"net"
	"strings"
)

const (
	CAM_ACTION_START string = "0"
	CAM_ACTION_SNAP  string = "1"
	CAM_ACTION_STOP  string = "2"
)

type cameraHelper struct {
	port int
}

func NewCameraHelper(port int) *cameraHelper {
	return &cameraHelper{
		port: port,
	}
}

func (cam *cameraHelper) do_snapshot(action string) {
	udpServer, err := net.ResolveUDPAddr("udp", fmt.Sprintf(":%d", cam.port))
	if err != nil {
		value.StaticValue.Debug.Println("ResolveUDPAddr failed:", err.Error())
		return
	}

	conn, err := net.DialUDP("udp", nil, udpServer)
	if err != nil {
		value.StaticValue.Debug.Println("Listen failed:", err.Error())
		return
	}

	//close the connection
	defer conn.Close()

	_, err = conn.Write([]byte(action))
	if err != nil {
		value.StaticValue.Debug.Println("Write data failed:", err.Error())
		return
	}
}

func (cam *cameraHelper) start() {
	cam.do_snapshot(CAM_ACTION_START)
}

func (cam *cameraHelper) stop() {
	cam.do_snapshot(CAM_ACTION_STOP)
}

func (cam *cameraHelper) snap() {
	cam.do_snapshot(CAM_ACTION_SNAP)
}

type Timelapse struct {
	camera *cameraHelper

	enable  bool
	started bool

	snap_x_pos float64
	snap_y_pos float64

	printer        *Printer
	reactor        IReactor
	gcode          *GCodeDispatch
	virtual_sdcard *VirtualSD
}

func NewTimelapse(config *ConfigWrapper) *Timelapse {
	self := &Timelapse{
		enable:  false,
		started: false,
		camera:  NewCameraHelper(10086),
	}

	self.snap_x_pos = config.Getfloat("snap_x_pos", 270, 0, 0, 0, 0, true)
	self.snap_y_pos = config.Getfloat("snap_y_pos", 200, 0, 0, 0, 0, true)

	self.printer = config.Get_printer()
	self.reactor = self.printer.Get_reactor()
	self.gcode = self.printer.Lookup_object("gcode", object.Sentinel{}).(*GCodeDispatch)
	self.virtual_sdcard = self.printer.Lookup_object("virtual_sdcard", object.Sentinel{}).(*VirtualSD)

	self.printer.Register_event_handler("project:ready", self._handle_ready)
	self.printer.Register_event_handler("project:disconnect", self._disconnect)

	self.printer.Register_event_handler("virtual_sdcard:start_print", self.start)
	self.printer.Register_event_handler("virtual_sdcard:cancel_print", self.cancel)
	self.printer.Register_event_handler("virtual_sdcard:finish_print", self.stop)
	self.printer.Register_event_handler("virtual_sdcard:change_layer", self.snapshot)

	return self
}

func (self *Timelapse) start(args []interface{}) error {
	if self.enable {
		value.StaticValue.Debug.Println("timelapse started")
		self.camera.start()
		self.started = true
	}
	return nil
}

func (self *Timelapse) homedXY() bool {
	kin := self.printer.Lookup_object("toolhead", object.Sentinel{}).(*Toolhead).Get_kinematics().(IKinematics)
	homed_axes := kin.Get_status(self.reactor.Monotonic())["homed_axes"].(string)
	return strings.Contains(homed_axes, "x") && strings.Contains(homed_axes, "y")
}

func (self *Timelapse) cancel(args []interface{}) error {
	if self.started {
		self.camera.stop()
		self.started = false
		value.StaticValue.Debug.Println("timelapse canceled")
	}
	return nil
}

func (self *Timelapse) stop(args []interface{}) error {
	if self.started {
		if self.virtual_sdcard.timelapseType == 0 {
			self.camera.snap()
			self.gcode.Run_script_from_command("G4 P400")
			self.camera.stop()
			self.started = false
		} else if self.virtual_sdcard.timelapseType == 1 {
			is_homed := self.homedXY()
			if !is_homed {
				self.gcode.Run_script_from_command("G28 X Y")
			}

			// move to safe pos & take a photo
			self.gcode.Run_script_from_command(fmt.Sprintf("G1 X%f Y%f F12000", self.snap_x_pos, self.snap_y_pos))
			self.gcode.Run_script_from_command("M400")
			self.camera.snap()
			self.gcode.Run_script_from_command("G4 P400")

			self.camera.stop()
			self.started = false

			if !is_homed {
				self.gcode.Run_script_from_command("M84")
			}
		}

		value.StaticValue.Debug.Println("timelapse stopped")
	}
	return nil
}

func (self *Timelapse) snapshot(args []interface{}) error {
	if self.started {
		if self.virtual_sdcard.timelapseType == 0 {
			self.camera.snap()
		} else if self.virtual_sdcard.timelapseType == 1 {
			retraction_length := self.virtual_sdcard.filamentRetractLength
			if retraction_length <= 0 {
				retraction_length = 1.5
			}

			value.StaticValue.Debug.Println("do snapshot")

			// lift a little and move to safe pos
			self.gcode.Run_script_from_command(fmt.Sprintf("G91\nG1 Z0.4 E-%f F300\nG90", retraction_length))
			self.gcode.Run_script_from_command("SAVE_GCODE_STATE NAME=timelapse")
			self.gcode.Run_script_from_command(fmt.Sprintf("G1 X%f Y%f F12000", self.snap_x_pos, self.snap_y_pos))
			self.gcode.Run_script_from_command("M400")

			// take a photo
			self.camera.snap()

			// restore position
			self.gcode.Run_script_from_command("G4 P400")
			self.gcode.Run_script_from_command("RESTORE_GCODE_STATE NAME=timelapse MOVE=1 MOVE_SPEED=12000")
			self.gcode.Run_script_from_command(fmt.Sprintf("G91\nG1 E%f F300\nG90", retraction_length))
		}
	}
	return nil
}

func (self *Timelapse) _handle_ready(args []interface{}) error {
	self.gcode.Register_command("SNAPSHOT", self.cmd_SNAPSHOT, false, "")
	self.gcode.Register_command("TIMELAPSE", self.cmd_TIMELAPSE, false, "")

	wh := MustLookupWebhooks(self.printer)
	wh.Register_endpoint("timelapse/set_config", self.handle_set_config)
	wh.Register_endpoint("timelapse/get_config", self.handle_get_config)

	return nil
}

func (self *Timelapse) _disconnect(args []interface{}) error {
	return nil
}

func (self *Timelapse) cmd_SNAPSHOT(arg interface{}) error {
	gcmd := arg.(*GCodeCommand)
	switch gcmd.Get_int("V", 0, nil, nil) {
	case 0:
		self.camera.start()
	case 1:
		self.camera.snap()
	case 2:
		self.camera.stop()
	}
	return nil
}

func (self *Timelapse) cmd_TIMELAPSE(arg interface{}) error {
	gcmd := arg.(*GCodeCommand)

	switch gcmd.Get_int("V", -1, nil, nil) {
	case 0:
		self.camera.start()
	case 1:
		self.camera.snap()
	case 2:
		self.camera.stop()
	}

	switch gcmd.Get_int("E", -1, nil, nil) {
	case 0:
		self.enable = false
	case 1:
		self.enable = true
	}

	return nil
}

func (self *Timelapse) handle_set_config(web_request *WebRequest) (interface{}, error) {
	enable := int(web_request.get_float("enable", -1.0))
	if enable == 0 {
		self.enable = false
	} else if enable == 1 {
		self.enable = true
	}
	web_request.Send(nil)
	return nil, nil
}

func (self *Timelapse) handle_get_config(web_request *WebRequest) (interface{}, error) {
	enable := 0
	if self.enable {
		enable = 1
	}
	web_request.Send(map[string]interface{}{
		"enable": enable,
	})
	return nil, nil
}

func Load_config_timelapse(config *ConfigWrapper) interface{} {
	return NewTimelapse(config)
}
