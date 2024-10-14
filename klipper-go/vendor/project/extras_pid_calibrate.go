package project

import (
	"fmt"
	"k3c/common/utils/cast"
	"k3c/common/utils/maths"
	"k3c/common/utils/object"
	"k3c/common/value"
	"log"
	"math"
	"os"
	"sort"
	"strings"
)

type PIDCalibrate struct {
	printer          *Printer
	move_xy_position []float64
	move_z_up        float64
	//fan_speed        float64
}

const cmd_PID_CALIBRATE_help = "Run PID calibration test"

const NeverReachZUp float64 = 99999.0

func NewPIDCalibrate(config *ConfigWrapper) *PIDCalibrate {
	self := new(PIDCalibrate)
	self.printer = config.Get_printer()
	gcode := MustLookupGcode(self.printer)
	gcode.Register_command("PID_CALIBRATE", self.cmd_PID_CALIBRATE, false, cmd_PID_CALIBRATE_help)

	self.move_xy_position = config.Getfloatlist("move_xy_position", nil, ",", 2, true)
	self.move_z_up = config.Getfloat("move_z_up", NeverReachZUp, 0, 0, 0., 0, true)
	//self.fan_speed = config.Getfloat("fan_speed", 0.0, 0, 0, 0., 0, true) * 255
	return self
}

func (self *PIDCalibrate) cmd_PID_CALIBRATE(argv interface{}) error {
	gcmd := argv.(*GCodeCommand)
	heater_name := gcmd.Get("HEATER", "", value.None, nil, nil, nil, nil)
	target := gcmd.Get_float("TARGET", 20, cast.Float64P(10), nil, nil, nil)
	write_file := gcmd.Get_int("WRITE_FILE", 0, nil, nil)
	gcode := MustLookupGcode(self.printer)

	gcode.Run_script_from_command("G28")
	gcode.Run_script_from_command("G90")
	if self.move_xy_position != nil && len(self.move_xy_position) == 2 {
		gcode.Run_script_from_command(fmt.Sprintf("G1 X%f Y%f F6000", self.move_xy_position[0], self.move_xy_position[1]))
	}

	if self.move_z_up != NeverReachZUp {
		gcode.Run_script_from_command(fmt.Sprintf("G1 Z%f", self.move_z_up))
	}

	// if self.fan_speed > 0 && self.fan_speed <= 255 {
	// 	defer gcode.Run_script_from_command("M106 S0")
	// 	gcode.Run_script_from_command(fmt.Sprintf("M106 S%f", self.fan_speed))
	// }

	pheaters := MustLookupHeaters(self.printer)
	heater := pheaters.Lookup_heater(heater_name)
	if heater == nil {
		panic("heater")
	}
	MustLookupToolhead(self.printer).Get_last_move_time()
	calibrate := NewControlAutoTune(heater, target, self.printer)
	old_control := heater.Set_control(calibrate)
	func() {
		defer func() {
			if r := recover(); r != nil {
				heater.Set_control(old_control)
				panic(r)
			}
		}()
		pheaters.Set_temperature(heater, target, true, 0)
	}()

	heater.Set_control(old_control)
	if write_file == 1 {
		calibrate.write_file("/tmp/heattest.txt")
	}
	if calibrate.Check_busy(0., 0., 0.) {
		panic("pid_calibrate interrupted")
	}

	// Log and report results
	Kp, Ki, Kd := calibrate.calc_final_pid()
	log.Printf("Autotune: final: Kp=%f Ki=%f Kd=%f", Kp, Ki, Kd)
	gcmd.Respond_info(fmt.Sprintf(
		"PID parameters: pid_Kp=%.3f pid_Ki=%.3f pid_Kd=%.3f\n"+
			"The SAVE_CONFIG command will update the printer config file\n"+
			"with these parameters and restart the printer.", Kp, Ki, Kd), true)
	// Store results for SAVE_CONFIG
	configfile := MustLookupConfigfile(self.printer)
	configfile.Set(heater_name, "control", "pid")
	configfile.Set(heater_name, "pid_Kp", fmt.Sprintf("%.3f", Kp))
	configfile.Set(heater_name, "pid_Ki", fmt.Sprintf("%.3f", Ki))
	configfile.Set(heater_name, "pid_Kd", fmt.Sprintf("%.3f", Kd))

	return nil
}

const TUNE_PID_DELTA = 5.0

type ControlAutoTune struct {
	heater           *Heater
	heater_max_power float64
	calibrate_temp   float64
	heating          bool
	peak             float64
	peak_time        float64
	peaks            [][2]float64
	last_pwm         float64
	pwm_samples      [][2]float64
	temp_samples     [][2]float64
	printer          *Printer
}

func NewControlAutoTune(heater *Heater, target float64, printer *Printer) *ControlAutoTune {
	self := new(ControlAutoTune)
	self.heater = heater
	self.heater_max_power = heater.Get_max_power()
	self.calibrate_temp = target
	// Heating control
	self.heating = false
	self.peak = 0.
	self.peak_time = 0.
	// Peak recording
	self.peaks = make([][2]float64, 0)
	// Sample recording
	self.last_pwm = 0.
	self.pwm_samples = make([][2]float64, 0)
	self.temp_samples = make([][2]float64, 0)
	self.printer = printer
	return self
}

/**
    # Heater control
    def set_pwm(self, read_time, value):
        if value != self.last_pwm:
            self.pwm_samples.append(
                (read_time + self.heater.get_pwm_delay(), value))
            self.last_pwm = value
        self.heater.set_pwm(read_time, value)
**/

func (self *ControlAutoTune) set_pwm(read_time, value float64) {
	if value != self.last_pwm {
		self.pwm_samples = append(self.pwm_samples, [2]float64{
			read_time + self.heater.Get_pwm_delay(),
			value,
		})
		self.last_pwm = value
	}

	self.heater.Set_pwm(read_time, value)
}

/**

  def temperature_update(self, read_time, temp, target_temp):
      self.temp_samples.append((read_time, temp))
      # Check if the temperature has crossed the target and
      # enable/disable the heater if so.
      if self.heating and temp >= target_temp:
          self.heating = False
          self.check_peaks()
          self.heater.alter_target(self.calibrate_temp - TUNE_PID_DELTA)
      elif not self.heating and temp <= target_temp:
          self.heating = True
          self.check_peaks()
          self.heater.alter_target(self.calibrate_temp)
      # Check if this temperature is a peak and record it if so
      if self.heating:
          self.set_pwm(read_time, self.heater_max_power)
          if temp < self.peak:
              self.peak = temp
              self.peak_time = read_time
      else:
          self.set_pwm(read_time, 0.)
          if temp > self.peak:
              self.peak = temp
              self.peak_time = read_time

*/

func (self *ControlAutoTune) Temperature_update(read_time, temp, target_temp float64) {
	self.temp_samples = append(self.temp_samples, [2]float64{
		read_time,
		temp,
	})
	// Check if the temperature has crossed the target and
	// enable/disable the heater if so.
	if self.heating && temp >= target_temp {
		self.heating = false
		self.check_peaks()
		self.heater.Alter_target(self.calibrate_temp - TUNE_PID_DELTA)
	} else if !self.heating && temp <= target_temp {
		self.heating = true
		self.check_peaks()
		self.heater.Alter_target(self.calibrate_temp)
	}

	// Check if this temperature is a peak and record it if so
	if self.heating {
		self.set_pwm(read_time, self.heater_max_power)
		if temp < self.peak {
			self.peak = temp
			self.peak_time = read_time
		}
	} else {
		self.set_pwm(read_time, 0.)
		if temp > self.peak {
			self.peak = temp
			self.peak_time = read_time
		}
	}
}

/**

  def check_busy(self, eventtime, smoothed_temp, target_temp):
      if self.heating or len(self.peaks) < 12:
          return True
      return False
*/

func (self *ControlAutoTune) Check_busy(eventtime, smoothed_temp, target_temp float64) bool {
	extruder_heater_check := self.printer.Lookup_object("verify_heater extruder", object.Sentinel{}).(*HeaterCheck)
	heater_bed_heater_check := self.printer.Lookup_object("verify_heater heater_bed", object.Sentinel{}).(*HeaterCheck)
	if (self.heating || len(self.peaks) < 12) && !(heater_bed_heater_check.err || extruder_heater_check.err) {
		return true
	}
	return false
}

// Analysis
/**

  def check_peaks(self):
      self.peaks.append((self.peak, self.peak_time))
      if self.heating:
          self.peak = 9999999.
      else:
          self.peak = -9999999.
      if len(self.peaks) < 4:
          return
      self.calc_pid(len(self.peaks)-1)
*/
func (self *ControlAutoTune) check_peaks() {
	self.peaks = append(self.peaks, [2]float64{self.peak, self.peak_time})
	if self.heating {
		self.peak = 9999999.
	} else {
		self.peak = -9999999.
	}
	if len(self.peaks) < 4 {
		return
	}
	self.calc_pid(len(self.peaks) - 1)
}

/**

  def calc_pid(self, pos):
      temp_diff = self.peaks[pos][0] - self.peaks[pos-1][0]
      time_diff = self.peaks[pos][1] - self.peaks[pos-2][1]
      # Use Astrom-Hagglund method to estimate Ku and Tu
      amplitude = .5 * abs(temp_diff)
      Ku = 4. * self.heater_max_power / (math.pi * amplitude)
      Tu = time_diff
      # Use Ziegler-Nichols method to generate PID parameters
      Ti = 0.5 * Tu
      Td = 0.125 * Tu
      Kp = 0.6 * Ku * heaters.PID_PARAM_BASE
      Ki = Kp / Ti
      Kd = Kp * Td
      logging.info("Autotune: raw=%f/%f Ku=%f Tu=%f  Kp=%f Ki=%f Kd=%f",
                   temp_diff, self.heater_max_power, Ku, Tu, Kp, Ki, Kd)
      return Kp, Ki, Kd
*/

func (self *ControlAutoTune) calc_pid(pos int) (float64, float64, float64) {
	temp_diff := self.peaks[pos][0] - self.peaks[pos-1][0]
	time_diff := self.peaks[pos][1] - self.peaks[pos-2][1]
	// Use Astrom-Hagglund method to estimate Ku and Tu
	amplitude := .5 * math.Abs(temp_diff)
	Ku := 4. * self.heater_max_power / (math.Pi * amplitude)
	Tu := time_diff
	// Use Ziegler-Nichols method to generate PID parameters
	Ti := 0.5 * Tu
	Td := 0.125 * Tu
	Kp := 0.6 * Ku * PID_PARAM_BASE
	//Ti := 0.0
	//Td := 0.0
	//Kp := 0.0

	//if strings.Contains(self.heater.Name, "bed") {
	//	Ti = 0.25 * Tu
	//	Td = 0.066 * Tu
	//	Kp = 0.62 * Ku * PID_PARAM_BASE
	//} else {
	//	// DP PID 喷嘴
	//	Ti = 0.95 * Tu
	//	Td = 0.115 * Tu
	//	Kp = 0.5 * Ku * PID_PARAM_BASE
	//}
	Ki := Kp / Ti
	Kd := Kp * Td
	log.Printf("Autotune: raw=%f/%f Ku=%f Tu=%f  Kp=%f Ki=%f Kd=%f",
		temp_diff, self.heater_max_power, Ku, Tu, Kp, Ki, Kd)
	return Kp, Ki, Kd
}

/**

  def calc_final_pid(self):
      cycle_times = [(self.peaks[pos][1] - self.peaks[pos-2][1], pos)
                     for pos in range(4, len(self.peaks))]
      midpoint_pos = sorted(cycle_times)[len(cycle_times)//2][1]
      return self.calc_pid(midpoint_pos)

*/

func (self *ControlAutoTune) calc_final_pid() (float64, float64, float64) {
	cycle_times := make([][2]float64, 0)
	for pos := 4; pos < len(self.peaks); pos++ {
		cycle_times = append(cycle_times, [2]float64{self.peaks[pos][1] - self.peaks[pos-2][1], float64(pos)})
	}

	// python sorted函数排序元组时候：
	// 它将会先按元组的第一个元素进行排序再按第二个元素进行排序，再按第三个、第四个…依次排序。
	sort.SliceStable(cycle_times, func(i, j int) bool {
		if cycle_times[i][0] < cycle_times[j][0] {
			return true
		} else if cycle_times[i][0] < cycle_times[j][0] {
			return false
		} else {
			return cycle_times[i][1] < cycle_times[j][1]
		}
	})
	midpoint_pos := cycle_times[maths.FloorDiv(len(cycle_times), 2)][1]
	return self.calc_pid(int(midpoint_pos))
}

/**

  def write_file(self, filename):
      pwm = ["pwm: %.3f %.3f" % (time, value)
             for time, value in self.pwm_samples]
      out = ["%.3f %.3f" % (time, temp) for time, temp in self.temp_samples]
      f = open(filename, "w")
      f.write('\n'.join(pwm + out))
      f.close()
*/

func (self *ControlAutoTune) write_file(filename string) {
	pwm := make([]string, 0, len(self.pwm_samples))
	for _, samples := range self.pwm_samples {
		pwm = append(pwm, fmt.Sprintf("pwm: %.3f %.3f", samples[0], samples[1]))
	}

	out := make([]string, 0, len(self.temp_samples))
	for _, samples := range self.temp_samples {
		out = append(out, fmt.Sprintf("%.3f %.3f", samples[0], samples[1]))
	}

	all := strings.Join(pwm, "\n") + strings.Join(out, "\n")
	os.WriteFile(filename, []byte(all), 0666)
}

func Load_config_PIDCalibrate(config *ConfigWrapper) interface{} {
	return NewPIDCalibrate(config)
}
