/*
// Code for coordinating events on the printer Toolhead
//
// Copyright (C) 2016-2021  Kevin O"Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
*/
package project

import "C"
import (
	"errors"
	"fmt"
	"k3c/common/constants"
	"k3c/common/utils/cast"
	"k3c/common/utils/object"
	"k3c/common/value"
	"k3c/project/chelper"
	"math"
	"strings"
)

/*
// Common suffixes: _d is distance (in mm), _v is velocity (in
//   mm/second), _v2 is velocity squared (mm^2/s^2), _t is time (in
//   seconds), _r is ratio (scalar between 0.0 and 1.0)
*/

type Move struct {
	Toolhead           *Toolhead
	Start_pos          []float64
	End_pos            []float64
	Accel              float64
	Junction_deviation float64
	Timing_callbacks   []func(float64)
	Is_kinematic_move  bool
	Axes_d             []float64
	Move_d             float64
	Axes_r             []float64
	Min_move_t         float64
	Max_start_v2       float64
	Max_cruise_v2      float64
	Delta_v2           float64
	Max_smoothed_v2    float64
	Smooth_delta_v2    float64
	Start_v            float64
	Cruise_v           float64
	End_v              float64
	Accel_t            float64
	Cruise_t           float64
	Decel_t            float64
}

func NewMove(toolhead *Toolhead, start_pos, end_pos []float64, speed float64) *Move {
	self := &Move{}
	self.Toolhead = toolhead
	self.Start_pos = append([]float64{}, start_pos...)
	self.End_pos = append([]float64{}, end_pos...)
	if speed >= 600 && toolhead.Max_accel > 15000 {
		self.Accel = 15000
	} else {
		self.Accel = toolhead.Max_accel
	}
	self.Junction_deviation = toolhead.Junction_deviation
	self.Timing_callbacks = []func(float64){}
	for _, vl := range self.Toolhead.VelocityRangeLimit {
		oldspeed := speed
		if speed > vl[0] && speed < vl[1] {
			if speed-vl[0] < vl[1]-speed {
				speed = vl[0]
			} else {
				speed = vl[1]
			}
			if self.Toolhead.VelocityRangeLimitHitLog {
				value.StaticValue.Debug.Printf("NewMove hit velocity range limit oldspeed: %f, newspeed: %f, range: %f - %f", oldspeed, speed, vl[0], vl[1])
			}
			break
		}
	}
	velocity := math.Min(speed, toolhead.Max_velocity)

	self.Is_kinematic_move = true
	axes_d := []float64{
		end_pos[0] - start_pos[0],
		end_pos[1] - start_pos[1],
		end_pos[2] - start_pos[2],
		end_pos[3] - start_pos[3],
	}
	self.Axes_d = axes_d
	Move_d := math.Sqrt(axes_d[0]*axes_d[0] + axes_d[1]*axes_d[1] + axes_d[2]*axes_d[2])

	self.Move_d = Move_d
	inv_move_d := 0.
	if Move_d < 0.000000001 {
		// Extrude-only move
		self.End_pos = []float64{start_pos[0], start_pos[1], start_pos[2], end_pos[3]}
		axes_d[0], axes_d[1], axes_d[2] = 0., 0., 0.
		move_d := math.Abs(axes_d[3])
		self.Move_d = move_d
		inv_move_d = 0.
		if move_d > 0. {
			inv_move_d = 1. / move_d
		}
		self.Accel = 99999999.9
		velocity = speed
		self.Is_kinematic_move = false
	} else {
		inv_move_d = 1. / self.Move_d
	}
	self.Axes_r = []float64{
		axes_d[0] * inv_move_d,
		axes_d[1] * inv_move_d,
		axes_d[2] * inv_move_d,
		axes_d[3] * inv_move_d,
	}
	self.Min_move_t = self.Move_d / velocity

	// Junction speeds are tracked in velocity squared.  The deltaV2 is
	// the maximum amount of this squared-velocity that can change in
	// this move.
	self.Max_start_v2 = 0.
	self.Max_cruise_v2 = velocity * velocity
	self.Delta_v2 = 2. * self.Move_d * self.Accel
	self.Max_smoothed_v2 = 0.
	self.Smooth_delta_v2 = 2. * self.Move_d * toolhead.Max_accel_to_decel
	//log.Printf("self.Move_d %.6f,self.Smooth_delta_v2 %.6f,self.Delta_v2 %.6f,self.Accel %.6f,toolhead.Max_accel_to_decel %.6f self.Min_move_t %.6f velocity %.6f",
	//	self.Move_d, self.Smooth_delta_v2, self.Delta_v2, self.Accel, toolhead.Max_accel_to_decel,self.Min_move_t,velocity)
	return self
}
func (self *Move) Limit_speed(speed float64, accel float64) {
	speed2 := speed * speed
	if speed2 < self.Max_cruise_v2 {
		self.Max_cruise_v2 = speed2
		self.Min_move_t = self.Move_d / speed
	}
	self.Accel = math.Min(self.Accel, accel)
	self.Delta_v2 = 2.0 * self.Move_d * self.Accel
	self.Smooth_delta_v2 = math.Min(self.Smooth_delta_v2, self.Delta_v2)
	//log.Printf("self.Move_d %.6f,self.Smooth_delta_v2 %.6f,self.Delta_v2 %.6f,self.Accel %.6f ,self.Max_cruise_v2 %.6f,self.Min_move_t %.6f",
	//	self.Move_d, self.Smooth_delta_v2, self.Delta_v2, self.Accel,self.Max_cruise_v2,self.Min_move_t)
}

func (self *Move) Move_error(msg string) error {
	ep := self.End_pos
	m := fmt.Sprintf("%s: %.3f %.3f %.3f [%.3f]", msg, ep[0], ep[1], ep[2], ep[3])
	return errors.New(m)
}

func (self *Move) Calc_junction(prev_move *Move) {
	if !self.Is_kinematic_move || !prev_move.Is_kinematic_move {
		return
	}
	// Allow extruder to calculate its maximum junction
	extruder_v2 := self.Toolhead.Extruder.(*PrinterExtruder).Calc_junction(prev_move, self)
	// Find max velocity using "approximated centripetal velocity"
	axes_r := self.Axes_r
	prev_axes_r := prev_move.Axes_r
	junction_cos_theta := -(axes_r[0]*prev_axes_r[0] + axes_r[1]*prev_axes_r[1] + axes_r[2]*prev_axes_r[2])
	if junction_cos_theta > 0.999999 {
		return
	}
	junction_cos_theta = math.Max(junction_cos_theta, -0.999999)
	sin_theta_d2 := math.Sqrt(0.5 * (1.0 - junction_cos_theta))
	R_jd := sin_theta_d2 / (1. - sin_theta_d2)
	// Approximated circle must contact moves no further away than mid-move
	tan_theta_d2 := sin_theta_d2 / math.Sqrt(0.5*(1.0+junction_cos_theta))
	move_centripetal_v2 := 0.5 * self.Move_d * tan_theta_d2 * self.Accel
	prev_move_centripetal_v2 := 0.5 * prev_move.Move_d * tan_theta_d2 * prev_move.Accel
	// Apply limits
	start_v2_arr := []float64{R_jd * self.Junction_deviation * self.Accel,
		R_jd * prev_move.Junction_deviation * prev_move.Accel,
		move_centripetal_v2, prev_move_centripetal_v2,
		extruder_v2, self.Max_cruise_v2, prev_move.Max_cruise_v2,
		prev_move.Max_smoothed_v2 + prev_move.Delta_v2}
	self.Max_start_v2 = R_jd * self.Junction_deviation * self.Accel
	for _, val := range start_v2_arr {
		if val < self.Max_start_v2 {
			self.Max_start_v2 = val
		}
	}
	self.Max_smoothed_v2 = math.Min(self.Max_start_v2, prev_move.Max_smoothed_v2+prev_move.Smooth_delta_v2)
}
func (self *Move) Set_junction(start_v2, cruise_v2, end_v2 float64) {
	// Determine accel, cruise, and decel portions of the move distance
	half_inv_accel := 0.5 / self.Accel
	accel_d := (cruise_v2 - start_v2) * half_inv_accel
	decel_d := (cruise_v2 - end_v2) * half_inv_accel
	cruise_d := self.Move_d - accel_d - decel_d
	// Determine move velocities
	start_v := math.Sqrt(start_v2)
	self.Start_v = start_v
	cruise_v := math.Sqrt(cruise_v2)
	self.Cruise_v = cruise_v
	end_v := math.Sqrt(end_v2)
	self.End_v = end_v
	// Determine time spent in each portion of move (time is the
	// distance divided by average velocity)
	self.Accel_t = accel_d / ((start_v + cruise_v) * 0.5)
	self.Cruise_t = cruise_d / cruise_v
	self.Decel_t = decel_d / ((end_v + cruise_v) * 0.5)
	//log.Printf("start_v2 %f, cruise_v2 %f, end_v2 %f "+
	//	"half_inv_accel %f decel_d %f cruise_d %f accel_d %f self.Accel %f self.Cruise_t %f self.Decel_t %f Move_d %f self %v", start_v2, cruise_v2, end_v2,
	//	half_inv_accel, decel_d, cruise_d, accel_d, self.Accel,self.Cruise_t,self.Decel_t,self.Move_d, unsafe.Pointer(self))
}

const LOOKAHEAD_FLUSH_TIME = 0.250

/*
// Class to track a list of pending move requests and to facilitate
// "look-ahead" across moves to reduce acceleration between moveself.
*/
type MoveQueue struct {
	Toolhead       *Toolhead
	queue          []*Move
	Junction_flush float64
}

func NewMoveQueue(Toolhead *Toolhead) *MoveQueue {
	return &MoveQueue{
		Toolhead:       Toolhead,
		queue:          []*Move{},
		Junction_flush: LOOKAHEAD_FLUSH_TIME,
	}
}

func (self *MoveQueue) Reset() {
	self.queue = []*Move{}
	self.Junction_flush = LOOKAHEAD_FLUSH_TIME
}

func (self *MoveQueue) Set_flush_time(flush_time float64) {
	self.Junction_flush = flush_time
}

func (self *MoveQueue) Get_last() *Move {
	if len(self.queue) > 0 {
		return self.queue[len(self.queue)-1]
	}

	return nil
}

type delayed_node struct {
	Move  *Move
	Ms_v2 float64
	Me_v2 float64
}

// lazy=False
func (self *MoveQueue) Flush(lazy bool) {
	self.Junction_flush = LOOKAHEAD_FLUSH_TIME
	update_flush_count := lazy
	queue := self.queue
	flush_count := len(queue)
	// Traverse queue from last to first move and determine maximum
	// junction speed assuming the robot comes to a complete stop
	// after the last move.
	delayed := []delayed_node{}
	next_end_v2, next_smoothed_v2, peak_cruise_v2 := 0., 0., 0.
	for i := flush_count - 1; i >= 0; i-- {
		move := queue[i]
		reachable_start_v2 := next_end_v2 + move.Delta_v2
		start_v2 := math.Min(move.Max_start_v2, reachable_start_v2)
		reachable_smoothed_v2 := next_smoothed_v2 + move.Smooth_delta_v2
		smoothed_v2 := math.Min(move.Max_smoothed_v2, reachable_smoothed_v2)
		//log.Printf("reachable_smoothed_v2 %.6f,smoothed_v2 %.6f,move.Max_smoothed_v2 %.6f,next_smoothed_v2 %.6f,move.Smooth_delta_v2 %.6f Delta_v2 %.6f Move_d %.6f", reachable_smoothed_v2, smoothed_v2, move.Max_smoothed_v2, next_smoothed_v2, move.Smooth_delta_v2,move.Delta_v2,move.Move_d)
		if smoothed_v2 < reachable_smoothed_v2 {
			// It"s possible for this move to accelerate
			if smoothed_v2+move.Smooth_delta_v2 > next_smoothed_v2 || len(delayed) > 0 {
				// This move can decelerate or this is a full accel
				// move after a full decel move
				if update_flush_count && peak_cruise_v2 != 0 {
					flush_count = i
					update_flush_count = false
				}
				// this
				peak_cruise_v2 = math.Min(move.Max_cruise_v2, (smoothed_v2+reachable_smoothed_v2)*.5)
				if len(delayed) > 0 {
					// Propagate peak_cruise_v2 to any delayed moves
					if !update_flush_count && i < flush_count {
						mc_v2 := peak_cruise_v2
						for j := len(delayed) - 1; j >= 0; j-- {
							m := delayed[j].Move
							ms_v2 := delayed[j].Ms_v2
							me_v2 := delayed[j].Me_v2
							mc_v2 = math.Min(mc_v2, ms_v2)
							m.Set_junction(math.Min(ms_v2, mc_v2), mc_v2, math.Min(me_v2, mc_v2))
						}
					}
					delayed = nil
				}
			}
			if !update_flush_count && i < flush_count {
				//log.Print(unsafe.Pointer(move),i,queue)
				cruise_v2 := math.Min((start_v2+reachable_start_v2)*.5,
					math.Min(move.Max_cruise_v2, peak_cruise_v2))
				move.Set_junction(math.Min(start_v2, cruise_v2), cruise_v2,
					math.Min(next_end_v2, cruise_v2))
			}
		} else {
			// Delay calculating this move until peak_cruise_v2 is known
			delayed = append(delayed, delayed_node{move, start_v2, next_end_v2})
		}
		next_end_v2 = start_v2
		next_smoothed_v2 = smoothed_v2
	}
	if update_flush_count || flush_count <= 0 {
		return
	}
	// Generate step times for all moves ready to be flushed
	self.Toolhead.Process_moves(queue[:flush_count])
	// Remove processed moves from the queue
	if flush_count < len(self.queue) && len(self.queue) > 0 {
		self.queue = self.queue[flush_count:]
	} else if len(self.queue) > 0 {
		self.queue = []*Move{}
	}
	//log.Printf("%v", self.queue)

}
func (self *MoveQueue) Add_move(move *Move) {
	self.queue = append(self.queue, move)
	if len(self.queue) == 1 {
		return
	}
	move.Calc_junction(self.queue[len(self.queue)-2])
	self.Junction_flush -= move.Min_move_t
	if self.Junction_flush <= 0. {
		// Enough moves have been queued to reach the target flush time.
		self.Flush(true)
	}
}

const (
	MIN_KIN_TIME    = 0.100
	MOVE_BATCH_TIME = 0.500
	SDS_CHECK_TIME  = 0.001 // step+dir+step filter in stepcompresself.c

	DRIP_SEGMENT_TIME = 0.050
	DRIP_TIME         = 0.100
)

type DripModeEndSignal struct {
}

func (d *DripModeEndSignal) Error() string {
	panic("implement me")
}

// Main code to track events (and their timing) on the printer Toolhead
type Toolhead struct {
	Printer                  *Printer
	Reactor                  IReactor
	All_mcus                 []*MCU
	Mcu                      *MCU
	Can_pause                bool
	Move_queue               *MoveQueue
	Commanded_pos            []float64
	Max_velocity             float64
	Max_accel                float64
	Limit_Max_accel          float64
	Requested_accel_to_decel float64
	Max_accel_to_decel       float64
	Square_corner_velocity   float64
	Junction_deviation       float64
	Buffer_time_low          float64
	Buffer_time_high         float64
	Buffer_time_start        float64
	Move_flush_time          float64
	Print_time               float64
	Special_queuing_state    string
	Need_check_stall         float64
	Flush_timer              *ReactorTimer
	Idle_flush_print_time    float64
	Print_stall              float64
	Drip_completion          *ReactorCompletion
	Kin_flush_delay          float64
	Kin_flush_times          []float64
	force_flush_time         float64
	Last_kin_move_time       float64
	Trapq                    interface{}
	Trapq_append             func(tq interface{}, print_time,
		accel_t, cruise_t, decel_t,
		start_pos_x, start_pos_y, start_pos_z,
		axes_r_x, axes_r_y, axes_r_z,
		start_v, cruise_v, accel float64)
	Trapq_finalize_moves     func(interface{}, float64)
	Step_generators          []func(float64 float64)
	Coord                    []string
	Extruder                 IExtruder
	Kin                      IKinematics
	VelocityRangeLimit       [][2]float64
	VelocityRangeLimitHitLog bool
	move_transform           Itransform
}

func NewToolhead(config *ConfigWrapper) *Toolhead {
	self := &Toolhead{}
	self.Printer = config.Get_printer()
	self.Reactor = self.Printer.Get_reactor()
	object_arr := self.Printer.Lookup_objects("mcu")
	self.All_mcus = []*MCU{}
	for _, m := range object_arr {
		for k1, m1 := range m.(map[string]interface{}) {
			if strings.HasPrefix(k1, "mcu") {
				self.All_mcus = append(self.All_mcus, m1.(*MCU))
			}
		}
	}
	self.Mcu = self.All_mcus[0]
	self.Can_pause = true
	if self.Mcu.Is_fileoutput() {
		self.Can_pause = false
	}
	self.Move_queue = NewMoveQueue(self)
	self.Commanded_pos = []float64{0., 0., 0., 0.}
	self.Printer.Register_event_handler("project:shutdown",
		self.Handle_shutdown)
	// Velocity and acceleration control

	self.Max_velocity = config.Getfloat("max_velocity", object.Sentinel{}, 0.0, 0.0, 0.0, 0.0, true)
	self.Max_accel = config.Getfloat("max_accel", object.Sentinel{}, 0.0, 0.0, 0.0, 0.0, true)
	self.Limit_Max_accel = self.Max_accel
	self.Requested_accel_to_decel = config.Getfloat("max_accel_to_decel", self.Max_accel*0.5, 0.0, 0.0, 0., 0.0, true)
	self.Max_accel_to_decel = self.Requested_accel_to_decel
	self.Square_corner_velocity = config.Getfloat("square_corner_velocity", 5., 0.0, 0.0, 0., 0.0, true)
	self.Junction_deviation = 0.
	self.Calc_junction_deviation()
	// Print time tracking
	self.Buffer_time_low = config.Getfloat("buffer_time_low", 1.000, 0.0, 0.0, 0., 0.0, true)
	self.Buffer_time_high = config.Getfloat(
		"buffer_time_high", 2.000, 0.0, 0.0, self.Buffer_time_low, 0.0, true)
	self.Buffer_time_start = config.Getfloat(
		"buffer_time_start", 0.250, 0.0, 0.0, 0., 0.0, true)
	self.Move_flush_time = config.Getfloat(
		"move_flush_time", 0.050, 0.0, 0.0, 0.0, 0.0, true)

	velocityRangeLimit := cast.ToString(config.Get("velocity_range_limit", "", true))
	velocityRangeLimitBeginMin := config.Getfloat("velocity_range_limit_begin_min", 20.0, 10.0, 0.0, 0., 0.0, true)
	self.VelocityRangeLimitHitLog = cast.ToBool(config.Get("velocity_range_limit_hit_log", "0", true))
	for _, vrl := range strings.Split(velocityRangeLimit, ",") {
		if !strings.Contains(vrl, "_") {
			continue
		}
		segments := strings.Split(strings.TrimSpace(vrl), "_")
		begin, end := cast.ToFloat64(segments[0]), cast.ToFloat64(segments[1])
		if begin >= end {
			value.StaticValue.Debug.Println("toolhead velocity range limit end should greater than begin")
			continue
		}
		if begin < velocityRangeLimitBeginMin {
			value.StaticValue.Debug.Printf("toolhead velocity range limit begin should greater than %f", velocityRangeLimitBeginMin)
			continue
		}
		self.VelocityRangeLimit = append(self.VelocityRangeLimit, [2]float64{
			begin,
			end,
		})
	}

	self.Print_time = 0.
	self.Special_queuing_state = "Flushed"
	self.Need_check_stall = -1.
	self.Flush_timer = self.Reactor.Register_timer(self.Flush_handler, constants.NEVER)
	self.Move_queue.Set_flush_time(self.Buffer_time_high)
	self.Idle_flush_print_time = 0.
	self.Print_stall = 0
	self.Drip_completion = nil
	// Kinematic step generation scan window time tracking
	self.Kin_flush_delay = SDS_CHECK_TIME
	self.Kin_flush_times = []float64{}
	self.force_flush_time, self.Last_kin_move_time = 0., 0.
	// Setup iterative solver
	//ffi_lib := chelper.Get_ffi()
	self.Trapq = chelper.Trapq_alloc()
	//runtime.SetFinalizer(self,self._Toolhead)
	self.Trapq_append = chelper.Trapq_append
	self.Trapq_finalize_moves = chelper.Trapq_finalize_moves
	self.Step_generators = []func(float64 float64){}
	// Create kinematics class
	gcode_obj := self.Printer.Lookup_object("gcode", object.Sentinel{})
	//if err != nil {
	//	value.StaticValue.Error.Println(err)
	//}
	gcode := gcode_obj.(*GCodeDispatch)
	self.Coord = append([]string{}, gcode.Coord...)
	self.Extruder = NewDummyExtruder(self.Printer)
	kin_name := config.Get("kinematics", object.Sentinel{}, true)
	//log.Print(kin_name)
	mod, err := Import_module(strings.Join([]string{"kinematics.", kin_name.(string)}, ""))
	if err != nil {
		panic(err.Error())
	}
	self.Kin = mod.(func(*Toolhead, *ConfigWrapper) interface{})(self, config).(IKinematics)
	// Register commands
	gcode.Register_command("G4", self.Cmd_G4, false, "")
	gcode.Register_command("M400", self.Cmd_M400, false, "")
	gcode.Register_command("SET_VELOCITY_LIMIT",
		self.Cmd_SET_VELOCITY_LIMIT,
		false,
		cmd_SET_VELOCITY_LIMIT_help)
	gcode.Register_command("SAVE_VELOCITY",
		self.Cmd_SAVE_VELOCITY,
		false,
		cmd_SAVE_VELOCITY_help)
	gcode.Register_command("M204", self.Cmd_M204, false, "")
	// Load some default modules 此处排除了 idle_timeout
	modules := []string{"gcode_move", "homing", "statistics",
		"manual_probe", "tuning_tower"}
	for _, module_name := range modules {
		self.Printer.Load_object(config, module_name, object.Sentinel{})
	}
	return self
}
func (self *Toolhead) _Toolhead() {
	chelper.Trapq_free(self.Trapq)
}

// Print time tracking
func (self *Toolhead) Update_move_time(next_print_time float64) {
	batch_time := MOVE_BATCH_TIME
	kin_flush_delay := self.Kin_flush_delay
	fft := self.force_flush_time
	for {
		self.Print_time = math.Min(self.Print_time+batch_time, next_print_time)
		sg_flush_time := math.Max(fft, self.Print_time-kin_flush_delay)
		for _, sg := range self.Step_generators {
			sg(sg_flush_time)
		}
		free_time := math.Max(fft, sg_flush_time-kin_flush_delay)
		self.Trapq_finalize_moves(self.Trapq, free_time)
		if _, ok := self.Extruder.(*PrinterExtruder); ok {
			self.Extruder.(*PrinterExtruder).Update_move_time(free_time)
		} else if _, ok := self.Extruder.(*DummyExtruder); ok {
			self.Extruder.(*DummyExtruder).Update_move_time(free_time)
		}
		mcu_flush_time := math.Max(fft, sg_flush_time-self.Move_flush_time)
		for _, m := range self.All_mcus {
			m.Flush_moves(mcu_flush_time)
		}
		if self.Print_time >= next_print_time {
			break
		}
	}
}
func (self *Toolhead) Calc_print_time() {
	curtime := self.Reactor.Monotonic()
	est_print_time := self.Mcu.Estimated_print_time(curtime)
	kin_time := math.Max(est_print_time+MIN_KIN_TIME, self.force_flush_time)
	kin_time += self.Kin_flush_delay
	min_print_time := math.Max(est_print_time+self.Buffer_time_start, kin_time)
	if min_print_time > self.Print_time {
		self.Print_time = min_print_time
		self.Printer.Send_event("toolhead:sync_print_time", []interface{}{curtime, est_print_time, self.Print_time})
	}
}
func (self *Toolhead) Process_moves(moves []*Move) {
	// Resync print_time if necessary
	if len(self.Special_queuing_state) > 0 {
		if self.Special_queuing_state != "Drip" {
			// Transition from "Flushed"/"Priming" state to main state
			self.Special_queuing_state = ""
			self.Need_check_stall = -1.
			self.Reactor.Update_timer(self.Flush_timer, constants.NOW)
		}
		self.Calc_print_time()
	}

	// Queue moves into trapezoid motion queue (trapq)
	next_move_time := self.Print_time
	//log.Print(moves)
	for _, move := range moves {
		//log.Println("_t as s;_d as mm;_v as mm/s: move_time", move.Accel_t + move.Cruise_t + move.Decel_t, "Move_d",move.Move_d," Accel_t", move.Accel_t, "Cruise_t", move.Cruise_t, " Decel_t", move.Decel_t,  "Start_v", move.Start_v,"Cruise_v", move.Cruise_v,"End_v",move.End_v,"Accel", move.Accel)
		//log.Println(time.Now().UnixMilli(), "move_time", move.Accel_t+move.Cruise_t+move.Decel_t, "Move_d", move.Move_d, " Accel_t", move.Accel_t, "Cruise_t", move.Cruise_t, " Decel_t", move.Decel_t, "Start_v", move.Start_v, "Cruise_v", move.Cruise_v, "End_v", move.End_v, "Accel", move.Accel, "Accel_d", move.Accel*move.Accel_t*move.Accel_t, "Decel_d", move.Accel*move.Decel_t*move.Decel_t)

		if move.Is_kinematic_move {
			self.Trapq_append(self.Trapq, next_move_time, move.Accel_t, move.Cruise_t, move.Decel_t, move.Start_pos[0], move.Start_pos[1], move.Start_pos[2], move.Axes_r[0], move.Axes_r[1], move.Axes_r[2], move.Start_v, move.Cruise_v, move.Accel)
		}
		if move.Axes_d[3] != 0.0 {
			self.Extruder.(*PrinterExtruder).Move(next_move_time, move)
		}
		next_move_time = next_move_time + move.Accel_t + move.Cruise_t + move.Decel_t
		for _, cb := range move.Timing_callbacks {
			cb(next_move_time)
		}
	}

	// Generate steps for moves
	if len(self.Special_queuing_state) > 0 {
		self.Update_drip_move_time(next_move_time)
	}
	self.Update_move_time(next_move_time)
	self.Last_kin_move_time = math.Max(self.Last_kin_move_time, next_move_time)
}
func (self *Toolhead) Flush_step_generation() {
	// Transition from "Flushed"/"Priming"/main state to "Flushed" state
	self.Move_queue.Flush(false)
	self.Special_queuing_state = "Flushed"
	self.Need_check_stall = -1.0
	self.Reactor.Update_timer(self.Flush_timer, constants.NEVER)
	self.Move_queue.Set_flush_time(self.Buffer_time_high)
	self.Idle_flush_print_time = 0.
	// Determine actual last "itersolve" flush time
	lastf := self.Print_time - self.Kin_flush_delay
	// Calculate flush time that includes kinematic scan windows
	flush_time := math.Max(lastf, self.Last_kin_move_time+self.Kin_flush_delay)
	if flush_time > self.Print_time {
		// Flush in small time chunks
		self.Update_move_time(flush_time)
	}
	self.force_flush_time = math.Max(self.force_flush_time, flush_time)
	self.Update_move_time(math.Max(self.Print_time, self.force_flush_time))
}
func (self *Toolhead) Flush_lookahead() {
	if self.Special_queuing_state != "" {
		self.Flush_step_generation()
	}
	self.Move_queue.Flush(false)
}
func (self *Toolhead) Get_last_move_time() float64 {
	self.Flush_lookahead()
	if self.Special_queuing_state != "" {
		self.Calc_print_time()
	}
	return self.Print_time
}
func (self *Toolhead) Check_stall() {
	eventtime := self.Reactor.Monotonic()
	if self.Special_queuing_state != "" {
		if self.Idle_flush_print_time > 0.0 {
			// Was in "Flushed" state and got there from idle input
			est_print_time := self.Mcu.Estimated_print_time(eventtime)
			if est_print_time < self.Idle_flush_print_time {
				self.Print_stall += 1
			}
			self.Idle_flush_print_time = 0.
		}
		// Transition from "Flushed"/"Priming" state to "Priming" state
		self.Special_queuing_state = "Priming"
		self.Need_check_stall = -1.
		self.Reactor.Update_timer(self.Flush_timer, eventtime+0.100)
	}
	// Check if there are lots of queued moves and stall if so
	est_print_time := 0.0
	for {
		est_print_time = self.Mcu.Estimated_print_time(eventtime)
		buffer_time := self.Print_time - est_print_time
		stall_time := buffer_time - self.Buffer_time_high
		if stall_time <= 0. {
			break
		}
		if !self.Can_pause {
			self.Need_check_stall = constants.NEVER
			return
		}
		eventtime = self.Reactor.Pause(eventtime + math.Min(1, stall_time))
	}
	if self.Special_queuing_state == "" {
		// In main state - defer stall checking until needed
		self.Need_check_stall = est_print_time + self.Buffer_time_high + 0.1
	}
}
func (self *Toolhead) Flush_handler(eventtime float64) float64 {
	printTime := self.Print_time
	bufferTime := printTime - self.Mcu.Estimated_print_time(eventtime)
	if bufferTime > self.Buffer_time_low {
		// Running normally - reschedule check
		return eventtime + bufferTime - self.Buffer_time_low
	}
	// Under ran low buffer mark - flush lookahead queue
	self.Flush_step_generation()
	if printTime != self.Print_time {
		self.Idle_flush_print_time = self.Print_time
	}
	defer func() {
		if err := recover(); err != nil {
			value.StaticValue.Error.Println("Exception in flush_handler")
			self.Printer.Invoke_shutdown("Exception in flush_handler")
		}
	}()
	return constants.NEVER
}

// Movement commands
func (self *Toolhead) Get_position() []float64 {
	commanded_pos_back := make([]float64, len(self.Commanded_pos))
	copy(commanded_pos_back, self.Commanded_pos)
	return commanded_pos_back
}
func (self *Toolhead) Set_position(newpos []float64, homingAxes []int) {
	self.Flush_step_generation()
	//ffiLib := chelper.Get_ffi()
	chelper.Trapq_set_position(self.Trapq, self.Print_time,
		newpos[0], newpos[1], newpos[2])
	self.Commanded_pos = append([]float64{}, newpos...)
	//Commanded_pos := make([]float64,len(newpos))
	//copy(Commanded_pos,newpos)
	//self.Commanded_pos=Commanded_pos
	//log.Print("Set_position", self.Commanded_pos)
	self.Kin.Set_position(newpos, homingAxes)
	self.Printer.Send_event("toolhead:set_position", nil)
}

// var A int = 0
func (self *Toolhead) Move(newpos []float64, speed float64) {
	move := NewMove(self, self.Commanded_pos, newpos, speed)
	//log.Printf("self.Commanded_pos:%v newpos:%v", self.Commanded_pos, newpos)
	/* 	if self.Commanded_pos[0] == 315 {
		//log.Printf("v:%v ", move.Move_d)
	} */
	if move.Move_d == 0.0 {
		return
	}

	if move.Is_kinematic_move {
		self.Kin.Check_move(move)
	}
	if move.Axes_d[3] != 0.0 {
		self.Extruder.(*PrinterExtruder).Check_move(move)
	}
	self.Commanded_pos = move.End_pos
	//Commanded_pos := make([]float64,len(newpos))
	//copy(Commanded_pos,newpos)
	//self.Commanded_pos=Commanded_pos
	//if(self.Commanded_pos[0]==120){
	//	A=A+1
	//	if(A==2){
	//		log.Print("")
	//	}
	//
	//}
	//log.Print("move", self.Commanded_pos)
	self.Move_queue.Add_move(move)
	if self.Print_time > self.Need_check_stall {
		self.Check_stall()
	}
}

func (self *Toolhead) Manual_move(coord []interface{}, speed float64) {
	length := int(math.Max(float64(len(self.Commanded_pos)), float64(len(coord))))
	curpos := make([]float64, length)
	copy(curpos, self.Commanded_pos)
	for i := 0; i < len(coord); i++ {
		if coord[i] != nil {
			curpos[i] = coord[i].(float64)
		}
	}
	self.Move(curpos, speed)
	self.Printer.Send_event("toolhead:manual_move", nil)
}

func (self *Toolhead) Dwell(delay float64) {
	next_print_time := self.Get_last_move_time() + math.Max(0., delay)
	self.Update_move_time(next_print_time)
	self.Check_stall()
}

func (self *Toolhead) Wait_moves() {
	self.Flush_lookahead()
	eventtime := self.Reactor.Monotonic()
	for self.Special_queuing_state == "" || self.Print_time >= self.Mcu.Estimated_print_time(eventtime) {
		if !self.Can_pause {
			break
		}
		eventtime = self.Reactor.Pause(eventtime + 0.100)
	}
}

func (self *Toolhead) Set_extruder(extruder IExtruder, extrude_pos float64) {
	self.Extruder = extruder
	self.Commanded_pos[3] = extrude_pos
}

func (self *Toolhead) Get_extruder() IExtruder {
	return self.Extruder
}

// Homing "drip move" handling
func (self *Toolhead) Update_drip_move_time(next_print_time float64) error {
	flush_delay := DRIP_TIME + self.Move_flush_time + self.Kin_flush_delay
	for self.Print_time < next_print_time {
		if self.Drip_completion.Test() {
			panic(&DripModeEndSignal{})
		}
		curTime := self.Reactor.Monotonic()
		est_printTime := self.Mcu.Estimated_print_time(curTime)
		wait_time := self.Print_time - est_printTime - flush_delay
		if wait_time > 0. && self.Can_pause {
			// Pause before sending more steps
			self.Drip_completion.Wait(curTime+wait_time, nil)
			continue
		}
		npt := math.Min(self.Print_time+DRIP_SEGMENT_TIME, next_print_time)
		self.Update_move_time(npt)
	}
	return nil
}
func (self *Toolhead) Drip_move(newpos []float64, speed float64, drip_completion *ReactorCompletion) error {
	self.Dwell(self.Kin_flush_delay)
	// Transition from "Flushed"/"Priming"/main state to "Drip" state、
	self.Move_queue.Flush(false)
	self.Special_queuing_state = "Drip"
	self.Need_check_stall = constants.NEVER
	self.Reactor.Update_timer(self.Flush_timer, constants.NEVER)
	self.Move_queue.Set_flush_time(self.Buffer_time_high)
	self.Idle_flush_print_time = 0.
	self.Drip_completion = drip_completion
	// Submit move
	self.tryCatchDrip_moveMove(newpos, speed)
	// Transmit move in "drip" mode
	self.tryCatchDrip_moveFlush(false)
	// Exit "Drip" state
	self.Flush_step_generation()
	return nil
}
func (self *Toolhead) tryCatchDrip_moveMove(newpos []float64, speed float64) {
	defer func() {
		if err := recover(); err != nil {
			_, ok := err.(*CommandError)
			if ok {
				self.Flush_step_generation()
			}
			panic(err)
		}
	}()
	self.Move(newpos, speed)
}
func (self *Toolhead) tryCatchDrip_moveFlush(lazy bool) {
	defer func() {
		if err := recover(); err != nil {
			_, ok := err.(*DripModeEndSignal)
			if ok {
				self.Move_queue.Reset()
				self.Trapq_finalize_moves(self.Trapq, constants.NEVER)
				return
			}
			panic(err)
		}
	}()
	self.Move_queue.Flush(lazy)
}

// Misc commands
func (self *Toolhead) Stats(eventtime float64) (bool, string) {
	for _, m := range self.All_mcus {
		m.Check_active(self.Print_time, eventtime)
	}
	buffer_time := self.Print_time - self.Mcu.Estimated_print_time(eventtime)
	is_active := buffer_time > -60.0 || self.Special_queuing_state == ""
	if self.Special_queuing_state == "Drip" {
		buffer_time = 0.
	}
	return is_active, fmt.Sprintf("print_time=%.3f buffer_time=%.3f print_stall=%.f",
		self.Print_time, math.Max(buffer_time, 0.), self.Print_stall)
}

func (self *Toolhead) Check_busy(eventtime float64) (float64, float64, bool) {
	est_print_time := self.Mcu.Estimated_print_time(eventtime)
	lookahead_empty := len(self.Move_queue.queue) == 0
	return self.Print_time, est_print_time, lookahead_empty
}
func (self *Toolhead) Get_status(eventtime float64) map[string]interface{} {
	//print_time := self.Print_time
	//estimated_print_time := self.Mcu.Estimated_print_time(eventtime)

	res := self.Kin.Get_status(eventtime)
	//res["print_time"] = print_time
	res["stalls"] = self.Print_stall
	//res["estimated_print_time"] = estimated_print_time
	res["extruder"] = self.Extruder.(*PrinterExtruder).Get_name()

	coord := make([]float64, len(self.Commanded_pos))
	copy(coord, self.Commanded_pos)
	res["position"] = coord
	res["max_velocity"] = self.Max_velocity
	res["max_accel"] = self.Max_accel
	res["max_accel_to_decel"] = self.Requested_accel_to_decel
	res["square_corner_velocity"] = self.Square_corner_velocity
	return res
}
func (self *Toolhead) Handle_shutdown([]interface{}) error {
	self.Can_pause = false
	self.Move_queue.Reset()
	return nil
}
func (self *Toolhead) Get_kinematics() interface{} {
	return self.Kin
}
func (self *Toolhead) Get_trapq() interface{} {
	return self.Trapq
}
func (self *Toolhead) Register_step_generator(handler func(float64)) {
	self.Step_generators = append(self.Step_generators, handler)
}
func (self *Toolhead) Note_step_generation_scan_time(delay, old_delay float64) {
	self.Flush_step_generation()
	// cur_delay := self.Kin_flush_delay
	if old_delay != 0.0 {
		index := 0
		for i, v := range self.Kin_flush_times {
			if v == old_delay {
				index = i
				break
			}
		}
		self.Kin_flush_times = append(self.Kin_flush_times[:index], self.Kin_flush_times[index+1:]...)
	}
	if delay != 0.0 {
		self.Kin_flush_times = append(self.Kin_flush_times, delay)
	}
	new_delay := 0.
	for _, val := range self.Kin_flush_times {
		val += SDS_CHECK_TIME
		if val > new_delay {
			new_delay = val
		}
	}
	self.Kin_flush_delay = new_delay
}
func (self *Toolhead) Register_lookahead_callback(callback func(float64)) {
	last_move := self.Move_queue.Get_last()
	if last_move == nil {
		callback(self.Get_last_move_time())
		return
	}
	last_move.Timing_callbacks = append(last_move.Timing_callbacks, callback)
}
func (self *Toolhead) Note_kinematic_activity(kin_time float64) {
	self.Last_kin_move_time = math.Max(self.Last_kin_move_time, kin_time)
}
func (self *Toolhead) Get_max_velocity() (float64, float64) {
	return self.Max_velocity, self.Max_accel
}
func (self *Toolhead) Calc_junction_deviation() {
	scv2 := self.Square_corner_velocity * self.Square_corner_velocity
	self.Junction_deviation = scv2 * (math.Sqrt(2.) - 1.) / self.Max_accel
	self.Max_accel_to_decel = math.Min(self.Requested_accel_to_decel,
		self.Max_accel)
}
func (self *Toolhead) Cmd_G4(arg interface{}) error {
	gcmd := arg.(*GCodeCommand)
	// Dwell
	minval := 0.
	delay := 0.0

	//毫秒级别的延时
	delay = gcmd.Get_float("P", 0., &minval, nil, nil, nil) / 1000.

	//秒级别的延时
	if delay == 0.0 {
		delay = gcmd.Get_float("S", 0., &minval, nil, nil, nil)
	}

	self.Dwell(delay)
	return nil
}
func (self *Toolhead) Cmd_M400(gcmd interface{}) error {
	// Wait for current moves to finish
	self.Wait_moves()
	return nil
}

const cmd_SET_VELOCITY_LIMIT_help = "Set printer velocity limits"

func (self *Toolhead) Cmd_SET_VELOCITY_LIMIT(arg interface{}) error {
	gcmd := arg.(*GCodeCommand)
	above := 0.0
	minval := 0.
	max_velocity := gcmd.Get_float("VELOCITY", nil, nil, nil, &above, nil)
	max_accel := gcmd.Get_float("ACCEL", nil, nil, nil, &above, nil)
	square_corner_velocity := gcmd.Get_float("SQUARE_CORNER_VELOCITY", nil, &minval, nil, nil, nil)
	requested_accel_to_decel := gcmd.Get_float("ACCEL_TO_DECEL", nil, nil, nil, &above, nil)

	if max_velocity != 0.0 {
		if self.Max_velocity != max_velocity {
			self.Max_velocity = max_velocity
		}

	}
	if max_accel != 0.0 {
		if self.Max_accel != max_accel {
			self.Max_accel = max_accel
		}
	}
	if square_corner_velocity != 0.0 {
		if self.Square_corner_velocity != square_corner_velocity {
			self.Square_corner_velocity = square_corner_velocity
		}

	}
	if requested_accel_to_decel != 0.0 {
		if self.Requested_accel_to_decel != requested_accel_to_decel {
			self.Requested_accel_to_decel = requested_accel_to_decel
		}
	}
	self.Calc_junction_deviation()
	msg := fmt.Sprintf("max_velocity: %.6f\n"+
		"max_accel: %.6f\n"+
		"max_accel_to_decel: %.6f\n"+
		"square_corner_velocity: %.6f",
		self.Max_velocity, self.Max_accel, self.Requested_accel_to_decel, self.Square_corner_velocity)
	self.Printer.Set_rollover_info("toolhead", fmt.Sprintf("toolhead: %s", msg), false)
	if max_velocity == 0.0 && max_accel == 0.0 && square_corner_velocity == 0.0 && requested_accel_to_decel == 0.0 {
		gcmd.Respond_info(msg, false)
	}
	return nil
}

const cmd_SAVE_VELOCITY_help = "save printer velocity"

func (self *Toolhead) Cmd_SAVE_VELOCITY(arg interface{}) error {
	gcmd := arg.(*GCodeCommand)
	name := gcmd.Get("NAME", "VELOCITY", value.None, nil, nil, nil, nil)
	var configfile = self.Printer.Lookup_object("configfile", object.Sentinel{})
	switch strings.ToUpper(name) {
	case "VELOCITY":
		configfile.(*PrinterConfig).Set("printer", "max_velocity", fmt.Sprintf("%.2f", self.Max_velocity))
	case "ACCEL":
		configfile.(*PrinterConfig).Set("printer", "max_accel", fmt.Sprintf("%.2f", self.Max_accel))
	case "SQUARE_CORNER_VELOCITY":
		configfile.(*PrinterConfig).Set("printer", "square_corner_velocity", fmt.Sprintf("%.2f", self.Square_corner_velocity))
	case "ACCEL_TO_DECEL":
		configfile.(*PrinterConfig).Set("printer", "max_accel_to_decel", fmt.Sprintf("%.2f", self.Requested_accel_to_decel))
	case "ALL":
		configfile.(*PrinterConfig).Set("printer", "max_velocity", fmt.Sprintf("%.2f", self.Max_velocity))
		configfile.(*PrinterConfig).Set("printer", "max_accel", fmt.Sprintf("%.2f", self.Max_accel))
		configfile.(*PrinterConfig).Set("printer", "square_corner_velocity", fmt.Sprintf("%.2f", self.Square_corner_velocity))
		configfile.(*PrinterConfig).Set("printer", "max_accel_to_decel", fmt.Sprintf("%.2f", self.Requested_accel_to_decel))
	default:
		msg := fmt.Sprintf("max_velocity: %.6f"+
			"max_accel: %.6f"+
			"max_accel_to_decel: %.6f"+
			"square_corner_velocity: %.6f",
			self.Max_velocity, self.Max_accel, self.Requested_accel_to_decel, self.Square_corner_velocity)
		gcmd.Respond_info(msg, false)

	}
	return nil
}

func (self *Toolhead) Cmd_M204(cmd interface{}) error {
	gcmd := cmd.(*GCodeCommand)
	// Use S for accel
	above := 0.0
	accel := gcmd.Get_float("S", math.NaN(), nil, nil, &above, nil)
	if math.IsNaN(accel) {
		// Use minimum of P and T for accel
		p := gcmd.Get_float("P", math.NaN(), nil, nil, &above, nil)
		t := gcmd.Get_float("T", math.NaN(), nil, nil, &above, nil)

		if math.IsNaN(p) == false && math.IsNaN(t) == false {
			accel = math.Min(p, t)
		} else if math.IsNaN(p) == false {
			accel = p
		} else if math.IsNaN(t) == false {
			accel = t
		} else {
			gcmd.Respond_info(fmt.Sprintf(
				"Invalid M204 command \"%s\"",
				gcmd.Get_commandline()), true)
			return nil
		}
	}

	if accel > self.Limit_Max_accel {
		accel = self.Limit_Max_accel
	}
	self.M204(accel)
	return nil
}

func (self *Toolhead) M204(accel float64) {
	self.Max_accel = accel
	self.Calc_junction_deviation()
}

func Add_printer_objects_toolhead(config *ConfigWrapper) {
	config.Get_printer().Add_object("toolhead", NewToolhead(config))
	Add_printer_objects_extruder(config)
}
