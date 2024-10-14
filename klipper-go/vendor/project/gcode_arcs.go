package project

import (
	"k3c/common/constants"
	"k3c/common/utils/cast"
	"k3c/common/value"
	"math"
)

/**
# Coordinates created by this are converted into G1 commands.
#
# supports XY, XZ & YZ planes with remaining axis as helical
*/

// Enum
var (
	ARC_PLANE_X_Y = 0
	ARC_PLANE_X_Z = 1
	ARC_PLANE_Y_Z = 2
)

// Enum
var (
	X_AXIS = 0
	Y_AXIS = 1
	Z_AXIS = 2
	E_AXIS = 3
)

type ArcSupport struct {
	printer            *Printer
	mm_per_arc_segment float64
	gcode_move         *GCodeMove
	gcode              *GCodeDispatch
	Coord              []string
	plane              int
}

func NewArcSupport(config *ConfigWrapper) ArcSupport {
	var self = ArcSupport{}
	self.printer = config.Get_printer()
	self.mm_per_arc_segment = config.Getfloat("resolution", 1., 0, 0, 0, 0.0, true)

	self.gcode_move = MustLoadGcodeMove(config)
	self.gcode = MustLookupGcode(self.printer)
	self.gcode.Register_command("G2", self.cmd_G2, false, "")
	self.gcode.Register_command("G3", self.cmd_G3, false, "")

	self.gcode.Register_command("G17", self.cmd_G17, false, "")
	self.gcode.Register_command("G18", self.cmd_G18, false, "")
	self.gcode.Register_command("G19", self.cmd_G19, false, "")

	self.Coord = self.gcode.Coord

	// backwards compatibility, prior implementation only supported XY
	self.plane = ARC_PLANE_X_Y
	return self
}

func (self *ArcSupport) cmd_G2(argv interface{}) error {
	self._cmd_inner(argv.(*GCodeCommand), true)
	return nil
}

func (self *ArcSupport) cmd_G3(argv interface{}) error {
	self._cmd_inner(argv.(*GCodeCommand), false)
	return nil
}

func (self *ArcSupport) cmd_G17(argv interface{}) error {
	self.plane = ARC_PLANE_X_Y
	return nil
}

func (self *ArcSupport) cmd_G18(argv interface{}) error {
	self.plane = ARC_PLANE_X_Z
	return nil
}

func (self *ArcSupport) cmd_G19(argv interface{}) error {
	self.plane = ARC_PLANE_Y_Z
	return nil
}

func (self *ArcSupport) _cmd_inner(gcmd *GCodeCommand, clockwise bool) {
	var gcodestatus = self.gcode_move.Get_status(constants.NOW)
	if value.Not(gcodestatus["absolute_coordinates"]) {
		panic("G2/G3 does not support relative move mode")
	}

	var currentPos = gcodestatus["gcode_position"].([]float64)

	// Parse parameters
	x := gcmd.Get_float("X", currentPos[0], nil, nil, nil, nil)
	y := gcmd.Get_float("Y", currentPos[1], nil, nil, nil, nil)
	z := gcmd.Get_float("Z", currentPos[2], nil, nil, nil, nil)

	/**
	asTarget = self.Coord(x=gcmd.get_float("X", currentPos[0]),
	                          y=gcmd.get_float("Y", currentPos[1]),
	                          z=gcmd.get_float("Z", currentPos[2]),
	                          e=None)
	*/
	asTarget := []float64{x, y, z, value.Float64None}

	_R := gcmd.Get_floatP("R", nil, nil, nil, nil, nil)
	if value.IsNotNone(_R) {
		panic("G2/G3 does not support R moves")
	}

	// determine the plane coordinates and the helical axis

	asPlanar := make([]float64, 2)
	for i, a := range []string{"I", "J"} {
		asPlanar[i] = gcmd.Get_float(a, 0., nil, nil, nil, nil)
	}

	axes := []int{X_AXIS, Y_AXIS, Z_AXIS}
	if self.plane == ARC_PLANE_X_Z {
		for i, a := range []string{"I", "K"} {
			asPlanar[i] = gcmd.Get_float(a, 0., nil, nil, nil, nil)
		}
		axes = []int{X_AXIS, Z_AXIS, Y_AXIS}
	} else if self.plane == ARC_PLANE_Y_Z {
		for i, a := range []string{"J", "K"} {
			asPlanar[i] = gcmd.Get_float(a, 0., nil, nil, nil, nil)
			axes = []int{Y_AXIS, Z_AXIS, X_AXIS}
		}
	}

	if value.Not(value.True(asPlanar[0]) || value.True(asPlanar[1])) {
		panic("G2/G3 requires IJ, IK or JK parameters")
	}

	var (
		_asE = gcmd.Get_floatP("E", nil, nil, nil, nil, nil)
		_asF = gcmd.Get_floatP("F", nil, nil, nil, nil, nil)
		asE  = cast.Float64(_asE)
		asF  = cast.Float64(_asF)
	)

	// Build list of linear coordinates to move
	coords := self.planArc(currentPos, asTarget, asPlanar,
		clockwise, axes[0], axes[1], axes[2]) // @todo

	e_per_move := 0.0
	e_base := 0.

	if value.IsNotNone(_asE) {
		if value.True(gcodestatus["absolute_extrude"]) {
			e_base = currentPos[3]
		}
		e_per_move = (asE - e_base) / float64(len(coords))
	}
	// Convert coords into G1 commands
	for _, coord := range coords {
		g1_params := map[string]string{"X": cast.ToString(coord[0]), "Y": cast.ToString(coord[1]), "Z": cast.ToString(coord[2])}
		if value.True(e_per_move) {
			g1_params["E"] = cast.ToString(e_base + e_per_move)
			if value.True(gcodestatus["absolute_extrude"]) {
				e_base += e_per_move
			}
		}

		if value.IsNotNone(_asF) {
			g1_params["F"] = cast.ToString(asF)
		}

		g1_gcmd := self.gcode.Create_gcode_command("G1", "G1", g1_params)

		self.gcode_move.Cmd_G1(g1_gcmd)
	}
}

/*
# function planArc() originates from marlin plan_arc()
# https://github.com/MarlinFirmware/Marlin
#
# The arc is approximated by generating many small linear segments.
# The length of each segment is configured in MM_PER_ARC_SEGMENT
# Arcs smaller then this value, will be a Line only
#
# alpha and beta axes are the current plane, helical axis is linear travel
*/

func (self *ArcSupport) planArc(currentPos []float64, targetPos []float64, offset []float64, clockwise bool,
	alpha_axis int, beta_axis int, helical_axis int) [][]float64 {

	// todo: sometimes produces full circles

	// Radius vector from center to current location

	r_P := -offset[0]
	r_Q := -offset[1]
	// Determine angular travel
	center_P := currentPos[alpha_axis] - r_P
	center_Q := currentPos[beta_axis] - r_Q
	rt_Alpha := targetPos[alpha_axis] - center_P
	rt_Beta := targetPos[beta_axis] - center_Q

	angular_travel := math.Atan2(r_P*rt_Beta-r_Q*rt_Alpha,
		r_P*rt_Alpha+r_Q*rt_Beta)

	if angular_travel < 0. {
		angular_travel += 2. * math.Pi
	}
	if clockwise {
		angular_travel -= 2. * math.Pi
	}
	if angular_travel == 0. &&
		currentPos[alpha_axis] == targetPos[alpha_axis] &&
		currentPos[beta_axis] == targetPos[beta_axis] {
		// Make a circle if the angular rotation is 0 and the
		// target is current position
		angular_travel = 2. * math.Pi
	}

	// Determine number of segments
	linear_travel := targetPos[helical_axis] - currentPos[helical_axis]
	radius := math.Hypot(r_P, r_Q)
	flat_mm := radius * angular_travel
	var mm_of_travel float64
	if value.True(linear_travel) {
		mm_of_travel = math.Hypot(flat_mm, linear_travel)
	} else {
		mm_of_travel = math.Abs(flat_mm)
	}

	segments := math.Max(1., math.Floor(mm_of_travel/self.mm_per_arc_segment))

	//  Generate coordinates
	theta_per_segment := angular_travel / segments
	linear_per_segment := linear_travel / segments
	coords := make([][]float64, 0)
	//for i := range iterator.RangeInt(1, cast.ToInt(segments)) {
	segmentsInt := cast.ToInt(segments)
	for i := 1; i < segmentsInt; i++ {
		iv := cast.ToFloat64(i)
		dist_Helical := iv * linear_per_segment
		cos_Ti := math.Cos(iv * theta_per_segment)
		sin_Ti := math.Sin(iv * theta_per_segment)
		r_P = -offset[0]*cos_Ti + offset[1]*sin_Ti
		r_Q = -offset[0]*sin_Ti - offset[1]*cos_Ti

		// Coord doesn't support index assignment, create list
		c := []float64{0, 0, 0, 0}
		c[alpha_axis] = center_P + r_P
		c[beta_axis] = center_Q + r_Q
		c[helical_axis] = currentPos[helical_axis] + dist_Helical
		// coords.append(self.Coord(*c))
		coords = append(coords, c)
	}

	coords = append(coords, targetPos)
	return coords

}

func load_config_ArcSupport(config *ConfigWrapper) interface{} {
	return NewArcSupport(config)
}
