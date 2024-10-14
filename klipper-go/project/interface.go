package project

type ITemperature interface {
	IGetTemperature
	ISetTemperature
}

type IGetTemperature interface {
	Get_temp(eventtime float64) (float64, float64)
}

type ISetTemperature interface {
	Set_temp(degrees float64)
}

type IKinematics interface {
	Set_position(newpos []float64, homing_axes []int)
	Check_move(move *Move)
	Get_status(eventtime float64) map[string]interface{}
	Get_steppers() []interface{}
	Set_rails_z_offset(offset float64)
	Set_limit_z(z float64)
	Note_z_not_homed()
	Calc_position(stepper_positions map[string]float64) []float64
	Home(homing_state *Homing)
	Set_home_callback(func(*Homing))
	Check_move_positon(move_pos []float64, bases_pos []float64) error
	Get_axis_range(axis int) (float64, float64)
}
