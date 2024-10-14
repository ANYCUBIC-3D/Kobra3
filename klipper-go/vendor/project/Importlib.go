package project

import (
	"errors"
	"fmt"
)

func LoadMainModule() map[string]interface{} {
	module := map[string]interface{}{
		"fan":                                    Load_config_fan,
		"safe_z_home":                            Load_config_safe_z_home,
		"homing":                                 Load_config_homing,
		"heaters":                                Load_config_heaters,
		"heater_bed":                             Load_config_Heater_bed,
		"adc_temperature":                        Load_config_adc_temperature,
		"thermistor NTC 100K MGB18-104F39050L32": Load_config_thermistor,
		"thermistor NTC 100K beta 3950":          Load_config_thermistor,
		"query_adc":                              Load_config_query_adc,
		"heater_fan extruder_fan":                Load_config_heater_fan,
		"controller_fan controller_fan":          Load_config_controller_fan,
		"stepper_enable":                         Load_config_StepperEnable,
		"gcode_move":                             Load_config_gcode_move,
		"timelapse":                              Load_config_timelapse,
		"probe":                                  Load_config_probe,
		"gcode_macro_1":                          Load_config_printer_gcode_macro,
		"gcode_macro":                            Load_config_gcode_macro,
		"query_endstops":                         Load_config_query_endstops,
		"virtual_sdcard":                         Load_config_virtual_sdcard,
		"idle_timeout":                           Load_config_idletimeout,
		"encoder_sensor":                         Load_config_prefix_EncoderSensor,
		"pause_resume":                           Load_config_pause_resume,
		"bed_mesh":                               Load_config_bed_mesh,
		"tmc2209":                                Load_config_TMC2209,
		"force_move":                             Load_config_force_move,
		"gcode_arcs":                             load_config_ArcSupport,
		"resonance_tester":                       Load_config_resonanceTester,
		"input_shaper":                           load_config_InputShaper,
		"print_stats":                            Load_config_PrintStats,
		"statistics":                             Load_config_PrinterStats,
		"filament_switch_sensor filament_sensor": Load_config_SwitchSensor,
		"buttons":                                Load_config_PrinterButtons,
		"lis2dw12":                               Load_config_LIS2DW12,
		"leviQ3":                                 Load_config_LeviQ3,
		"pid_calibrate":                          Load_config_PIDCalibrate,
		"verify_heater":                          Load_config_verify_heater,
		"output_pin":                             Load_config_prefix_DigitalOut,
		"led_pin":                                Load_config_prefix_LedOutPut,
		"cs1237":                                 Load_config_cs1237,
		"printer_marco":                          Load_config_printer_marco,
		"exclude_object":                         Load_config_ExcludeObject,
	}
	return module
}

func Import_module(moduleName string) (interface{}, error) {
	if moduleName == "kinematics.cartesian" {
		return Load_kinematics_cartesian, nil
	}
	if moduleName == "kinematics.none" {
		return Load_kinematics_none, nil
	}
	return nil, errors.New(fmt.Sprintf("module about %s not support", moduleName))
}
