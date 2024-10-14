package project

import (
	"fmt"
	"k3c/common/utils/object"
	"k3c/common/value"
)

func Load_config_heater_Generic(config *ConfigWrapper) interface{} {
	pheaters := config.Get_printer().Load_object(config, "heaters", object.Sentinel{})
	pheater, ok := pheaters.(*PrinterHeaters)
	if !ok {
		value.StaticValue.Error.Printf("pheaters type is %T not *PrinterHeaters\n", pheater)
		panic(fmt.Errorf("pheaters type is %T not *PrinterHeaters", pheater))
	}
	return pheater.Setup_heater(config, "")
}
