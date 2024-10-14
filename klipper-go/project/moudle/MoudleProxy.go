package moudle

func getMouleInit(moudleName string) interface{} {
	switch moudleName {
	case "adc_temperature":
		return ""
	case "bme280":
		return ""

	case "ds18b20":
		return ""

	case "htu21d":
		return ""

	case "lm75":
		return ""

	case "spi_temperature":
		return ""

	case "temperature_host":
		return ""

	case "temperature_mcu":
		return ""
	}
	return nil
}
