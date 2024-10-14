package model

// 配置文件${home}/api.cfg
type Config struct {
	Sys       Sys         `json:"sys"`
	Plan      Plan        `json:"plan"`
	PlanFile  PlanFile    `json:"planFile"`
	Controler []Controler `json:"controler"`
	PProf     PProf       `json:"pprof"`
}
type Plan struct {
	Url       string `json:"url"`
	AppId     string `json:"appId"`
	AppPasswd string `json:"appPasswd"`
}
type PlanFile struct {
	Ip         string `json:"ip"`
	Port       int    `json:"port"`
	User       string `json:"user"`
	UserPasswd string `json:"userPasswd"`
}

type Sys struct {
	ServerSocketPort int    `json:"serverSocketPort"`
	DataPath         string `json:"dataPath"`
	PrinterPath      string `json:"printerPath"`
	Uds              string `json:"uds"`
	LogLevel         string `json:"logLevel"`
	StatePath        string `json:"StatePath"`
	OtaPath          string `json:"OtaPath"`
	McuTargetVersion string `json:"McuTargetVersion"`
}

type Controler struct {
	Code string `json:"code"`
	Impl string `json:"impl"`
}

// 密码文件	${data}/passwd.cfg
type Passwds struct {
	Passwd []Passwd `json:"passwd"`
}
type Passwd struct {
	AppId string `json:"appId"`
	Pwd   string `json:"pwd"`
}

type PProf struct {
	Enable bool   `json:"enable"`
	Addr   string `json:"addr"`
	Prefix string `json:"prefix"`
}
