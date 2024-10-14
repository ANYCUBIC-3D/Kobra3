package value

import (
	"encoding/json"
	"io/ioutil"
	"k3c/common/jsons/model"
	"k3c/common/utils/file"
	"log"
	"os"
)

var StaticValue *ApiValue

type ApiValue struct {
	Debug  *log.Logger
	Error  *log.Logger
	Config *model.Config
	Passwd map[string]string
	Status string
}

func InitValue() *ApiValue {

	value := ApiValue{Status: "start", Passwd: make(map[string]string)}
	value.Config = value.loadConfig()
	if value.Config == nil {
		return nil
	}

	if "debug" == value.Config.Sys.LogLevel || "" == value.Config.Sys.LogLevel {
		value.Debug = log.New(os.Stdout, "DEBUG ", log.LstdFlags|log.Lshortfile|log.Lmsgprefix)
		value.Error = log.New(os.Stderr, "ERROR ", log.LstdFlags|log.Lshortfile|log.Lmsgprefix)
	} else if "error" == value.Config.Sys.LogLevel {
		value.Debug = log.New(ioutil.Discard, "DEBUG ", log.LstdFlags|log.Lshortfile|log.Lmsgprefix)
		value.Error = log.New(os.Stderr, "ERROR ", log.LstdFlags|log.Lshortfile|log.Lmsgprefix)
		log.SetOutput(ioutil.Discard)
	} else {
		value.Debug = log.New(ioutil.Discard, "DEBUG ", log.LstdFlags|log.Lshortfile|log.Lmsgprefix)
		value.Error = log.New(ioutil.Discard, "ERROR ", log.LstdFlags|log.Lshortfile|log.Lmsgprefix)
	}

	log.SetPrefix("INFO ")
	log.SetFlags(log.LstdFlags | log.Lmsgprefix)

	return &value
}
func (self ApiValue) loadConfig() *model.Config {
	fileName := "config/api.cfg"
	bs, err := file.GetBytes(fileName)
	if err != nil {
		log.Println("加载配置文件", fileName, "失败", err.Error())
		return nil
	}
	config := model.Config{}
	err = json.Unmarshal(bs, &config)
	if err != nil {
		log.Println("加载配置文件", fileName, err)
		os.Exit(1)
	}

	return &config
}

func SetDebug(flag bool) {
	if flag {
		StaticValue.Debug.SetOutput(os.Stdout)
	} else {
		StaticValue.Debug.SetOutput(ioutil.Discard)
	}
}
