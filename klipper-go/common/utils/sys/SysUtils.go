package sys

import (
	"bytes"
	"errors"
	"fmt"
	"k3c/common/value"
	"os"
	"path"
	"reflect"
	"runtime"
	"runtime/debug"
	"strconv"
	"strings"
)

func GetGID() uint64 {
	b := make([]byte, 64)
	b = b[:runtime.Stack(b, false)]
	b = bytes.TrimPrefix(b, []byte("goroutine "))
	b = b[:bytes.IndexByte(b, ' ')]
	n, _ := strconv.ParseUint(string(b), 10, 64)
	return n
}

//取得CPU信息
func GetCpuInfo() string {
	cpuInfoFilebytes, err := os.ReadFile("/proc/cpuinfo")
	if err != nil {
		return "?"
	}

	var coreCount int
	var modelName string
	cpuInfoLines := strings.Split(string(cpuInfoFilebytes), "\n")
	for _, line := range cpuInfoLines {
		if strings.Index(line, ":") == -1 {
			continue
		}
		lines := strings.Split(line, ":")
		fieldName := strings.TrimSpace(lines[0])
		if fieldName == "processor" {
			coreCount++
		} else if fieldName == "model name" {
			modelName = strings.TrimSpace(lines[1])
		}
	}
	return fmt.Sprintf("%d core %s", coreCount, modelName)
}

//
func GetSoftwareVersion() string {
	//info, _ := cpu.Info() //总体信息
	//fmt.Println(info)
	return ""
}

//检查程序是否关闭
func CheckShutdown() bool {

	if value.StaticValue.Status == "shutdown" {
		return true
	} else {
		return false
	}

}
func CheckClose() bool {

	if value.StaticValue.Status == "stop" {
		return true
	} else {
		return false
	}

}
func CheckStart() bool {

	if value.StaticValue.Status == "start" {
		return true
	} else {
		return false
	}

}
func CheckRun() bool {

	if value.StaticValue.Status == "run" {
		return true
	} else {
		return false
	}

}

func enumerate(obj interface{}) map[int]interface{} {

	var value = reflect.ValueOf(obj)
	var res map[int]interface{}

	if value.Kind() == reflect.String {
		for i, s := range value.String() {
			res[i] = s
		}
	} else if value.Elem().Kind() == reflect.Slice {
		var l = value.Len()
		for i := 0; i < l; i++ {
			v := value.Index(i) // Value of item
			typel := v.Type()   // Type of item
			if typel.Kind() == reflect.Struct {
				return nil
			}
			res[i] = v
		}
	}
	return res
}

func CatchPanic() {
	//处理panic
	if err := recover(); err != nil {
		msg, ok := err.(string)
		if ok && "exit" == msg {
			panic(msg)
		}
		s := string(debug.Stack())
		value.StaticValue.Error.Println("panic:", GetGID(), err, s)
	}
}

// PidStatInfo
// @see https://github.com/struCoder/pidusage
type PidStatInfo struct {
	Utime  float64 // CPU time spent in user code, measured in clock ticks
	Stime  float64 // CPU time spent in kernel code, measured in clock ticks
	Cutime float64 // Waited-for children's CPU time spent in user code (in clock ticks)
	Cstime float64 // Waited-for children's CPU time spent in kernel code (in clock ticks)
	Rss    float64 // Resident Set Size
}

type LoadavgInfo struct {
	Load1  float64 // the system load averages for the past 1 minute
	Load5  float64 // the system load averages for the past 5 minutes
	Load15 float64 // the system load averages for the past 15 minutes
}

func PidStat(pid ...int) (stat PidStatInfo, err error) {
	var statfile = "/proc/self/stat"
	if len(pid) > 0 {
		statfile = path.Join("/proc", strconv.Itoa(pid[0]), "stat")
	}
	procStatFileBytes, err := os.ReadFile(statfile)
	if err != nil {
		return stat, err
	}

	splitAfter := strings.SplitAfter(string(procStatFileBytes), ")")
	if len(splitAfter) == 0 || len(splitAfter) == 1 {
		return stat, errors.New("Can't find process info from " + statfile)
	}
	infos := strings.Split(splitAfter[1], " ")
	stat = PidStatInfo{
		Utime:  parseFloat(infos[12]),
		Stime:  parseFloat(infos[13]),
		Cutime: parseFloat(infos[14]),
		Cstime: parseFloat(infos[15]),
		Rss:    parseFloat(infos[22]),
	}

	return stat, nil
}

func Loadavg() (info LoadavgInfo, err error) {
	uptimeFileBytes, err := os.ReadFile(path.Join("/proc", "loadavg"))
	if err != nil {
		return
	}
	infos := strings.Split(string(uptimeFileBytes), " ")
	if len(infos) < 3 {
		return info, fmt.Errorf("loadavg info invalid: %s", string(uptimeFileBytes))
	}

	info = LoadavgInfo{
		Load1:  parseFloat(infos[0]),
		Load5:  parseFloat(infos[1]),
		Load15: parseFloat(infos[2]),
	}
	return info, nil
}

func parseFloat(val string) float64 {
	floatVal, _ := strconv.ParseFloat(val, 64)
	return floatVal
}
