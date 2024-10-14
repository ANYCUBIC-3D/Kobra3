package main

import (
	"k3c/common/utils/sys"
	"k3c/common/value"
	K3C "k3c/project"
	"os"
	"time"
)

func main() {
	value.StaticValue = value.InitValue()
	if value.StaticValue == nil {
		os.Exit(-1)
		return
	}
	value.StaticValue.Status = "start"
	value.StaticValue.Debug.Printf("main thread %d running", sys.GetGID())
	K3C.ModuleK3C()
	k3c := K3C.NewK3C()
	k3c.Main()
	for {
		if sys.CheckShutdown() {
			break
		} else {
			time.Sleep(1 * time.Second)
		}
	}
	os.Exit(0)
	return
}
