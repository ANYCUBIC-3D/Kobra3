package main

import (
	"flag"
	"fmt"
	"k3c/common/value"
	"k3c/project"
	"os"
	"strings"
)

type OptionParser struct {
	Verbose      bool
	Baud         int
	CanbusIface  string
	CanbusNodeid int
}

func main() {
	flag.Usage = func() {
		fmt.Fprintf(os.Stderr, "%s, [options] <serialdevice>\n", os.Args[0])
		flag.PrintDefaults()
	}

	var options OptionParser
	flag.BoolVar(&options.Verbose, "v", false, "enable debug messages")
	flag.IntVar(&options.Baud, "b", 0, "baud rate")
	flag.StringVar(&options.CanbusIface, "c", "", "Use CAN bus interface; serialdevice is the chip UUID")
	flag.IntVar(&options.CanbusNodeid, "i", 64, "The CAN nodeid to use (default 64)")
	flag.Parse()

	args := flag.Args()
	if len(args) != 1 {
		fmt.Fprintf(os.Stderr, "Incorrect number of arguments\n")
		os.Exit(1)
	}

	serialport := args[0]
	baud := options.Baud
	if options.Baud == 0 && !(strings.HasPrefix(serialport, "/dev/rpmsg_") || strings.HasPrefix(serialport, "/tmp/")) {
		baud = 250000
	}

	debuglevel := project.INFO
	if options.Verbose {
		debuglevel = project.DEBUG
	}
	_ = debuglevel

	r := project.NewEPollReactor(true)
	project.NewKeyboardReader(r, serialport, baud, options.CanbusIface, options.CanbusNodeid)
	if err := r.Run(); err != nil {
		value.StaticValue.Error.Fatalf("NewEPollReactor run error: %v", err)
	}
}
