// Low level unix utility functions
//
// Copyright (C) 2016-2020  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
package util

import (
	"log"
	"net"
	"os"
	"os/signal"
	"strings"
	"syscall"
)

/*#####################################################################
# Low-level Unix commands
######################################################################
*/

func init() {
	fix_sigint()
	setup_python2_wrappers()
}

// Return the SIGINT interrupt handler back to the OS default
func fix_sigint() {
	signal.Reset(os.Interrupt)
}

// Set a file-descriptor as non-blocking
func Set_nonblock(fd int) {
	syscall.SetNonblock(fd, true)
}

// Clear HUPCL flag
func Clear_hupcl(fd uintptr) {
}

// Support for creating a pseudo-tty for emulating a serial port
func create_pty(ptyname string) int {
	return 0
}

/*#####################################################################
# Helper code for extracting mcu build info
######################################################################
*/

func Dump_file_stats(build_dir, filename string) {
	fname := strings.Join([]string{build_dir, filename}, string(os.PathSeparator))
	fileInfo, err := os.Stat(fname)
	mtime := fileInfo.ModTime()
	fsize := fileInfo.Size()
	timestr := mtime.Format("2006-01-02 15:04:05")
	log.Printf("Build file %s(%d): %s", fname, fsize, timestr)
	if err != nil {
		log.Println("No build file %s", fname)
	}
}

// Try to log information on the last mcu build
func Dump_mcu_build() {
}

/*
#####################################################################
# Python2 wrapper hacks
######################################################################
*/
func setup_python2_wrappers() {
}

/*
#####################################################################
# General system and software information
######################################################################
*/
func get_cpu_info() string {
	return "?"
}

func get_version_from_file(K3c_src string) string {
	return "?"
}

func get_git_version(from_file bool) string {
	return "?"
}

func Fileno(conn net.Conn) int {
	c := conn.(*net.UnixConn)
	file, err := c.File()
	if err != nil {
		log.Print(err.Error())
	}
	return int(file.Fd())
}

func Reverse(s []string) []string {
	newS := make([]string, len(s))
	for i, j := 0, len(s)-1; i <= j; i, j = i+1, j-1 {
		newS[i], newS[j] = s[j], s[i]
	}
	return newS
}
