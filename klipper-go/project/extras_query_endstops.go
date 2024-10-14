/*
Utility for querying the current state of all endstops

Copyright (C) 2018-2019  Kevin O'Connor <kevin@koconnor.net>

This file may be distributed under the terms of the GNU GPLv3 license.
*/

package project

import (
	"container/list"
	"fmt"
	"k3c/common/utils/object"
	"strings"
)

type QueryEndstops struct {
	Printer    *Printer
	Endstops   *list.List
	Last_state *list.List
}

func NewQueryEndstops(config *ConfigWrapper) *QueryEndstops {
	self := &QueryEndstops{}
	self.Printer = config.Get_printer()
	self.Endstops = list.New()
	self.Last_state = list.New()
	// Register webhook if server is available
	webhooks := self.Printer.Lookup_object("webhooks", nil)
	//if err != nil {
	//	value.StaticValue.Error.Println(err)
	//}
	webhooks.(*WebHooks).Register_endpoint(
		"query_endstops/status", self.Handle_web_request)
	gcode := self.Printer.Lookup_object("gcode", object.Sentinel{})
	//if err != nil {
	//	value.StaticValue.Error.Println(err)
	//}
	cmd_QUERY_ENDSTOPS_help := "Report on the status of each endstop"
	gcode.(*GCodeDispatch).Register_command("QUERY_ENDSTOPS", self.cmd_QUERY_ENDSTOPS,
		false, cmd_QUERY_ENDSTOPS_help)
	gcode.(*GCodeDispatch).Register_command("M119", self.cmd_QUERY_ENDSTOPS, false, "")
	return self
}

func (self *QueryEndstops) Register_endstop(mcu_endstop interface{}, name string) {
	node := list.New()
	node.PushBack(mcu_endstop)
	node.PushBack(name)
	self.Endstops.PushBack(node)
}

func (self *QueryEndstops) Get_status(eventtime interface{}) map[string]map[string]interface{} {
	data := make(map[string]interface{})
	for i := self.Last_state.Front(); i != nil; i = i.Next() {
		node := i.Value.(list.List)
		name := node.Front().Next().Value.(string)
		value := node.Front().Next().Value
		data[name] = value
	}
	last_query := map[string]map[string]interface{}{
		"last_query": data,
	}
	return last_query
}

func (self *QueryEndstops) Handle_web_request(web_request *WebRequest) (interface{}, error) {
	gcode := self.Printer.Lookup_object("gcode", object.Sentinel{})
	//if err != nil {
	//	value.StaticValue.Error.Println(err)
	//}
	gc_mutex := gcode.(*GCodeDispatch).Get_mutex()
	toolhead := self.Printer.Lookup_object("toolhead", object.Sentinel{})
	//if err != nil {
	//	value.StaticValue.Error.Println(err)
	//}
	gc_mutex.Lock()
	print_time := toolhead.(*Toolhead).Get_last_move_time()
	last_state := list.New()
	for i := self.Endstops.Front(); i != nil; i = i.Next() {
		node := i.Value.(*list.List)
		mcu_endstop := node.Front().Value
		name := node.Front().Next().Value.(string)
		newNode := list.New()
		newNode.PushBack(name)
		if _, ok := mcu_endstop.(*ProbeEndstopWrapper); ok {
			newNode.PushBack(mcu_endstop.(*ProbeEndstopWrapper).Query_endstop(print_time))
		} else {
			newNode.PushBack(mcu_endstop.(*MCU_endstop).Query_endstop(print_time))
		}

		last_state.PushBack(newNode)
	}
	defer gc_mutex.Unlock()
	codeArr := []string{"open", "TRIGGERED"}
	sendMap := make(map[string]string)
	for i := self.Endstops.Front(); i != nil; i = i.Next() {
		node := i.Value.(*list.List)
		name := node.Front().Next().Value.(string)
		t := node.Front().Next().Value
		var code string
		if t == 0 {
			code = codeArr[0]
		} else {
			code = codeArr[1]
		}
		sendMap[name] = code
	}
	web_request.Send(sendMap)
	return nil, nil
}

func (self *QueryEndstops) cmd_QUERY_ENDSTOPS(gcmd interface{}) (interface{}, error) {
	toolhead := self.Printer.Lookup_object("toolhead", object.Sentinel{})
	//if err != nil {
	//	value.StaticValue.Error.Println(err)
	//}
	print_time := toolhead.(*Toolhead).Get_last_move_time()
	last_state := list.New()
	for i := self.Endstops.Front(); i != nil; i = i.Next() {
		node := i.Value.(list.List)
		// todo 强转成对应类型
		mcu_endstop := node.Front().Next().Value
		name := node.Front().Next().Value.(string)
		newNode := list.New()
		newNode.PushBack(name)
		newNode.PushBack(mcu_endstop.(*MCU_endstop).Query_endstop(print_time))
		last_state.PushBack(newNode)
	}

	codeArr := []string{"open", "TRIGGERED"}
	msgArr := []string{}
	for i := self.Endstops.Front(); i != nil; i = i.Next() {
		node := i.Value.(list.List)
		name := node.Front().Next().Value.(string)
		t := node.Front().Next().Value
		var code string
		if t == 0 {
			code = codeArr[0]
		} else {
			code = codeArr[1]
		}
		msgArr = append(msgArr, fmt.Sprintf("%s:%s", name, code))
	}
	gcmd.(*GCodeCommand).Respond_raw(strings.Join(msgArr, " "))
	return nil, nil
}

func Load_config_query_endstops(config *ConfigWrapper) interface{} {
	return NewQueryEndstops(config)
}
