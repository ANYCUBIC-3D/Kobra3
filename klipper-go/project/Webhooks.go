// K3c WebHooks registration and server connection
//
// Copyright (C) 2020 Eric Callahan <arksine.code@gmail.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license
package project

import (
	"container/list"
	"encoding/json"
	"errors"
	"fmt"
	"k3c/common/constants"
	kerror "k3c/common/errors"
	"k3c/common/utils/cast"
	"k3c/common/utils/collections"
	"k3c/common/utils/object"
	"k3c/common/utils/reflects"
	"k3c/common/utils/str"
	"k3c/common/utils/sys"
	"k3c/common/value"
	"k3c/project/chelper"
	"k3c/project/util"
	"runtime/debug"
	"sync"
	"time"

	uuid "github.com/satori/go.uuid"

	"io"

	//"io/ioutil"
	"reflect"
	"strings"

	"k3c/common/utils/file"
	"log"
	"net"
	"os"
	"syscall"
)

const (
	REQUEST_LOG_SIZE = 20
)

// Json decodes strings as unicode types in Python 2.x.  This doesn't
// play well with some parts of Klipper (particuarly displays), so we
// need to create an object hook. This solution borrowed from:
//
// https://stackoverflow.com/questions/956867/
//
//json_loads_byteify = nil
// type WebRequestError struct {
// 	message string
// }

// func NewWebRequestError(message string) *WebRequestError {
// 	self := WebRequestError{}
// 	self.message = message
// 	return &self
// }
// func (self *WebRequestError) to_dict() map[string]string {
// 	return map[string]string{
// 		"error":   "WebRequestError",
// 		"message": self.message,
// 	}
// }

type WebRequest struct {
	//error1      *Error
	client_conn *ClientConnection

	id     float64
	method string
	params map[string]interface{}

	response interface{}
	is_error bool
}

func NewWebRequest(client_conn *ClientConnection, request string) *WebRequest {
	self := WebRequest{}
	self.client_conn = client_conn
	base_request := make(map[string]interface{})
	err := json.Unmarshal([]byte(request), &base_request)
	if err != nil {
		log.Print(err.Error())
		return nil
	}
	var ok bool
	self.id, ok = base_request["id"].(float64)
	if !ok {
		log.Print("id is Not a number")
		return nil
	}
	self.method, ok = base_request["method"].(string)
	if !ok {
		log.Print("Invalid request type")
		return nil
	}

	if _, ok = base_request["params"]; ok {
		self.params, ok = base_request["params"].(map[string]interface{})
		if !ok {
			log.Print("Invalid request type")
			return nil
		}
	} else {
		self.params = make(map[string]interface{})
	}
	self.response = nil
	self.is_error = false
	return &self
}
func (self *WebRequest) Send(data interface{}) error {
	if self.response != nil {
		return errors.New("Multiple calls to send not allowed")
	}
	self.response = data
	return nil
}
func (self *WebRequest) get_dict(item string, default1 interface{}) map[string]interface{} {
	obj := self.get(item, default1, []reflect.Kind{reflect.Map})
	if obj == nil {
		return nil
	}
	v, ok := obj.(map[string]interface{})
	if ok {
		return v
	}
	return nil
}
func (self *WebRequest) get_str(item string, default1 interface{}) string {
	obj := self.get(item, default1, []reflect.Kind{reflect.String})
	if obj == nil {
		return ""
	}
	v, ok := obj.(string)
	if ok {
		return v
	}
	return ""
}
func (self *WebRequest) get_int(item string, default1 interface{}) int {
	obj := self.get(item, default1, []reflect.Kind{reflect.Int})
	if obj == nil {
		return 0
	}
	v, ok := obj.(int)
	if ok {
		return v
	}
	return 0
}
func (self *WebRequest) get_float(item string, default1 interface{}) float64 {
	obj := self.get(item, default1, []reflect.Kind{reflect.Int, reflect.Float64})
	if obj == nil {
		return 0
	}
	v, ok := obj.(float64)
	if ok {
		return v
	}
	return 0

}
func (self *WebRequest) get(item string, default1 interface{}, types []reflect.Kind) interface{} {
	value, ok := self.params[item]
	if !ok {
		if object.IsSentinel(default1) {
			panic(kerror.NewWebRequestError(kerror.MissingArgumentCode, fmt.Sprintf("Missing Argument [%s]", item)))
		} else {
			value = default1
		}
	}
	if types != nil && collections.NotInKind(types, value) && collections.InStringMap(self.params, item) {
		panic(kerror.NewWebRequestError(kerror.InvalidArgumentCode, fmt.Sprintf("Invalid Argument Type [%s]", item)))
	}
	return value
}
func (self *WebRequest) get_method() string {
	return self.method
}
func (self *WebRequest) get_client_connection() *ClientConnection {
	return self.client_conn
}

func (self *WebRequest) set_error(err error) {
	kerr := kerror.FromError(err)
	self.is_error = true
	self.response = map[string]interface{}{
		"error":   kerr.Typ,
		"code":    kerr.Code,
		"message": kerr.Message,
	}
}

func (self *WebRequest) finish() map[string]interface{} {
	if self.id == 0 {
		return nil
	}
	rtype := "result"
	if self.is_error {
		rtype = "error"
	}
	if self.response == nil {
		// No error was set and the user never executed
		// send, default response is {}
		self.response = map[string]string{}
	}
	return map[string]interface{}{"id": self.id, rtype: self.response}
}

type ServerSocket struct {
	printer   *Printer
	webhooks  *WebHooks
	reactor   IReactor
	sock      net.Listener
	fd_handle *ReactorFileHandler
	clients   map[string]*ClientConnection
}

func NewServerSocket(webhooks *WebHooks, printer *Printer) *ServerSocket {
	self := ServerSocket{}
	self.printer = printer
	self.webhooks = webhooks
	self.reactor = printer.Get_reactor()
	self.sock = nil
	self.fd_handle = nil
	self.clients = map[string]*ClientConnection{}
	start_args := printer.Get_start_args()
	server_address, is_server_address := start_args["apiserver"]
	_, is_fileinput := start_args["debuginput"]
	if !is_server_address || is_fileinput {
		fmt.Print(is_fileinput)
		// Do not enable server
		return &self
	}
	self._remove_socket_file(server_address.(string))
	sa, err1 := net.ResolveUnixAddr("unix", server_address.(string))
	if err1 != nil {
		log.Printf(err1.Error())
	}
	l, err1 := net.ListenUnix("unix", sa)
	if err1 != nil {
		log.Print(err1.Error())
	}
	self.sock = l
	file, err1 := l.File()
	if err1 != nil {
		log.Print(err1.Error())
	}
	fd := int(file.Fd())
	err1 = syscall.SetNonblock(fd, false)
	if err1 != nil {
		log.Printf(err1.Error())
	}
	self.fd_handle = self.reactor.Register_fd(
		fd, self._handle_accept, nil)
	printer.Register_event_handler(
		"project:disconnect", self._handle_disconnect)
	printer.Register_event_handler(
		"project:shutdown", self._handle_shutdown)
	//go self.handl_accept()
	return &self

}
func (self *ServerSocket) handl_accept() {
	for {
		self._handle_accept(constants.NOW)
		time.Sleep(1000)
	}
}
func (self *ServerSocket) _handle_accept(eventtime float64) interface{} {

	sock, err := self.sock.Accept()
	if err != nil {
		log.Println(err.Error())
		return "exit"
	}
	err = syscall.SetNonblock(util.Fileno(sock), false)
	if err != nil {
		log.Printf(err.Error())
	}
	client := NewClientConnection(self, sock)
	self.clients[client.uid] = client
	return nil
}
func (self *ServerSocket) _handle_disconnect(params []interface{}) error {
	for _, client := range self.clients {
		//log.Println(k, client)
		err := client.Close()
		if err != nil {
			log.Println(err.Error())
		}
	}
	if self.sock != nil {
		self.reactor.Unregister_fd(self.fd_handle)
		err := self.sock.Close()
		if err != nil {
			log.Println(err.Error())
		}
	}
	return nil
}
func (self *ServerSocket) _handle_shutdown(params []interface{}) error {
	for _, client := range self.clients {
		client.dump_request_log()
	}
	return nil
}

func (self *ServerSocket) _remove_socket_file(file_path string) error {
	err := os.Remove(file_path)
	if err != nil {
		exist, err1 := file.PathExists(file_path)
		if err1 != nil {
			log.Printf(err1.Error())
		} else {
			if exist {
				log.Printf(
					"webhooks: Unable to delete socket file '%s'", file_path)
				return err
			}
		}
	}

	return nil
}
func (self *ServerSocket) pop_client(client_id string) {
	delete(self.clients, client_id)
}

type ClientConnection struct {
	printer        *Printer
	webhooks       *WebHooks
	reactor        IReactor
	server         *ServerSocket
	uid            string
	fd_handle      *ReactorFileHandler
	partial_data   []byte
	is_blocking    bool
	blocking_count int
	sock           net.Conn
	send_buffer    []byte
	request_log    *list.List
	sendMutex      sync.Mutex
}

func NewClientConnection(server *ServerSocket, sock net.Conn) *ClientConnection {
	self := ClientConnection{}
	self.printer = server.printer
	self.webhooks = server.webhooks
	self.reactor = server.reactor
	self.server = server
	self.uid = uuid.NewV4().String()
	self.sock = sock
	self.fd_handle = self.reactor.Register_fd(util.Fileno(sock), self._process_received, self._do_send)
	self.partial_data = []byte{}
	self.send_buffer = []byte{}
	self.is_blocking = false
	self.blocking_count = 0
	self.set_client_info("?", "New connection")

	self.request_log = list.New()
	self.sendMutex = sync.Mutex{}
	//go self.process_received()
	return &self
}
func (self *ClientConnection) dump_request_log() {
	out := []string{}
	out = append(out, fmt.Sprintf("Dumping %d requests for client %s", 1, self.uid))
	//for eventtime, request := range self.request_log {
	//	out = append(out, fmt.Sprintf("Received %f: %s", eventtime, request))
	//}
	log.Println(strings.Join(out, "\n"))

}
func (self *ClientConnection) set_client_info(client_info string, state_msg string) {
	if state_msg == "" {
		state_msg = fmt.Sprintf("Client info %s", client_info)
	}
	log.Printf("webhooks client %s: %s", self.uid, state_msg)
	log_id := fmt.Sprintf("webhooks %s", self.uid)
	if client_info == "" {
		self.printer.Set_rollover_info(log_id, "", false)
		return
	}
	rollover_msg := fmt.Sprintf("webhooks client %s: %s", self.uid, client_info)
	self.printer.Set_rollover_info(log_id, rollover_msg, false)
}
func (self *ClientConnection) Close() error {
	if self.fd_handle == nil {
		return nil
	}
	self.set_client_info("", "Disconnected")
	self.reactor.Unregister_fd(self.fd_handle)
	self.fd_handle = nil

	err := self.sock.Close()
	if err != nil {
		log.Println(err.Error())
	}
	self.server.pop_client(self.uid)
	return nil
}
func (self *ClientConnection) is_closed() bool {
	if self.fd_handle == nil {
		return true
	} else {
		return false
	}
}

func (self *ClientConnection) _process_received(eventtime float64) interface{} {
	bs := make([]byte, 4096)
	c, err := self.sock.Read(bs)
	if err != nil {
		// If bad file descriptor allow connection to be
		// closed by the data check
		if err == syscall.EBADF {
			self.Close()
			return nil
		}
		if err == io.EOF {
			self.Close()
			return nil
		}
		self.Close()
		return err
	}
	var requests []string
	for j := 0; j < c; j++ {
		b := bs[j]
		if b == 03 {
			requests = append(requests, string(self.partial_data))
			self.partial_data = []byte{}
			continue
		}
		self.partial_data = append(self.partial_data, bs[j])
	}
	for _, req := range requests {

		self.request_log.PushBack(eventtime)
		web_request := NewWebRequest(self, req)
		if web_request == nil {
			log.Printf("webhooks: Error decoding Server Request %s", req)
			continue
		}
		func_ := func(argv interface{}) interface{} {
			self._process_request(web_request)
			return argv.(float64)
		}
		self.reactor.Register_callback(func_, constants.NOW)
	}
	return nil
}

func (self *ClientConnection) _process_request(web_request *WebRequest) {
	value.StaticValue.Debug.Print("request_method:", web_request.get_method(), "params:", web_request.params)
	f := self.webhooks.get_callback(web_request.get_method())
	if f == nil {
		value.StaticValue.Error.Print("unregistered for method:", web_request.get_method())
		web_request.set_error(kerror.NewWebRequestError(kerror.MethodUnregisteredCode, "web request method: "+web_request.get_method()+" unregistered"))
	} else {
		_, err := self._process_request1(f, web_request)
		if err != nil {
			web_request.set_error(kerror.FromError(err))
		}
	}

	result := web_request.finish()
	if result == nil {
		return
	}
	self.Send(result)
	//log.Println(result)
}
func (self *ClientConnection) _process_request1(f func(web_request *WebRequest) (interface{}, error), web_request *WebRequest) (interface{}, error) {
	defer sys.CatchPanic()
	return f(web_request)
}
func (self *ClientConnection) Send(data interface{}) {
	if data == nil {
		//处理空指针
		return
	}
	jmsg, err := json.Marshal(data)
	if err != nil {
		log.Print(err.Error())
		return
	}
	self.sendMutex.Lock()
	self.send_buffer = append(jmsg, '\x03')
	self.sendMutex.Unlock()
	if !self.is_blocking {
		self._do_send(0)
	}
}
func (self *ClientConnection) _do_send(eventtime float64) interface{} {
	if self.fd_handle == nil {
		return nil
	}
	self.sendMutex.Lock()
	defer self.sendMutex.Unlock()
	sent, err := self.sock.Write(self.send_buffer)
	if err != nil {
		log.Print("webhooks: socket write error", self.uid, err.Error())
		if err == io.EOF {
			self.Close()
			return nil
		}
		sent = 0
	}
	if sent < len(self.send_buffer) {
		if !self.is_blocking {
			self.reactor.Set_fd_wake(self.fd_handle, false, true)
			self.is_blocking = true
			self.blocking_count = 5
		}
	} else if self.is_blocking {
		self.reactor.Set_fd_wake(self.fd_handle, true, false)
		self.is_blocking = false
	}
	self.send_buffer = self.send_buffer[sent:]
	return nil
}

type WebHooks struct {
	printer         *Printer
	_endpoints      map[string]func(web_request *WebRequest) (interface{}, error)
	_remote_methods map[string]interface{}
	_mux_endpoints  map[string]MuxEndpoint

	sconn *ServerSocket
}

type MuxEndpoint struct {
	key    string
	values map[string]func(*WebRequest)
}

func NewWebHooks(printer *Printer) *WebHooks {
	self := WebHooks{}
	self.printer = printer
	self._endpoints = map[string]func(web_request *WebRequest) (interface{}, error){}
	self._endpoints["list_endpoints"] = self._handle_list_endpoints

	self._remote_methods = map[string]interface{}{}
	self._mux_endpoints = make(map[string]MuxEndpoint)
	self.Register_endpoint("info", self._handle_info_request)
	self.Register_endpoint("emergency_stop", self._handle_estop_request)
	self.Register_endpoint("register_remote_method", self._handle_rpc_registration)

	self.printer.Register_event_handler("project:ready", self._handle_ready)

	return &self
}
func (self WebHooks) _handle_ready(args []interface{}) error {
	self.sconn = NewServerSocket(&self, self.printer)
	return nil
}
func (self WebHooks) Register_endpoint(path string, callback func(*WebRequest) (interface{}, error)) error {
	_, ok := self._endpoints[path]
	if ok {
		return errors.New("Path already registered to an endpoint")
	}
	self._endpoints[path] = callback
	return nil
}
func (self *WebHooks) _handle_list_endpoints(web_request *WebRequest) (interface{}, error) {
	//self._endpoints
	response := map[string]interface{}{
		"endpoints": str.MapStringKeys(self._endpoints),
	}
	web_request.Send(response)
	return nil, nil
}

func (self *WebHooks) _handle_info_request(web_request *WebRequest) (interface{}, error) {
	client_info := web_request.get_dict("client_info", nil)
	if client_info != nil {
		web_request.get_client_connection().set_client_info(fmt.Sprintf("%s", client_info), "")
	}
	state_message, state := self.printer.get_state_message()
	response := map[string]interface{}{
		"state":         state,
		"state_message": state_message,
		"device_type":   self.printer.device_type,
		"print_size":    self.printer.print_size,
		"process_id":    os.Getpid(),
		"user_id":       os.Getuid(),
		"group_id":      os.Getgid(),
	}
	start_args := self.printer.Get_start_args()
	for _, sa := range []string{"log_file", "config_file", "software_version", "cpu_info"} {
		saValue := start_args[sa]
		if saValue != nil {
			response[sa] = saValue.(string)
		}
	}
	web_request.Send(response)
	return nil, nil
}
func (self *WebHooks) _handle_estop_request(web_request *WebRequest) (interface{}, error) {
	self.printer.Invoke_shutdown("Shutdown due to webhooks request")
	return nil, nil
}
func (self *WebHooks) _handle_rpc_registration(web_request *WebRequest) (interface{}, error) {
	template := web_request.get_dict("response_template", object.Sentinel{})
	method := web_request.get_str("remote_method", object.Sentinel{})
	new_conn := web_request.get_client_connection()
	log.Printf("webhooks: registering remote method '%s' for connection id: %d", method, &new_conn)

	childMap, ok := self._remote_methods[method]
	if ok {
		childMap.(map[*ClientConnection]interface{})[new_conn] = template

	} else {
		childMap = map[*ClientConnection]interface{}{}
		childMap.(map[*ClientConnection]interface{})[new_conn] = template
		self._remote_methods[method] = childMap
	}
	return nil, nil
}
func (self *WebHooks) get_callback(path string) func(web_request *WebRequest) (interface{}, error) {
	cb := self._endpoints[path]
	if cb == nil {
		msg := fmt.Sprintf("webhooks: No registered callback for path '%s'", path)
		log.Print(msg)
		return nil
	}
	return cb
}
func (self *WebHooks) Get_status(eventtime float64) map[string]interface{} {
	state_message, state := self.printer.get_state_message()
	return map[string]interface{}{"state": state, "state_message": state_message}
}

func (self *WebHooks) Call_remote_method(method string, kwargs interface{}) error {
	if _, ok := self._remote_methods[method]; !ok {
		return fmt.Errorf("Remote method '%s' not registered", (method))
	}
	conn_map := self._remote_methods[method].(map[*ClientConnection]interface{})
	valid_conns := make(map[*ClientConnection]interface{})
	for conn, template := range conn_map {
		if !conn.is_closed() {
			valid_conns[conn] = template
			out := map[string]interface{}{
				"params": kwargs,
			}
			for k, v := range template.(map[string]interface{}) {
				out[k] = v
			}
			conn.Send(out)
		}
	}

	for len(valid_conns) == 0 {
		delete(self._remote_methods, method)
		return fmt.Errorf("No active connections for method '%s'", method)
	}

	self._remote_methods[method] = valid_conns
	return nil
}

func (self *WebHooks) Register_mux_endpoint(path, key, value string, callback func(*WebRequest)) {
	prev, ok := self._mux_endpoints[path]
	if !ok {
		self.Register_endpoint(path, self._handle_mux)
		prev = MuxEndpoint{
			key:    key,
			values: make(map[string]func(*WebRequest)),
		}
		self._mux_endpoints[path] = prev
	}
	prev_key, prev_values := prev.key, prev.values
	if prev_key != key {
		panic(fmt.Errorf("mux endpoint %s %s %s may have only one key (%s)", path, key, value, prev_key))
	}

	if _, ok := prev_values[value]; ok {
		panic(fmt.Errorf("mux endpoint %s %s %s already registered (%+v)", path, key, value, prev_values))
	}
	prev_values[value] = callback
}

/**
  def _handle_mux(self, web_request):
      key, values = self._mux_endpoints[web_request.get_method()]
      if None in values:
          key_param = web_request.get(key, None)
      else:
          key_param = web_request.get(key)
      if key_param not in values:
          raise web_request.error("The value '%s' is not valid for %s"
                                  % (key_param, key))
      values[key_param](web_request)
*/

func (self *WebHooks) _handle_mux(web_request *WebRequest) (interface{}, error) {
	ep := self._mux_endpoints[web_request.get_method()]
	key, values := ep.key, ep.values
	var key_param string
	/**
		if None in values:
	            key_param = web_request.get(key, None)
	*/
	if _, ok := values[""]; ok {
		key_param = cast.ToString(web_request.get(key, "", nil))
	} else {
		key_param = cast.ToString(web_request.get(key, &object.Sentinel{}, nil))
	}

	if _, ok := values[key_param]; !ok {
		panic(fmt.Errorf("The value '%s' is not valid for %s", key_param, key))
	}
	values[key_param](web_request)
	return nil, nil
}

type GCodeHelper struct {
	printer *Printer
	gcode   *GCodeDispatch

	is_output_registered bool
	clients              map[*ClientConnection]interface{}
	handle_help          interface{}
	handle_script        interface{}
	handle_restart       interface{}

	handle_firmware_restart interface{}

	handle_subscribe_output interface{}
}

func NewGCodeHelper(printer *Printer) *GCodeHelper {
	self := GCodeHelper{}
	self.printer = printer
	gcode := printer.Lookup_object("gcode", object.Sentinel{})
	//if err != nil {
	//	log.Println(err.Error())
	//}
	self.gcode = gcode.(*GCodeDispatch)
	// Output subscription tracking
	self.is_output_registered = false
	self.clients = map[*ClientConnection]interface{}{}
	// Register webhooks
	webhooks := printer.Lookup_object("webhooks", object.Sentinel{})
	//if err1 != nil {
	//	log.Println(err.Error())
	//}

	wh := webhooks.(*WebHooks)
	wh.Register_endpoint("gcode/help", self._handle_help)
	wh.Register_endpoint("gcode/script", self._handle_script)
	wh.Register_endpoint("gcode/restart", self._handle_restart)
	wh.Register_endpoint("gcode/firmware_restart",
		self._handle_firmware_restart)
	wh.Register_endpoint("gcode/subscribe_output",
		self._handle_subscribe_output)
	//log.Print(wh)
	return &self
}
func (self *GCodeHelper) _handle_help(web_request *WebRequest) (interface{}, error) {
	web_request.Send(self.gcode.Get_command_help())
	return nil, nil
}
func (self *GCodeHelper) _handle_script(web_request *WebRequest) (interface{}, error) {
	defer func() {

		if err := recover(); err != nil {
			msg, ok := err.(string)
			if ok && "exit" == msg {
				//quit to parent
				panic(msg)
			}
			// if _, ok := err.(error); ok {
			// 	msg = err.(error).Error()
			// }
			s := string(debug.Stack())
			value.StaticValue.Error.Println("panic:", sys.GetGID(), err, s)
			if _, ok := err.(error); ok {
				web_request.set_error(err.(error))
			} else {
				web_request.set_error(fmt.Errorf("%v", err))
			}
		}
	}()
	str := web_request.get_str("script", object.Sentinel{})
	//log.Print(str)
	self.gcode.Run_script(str)
	return nil, nil
}
func (self *GCodeHelper) _handle_restart(web_request *WebRequest) (interface{}, error) {
	defer func() {

		if err := recover(); err != nil {
			msg, ok := err.(string)
			if ok && "exit" == msg {
				panic(msg)
			}
			s := string(debug.Stack())
			value.StaticValue.Error.Println("panic:", sys.GetGID(), err, s)
			web_request.is_error = true
			web_request.response = map[string]interface{}{
				"error":   "WebRequestError",
				"message": msg}
		}
	}()
	self.gcode.Run_script("restart")
	return nil, nil
}
func (self *GCodeHelper) _handle_firmware_restart(web_request *WebRequest) (interface{}, error) {
	defer func() {

		if err := recover(); err != nil {
			msg, ok := err.(string)
			if ok && "exit" == msg {
				panic(msg)
			}
			s := string(debug.Stack())
			value.StaticValue.Error.Println("panic:", sys.GetGID(), err, s)
			web_request.is_error = true
			web_request.response = map[string]interface{}{
				"error":   "WebRequestError",
				"message": msg}
		}
	}()
	self.gcode.Run_script("firmware_restart")
	return nil, nil
}
func (self *GCodeHelper) _output_callback(msg string) {
	for cconn, template := range self.clients {
		if cconn.is_closed() {
			self.clients[cconn] = nil
			continue
		}
		tmp := template.(map[string]interface{})
		tmp["params"] = map[string]interface{}{"response": msg}
		cconn.Send(tmp)
	}
}
func (self *GCodeHelper) _handle_subscribe_output(web_request *WebRequest) (interface{}, error) {
	cconn := web_request.get_client_connection()
	template := web_request.get_dict("response_template", map[string]interface{}{})
	self.clients[cconn] = template
	if !self.is_output_registered {
		self.gcode.Register_output_handler(self._output_callback)
		self.is_output_registered = true
	}
	return nil, nil
}

const SUBSCRIPTION_REFRESH_TIME = .25

type QueryStatusHelper struct {
	printer         *Printer
	clients         map[*ClientConnection][]interface{}
	pending_queries []interface{}
	query_timer     *ReactorTimer
	last_query      map[string]interface{}
}

func NewQueryStatusHelper(printer *Printer) *QueryStatusHelper {
	self := QueryStatusHelper{}
	self.printer = printer
	self.clients = map[*ClientConnection][]interface{}{}
	self.pending_queries = []interface{}{}
	self.query_timer = nil
	self.last_query = map[string]interface{}{}
	// Register webhooks
	wh := printer.Lookup_object("webhooks", object.Sentinel{})
	//if err != nil {
	//	log.Println(err.Error())
	//}
	webhooks := wh.(*WebHooks)
	webhooks.Register_endpoint("objects/list", self._handle_list)
	webhooks.Register_endpoint("objects/query", self._handle_query)
	webhooks.Register_endpoint("objects/subscribe", self._handle_subscribe)
	webhooks.Register_endpoint("objects/object_query", self._handle_object_query)
	return &self
}
func (self *QueryStatusHelper) _handle_list(web_request *WebRequest) (interface{}, error) {
	objectList := self.printer.Lookup_objects("")
	objects := []interface{}{}
	for _, o := range objectList {
		if o == "Get_status" {
			objects = append(objects, o)
		}
	}

	web_request.Send(map[string][]interface{}{"objects": objects})
	return nil, nil
}
func (self *QueryStatusHelper) _do_query(eventtime float64) float64 {
	last_query := self.last_query
	self.last_query = map[string]interface{}{}
	query := self.last_query
	msglist := self.pending_queries
	self.pending_queries = []interface{}{}
	for _, v := range self.clients {
		msglist = append(msglist, v)
	}

	// Generate Get_status() info for each client
	for _, querys := range msglist {
		q := querys.([]interface{})
		var cconn *ClientConnection
		if q[0] != nil {
			cconn = q[0].(*ClientConnection)
		}
		var subscription map[string]interface{}
		if q[1] != nil {
			subscription = q[1].(map[string]interface{})
		}
		var send_func func(interface{})
		if q[2] != nil {
			send_func = q[2].(func(interface{}))
		}
		var template map[string]interface{}
		if q[3] != nil {
			template = q[3].(map[string]interface{})
		}
		is_query := cconn == nil
		if !is_query && cconn.is_closed() {
			delete(self.clients, cconn)
			continue
		}
		// Query each requested printer object
		cquery := map[string]interface{}{}
		for obj_name, req_items := range subscription {
			res := query[obj_name]
			if res == nil {
				po := self.printer.Lookup_object(obj_name, nil)
				//if err != nil {
				//	log.Print(err)
				//}
				if po == nil {
					res = map[string]interface{}{}
					query[obj_name] = res
				} else {
					method := reflects.GetMethod(po, "Get_status")
					if method == nil || method.(reflect.Value).IsNil() {
						res = map[string]interface{}{}
						query[obj_name] = res

					} else {
						argv := []reflect.Value{reflect.ValueOf(eventtime)}
						t := method.(reflect.Value).Call(argv)
						if len(t) > 0 {
							res = t[0].Interface()
							query[obj_name] = res

						}
					}
				}
			}
			if req_items == nil {
				var keys []string
				if val, ok := res.(map[string]float64); ok {
					for key, _ := range val {
						keys = append(keys, key)
					}
				} else if val, ok := res.(map[string]string); ok {
					for key, _ := range val {
						keys = append(keys, key)
					}
				} else if val, ok := res.(map[string]interface{}); ok {
					for key, _ := range val {
						keys = append(keys, key)
					}
				}
				req_items = keys
				if req_items != nil && len(req_items.([]string)) != 0 {
					subscription[obj_name] = req_items
				}
			}
			lres := last_query[obj_name]
			if lres == nil {
				lres = map[string]interface{}{}
			}
			//
			cres := map[string]interface{}{}
			keys := req(req_items)
			for _, ri := range keys {
				var rd, obj interface{}
				if val, ok := res.(map[string]float64); ok {
					rd = val[ri]
				} else if val, ok := res.(map[string]string); ok {
					rd = val[ri]
				} else if val, ok := res.(map[string]interface{}); ok {
					rd = val[ri]
				}

				if val, ok := lres.(map[string]float64); ok {
					obj = val[ri]
				} else if val, ok := lres.(map[string]string); ok {
					obj = val[ri]
				} else if val, ok := lres.(map[string]interface{}); ok {
					obj = val[ri]
				}

				if is_query || !reflect.DeepEqual(rd, obj) {
					cres[ri] = rd
				}
			}

			if len(cres) != 0 || is_query {

				cquery[obj_name] = cres
			}
		}
		// Send data
		if len(cquery) != 0 || is_query {
			if template != nil {
				tmp := template
				tmp["params"] = map[string]interface{}{"eventtime": eventtime, "status": cquery}
				send_func(tmp)
			}
		}
	}
	if query == nil {
		//Unregister timer if there are no longer any subscriptions
		reactor := self.printer.Get_reactor()
		reactor.Unregister_timer(self.query_timer)
		self.query_timer = nil
		return constants.NEVER
	}
	return eventtime + SUBSCRIPTION_REFRESH_TIME

}

func req(reqItems interface{}) []string {

	elems := make([]string, 0)
	if items, ok := reqItems.([]interface{}); ok {
		for _, key := range items {
			elems = append(elems, key.(string))
		}

	}

	if items, ok := reqItems.([]string); ok {
		for _, key := range items {
			elems = append(elems, key)
		}
	}
	return elems
}
func (self *QueryStatusHelper) _handle_query(web_request *WebRequest) (interface{}, error) {

	return self.handle_query(web_request, false)
}
func (self *QueryStatusHelper) handle_query(web_request *WebRequest, is_subscribe bool) (interface{}, error) {
	objects := web_request.get_dict("objects", object.Sentinel{})
	// Validate subscription format
	for k, v := range objects {
		if reflect.TypeOf(k).Kind() != reflect.String {
			return nil, errors.New("Invalid argument")
		}
		if v != nil {
			v1 := v.([]interface{})
			for _, ri := range v1 {
				if reflect.TypeOf(ri).Kind() != reflect.String {
					return nil, errors.New("Invalid argument")
				}
			}
		}
	}
	// Add to pending queries
	cconn := web_request.get_client_connection()
	template := web_request.get_dict("response_template", map[string]interface{}{})
	if is_subscribe {
		if _, ok := self.clients[cconn]; ok {
			delete(self.clients, cconn)
		}
	}
	reactor := self.printer.Get_reactor()
	complete := reactor.Completion()
	self.pending_queries = append(self.pending_queries, []interface{}{nil, objects, complete.Complete, map[string]interface{}{}})

	// Start timer if needed
	if self.query_timer == nil {
		qt := reactor.Register_timer(self._do_query, constants.NOW)
		self.query_timer = qt
	}
	// Wait for data to be queried
	msg := complete.Wait(constants.NEVER, nil)
	if msg == nil {
		msg = map[string]interface{}{}
	}
	web_request.Send(msg.(map[string]interface{})["params"])
	if is_subscribe {
		self.clients[cconn] = []interface{}{cconn, objects, cconn.Send, template}
	}
	//log.Print(template)
	return nil, nil
}

func (qsh *QueryStatusHelper) _handle_object_query(web_request *WebRequest) (interface{}, error) {
	objects := web_request.get_dict("objects", object.Sentinel{})
	if len(objects) == 0 {
		return nil, errors.New("objects empty")
	}

	objectNames := make([]string, 0, len(objects))
	for k := range objects {
		objectNames = append(objectNames, k)
	}
	var status = make(map[string]interface{})
	var eventtime = Monotonic()
	for _, name := range objectNames {
		obj := qsh.printer.Lookup_object(name, nil)
		if obj == nil {
			return nil, fmt.Errorf("object: %s not found", name)
		}

		method := reflects.GetMethod(obj, "Get_status")
		if method == nil || method.(reflect.Value).IsNil() {
			return nil, fmt.Errorf("object: %s method Get_status not found", name)
		}

		argv := []reflect.Value{reflect.ValueOf(eventtime)}
		rv := method.(reflect.Value).Call(argv)
		if len(rv) != 1 {
			return nil, fmt.Errorf("object: %s method Get_status return value should only one", name)
		}
		status[name] = rv[0].Interface()
	}

	rt := map[string]interface{}{
		"eventtime": eventtime,
		"status":    status,
	}
	web_request.Send(rt)
	return nil, nil
}

func Monotonic() float64 {
	lib := chelper.Get_ffi()
	return lib.Get_monotonic()
}

func (self *QueryStatusHelper) _handle_subscribe(web_request *WebRequest) (interface{}, error) {
	self.handle_query(web_request, true)
	return nil, nil
}

func Add_early_printer_objects_webhooks(printer *Printer) {
	printer.Add_object("webhooks", NewWebHooks(printer))
	NewGCodeHelper(printer)
	NewQueryStatusHelper(printer)
}
