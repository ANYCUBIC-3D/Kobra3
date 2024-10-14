package project

import (
	"encoding/json"
	"k3c/common/value"
	"syscall"
)

type ACEPacketRequest struct {
	Id     int         `json:"id"`
	Method string      `json:"method"`
	Params interface{} `json:"params,omitempty"`
}

type ACEPacketResponse struct {
	Id     int         `json:"id"`
	Code   int         `json:"code"`
	Msg    string      `json:"msg"`
	Result interface{} `json:"result"`
}

type SwitchFilament struct {
	Index int `json:"index"`
}

type UnwindingFilament struct {
	Index  int `json:"index"`
	Length int `json:"length"`
	Speed  int `json:"speed"`
	Mode   int `json:"mode"` // 0 - 普通模式；1 - 增强模式
}

type StopUnwindingFilament struct {
	Index int `json:"index"`
}

type FeedFilament struct {
	Index  int `json:"index"`
	Length int `json:"length"`
	Speed  int `json:"speed"`
}

type UpdateFeedingSpeed struct {
	Index int `json:"index"`
	Speed int `json:"speed"`
}

type UpdateUnwindingSpeed struct {
	Index int `json:"index"`
	Speed int `json:"speed"`
}

type StopFeedFilament struct {
	Index int `json:"index"`
}

type StartFeedAssist struct {
	Index int `json:"index"`
}

type StopFeedAssist struct {
	Index int `json:"index"`
}

type ContinueFilament struct {
	Auto int `json:"auto"`
}

type SetFanSpeed struct {
	FanSpeed int `json:"fan_speed"`
}

type Drying struct {
	Temp     int `json:"temp"`
	FanSpeed int `json:"fan_speed"`
	Duration int `json:"duration"`
}

type DryerStatus struct {
	Status     string `json:"status"`
	TargetTemp int    `json:"target_temp"`
	Duration   int    `json:"duration"`
	RemainTime int    `json:"remain_time"`
}

type RfidStatus int

const (
	RfidNotFound    RfidStatus = 0
	RfidUnRecognize RfidStatus = 1
	RfidRecognized  RfidStatus = 2
	RfidRecognizing RfidStatus = 3
)

type SlotStatus struct {
	Index    int        `json:"index"`
	Status   string     `json:"status"`
	Sku      string     `json:"sku"`
	Type     string     `json:"type"`
	Color    []int      `json:"color"`
	Rfid     RfidStatus `json:"rfid,omitempty"`     // 0 - 未找到rfid信息， 1 - 未能识别， 2 - 已识别
	Source   int        `json:"source,omitempty"`   // 0 - Unknown, 1 - from RFID, 2 - User Defined, 3 - Empty
	Redirect string     `json:"redirect,omitempty"` // 自动续料时重定向其他耗材
}

type GetStatus struct {
	Status          string       `json:"status"` // startup/busy/ready
	Action          string       `json:"action"` // feeding/unwinding
	Temp            int          `json:"temp"`
	Humidity        int          `json:"humidity"`
	EnableRfid      int          `json:"enable_rfid"`
	DryerStatus     DryerStatus  `json:"dryer_status"`
	FeedAssistCount int          `json:"feed_assist_count"`
	ContAssistTime  float64      `json:"cont_assist_time"` // 持续进料时间，单位ms
	Slots           []SlotStatus `json:"slots"`
}

type Info struct {
	Id       int    `json:"id"`
	Slots    int    `json:"slots"`
	SN       string `json:"SN"`
	Date     string `json:"date"`
	Model    string `json:"model"`
	Firmware string `json:"firmware"`
}

type IntRange struct {
	Min int `json:"min"`
	Max int `json:"max"`
}

const (
	Unknown = 0
	RFID    = 1
	Custom  = 2
	Empty   = 3
)

type FilamentInfo struct {
	Id           int        `json:"id,omitempty"`
	Index        int        `json:"index"`
	Sku          string     `json:"sku"`
	Brand        string     `json:"brand"`
	Type         string     `json:"type"`
	Color        []int      `json:"color"`
	ExtruderTemp IntRange   `json:"extruder_temp"`
	HotbedTemp   IntRange   `json:"hotbed_temp"`
	Diameter     float32    `json:"diameter"`
	Rfid         RfidStatus `json:"rfid,omitempty"`   // 0 - 未找到rfid信息， 1 - 未能识别， 2 - 已识别
	Source       int        `json:"source,omitempty"` // 0 - Unknown, 1 - from RFID, 2 - User Defined
}

type SetFilamentInfo struct {
	Index int    `json:"index"`
	Type  string `json:"type"`
	Color []int  `json:"color"`
}

type RefreshFilamentInfo struct {
	Index int `json:"index"`
}

type ACEProto struct {
	last_msg_id uint32
}

func NewACEProto() *ACEProto {
	return &ACEProto{last_msg_id: 0}
}

func (self *ACEProto) calcCrc16(buf []byte) uint16 {
	var _crc uint16 = 0xffff
	for i := 0; i < len(buf); i++ {
		data := uint16(buf[i])
		data ^= _crc & 0xff
		data ^= (data & 0x0f) << 4
		_crc = ((data << 8) | (_crc >> 8)) ^ (data >> 4) ^ (data << 3)
	}
	//value.StaticValue.Debug.Println("calcCrc16: ", _crc)
	return _crc
}

func (self *ACEProto) writeAll(fd int, buf []byte) error {
	sent := 0
	for sent < len(buf) {
		w, err := syscall.Write(fd, buf[sent:])
		if err != nil {
			value.StaticValue.Debug.Printf("write data to fd %d failed, %s", fd, err)
			return err
		}
		sent += w
	}
	return nil
}

func (self *ACEProto) buildPacket(pkt *ACEPacketRequest) []byte {
	var buf []byte
	buf = append(buf, 0xFF)
	buf = append(buf, 0xAA)

	bts, _ := json.Marshal(pkt)
	size := len(bts)

	buf = append(buf, byte(size))
	buf = append(buf, byte(size>>8))

	crc := self.calcCrc16(bts)

	buf = append(buf, bts...)
	buf = append(buf, byte(crc))
	buf = append(buf, byte(crc>>8))
	buf = append(buf, 0xFE)

	return buf
}

func (self *ACEProto) buildPacket_by_buf(data []byte, buf_len uint32) []byte {
	var buf []byte
	buf = append(buf, 0xFF)
	buf = append(buf, 0xAA)

	bts := data
	size := buf_len

	buf = append(buf, byte(size))
	buf = append(buf, byte(size>>8))

	crc := self.calcCrc16(bts)

	buf = append(buf, bts...)
	buf = append(buf, byte(crc))
	buf = append(buf, byte(crc>>8))
	buf = append(buf, 0xFE)

	// value.StaticValue.Debug.Println("sendPacket: ", string(bts))

	return buf
}
