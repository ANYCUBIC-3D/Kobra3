package project

import (
	"fmt"
	"k3c/common/utils/object"
)

// MCU_counter
type MCU_counter struct {
	mcu        *MCU
	oid        int
	pin        string
	pullup     int
	pollTime   float64
	pollTicks  int64
	sampleTime float64
	callback   interface{}
	lastCount  int64
}

// 定义构造函数，用来初始化MCU_counter
func NewMCUCounter(printer *Printer, pin string, sampleTime float64, pollTime float64) *MCU_counter {
	ppin := printer.Lookup_object("pins", object.Sentinel{})
	pinParams := ppin.(*PrinterPins).Lookup_pin(pin, true, false, nil)
	mcu_counter := &MCU_counter{
		mcu:        pinParams["chip"].(*MCU),
		pin:        pinParams["pin"].(string),
		pullup:     pinParams["pullup"].(int),
		pollTime:   pollTime,
		pollTicks:  0,
		sampleTime: sampleTime,
		callback:   nil,
		lastCount:  0,
	}
	mcu_counter.oid = mcu_counter.mcu.Create_oid()
	mcu_counter.mcu.Register_config_callback(mcu_counter.Build_config)
	return mcu_counter
}

func (self *MCU_counter) Build_config() {
	self.mcu.Add_config_cmd(fmt.Sprintf("config_counter oid=%d pin=%v pull_up=%v", self.oid, self.pin, self.pullup), false, false)
	clock := self.mcu.Get_query_slot(self.oid)
	self.pollTicks = self.mcu.Seconds_to_clock(self.pollTime)
	sampleTicks := self.mcu.Seconds_to_clock(self.sampleTime)
	self.mcu.Add_config_cmd(fmt.Sprintf("query_counter oid=%d clock=%d poll_ticks=%d sample_ticks=%d", self.oid, clock, self.pollTicks, sampleTicks), true, false)
	self.mcu.Register_response(self._handle_counter_state, "counter_state", self.oid)
}

func (self *MCU_counter) Setup_callback(cb interface{}) {
	self.callback = cb
}

func (self *MCU_counter) _handle_counter_state(params map[string]interface{}) error {
	nextClock := self.mcu.Clock32_to_clock64(params["next_clock"].(int64))
	time := self.mcu.Clock_to_print_time(nextClock - self.pollTicks)
	countClock := self.mcu.Clock32_to_clock64(params["count_clock"].(int64))
	countTime := self.mcu.Clock_to_print_time(countClock)
	lastCount := self.lastCount
	deltaCount := int64(int64(params["count"].(int64)-lastCount) & 0xffffffff)
	count := lastCount + deltaCount
	self.lastCount = count
	if self.callback != nil {
		self.callback.(func(float64, int64, float64))(time, count, countTime)
	}
	return nil
}

type FrequencyCounter struct {
	callback  interface{}
	lastTime  float64
	lastCount int64
	freq      float64
	counter   *MCU_counter
}

func NewFrequencyCounter(printer *Printer, pin string, sampleTime float64, pollTime float64) *FrequencyCounter {
	self := &FrequencyCounter{
		callback:  nil,
		lastTime:  0,
		lastCount: 0,
		freq:      0.0,
		counter:   NewMCUCounter(printer, pin, sampleTime, pollTime),
	}
	self.counter.Setup_callback(self._counterCallback)
	return self
}

func (self *FrequencyCounter) _counterCallback(time float64, count int64, countTime float64) {
	if self.lastTime == 0 {
		self.lastTime = time
	} else {
		deltaTime := countTime - self.lastTime
		if deltaTime > 0 {
			self.lastTime = countTime
			deltaCount := count - self.lastCount
			self.freq = float64(deltaCount) / float64(deltaTime)
		} else {
			self.lastTime = time
			self.freq = 0.0
		}
		if self.callback != nil {
			self.callback.(func(float64, float64))(time, self.freq)
		}
	}
	self.lastCount = count
}

func (self *FrequencyCounter) Get_frequency() float64 {
	return self.freq
}
