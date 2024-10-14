package greenlet

import (
	"k3c/common/constants"
	"k3c/common/utils/sys"
	"k3c/common/value"
	"reflect"
	"runtime"
	"runtime/debug"
	"strings"

	//"reflect"
	//"runtime"
	"sync"
)

var lock sync.Mutex
var greenlet sync.Map

func init() {
	//Wait = make(chan uint64)
}

type ReactorGreenlet struct {
	parents []*ReactorGreenlet //parent thread
	Name    string             // thread name
	run     func(interface{}) interface{}
	Timer   interface{}
	GId     uint64
	wg      sync.WaitGroup
	param   interface{}
	ret     interface{}
	state   string
	wgNum   int
	exit    bool
}

func NewReactorGreenlet(run func(interface{}) interface{}) *ReactorGreenlet {
	self := ReactorGreenlet{}
	self.parents = []*ReactorGreenlet{}
	self.exit = false
	self.run = run
	self.Timer = nil
	//test
	self.Name = runtime.FuncForPC(reflect.ValueOf(run).Pointer()).Name()
	index := strings.Index(self.Name, ".")
	if index != -1 {
		self.Name = self.Name[index:]
	}
	//
	self.wgNum = 0
	//self.wg.Add(1)
	self.state = "start"
	//go self.goGreenlet()

	return &self
}
func (self *ReactorGreenlet) goGreenlet() {
	defer func() {
		//处理panic
		if err := recover(); err != nil {
			if !self.exit {
				s := string(debug.Stack())
				value.StaticValue.Error.Println("panic:", sys.GetGID(), err, s)
			}
		}
		//log.Println(self.GId,"quit")
		lock.Lock()
		defer lock.Unlock()
		greenlet.Delete(self.GId) //delete greenlet when task end
		self.state = "end"
		self.GId = 0

		for i := len(self.parents) - 1; i >= 0; i-- {
			//return to parent
			parent := self.parents[i]
			_, ok := greenlet.Load(parent.GId)
			if !ok {
				continue
			}
			if parent.state == "end" {
				continue
			}
			if parent.state == "start" {
				continue
			}
			if parent == self {
				continue
			}
			self.parents = make([]*ReactorGreenlet, 0)
			parent.done()
			return
		}
		self.parents = make([]*ReactorGreenlet, 0)
	}()
	self.GId = sys.GetGID()
	greenlet.Store(self.GId, self)
	self.state = "run"

	//
	if self.run != nil && !self.exit {
		self.ret = self.run(self.param)
	}
}
func (self *ReactorGreenlet) SwitchTo(waketime float64) float64 {
	//if(waketime==constants.NEVER){
	//
	//}else{
	//	t:=time.Duration(int(waketime*100))
	//	time.Sleep(t)
	//
	//}
	//log.Print("ReactorGreenlet SwitchTo ", self.Name, " ", Getcurrent().GId, " >> ", self.GId)
	lock.Lock()
	cSelf := Getcurrent()
	if cSelf == nil {
		//start
		cSelf = &ReactorGreenlet{}
		cSelf.GId = sys.GetGID()
		greenlet.Store(cSelf.GId, cSelf)
		cSelf.state = "run"
	}
	if self == cSelf { //to my self go out
		t := self.ret
		self.ret = waketime
		lock.Unlock()
		if t == nil {
			return constants.NEVER
		} else {
			return t.(float64)
			//return waketime
		}
	}
	if self == nil {
		lock.Unlock()
		if cSelf.ret == nil {
			return constants.NEVER
		} else {
			return cSelf.ret.(float64)
		}
	}
	if len(self.parents) > 0 {
		if self.parents[len(self.parents)-1] != cSelf {
			self.parents = append(self.parents, cSelf)
		}
	} else {
		if len(cSelf.parents) > 0 {
			self.parents = append(self.parents, cSelf.parents[0])
		}
		self.parents = append(self.parents, cSelf)
	}
	if self.GId == 0 {
		//run thread
		self.param = waketime
		go self.goGreenlet()
	} else {
		self.ret = waketime
		self.done()
	}
	cSelf.preWait()
	lock.Unlock()
	cSelf.wait()
	//

	if cSelf.ret == nil {
		return constants.NEVER
	} else {
		return cSelf.ret.(float64)
		//return waketime
	}
}
func (self *ReactorGreenlet) Switch() {
	self.goGreenlet()
}
func (self *ReactorGreenlet) Parent(parent *ReactorGreenlet) {
	if len(self.parents) > 0 {
		self.parents = self.parents[:1]
	}
	self.parents = append(self.parents, parent)
	value.StaticValue.Debug.Print(self.parents)
}
func (self *ReactorGreenlet) done() {
	self.wgNum = self.wgNum - 1
	self.wg.Done()
}
func (self *ReactorGreenlet) preWait() {
	self.wgNum = self.wgNum + 1
	self.wg.Add(1)
	self.state = "wait"

	return
}
func (self *ReactorGreenlet) wait() {

	self.wg.Wait()
	if self.exit {
		//panic(fmt.Sprintf("end %d %s is ",cSelf.GId,cSelf.Name))
		panic("exit")
	}
	self.state = "run"
	//}
	return
}
func Getcurrent() *ReactorGreenlet {
	v, ok := greenlet.Load(sys.GetGID())
	if ok {
		return v.(*ReactorGreenlet)
	} else {
		return nil
	}

}

func Start() {

	return
}
func Close() {
	lock.Lock()
	defer lock.Unlock()
	greenlet.Range(func(key, val interface{}) bool {
		greenlet.Delete(key)
		g := val.(*ReactorGreenlet)
		g.exit = true
		if g.wgNum == 1 {
			g.done()
		}
		return true
	})
	return
}

//type ReactorMutex struct {
//	Reactor      project.IReactor
//	Is_locked    bool
//	Next_pending bool
//	Queue        []*sync.Map
//	Lock         func()
//	Unlock       func(type_, value_, tb interface{})
//}

//func NewReactorMutex(reactor project.IReactor, is_locked bool) *ReactorMutex {
//	self := &ReactorMutex{}
//	self.Reactor = reactor
//	self.Is_locked = is_locked
//	self.Next_pending = false
//	self.Queue = []*sync.Map{}
//	// self.Lock = self.enter
//	// self.Unlock = self.exit
//	return self
//}
//func Test(self *ReactorMutex) bool {
//	return self.Is_locked
//}
//func enter(self *ReactorMutex) {
//	if !self.Is_locked {
//		self.Is_locked = true
//		return
//	}
//	// g := greenlet.
//}
