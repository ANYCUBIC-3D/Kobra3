package queue

import (
	"container/list"
	"sync"
)

type Queue struct {
	rows *list.List
	lock sync.Locker
}

func NewQueue(size int) *Queue {
	self := Queue{}
	self.rows = list.New()
	self.lock = &sync.Mutex{}
	return &self
}
func (self *Queue) Put_nowait(data interface{}) {
	if data == nil {
		return
	}
	self.lock.Lock()
	defer self.lock.Unlock()
	self.rows.PushBack(data)
}

func (self *Queue) Get_nowait() interface{} {
	self.lock.Lock()
	defer self.lock.Unlock()
	front := self.rows.Front()
	if front == nil {
		return nil
	}
	ret := front.Value
	self.rows.Remove(front)
	return ret
}
