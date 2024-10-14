package lock

import (
	"runtime"
	"sync/atomic"
)

// SpinLock 自旋锁对象定义
type SpinLock uint32

const maxBackOff = 32

// Lock 加锁
func (sl *SpinLock) Lock() {
	backoff := 1
	// 自旋尝试获取锁
	for !atomic.CompareAndSwapUint32((*uint32)(sl), 0, 1) {
		for i := 0; i < backoff; i++ {
			runtime.Gosched()
		}
		if backoff < maxBackOff {
			backoff <<= 1
		}

	}
}

// UnLock 解锁
func (sl *SpinLock) UnLock() {
	atomic.CompareAndSwapUint32((*uint32)(sl), 1, 0)
}
