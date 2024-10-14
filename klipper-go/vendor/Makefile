GO = go
APP = gklib
LIB = libc_helper.so
RUN_OPTIONS = -I /dev/ttyUSB0 -a /home/aston/unix_uds1 ~/printer_data/config/printer.cfg
DLV = dlv
GCC = gcc
ARMROOT = /opt/arm-rockchip830-linux-uclibcgnueabihf
ARMCC = $(ARMROOT)/bin/arm-rockchip830-linux-uclibcgnueabihf-gcc
ARMLIB = $(ARMROOT)/arm-rockchip830-linux-uclibcgnueabihf/sysroot/lib
ARMLIBCHELPER = $(PWD)/chelper/build_chelper_arm.sh
386LIBCHELPER = $(PWD)/chelper/build_chelper_386.sh

.PHONY: build run tools tidy debug clean

build:export GOOS=linux
build:export GOARCH=386
build:export CGO_ENABLED=1
build:export LD_LIBRARY_PATH=$(LIB)
build:export CC=${GCC}
build:
	$(386LIBCHELPER)
	$(GO) build -v -o main/$(APP) ./main

build-arm:export GOOS=linux
build-arm:export GOARCH=arm
build-arm:export CGO_ENABLED=1
build-arm:export LD_LIBRARY_PATH=$(LIB)
build-arm:export CC=${ARMCC}
build-arm:
	$(ARMLIBCHELPER)
	$(GO) build -v -o main/$(APP) ./main

build-armlibchelper:
	$(ARMLIBCHELPER)

build-386libchelper:
	$(386LIBCHELPER)

run:export LD_LIBRARY_PATH=project/chelper
run:
	main/$(APP) $(RUN_OPTIONS)

debug:
	 $(DLV) exec ./main/gklib -- $(RUN_OPTIONS)

tidy:
	$(GO) mod tidy

vendor:
	$(GO) mod vendor

tools:
	$(GO) install github.com/go-delve/delve/cmd/dlv@v1.7.3

clean:
	rm -rf main/$(APP)
	rm -rf project/chelper/$(LIB)

run32: build-386libchelper build-386 run
