#!/bin/bash
set -x

CC=arm-rockchip830-linux-uclibcgnueabihf-gcc
opt="-Wall -g -O2 -shared -fPIC -flto -fwhole-program -fno-use-linker-plugin"
rm -rf ./libc_helper.so
$CC $opt -o $PWD/chelper/libc_helper.so $PWD/chelper/*.c
mv $PWD/chelper/libc_helper.so $PWD/project/chelper/