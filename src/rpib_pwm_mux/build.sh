#!/usr/bin/env sh

BCM2835_DIR=./bcm2835-1.17

arm-linux-gnueabi-gcc \
-O2 -march=armv6zk -mtune=arm1176jzf-s \
-lrt \
-I$BCM2835_DIR/src \
main.c \
$BCM2835_DIR/src/bcm2835.c
