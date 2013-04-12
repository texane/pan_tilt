#!/usr/bin/env sh

BCM2835_DIR=./bcm2835-1.17

CROSS_COMPILE=$HOME/repo/lfs/_work_rpib/host_install/armv6-rpi-linux-gnueabi/bin/armv6-rpi-linux-gnueabi-
$CROSS_COMPILE\gcc \
-static -O2 -march=armv6zk -mtune=arm1176jzf-s \
-I$BCM2835_DIR/src \
main.c \
$BCM2835_DIR/src/bcm2835.c \
-lrt