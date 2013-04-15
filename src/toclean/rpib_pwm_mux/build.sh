#!/usr/bin/env sh

. $HOME/repo/lfs/_work_rpib_home/sdk/build/env.sh

BCM2835_DIR=./bcm2835-1.17

$LFS_SDK_CROSS_COMPILE\gcc \
-static -O2 -march=armv6zk -mtune=arm1176jzf-s \
-I$BCM2835_DIR/src \
main.c \
$BCM2835_DIR/src/bcm2835.c \
-lrt
