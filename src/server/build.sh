#!/usr/bin/env sh

# dependencies
MG_DIR=mongoose
BCM2835_DIR=bcm2835-1.17

$LFS_SDK_CROSS_COMPILE\gcc -Wall -O2 \
main.c pt.c $MG_DIR/mongoose.c $BCM2835_DIR/src/bcm2835.c \
-I. \
-I$MG_DIR \
-I$BCM2835_DIR/src \
-L$LFS_SDK_DEPS_DIR/lib -lpthread -ldl -lrt
