#!/usr/bin/env sh

. $HOME/repo/lfs/_work_rpib_home/sdk/build/env.sh

mkdir -p install/{bin,lib,www}

LFLAGS="-L$LFS_SDK_DEPS_DIR/lib" \
CFLAGS="-I$LFS_SDK_DEPS_DIR/include" \
fCXX=$LFS_SDK_CROSS_COMPILE\g++ \
CC=$LFS_SDK_CROSS_COMPILE\gcc \
PREFIX=$LFS_SDK_DEPS_DIR \
DESTDIR=`pwd`/install make clean install

rm -f install.tar; tar cvf install.tar install
