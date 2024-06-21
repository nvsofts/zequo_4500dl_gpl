#!/bin/bash
export CROSS_COMPILE_BH=$FULL_TOOLCHAIN_PRERIX
export CROSS_COMPILE=$FULL_TOOLCHAIN_PRERIX
./build.pl -f nand -i spi:nand -b ac3_rd
#./build.pl -f nand -b ac3_db

cp -vf $ROL_BASE_DIR/$UBOOT_PATH/u-boot.bin $ROL_BUILD_DIR/uboot/u-boot_msys.bin
