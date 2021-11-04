#!/bin/sh
set -x
if [ -e mymake ];then
    ./mymake vmlinux.bin
else
make ARCH=mips CROSS_COMPILE= /opt/gcc-4.9.3-64-gnu/bin/mips64el-linux- vmlinux.bin
fi
lzma -c < arch/mips/boot/vmlinux.bin > vmlinux.bin.lzma
entry=$(LC_ALL=C readelf  -e vmlinux|grep Entry|grep -o 0x.*)
rootfs=/loongson/buildroot/output.ls2k/images/rootfs.cpio.lzma
dtb=/loongson/ls3a/pmon-loongson3/zloader.ls2k/LS2K.dtb 
/loongson/u-boot-latest/.build/tools/mkimage  -F -A mips -O linux -C lzma -T kernel -a 0xffffffff80200000 -e $entry -n 'Linux-4.19.190' -d vmlinux.bin.lzma -b $dtb -i $rootfs -f auto uImage

