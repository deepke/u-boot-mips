#!/bin/sh
set -x
if [ -e mymake ];then
    ./mymake vmlinux.bin V=1
else
make ARCH=mips CROSS_COMPILE= /opt/gcc-4.9.3-64-gnu/bin/mips64el-linux- V=1 vmlinux.bin
fi
lzma -c < arch/mips/boot/vmlinux.bin > vmlinux.bin.lzma
load=$(LC_ALL=C readelf -l vmlinux|grep LOAD |grep -o 0x.*|cut -d' ' -f2)
#load=$(printf "0x%08x 0x%08x" $((($load>>32)&0xffffffff)) $(($load&0xffffffff)))
entry=$(LC_ALL=C readelf  -e vmlinux|grep Entry|grep -o 0x.*)
#entry=$(printf "0x%08x 0x%08x" $((($entry>>32)&0xffffffff)) $(($entry&0xffffffff)))
vmlinux=vmlinux.bin.lzma
rootfs=/loongson/buildroot/output.ls2k/images/rootfs.cpio.lzma
dtb=/loongson/ls3a/pmon-loongson3/zloader.ls2k/LS2K.dtb 

cat > multi.its << AAA

/*
 * U-Boot uImage source file with multiple kernels, ramdisks and FDT blobs
 */

/dts-v1/;

/ {
	description = "Various kernels, ramdisks and FDT blobs";
	#address-cells = <1>;

	images {
		kernel-1 {
			description = "vmlinux";
			data = /incbin/("$vmlinux");
			type = "kernel";
			arch = "mips";
			os = "linux";
			compression = "lzma";
			load = <$load>;
			entry = <$entry>;
			hash-1 {
				algo = "md5";
			};
			hash-2 {
				algo = "sha1";
			};
		};


		ramdisk-1 {
			description = "rootfs";
			data = /incbin/("$rootfs");
			type = "ramdisk";
			arch = "loongarch";
			os = "linux";
			compression = "none";
			load = <0xffffffff84000000>;
			entry = <0xffffffff84000000>;
			hash-1 {
				algo = "sha1";
			};
		};


		fdt-1 {
			description = "fdt";
			data = /incbin/("$dtb");
			type = "flat_dt";
			arch = "loongarch";
			compression = "none";
			load = <0xffffffff80100000>;
			entry = <0xffffffff80100000>;
			hash-1 {
				algo = "crc32";
			};
		};
	
		gdb-1 {
			description = "gsbstub";
			data = /incbin/("/loongson/ls3a/pmon-loongson3/zloader.ls2k/gdbstub.bin64");
			type = "fpga";
			arch = "mips";
			os = "linux";
			compression = "none";
			load = <0x80008000>;
			entry = <0x80008000>;
			hash-1 {
				algo = "sha1";
			};
		};

	};

	configurations {
		default = "config-1";

		config-1 {
			description = "ls2k configuration";
			kernel = "kernel-1";
			ramdisk = "ramdisk-1";
			fdt = "fdt-1";
			loadables = "kernel-1", "ramdisk-1", "gdb-1";
		};

	};
};
AAA

mkimage -f multi.its uImage
