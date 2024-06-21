#!/bin/sh -x

#
## create flash images:
##	jffs2
##	ubifs for flash with erase block size of 256KB and 512KB
#

#
## parameters:
#
## 1. kernel image
## 2. device tree blob
## 3. root_fs
## 4. endianess: big_endian | little_endian
## 5. output directory
## 6. script text file (optional)
##
## example: ./cr_jffs2_img_arm_mv7sft.sh /tftp/zImage /tftp/devtree.dtb /tftp/rootfs_arm-mv7sft \
##                                       [big_endian | little_endian] /tftp/output

usage_exit() {
	@echo
	@echo "Usage: $0 <kernel_image> <dtb_file> <rootfs_dir> <big_endian | little_endian> <output_dir> [script]"
	@echo " "
	@echo "Example: ./cr_jffs2_img_arm_mv7sft.sh zImage devtree.dtb /tftp/rootfs_arm-mv7sft-be big_endian \\"
	@echo "                                       /tftp/output /tftp/output/u-boot-script.txt"
	exit
}

KERNEL=$1
DTB=$2
ROOT_FS=$3
ENDIANESS=$4
OUTPUT_DIR=$5
SCRIPT=$6

#
### create jffs2 image
#

if [ ! -f "${KERNEL}" ]; then
	echo
	echo kernel missing or incorrect
	usage_exit
fi

if [ ! -d "${ROOT_FS}" ]; then
	echo
	echo rootfs_dir missing or incorrect
	usage_exit
fi
if [ "${ENDIANESS}" = "big_endian" ]; then
	endianess="-b"
elif [ "${ENDIANESS}" = "little_endian" ]; then
	endianess="-l"
else
	echo endianess missing or incorrect
	usage_exit
fi

if [ ! -d "${OUTPUT_DIR}" ]; then
	echo
	echo output_dir missing or incorrect
	usage_exit
fi

if [ -n "${SCRIPT}" ]; then
	mkimage -T script -C none -n 'Uboot script' -d ${SCRIPT} ${SCRIPT}.img
fi

echo ">>>>> creating jffs2 image <<<<<"

IMAGE=${OUTPUT_DIR}/jffs2_arm.image
TEMP_IMAGE=${OUTPUT_DIR}/temp_image

rm -f ${IMAGE} ${OUTPUT_DIR}/temp_image ${OUTPUT_DIR}/temp_kernel

mkfs.jffs2 --eraseblock=16KiB ${endianess} -p -n -d ${ROOT_FS} -o ${TEMP_IMAGE}

bzip2 -c ${KERNEL} > ${IMAGE}
bzip2 -c ${DTB} >> ${IMAGE}
bzip2 -c ${TEMP_IMAGE} >> ${IMAGE}

# add the script if required
if [ -f ${SCRIPT}.img ]; then
	bzip2 -c ${SCRIPT}.img >> ${IMAGE}
fi

rm ${TEMP_IMAGE}
echo
echo "file ${IMAGE} is ready"
echo "Use the mtdburn command to burn it to flash "
echo


echo ">>>>> creating 2 ubifs-nand images <<<<<"

# remove prev build outputs
rm -f ${OUTPUT_DIR}/temp_ubi*

# use same ubi config for all ubi images
UBI_CFG=${OUTPUT_DIR}/temp_ubinize.cfg
cat << EOF >  ${UBI_CFG}
[ubifs]
mode=ubi
image=${OUTPUT_DIR}/temp_ubi_rootfs.img
vol_id=0
vol_type=dynamic
vol_name=rootfs_nand
vol_alignment=1
vol_flags=autoresize
EOF

IMAGE=${OUTPUT_DIR}/ubifs_arm_256eb_nand_v2_5.image
rm -f ${IMAGE}

# create ubi image
mkfs.ubifs -r ${ROOT_FS} -m 4096 -e 253952 -c 4096 -R 24 -o ${OUTPUT_DIR}/temp_ubi_rootfs.img -v
ubinize -o ${OUTPUT_DIR}/temp_ubifs_rootfs.img -m 4096  -p 256KiB -s 4096 ${UBI_CFG} -v

# concat kernel zImage, DTB and ubi image
bzip2 -c ${KERNEL} > ${IMAGE}
bzip2 -c ${DTB} >> ${IMAGE}
bzip2 -c ${OUTPUT_DIR}/temp_ubifs_rootfs.img >> ${IMAGE}

# add the script if required
if [ -f ${SCRIPT}.img ]; then
	bzip2 -c ${SCRIPT}.img >> ${IMAGE}
fi

echo
echo "file ${IMAGE} is ready"

IMAGE=${OUTPUT_DIR}/ubifs_arm_512eb_nand.image

# remove prev build outputs
rm -f ${IMAGE} ${OUTPUT_DIR}/temp_ubi_rootfs.img

# create ubi image
mkfs.ubifs -r ${ROOT_FS} -m 4096 -e 516096 -c 4096 -R 24 -o ${OUTPUT_DIR}/temp_ubi_rootfs.img -v
ubinize -o ${OUTPUT_DIR}/temp_ubifs_rootfs.img -m 4096  -p 512KiB -s 4096 ${UBI_CFG} -v

# concat kernel zImage, DTB and ubi image
bzip2 -c ${KERNEL} > ${IMAGE}
bzip2 -c ${DTB} >> ${IMAGE}
bzip2 -c ${OUTPUT_DIR}/temp_ubifs_rootfs.img >> ${IMAGE}

# add the script if required
if [ -f ${SCRIPT}.img ]; then
	bzip2 -c ${SCRIPT}.img >> ${IMAGE}
fi

# cleanup temporary files
rm -f ${OUTPUT_DIR}/temp_ubi*
rm -f ${SCRIPT}.img

echo
echo "file ${IMAGE} is ready"
echo "Use the mtdburn command to burn it to flash "
echo


