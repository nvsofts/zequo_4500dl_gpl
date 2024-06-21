/*******************************************************************************
Copyright (C) Marvell International Ltd. and its affiliates

********************************************************************************
Marvell GPL License Option

If you received this File from Marvell, you may opt to use, redistribute and/or
modify this File in accordance with the terms and conditions of the General
Public License Version 2, June 1991 (the "GPL License"), a copy of which is
available along with the File in the license.txt file or by writing to the Free
Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 or
on the worldwide web at http://www.gnu.org/licenses/gpl.txt.

THE FILE IS DISTRIBUTED AS-IS, WITHOUT WARRANTY OF ANY KIND, AND THE IMPLIED
WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE ARE EXPRESSLY
DISCLAIMED.  The GPL License provides additional details about this warranty
disclaimer.

*******************************************************************************/

#include "common.h"
#include "command.h"
#include "malloc.h"
#include "nand.h"
#include "fs.h"
#include "image.h"
#include "uart/mvUart.h"
#include "common/mvCommon.h"
#include "mvOs.h"

#include "mvRol.h"


#define MV_ROL_FLASH_ROOT               "/mnt/flash"
#define MV_ROL_ACTIVE_IMAGE_FILENAME    "system/images/active-image"
#define MV_ROL_INACTIVE_IMAGE_FILENAME  "system/images/inactive-image"

#define MV_ROL_SD_UPDATE_IMAGE_ENV      "rol_sd_image_name"

#define MV_ROL_SD_DEVICE_ENV            "rol_sd_device"

#define ODM_IMAGE_OFFSET	0x400000
#define ODM_MFG_OFFSET		0xC800000

/* Taken from /local/store/tesla/core/routers/src/lib/file_system/exp/file_system.h - tbd replace with #include */
#define BOOTP_mfg_basename 	            "mfg.bin"
#define BOOTP_mfg_flag_basename	        "mfg_flag"

/* Helper preprocessor functions to stringnify magic value */
#define xstr(s) str(s)
#define str(s) #s

typedef enum {
	MV_ROL_RET_OK,
	MV_ROL_RET_CRC_FAILED
} MV_ROL_RET_ENUM;


extern int do_ubi(cmd_tbl_t * cmdtp, int flag, int argc, char * const argv[]);
extern int do_ubifs_mount(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[]);
extern int do_ubifs_umount(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[]);
extern int do_ubifs_load(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[]);
extern int do_xmodem(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[]);
extern int do_nand(cmd_tbl_t * cmdtp, int flag, int argc, char * const argv[]);
extern uint64_t get_len_incl_bad (nand_info_t *nand, uint64_t offset, const uint64_t length);
extern uint32_t crc32 (uint32_t, const unsigned char *, unsigned int);
extern int do_bootz(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[]);

static int mvRolDebugModeEnable (void);
static int mvRolLoadImageFromFlashOffset(cmd_tbl_t *cmdtp, char *local_args[5], __u32 nand_pagesize, MV_U32 from_offset, MV_U32 to_ram_addr, MV_U32* image_size_ptr);
static MV_ROL_RET_ENUM mvRolLoadMfgFromFlashOffset(cmd_tbl_t *cmdtp, char *local_args[5], __u32 nand_pagesize, MV_U32 from_offset, MV_U32 to_ram_addr);

static int mvRolLabMode = 1; /* Lab mode chosen */
static int mvRolXmodemUsed = 1; /* Running from image loaded by xmodem (not from flash) */
static int mvRolDebug = 1;
static unsigned int mvRolImageaddr = 0xFFFFFFFF; /*Address in RAM where the full Image is stored*/
static char mvRolCurUbootVer[MV_ROL_STRING_LENGTH];
static MV_ROL_IMAGE_PARAM *mvRolImageParam = NULL;
static enum IMAGE_KERNEL_VER mvRolImageKernelVer = image_kernel_ver_unknown;
static MV_U32 mvRolPPtype = 0;

/* user-visible printf */
static int user_printf(const char *format, ...) __attribute__((format(printf,1,2)));
static int user_printf(const char *format, ...) {
	va_list args;
	va_start(args, format);
	
	MV_BOOL silent_mode = mvUartSetSilent(MV_FALSE); 
	int ret = vprintf(format,args);
	mvUartSetSilent(silent_mode);

	va_end(args);
	
	return ret;
}

	/* 
	 * Convert string to hex value
	 */
unsigned int mvRolHtoi (const char *ptr)
{
	unsigned int value = 0;
	char ch = *ptr;
	while (ch == ' ' || ch == '\t')
		ch = *(++ptr);

	for (;;) {
		if (ch < '0') {
			 return value;
		}
		if ((ch >= '0') && (ch <= '9'))
			value = (value << 4) + (ch - '0');
		else if ((ch >= 'A') && (ch <= 'F'))
			value = (value << 4) + (ch - 'A' + 10);
		else if ((ch >= 'a') && (ch <= 'f'))
			value = (value << 4) + (ch - 'a' + 10);
		else
			return value;
		ch = *(++ptr);
	}
}

	/*
	 * Read line from CLI
	 */
static char *getline(void)
{
	static char buffer[100];
	char c;
	size_t i;

	i = 0;
	while (1) {
		buffer[i] = '\0';

		c = getc();

		switch (c) {
		case '\r':	/* Enter/Return key */
		case '\n':
			puts("\n");
			return buffer;

		case 0x03:	/* ^C - break */
			return NULL;

		case 0x5F:
		case 0x08:	/* ^H  - backspace */
		case 0x7F:	/* DEL - backspace */
			if (i) {
				puts("\b \b");
				i--;
			}
			break;

		default:
			/* Ignore control characters */
			if (c < 0x20)
				break;
			/* Queue up all other characters */
			buffer[i++] = c;
			printf("*");
			break;
		}
	}
}

  /*
   * Default calculation:
   * Parse and calculate hash of version string, assuming it's format is X.X.X
   */
unsigned int mvRolVerToInt (char *ptr)
{
	char   *tmp_str = NULL;
	MV_U32  sub_ver[4] = {0} , j, i = 0, ret = 0;
	char    version_str[MV_ROL_STRING_LENGTH];

	strncpy(version_str, ptr, MV_ROL_STRING_LENGTH);

	/* Parse version string */
	tmp_str = strtok(version_str, ".");
	while (tmp_str != NULL) {
		sub_ver[i] = mvRolHtoi(tmp_str);
		tmp_str = strtok(NULL, ".");
		i++;
	}

	/* Calculate hash */
	for(j=0; j<i; j++){
		ret += (sub_ver[i-1-j] << (j * 8));
	}
	return ret;
}

int xmodem_validate_file(void)
{
	printf("************xmodem_validate_file*************\n");
	return 1;
}


	/*
	 * Booton passes a boot string to uboot in a hard-coded offset.
	 * Return parameter value from string by name.
	 */
int mvRolGetParam(const char *param_name)
{
	MV_U32   len, name_len = 0;
	MV_U8   *param_str;
	char    *ptr;

	/* Point to RAM offset where booton stores the boot string */
	param_str = (MV_U8*)(MV_ROL_UBOOT_LOAD_ADRS - MV_ROL_PARAM_OFFSET );
	/*len = strlen((const char *)param_str);*/

	/* Limit boot string size */
	/*if (len > MV_ROL_PARAM_SIZE) {*/
		len = MV_ROL_PARAM_SIZE;
		param_str[len -1] = 0;
	/*}*/
	if (param_name != NULL) {
		name_len = strlen(param_name);
	}
	if ((name_len == 0) ||(len < name_len)) {
		/* search failed */
		return 0;
	} else {
		ptr = strstr((const char *)param_str, param_name);
		if (ptr != NULL) {
			/* param was found (currently all params are numbers) */
			ptr += name_len + 1;
			return (mvRolHtoi((const char *)ptr));
		}
	}
	return 0;
}

	/*
	 * Check whether lab mode has been chosen in booton.
	 * If not, disable u-boot printing by default.
	 * this function is called very early before the uboot is copied
	 */
int mvRolBanner(void)
{
	mvRolLabMode = mvRolGetParam(MV_ROL_LAB_STR);
	mvRolDebug = mvRolLabMode ;

	/*printf("mvRolInitParams: lab mode = %d\n",mvRolLabMode);*/
	if (mvRolDebug == 1) {
		mvUartSetSilent(MV_FALSE);
		printf("mvRolBanner debug:%d ECHO on mvRolLabMode: %d\n",mvRolDebug, mvRolLabMode);
	}

	return 0;
}

int mvRolInitParams(void)
{
	mvRolBanner();
	mvRolXmodemUsed = mvRolGetParam(MV_ROL_XMODEM_STR);
	mvRolImageaddr = 0x1c000000;/*mvRolGetParam(MV_ROL_IMAGE_STR); */
	printf("mvRolInitParams: mvRolXmodemUsed = %d\n",mvRolXmodemUsed);
	printf("mvRolInitParams: mvRolImageaddr = 0x%x\n",mvRolImageaddr);
	if (sizeof(MV_ROL_IMAGE_PARAM) > MV_ROL_PARAM_OFFSET) {
				mvUartSetSilent(MV_FALSE);
		printf("\n***STRUCT MV_ROL_IMAGE_PARAM is too big ***\n");
		return -1;
	}
	/* Init Image param strucure address in RAM */
	mvRolImageParam = (MV_ROL_IMAGE_PARAM*)(mvRolImageaddr - MV_ROL_PARAM_OFFSET);
	mvRolImageParam->lab_mode = mvRolLabMode;

	/* We limit the RAM size we work with to 512M, because we have problmes to tune the system to
	   load with both 256M and 1G */
	if (mvRolImageaddr > MV_ROL_RAM_SIZE_LIMIT) {
		mvRolImageaddr = MV_ROL_RAM_SIZE_LIMIT - MV_ROL_MAX_IMAGE_SIZE;
		printf("mvRolInitParams: mvRolImageaddr was tuned to 0x%x\n",mvRolImageaddr);
	}

	/* Calc ram_size is a bit owkward:
	   address where rolImage should reside, is passed by booton, and placed in mvRolImageaddr. It is ram_size - MAX_IMAGE_SIZE.
	   Hence, we add MAX_IMAGE_SIZE to mvRolImageaddr */
	mvRolImageParam->ram_size = mvRolImageaddr + MV_ROL_MAX_IMAGE_SIZE;
	return 0;
}

	/*
	 * Prepare some environment variables for booting ROL
	 */
void mvRolLateInit(void)
{
	char buf [12];
	char *env;
	char str[256] = {0};

	if (isBoardMsys()) {
        mvRolPPtype = PP_TYPE_MSYS;
        /* Fix bar2 & bar4 size - to be used by Linux */

        MV_MEMIO_LE32_WRITE(0xE0041804, 0x03ff0000);
        MV_MEMIO_LE32_WRITE(0xE0041804, 0x03ff0001);
        MV_MEMIO_LE32_WRITE(0xE0041808, 0x007f0000);
        MV_MEMIO_LE32_WRITE(0xE0041808, 0x007f0001);
	} else {
        strcpy(str, getenv("othbootargs"));
        if (!strstr(str," axp_board"))
        {
			strcat(str, " axp_board");
		}
        setenv("othbootargs", str);
        mvRolPPtype = PP_TYPE_AXP;
    }



	mvRolDebug = mvRolGetParam(MV_ROL_LAB_STR);

	/* Debug mode password protection */
#if 0
~~~no password
	if (mvRolDebug) {
		mvRolLabMode = mvRolDebug = mvRolDebugModeEnable();
	}
#endif

	printf("mvRolLateInit debug:%d mvRolLabMode:%d \n", mvRolDebug, mvRolLabMode);
	memset(buf,0,12);
	sprintf(buf, "%x", MV_ROL_UIMAGE_EXEC_ADRS);
	env = getenv("linux_loadaddr");
	if (!env)
		setenv ("linux_loadaddr", buf);
	env = getenv("loadaddr");
	if (!env)
		setenv ("loadaddr", buf);
	env = getenv("imageaddr");
	if (!env){
		memset(buf,0,12);
		sprintf(buf, "%x", mvRolImageaddr);
		setenv ("imageaddr", buf);
	}
	/*setenv ("othbootargs","ubi.mtd=2");*/
	/*setenv ("bootargs", "root=/dev/ram0 rw rootfstype=ramfs rdinit=/sbin/init ip=$(ipaddr):$(serverip)::$(netmask):$(hostname):$(netdev):off console=$(consoledev),$(baudrate) $(othbootargs) $(linux_parts)");*/
	/*setenv ("bootargs", "maxcpus=1 root=/dev/nfs rw nfsroot=$serverip:$rootpath ip=$ipaddr:$serverip:$gatewayip:$netmask:$hostname:$netdev:off $console $othbootargs $mtdparts loglevel=$loglevel earlyprintk=serial,uart0,115200 $mvNetConfig; tftp $linux_loadaddr $image_name;bootm $linux_loadaddr");*/
	/*setenv ("run_image", "bootm $(linux_loadaddr)");*/
	/* ubi parameters for uboot */
	/*setenv ("mtdids", "nand0=armada-nand");*/
	/*setenv("mtdparts", "mtdparts=armada-nand:4m(uboot),-(rootfs)");*/
	/*setenv ("sa_ramfs", "setenv bootargs maxcpus=1 root=/dev/ram0 rw rootfstype=ramfs rdinit=/sbin/init ip=$ipaddr:$serverip:$gatewayip:$netmask:$hostname:$netdev:off $console $othbootargs $mtdparts imageaddr=$imageaddr loglevel=$loglevel earlyprintk=serial,uart0,115200 $mvNetConfig; tftp $linux_loadaddr $image_name;bootm $linux_loadaddr");*/
	/*setenv ("nfsboot", "setenv bootargs maxcpus=1 root=/dev/nfs rw nfsroot=$serverip:$rootpath ip=$ipaddr:$serverip:$gatewayip:$netmask:$hostname:$netdev:off $console $othbootargs $mtdparts imageaddr=$imageaddr loglevel=$loglevel earlyprintk=serial,uart0,115200 $mvNetConfig; tftp $linux_loadaddr $image_name;bootm $linux_loadaddr");*/
	/*setenv ("quiet", "1");*/
	/*setenv ("rol_image_name", "image1.bin");*/
	mvRolImageParam->boot_mode = MV_ROL_BOOT_MODE_NFS;
	mvRolImageParam->image_pattern = MV_ROL_IMAGE_PATTERN;

	/* Update image name per MSYS target, only if was not saved previously */
	/* if (((env = getenv("rol_image_name")) == NULL) || (strcmp(env,"image1_msys.bin") != 0)) {
		setenv ("rol_image_name", "image1_msys.bin");
		saveenv();
	} */
	strcpy(mvRolCurUbootVer, CONFIG_PRIVATE_STRING);
	if (mvRolDebug == 0) {
		setenv("bootdelay", "0");
		setenv("loglevel", "0");
		/*setenv("bootcmd","ubi part rootfs 2048;setenv bootargs maxcpus=1 root=/dev/ram0 rw rootfstype=ramfs rdinit=/sbin/init $console $othbootargs $mtdparts loglevel=$loglevel earlyprintk=serial,uart0,115200 $mvNetConfig; setenv ethact egiga1;rolimage");*/

	} else {
		/*setenv("upd_image", "tftp 0x1c000000 image1.bin");*/
		/*setenv("upd_booton", "tftp $linux_loadaddr booton.bin ;nand erase 0x0 0x20000; nand write.e 0x2000000 0x0 0x20000");*/
		/*setenv("upd_uboot", "tftp $linux_loadaddr ros_uboot.bin; nand erase 0x60000 0x140000; nand write.e 0x2000000 0x60000 0x140000; nand erase 0x1a0000 0x140000; nand write.e 0x2000000 0x1a0000 0x140000");*/
		setenv("loglevel", "7");
		setenv("bootdelay", "5");
		/*setenv("bootcmd","ubi part rootfs 2048;setenv bootargs maxcpus=1 root=/dev/nfs rw nfsroot=$serverip:$rootpath ip=$ipaddr:$serverip:$gatewayip:$netmask:$hostname:$netdev:off $console $othbootargs $mtdparts loglevel=$loglevel earlyprintk=serial,uart0,115200 $mvNetConfig;rolimage");*/
		/*setenv("rol_tftp","ubi part rootfs 2048;ubi create rootfs;setenv bootargs maxcpus=1 root=/dev/ram0 rw rootfstype=ramfs rdinit=/sbin/init ip=$ipaddr:$serverip:$gatewayip:$netmask:$hostname:$netdev:off $console $othbootargs $mtdparts loglevel=$loglevel earlyprintk=serial,uart0,115200 $mvNetConfig; setenv ethact egiga1; tftp $imageaddr $rol_image_name; rolimage_tftp ");*/
	}
}
static int mvRolCheckCrc(MV_U32 crc, MV_U8 *buf, MV_U32 size)
{
	MV_U32 calc_crc = 0;
	if (buf ==NULL) {
		return -1;
	}
	calc_crc = crc32(calc_crc, buf, size);
	printf("calc_crc 0x%x buf 0x%x size %d\n",calc_crc, (int)buf, size );
	if (calc_crc == crc) {
		return 0;
	}
	else
		return -1;
}

/* mvRolCheckUbootCrc -  load existing uboot from FLASH into RAM and check CRC */
static int mvRolCheckUbootCrc(cmd_tbl_t *cmdtp, char *local_args[5], MV_U32 ram_addr, MV_U32 uboot_offset)
{
	int     status;
	MV_U32  crc = 0, uboot_size, header_size;

	/* Syntax is:
	 *   0    1        2     3    4
	 *   nand read  [addr off size]*/
	memset(local_args[1], 0, 12);
	sprintf(local_args[1], "%s", "read");
	memset(local_args[2], 0, 12);
	sprintf(local_args[2], "%x", ram_addr);
	memset(local_args[3], 0, 12);
	sprintf(local_args[3], "%x", uboot_offset);
	memset(local_args[4], 0, 12);
	sprintf(local_args[4], "%x", MV_ROL_UBOOT_SIZE);

	status = do_nand(cmdtp, 0, 5, local_args);
	if (status == 0 ) { /* read was ok */
		/* check file CRC based on values from image header */
		crc = MV_BYTE_SWAP_32BIT(((MV_ROL_UBOOT_HEADER*)ram_addr)->uboot_crc);
		uboot_size = MV_BYTE_SWAP_32BIT(((MV_ROL_UBOOT_HEADER*)ram_addr)->uboot_size);
		header_size = MV_BYTE_SWAP_16BIT(((MV_ROL_UBOOT_HEADER*)ram_addr)->header_size);

		printf("mvRolCheckUbootCrc: version %s size %d crc 0x%x\n",((MV_ROL_UBOOT_HEADER*)ram_addr)->uboot_version, uboot_size + header_size, crc);
		((MV_ROL_UBOOT_HEADER*)ram_addr)->uboot_crc = 0;
		status = mvRolCheckCrc(crc, (MV_U8 *)ram_addr, uboot_size + header_size);
		((MV_ROL_UBOOT_HEADER*)ram_addr)->uboot_crc = MV_BYTE_SWAP_32BIT(crc);
		if (status == 0) { /* crc is correct - file is good */
			printf("mvRolCheckUbootCrc: crc is correct\n");
		}
	}
	printf("mvRolCheckUbootCrc: check uboot file at address 0x%x status %d\n", uboot_offset, status);
	return status;
}

static int mvRolEraseUboot(cmd_tbl_t *cmdtp, char *local_args[5], MV_U32 uboot_offset)
{
	printf("mvRolEraseUboot: erase FLASH 0x%x size 0x%x\n", uboot_offset, MV_ROL_UBOOT_SIZE);

	/* Syntax is:
	 *   0    1        2     3    4
	 *   nand erase  [off size] */
	memset(local_args[1], 0, 12);
	sprintf(local_args[1], "%s", "erase");
	memset(local_args[2], 0, 12);
	sprintf(local_args[2], "%x", uboot_offset);
	memset(local_args[3], 0, 12);
	sprintf(local_args[3], "%x", MV_ROL_UBOOT_SIZE);
	return do_nand(cmdtp, 0, 4, local_args);
}

/* mvRolWriteUboot - write file from RAM to FLASH.
   before writing we check that there is enough place including bad sectors */
static int mvRolWriteUboot(cmd_tbl_t *cmdtp, char *local_args[5], MV_U32 ram_addr, MV_U32 uboot_offset, MV_U32	file_size)
{
	MV_U32  total_size;
	int     status;

	printf("mvRolWriteUboot: copy from RAM 0x%x to FLASH 0x%x size %d\n", ram_addr, uboot_offset, file_size);
	total_size = get_len_incl_bad(&nand_info[0], uboot_offset, file_size);
	if (total_size > MV_ROL_UBOOT_SIZE) {
		printf("mvRolWriteUboot: Can't write file too many bad blocks\n");
		return -1;
	}
	/* Syntax is:
	 *   0    1        2     3    4
	 *   nand write  [addr off size]*/
	memset(local_args[1], 0, 12);
	sprintf(local_args[1], "%s", "write");
	memset(local_args[2], 0, 12);
	sprintf(local_args[2], "%x", ram_addr);
	memset(local_args[3], 0, 12);
	sprintf(local_args[3], "%x", uboot_offset);
	memset(local_args[4], 0, 12);
	sprintf(local_args[4], "%x", file_size);
	status = do_nand(cmdtp, 0, 5, local_args);
	if (status != 0)
		printf("mvRolWriteUboot: Can't write \n");

	return status;
}

/* mvRolCopyUboot - when updating new uboot we need to do:
   1) erase old uboot.
   2) write new uboot from RAM to FLASH.
   3) check new uboot integrity by coping it to RAM and checking crc */
int mvRolCopyUboot(cmd_tbl_t *cmdtp, char *local_args[5], MV_U32 ram_addr, MV_U32 uboot_offset, MV_U32	file_size)
{
	int status;

	mvRolEraseUboot(cmdtp, local_args, uboot_offset);
	status = mvRolWriteUboot(cmdtp, local_args, ram_addr, uboot_offset, file_size);
	if (status == 0) {
		/* check that the new file is correct */
		status = mvRolCheckUbootCrc(cmdtp, local_args, MV_ROL_UBOOT_CHECK_ADRS, uboot_offset);
	}
	return status;
}
/* Get Uimage size and offset from Image file in RAM */
MV_BOOL rolGetUimageParam(MV_U32 image_ram_addr, MV_U32 *uimage_size, MV_U32 *uimage_offset)
{
	MV_U32 file_cnt;
	MV_U32 num_of_files, size, offset;
	MV_ROL_IMAGE_FILE_STC *image_file_ptr;
	MV_ROL_IMAGE_HEADER *image_header = ((MV_ROL_IMAGE_HEADER*)image_ram_addr);;

    if (strncmp((char*)(image_header->image_magic), xstr(MV_ROL_PROJECT_MAGIC), MV_ROL_IMAGE_MAGIC_LENGTH)) {
        printf("\nInvalid magic %s found (right image file?)\n", image_header->image_magic);
        return MV_FALSE;
    }

    num_of_files = MV_32BIT_BE(image_header->num_of_files);
	/* Per each file, the header stores a desc with name, size and offset */
	for (file_cnt = 0; file_cnt < num_of_files; file_cnt++) {
		image_file_ptr = (MV_ROL_IMAGE_FILE_STC *)(image_ram_addr + sizeof(MV_ROL_IMAGE_HEADER) + file_cnt * sizeof(MV_ROL_IMAGE_FILE_STC));
		size = MV_32BIT_BE(image_file_ptr->file_size);
		offset = MV_32BIT_BE(image_file_ptr->file_offset);
		if (strncmp((const char*)image_file_ptr->file_name, "uimage", sizeof("uimage"))==0){
			*uimage_offset = offset;
			*uimage_size = size;
            printf("uimage_offset: %d, uimage_size: %d", offset, size);
            return MV_TRUE;
		}
	}
	return MV_FALSE;
}

/**************************************************************************
* rolCopyFile
*
* Params: 	dst -
*			src	-
*			size
*
**************************************************************************/
MV_BOOL rolCopyFile(MV_U32 dst, MV_U32 src, MV_U32 size)
{
	MV_U32 i;

    printf("%s: dst 0x%x src 0x%x size %u\n", __FUNCTION__, dst, src, size);
    /* copy file */
    for (i = 0; i < size; i = i +4) {
        *(unsigned int *)(dst + i) = *((unsigned int *)(src + i));
    }
    printf("crc: %x\n", crc32(0, (void*)dst, size));

    return MV_TRUE;
}

	/*
	 * Read header of ROL image in RAM, make sure u-boot version in flash
	 * is compatible with kernel. If not - replace it with the one in the image.
	 */
MV_BOOL rolCheckImage(cmd_tbl_t *cmdtp, char *local_args[5], MV_U32 file_size, MV_U32 image_ram_addr)
{
	MV_U32                 header_crc, header_size, header_version, size, file_cnt;
	MV_U32                 image_size, image_crc, num_of_files, uboot_size, uimage_size;
	MV_U32                 uboot_offset = 0, uimage_offset = 0, offset;
    MV_U32                 DTB_offset = 0, DTB_size = 0;
	MV_ROL_IMAGE_HEADER   *image_header = ((MV_ROL_IMAGE_HEADER*)image_ram_addr);;
	MV_U8                  image_magic[MV_ROL_IMAGE_MAGIC_LENGTH+1]= {0};
	MV_ROL_IMAGE_FILE_STC *image_file_ptr;
    struct MV_ROL_FILE_STC *file_ptr;
	int                    status;

	header_size = MV_16BIT_BE(image_header->header_size);
	header_version= MV_16BIT_BE(image_header->header_version);
	header_crc = MV_32BIT_BE(image_header->header_crc);
	image_size= MV_32BIT_BE(image_header->image_size);
	image_crc = MV_32BIT_BE(image_header->image_crc);
	num_of_files = MV_32BIT_BE(image_header->num_of_files);
	memcpy(image_magic, image_header->image_magic, MV_ROL_IMAGE_MAGIC_LENGTH);
    mvRolImageKernelVer = (header_version<=1) ? image_kernel_ver_3_4 : image_kernel_ver_3_10;

	/* Validating file size */
	if (image_size != file_size) {
		printf("rolCheckImage: Invalid image size %d vs file size %d\n",image_size, file_size);
		return MV_FALSE;
	}
	printf("Current Uboot version: %s [0x%x]\n",mvRolCurUbootVer, mvRolVerToInt(mvRolCurUbootVer));
	printf("Image header: image_version %s\n\timage_time %s\n\theader_size %d\n\theader_version %d\n\theader_crc 0x%x\n\timage_size %d\n\timage_magic %s\n\timage_crc 0x%x\n\tminimum_uboot_version %s [0x%x]\n\tnum_of_files %d\n",
	image_header->image_version,
	image_header->image_time,
	header_size,
	header_version,
	header_crc,
	image_size,
	image_magic,
	image_crc,
	image_header->minimum_uboot_version, mvRolVerToInt((char *)image_header->minimum_uboot_version),
	num_of_files);

    /* Per each file, the header stores a desc with name, size and offset.
       We use reserved names (aka "uboot", "uimage") and not file_type (name[27]), because files_type
       is relevant only from Tesla 2.3.5
    */
	for (file_cnt = 0; file_cnt < num_of_files; file_cnt++) {
		image_file_ptr = (MV_ROL_IMAGE_FILE_STC *)(image_ram_addr + sizeof(MV_ROL_IMAGE_HEADER) + file_cnt * sizeof(MV_ROL_IMAGE_FILE_STC));
		size = MV_32BIT_BE(image_file_ptr->file_size);
		offset = MV_32BIT_BE(image_file_ptr->file_offset);
		printf("\t  file: %12s\t size: %d\t offset: %d\t crc: %x\n",image_file_ptr->file_name, size, offset, 
               crc32(0, image_ram_addr + offset, size) );
		if (strncmp((const char*)image_file_ptr->file_name, "uimage", sizeof("uimage"))==0){
			uimage_offset = offset;
			uimage_size = size;
		} else if(strncmp((const char*)image_file_ptr->file_name, "uboot", sizeof("uboot"))==0) {
			uboot_offset = offset;
			uboot_size = size;
		} else if(mvRolImageKernelVer == image_kernel_ver_3_10) {
            file_ptr = (struct MV_ROL_FILE_STC *)image_file_ptr;
            if ((file_ptr->type == included_file_type_dtb) && (MV_32BIT_BE(file_ptr->compt_bmp) & mvRolPPtype) ) {
                printf("DTB file matches (for CPU type %d)\n", mvRolPPtype);
                DTB_offset = MV_32BIT_BE(file_ptr->file_offset);
                DTB_size = MV_32BIT_BE(file_ptr->file_size);
            }
        }
	}

	if (uboot_offset == 0) {
		printf("rolCheckImage: can't find uboot file\n");
		return MV_FALSE;
	}

	if (mvRolVerToInt(mvRolCurUbootVer) <  mvRolVerToInt((char *)image_header->minimum_uboot_version)) {
	/*
	 * Image does not support the current uboot
	 * Securely replace u-boot copies in flash:
	 * Find a valid copy, replace the other copy, then replace the valid one.
	 */
		status = mvRolCheckUbootCrc(cmdtp, local_args, MV_ROL_UIMAGE_LOAD_ADRS, MV_ROL_UBOOT_1_OFFSET);
		if (status != 0) { /* uboot 1 is invalid - start to update it */
			status = mvRolCopyUboot(cmdtp, local_args, (image_ram_addr + uboot_offset), MV_ROL_UBOOT_1_OFFSET, uboot_size);
			if (status == 0) {/* update the second uboot */
				status = mvRolCopyUboot(cmdtp, local_args, (image_ram_addr + uboot_offset), MV_ROL_UBOOT_2_OFFSET, uboot_size);
			} else {
				printf("rolCheckImage: can't update uboot 1\n");
				return MV_FALSE;
			}
		} else { /* uboot 1 is valid - start to update from uboot 2 */
			status = mvRolCopyUboot(cmdtp, local_args, (image_ram_addr + uboot_offset), MV_ROL_UBOOT_2_OFFSET, uboot_size);
			if (status == 0) {/* update the first uboot */
				status = mvRolCopyUboot(cmdtp, local_args, (image_ram_addr + uboot_offset), MV_ROL_UBOOT_1_OFFSET, uboot_size);
			} else {
				printf("rolCheckImage: can't update uboot 2\n");
				return MV_FALSE;
			}
		}
		/* Reinit board to run initialization code again */
		do_reset (NULL, 0, 0, NULL);
	}

    if (uimage_offset == 0) {
        printf("%s: can't find uimage file\n", __FUNCTION__);
        return MV_FALSE;
    }

    /* Copy uImage file */
    if (MV_FALSE == rolCopyFile(MV_ROL_UIMAGE_LOAD_ADRS, image_ram_addr + uimage_offset, uimage_size))
        return MV_FALSE;
    /* Copy DTB file - only if LK 3.10 is used */
    if (mvRolImageKernelVer == image_kernel_ver_3_10) {
        if (!DTB_offset) {
            printf("%s: can't find compatible DTB file\n", __FUNCTION__);
            return MV_FALSE;
        }
        return rolCopyFile(MV_ROL_UIMAGE_LOAD_ADRS + uimage_size, image_ram_addr + DTB_offset, DTB_size);
    } else
        return MV_TRUE;
}


char * rolStrnstr(const char * s1,const char * s2,int size)
{
	int l1, l2;

	l2 = strlen(s2);
	if (!l2)
		return (char *) s1;
	/*l1 = strlen(s1);
	if (l1 > size) {*/
		l1 = size;
	/* }*/
	while (l1 >= l2) {
		l1--;
		if (!memcmp(s1,s2,l2))
			return (char *) s1;
		s1++;
	}
	return NULL;
}

/*  1) Read Image name from image-list file
	2) copy Image file from FLASH to RAM at a given address
	3) check that Image is valid
*/
MV_BOOL rolCopyImageToRam(cmd_tbl_t *cmdtp, char *local_args[5], MV_U32 image_type, MV_U32 image_ram_addr)
{
	MV_BOOL result;
	int     ret_val;
	MV_U32  image_size = 0;
	char   *s;

	switch (image_type) {
	case MV_ROL_IMAGE_TYPE_ACTIVE:
		strcpy(local_args[2], MV_ROL_ACTIVE_IMAGE_FILENAME);
		break;
	case MV_ROL_IMAGE_TYPE_INACTIVE:
		strcpy(local_args[2], MV_ROL_INACTIVE_IMAGE_FILENAME);
		break;
	default:
		printf("rolCopyImageToRam: Invalid image type\n");
		return MV_FALSE;
	}

	memset(local_args[1], 0, 12);
	sprintf(local_args[1], "%x", image_ram_addr);

	ret_val = do_ubifs_load(cmdtp, 0, 3, local_args);
	if ((s = getenv("filesize")) != NULL) {
		image_size = simple_strtoul(s, NULL, 16);
	}

	if ((ret_val != 0) || (image_size == 0)) { /* image not exist */
		printf("Image file %s does not exist %d\n", local_args[2], ret_val);
		result = MV_FALSE;
	} else { /* image-list OK */
		result = rolCheckImage(cmdtp, local_args, image_size, image_ram_addr);
	}
	return result;
}

static MV_BOOL rolGetNandPageSize(u_int32_t *pageSize)
{
	int i;

	for (i = 0; i < CONFIG_SYS_MAX_NAND_DEVICE; i++)
		if (nand_info[i].name)
			break;
	if (i == CONFIG_SYS_MAX_NAND_DEVICE) /* No NAND found - Should never happen */
		return MV_FALSE;

	*pageSize = nand_info[i].writesize;
	return MV_TRUE;
}

/* Looks in flash for MFG flag.
   If found, tries to read MFG image to RAM */
MV_BOOL rolCopyMFG(cmd_tbl_t *cmdtp, MV_U32 image_ram_addr)
{
	MV_U32  image_size = 0;
	char   *s;
	int     ret_val;
	MV_BOOL result = MV_TRUE;
	char buf1[12], buf2[12];
	char* local_args[3] = {NULL, buf1, buf2}; /* do_ubifs_load requires minimum of 3 args */

	snprintf(buf1, 12, "%x", image_ram_addr);
	strncpy(buf2, BOOTP_mfg_flag_basename, 12);

	ret_val = do_ubifs_load(cmdtp, 0, 3, local_args);

	if (ret_val != 0) {
		printf("mfg flag not found\n");
		return MV_FALSE;
	}

	strcpy(local_args[2], BOOTP_mfg_basename);
	ret_val = do_ubifs_load(cmdtp, 0, 3, local_args);
	if ((s = getenv("filesize")) != NULL) {
		image_size = simple_strtoul(s, NULL, 16);
	}

	if ((ret_val != 0) || (image_size == 0)) { /* MFG does not exist */
		printf("Failed loading mfg image\n");
		result = MV_FALSE;
	}

	return result;
}


/* 1) read from flash the image list and get the active image
   2) can't read image list --> tries to load image to RAM from the ODM (pre-defined) flash location */
MV_BOOL rolCopyImage(cmd_tbl_t *cmdtp, char *local_args[5], MV_U32 image_ram_addr, MV_BOOL is_ubi_mounted)
{
	MV_BOOL result, silentMode;
	MV_U32  image_size = 0;
	u_int32_t	nandPageSize;
	int     status;
	char*	do_nand_args[3] = {NULL, "erase.part", "rootfs"};
	char*	ubi_args_part[3] = {NULL, "part", "rootfs"};
	char*	ubi_args_create[3] = {NULL, "create", "rootfs"};


	if (is_ubi_mounted) {
		silentMode = mvUartSetSilent(MV_FALSE);
		printf("Loading %s ...\n", MV_ROL_ACTIVE_IMAGE_FILENAME);
		mvUartSetSilent(silentMode);
		result = rolCopyImageToRam(cmdtp, local_args, MV_ROL_IMAGE_TYPE_ACTIVE, image_ram_addr);
		if (result == MV_FALSE) {
			silentMode = mvUartSetSilent(MV_FALSE);
			printf("Failed loading %s !\n", MV_ROL_ACTIVE_IMAGE_FILENAME);
			printf("Loading %s ...\n", MV_ROL_INACTIVE_IMAGE_FILENAME);
			mvUartSetSilent(silentMode);
			/* check secondary image */
			mvRolImageParam->boot_mode = MV_ROL_BOOT_MODE_SECONDARY;
			result = rolCopyImageToRam(cmdtp, local_args, MV_ROL_IMAGE_TYPE_INACTIVE, image_ram_addr);
			if (result == MV_FALSE) {
				silentMode = mvUartSetSilent(MV_FALSE);
				printf("Failed loading %s !\n", MV_ROL_INACTIVE_IMAGE_FILENAME);
				mvUartSetSilent(silentMode);
			}
		}
	} else
		result = MV_FALSE;

	if (result == MV_FALSE) {
		if (rolGetNandPageSize(&nandPageSize) == MV_FALSE)
			return MV_FALSE; /* No NAND found - Should never happen */

		/* Check if IMAGE was placed in flash. If not - continue with flow */
		if (mvRolLoadImageFromFlashOffset(cmdtp, local_args, nandPageSize, ODM_IMAGE_OFFSET, image_ram_addr, &image_size)) {
			mvUartSetSilent(MV_FALSE);

			/* Reinit board to run initialization code again */
			do_reset (NULL, 0, 0, NULL);
		} else {
			/* Check if MFG was placed in flash. If not - continue with flow */
			if (MV_ROL_RET_CRC_FAILED ==
					mvRolLoadMfgFromFlashOffset(cmdtp, local_args, nandPageSize, ODM_MFG_OFFSET, image_ram_addr - MFG_RAM_OFFSET_BEFORE_ROL_IMAGE)
			   )
				do_reset (NULL, 0, 0, NULL); /* Might load correctly from flash next time */

			/* Create UBI vol rootfs */
			status = do_nand(cmdtp, 0, 3, do_nand_args);
			if (status == 0)
				status = do_ubi(cmdtp, 0, 3, ubi_args_part);
			if (status == 0)
				status = do_ubi(cmdtp, 0, 3, ubi_args_create);
			if (status == 0)
				result = rolCheckImage(cmdtp, local_args, image_size, image_ram_addr);

			mvRolImageParam->boot_mode = MV_ROL_BOOT_MODE_XMODEM;
			result = (status ? MV_FALSE : MV_TRUE);
		}
	}
	printf("rolCopyImage: End %d\n",result);
	return result;
}

static int rolImage_boot(cmd_tbl_t *cmdtp, int boot_mode) {
	char addr[16];
	char *args[2] = { "", addr };

    mvRolImageParam->boot_mode = boot_mode;
	
	sprintf(addr, "%x", MV_ROL_UIMAGE_EXEC_ADRS);		
	return do_bootm(cmdtp, 0, sizeof(args)/sizeof(*args), args);
}

static int fs_read_file_from_device(const char *ifname, const char *dev_part_str, int fstype,
									const char *filename, ulong addr, int offset, int len) {
	printf("Setting up block device[%s:%s]:%d \n",ifname,dev_part_str,fstype);
	if (fs_set_blk_dev(ifname, dev_part_str, fstype) < 0) {
		user_printf("Failed setting up block device[%s:%s]:%d \n",ifname,dev_part_str,fstype);
		return -1;
	}
	
	printf("Reading file[%s]\n",filename);
	int file_len = fs_read(filename, addr, offset, len);
	if(len < 0) {
		user_printf("Failed reading file[%s]\n",filename);
		return -2;
	}
	
	return file_len;
}

static int read_file_from_sd(int fstype, const char *filename, ulong addr, int offset, int len) {
	const char *device_str = getenv(MV_ROL_SD_DEVICE_ENV);
	if(!device_str) {
		user_printf("Environment variable for sd device not specified[%s]\n",device_str);
		return -1;
	}
	char *device_str_copy = strdup(device_str);
	if(!device_str_copy) {
		user_printf("Memory allocation for sd config string failed\n");
		return -2;
	}
	
	char *ifname = strtok(device_str_copy," ");
	char *dev_part_str = strtok(0, " ");
	
	int ret = fs_read_file_from_device(ifname,dev_part_str,fstype,filename,addr,offset,len);    
	
	free(device_str_copy);
	
	return ret;
}

int do_rolImage_cmd ( cmd_tbl_t *cmdtp, int flag, int argc,  char * const argv[])
{
	int     ret_val;
	char 	*local_args[5];
	char buf1[12], buf2[MV_ROL_IMAGE_NAME_SIZE], buf3[12], buf4[12];
	MV_BOOL result = MV_TRUE;
	MV_U32 uImage_size, offset, num_of_files, file_cnt, header_version;
    MV_U32 DTB_offset = 0, DTB_size = 0;
    MV_ROL_IMAGE_HEADER *image_header = (MV_ROL_IMAGE_HEADER*)mvRolImageaddr;
    struct MV_ROL_FILE_STC *file_ptr;

	printf("do_rolImage_cmd: start\n");
	if (argc != 1) {
		printf ("Usage:\n%s\n", cmdtp->usage);
		return 1;
	}

	printf("Initializing ROL image ...\n");
	/* 1) check if uimage is already placed in ram (xmodem was used)
	   1) get active image
	   2) check if image active
	   3) if not get backup*/
	/* run image from RAM */
	local_args[0] = argv[0];
	local_args[1] = buf1;
	local_args[2] = buf2;
	local_args[3] = buf3;
	local_args[4] = buf4;
	mvRolImageParam->boot_mode = MV_ROL_BOOT_MODE_NORMAL;
	if (mvRolXmodemUsed == 1) {
		/* image was copied during booton or rol_tftp/rol_next */
	    printf("We assume that image was copied to RAM address 0x%x\n",mvRolImageaddr);
		if (rolGetUimageParam(mvRolImageaddr, &uImage_size, &offset) == MV_FALSE)
	        return MV_FALSE;
	    if (offset == 0) {
	        printf("%s: can't find uImage file\n", __FUNCTION__);
	        return MV_FALSE;
	    }

	    header_version = MV_16BIT_BE(image_header->header_version);
	    printf("Ros image Header version: %d\n", header_version);
	    mvRolImageKernelVer = (header_version<=1) ? image_kernel_ver_3_4 : image_kernel_ver_3_10;

	    if (MV_FALSE == rolCopyFile(MV_ROL_UIMAGE_LOAD_ADRS, mvRolImageaddr + offset, uImage_size))
	        return MV_FALSE;

	    if (mvRolImageKernelVer == image_kernel_ver_3_10) {
	        num_of_files = MV_32BIT_BE(image_header->num_of_files);

	        for (file_cnt = 0; file_cnt < num_of_files; file_cnt++) {
	            file_ptr = (struct MV_ROL_FILE_STC *)(mvRolImageaddr + sizeof(MV_ROL_IMAGE_HEADER) + file_cnt * sizeof(MV_ROL_IMAGE_FILE_STC));
	            if ((file_ptr->type == included_file_type_dtb) && (MV_32BIT_BE(file_ptr->compt_bmp) & mvRolPPtype) ) {
	                printf("DTB file matches (for CPU type %d)\n", mvRolPPtype);
	                DTB_offset = MV_32BIT_BE(file_ptr->file_offset);
	                DTB_size = MV_32BIT_BE(file_ptr->file_size);
	                break;
	            }
	        }
	        if (!DTB_offset) {
	            printf("%s: can't find compatible DTB file\n", __FUNCTION__);
	            return MV_FALSE;
	        }
	        if (MV_FALSE == rolCopyFile(MV_ROL_UIMAGE_LOAD_ADRS + uImage_size, mvRolImageaddr + DTB_offset, DTB_size))
	            return MV_FALSE;
	    }
		printf("uImage in RAM at addr 0x%08x ...\n", MV_ROL_UIMAGE_EXEC_ADRS);
		mvRolImageParam->boot_mode = MV_ROL_BOOT_MODE_XMODEM;
	} else {
		memset(local_args[1], 0, 12);
		sprintf(local_args[1], "%s", "rootfs");
		printf("do UBI mount rootfs \n");
		ret_val = do_ubifs_mount(cmdtp, 0, 2, local_args);
		if (ret_val != 0) { /* mount ubi failed */
			printf("UBI mount rootfs FAILED %d\n", ret_val);
			result = MV_FALSE;

			result = rolCopyImage(cmdtp, local_args, mvRolImageaddr, MV_FALSE);
		} else { /* mount UBI OK */
			result = MV_FALSE;
			if (getenv("skip_mfg"))
				printf("Skipping MFG image\n");
			else {
				printf("Doing rolCopyMFG\n");
				result = rolCopyMFG(cmdtp, MV_ROL_UIMAGE_EXEC_ADRS);
			}
			if (result == MV_FALSE) {
				printf("Find active image and copy to RAM\n");
				result = rolCopyImage(cmdtp, local_args, mvRolImageaddr, MV_TRUE);
			}
			else
			{
			   /* in case of loading image set mvRolImageKernelVer according to MFG kernel version */
		       /* need to add read from MFG header the Linux kernel version in case of support in different kernel versions , default is image_kernel_ver_3_4*/
  		    	mvRolImageKernelVer = image_kernel_ver_3_4;
			}
		}
	}

	if (result == MV_TRUE) {
		memset(buf1,0,12);
		sprintf(buf1, "%x", MV_ROL_UIMAGE_EXEC_ADRS);

        printf("ROL image kernel ver: %s\n", 
               mvRolImageKernelVer==image_kernel_ver_3_4 ? "3.4" : "3.10");
        switch (mvRolImageKernelVer) {
        case image_kernel_ver_3_4:
            /* Run either MFG or uImage */
            do_bootm(cmdtp, 0, 2, local_args);
            break;

        case image_kernel_ver_3_10:
            do_bootz(cmdtp, 0, 2, local_args);
            break;

        default:
            printf("Error: Kernel version was not identified\n");
        }
	} else
		printf("FAILED to run ROL image\n");

	return 0;
}

U_BOOT_CMD(
	rolimage,     1,      1,       do_rolImage_cmd,
	"rolimage - Initialize ROL image\n",
	"\n" "	- Initialize ROL image\n"
);

int do_rolImage_sd_next_cmd( cmd_tbl_t *cmdtp, int flag, int argc,  char * const argv[]) {
	printf("%s: start\n",__func__);
	if (argc != 1) {
		printf ("Usage:\n%s\n", cmdtp->usage);
		return 1;
	}

	user_printf("Updating from SD...\n");
	
	char *image_name = getenv(MV_ROL_SD_UPDATE_IMAGE_ENV);
	if(!image_name) {
		user_printf("Environment variable[%s] with SD update filename is not set.\n",MV_ROL_SD_UPDATE_IMAGE_ENV);
		return do_rolImage_cmd(cmdtp,flag,argc,argv);
	}    
	
	unsigned long time = get_timer(0);
	user_printf("Loading ROL image from sd[%s]\n",image_name);
	int image_size = read_file_from_sd(FS_TYPE_ANY,image_name,mvRolImageaddr,0,0);
	if(image_size <= 0) {
		user_printf("Failed loading image file from sd[%s]\n",image_name);
	} else {
		time = get_timer(time);
		user_printf("%d bytes read in %lu ms\n", image_size, time);
		
		char buf1[12], buf2[MV_ROL_IMAGE_NAME_SIZE], buf3[12], buf4[12];
		char *local_args[5] = { "rolimage_sd_next", buf1, buf2, buf3, buf4 };
		if(rolCheckImage(cmdtp, local_args, image_size, mvRolImageaddr) == MV_TRUE) {
			mvRolXmodemUsed = 1;
			user_printf("Image[%s] ready for boot\n",image_name);
		} else {
			user_printf("Image[%s] is invalid\n",image_name);
		}
	}
	
	return do_rolImage_cmd(cmdtp,flag,argc,argv);
}

U_BOOT_CMD(
	rolimage_sd_next,     1,      1,       do_rolImage_sd_next_cmd,
	"rolimage_sd_next - Update ROL image from sd\n",
	"\n" "	- Update ROL image from sd\n"
);

/* run image from tftp instead of using xmodem */
int do_rolImage_tftp_cmd ( cmd_tbl_t *cmdtp, int flag, int argc,  char * const argv[])
{
	mvRolXmodemUsed = 1;
	return do_rolImage_cmd(cmdtp,flag,argc,argv);
}



U_BOOT_CMD(
	rolimage_tftp,     1,      1,       do_rolImage_tftp_cmd,
	"rolimage_tftp - Initialize ROL image with tftp \n",
	"\n" "	- Initialize ROL image using tftp\n"
);


/***********************************************************************************
 * MAC based password support
 **********************************************************************************/
/* F, G, H and I are basic MD5 functions.  */
#define MV_ROL_MD5MOD_FUNC_F_MAC(x, y, z) (((x) & (y)) | ((~x) & (z)))
#define MV_ROL_MD5MOD_FUNC_G_MAC(x, y, z) (((x) & (z)) | ((y) & (~z)))
#define MV_ROL_MD5MOD_FUNC_H_MAC(x, y, z) ((x) ^ (y) ^ (z))
#define MV_ROL_MD5MOD_FUNC_I_MAC(x, y, z) ((y) ^ ((x) | (~z)))

/* ROTATE_LEFT rotates x left n bits. */
#define MV_ROL_MD5MOD_ROTATE_LEFT_MAC(x, n) (((x) << (n)) | ((x) >> (16-(n))))

/* ROS passport and registry */
#define MV_ROL_PASSPORT_MAGIC_CNS         "PAMG"
#define MV_ROL_PASSPORT_EMPTY_CNS         0xFFFFFFFF
#define MV_ROL_PASSPORT_RET_VALID_CNS     0
#define MV_ROL_PASSPORT_RET_INVALID_CNS   1
#define MV_ROL_PASSPORT_RET_EMPTY_CNS     2
#define MV_ROL_SERIAL_NUM_STRING_SIZE_CNS 32
#define MV_ROL_REGISTRY_FILE              "system/.registry"
#define MV_ROL_REGISTRY_MAC_PREFIX        "0x1005="

/* Enumerator for hash function type: F, G, H or I */
typedef enum{
	MV_ROL_MD5MOD_FUNC_F_E,
	MV_ROL_MD5MOD_FUNC_G_E,
	MV_ROL_MD5MOD_FUNC_H_E,
	MV_ROL_MD5MOD_FUNC_I_E
}MV_ROL_MD5MOD_FUNC_ENT;

typedef struct _mv_rol_passport_param
{
   unsigned int     length;
   unsigned int     magic;
   char             mac_address[MV_ROL_SERIAL_NUM_STRING_SIZE_CNS];
   char             serial_num[MV_ROL_SERIAL_NUM_STRING_SIZE_CNS];
   char             hw_version[MV_ROL_SERIAL_NUM_STRING_SIZE_CNS];
   unsigned int     crc;
}MV_ROL_PASSPORT_PARAM_STC;


/* Global state array */
static unsigned short mvRolMd5modState[4];

/* constants used for variable bit shift */
static const unsigned char mvRolMd5modS[4][4]  = {{7, 12, 1, 6},
												  {5, 9, 14, 4},
												  {4, 11, 0, 7},
												  {6, 10, 15, 5}};

/* Constants used for permutations. Calculated by abs(sin((i+2)/6))*2^16 */
static const unsigned short mvRolMd5modAC[4][8] =
	{{0x53C3, 0x7ABB, 0x9E4D, 0xBD7C, 0xD76A, 0xEB60, 0xF8D0, 0xFF5B},
	 {0xFED3, 0xF73A, 0xE8C7, 0xD3E1, 0xB91C, 0x9935, 0x750F, 0x4DAB},
	 {0x2420, 0x066B, 0x30C9, 0x59CC, 0x8053, 0xA34C, 0xC1BD, 0xDAD1},
	 {0xEDD3, 0xFA3F, 0xFFBB, 0xFE21, 0xF57C, 0xE609, 0xD036, 0xB49E}};

static void mvRolMd5modPermute(
	unsigned char idx,
	unsigned short x,
	unsigned char s,
	unsigned short ac,
	MV_ROL_MD5MOD_FUNC_ENT type)
{
	unsigned char a, b, c, d;

	a = idx; b = (idx+1)%4, c = (idx+2)%4, d = (idx+3)%4;
	switch(type) {
	case MV_ROL_MD5MOD_FUNC_F_E:
		mvRolMd5modState[a] += MV_ROL_MD5MOD_FUNC_F_MAC(mvRolMd5modState[b],
													 mvRolMd5modState[c],
													 mvRolMd5modState[d]);
		break;
	case MV_ROL_MD5MOD_FUNC_G_E:
		mvRolMd5modState[a] += MV_ROL_MD5MOD_FUNC_G_MAC(mvRolMd5modState[b],
													mvRolMd5modState[c],
													mvRolMd5modState[d]);
		break;
	case MV_ROL_MD5MOD_FUNC_H_E:
		mvRolMd5modState[a] += MV_ROL_MD5MOD_FUNC_H_MAC(mvRolMd5modState[b],
													mvRolMd5modState[c],
													mvRolMd5modState[d]);
		break;
	case MV_ROL_MD5MOD_FUNC_I_E:
		mvRolMd5modState[a] += MV_ROL_MD5MOD_FUNC_I_MAC(mvRolMd5modState[b],
													mvRolMd5modState[c],
													mvRolMd5modState[d]);
	}
	mvRolMd5modState[a] += x + ac;
	mvRolMd5modState[a] = MV_ROL_MD5MOD_ROTATE_LEFT_MAC(mvRolMd5modState[a], s);
	mvRolMd5modState[a] += mvRolMd5modState[b];


}

static void mvRolMd5modTransform(unsigned short *state, char *input)
{
  unsigned short x[8];
  unsigned int i;

  /* Copy given state values to global array and convert given unsigned char array
     to unsigned short array */
  for(i=0; i<3; i++)
  {
    x[i] = (input[2*i] << 8) + input [2*i+1];
    x[6-i] = x[i] + (i+1)*0x4D1B;
    mvRolMd5modState[i] = state[i];
  }

  x[3] = 0x0680;
  x[7] = 0x9FF5;
  mvRolMd5modState[3] = state[3];

  /* Round 0 */
  for(i=0; i<8; i++)
    mvRolMd5modPermute(3-((i+3)%4),
                         x[i],
                         mvRolMd5modS[0][i%4],
                         mvRolMd5modAC[0][i],
                         MV_ROL_MD5MOD_FUNC_F_E);

  /* Round 1 */
  for(i=0; i<8; i++)
    mvRolMd5modPermute(3-((i+3)%4),
                         x[(5*i + 1)%8],
                         mvRolMd5modS[1][i%4],
                         mvRolMd5modAC[1][i],
                         MV_ROL_MD5MOD_FUNC_G_E);

  /* Round 2 */
  for(i=0; i<8; i++)
    mvRolMd5modPermute(3-((i+3)%4),
                         x[(3*i + 5)%8],
                         mvRolMd5modS[2][i%4],
                         mvRolMd5modAC[2][i],
                         MV_ROL_MD5MOD_FUNC_H_E);

  /* Round 3 */
  for(i=0; i<8; i++)
    mvRolMd5modPermute(3-((i+3)%4),
                         x[(7*i)%8],
                         mvRolMd5modS[3][i%4],
                         mvRolMd5modAC[3][i],
                         MV_ROL_MD5MOD_FUNC_I_E);

  for(i=0; i<4; i++)
  state[i] += mvRolMd5modState[i];

}

 /**************************************************************************
  * mvRolPassword - Calculate MD5 based hash over given vector.
  *
  * RETURNS: status - 0 for success
  *
  * ALGORITHM: transform the given vector to unsigned short values, calculate hash
  *            and return state[0] and state[2] as 8 character string
  *
  * ASSUMPTIONS: input is a 6 x unsigned char values vector (MAC recommended)
  ***************************************************************************/
static unsigned int mvRolPassword(char *output, char *input)
{
	strcpy(output,"franchise");

	return 0;
}


 /**************************************************************************
  * mvRolDebugModeEnable - Enable debug / LAB mode, based on password
  *
  * RETURNS: 1 to enable debug mode, 0 to disable.
  *          Debug mode will be enabled in the following cases:
  *          - Flash is inaccessible
  *          - Passport info not found / corrupted
  *          - Correctly typed passport during 3 trials.
  *
  * ALGORITHM: Find MAC address in flash sector 0 / 1 after booton in passport
  *            area, check registry ini file for overwritten MAC address.
  *            If not found, set to default ff:ff:ff:ff:ff:ff
  *            Calculate password, prompt user to input password and compare.
  *
  ***************************************************************************/

static int mvRolDebugModeEnable (void)
{
	char mac_addr[6], *tmp_str, output[9], *mac_str = "ff:ff:ff:ff:ff:ff", *line;
	char *do_nand_args[5] = {NULL, "read", "00000000", "1f800", "200"};
	char *do_ubifs_args[5] = {NULL, "00000000", MV_ROL_REGISTRY_FILE, NULL, NULL};
	char *do_ubi_mount_args[2] = {NULL, "rootfs"};
	char *do_ubi_args[3] = {NULL, "part", "rootfs"}; 
	int i, status = 1;
	cmd_tbl_t cmdtp;
	MV_ROL_PASSPORT_PARAM_STC *passport;

	/* Try loading registry file */

	/* Set partition */
	status = do_ubi(&cmdtp, 0, 3, do_ubi_args);

	/* Mount volume */
	if (status == 0) {
		status = do_ubifs_mount(&cmdtp, 0, 2, do_ubi_mount_args);
	}

	/* Read file */
	if (status == 0) {
		snprintf(do_ubifs_args[1], 8, "%x", MV_ROL_UIMAGE_LOAD_ADRS);
		status = do_ubifs_load(&cmdtp, 0, 3, do_ubifs_args);
	}

	/* Read MAC address from registry file */
	if (status == 0) {
		status = 1;

		if ((tmp_str = getenv("filesize")) != NULL) {
			i = simple_strtoul(tmp_str, NULL, 16);

			if (i > 0) { /* registry file found */
				*(char *)(MV_ROL_UIMAGE_LOAD_ADRS + i) = '\0';
				tmp_str = (char *)(MV_ROL_UIMAGE_LOAD_ADRS);
				tmp_str = strstr(tmp_str, MV_ROL_REGISTRY_MAC_PREFIX);

				if (tmp_str != NULL) {
					strncpy(mac_str, tmp_str + strlen(MV_ROL_REGISTRY_MAC_PREFIX), 18);
					status = 0;
				}
			}
		}
		/* unmount UBI file system */
		do_ubifs_umount(&cmdtp, 0, 1, do_ubi_args);
	}

	/* In case MAC was not found in registry, read from passport */
	if (status != 0) {
		/* Read passport from sector 0 */
		snprintf(do_nand_args[2], 8, "%x", MV_ROL_UIMAGE_LOAD_ADRS);
		status = do_nand(&cmdtp, 0, 5, do_nand_args);

		if (status != 0) { /* If flash access fails, enable debug mode by default */
			printf("Error reading from flash\n");
			return 1;
		}

		/* Verify passport structure not corrupted */
		passport = (MV_ROL_PASSPORT_PARAM_STC *)MV_ROL_UIMAGE_LOAD_ADRS;

		if (!strncmp((char *)&passport->magic, MV_ROL_PASSPORT_MAGIC_CNS, 4)) {
			strncpy(mac_str, passport->mac_address, 18); /* Copy MAC from passport */
		} else {
			return 1; /* If passport does not exist, enable debug mode by default */
		}
	}

	/* Parse MAC address into 6 bytes */
	tmp_str = strtok(mac_str, ":");
	for (i = 0; i < 6; i++) {
		mac_addr[i] = mvRolHtoi(tmp_str);
		tmp_str = strtok(NULL, ":");
	}

	printf("\n\nProduct MAC address is %02x:%02x:%02x:%02x:%02x:%02x\n%s",
			mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5],
			"LAB mode was pressed.\n"
			"Access to debug menu is restricted and requires password.\n"
			"Please contact customer support.\n");

    if (getenv("skip_psw")) {
        printf("\nEntering LAB mode...\n");
        return 1;
    }

	/* Calculate password */
	mvRolPassword(output, mac_addr);

	/* Request password from user */
	for (i = 0; i < 3; i++) {
		printf("Password: ");
		line = getline();
		/* CTRL+C */
		if (!line)
			i = 3;
		if (line[0] == '\0')
			continue;
		if (!strncmp(line, output, 8)) {
			printf("\nEntering LAB mode...\n");
			return 1;
		}
	}

	/* Too many retries / CTRL+C pressed */
	printf("\nFailed...\nProduct will load in normal mode.\n\n");
	return 0;
}


 /**************************************************************************
  * mvRolLoadImageFromFlashOffset
  *
  * RETURNS: 0 for success
  * 		 image_size_ptr - size of image loaded to RAM
  *
  * ALGORITHM: 1. Load image first page from flash, and check that it begins with MAGIC. If yes -
  * 		   2. Load the entire image to RAM, and check crc.
  *
  * ASSUMPTIONS:
  ***************************************************************************/
static int mvRolLoadImageFromFlashOffset(cmd_tbl_t *cmdtp, char *local_args[5], __u32 nand_pagesize, MV_U32 from_offset, MV_U32 to_ram_addr, MV_U32* image_size_ptr)
{
	MV_U32              image_crc, header_crc_not_swapped;
	MV_ROL_IMAGE_HEADER *image_header;
	char                project_magic[MV_ROL_IMAGE_MAGIC_LENGTH];
	char				image_magic_buf[MV_ROL_IMAGE_MAGIC_LENGTH+1] = {0};
	int                 status;

	/* Load magic initialization constants. We use it to verify the image header. */
	strncpy(project_magic, xstr(MV_ROL_PROJECT_MAGIC), MV_ROL_IMAGE_MAGIC_LENGTH);

	/* Syntax is:
	 *   0    1        2     3    4
	 *   nand read  [addr off size]*/
	memset(local_args[1], 0, 12);
	sprintf(local_args[1], "%s", "read.e");
	memset(local_args[2], 0, 12);
	sprintf(local_args[2], "%x", to_ram_addr);
	memset(local_args[3], 0, 12);
	sprintf(local_args[3], "%x", from_offset);
	memset(local_args[4], 0, 12);
	sprintf(local_args[4], "%x", nand_pagesize); /* single page */

	status = do_nand(cmdtp, 0, 5, local_args);
	if (status == 0 ) { /* read was ok */
		/* check image MAGIC */
		image_header = (MV_ROL_IMAGE_HEADER*)to_ram_addr;
		strncpy(image_magic_buf, (char*)image_header->image_magic, MV_ROL_IMAGE_MAGIC_LENGTH);
		if (strncmp((char*)(image_header->image_magic), project_magic, MV_ROL_IMAGE_MAGIC_LENGTH)) {
			printf("\nInvalid magic %s found - not loading from NAND\n", image_magic_buf);
			return 1;
		}

		printf("\nODM image found (valid magic %s)\n", image_magic_buf);

		/* Load the full image to ram */
		*image_size_ptr = MV_32BIT_BE(image_header->image_size);
		memset(local_args[4], 0, 12);
		sprintf(local_args[4], "%x", *image_size_ptr);
		printf("loading ROL image to 0x%x\n", to_ram_addr);
		status = do_nand(cmdtp, 0, 5, local_args);
		if (status == 0 ) { /* read was ok */
			/* Check CRC */
			image_crc = MV_32BIT_BE(image_header->image_crc);
			header_crc_not_swapped = image_header->header_crc;
			image_header->image_crc = 0;
			image_header->header_crc = 0;
			if (mvRolCheckCrc(image_crc, (MV_U8*)to_ram_addr, *image_size_ptr)) {
				mvUartSetSilent(MV_FALSE);
				printf("Inner crc: 0x%x. Invalid image CRC !!\n", image_crc);
				return 1;
			}
			/* restore CRCs*/
			image_header->image_crc = MV_BYTE_SWAP_32BIT(image_crc);
			image_header->header_crc = header_crc_not_swapped;
		}
	}

	return status;
}


 /**************************************************************************
  * mvRolLoadMfgFromFlashOffset
  *
  * RETURNS: MV_ROL_RET_CRC_FAILED - when CRC check of mfg loaded to RAM fails
  * 		 MV_ROL_RET_OK		   - otherwise
  *
  * ALGORITHM: 1. Load MFG from flash, and check that it begins with MAGIC. If yes -
  * 		   2. Load the entire MFG to RAM, and check crc.
  *
  * ASSUMPTIONS:
  ***************************************************************************/
static MV_ROL_RET_ENUM mvRolLoadMfgFromFlashOffset(cmd_tbl_t *cmdtp, char *local_args[5], __u32 nand_pagesize, MV_U32 from_offset, MV_U32 to_ram_addr)
{
	image_header_t 		*image_header = (image_header_t*)to_ram_addr;	/* uImage header */
	MV_U32				image_size;
	int                 status;

	/* Syntax is:
	 *   0    1        2     3    4
	 *   nand read  [addr off size]*/
	memset(local_args[1], 0, 12);
	sprintf(local_args[1], "%s", "read.e");
	memset(local_args[2], 0, 12);
	sprintf(local_args[2], "%x", to_ram_addr);
	memset(local_args[3], 0, 12);
	sprintf(local_args[3], "%x", from_offset);
	memset(local_args[4], 0, 12);
	sprintf(local_args[4], "%x", nand_pagesize); /* single page */

	status = do_nand(cmdtp, 0, 5, local_args);
	if (status == 0 ) { /* read was ok */
		/* check image MAGIC */
		if (!image_check_magic(image_header)) {
			printf("\nDid not found MFG magic at flash offset 0x%x\n", from_offset);
			return MV_ROL_RET_OK;
		}

		image_size = image_get_image_size(image_header);
		printf("\nODM MFG found. Loading %u bytes to 0x%x\n", image_size, to_ram_addr);

		/* Load the full MFG to ram */
		memset(local_args[4], 0, 12);
		sprintf(local_args[4], "%x", image_size);
		printf("loading MFG to 0x%x\n", to_ram_addr);
		status = do_nand(cmdtp, 0, 5, local_args);
		if (status == 0 ) {
			if (!image_check_dcrc(image_header)) {
				mvUartSetSilent(MV_FALSE);
				printf("Invalid MFG CRC !!\n");
				return MV_ROL_RET_CRC_FAILED;
			}

			printf("CRC OK\n");
		}
	}

	return MV_ROL_RET_OK;
}

