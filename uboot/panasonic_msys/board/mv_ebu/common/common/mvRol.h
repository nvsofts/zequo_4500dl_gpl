/*******************************************************************************
Copyright (C) Marvell International Ltd. and its affiliates

This software file (the "File") is owned and distributed by Marvell 
International Ltd. and/or its affiliates ("Marvell") under the following
alternative licensing terms.  Once you have made an election to distribute the
File under one of the following license alternatives, please (i) delete this
introductory statement regarding license alternatives, (ii) delete the two
license alternatives that you have not elected to use and (iii) preserve the
Marvell copyright notice above.

********************************************************************************
Marvell Commercial License Option

If you received this File from Marvell and you have entered into a commercial
license agreement (a "Commercial License") with Marvell, the File is licensed
to you under the terms of the applicable Commercial License.

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
********************************************************************************
Marvell BSD License Option

If you received this File from Marvell, you may opt to use, redistribute and/or 
modify this File under the following licensing terms. 
Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:

    *   Redistributions of source code must retain the above copyright notice,
	    this list of conditions and the following disclaimer. 

    *   Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution. 

    *   Neither the name of Marvell nor the names of its contributors may be 
        used to endorse or promote products derived from this software without 
        specific prior written permission. 
    
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND 
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR 
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES 
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON 
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS 
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*******************************************************************************/

#ifndef _INC_MVROL_H
#define _INC_MVROL_H
#include "mvTypes.h"

#define MV_ROL_PARAM_OFFSET			0x200 /* offset from Image/Uboot whew params are store */
#define MV_ROL_PARAM_SIZE			0x100

#define MV_ROL_LAB_STR				"LAB"
#define MV_ROL_XMODEM_STR			"XMODEM"
#define MV_ROL_IMAGE_STR			"IMAGE"
#define MV_ROL_UBOOT_1_OFFSET 		0x60000
#define MV_ROL_UBOOT_2_OFFSET 		0x1a0000
#define MV_ROL_UBOOT_SECTOR_SIZE	0x20000
#define MV_ROL_UBOOT_SIZE			0x140000
#define MV_ROL_NAND_SIZE			0x10000000

#define MV_ROL_UBOOT_LOAD_ADRS 		0x1200000
#define MV_ROL_UBOOT_EXEC_ADRS 		0x1200000

#define MV_ROL_UIMAGE_LOAD_ADRS		0x2000000
#define MV_ROL_UIMAGE_EXEC_ADRS		0x2000000
#define MV_ROL_UBOOT_CHECK_ADRS		0x8000000

#define MV_ROL_IMAGE_LIST_ADRS		0x9f00000
#define MV_ROL_IMAGE_SIZE			(64 *_1M)) 	/* offset from end of RAM where the full Image file will be store */
#define MV_ROL_RAM_SIZE_LIMIT		(512*_1M)	/* RAM size limit we are able to work with. Corresponding define should reside in boot appl */

#define MFG_RAM_OFFSET_BEFORE_ROL_IMAGE	(16*_1M) /* Offset in RAM, before ROL image, where MFG might reside */

#define MV_ROL_IMAGE_NAME_SIZE		256
#define MV_ROL_IMAGE_TYPE_ACTIVE	1
#define MV_ROL_IMAGE_TYPE_INACTIVE	2

#define MV_ROL_KEY_RETURN			10
#define MV_ROL_KEY_EOF				0xFF

#define MV_ROL_STRING_LENGTH   		32
#define MV_ROL_IMAGE_MAGIC_LENGTH 	4

#define	MV_ROL_MAX_IMAGE_SIZE		(64*_1M)

/* This enum is identical to the numbers used in image_files_list.txt to build the rosImage */
enum INCLUDED_FILE_TYPE {
    included_file_type_uboot = 0,
    included_file_type_kernel_image,
    included_file_type_projet_info,
    included_file_type_dtb
};

enum IMAGE_KERNEL_VER {
    image_kernel_ver_unknown,
    image_kernel_ver_3_4,
    image_kernel_ver_3_10
};

#define PP_TYPE_AXP     1
#define PP_TYPE_MSYS    2


/* An Image file includes the following components:
    1)Header:
		Header size - the image header size without the added files.
    	Header version - the header version.
    	Header crc	- only the header crc without the files.
    	Image size - the total image size (header + files).
    	Image magic number - taken from ros .
    	Image crc - calculated crc on all image, when header_crc & image_crc filed are zerod.
    	Image version - taken from ros.
    	Image time - The time the Image was created.
    	Minimum UBoot - Minimum UBoot version supporting the Image.
    	Num of files - the number of file that are included in this image file,
				    	there must be at least 2 files UBoot and Uimage.
    2) Files included:
    	File name - the included file name ("ubbot","uimage")
    	File size - the included file size.
    	File offset - the offset from the begining of the image file where the included file starts
 
	Default files included:
		U-Boot - include UBoot header and data.
		U-Image - includes the following components:
				Linus Kernel
				RAM File System, including all ros applications
*/
typedef struct _mv_rol_image_header
{
   MV_U16           header_size;
   MV_U16           header_version;
   MV_U32           header_crc;
   MV_U32           image_size;
   MV_U8         	image_magic[MV_ROL_IMAGE_MAGIC_LENGTH];
   MV_U32           image_crc;
   MV_U8            image_version[MV_ROL_STRING_LENGTH];
   MV_U8            image_time[MV_ROL_STRING_LENGTH];
   MV_U8            minimum_uboot_version[MV_ROL_STRING_LENGTH];   
   MV_U32           num_of_files;
}MV_ROL_IMAGE_HEADER;

#pragma pack(1)
typedef struct _mv_rol_image_file_stc
{
   MV_U8            file_name[MV_ROL_STRING_LENGTH];
   MV_U32        	file_size;
   MV_U32           file_offset;
}MV_ROL_IMAGE_FILE_STC;

/* This file describing structure is used for kernel 3.10 & up, and has same size as MV_ROL_IMAGE_FILE_STC */
struct MV_ROL_FILE_STC
{
   MV_U8            file_name[MV_ROL_STRING_LENGTH-5];
   MV_U8            type;
   MV_U32           compt_bmp;     /* Used for testing compatibilty */
   MV_U32        	file_size;
   MV_U32           file_offset;
};
#pragma pack()

/* in order to create the ros_uboot.bin we add the following header to the
   uboot.bin file. the new created file is located in the FLASH in 2 copies for backup
   the first copy is located at MV_ROL_UBOOT_1_OFFSET and the second at MV_ROL_UBOOT_2_OFFSET*/
typedef struct _mv_rol_uboot_header
{
	MV_U16				header_size;
	MV_U16				header_version;
	MV_U32				uboot_size;     
	MV_U32				uboot_crc;
	MV_U8				uboot_version[MV_ROL_STRING_LENGTH];   
	MV_U8				RESERVED[4];  /*added to set alignment to 16 bytes */ 
}MV_ROL_UBOOT_HEADER;

#define MV_ROL_BOOT_MODE_NORMAL		0	/* 0 - normal mode */
#define MV_ROL_BOOT_MODE_XMODEM		1	/* 1 - boot with xmodem */
#define MV_ROL_BOOT_MODE_SECONDARY	2	/* 2 - boot from secondary image */
#define MV_ROL_BOOT_MODE_NFS		3	/* 3 - boot from nfs */

/* In order to pass information from UBoot to the Ros init application,
   wa add the following structure in a pre defined place in RAM
   (Image start address - MV_ROL_PARAM_OFFSET) */

#define MV_ROL_IMAGE_PATTERN		0xA5A5A5A5		
typedef struct _mv_rol_image_param_stc
{
	MV_U32				image_pattern;
	MV_U16				boot_mode;	
	MV_U16				lab_mode;
	MV_U32				ram_size;
}MV_ROL_IMAGE_PARAM;


int mvRolInitParams(void);
void mvRolLateInit(void);
int mvRolBanner(void);
/*int mv_rol_get_param(char *param_name);*/

#endif /* _INC_MVROL_H */

