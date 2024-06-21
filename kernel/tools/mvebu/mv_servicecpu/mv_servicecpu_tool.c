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
available along with the File in the license.txt file.

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

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/poll.h>
#include <sys/mman.h>
#include <stdbool.h>
#include <err.h>
#include <getopt.h>

#define TRUE	1
#define FALSE	0

/* There are 6 parameters to congigure SRAM(cache size, free area size,hostmem size
	, oamdb area size, share memory size and firmware size),
	also 4 parameter to configure DRAM(shmem offset, shmem size,
	hostmem offset and hostmem size), it's too complicated if we set all parameters
	in one command. So we make the conguration of SRAM and DRAM seperately,
	then apply together, this flag will be passed first to indicate which part we
	are going to set, the apply operation only disable IPC and set memory sections'
	size/base in software, but not setup memory in hardware, the memory hardware
	set up work will be done in device open*/
/* For external mode with firmware in SRAM, only CONFIG_SRAM works, CONFIG_SRAM will
	disable IPC, set memory sections' size/base in software, the memory hardware
	set up work will be done in device open*/
/* For external mode with firmware in SRAM, CONFIG_SRAM/CONFIG_DRAM/CONFIG_APPLY works,
	CONFIG_SRAM only stores SRAM configuration and CONFIG_DRAM only stores DRAM configuration,
	CONFIG_APPLY will disable IPC, set memory sections' size/base in software, the memory hardware
	set up work will be done in device open*/
enum scpu_config_flag {
	CONFIG_SRAM = 0,/* set SRAM configuration items, valid when firmware is in either SRAM or DRAM */
	CONFIG_DRAM,	/* set DRAM configuration items, valid only when firmware is in DRAM */
	CONFIG_APPLY,	/* apply SRAM&DRAM configurations, valid only when firmware is in DRAM */
	CONFIG_MAX
};

#define SERVICECPU_HOSTMEM_INFO_ITEMS 4

struct servicecpu_sram_config {
	unsigned int seg_cache_size;
	unsigned int seg_free_size;
	unsigned int seg_hostmem_size;
	unsigned int seg_oamdb_size;
	unsigned int seg_shmem_size;
	unsigned int seg_code_size;
};

struct servicecpu_dram_config {
	unsigned int seg_shmem_in_dram_offset;
	unsigned int seg_shmem_in_dram_size;
	unsigned int seg_hostmem_in_dram_offset;
	unsigned int seg_hostmem_in_dram_size;
};

struct servicecpu_config {
	unsigned int config_flag;
	union {
		struct servicecpu_sram_config sram_config;
		struct servicecpu_dram_config dram_config;
	};
};

struct servicecpu_hostmem_info {
	unsigned int items[SERVICECPU_HOSTMEM_INFO_ITEMS];
};

#ifdef CONFIG_MV_SERVICECPU_IPC_SUPPORT
struct servicecpu_ipcchn_set {
	unsigned int	chn_id;
	unsigned int	chn_en;
	char		*dev_name;
};
#endif

#define SERVICECPU_IOC_MAGIC		's'
#define SERVICECPU_IOC_UNRESET		_IOW(SERVICECPU_IOC_MAGIC, 1, unsigned int)
#define SERVICECPU_IOC_SENDIRQ		_IO(SERVICECPU_IOC_MAGIC, 2)
#define SERVICECPU_IOC_CONFIG		_IOW(SERVICECPU_IOC_MAGIC, 3, struct servicecpu_config)
#define SERVICECPU_IOC_MMAPINFO		_IOR(SERVICECPU_IOC_MAGIC, 4, struct servicecpu_hostmem_info)
#ifdef CONFIG_MV_SERVICECPU_IPC_SUPPORT
#define SERVICECPU_IOC_SET_IPCCHN	_IOW(SERVICECPU_IOC_MAGIC, 5, struct servicecpu_ipcchn_set)
#endif

#define _4K         0x00001000

void show_usage(void)
{
	fprintf(stderr,
		"Usage:\n"
		" mv_servicecpu_tool -h\n"
		"   Display this help.\n"
		"\n"
		"	-m, --mmap [info][offset] :\n"
		"		return info of HOST memory or mmap first 4KB of HOST memory\n"
		"		base on offset(default 0x0), dumps first 32byte and munmap\n"
		"	    read m_size from Service CPU's SRAM from f_off offset\n"
		"	    write f_size bytes from f_name to Service CPU's SRAM in f_off offset\n"
		"	-c, --config [SRAM/DRAM] [config_list] :\n"
		"		SRAM config_list:  when serviceCPU firmware is in SRAM,\n"
		"		                   cache_size free_size hostmem_size oamdb_size shmem_size code_size\n"
		"		                   when firmware is in DRAM,\n"
		"                                  cache_size free_size hostmem_size oamdb_size :\n"
		"		DRAM config_list:  when firmware is in DRAM,\n"
		"		                   shmem_offset shmem_size hostmem_offset hostmem_size :\n"
		"		                   configure offset and size of shared memory and\n"
		"		                   host control memory in local dram\n"
#ifdef CONFIG_MV_SERVICECPU_IPC_SUPPORT
		"	-e, --setipcchn [chn_id][chn_en] :\n"
		"		set ipc link channel status\n"
		"		chn_id - serviceCPU ipc link channel ID\n"
		"		chn_en - 0 disable 1 enable\n"
#endif
		"    Special for external servicecpu mode with firmware run in SRAM:\n"
		"	-t, --reset		: put Service CPU into reset\n"
		"	-u, --unreset	: un-reset Service CPU\n"
		"	-s, --sendirq	: send irq to Service CPU\n"
		"	-r, --read f_off m_size :\n"
		"	-w, --write f_off f_name f_size :\n"
		"\n\n"
		"   Example:\n"
		"	./mv_servicecpu_tool --mmap info\n"
		"	./mv_servicecpu_tool --mmap 0x10000\n"
#ifdef CONFIG_MV_SERVICECPU_IPC_SUPPORT
		"	./mv_servicecpu_tool --setipcchn 0 1(enable ipc channel 0)\n"
		"	./mv_servicecpu_tool --setipcchn 0 0(disable ipc channel 0)\n"
#endif
		"    Special for external servicecpu mode with firmware run in SRAM:\n"
		"	./mv_servicecpu_tool --write 0 RTOSDemo.bin 15312\n"
		"	./mv_servicecpu_tool --read 0 15312 > sram_read.bin\n"
		"	./mv_servicecpu_tool --unreset\n"
		"	./mv_servicecpu_tool --reset\n"
		"	./mv_servicecpu_tool --sendirq\n"
		"	./mv_servicecpu_tool --config SRAM 0x80000 0x0 0x80000 0x40000 0x40000 0x80000\n"
		"    Special for external servicecpu mode with firmware run in SRAM:\n"
		"	./mv_servicecpu_tool --config SRAM 0x180000 0x0 0x40000 0x40000\n"
		"	./mv_servicecpu_tool --config DRAM 0x180000 0x40000 0x2c0000 0x100000\n"
	);
}

static int write_to_servicecpu(int fd_servicecpu, char *off, char *f_name, char *size)
{
	int rfd;
	ssize_t nr, nw, bsize;
	char *buf = NULL;

	/* open file with firmware */
	rfd = open(f_name, O_RDONLY);
	if (rfd <= 0) {
		fprintf(stderr, "Cannot open %s file.\n", f_name);
		return -1;
	}

	bsize = atoi(size);
	buf = malloc(bsize);
	if (buf == NULL) {
		fprintf(stderr, "Cannot malloc buf.\n");
		goto fail_write_close;
	}

	if (lseek(fd_servicecpu, atoi(off), SEEK_SET) < 0)
		goto fail_write_free;

	nr = read(rfd, buf, bsize);
	if (nr < 0) {
		fprintf(stderr, "Cannot read from %s.\n", f_name);
		goto fail_write_free;
	}

	nw = write(fd_servicecpu, buf, (size_t)nr);
	if (nw < 0) {
		fprintf(stderr, "Cannot write to %s.\n", f_name);
		goto fail_write_free;
	}

	free(buf);
	close(rfd);

	fprintf(stdout, "File name %s, size 0x%x\n", f_name, bsize);

	return 0;

fail_write_free:
	free(buf);
fail_write_close:
	close(rfd);
	return -1;
}

static int read_from_servicecpu(int fd_servicecpu, char *off, char *size)
{
	int wfd;
	ssize_t nr, nw, bsize;
	char *buf = NULL;

	wfd = fileno(stdout);

	bsize = atoi(size);
	buf = malloc(bsize);
	if (buf == NULL) {
		fprintf(stderr, "Cannot malloc buf.\n");
		return -1;
	}

	if (lseek(fd_servicecpu, atoi(off), SEEK_SET) < 0)
		goto fail_read_free;

	nr = read(fd_servicecpu, buf, bsize);
	if (nr < 0) {
		fprintf(stderr, "Cannot read from Service CPU.\n");
		goto fail_read_free;
	}

	nw = write(wfd, buf, nr);
	if (nw < 0) {
		fprintf(stderr, "Cannot write to stdout.\n");
		goto fail_read_free;
	}

	free(buf);

	return 0;

fail_read_free:
	free(buf);
	return -1;
}

int main(int argc, char *argv[])
{
	char *name = "/dev/servicecpu";
	int i, fd, rfd, rc = 0;
	unsigned int msg;
	char *mapped = NULL;
	int map_offset = 0;
	struct servicecpu_config config;
	struct servicecpu_hostmem_info hostmem_info;
#ifdef CONFIG_MV_SERVICECPU_IPC_SUPPORT
	struct servicecpu_ipcchn_set ipcchn_set;
	char dev_name[32];
#endif

	/* Specifying the expected options */
	static struct option long_options[] = {
		{"help",	no_argument,		0,	'h' },
		{"reset",	no_argument,		0,	't' },
		{"unreset",	no_argument,		0,	'u' },
		{"sendirq",	no_argument,		0,	's' },
		{"read",	required_argument,	0,	'r' },
		{"write",	required_argument,	0,	'w' },
		{"mmap",	required_argument,	0,	'm' },
		{"config",	required_argument,	0,	'c' },
#ifdef CONFIG_MV_SERVICECPU_IPC_SUPPORT
		{"setipcchn",	required_argument,	0,	'e' },
#endif
		{0,		required_argument,	0,	0}
	};
	int long_index = 0, opt = 0;

	if (argc < 2) {
		fprintf(stderr, "missing arguments\n");
		exit(2);
	}

	/* open the device */
	fd = open(name, O_RDWR);
	if (fd <= 0) {
		fprintf(stderr, "Cannot open %s device.\n", name);
		exit(1);
	}

#ifdef CONFIG_MV_SERVICECPU_IPC_SUPPORT
	opt = getopt_long(argc, argv, "mhtusr:w:c:e:", long_options, &long_index);
#else
	opt = getopt_long(argc, argv, "mhtusr:w:c:", long_options, &long_index);
#endif
	if (opt == -1)
		goto fail_close;

	switch (opt) {
	case 'h':
		show_usage();
		break;
	case 't': /* put ServiceCPU into reset */
		msg = FALSE;
		rc = ioctl(fd, SERVICECPU_IOC_UNRESET, &msg);
		break;
	case 'u': /* take ServiceCPU from reset */
		msg = TRUE;
		rc = ioctl(fd, SERVICECPU_IOC_UNRESET, &msg);
		break;
	case 's': /* send irq to ServiceCPU */
		rc = ioctl(fd, SERVICECPU_IOC_SENDIRQ, NULL);
		break;
	case 'r': /* read from ServiceCPU */
		if (argc < 4) {
			fprintf(stderr, "missing read arguments\n");
			goto fail_close;
		}
		rc = read_from_servicecpu(fd, argv[2], argv[3]);
		break;
	case 'w': /* write to ServiceCPU */
		if (argc < 5) {
			fprintf(stderr, "missing write arguments\n");
			goto fail_close;
		}
		rc = write_to_servicecpu(fd, argv[2], argv[3], argv[4]);
		break;
	case 'c': /* configure SRAM/DRAM segments */
		memset(&config, 0, sizeof(struct servicecpu_config));
		if (argc < 3 || (argc == 3 && (strcmp("APPLY", argv[2]) == 0)))
			config.config_flag = CONFIG_APPLY;
		else {
			if (strcmp("SRAM", argv[2]) == 0) {
				config.config_flag = CONFIG_SRAM;
				config.sram_config.seg_cache_size = strtol(argv[3], NULL, 16);
				config.sram_config.seg_free_size = strtol(argv[4], NULL, 16);
				config.sram_config.seg_hostmem_size = strtol(argv[5], NULL, 16);
				config.sram_config.seg_oamdb_size = strtol(argv[6], NULL, 16);
				config.sram_config.seg_shmem_size = strtol(argv[7], NULL, 16);
				config.sram_config.seg_code_size = strtol(argv[8], NULL, 16);
			} else if (strcmp("DRAM", argv[2]) == 0) {
				config.config_flag = CONFIG_DRAM;
				config.dram_config.seg_shmem_in_dram_offset = strtol(argv[3], NULL, 16);
				config.dram_config.seg_shmem_in_dram_size = strtol(argv[4], NULL, 16);
				config.dram_config.seg_hostmem_in_dram_offset = strtol(argv[5], NULL, 16);
				config.dram_config.seg_hostmem_in_dram_size = strtol(argv[6], NULL, 16);
			} else {
				fprintf(stderr, "unsupported arguments\n");
				goto fail_close;
			}
		}

		rc = ioctl(fd, SERVICECPU_IOC_CONFIG, &config);
		break;
	case 'm': /* mmap Host control memory */
		memset(&hostmem_info, 0, sizeof(struct servicecpu_hostmem_info));
		if (argc == 3 && (strcmp("info", argv[2]) == 0)) {
			rc = ioctl(fd, SERVICECPU_IOC_MMAPINFO, &hostmem_info);
			fprintf(stdout, "HOSTMEM:\n");
			fprintf(stdout, "	SRAM: addr=0x%-8x offset=0x%-8x size=0x%-8x\n",
						hostmem_info.items[0], 0, hostmem_info.items[1]);
			fprintf(stdout, "	DRAM: addr=0x%-8x offset=0x%-8x size=0x%-8x\n",
				hostmem_info.items[2], hostmem_info.items[2]-hostmem_info.items[0],
				hostmem_info.items[3]);
			break;
		}
		if (argc == 3)
			map_offset = strtol(argv[2], NULL, 16);
		mapped = mmap(NULL, _4K, PROT_READ | PROT_WRITE, MAP_SHARED, fd, map_offset);

		for (i = 0; i < 32; i++) {
			if (!(i % 16))
				fprintf(stdout, "\n%p: ", mapped);
			else if (!(i % 4))
				fprintf(stdout, "\t");

			fprintf(stdout, "%02x", *mapped++);
		}
		fprintf(stdout, "\n");

		munmap(mapped, _4K);

		break;
#ifdef CONFIG_MV_SERVICECPU_IPC_SUPPORT
	case 'e': /* set ipc link channel status, 0 disable, others are enable*/
		if (argc < 4) {
			fprintf(stderr, "missing write arguments\n");
			goto fail_close;
		}

		ipcchn_set.chn_id = strtol(argv[2], NULL, 10);
		ipcchn_set.chn_en = strtol(argv[3], NULL, 10);
		ipcchn_set.dev_name = dev_name;
		rc = ioctl(fd, SERVICECPU_IOC_SET_IPCCHN, &ipcchn_set);

		fprintf(stdout, "serviceCPU: %s ipc channel %d (%s)\n",
			ipcchn_set.chn_en ? "enable" : "disable",
			ipcchn_set.chn_id, ipcchn_set.dev_name);
		break;
#endif
	default:
		show_usage();
		break;
	}

	if (rc < 0) {
		fprintf(stderr, "ServiceCPU Tool failed to perform action!\n");
		goto fail_close;
	}

	close(fd);

	return 0;

fail_close:
	close(fd);
	exit(2);
}
