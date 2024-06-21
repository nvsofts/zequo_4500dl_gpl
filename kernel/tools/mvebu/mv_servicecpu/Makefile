# kernel sources
srctree  := $(shell /bin/pwd)

ifndef KSRC
KSRC  := $(srctree)/../../..
endif

include $(KSRC)/.config

CFLAGS  := $(EXTRA_CFLAGS)

ifdef CONFIG_CPU_BIG_ENDIAN
CFLAGS  += -DCONFIG_CPU_BIG_ENDIAN
endif

ifdef CONFIG_MV_SERVICECPU_IPC_SUPPORT
CFLAGS += -DCONFIG_MV_SERVICECPU_IPC_SUPPORT
endif

CC := $(CROSS_COMPILE)gcc
LD := $(CROSS_COMPILE)ld

SUBDIRS +=

TARGETS =

ifeq ($(CONFIG_MV_INCLUDE_SERVICECPU),y)
TARGETS += mv_servicecpu_tool
CFLAGS += -I$(KSRC)/drivers/misc/mvebu/mv_servicecpu/
ifeq ($(CONFIG_MV_SERVICECPU_FIRMWARE_IN_SRAM),y)
CFLAGS += -DCONFIG_MV_SERVICECPU_FIRMWARE_IN_SRAM
else
CFLAGS += -DCONFIG_MV_SERVICECPU_FIRMWARE_IN_DRAM
endif
endif

all : Makefile $(TARGETS)
	@for subdir in $(SUBDIRS); do \
                (cd $$subdir && $(MAKE) $@) || exit 1; \
        done

mv_servicecpu_tool:mv_servicecpu_tool.c
	$(CC) $(CFLAGS) -static mv_servicecpu_tool.c -o mv_servicecpu_tool

clean:
	rm -f $(TARGETS) *.o
	@for subdir in $(SUBDIRS); do \
                (cd $$subdir && $(MAKE) $@) || exit 1; \
        done
