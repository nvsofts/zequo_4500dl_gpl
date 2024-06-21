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

ifdef CONFIG_MV_DRAGONITE_IPC_SUPPORT
CFLAGS += -DCONFIG_MV_DRAGONITE_IPC_SUPPORT
endif

CC := $(CROSS_COMPILE)gcc
LD := $(CROSS_COMPILE)ld

ifeq ($(CONFIG_MV_INCLUDE_DRAGONITE_XCAT),y)
TARGETS += mv_dragonite_tool
CFLAGS += -I$(KSRC)/drivers/misc/mvebu/mv_dragonite/
CFLAGS += -Wall
endif


mv_dragonite_tool:mv_dragonite_tool.c
	$(CC) $(CFLAGS) -static mv_dragonite_tool.c -o mv_dragonite_tool

clean:
	rm -f $(TARGETS) *.o

