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

CC := $(CROSS_COMPILE)gcc
LD := $(CROSS_COMPILE)ld

# TODO: make distinction between dragonite and cm3 in the future
ifeq ($(CONFIG_MVEBU_CM3),y)
TARGETS += mv_cm3_tool
CFLAGS += -I$(KSRC)/drivers/misc/mvebu/mv_cm3/
CFLAGS += -DCONFIG_MV_CM3
CFLAGS += -Wall

ifeq ($(CONFIG_MV_CM3_IPC_SUPPORT),y)
CFLAGS += -DCONFIG_MV_CM3_IPC_SUPPORT
endif

endif


mv_cm3_tool:mv_cm3_tool.c
	$(CC) $(CFLAGS) -static mv_cm3_tool.c -o mv_cm3_tool

clean:
	rm -f $(TARGETS) *.o

