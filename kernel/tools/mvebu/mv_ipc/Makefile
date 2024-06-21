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

TARGETS += mv_ipc_tool
CFLAGS += -I$(KSRC)/drivers/misc/mvebu/mv_ipc/
CFLAGS += -Wall

mv_ipc_tool:mv_ipc_tool.c
	$(CC) $(CFLAGS) -static mv_ipc_tool.c -o mv_ipc_tool

clean:
	rm -f $(TARGETS) *.o

