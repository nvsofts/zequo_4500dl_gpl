#
# Makefile for mvebu misc devices.
#
CPU_ARCH    = ARM
ifeq ($(CONFIG_CPU_BIG_ENDIAN),y)
ENDIAN      = BE
else
ENDIAN      = LE
endif

MVEBU_SCPU_FLAGS := -DMV_LINUX -DMV_CPU_$(ENDIAN) -DMV_$(CPU_ARCH)
export MVEBU_SCPU_FLAGS

ccflags-y       += $(MVEBU_SCPU_FLAGS)

ifeq ($(CONFIG_ARCH_MVEBU),y)
INCLUDE_DIRS	+= -I$(srctree)/arch/arm/mach-mvebu/include
INCLUDE_DIRS	+= -I$(srctree)/drivers/net/ethernet/mvebu_net/prestera/pci
else
INCLUDE_DIRS	+= -I$(srctree)/arch/arm/plat-armada
endif

ccflags-y	+= $(INCLUDE_DIRS)

obj-$(CONFIG_MVEBU_CM3)	+= mv_cm3/mv_cm3.o
obj-$(CONFIG_MSYS_ADDR_COMPL)	+= mv_addr_compl/mv_addr_compl.o
obj-$(CONFIG_MV_INCLUDE_SERVICECPU) += mv_servicecpu/servicecpu.o
obj-$(CONFIG_MV_INCLUDE_DRAGONITE_XCAT)	+= mv_dragonite/dragonite_xcat.o

#kernel<->freeRTOS AMP mode
obj-$(CONFIG_MV_IPC_FREERTOS_DRIVER) += mv_ipc/mv_ipc.o mv_ipc/mv_ipc_wrapper.o

ccflags-$(CONFIG_MV_IPC_FREERTOS_DRIVER) += -Idrivers/misc/mvebu/mv_ipc

# create subdirs for all objects
override obj-dirs := $(dir $(obj)) $(addprefix $(obj)/, $(sort $(dir $(filter %.o,$(obj-y)))))
