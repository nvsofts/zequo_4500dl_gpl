obj-$(CONFIG_IRQCHIP)			+= irqchip.o

obj-$(CONFIG_ARCH_BCM2835)		+= irq-bcm2835.o
obj-$(CONFIG_ARCH_EXYNOS)		+= exynos-combiner.o
obj-$(CONFIG_ARCH_MVEBU)		+= irq-armada-370-xp.o
obj-$(CONFIG_MACH_MSYS)			+= irq-mv-msys-switch.o
obj-$(CONFIG_MACH_MSYS)			+= irq-mv-msys/irq-mv-msys-switch-mg.o
obj-$(CONFIG_ARCH_MXS)			+= irq-mxs.o
obj-$(CONFIG_ARCH_S3C24XX)		+= irq-s3c24xx.o
obj-$(CONFIG_METAG)			+= irq-metag-ext.o
obj-$(CONFIG_METAG_PERFCOUNTER_IRQS)	+= irq-metag.o
obj-$(CONFIG_ARCH_SUNXI)		+= irq-sun4i.o
obj-$(CONFIG_ARCH_SPEAR3XX)		+= spear-shirq.o
obj-$(CONFIG_ARM_GIC)			+= irq-gic.o
obj-$(CONFIG_ARM_VIC)			+= irq-vic.o
obj-$(CONFIG_SIRF_IRQ)			+= irq-sirfsoc.o
obj-$(CONFIG_RENESAS_INTC_IRQPIN)	+= irq-renesas-intc-irqpin.o
obj-$(CONFIG_RENESAS_IRQC)		+= irq-renesas-irqc.o
obj-$(CONFIG_VERSATILE_FPGA_IRQ)	+= irq-versatile-fpga.o
obj-$(CONFIG_ARCH_VT8500)		+= irq-vt8500.o

# create subdirs for all objects
override obj-dirs := $(dir $(obj)) $(addprefix $(obj)/, $(sort $(dir $(filter %.o,$(obj-y)))))
