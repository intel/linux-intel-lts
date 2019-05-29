ccflags-y := -I$(srctree)/drivers/platform/x86/socperf/include -I$(srctree)/drivers/platform/x86/socperf/inc

obj-$(CONFIG_SOCPERF) += socperf3.o

socperf3-y := 	socperfdrv.o	\
		control.o	\
		utility.o	\
		pci.o		\
		soc_uncore.o	\
		haswellunc_sa.o	\
		npk_uncore.o

