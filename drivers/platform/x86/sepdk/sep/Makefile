ccflags-y := -I$(src)/../include -I$(src)/../inc -I$(src)/..
ccflags-y += -DSEP_CONFIG_MODULE_LAYOUT
# TODO: verify kaiser.h
#ccflags-y += -DKAISER_HEADER_PRESENT
ccflags-y += -DDRV_CPU_HOTPLUG -DDRV_USE_TASKLET_WORKAROUND

asflags-y := -I$(src)/..

ifdef CONFIG_SEP_PER_USER_MODE
	ccflags-y += -DSECURE_SEP
endif

ifdef CONFIG_SEP_MINLOG_MODE
	ccflags-y += -DDRV_MINIMAL_LOGGING
endif

ifdef CONFIG_SEP_MAXLOG_MODE
	ccflags-y += -DDRV_MAXIMAL_LOGGING
endif

ifdef CONFIG_SEP_PRIVATE_BUILD
	ccflags-y += -DENABLE_CPUS -DBUILD_CHIPSET -DBUILD_GFX
endif

ifdef CONFIG_SEP_ACRN
	ccflags-y += -DDRV_SEP_ACRN_ON
endif

obj-$(CONFIG_SEP)		+= sep5.o

sep5-y	:=	lwpmudrv.o        \
		control.o         \
		cpumon.o          \
		eventmux.o        \
		linuxos.o         \
		output.o          \
		pmi.o             \
		sys_info.o        \
		utility.o         \
		valleyview_sochap.o    \
		unc_power.o	  \
		core2.o           \
		perfver4.o        \
		silvermont.o      \
		pci.o             \
		apic.o            \
		pebs.o            \
		unc_gt.o          \
		unc_mmio.o        \
		unc_msr.o         \
		unc_common.o      \
		unc_pci.o         \
		sepdrv_p_state.o


ifdef CONFIG_X86_64
	sep5-y	+=	sys64.o
endif

ifdef CONFIG_X86_32
	sep5-y	+=	sys32.o
endif

sep5-$(CONFIG_SEP_PRIVATE_BUILD)	+=	chap.o    \
						gmch.o    \
						gfx.o     \
						unc_sa.o
