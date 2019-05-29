
# Makefile for the socwatch hv driver.
#

DRIVER_BASE=socwatchhv
DRIVER_MAJOR=2
DRIVER_MINOR=0
# basic name of driver
DRIVER_NAME=${DRIVER_BASE}${DRIVER_MAJOR}_${DRIVER_MINOR}

HYPERVISOR=2 # ACRN

EXTRA_CFLAGS +=	-Idrivers/ \
		-I$(src)/inc/ \
		-DHYPERVISOR=$(HYPERVISOR)

obj-$(CONFIG_INTEL_SOCWATCH_HV)    += $(DRIVER_NAME).o

$(DRIVER_NAME)-objs :=	swhv_driver.o \
			swhv_acrn.o
