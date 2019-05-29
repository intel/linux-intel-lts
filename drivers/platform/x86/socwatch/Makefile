#
# Makefile for the socwatch driver.
#

DRIVER_BASE=socwatch
DRIVER_MAJOR=2
DRIVER_MINOR=10
# basic name of driver
DRIVER_NAME=${DRIVER_BASE}${DRIVER_MAJOR}_${DRIVER_MINOR}

DO_DRIVER_PROFILING=0

ccflags-y :=	-I$(srctree)/drivers/platform/x86/socwatch/inc
ccflags-y +=	-DDO_DRIVER_PROFILING=$(DO_DRIVER_PROFILING)

obj-$(CONFIG_INTEL_SOCWATCH)    += $(DRIVER_NAME).o

$(DRIVER_NAME)-objs	:= sw_driver.o sw_hardware_io.o \
			sw_output_buffer.o sw_tracepoint_handlers.o \
			sw_mem.o sw_collector.o sw_telem.o \
			sw_file_ops.o sw_internal.o sw_ops_provider.o \
			sw_reader.o sw_trace_notifier_provider.o
