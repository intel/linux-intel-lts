KERNEL_VERSION := $(shell echo $(KERNELRELEASE) | cut -d- -f1 | sed -r 's/([0-9]+\.[0-9]+).*/\1.0/g')

ccflags-y += -I$(src)/../../../../$(KERNEL_VERSION)/include-overrides

obj-m 		+= max9x.o
max9x-y += serdes.o regmap-retry.o max9296.o max96724.o max9295.o max96717.o
