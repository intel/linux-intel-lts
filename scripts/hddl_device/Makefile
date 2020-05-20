ifndef KERNELSRC
	KERNELSRC=/lib/modules/$(shell uname -r)/build
endif

obj-y += ../../drivers/misc/xlink-core/
obj-y += ../../drivers/misc/xlink-pcie/
obj-y += ../../drivers/misc/hddl_device/
obj-y += ../../drivers/misc/host_kmb_tj/
obj-y += ../../drivers/misc/xlink-smbus/
obj-y += ../../drivers/misc/lm75/


subdir-ccflags-y += -I$(src)/include
subdir-ccflags-y += -DCONFIG_XLINKI2C_ADAPTER

ifdef DEBUG
	subdir-ccflags-y += -DXLINK_PCIE_DEBUG
endif

PHONY: help
help:
	@echo ""
	@echo "make remote_hddl -> builds remote hddl_device driver"
	@echo "make clean -> delete build artifacts"
	@echo ""
	@echo "If needed the cross-compile environment should be set up first"
	@echo "Set KERNELSRC if use other source tree, otherwise using the current running kernel"
	@echo ""

remote_hddl:
	make -C $(KERNELSRC) M=$(PWD) modules CONFIG_HDDL_DEVICE=m  CONFIG_HDDL_LOCAL_HOST=n
	cp ../../drivers/misc/hddl_device/hddl_device.ko .

host_kmb_tj:
	make -C $(KERNELSRC) M=$(PWD) modules CONFIG_HOST_KMB_TJ=m
	cp ../../drivers/misc/host_kmb_tj/host_kmb_tj.ko .

xlink_smbus:
	make -C $(KERNELSRC) M=$(PWD) modules CONFIG_XLINK_SMBUS=m CONFIG_XLINKI2C_ADAPTER=y
	cp ../../drivers/misc/xlink-smbus/xlink-smbus.ko .

lm75:
	make -C $(KERNELSRC) M=$(PWD) modules CONFIG_SENSORS_LM75=m
	cp ../../drivers/misc/lm75/lm75.ko .

clean:
	make -C $(KERNELSRC) M=$(PWD) clean
