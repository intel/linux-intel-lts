ccflags-y += -Wall -Wno-unused-function -Werror -DXLINK_PCIE_REMOTE=1

obj-$(CONFIG_XLINK_PCIE_RH_DRIVER) += mxlk.o
mxlk-objs := mxlk_main.o
mxlk-objs += mxlk_pci.o
mxlk-objs += mxlk_core.o
mxlk-objs += mxlk_inf.o
mxlk-objs += ../common/mxlk_util.o
