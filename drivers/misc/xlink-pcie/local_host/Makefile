ccflags-y += -Wall -Wno-unused-function -Werror -DXLINK_PCIE_LOCAL=1
ccflags-y += -I$(srctree)/drivers/pci/controller/dwc

obj-$(CONFIG_XLINK_PCIE_LH_DRIVER) += mxlk_ep.o
mxlk_ep-objs := mxlk_epf.o
mxlk_ep-objs += mxlk_core.o
mxlk_ep-objs += mxlk_dma.o
mxlk_ep-objs += mxlk_inf.o
mxlk_ep-objs += ../common/mxlk_util.o
