#export KBUILD_VERBOSE=1
#export ARCH=arm64
#export CROSS_COMPILE=aarch64-linux-gnu-

obj-m := hantrodriver.o
hantrodriver-objs = hantro.o hx280enc.o hantrodec.o hantro_fence.o hantrocache.o hantrodec400.o slicemgr.o
KDIR := /* Path to kernel source */ 
PWD := $(shell pwd)
default:
	$(MAKE) -C $(KDIR) M=$(PWD) modules CFLAGS_hantro.o="-DUSE_CMA=0 -DUSE_HW=1 "

clean::
	$(MAKE) -C $(KDIR) M=$(PWD) clean
	-rm -f .version





