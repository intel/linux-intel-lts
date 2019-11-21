obj-m := hantrodriver.o
hantrodriver-objs = hantro.o hx280enc.o hantrodec.o hantro_fence.o hantrocache.o slicemgr.o
KDIR := /* please change this to your kernel-source dir */
PWD := $(shell pwd)
default:
	$(MAKE) -C $(KDIR) M=$(PWD) modules CFLAGS_hantro.o="-DUSE_CMA=0 -DUSE_HW=1"

clean::
	$(MAKE) -C $(KDIR) M=$(PWD) clean
	-rm -f .version





