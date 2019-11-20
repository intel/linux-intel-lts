Module: HANTRO DRIVER
Platform: THUNDER BAY HARBOR
Date: 11 NOV 2019

1. To make driver module:
	make clean
	make
Driver module hantrodriver.ko is then created.
2. Insert our driver to kernel:
	sudo insmod hantrodriver.ko
  Now our driver is ready for use. and "/dev/dri/cardX" and /dev/dri/renderD12X" is created.
3. Use our driver:
	Using libdrm's API drmOpen:
		int fd = drmOpen("hantro", NULL);
	Then fd is the handle to access our kernel driver.
4. check style before sumbit.
	scripts/checkpatch.pl --no-tree -f *.c *.h Makefile*
	scripts/checkpatch.pl --no-tree --summary-file -f *.c *.h Makefile* | grep 'lines checked'
