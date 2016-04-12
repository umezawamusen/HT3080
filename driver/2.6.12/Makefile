# The path to the kernel source (should have been compiled first)
KERNELDIR ?= ../atmark-dist-20101220/linux-2.6.x

ifneq ($(KERNELRELEASE),)

obj-m := xr1678x.o

else

PWD := $(shell pwd)

modules:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules

endif

clean:
	rm -rf *.o *~ core .*.cmd *.ko *.mod.c .tmp_versions





