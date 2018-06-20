ifeq ($(KERNELRELEASE),)

    KERNELDIR ?= /home/alex/Zynq/kernel/buildroot/output/build/linux-4.16
    PWD := $(shell pwd)

modules:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules

modules_install:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules_install

clean:
	rm -rf *.o *~ core .depend .*.cmd *.ko *.mod.c .tmp_versions

.PHONY: modules modules_install clean

else

    obj-m := int_driver.o
endif


