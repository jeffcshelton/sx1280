obj-m += sx1280.o

KERNELRELEASE ?= $(shell uname -r)
KERNELDIR ?= /lib/modules/$(KERNELRELEASE)/build
PWD := $(shell pwd)
ARCH ?= $(shell uname -m | sed -e s/x86_64/x86/ -e s/aarch64/arm64/)

ccflags-y := -Wall

all: modules

modules:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) modules

clean:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) clean

install:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) ARCH=$(ARCH) INSTALL_MOD_PATH=$(INSTALL_MOD_PATH) modules_install
	depmod -a

insmod:
	sudo insmod sx1280.ko

rmmod:
	sudo rmmod sx1280

.PHONY: all modules clean install insmod rmmod
