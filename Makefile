obj-m += sx1280.o

hello:
	make -C $(KDIR) M=$(PWD) modules CC=$(CC)

clean:
	make -C $(KDIR) M=$(PWD) clean
