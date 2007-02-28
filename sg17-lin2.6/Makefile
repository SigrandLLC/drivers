obj-y += sdfe4_lib/
obj-m += sg17lan.o
sg17lan-objs := sg17main.o sg17sci.o sdfe4_lib/sdfe4_lib.o sg17debug.o sg17ring.o sg17sysfs.o
KERNELVERSION=`uname -r`
#MODULEPATH=/lib/modules/$(KERNELVERSION)/kernel/drivers/net/sg16lan.ko
CFLAGS+=

sg17lan.ko: sg17main.c sg17sci.c sg17debug.c sg17ring.c sdfe4_lib/sdfe4_lib.o
	make -C ./linux-src SUBDIRS=$(PWD)  modules


sdfe4_lib/sdfe4_lib.o:
	make -C ./linux-src SUBDIRS=$(PWD)


all: sg17lan.ko 


#install: sg17lan.ko
#	install -m 644 sg16lan.ko $(MODULEPATH)
#	install -m 744 sg16cfg /sbin
#	cp ./firmware/sg16.bin /lib/firmware/

clean:
	make -C sdfe4_lib clean
	rm --preserve-root -f *.o*
	rm --preserve-root -f .*.o*
	rm --preserve-root -f *.ko*
	rm --preserve-root -f *.mod.*
	rm --preserve-root -f .sg17*
	rm --preserve-root -f -R .tmp*
