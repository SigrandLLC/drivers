obj-m += sg17lan.o
sg17lan-objs := sg17main.o sg17sci.o sdfe4_lib.o sg17debug.o sg17ring.o sg17sysfs.o
KERNELVERSION=`uname -r`
#MODULEPATH=/lib/modules/$(KERNELVERSION)/kernel/drivers/net/sg16lan.ko
CFLAGS+=

sg17lan.ko: sg17main.c sg17sci.c sg17debug.c sg17ring.c sdfe4_lib.c
	make -C ./linux-src SUBDIRS=$(PWD)  modules

all: sg17lan.ko 

clean:
	rm --preserve-root -f *.o*
	rm --preserve-root -f .*.o*
	rm --preserve-root -f *.ko*
	rm --preserve-root -f *.mod.*
	rm --preserve-root -f .sg17*
	rm --preserve-root -f -R .tmp*
