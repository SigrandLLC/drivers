obj-m := mr16g_hdlc.o
KERNELVERSION=`uname -r`
CFLAGS+=
LINDIR=/home/artpol/router/kernels/linux-2.6.16-2/
mr16g_hdlc.ko: mr16g_hdlc.c
	make -C $(LINDIR) SUBDIRS=$(PWD) modules


all: mr16g_hdlc.ko

clean:
	rm --preserve-root -f *.o*
	rm --preserve-root -f *.ko*
	rm --preserve-root -f *.mod.*
	rm --preserve-root -f .sg16*
	rm --preserve-root -f -R .tmp*


