ifeq ("$(KERNELSOURCEPATH)","")
KERNELSOURCEPATH = /usr/src/linux
endif
ifeq ("$(KERNELVERSION)","")
KERNELVERSION = `uname -a | cut -f 3 -d ' '`
endif
ifeq ("$(MODULEPATH)","")
MODULEPATH = /lib/modules/$(KERNELVERSION)/net/sg16lan
endif

CFLAGS = -D__KERNEL__ -I$(KERNELSOURCEPATH)/include -Wall \
	-Wstrict-prototypes -O2 -fomit-frame-pointer -fno-strict-aliasing \
	-pipe -fno-strength-reduce -m486 \
	-malign-loops=2 -malign-jumps=2 -malign-functions=2 -DMODULE

SMP = $(shell if [ "`grep SMP /proc/version`" != "" ] ; then echo "-D__SMP__" ; fi)

sg16.o:	sg16.c cx28975.h
	gcc $(CFLAGS) $(SMP) -c -o sg16.o sg16.c

sg16cfg:	sg16cfg.c
	gcc -O2 -o sg16cfg sg16cfg.c

install:	sg16.o sg16cfg
	install -m 644 sg16.o $(MODULEPATH)
	install -s sg16cfg /sbin

