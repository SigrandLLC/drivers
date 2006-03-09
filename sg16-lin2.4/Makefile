ifeq ("$(KERNELSOURCEPATH)","")
KERNELSOURCEPATH = /usr/src/linux
endif
ifeq ("$(KERNELVERSION)","")
KERNELVERSION = `uname -a | cut -f 3 -d ' '`
endif
ifeq ("$(MODULEPATH)","")
MODULEPATH = /lib/modules/$(KERNELVERSION)/kernel/drivers/net/sg16lan
endif

CFLAGS = -D__KERNEL__ -I$(KERNELSOURCEPATH)/include -Wall \
	-Wstrict-prototypes -O2 -fomit-frame-pointer -fno-strict-aliasing \
	-pipe -fno-strength-reduce -fno-common -mpreferred-stack-boundary=2 \
	-falign-loops=2 -falign-jumps=2 -falign-functions=2 -DMODULE \
	-DKBUILD_BASENAME=sg16lan

ARCH = `uname -m`

sg16lan.o:	sg16lan.c cx28975.h
	gcc $(CFLAGS) -march=$(ARCH) -c -o sg16lan.o sg16lan.c

sg16cfg:	sg16cfg.c
	gcc -O2 -g -o sg16cfg sg16cfg.c

install:	sg16lan.o sg16cfg
	install -m 644 sg16lan.o $(MODULEPATH)
	install -s sg16cfg /sbin

all:	sg16lan.o sg16cfg
	

