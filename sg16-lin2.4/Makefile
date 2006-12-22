ifeq ("$(KERNELSOURCEPATH)","")
KERNELSOURCEPATH = /usr/src/linux
endif
ifeq ("$(KERNELVERSION)","")
KERNELVERSION = `uname -a | cut -f 3 -d ' '`
endif
ifeq ("$(MODULEPATH)","")
MODULEPATH = /lib/modules/$(KERNELVERSION)/kernel/drivers/net/sg16lan
endif

CFLAGS = -D__KERNEL__ -I$(KERNELSOURCEPATH)/include \
	-Wall -Wstrict-prototypes -Wno-trigraphs -O2 -fno-strict-aliasing -fno-common \
	-fomit-frame-pointer -pipe -mpreferred-stack-boundary=2 \
	-DMODULE -nostdinc -iwithprefix include \
	-DKBUILD_BASENAME=sg16lan

#	-falign-loops=2 -falign-jumps=2 -falign-functions=2  \


ARCH = `uname -m`

sg16lan.o:	sg16lan.c cx28975.h
	gcc $(CFLAGS) -march=$(ARCH) -c -o sg16lan.o sg16lan.c

sg16fw_dload: sg16fw_dload.c
	gcc -O2 -o sg16fw_dload sg16fw_dload.c

install:	sg16lan.o sg16fw_dload sg16cfg
	install -m 644 sg16lan.o $(MODULEPATH)
	install -m 744 sg16cfg /sbin
	install -s sg16fw_dload /sbin	

all:	sg16lan.o sg16fw_dload sg16cfg


clean:
	rm --preserve-root -f sg16lan.o
	rm --preserve-root -f sg16fw_dload
