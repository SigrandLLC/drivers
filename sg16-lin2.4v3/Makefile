ifeq ("$(KERNELSOURCEPATH)","")
KERNELSOURCEPATH = /usr/src/linux
endif
ifeq ("$(KERNELVERSION)","")
KERNELVERSION = `uname -a | cut -f 3 -d ' '`
endif
ifeq ("$(MODULEPATH)","")
MODULEPATH = /lib/modules/$(KERNELVERSION)/kernel/drivers/net/sg16lan
endif

export KERNELSOURCEPATH

sg16lan/sg16lan.o: sg16lan/sg16lan.c sg16lan/sg16lan.h sg16lan/sg16debug.h sg16lan/cx28975.h
	make -C $(KERNELSOURCEPATH) SUBDIRS="$(PWD)/sg16lan" modules

_sg16fw_dload: sg16fw_dload.c
	gcc -O2 -o _sg16fw_dload sg16fw_dload.c

all: sg16lan.o _sg16fw_dload

install: sg16lan/sg16lan.o _sg16fw_dload sg16cfg
	install -m 644 sg16lan/sg16lan.o $(MODULEPATH)
	install -m 744 sg16cfg /sbin
	install -s _sg16fw_dload /sbin	

all:	sg16lan.o _sg16fw_dload sg16cfg


clean:
	rm -f sg16lan/sg16lan.o
	rm -f sg16lan/.sg16lan.o.flags
	rm -f _sg16fw_dload



