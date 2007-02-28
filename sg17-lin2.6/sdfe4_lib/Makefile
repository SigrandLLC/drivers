obj-y += sdfe4_lib.o 

sdfe4_lib.o: sdfe4_lib.c
	make -C $(KERNEL_SRC) SUBDIRS=$(PWD)

clean:
	rm --preserve-root -f *.o*
	rm --preserve-root -f .*.o*
	rm --preserve-root -f *.ko*
	rm --preserve-root -f *.mod.*
	rm --preserve-root -f .sg17*
	rm --preserve-root -f -R .tmp*
