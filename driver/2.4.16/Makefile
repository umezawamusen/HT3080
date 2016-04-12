INCLUDEDIR = ../linux-2.4.16-rmk2-armadillo-3/include
CC=arm-linux-gcc
CPPFLAGS =-I$(INCLUDEDIR) -DCONFIG_ARCH_ARMADILLO -Uarm

# Don't remove "-O3" or bad things will happen!
CFLAGS = -O3 -Wall -Wstrict-prototypes -Wa,--no-warn -pipe
CPPFLAGS += -D__KERNEL__ -DMODULE

SRCS    = xr1678x.c 
MODULES = xr1678x.o 
EXTRA   =


all:	$(MODULES) $(EXTRA)


clean:
	rm -f core core.* *.o .*.o *.s *.a *~ .depend .depfiles/*.d

%.o : %.c
	$(CC) -MD $(CFLAGS) $(CPPFLAGS) -c $<
	@mkdir -p .depfiles ; mv $*.d .depfiles

