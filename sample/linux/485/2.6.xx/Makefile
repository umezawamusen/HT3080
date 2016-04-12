CC=arm-linux-gcc
EXEC = test485

SRCS = test.c 3080.c
OBJS = $(SRCS:.c=.o)

LFLAGS += -lpthread

CFLAGS += -I$(ROOTDIR)/$(LINUXDIR)/include

all: $(EXEC)

$(EXEC): $(OBJS)
	$(CC) $(LDFLAGS) -o $@ $(OBJS) $(LDLIBS) $(LFLAGS)

romfs:
	$(ROMFSINST) /bin/$(EXEC)

clean:
	-rm -f $(EXEC) *.elf *.gdb *.o

