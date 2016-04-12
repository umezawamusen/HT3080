#!/bin/sh

mv /home/ftp/pub/libpthread-0.10.so /lib/
ln -s /lib/libpthread-0.10.so /lib/libpthread.so.0


mknod -m 666 /dev/ttyXR78x0 c 40 0
mknod -m 666 /dev/ttyXR78x1 c 40 1
mknod -m 666 /dev/ttyXR78x2 c 40 2
mknod -m 666 /dev/ttyXR78x3 c 40 3
mknod -m 666 /dev/ttyXR78x4 c 40 4
mknod -m 666 /dev/ttyXR78x5 c 40 5
mknod -m 666 /dev/ttyXR78x6 c 40 6
mknod -m 666 /dev/ttyXR78x7 c 40 7

insmod xr1678x.ko io=0x500 irq=4
