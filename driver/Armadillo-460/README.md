# Armadillo-460�pHT3080�h���C�o�\�z

## ��
* URL
http://armadillo.atmark-techno.com/armadillo-460/downloads

* ATDE3
V20120709

* ���[�U�����h
V20160126

* Linux�J�[�l��
V2.6.26-at25

## ���\�z���
[Atmark-dist�J���K�C�h](http://armadillo.atmark-techno.com/files/downloads/dist/atmark-dist_developers_guide_ja-1.0.9.pdf)
11.�V�K�f�o�C�X�h���C�o�̒ǉ����@
��Makefile��p���č쐬

##�R���p�C��
```
$ make CROSS_COMPILE=arm-linux-gnueabi-
```

## �f�o�C�X�m�[�h�ǉ�
mknod -m 666 /dev/ttyXR78x0 c 40 0
mknod -m 666 /dev/ttyXR78x1 c 40 1
mknod -m 666 /dev/ttyXR78x2 c 40 2
mknod -m 666 /dev/ttyXR78x3 c 40 3
mknod -m 666 /dev/ttyXR78x4 c 40 4
mknod -m 666 /dev/ttyXR78x5 c 40 5
mknod -m 666 /dev/ttyXR78x6 c 40 6
mknod -m 666 /dev/ttyXR78x7 c 40 7

# �f�o�C�X�h���C�o�̃C���X�g�[��
```
# insmod xr1678x.ko io=0xXXX(�A�h���X0x000�`0xF00) irq=x(2�`7) major=xx
```
(�P�O�i���A�ȗ���)
ex)insmod xr1678.ko io=0x300 irq=5 major=40
ex������)insmod xr1678.ko io=0x300,0x400 irq=5,6
���W���[�ԍ��̕W����40�Ƃ��Ă���B�w�肳�ꂽ�ꍇ�A�f�o�C�X�m�[�h�̃��W���[�ԍ��𒍈ӂ��邱�ƁB



