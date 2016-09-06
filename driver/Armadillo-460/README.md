# Armadillo-460用HT3080ドライバ構築

## 環境
* URL
http://armadillo.atmark-techno.com/armadillo-460/downloads

* ATDE3
V20120709

* ユーザランド
V20160126

* Linuxカーネル
V2.6.26-at25

## 環境構築情報
[Atmark-dist開発ガイド](http://armadillo.atmark-techno.com/files/downloads/dist/atmark-dist_developers_guide_ja-1.0.9.pdf)
11.新規デバイスドライバの追加方法
のMakefileを用いて作成

##コンパイル
```
$ make CROSS_COMPILE=arm-linux-gnueabi-
```

## デバイスノード追加
mknod -m 666 /dev/ttyXR78x0 c 40 0  
mknod -m 666 /dev/ttyXR78x1 c 40 1  
mknod -m 666 /dev/ttyXR78x2 c 40 2  
mknod -m 666 /dev/ttyXR78x3 c 40 3  
mknod -m 666 /dev/ttyXR78x4 c 40 4  
mknod -m 666 /dev/ttyXR78x5 c 40 5  
mknod -m 666 /dev/ttyXR78x6 c 40 6  
mknod -m 666 /dev/ttyXR78x7 c 40 7  

# デバイスドライバのインストール
```
# insmod xr1678x.ko io=0xXXX(アドレス0x000～0xF00) irq=x(2～7) major=xx
```
(１０進数、省略可)
```
ex)insmod xr1678.ko io=0x300 irq=5 major=40　　
ex複数枚)insmod xr1678.ko io=0x300,0x400 irq=5,6
メジャー番号の標準は40としてある。指定された場合、デバイスノードのメジャー番号を注意すること。
```



