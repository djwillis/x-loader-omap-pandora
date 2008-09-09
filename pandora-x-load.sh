#!/bin/sh
rm MLO
rm x-load.bin.ift
make CROSS_COMPILE=arm-none-linux-gnueabi- distclean 
make CROSS_COMPILE=arm-none-linux-gnueabi- pandora_config
make CROSS_COMPILE=arm-none-linux-gnueabi- 
cp x-load.bin ../x-load-usb.bin
../signGP/signGP x-load.bin
cp x-load.bin.ift ../MLO
