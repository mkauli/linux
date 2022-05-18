#!/bin/sh

cp arch/arm/boot/zImage /tftp/zImage
cp arch/arm/boot/dts/am335x-galileo-33xx.dtb /tftp/oftree

cp arch/arm/boot/zImage .
cp arch/arm/boot/dts/am335x-galileo-33xx.dtb .
