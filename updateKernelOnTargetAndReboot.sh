#!/bin/bash

sshpass -p "$1" scp -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null arch/arm/boot/zImage root@192.168.60.101:/data/zImage.img
sshpass -p "$1" scp -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null arch/arm/boot/dts/am335x-galileo-33xx.dtb root@192.168.60.101:/data/oftree.img

sshpass -p "$1" ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null root@192.168.60.101 "rauc write-slot kernel.0 /data/zImage.img"
sshpass -p "$1" ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null root@192.168.60.101 "rauc write-slot dtb.0 /data/oftree.img"

sshpass -p "$1" ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null root@192.168.60.101 "sync && reboot"

