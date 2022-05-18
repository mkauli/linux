#!/bin/sh

TMP_DIR=bootfs_tmp

[ ! -d "$TMP_DIR" ] && mkdir -p $TMP_DIR

cp zImage $TMP_DIR/
cp am335x-galileo-33xx.dtb $TMP_DIR/

fakeroot /bin/sh <<EOF
  chown -R root:root $TMP_DIR
 
  /usr/sbin/mkfs.ubifs -r $TMP_DIR -m 2048 -e 126976 -c 65 -o ams1xx-bootfs.ubifs
EOF

rm -rf $TMP_DIR
