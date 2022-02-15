#!/bin/bash
# vim: et sts=2 sw=2 ts=2 si

/bin/bash --init-file <( \
  cat /etc/profile ~/.bashrc; \
  echo 'source /opt/ssd/3.4.1/environment-setup-cortexa8hf-neon-ssd-linux-gnueabi'; \
  echo 'export ARCH=arm'; \
  echo 'export CROSS_COMPILE=$TARGET_PREFIX'; \
  echo 'unset LDFLAGS'; \
)

