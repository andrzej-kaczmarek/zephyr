BOARD ?= qemu_x86

ifeq ($(BOARD),nrf51_pca10028)
	CONF_FILE ?= nrf5x_nffs.conf
else ifeq ($(BOARD),nrf52_pca10040)
	CONF_FILE ?= nrf5x_nffs.conf
else ifeq ($(BOARD),nrf52840_pca10056)
	CONF_FILE ?= nrf5x_nffs.conf
else
	CONF_FILE ?= prj.conf
endif

QEMU_EXTRA_FLAGS = -serial unix:/tmp/bt-server-bredr

include $(ZEPHYR_BASE)/Makefile.inc
