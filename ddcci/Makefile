#!/usr/bin/make -f
# (c) 2015 Christoph Grenz <christophg@grenz-bonn.de>
# This file is part of ddcci-driver-linux.
#
# ddcci-driver-linux is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 2 of the License, or
# (at your option) any later version.
#
# ddcci-driver-linux is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with ddcci-driver-linux.  If not, see <http://www.gnu.org/licenses/>.

MODULE_NAME := ddcci
MODULE_VERSION := 0.4.1

KVER := $(shell uname -r)
LIB_MODULES_PATH := /lib/modules
KERNEL_MODLIB := $(LIB_MODULES_PATH)/$(KVER)
MODDIR := $(KERNEL_MODLIB)/extra
INCLUDEDIR := /usr/local/include
KDIR := $(KERNEL_MODLIB)/build
PWD := $(shell pwd)

obj-m := ddcci.o

src ?= $(PWD)
obj ?= .
ccflags-y := -I$(src)/../include

module: ddcci.ko

ddcci.ko: ddcci.c
	$(MAKE) -C "$(KDIR)" M="$(src)" modules

ddcci.mod.c: ddcci.ko
ddcci.mod.o: ddcci.mod.c

clean:
	rm $(obj-m) ddcci.mod.c ddcci.mod.o ddcci.ko modules.order Module.symvers .ddcci*.cmd .tmp_versions/ddcci.mod || true
	rmdir .tmp_versions 2>/dev/null || true

install: module ddcci.mod.o
	install -d "$(MODDIR)"
	install -m 644 ddcci.ko "$(MODDIR)/"
	install -d "$(INCLUDEDIR)/linux/"
	install -m 644 ../include/linux/ddcci.h "$(INCLUDEDIR)/linux/"
	depmod "$(KVER)"

uninstall:
	rm "$(MODDIR)"/ddcci.ko || true
	rmdir --ignore-fail-on-non-empty "$(MODDIR)"
	rm "$(INCLUDEDIR)/linux/ddcci.h" || true
	rmdir --ignore-fail-on-non-empty "$(INCLUDEDIR)/linux"
	depmod "$(KVER)"

load: module
	insmod ddcci.ko

unload:
	! grep -q "^ddcci " /proc/modules || rmmod ddcci

.PHONY: module clean install uninstall load unload install-dkms uninstall-dkms
