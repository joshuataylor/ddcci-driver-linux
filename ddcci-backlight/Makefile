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

MODULE_NAME := ddcci-backlight
MODULE_VERSION := 0.3.2

KVER := $(shell uname -r)
KERNEL_MODLIB := /lib/modules/$(KVER)
MODDIR := $(KERNEL_MODLIB)/extra
KDIR := /lib/modules/$(KVER)/build
PWD := $(shell pwd)

obj-m := ddcci-backlight.o
src ?= $(PWD)
obj ?= .
ccflags-y := -I$(src)/../include

KBUILD_EXTRA_SYMBOLS := $(src)/../ddcci/Module.symvers

module: ddcci-backlight.ko

ddcci-backlight.ko: ddcci-backlight.c
	$(MAKE) -C "$(KDIR)" M="$(src)" modules

ddcci-backlight.mod.c: ddcci-backlight.ko
ddcci-backlight.mod.o: ddcci-backlight.mod.c

clean:
	rm $(obj-m) ddcci-backlight.mod.c ddcci-backlight.mod.o ddcci-backlight.ko modules.order Module.symvers .ddcci-backlight*.cmd .tmp_versions/ddcci-backlight.mod || true
	rmdir .tmp_versions 2>/dev/null || true

install: module ddcci-backlight.mod.o
	install -d "$(MODDIR)"
	install -m 644 ddcci-backlight.ko "$(MODDIR)/"
	depmod "$(KVER)"

uninstall:
	rm "$(MODDIR)"/ddcci-backlight.ko || true
	rmdir --ignore-fail-on-non-empty "$(MODDIR)"
	depmod "$(KVER)"

load: module
	insmod ddcci-backlight.ko

unload:
	! grep -q "^ddcci[_-]backlight " /proc/modules || rmmod ddcci-backlight


.PHONY: module clean install uninstall load unload install-dkms uninstall-dkms
