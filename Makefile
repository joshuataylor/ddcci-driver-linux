#!/usr/bin/make -f
# (c) 2015 Christoph Grenz <christophg@grenz-bonn.de>
# This file is part of ddcci-bus-linux.
#
# ddcci-bus-linux is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 2 of the License, or
# (at your option) any later version.
#
# ddcci-bus-linux is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with ddcci-bus-linux. If not, see <http://www.gnu.org/licenses/>.

MODULES := ddcci ddcci-backlight

ddcci-backlight: ddcci

reverse = $(if $(1),$(call reverse,$(wordlist 2,$(words $(1)),$(1)))) $(firstword $(1))
REVMODS := $(call reverse,$(MODULES))

all: $(MODULES)

$(MODULES):
	$(MAKE) -C "$@"

CLEANMODS := $(addprefix clean-,$(MODULES))
$(CLEANMODS):
	$(MAKE) -C "$(subst clean-,,$@)" clean
clean: $(CLEANMODS)

LOADMODS := $(addprefix load-,$(MODULES))
$(LOADMODS):
	$(MAKE) -C "$(subst load-,,$@)" load
load: $(LOADMODS)

UNLOADMODS := $(addprefix unload-,$(REVMODS))
$(UNLOADMODS):
	$(MAKE) -C "$(subst unload-,,$@)" unload
unload: $(UNLOADMODS)

INSTALLMODS := $(addprefix install-,$(MODULES))
$(INSTALLMODS):
	$(MAKE) -C "$(subst install-,,$@)" install
install: $(INSTALLMODS)

UNINSTALLMODS := $(addprefix uninstall-,$(REVMODS))
$(UNINSTALLMODS):
	$(MAKE) -C "$(subst uninstall-,,$@)" uninstall
uninstall: $(UNINSTALLMODS)

.PHONY: all clean $(MODULES) $(CLEANMODS) $(LOADMODS) $(UNLOADMODS) $(INSTALLMODS) $(UNINSTALLMODS)