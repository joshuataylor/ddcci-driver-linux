# ddcci-driver-linux #

A pair of Linux kernel drivers for DDC/CI monitors.

DDC/CI is a control protocol for monitor settings supported by most monitors since about 2005.
It is based on ACCESS.bus (an early USB predecessor).

## ddcci (bus driver) ##

This driver detects DDC/CI devices on DDC I²C busses, identifies them and creates corresponding devices.

As this is a I²C driver it won't be autoloaded and must be manually loaded, for example by putting a line with `ddcci`
in `/etc/modules`.

### sysfs interface ###

Each detected DDC/CI device gets a directory in `/sys/bus/ddcci/devices`.

The main device on a bus is named `ddcci[I²C bus number]`.

Internal dependent devices are named `ddcci[I²C bus number]i[hex address]`

External dependent devices are named `ddcci[I²C bus number]e[hex address]`

There the following files export information about the device:

#### capabilities ####

The full ACCESS.bus capabilities string. It contains the protocol, type and model of the device,
a list of all supported command codes, etc.

See the ACCESS.bus spec for more information.

#### idProt ####

ACCESS.bus protocol supported by the device. Usually "monitor".

#### idType ####

ACCESS.bus device subtype. Usually "LCD" or "CRT".

#### idModel ####

ACCESS.bus device model identifier. Usually a shortened form of the device model name.

#### idVendor ####

ACCESS.bus device vendor identifier. Empty if the Identification command is not supported.

#### idModule ####

ACCESS.bus device module identifier. Empty if the Identification command is not supported.

#### idSerial ####

32 bit device number. A fixed serial number if it's positive, a temporary serial number if negative and zero if the
Identification command is not supported.

### Character device interface ###

For each DDC/CI device a character device in `/dev/bus/ddcci/[I²C bus number]/` is created.

The main device on the bus is named `display`.

Internal dependent devices are named `i[hex address]`

External dependent devices are named `e[hex address]`

These character devices can be used to issue commands to a DDC/CI device more easily than over i2c-dev devices.
They should be opened unbuffered and may be opened with O_EXCL if you want exclusive access.

To send a command just write the command byte and the arguments with a single `write()` operation. The length byte and
checksum are automatically calculated.

To read a response use `read()` with a buffer big enough for the expected answer.

NOTE: The maximum length of a DDC/CI message is 127 bytes.

An Example (in Python):

	with open('/dev/bus/ddcci/3/display', 'r+b', buffering=0) as f:
		# Read contrast
		f.write(bytes([0x01, 0x12]))
		response = f.read(8)
		print("Contrast:", response[6] * 256 + response[7], "/", response[4] * 256 + response[5])

The following error codes are used:

* EAGAIN: there was no response yet or (with O_NONBLOCK) the device was in use by another thread
* EBADMSG: there was a response but the checksum didn't match
* EBUSY: the device is opened exclusively by another thread (on open())
* EINVAL: message too big (on write())
* EIO: generic I/O failure
* EMSGSIZE: the buffer was too small (on read())
* ENOMEM: not enough free memory to allocate buffers (on open())

Lower layers may pass error codes not in this list like ENXIO, so be prepared for that.

## ddcci-backlight (monitor backlight driver) ##

For each monitor that supports accessing the Luminance property, a backlight device of type "raw" named like the corresponding ddcci device is created. You can find them
in `/sys/class/backlight/`.

## Limitations ##

Dependent device (sub devices using DDC/CI directly wired to the monitor, like Calibration devices, IR remotes, etc.) aren't automatically detected. 

You can force detection of internal dependent devices by setting the `autoprobe_addrs` module parameter of ddcci.

You can force detection of external dependent devices by writing "ddcci-dependent [address]" into
/sys/bus/i2c/i2c-?/new_device.

Currently only the Luminance setting is used by ddcci-backlight. There seems to be a setting called "Backlight Control"
(0x13) but I don't own a monitor supporting this VCP, so I couldn't test it.

There is no direct synchronization if you manually change the luminance with the buttons on your monitor, as this can
only be realized through polling and some monitors close their OSD every time a DDC/CI command is received.

## Installation ##

Generally, you only need to clone the repository and run make to build both kernel modules, then run make load. To permanently install the drivers, use `make install` (or `make -f Makefile.dkms install` if you prefer DKMS).
