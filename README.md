# Reverse Engineering of Cypress USB-Serial Configuration Utility

This is an effort to reverse engineer undocumented protocol and
data format used by the Cypress USB-Serial Configuration Utility.

This tool is needed to configure Cypress CY7C65211 and other
CY7C6521x series of USB-to-UART/SPI/I2C/JTAG/etc bridge chip.

Although open source (LGPL) library SDK is provided by Cypress
to access these chips, it lacks management features. So I am
forced to go back to Windows whenever I need to change chip
configuration.

# Status

* Management USB Protocol can now be sent with a Python script
 * However, exact meaning of some values are still unclear
* Configuration memory data format is fairly understood now
 * However, not all configuration details are fully documented yet

# Trying it out
```
$ ./cyusb.py
cyusb.py - Reprogram Cypress USB-to-Serial chip (CY7C65211, etc)
Usage: cyusb.py [options] (save|load|mode) args...
Options:
  -V, --vid vid: VID of device to connect (0x04b4)
  -P, --pid pid: PID of device to connect (0x0004)
  -n, --nth N  : Select Nth device (0)
  -s, --scb N  : Select Nth SCB block (0)
Example:
 $ cyusb.py save save.bin
 $ cyusb.py load save.bin
NOTE:
- Detail of configuration memory is still under investigation.
- Interface is likely to change after further discovery.
```

# What have been discovered
* For undocumented management USB protocol, see cyusb.py.
* For undocumented configuration memory format, see config.txt

# Roadmap

After this hack is complete, I will add pure-Python library to
access Cypress chips to python-ucdev library. I already have
an interface to libcyusbserial (Cypress's own SDK) API, but
mine will have more features with above-hacked management capability.

My plan is to create a simple command-line tool to reconfigure
Cypress chips from any non-Windows environment, using this
enhancement to python-ucdev library.
