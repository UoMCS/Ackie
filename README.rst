Ackie - ARM-Board Host Program
==============================

An ARM program designed to run on the Manchester Lab Board v1.1 to provide
loading, debug and control facilities to a simple 16-bit processor running
onboard the FPGA.

This program implements the Komodo comms interface and exposes a
suitably-instrumented 16-bit memory-word processor implemented on the FPGA. The
CPU ID/Sub-ID are derrived from the device loaded onto the FPGA and the
scan-path description loaded into the host. (See host.s for details).

A 128KB memory is exposed to loaded designs which contains a number of
memory-mapped devices. The following areas are mapped::

	0x0800: Bottom 8-bits are mapped to the LEDs
	0x0801: Bottom 3-bits are mapped to the buttons
	0x0802: Bottom bit is the LCD backlight state
	0x0820: This and the next 31 words are the current
	        contents of the LCD's 32 characters.
	0x0840: This and the next 16 words are the current
	        contents of the LCD's 32 characters as
	        two consecutive characters packed into each
	        byte (for convenient loading of strings)


Download of Host Program
------------------------

To download the host program onto the ARM board you need to add an entry to the
boot table and load the program into ROM.

Setting up the boot-table
`````````````````````````
An example boot table is given in boot_table.s. This file defines three boots,
with the loader and ARM-emulator as boots 0 and 1 and Ackie as the third boot.
Assemble using::

	aasm -e boot_table.elf boot_table.s

And then load onto the device using::

	$ flash
	flash> erase 4000
	flash> elf boot_table.elf

Installing the host program
```````````````````````````
Assemble using::

	aasm -e host.s host.s

And then load onto the device using::

	$ flash
	flash> erase 30000
	flash> elf host.elf


The Name
--------
The Ackie is a small monitor lizzard.
