Ackie - ARM-Board Host Program
==============================

An ARM program designed to run on the Manchester Lab Board v1.1 to provide
loading, debug and control facilities to a simple 16-bit processor running
onboard the FPGA.

This program implements the Komodo comms interface and exposes a
suitably-instrumented 16-bit memory-word processor implemented on the FPGA. The
CPU ID/Sub-ID are derrived from the device loaded onto the FPGA and the
scan-path description loaded into the host. (See host.s for details).

Clocking
--------

The clock signal generated for device on the FPGA is very slow. When the
device is stepped (i.e. supposed to run a single instruction) clock pulses are
sent to the processor until its FETCH signal goes high.

The HOLD_TIME constant in host.s specifies the delay in system timer ticks
before a clock edge can be sent.


Memory Map
----------

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

To a simple approximation, the memory is synchronous on the back-edge of the
clock.

In reality, the memory interface is sensitive at all times between the back-edge
of the clock and the next leading clock edge. This is done to ensure that
changing the memory containing the next instruction has the expected effect.
Note that this can have unexpected side-effects during scanning (see the section
on scanning).

Scanning
--------

The host operates the scan-path in the device on the FPGA to get and set the
contents of its registers.

The scan-enable and scan-clock signals will not be asserted while the main clock
is being cycled.

The scan-clock is toggled once while scan-enable is low before scanning the
register values out of the system (see the note below).

Changing Scan Registers, A Warning
``````````````````````````````````
Because the memory interface is active both before and after scanning completes
(as it is active while the clock signal is low) if the values scanned into the
system change the address, data the CPU is writing to or the write enable
signal then the memory will end up being written twice (once before and after
the scan occurred).

Note About Clocking with Scan-Enable Low
````````````````````````````````````````
This behaviour is probably not common but it tests that scan-enable is doing its
job(!) and also means that scan registers get a pulse before scanning begins in
which to initialise, should they wish.


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
