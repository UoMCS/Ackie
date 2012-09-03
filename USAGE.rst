Usage
=====

Ackie is designed to be used with the Perentie debug monitor and an MU0 or STUMP
processor running on the Manchester Lab Board FPGA.


Hardware Design
---------------

You should synthesise your processor connected to the appropriate "debugger"
module, i.e. stump_debugger or mu0_debugger. This module provides a clock, reset
and memory interface for the processor. It also expects a 'fetch' signal from
the processor which is asserted whenever it is in the fetch state. The debugger
module connects to the pins of the FPGA and communicates with the
microcontroller (running ackie).

You can optionally attach certain internal registers of your processor to the
debugger module. For the STUMP, you should add an extra register bank port
("port c"), and expose the cc register's value. For the MU0, you should expose
the accumulator, program counter and two flag bits (Bit0: Z, Bit1: N).

If the internal registers are exposed, the regs_connected signal should be held
high. If no registers are exposed, this signal should be held low.

Use with Perentie
-----------------

The Manchester Lab Board's boot dip-switches should be set to boot Ackie (boot 2
at the time of writing) and the board should be connected via serial to a
computer running Perentie.

Perentie will detect the board and display an "unknown architecture" warning. To
load your design onto the FPGA, click the "FPGA" button on the toolbar,
select the synthesized bit-file and click "Download to FPGA".

If everything synthesized correctly, Perentie's interface should update once the
download is complete and show a memory viewer and, if you attached them to the
debugger module, a system register viewer. The tiltle bar should also report the
correct archtecture for the design you uploaded.

Clicking the "Debug" toolbar button opens a debug controller which allows
low-level control and observation of your design. Here you can see the number of
cycles taken to execute the last instruction, manually clock your design and see
the state of the memory interface. See the "Debug Controller" section of the
Perentie "Peripheral Guide" for more info.

Memory Mapped Hardware
----------------------

The Manchester Lab Board's LEDs, buttons and LCD have been memory mapped into
the address space accessible by your design. See the "Memory Map" section of the
Ackie README.rst for details.
