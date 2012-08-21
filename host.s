	; ARM Host Program
	;
	; (c) August 2012
	; Jonathan Heathcote / University of Manchester
	;
	; This program implements the Komodo comms
	; interface and exposes a suitably-instrumented
	; 16-bit memory-word processor implemented on the
	; FPGA.
	;
	; The program begins execution from 'main'. It
	; executes a main-loop which deals with communications
	; with the host debugger via the KMD comms protocol.
	; Whenever this process blocks on IO (and also at every
	; iteration of the main-loop) the idle_process function
	; is called which is a state-machine which handles
	; clocking the attached FPGA and scanning out
	; registers.
	;
	; A number of registers are reserved for global use.
	;
	; This program, due to its somewhat more dynamic than
	; usual roles, has a few specific behavrious which
	; might not be obvious. These are described below.
	;
	; The CPU ID/Sub-ID is taken as follows:
	;   ID: The CPU ID returned by the FPGA at address
	;       FPGA_REG_DUT_CPU_TYPE. When the FPGA is
	;       unprogrammed, 0xFFFF is returned from all
	;       addresses which means that the CPU will be of
	;       type FF unless it is correclty programmed.
	;   Sub-ID: The bottom-byte of the sub-id is taken from
	;           the bottom-byte of the FPGA's address
	;           FPGA_REG_DUT_CPU_SUBTYPE. Again, this is FF
	;           if the FPGA is unprogrammed. The top byte
	;           is ignored. This byte should be one
	;           of CPU_SUBTYPE_REG_AND_MEM or
	;           CPU_SUBTYPE_MEMORY_ONLY depending on
	;           whether a scan-path for the registers in
	;           the system is provided. The top byte of the
	;           sub-id is the number of extra user
	;           registers in the scan path.
	;
	; The system features logic which can scan data
	; into and out of the register banks of the system
	; running on the FPGA. Because the system registers the
	; debugger expects to find are pre-defined, the first 9
	; registers scanned out must be R0-R7 then CC. As a
	; result, this part of the scan-path is hard-coded in
	; the table REG_DEFAULTS. Up to 255 additional user
	; registers of up to 16 bits can be defined by sending
	; data in the same format as these tables to peripheral
	; 1.
	;
	; Additional control/visibility over the attached CPU
	; in the form of manual clocking and observability of
	; the memory interface is provided by peripheral 1.
	; Reading its state will yield various signals (see
	; cmd_fr_get) and setting its state to any value will
	; cause a clock pulse to be sent to the CPU (this pauses
	; execution if it was running).
	;
	; If fetch is not asserted for 255 clocks, the
	; processor is stopped.
	
	include header.s
	include usart.h
	include pio.h
	include tc.h
	include pmc.h
	
	
	;------------------------------------------------------
	; Global Definitions
	;
	; The following registers are also globals:
	;
	; R7:  Idle process state
	; R8:  CPU State
	; R9:  Steps executed since reset
	; R10: Steps remaining
	; R11: USART Base
	; R12: FPGA Base
	; R13: SP
	; R14: LR
	;------------------------------------------------------
	
	; The location of the program in ROM
ROM_START	EQU 0x30000
	
	; Default MMU translation table included in ROM
TRANSLATION_TABLE	EQU 0x8000 | ROM_base
	
	; The serial port to use
USART_BASE	EQU USART1_BASE
	
	; The CPU type
CPU_TYPE_UNKNOWN	EQU 0xFF
CPU_TYPE_STUMP	EQU 3
CPU_TYPE_MU0	EQU 4
	; The CPU sub-type (only regard the LSB)
CPU_SUBTYPE_UNKNOWN	EQU 0xFF
CPU_SUBTYPE_REG_AND_MEM	EQU 0x00
CPU_SUBTYPE_MEMORY_ONLY	EQU 0x01
	
	; The FPGA (feature) type
FPGA_TYPE	EQU 0x14
FPGA_SUBTYPE	EQU 0x0211
	
	; The Controller (feature) type
CONTROLLER_TYPE	EQU 0x01
CONTROLLER_SUBTYPE	EQU 0x0000
	
	; Feature numbers
FPGA_FEATURE	EQU 0
CONTROLLER_FEATURE	EQU 1
	
	;------------------------------------------------------
	; Idle-process global state register constants
	;
	; The register has four 8-bit fields:
	;    7:0 - The current state-machine state number
	;   15:8 - Bit-field containing request triggers
	;  23:16 - Bit-field containing status bits
	;  31:24 - A counter field (clocks since fetch)
	;------------------------------------------------------
	
	; A mask over R7's idle state bits
IDLE_STATE_MASK	EQU 0xFF
	
	; A mask over R7's idle request bits
IDLE_REQUEST_MASK	EQU 0xFF00
	
	; A mask over R7's flag bits
IDLE_FLAG_MASK	EQU 0xFF0000
	
	; A mask over R7's counter bits
IDLE_COUNT_MASK	EQU 0xFF000000
	
	; Shift amount to access  R7's counter bits
IDLE_COUNT_SHIFT	EQU 24
	
	;- - - - - - - - - - - - - - - - - - - - - - - - - - -
	
	; The idle process should aim to start resetting a
	; newly connected processor
IDLE_START	EQU 0x00
	
	; The processor reset line is being held down, wait for
	; a while and then release it.
IDLE_RESETTING	EQU 0x01
	
	; State when the processor is set up and the idle
	; process should pay attention to the requested
	; processor state variables
IDLE_NORMAL	EQU 0x02
	
	; Make sure the memory interface shows the correct data
	; before clocking begins.
IDLE_CLOCK_INIT	EQU 0x03
	
	; The clock is being cycled.
IDLE_CLOCK	EQU 0x04
	
	; The scan-path is being initialised
IDLE_SCAN_INIT	EQU 0x05
	
	; The scan-path is being cycled
IDLE_SCANNING	EQU 0x06
	
	;- - - - - - - - - - - - - - - - - - - - - - - - - - -
	
	; Request a register read/write operation to be carried
	; out when the processor is next in the fetch state.
	; Currently not treated differently
IDLE_REQ_REG_READ	EQU 0x0100
IDLE_REQ_REG_WRITE	EQU 0x0200
	
	; Request a single clock pulse be issued
IDLE_REQ_CLOCK	EQU 0x0400
	
	;- - - - - - - - - - - - - - - - - - - - - - - - - - -
	
	; A flag which indicates that the register values
	; currently loaded are up-to-date
IDLE_FLAG_REGS_AVAIL	EQU 0x010000
	
	; Indicates if the processor was recently reset and has
	; not since been stepped.
IDLE_FLAG_RESET	EQU 0x020000
	
	; Flag indicates that the last bit was just scanned
	; into the scan-path
IDLE_FLAG_SCANNED_LAST	EQU 0x040000
	
	; Flag indicates that the last bit is now clocked in
IDLE_FLAG_SCAN_COMPLETE	EQU 0x080000
	
	;------------------------------------------------------
	; RAM Allocation (Addresses of variables/arrays in the
	; main system RAM)
	;------------------------------------------------------
	
	; Start address of the memory the DUT uses (in the
	; ARM address space
MEMORY_START	EQU 0x00000000
	; DUT Memory address (16-bit word wise address) mask
MEMORY_MASK	EQU 0xFFFF
	
	; Location in ram (just after program memory) to use as
	; a buffer for incoming FPGA data.
FPGA_DATA_BUFFER	EQU MEMORY_START + (MEMORY_MASK<<1) + 2
FPGA_DATA_BUFFER_SIZE	EQU 256
	
	; The current cpu info (for wot_r_u)
CPU_INFO	EQU FPGA_DATA_BUFFER + FPGA_DATA_BUFFER_SIZE
	
	; The current register being scanned
REG_CUR_NUM	EQU CPU_INFO + CPU_INFO_LENGTH
	
	; The bit in the register currently being scanned
REG_CUR_BIT	EQU REG_CUR_NUM + 4
	
	; Start address of the register data table (see
	; REG_DEFAULTS for format).
REG_START	EQU REG_CUR_BIT + 4
	
	;------------------------------------------------------
	; Register/scan-path constants
	;------------------------------------------------------
	
	; A register has been written to and should be sent
	; back to the device
SCAN_DIRTY	EQU 2
	
	;------------------------------------------------------
	; System Control Parameters
	;------------------------------------------------------
	
	; Number of ticks the system timer must reach before
	; changing a signal in the FPGA
HOLD_TIME	EQU 0x80
	
	;------------------------------------------------------
	; Komodo Protocol Constants
	;------------------------------------------------------
	
	; Reponse to PINGs ("OK00" in ASCII)
PING_RESPONSE	EQU 0x30304B4F
	
	; CPU States
STATE_RESET	EQU 0x00
STATE_BUSY	EQU 0x01
STATE_STOPPED	EQU 0x40
STATE_STOPPED_BREAKPOINT	EQU 0x41
STATE_STOPPED_WATCHPOINT	EQU 0x42
STATE_STOPPED_MEM_FAULT	EQU 0x43
STATE_STOPPED_PROG_REQ	EQU 0x44
STATE_RUNNING	EQU 0x80
STATE_SWI	EQU 0x81
	
	; Target bits for read/write
TARGET_MASK	EQU 0x3<<4
TARGET_MEMORY	EQU 0x0<<4
TARGET_REGISTERS	EQU 0x1<<4
	
	; Read/Write bit for read/write commands
READ_BIT	EQU 0x1<<3
	
	; Element-size mask for read/write commands
ELEMENT_SIZE_MASK	EQU 0x7
ELEMENT_SIZE_1	EQU 0x0 ; 1 Byte
ELEMENT_SIZE_2	EQU 0x1 ; 2 Bytes
ELEMENT_SIZE_4	EQU 0x2 ; 4 Bytes
ELEMENT_SIZE_8	EQU 0x3 ; 8 Bytes
	
	;------------------------------------------------------
	; FPGA Interface Definitions
	;------------------------------------------------------
	
	; Base address of the FPGA
FPGA_BASE	EQU 0x30000000
	
	; Register addresses within the FPGA address space
FPGA_REG_SCAN_CONTROL	EQU 0x00
FPGA_REG_SCAN_IN	EQU 0x02
FPGA_REG_SCAN_OUT	EQU 0x04
FPGA_REG_DUT_CONTROL	EQU 0x06
FPGA_REG_DUT_FETCH	EQU 0x08
FPGA_REG_DUT_ADDR	EQU 0x0A
FPGA_REG_DUT_RWEN	EQU 0x0C
FPGA_REG_DUT_DATA_OUT	EQU 0x0E
FPGA_REG_DUT_DATA_IN	EQU 0x10
FPGA_REG_DUT_CPU_TYPE	EQU 0x20
FPGA_REG_DUT_CPU_SUBTYPE	EQU 0x22
	
	; FPGA_REG_DUT_CONTROL bits
DUT_CLK	EQU 0x1
DUT_RESET	EQU 0x2
	
	; FPGA_REG_DUT_RWEN bits
DUT_RD	EQU 0x1
DUT_WR	EQU 0x2
	
	; FPGA_REG_SCAN_CONTROL bits
SCAN_CLK	EQU 0x1
SCAN_EN	EQU 0x2
	
	;------------------------------------------------------
	; FPGA Loading Definitions
	;------------------------------------------------------
	
	; FPGA Pins
FPGA_INIT_B	EQU 0x00040000
FPGA_RDWR_B	EQU 0x00020000
FPGA_CS_B	EQU 0x00010000
FPGA_CCLK	EQU 0x00000002
FPGA_PROG_B	EQU 0x00000001
	
	; The FPGA programming bus
FPGA_BUS_SHIFT	EQU 23
FPGA_BUS	EQU 0xFF << FPGA_BUS_SHIFT
	
	
	;------------------------------------------------------
	; IO Peripheral Definitions
	;------------------------------------------------------
	
	; The LEDs & Buttons
LED_BUS_SHIFT	EQU 23
BUTTONS_BUS_SHIFT	EQU 4
FIQ_BUTTON_BUS_SHIFT	EQU 11
	
	
	;------------------------------------------------------
	; Start of ROM Image
	;------------------------------------------------------
	
	ORG ROM_START
	ENTRY
	
	B main
	
	; String describing the program
	DCB "FPGA CPU Debugger Host "
	DCB "Jonathan Heathcote, University of Manchester "
	DCB "(c) August 2012"
	
	ALIGN
	
	;------------------------------------------------------
	; Data
	;------------------------------------------------------
	
	;- - - - - - - - - - - - - - - - - - - - - - - - - - -
	; Komodo CPU info-string
	;- - - - - - - - - - - - - - - - - - - - - - - - - - -
DEFAULT_CPU_INFO	; Length of info
	DCW DEFAULT_CPU_INFO_END - DEFAULT_CPU_INFO_BODY
	
DEFAULT_CPU_INFO_BODY	; CPU type information
DEFAULT_CPU_INFO_TYPE_B	DCB CPU_TYPE_UNKNOWN
DEFAULT_CPU_INFO_SUBTYPE_H	DCW CPU_SUBTYPE_UNKNOWN
	
	; Two features: a Spartan-3 FPGA & a controller
	; interface to allow manual clocking
	DCB 2
	DCB FPGA_TYPE
	DCW FPGA_SUBTYPE
	DCB CONTROLLER_TYPE
	DCW CONTROLLER_SUBTYPE
	
	; One memory segment
	DCB 1
	DCD 0
	DCD (MEMORY_MASK+1)
DEFAULT_CPU_INFO_END
	
	; Useful values/offsets
CPU_INFO_LENGTH	EQU DEFAULT_CPU_INFO_END - DEFAULT_CPU_INFO
CPU_INFO_CPU_TYPE	EQU DEFAULT_CPU_INFO_TYPE_B - DEFAULT_CPU_INFO
CPU_INFO_CPU_SUBTYPE	EQU DEFAULT_CPU_INFO_SUBTYPE_H - DEFAULT_CPU_INFO
	
	ALIGN
	
	;- - - - - - - - - - - - - - - - - - - - - - - - - - -
	; Default scan-path/registers
	;- - - - - - - - - - - - - - - - - - - - - - - - - - -
	
	; The register table format consists of rows of:
	;
	;        +-----+----------- Register value (16-bits)
	;        |     |     +----- Register Width (bits)
	;        |     |     |   +- Dirty flag
	;        |     |     |   |
	;   DCB 0x00, 0x00, 16, 0x00
	;
	; Terminated by a null/zero (32-bit) row

	; Lookup table pointing to compatible architectures
	; Contains half-word pairs where the first half-word
	; has the MSB set to the cpu type and and LSB set to
	; the cpu sub-type's LSB. The second half-word is the
	; Offset from the start of the table to the register
	; table listing.

REG_DEFAULTS	; STUMP With Register List
	DCW (CPU_TYPE_STUMP<<8) | CPU_SUBTYPE_REG_AND_MEM
	DCW REG_DEFAULTS_STUMP - REG_DEFAULTS
	
	; STUMP Without Register List
	DCW (CPU_TYPE_STUMP<<8) | CPU_SUBTYPE_MEMORY_ONLY
	DCW REG_DEFAULTS_EMPTY - REG_DEFAULTS

	; MU0 With Register List
	DCW (CPU_TYPE_MU0<<8) | CPU_SUBTYPE_REG_AND_MEM
	DCW REG_DEFAULTS_MU0 - REG_DEFAULTS
	
	; MU0 Without Register List
	DCW (CPU_TYPE_MU0<<8) | CPU_SUBTYPE_MEMORY_ONLY
	DCW REG_DEFAULTS_EMPTY - REG_DEFAULTS
	
	; End of list (note that this prevents us supporting
	; ARMs with CPU-id 00/0000 but that wasn't going to be
	; possible anyway since everything is 16-bit).
	DCD 0x00000000
	
	;- - - - - - - - - - - - - - - - - - - - - - - - - - -
	
	; An empty register list
REG_DEFAULTS_EMPTY	DCD 0x00000000 ; Null terminate list
	
	; Stump registers
REG_DEFAULTS_STUMP	DCB 0x00, 0x00, 16, 0x00 ; R0
	DCB 0x00, 0x00, 16, 0x00 ; R1
	DCB 0x00, 0x00, 16, 0x00 ; R2
	DCB 0x00, 0x00, 16, 0x00 ; R3
	DCB 0x00, 0x00, 16, 0x00 ; R4
	DCB 0x00, 0x00, 16, 0x00 ; R5
	DCB 0x00, 0x00, 16, 0x00 ; R6
	DCB 0x00, 0x00, 16, 0x00 ; R7/PC
	DCB 0x00, 0x00,  4, 0x00 ; CC
	DCD 0x00000000;            NULL to terminate list
	
	; MU0 registers
REG_DEFAULTS_MU0	DCB 0x00, 0x00, 16, 0x00 ; Accumulator
	DCB 0x00, 0x00, 12, 0x00 ; Program Counter
	DCB 0x00, 0x00,  2, 0x00 ; Flags
	DCD 0x00000000;            NULL to terminate list
	
	ALIGN
	
	;------------------------------------------------------
	; Host program system initialisation & mainloop
	;------------------------------------------------------
	
	;- - - - - - - - - - - - - - - - - - - - - - - - - - -
	; MMU & Cache Initialisation
	;- - - - - - - - - - - - - - - - - - - - - - - - - - -
main	; Set the default MMU translation table from ROM
	MOVX R0, #TRANSLATION_TABLE
	MCR  P15, 0, R0, C2, C0, 0
	
	; Set manager permissions for domains 0, 1 in domain
	; access control register
	MOV  R0, #&0000_000F
	MCR  P15, 0, R0, C3, C0, 0
	
	; Enable MMU/caches
	MRC  P15, 0, R0, C1, C0, 0
	ORRX R0, R0, #Icache_enable | Dcache_enable | MMU_enable
	MCR  P15, 0, R0, C1, C0, 0
	
	;- - - - - - - - - - - - - - - - - - - - - - - - - - -
	; Setup PIO for FPGA
	;- - - - - - - - - - - - - - - - - - - - - - - - - - -
	; Get PIO port base address
	MOVX R0, #PIOC
	
	; FPGA data-in bus pins
	MOVX R1, #FPGA_BUS
	MVN  R2, R1
	
	; Output-enable the bus
	STR  R1, [R0, #PIO_OER]
	; Enable bus direct write for data bus
	STR  R1, [R0, #PIO_OWER]
	; Disable for all other pins
	STR  R2, [R0, #PIO_OWDR]
	
	; Setup FPGA ctrl bits initially
	; RDWR=0
	MOV R1, #FPGA_RDWR_B
	STR R1, [R0, #PIO_CODR]
	; CCLK=1
	MOV R1, #FPGA_CCLK
	STR R1, [R0, #PIO_SODR]
	; CS=0
	MOV R1, #FPGA_CS_B
	STR R1, [R0, #PIO_CODR]
	
	;- - - - - - - - - - - - - - - - - - - - - - - - - - -
	; Set up the timer
	;- - - - - - - - - - - - - - - - - - - - - - - - - - -
	; Power up the first timer
	MOVX R0, #PMC_BASE
	MOV  R1, #PMC_TC0
	STR  R1, [R0, #PMC_PCER]
	
	; Load the timer base-address
	MOVX R0, #TC_BASE
	
	; Set up timer
	;           +- MCK/32 as clock source
	;           |        +- TIOA as ext trigger
	;           |        |         +- Wave mode
	;           |        |         |
	MOVX R1, #0b011 | (1<<10) | (1<<15)
	STR  R1, [R0, #TC_CMR]
	
	; Enable timer and trigger it
	MOVX R1, #(1<<0) | (1<<2)
	STR  R1, [R0, #TC_CCR]
	
	MOVX R0, #PMC_MCKR
	LDR  R1, [R0]
	MOV  R0, #0
	STR  R1, [R0]
	STR  R1, [R0,#4]
	
	;- - - - - - - - - - - - - - - - - - - - - - - - - - -
	; Set up variables/globals
	;- - - - - - - - - - - - - - - - - - - - - - - - - - -
	; Set up global register values
	MOVX R7,  #IDLE_START ; Idle process state
	MOVX R8,  #STATE_BUSY ; CPU State
	MOVX R9,  #0          ; Steps executed
	MOVX R10, #0          ; Steps remaining
	MOVX R11, #USART_BASE
	MOVX R12, #FPGA_BASE
	
	;- - - - - - - - - - - - - - - - - - - - - - - - - - -
	; Load the device info into RAM (for wot_r_u)
	;- - - - - - - - - - - - - - - - - - - - - - - - - - -
	
	ADRL R0, DEFAULT_CPU_INFO
	ADRL R1, DEFAULT_CPU_INFO_END
	MOVX R2, #CPU_INFO
	
0	LDRB  R3, [R0], #1
	STRB  R3, [R2], #1
	; Loop until we reach the end
	CMP  R0, R1
	BNE  %b0
	
	;- - - - - - - - - - - - - - - - - - - - - - - - - - -
	; Load the default (empty) register table/scan-path
	;- - - - - - - - - - - - - - - - - - - - - - - - - - -
	
	BL  get_default_reg_table
	ADR R0, REG_DEFAULTS_EMPTY
	BL  load_reg_table
	
	;- - - - - - - - - - - - - - - - - - - - - - - - - - -
	; Mainloop
	;- - - - - - - - - - - - - - - - - - - - - - - - - - -
main_loop	; Handle incoming commands
	BL handle_command
	
	; Run the idle process (just to make sure it happens at
	; some point)
	BL idle_process
	
	; Loop-de-loop!
	B  main_loop
	
	
	
	
	;------------------------------------------------------
	; Read a byte from the serial port. This function will
	; block until a byte is read.
	;
	; Returns:
	;   R0: The byte read
	;------------------------------------------------------
	
serial_read	STMFD SP!, {LR}
	; Check the status register to see if data is available
	; Run the idle process while waiting
0	LDR R0, [R11, #US_CSR]
	TST R0, #US_RXRDY
	BNE %f1
	BL  idle_process
	B   %b0
1	
	
	; Data is available, retrieve it
	LDR R0, [R11, #US_RHR]
	
	; Mask to just the byte
	AND R0, R0, #0xFF
	
	; Return
	LDMFD SP!, {PC}
	
	
	;------------------------------------------------------
	; Write a byte to the serial port. This function will
	; block until the write completes.
	;
	; Argument:
	;   R0: The byte to write
	;
	; Uses Internally:
	;   R1: Status bits
	;------------------------------------------------------
	
serial_write	STMFD SP!, {R0, LR}
	
	; Mask the input data to just the byte to send
	AND R0, R0, #0xFF
	
	; Check the status register to see if the transmitter
	; is ready. Run the idle process while waiting.
0	LDR LR, [R11, #US_CSR]
	TST LR, #US_TXRDY
	BNE %f1
	BL  idle_process
	B   %b0
1	
	
	; Transmitter is ready, send the data
	STR R0, [R11, #US_THR]
	
	; Return
	LDMFD SP!, {R0, PC}
	
	
	;------------------------------------------------------
	; Write a word to the serial port. This function will
	; block until the write completes.
	;
	; Argument:
	;   R0: The word to write
	;------------------------------------------------------
	
serial_write_word	STMFD SP!, {R0, LR}
	
	BL serial_write
	MOV R0, R0, LSR #8
	BL serial_write
	MOV R0, R0, LSR #8
	BL serial_write
	MOV R0, R0, LSR #8
	BL serial_write
	
	; Return
	LDMFD SP!, {R0, PC}
	
	
	;------------------------------------------------------
	; Read a word from the serial port. This function will
	; block until the read completes.
	;
	; Returns:
	;   R0: The word read
	;------------------------------------------------------
	
serial_read_word	STMFD SP!, {R1, LR}
	
	BL serial_read
	MOV R1, R0
	BL serial_read
	ORR R1, R1, R0, LSL #8
	BL serial_read
	ORR R1, R1, R0, LSL #16
	BL serial_read
	ORR R0, R1, R0, LSL #24
	
	; Return
	LDMFD SP!, {R1, PC}
	
	
	;------------------------------------------------------
	; Write a half-word to the serial port. This function
	; will block until the write completes.
	;
	; Argument:
	;   R0: The half-word to write
	;------------------------------------------------------
	
serial_write_half	STMFD SP!, {R0, LR}
	
	BL serial_write
	MOV R0, R0, LSR #8
	BL serial_write
	
	; Return
	LDMFD SP!, {R0, PC}
	
	
	;------------------------------------------------------
	; Read a half-word from the serial port. This function
	; will block until the read completes.
	;
	; Returns:
	;   R0: The half-word read
	;------------------------------------------------------
	
serial_read_half	STMFD SP!, {R1, LR}
	
	BL serial_read
	MOV R1, R0
	BL serial_read
	ORR R0, R1, R0, LSL #8
	
	; Return
	LDMFD SP!, {R1, PC}
	
	
	;------------------------------------------------------
	; Write some data to the serial port. This function
	; will block until the write completes.
	;
	; Argument:
	;   R1: The start address of the data
	;   R2: The length in bytes
	;------------------------------------------------------
	
serial_write_data	STMFD SP!, {R0, R1, R2, LR}
	
	; Load each byte in turn and send it
serial_write_loop	LDRB R0, [R1], #1
	BL   serial_write
	SUBS R2, R2, #1
	BNE  serial_write_loop
	
	; Return
	LDMFD SP!, {R0, R1, R2, PC}
	
	
	;------------------------------------------------------
	; Read some data from the serial port. This function
	; will block until the read completes.
	;
	; Argument:
	;   R1: The start address of the location to write to
	;   R2: The length in bytes
	;------------------------------------------------------
	
serial_read_data	STMFD SP!, {R0, R1, R2, LR}
	
	; Receive each byte in turn and send it
serial_read_loop	BL   serial_read
	STRB R0, [R1], #1
	SUBS R2, R2, #1
	BNE  serial_read_loop
	
	; Return
	LDMFD SP!, {R0, R1, R2, PC}
	
	
	;------------------------------------------------------
	; Read from the memory space occupied by the CPU
	;
	; The following areas are mapped:
	;   0x0800: Bottom 8-bits are mapped to the LEDs
	;   0x0801: Bottom 3-bits are mapped to the buttons
	;
	; Argument:
	;   R2: Address
	; Returns:
	;   R0: Value
	;------------------------------------------------------
memory_read	STMFD SP!, {LR}
	
	; Mask off the address
	MOVX LR, #MEMORY_MASK
	AND  LR, LR, R2
	
	; Is this an LED read?
	CMP LR, #0x800
	BNE %f0
	BL  led_read
	B   memory_read_return
	
	;- - - - - - - - - - - - - - - - - - - - - - - - - - -
	
0	; Is this a button read?
	MOVX R0, #0x801
	CMP  LR, R0
	BNE  %f1
	BL   button_read
	B    memory_read_return
	
	;- - - - - - - - - - - - - - - - - - - - - - - - - - -
	
1	; This is a plain-n-simple memory access
	; And convert into a byte address
	MOV LR, LR, LSL #1
	
	; Load the value
	LDRH  R0, [LR, #MEMORY_START]
	
memory_read_return	LDMFD SP!, {PC}
	
	
	;------------------------------------------------------
	; Read the state of the LEDs
	;
	; Returns:
	;   R0: LED bits
	;------------------------------------------------------
led_read	; Read the state and shift/mask
	MOVX R0, #PIOC
	LDR  R0, [R0, #PIO_PDSR]
	MOV  R0, R0, LSR #LED_BUS_SHIFT
	AND  R0, R0, #0xFF
	
	; Return
	MOV PC, LR
	
	
	;------------------------------------------------------
	; Read the state of the buttons
	;
	; Returns:
	;   R0: Button states
	;------------------------------------------------------
button_read	STMFD SP!, {LR}
	
	; Read the state, invert to active high and shift/mask
	MOVX  LR, #PIOC
	LDR   LR, [LR, #PIO_PDSR]
	MVN   R0, LR, LSR #BUTTONS_BUS_SHIFT ; Main buttons
	AND   R0, R0, #0x03
	TST   LR, #1<<FIQ_BUTTON_BUS_SHIFT   ; FIQ (top) button
	ORREQ R0, R0, #0x4
	
	LDMFD SP!, {PC}
	
	
	;------------------------------------------------------
	; Write to the memory space occupied by the CPU
	;
	; The following areas are mapped:
	;   0x0800: Bottom 8-bits are mapped to the LEDs
	;   0x0801: Bottom 3-bits are mapped to the buttons
	;
	; Argument:
	;   R0: Value
	;   R2: Address
	;------------------------------------------------------
memory_write	STMFD SP!, {LR}
	
	; Mask off the address
	MOVX LR, #MEMORY_MASK
	AND  LR, LR, R2
	
	; Is this an LED write?
	CMP LR, #0x800
	BNE %f0
	BL  led_write
	B   memory_write_return
	
	;- - - - - - - - - - - - - - - - - - - - - - - - - - -
	
0	; This is a plain-n-simple memory access
	; And convert into a byte address
	MOV LR, LR, LSL #1
	
	; Store the value
	STRH  R0, [LR, #MEMORY_START]
	
memory_write_return	LDMFD SP!, {PC}
	
	
	;------------------------------------------------------
	; Set the state of the LEDs
	;
	; Argument:
	;   R0: LED bits
	;------------------------------------------------------
led_write	STMFD SP!, {R0, LR}
	
	; Shift and mask
	AND  R0, R0, #0xFF
	MOV  R0, R0, LSL #LED_BUS_SHIFT
	
	; Write to the LEDs
	MOVX LR, #PIOC
	STR  R0, [LR, #PIO_ODSR]
	
	LDMFD SP!, {R0, PC}
	
	;------------------------------------------------------
	; Read register values from the CPU
	;
	; Argument:
	;   R2: Register 'Address'
	; Returns:
	;   R0: Value
	;------------------------------------------------------
register_read	STMFD SP!, {LR}
	
	; Set the flag ensuring registers are kept up-to-date
	ORR R7, R7, #IDLE_REQ_REG_READ
	
	; Get the start address of the register memory
	MOVX LR, #REG_START
	
	; Calculate offset into the list of registers
	ADD  LR, LR, R2, LSL #2
	
	; Load the register's value
	LDRH  R0, [LR]
	
	LDMFD SP!, {PC}
	
	
	;------------------------------------------------------
	; Write to a register in the CPU.
	;
	; Argument:
	;   R0: Value
	;   R2: Register 'Address'
	;------------------------------------------------------
register_write	STMFD SP!, {R0-R1, LR}
	
	; Set the flag ensuring registers are kept up-to-date
	ORR R7, R7, #IDLE_REQ_REG_WRITE
	
	; Get the start address of the register memory
	MOVX LR, #REG_START
	
	; Calculate offset into the list of registers
	ADD  LR, LR, R2, LSL #2
	
	; If trying to write to the terminating zero register
	; in the table, fail.
	LDR R0, [LR]
	CMP R0, #0
	BEQ %f0
	
	; Store the register's new value
	STRH  R0, [LR]
	
	; Set the dirty flag
	MOV  R0, #SCAN_DIRTY
	STRB R0, [LR, #3]
	
0	LDMFD SP!, {R0-R1, PC}
	
	
	;-----------------------------------------------------
	; Get the address of the register table for the current
	; architecture. Returns zero if no table was found.
	;
	; Returns:
	;   R0: The address of the table or zero.
	;   Sets the flags to be EQ if R0 is zero, NE otherwise
	;-----------------------------------------------------
get_default_reg_table	STMFD SP!, {R1-R2, LR}
	
	; Get the CPU type/LSB-of-subtype into a half-word in
	; R1
	MOVX  LR, #CPU_INFO
	LDRB  R0, [LR, #CPU_INFO_CPU_TYPE]
	LDRB  R1, [LR, #CPU_INFO_CPU_SUBTYPE]
	ORR   R1, R1, R0, LSL #8
	
	; Search the set of register tables for a suitable
	; table
	ADR  LR, REG_DEFAULTS
	MOV  R2, LR
0	LDRH R0, [R2], #4
	CMP  R1, R0
	BEQ  %f1
	CMP  R0, #0
	BNE  %b0
	; Not found (reached a null terminator), just return
	; the zero
	B %f2
	
	; Found an entry! Load the offset
1	LDRH R0, [R2, #-2]
	; Add it to the table start address and return (setting
	; the flags)
	ADDS R0, R0, LR
	
2	LDMFD SP!, {R1-R2, PC}
	
	
	;-----------------------------------------------------
	; Load the stated register table from ROM
	;
	; Arguments:
	;   R0: The table address
	;-----------------------------------------------------
load_reg_table	STMFD SP!, {R0-R1, LR}
	
	MOVX R1, #REG_START
0	LDR  LR, [R0], #4
	STR  LR, [R1], #4
	; Loop until reach a NULL
	CMP  LR, #0
	BNE  %b0
	
	LDMFD SP!, {R0-R1, PC}
	
	
	;------------------------------------------------------
	; Read and execute a command from the host computer
	;------------------------------------------------------
handle_command	STMFD SP!, {R0-R3, LR}
	
	; Read the command into R0
	BL serial_read
	
	; Extract command type
	MOV R1, R0, LSR #6
	
	; Jump based on command type
	CMP R1, #0b00 ; General command
	BEQ handle_command_gen
	CMP R1, #0b01 ; Memory command
	BEQ handle_command_mem
	CMP R1, #0b10 ; Execution control command
	BEQ handle_command_exec
	; Auxillary commands/unrecognised category
	B   cmd_nop
	
	;- - - - - - - - - - - - - - - - - - - - - - - - - - -
	; General command dispatcher
	;- - - - - - - - - - - - - - - - - - - - - - - - - - -
handle_command_gen	; Ignore unknown commands
	CMP R0, #cmd_jmp_table_end - cmd_jmp_table
	BGE cmd_nop
	
	; Jump to the requested command in the jump table
	; Because PC=PC+8, decrement the command by two to take
	; this into account.
	SUB R0, R0, #1
	ADD PC, PC, R0, LSL #2
	
	; General Command Jump-table
cmd_jmp_table	B cmd_nop         ; 0x00
	B cmd_ping        ; 0x01 Ping
	B cmd_wot_r_u     ; 0x02 Get CPU Info
	B cmd_nop         ; 0x03
	B cmd_reset       ; 0x04 Reset
	B cmd_nop         ; 0x05
	B cmd_nop         ; 0x06
	B cmd_nop         ; 0x07
	B cmd_nop         ; 0x08
	B cmd_nop         ; 0x09
	B cmd_nop         ; 0x0A
	B cmd_nop         ; 0x0B
	B cmd_nop         ; 0x0C
	B cmd_nop         ; 0x0D
	B cmd_nop         ; 0x0E
	B cmd_nop         ; 0x0F
	B cmd_fr_get      ; 0x10 Feature get status
	B cmd_fr_set      ; 0x11 Feature set status
	B cmd_nop;cmd_fr_write    ; 0x12 Feature send message
	B cmd_fr_read     ; 0x13 Feature get message
	B cmd_fr_file     ; 0x14 Feature download header
	B cmd_fr_send     ; 0x15 Feature download data
	B cmd_nop         ; 0x16
	B cmd_nop         ; 0x17
	B cmd_nop         ; 0x18
	B cmd_nop         ; 0x19
	B cmd_nop         ; 0x1A
	B cmd_nop         ; 0x1B
	B cmd_nop         ; 0x1C
	B cmd_nop         ; 0x1D
	B cmd_nop         ; 0x1E
	B cmd_nop         ; 0x1F
	B cmd_wot_u_do    ; 0x20 Get execution status
	B cmd_stop        ; 0x21 Stop
	B cmd_pause       ; 0x22 Pause
	B cmd_continue    ; 0x23 Continue
cmd_jmp_table_end	
	
	
	;- - - - - - - - - - - - - - - - - - - - - - - - - - -
	; Ping
	;- - - - - - - - - - - - - - - - - - - - - - - - - - -
cmd_ping	MOVX R0, #PING_RESPONSE
	BL   serial_write_word
	B    handle_command_return
	
	
	;- - - - - - - - - - - - - - - - - - - - - - - - - - -
	; Get CPU Info
	;- - - - - - - - - - - - - - - - - - - - - - - - - - -
cmd_wot_r_u	MOVX R1, #CPU_INFO
	MOVX R2, #CPU_INFO_LENGTH
	BL   serial_write_data
	B    handle_command_return
	
	
	;- - - - - - - - - - - - - - - - - - - - - - - - - - -
	; Reset the CPU being tested
	;- - - - - - - - - - - - - - - - - - - - - - - - - - -
cmd_reset	; Does nothing if busy
	CMP R8,  #STATE_BUSY
	BEQ handle_command_return
	
	MOV R8,  #STATE_RESET ; Enter reset state
	MOV R9,  #0           ; Reset step counter
	MOV R10, #0           ; No more steps to execute
	
	B handle_command_return
	
	
	;- - - - - - - - - - - - - - - - - - - - - - - - - - -
	; Feature get state
	;- - - - - - - - - - - - - - - - - - - - - - - - - - -
cmd_fr_get	; Read feature number
	BL serial_read
	
	CMP R0, #FPGA_FEATURE
	BEQ cmd_fr_get_fpga
	
	; Controller state:
	;   7:0 -- Clocks since last fetch
	;     8 -- Read enable
	;     9 -- Write enable
	;    10 -- Fetch
	; 31:11 -- Unused
	
	; Clocks since last fetch
	MOV  R0, R7, LSR #IDLE_COUNT_SHIFT
	; Read/write enable
	LDRH R1, [R12, #FPGA_REG_DUT_RWEN]
	ORR  R0, R0, R1, LSL #8
	; Fetch
	LDRH R1, [R12, #FPGA_REG_DUT_FETCH]
	ORR  R0, R0, R1, LSL #10
	
	BL serial_write_word
	B handle_command_return
	
	;- - - - - - - - - - - - - - - - - - - - - - - - - - -
	
cmd_fr_get_fpga	; FPGA state: Always return 0
	MOV R0, #0
	BL  serial_write_word
	
	B handle_command_return
	
	
	;- - - - - - - - - - - - - - - - - - - - - - - - - - -
	; Feature set state
	;- - - - - - - - - - - - - - - - - - - - - - - - - - -
cmd_fr_set	; Read feature number
	BL serial_read
	
	CMP R0, #FPGA_FEATURE
	BEQ cmd_fr_set_fpga
	
	; If we get any new status sent to the controller,
	; that means clock the CPU!
	BL serial_read_word
	
	; Does nothing if busy
	CMP R8,  #STATE_BUSY
	BEQ handle_command_return
	; Stop the CPU
	MOV R8, #STATE_STOPPED
	; Request it be clocked
	ORR R7, R7, #IDLE_REQ_CLOCK
	B handle_command_return
	
	;- - - - - - - - - - - - - - - - - - - - - - - - - - -
	
cmd_fr_set_fpga	; Ignore value recieved
	BL serial_read_word
	
	B handle_command_return
	
	;- - - - - - - - - - - - - - - - - - - - - - - - - - -
	; Feature get message
	;- - - - - - - - - - - - - - - - - - - - - - - - - - -
cmd_fr_read	; Read feature number
	BL serial_read
	
	CMP R0, #FPGA_FEATURE
	BEQ cmd_fr_read_nop
	
	; Assume we're requesting the memory address and data
	; (2*3 bytes). If not, return nothing.
	BL serial_read
	CMP R0, #(2*3)
	BNE cmd_fr_read_nothing
	
	; We're about to return that many bytes
	BL serial_write
	
	; Memory address
	LDRH R0, [R12, #FPGA_REG_DUT_ADDR]
	BL   serial_write_half
	LDRH R0, [R12, #FPGA_REG_DUT_DATA_IN]
	BL   serial_write_half
	LDRH R0, [R12, #FPGA_REG_DUT_DATA_OUT]
	BL   serial_write_half
	
	B handle_command_return
	
	;- - - - - - - - - - - - - - - - - - - - - - - - - - -
	
cmd_fr_read_nop	; Ignore request, return nothing
	BL serial_read
cmd_fr_read_nothing	MOV R0, #0
	BL serial_write
	
	B handle_command_return
	
	;- - - - - - - - - - - - - - - - - - - - - - - - - - -
	; Feature recieve download header
	;- - - - - - - - - - - - - - - - - - - - - - - - - - -
cmd_fr_file	; Read feature number
	BL serial_read
	
	CMP R0, #FPGA_FEATURE
	BEQ cmd_fr_file_fpga
	
	; Controller feature
	; Get the number of registers to be recieved
	BL serial_read_word
	
	; Fail if not a multiple of 4 (as registers are
	; described as 4-byte words as in the register table).
	TST  R0, #0x3
	BNE  %f0
	; Fail if more than 255 registers to be sent
	CMP R0, #255<<2
	BGT %f0
	
	; Divide by 4 to get number of registers
	MOV   R0, R0, LSR #2
	
	; Set the number of user registers in the CPU subtype
	MOVX  LR, #CPU_INFO
	STRB  R0, [LR, #CPU_INFO_CPU_SUBTYPE + 1]
	
	; Find out what register table to use (default to the
	; empty one if none supplied) and Reset the register
	; table to its defaults
	BL     get_default_reg_table
	ADRLEQ R0, REG_DEFAULTS_EMPTY
	BL     load_reg_table
	
	; Return 'A' to confirm OK
	MOV R0, #'A'
	BL  serial_write
	
	B handle_command_return
	
0	; Non multiple of 4 recieved, return an error.
	MOV R0, #'4'
	BL  serial_write
	B handle_command_return
	
	;- - - - - - - - - - - - - - - - - - - - - - - - - - -
	
cmd_fr_file_fpga	; As we're downloading a CPU, set it as busy as nothing can
	; happen to it for now.
	MOV R8, #STATE_BUSY
	
	; Get size of file and use R10 to store it as a CPU is
	; being loaded so this register isn't used
	BL  serial_read_word
	MOV R10, R0
	
	; Initialise the FPGA
	BL  fpga_init
	
	; Return 'A' to confirm OK
	MOV R0, #'A'
	BL  serial_write
	
	B handle_command_return
	
	;- - - - - - - - - - - - - - - - - - - - - - - - - - -
	; Feature recieve download data
	;- - - - - - - - - - - - - - - - - - - - - - - - - - -
cmd_fr_send	; Get the feature number
	BL serial_read
	
	CMP R0, #FPGA_FEATURE
	BEQ cmd_fr_send_fpga
	
	; Getting a controller packet
	
	; Read the number of bytes in packet (0 = 256)
	BL serial_read
	MOVS  R2, R0
	MOVEQ R2, #256
	
	; Fail if not a multiple of 4
	TST  R2, #0x3
	BEQ  %f1
	; Absorb the data
0	BL   serial_read
	SUBS R2, R2, #1
	BNE  %b0
	
	; Return a faliure
	MOV R0, #'4'
	BL  serial_write
	B   handle_command_return
	
1	; We have a multiple of four. Stop the current scanner
	; running if it is (as we are about to clobber the
	; null terminator it looks for).
	ORR R7, R7, #IDLE_FLAG_SCANNED_LAST
	
	; Find the end of the register table
	MOVX R1, #REG_START
0	LDR  R0, [R1], #4
	; Loop until reach the terminating NULL
	CMP  R0, #0
	BNE  %b0
	
	; Move the pointer back onto the last entry
	SUB  R1, R1, #4
	
	; Read the data into the table
	BL serial_read_data
	
	; Null terminate the table
	MOV R0, #0
	STR R0, [R1, R2]
	
	; Return OK!
	MOV R0, #'A'
	BL  serial_write
	B   handle_command_return
	
	
	;- - - - - - - - - - - - - - - - - - - - - - - - - - -
	
cmd_fr_send_fpga	; Getting an FPGA Packet
	
	; Read the number of bytes in packet (0 = 256)
	BL serial_read
	MOVS  R2, R0
	MOVEQ R2, #256
	
	; Write into the CPU's ram space as its not being used
	; for anything else...
	MOVX R1, #FPGA_DATA_BUFFER
	
	; Read the packet into a buffer (so that no bytes are
	; missed due to slowness in loading onto FPGA
	BL serial_read_data
	
	; Send buffer to fpga (returns success char in R0)
	BL fpga_send
	
	; If the download has completed, set the system up
	; ready to drive it and start the FPGA running.
	SUBS R10, R10, R2
	BGT  %f0
	
	; Set up system
	MOV R8,  #STATE_RESET ; In the reset state
	MOV R9,  #0           ; No steps executed
	MOV R10, #0           ; No steps remaining
	
	; Done! Return the success char.
0	BL  serial_write
	
	B handle_command_return
	
	;- - - - - - - - - - - - - - - - - - - - - - - - - - -
	; Get CPU status
	;- - - - - - - - - - - - - - - - - - - - - - - - - - -
cmd_wot_u_do	; CPU State
	MOV R0, R8
	BL serial_write
	
	; Steps remaining
	MOV R0, R10
	BL serial_write_word
	
	; Steps executed
	MOV R0, R9
	BL serial_write_word
	
	B handle_command_return
	
	
	;- - - - - - - - - - - - - - - - - - - - - - - - - - -
	; Stop the CPU & clear remaining steps
	;- - - - - - - - - - - - - - - - - - - - - - - - - - -
cmd_stop	; Does nothing if busy
	CMP R8,  #STATE_BUSY
	BEQ handle_command_return
	
	; Stop the CPU
	MOV R8, #STATE_STOPPED
	
	; Clear remaining steps
	MOV R10, #0
	
	B handle_command_return
	
	
	;- - - - - - - - - - - - - - - - - - - - - - - - - - -
	; Pause the CPU not clearning remaining steps
	;- - - - - - - - - - - - - - - - - - - - - - - - - - -
cmd_pause	; Does nothing if busy
	CMP R8,  #STATE_BUSY
	BEQ handle_command_return
	
	; Stop the CPU
	MOV R8, #STATE_STOPPED
	
	B handle_command_return
	
	
	;- - - - - - - - - - - - - - - - - - - - - - - - - - -
	; Resume CPU execution
	;- - - - - - - - - - - - - - - - - - - - - - - - - - -
cmd_continue	; Does nothing if busy
	CMP R8,  #STATE_BUSY
	BEQ handle_command_return
	
	; Don't resume if no steps remaining
	CMP R10, #0
	BEQ handle_command_return
	
	; Resume execution
	MOV R8, #STATE_RUNNING
	
	B handle_command_return
	
	
	;- - - - - - - - - - - - - - - - - - - - - - - - - - -
	; Memory command dispatcher
	;
	; The following registers will be set
	;   R1: The command
	;   R2: The address
	;   R3: The length to read/write
	;- - - - - - - - - - - - - - - - - - - - - - - - - - -
handle_command_mem	; Store the command in R1
	MOV R1, R0
	
	; Read the address into R2
	BL  serial_read_word
	MOV R2, R0
	
	; Read the length into R3
	BL  serial_read_half
	MOV R3, R0
	
	; If length == 0 then do nothing (if not checked the
	; below code can infini-loop
	CMP R3, #0
	BEQ cmd_nop
	
	; Registers or memory? (R0 is scratch)
	AND R0, R1, #TARGET_MASK
	CMP R0, #TARGET_MEMORY
	BEQ cmd_mem
	CMP R0, #TARGET_REGISTERS
	BEQ cmd_reg
	; Unregocgnised
	B cmd_nop
	
	;- - - - - - - - - - - - - - - - - - - - - - - - - - -
	; Memory read/write command
	;
	; Byte-wise accesses are not supported and will result
	; in undefined behaviour.
	;- - - - - - - - - - - - - - - - - - - - - - - - - - -
cmd_mem	; Read or write? (flag is stored until branch)
	TST R1, #READ_BIT
	
	; Mask off the element size and scale so that it is in
	; words and not bytes
	AND R1, R1, #ELEMENT_SIZE_MASK
	SUB R1, R1, #1
	
	; Put the target address in R1 (note that the length is
	; shifted/multiplied to take into account the element
	; size)
	ADD R1, R2, R3, LSL R1
	
	; Flag from TST is used here
	BEQ cmd_mem_write
	; Fall through to read
	
cmd_mem_read	; Read from memory
	
	; Send back a single word
0	BL  memory_read
	BL  serial_write_half
	
	; Advance to next address and loop
	ADD R2, R2, #1
	CMP R2, R1
	BNE %b0
	
	
	B handle_command_return
	
cmd_mem_write	; Write to memory
	
	; Get the word from serial and store it
0	BL  serial_read_half
	BL  memory_write
	
	; Advance to next address and loop
	ADD R2, R2, #1
	CMP R2, R1
	BNE %b0
	
	B handle_command_return
	
	;- - - - - - - - - - - - - - - - - - - - - - - - - - -
	; Register read/write command
	;
	; Byte-accesses are supported due to the CC register
	; being read by such a access.
	;
	; Behaviour when >1 word read/writes are done is
	; undefined.
	;- - - - - - - - - - - - - - - - - - - - - - - - - - -
cmd_reg	; Store a copy of R1 before masking
	MOV R0, R1
	
	; Mask off the element size and take a copy in terms of
	; words (R4) and in bytes (R0).
	AND  R1, R1, #ELEMENT_SIZE_MASK
	SUBS R4, R1, #1
	
	; If reading bytes this value will be -1 but since
	; 1<<-1 wouldn't yield a sensible step-size for each
	; read as it would when not -1, just use 0 and we'll
	; step one word at a time (which, while not strictly
	; correct, is about the most sensible thing we can do
	; given we can't step by bytes).
	MOVMI R4, #0
	
	; Read or write? (flag is stored until branch)
	TST R0, #READ_BIT
	
	; Put the target address in R4 (note that the length is
	; shifted/multiplied to take into account the element
	; size)
	ADD R4, R2, R3, LSL R4
	
	; Flag from TST is used here
	BEQ cmd_reg_write
	; Fall through to read
	
cmd_reg_read	; Read from register
	
	; Get the register's valuek
0	BL  register_read
	
	; Act depending on byte access
	CMP R1, #0
	BEQ %f1
	
	; Half-word access
	BL  serial_write_half
	; Advance to next address and loop
	ADD R2, R2, #1
	CMP R2, R4
	BNE %b0
	B   handle_command_return
	
	; Byte access
1	BL  serial_write
	; Advance to next address and loop
	ADD R2, R2, #1
	CMP R2, R4
	BNE %b0
	B   handle_command_return
	
	
cmd_reg_write	; Write to memory
	
0	; Act depending on byte access
	CMP R1, #0
	BEQ %f1
	
	; Half-word access
	BL  serial_read_half
	BL  register_write
	; Advance to next address and loop
	ADD R2, R2, #1
	CMP R2, R4
	BNE %b0
	B   handle_command_return
	
1	; Byte access
	BL  serial_read
	BL  register_write
	; Advance to next address and loop
	ADD R2, R2, #1
	CMP R2, R4
	BNE %b0
	B   handle_command_return
	
	
	;- - - - - - - - - - - - - - - - - - - - - - - - - - -
	; Execution control command handler
	;
	; None of the flags included in the command are
	; currently supported and will be ignored.
	;- - - - - - - - - - - - - - - - - - - - - - - - - - -
handle_command_exec	; Read the number of steps
	BL  serial_read_word
	
	; Do nothing if busy
	CMP R8,  #STATE_BUSY
	BEQ handle_command_return
	
	; Set the number of steps remaining
	MOV R10, R0
	
	; Start execution
	MOV R8, #STATE_RUNNING
	
	; Done
	B handle_command_return
	
	
	;- - - - - - - - - - - - - - - - - - - - - - - - - - -
	; No-operation
	;- - - - - - - - - - - - - - - - - - - - - - - - - - -
cmd_nop	; Not implemented, just fall through to return
	
	; Return
handle_command_return	LDMFD SP!, {R0-R3, PC}


	;-----------------------------------------------------
	; Get the FPGA ready for a download.
	;-----------------------------------------------------
fpga_init	STMFD SP!, {R1-R2, LR}
	
	; Get PIO port base address
	MOVX LR, #PIOC
	
	; CS=0
	MOV R6, #FPGA_CS_B
	STR R6, [LR, #PIO_CODR]

	; Put the FPGA in programming mode (set PROG_B = 0)
	MOV  R1, #FPGA_PROG_B
	STR  R1, [LR, #PIO_CODR]
	
	; Wait for >500ns to allow memory clearing to start
	MOV  R2, #125
0	SUBS R2, R2, #1
	BNE  %b0
	
	; Check INIT_B is low indicating FPGA is ready to be
	; programmed
1	LDR  R2, [LR, #PIO_PDSR]
	TST  R2, #FPGA_INIT_B
	BNE  %b1
	
	; Set PROG_B high again
	STR  R1, [LR, #PIO_SODR]
	
	; Wait for FPGA to be ready (INIT_B to become 1)
2	LDR  R2, [LR, #PIO_PDSR]
	TST  R2, #FPGA_INIT_B
	BEQ  %b2
	
	; CS=1
	MOV R6, #FPGA_CS_B
	STR R6, [LR, #PIO_SODR]
	
	; Download should not commence for >5us. This is
	; assumed to be ensured by the speed of the serial
	; port.
	
	LDMFD SP!, {R1-R2, PC}
	
	;-----------------------------------------------------
	; Load some data (from a bit file) into the FPGA
	;
	; Arguments:
	;   R1: Start-address of data in memory
	;   R2: Number of bytes
	;
	; Returns:
	;   R0: "A" if good, "N" otherwise.
	;-----------------------------------------------------
fpga_send	STMFD SP!, {R1-R2, R5-R6, LR}
	
	; Get Port-C's base address
	MOVX LR, #PIOC
	
	; CS=0
	MOV R6, #FPGA_CS_B
	STR R6, [LR, #PIO_CODR]
	
	MOV R6, #FPGA_CCLK

fpga_send_byte	; CCLK low
	STR R6, [LR, #PIO_CODR]
	
	; Check for FPGA load failiure
	LDR R5, [LR, #PIO_PDSR]
	TST R5, #FPGA_INIT_B
	
	; Load failed (e.g. checksum failed), return 'C'
	MOVEQ R0, #'C'
	BEQ   fpga_send_return
	
	; Get the byte to send from the buffer
	LDRB R0, [R1], #1
	
	; Shift up to bits 30-23 (the data pins on the bus) and
	; send to the bus
	MOV R0, R0, LSL #FPGA_BUS_SHIFT
	STR R0, [LR, #PIO_ODSR]
	
	; CCLK high
	STR R6, [LR, #PIO_SODR]
	
	; Decrement the counter
	SUBS R2, R2, #1
	BNE  fpga_send_byte
	
	MOV  R0, #'A'
	
	
fpga_send_return	; CCLK low
	STR R6, [LR, #PIO_CODR]
	
	; CS=1
	MOV R6, #FPGA_CS_B
	STR R6, [LR, #PIO_SODR]
	
	LDMFD SP!, {R1-R2, R5-R6, PC}
	
	
	
	;-----------------------------------------------------
	; Idle process which should be executed on a frequent
	; basis. Good times are while blocking on IO and each
	; iteration of a mainloop. This call does not block.
	;
	; Contains a state machine which clocks the CPU,
	; scans-out register values and emulates memory for the
	; CPU running in the FPGA.
	;
	; A system timer is used to check when suitable delays
	; have occurred between signals being produced (for
	; example a clock signal).
	;-----------------------------------------------------
idle_process	STMFD SP!, {R1-R2, LR}
	
	MOVX  LR, #CPU_INFO
	
	; Update the CPU type (checking if it has changed)
	LDRH  R1, [R12, #FPGA_REG_DUT_CPU_TYPE]
	LDRB  R2, [LR, #CPU_INFO_CPU_TYPE]
	CMP   R1, R2 ; Note if it changed
	STRB  R1, [LR, #CPU_INFO_CPU_TYPE]
	
	; Update the CPU sub-type bottom-byte
	LDRH  R1, [R12, #FPGA_REG_DUT_CPU_SUBTYPE]
	LDRB  R2, [LR, #CPU_INFO_CPU_SUBTYPE]
	CMPEQ R1, R2 ; Note if it changed also
	STRB  R1, [LR, #CPU_INFO_CPU_SUBTYPE]
	
	; Load the register table if the CPU changed and not
	; unconfigured (unconfigured FPGAs return FFFF on all
	; outputs and thus set the top 8 bits of the subtype)
	BEQ    %f1
	TST    R1, #0xFF00
	BNE    %f2 ; Don't load table if FPGA not programmed
	BL     get_default_reg_table
	ADRLEQ R0, REG_DEFAULTS_EMPTY
	BL     load_reg_table
	B      %f0 ; Table loaded, resume idle_process
	
	; Check that the FPGA isn't unconfigured (i.e.
	; returning all 1s for the sub-type field (the top byte
	; should be zeros))
1	TST   R1, #0xFF00
	BEQ   %f0
	
	; Not programmed, set state to busy and give up.
2	MOV R8, #STATE_BUSY
	; Also set the state to the start state and assert the
	; registers available flag so that nothing will be
	; stuck waiting for the registers to become available.
	MOV R7, #IDLE_START
	ORR R7, R7, #IDLE_FLAG_REGS_AVAIL
	B   idle_process_return
	
	; Jump depemding on the state of the idle process
0	AND LR, R7, #IDLE_STATE_MASK
	ADD PC, PC, LR, LSL #2
	NOP
	; Start of jump-table
	B idle_start
	B idle_resetting
	B idle_normal
	B idle_clock_init
	B idle_clock
	B idle_scan_init
	B idle_scanning
	; End of table
	
	
	;- - - - - - - - - - - - - - - - - - - - - - - - - - -
	; Idle process start: try and spin up a newly connected
	; device. This means reset it.
	;- - - - - - - - - - - - - - - - - - - - - - - - - - -
idle_start	; Reset the CPU
	MOV  LR, #DUT_RESET
	STRH LR, [R12, #FPGA_REG_DUT_CONTROL]
	
	; Make sure the scan-path is off
	MOV  LR, #0
	STRH LR, [R12, #FPGA_REG_SCAN_CONTROL]
	
	; Zero-out the DUT's registers
	BL zero_registers
	
	; Update state
	MOV R8,  #STATE_RESET
	MOV R9,  #0 ; Steps executed
	MOV R10, #0 ; Steps remaining
	
	; Start the idle process in the resetting state with
	; the cycle counter zeroed out
	BIC R7, R7, #IDLE_COUNT_MASK
	BIC R7, R7, #IDLE_STATE_MASK
	ORR R7, R7, #IDLE_RESETTING
	BL  reset_timer
	
	B idle_process_return
	
	
	;- - - - - - - - - - - - - - - - - - - - - - - - - - -
	; Allow the reset line to be held down for some period
	;- - - - - - - - - - - - - - - - - - - - - - - - - - -
idle_resetting	; If the timer hasn't expired, wait.
	BL  hold_time_elapsed
	BLT idle_process_return
	
	; Is the reset signal still high
	LDRH LR, [R12, #FPGA_REG_DUT_CONTROL]
	TST  LR, #DUT_RESET
	BEQ  %f0
	
	; Reset signal is still high, clear it
	MOV  LR, #0
	STRH LR, [R12, #FPGA_REG_DUT_CONTROL]
	BL   reset_timer
	B    idle_process_return
	
	; Reset signal has been lifted.
	; Set the idle process to the normal state
0	ORR R7, R7, #IDLE_FLAG_RESET
	BIC R7, R7, #IDLE_STATE_MASK
	ORR R7, R7, #IDLE_NORMAL
	
	B idle_process_return
	
	;- - - - - - - - - - - - - - - - - - - - - - - - - - -
	; Nothing in particular is being done, the CPU is
	; initialised and reset. Look for things to do.
	;- - - - - - - - - - - - - - - - - - - - - - - - - - -
idle_normal	; Are the register values are being requested?
	TST R7, #(IDLE_REQ_REG_READ|IDLE_REQ_REG_WRITE)
	BEQ %f0
	
	; Scan the values out!
	; Send a clock pulse to the scanpath while it is
	; disabled to cause the scan-registers to sample the
	; register values.
idle_normal_do_scan	MOV  LR, #SCAN_CLK
	STRH LR, [R12, #FPGA_REG_SCAN_CONTROL]
	BL   reset_timer
	
	; Enter the scan initialisation state and clear the
	; request and flags.
	BIC R7, R7, #IDLE_STATE_MASK
	ORR R7, R7, #IDLE_SCAN_INIT
	B   idle_process_return
	
	;- - - - - - - - - - - - - - - - - - - - - - - - - - -
	
0	; If a clock pulse has been requested, clock it!
	TST R7, #IDLE_REQ_CLOCK
	BNE %f1
	; If processor is running, also clock it!
	CMP  R8, #STATE_RUNNING
	BNE  %f0
	
1	; Update the memory interface
	BL emulate_memory
	
	BL reset_timer
	
	; Enter the clocking init state, clearing the clock
	; request, invalidate fetched register values, and
	; note that we're not reset anymore.
	BIC R7, R7, #IDLE_REQ_CLOCK
	BIC R7, R7, #IDLE_FLAG_REGS_AVAIL | IDLE_FLAG_RESET
	BIC R7, R7, #IDLE_STATE_MASK
	ORR R7, R7, #IDLE_CLOCK_INIT
	B   idle_process_return
	
	;- - - - - - - - - - - - - - - - - - - - - - - - - - -
	
0	; If the processor is being reset, reset it unless it
	; has already been reset
	CMP  R8, #STATE_RESET
	BNE  %f0
	TST  R7, #IDLE_FLAG_RESET
	BNE  %f0
	MOV  LR, #DUT_RESET
	STRH LR, [R12, #FPGA_REG_DUT_CONTROL]
	
	; Enter the reset state and invalidate fetched register
	; values and clear the cycle count
	BIC R7, R7, #IDLE_COUNT_MASK
	BIC R7, R7, #IDLE_FLAG_REGS_AVAIL
	BIC R7, R7, #IDLE_STATE_MASK
	ORR R7, R7, #IDLE_RESETTING
	B   idle_process_return
	
	;- - - - - - - - - - - - - - - - - - - - - - - - - - -
	
0	; Nothing to do, just keep the memory interface
	; up-to-date.
	BL emulate_memory
	
	B idle_process_return
	
	
	;- - - - - - - - - - - - - - - - - - - - - - - - - - -
	; A delay step before clocking begins where the memory
	; data fetched (when this state was selected) is
	; allowed to settle.
	;- - - - - - - - - - - - - - - - - - - - - - - - - - -
idle_clock_init	; Has the timer expired?
	BL  hold_time_elapsed
	BLT idle_process_return
	
	; Clear the clock count if the CPU is fetching
	LDRH  LR, [R12, #FPGA_REG_DUT_FETCH]
	CMP   LR, #1
	BICEQ R7, R7, #IDLE_COUNT_MASK
	
	; Increment the clock count
	MOV R1, R7, LSR #IDLE_COUNT_SHIFT
	CMP R1, #0xFF
	BEQ idle_clock_init_t_out
	ADD R1, R1, #1
	BIC R7, R7, #IDLE_COUNT_MASK
	ORR R7, R7, R1, LSL #IDLE_COUNT_SHIFT
	
	; Set the clock high
	MOV  LR, #DUT_CLK
	STRH LR, [R12, #FPGA_REG_DUT_CONTROL]
	
1	; Start clocking normally
	BIC R7, R7, #IDLE_STATE_MASK
	ORR R7, R7, #IDLE_CLOCK
	
	B idle_process_return
	
idle_clock_init_t_out	; The device has been clocked 255 times without a
	; fetch, stop the CPU (time out!).
	
	MOVEQ R8, #STATE_STOPPED_PROG_REQ
	
1	; Return to the normal state
	BIC R7, R7, #IDLE_STATE_MASK
	ORR R7, R7, #IDLE_NORMAL
	
	B idle_process_return
	
	;- - - - - - - - - - - - - - - - - - - - - - - - - - -
	; The clock is being cycled. Clear it after one timeout
	; then return to the start state after another.
	;- - - - - - - - - - - - - - - - - - - - - - - - - - -
idle_clock	; Has the timer expired?
	BL  hold_time_elapsed
	BLT idle_process_return
	
	; Timer has expired, check which state the clock is in
	LDRH LR, [R12, #FPGA_REG_DUT_CONTROL]
	TST  LR, #DUT_CLK
	BEQ  idle_clock_low
	
	; The clock is high, clear it
	BIC  LR, LR, #DUT_CLK
	STRH LR, [R12, #FPGA_REG_DUT_CONTROL]
	
	; Do any memory accesses the CPU is requesting (at the
	; negative edge of the clock)
	BL emulate_memory
	
	; Wait again
	BL reset_timer
	B  idle_process_return
	
idle_clock_low	; The clock is low so we've now finished cycling it
	; Check to see if the CPU is now in the fetch state
	; (i.e. a step has completed)
	LDRH LR, [R12, #FPGA_REG_DUT_FETCH]
	TST  LR, #1
	BEQ  %f1
	
	; Fetch is high, step the counters
	ADD R9, R9, #1
	
	; Are there unlimited steps? If so, don't decrement the
	; counter
	CMP R10, #0
	BEQ %f1
	
	; Decrement the step counter and stop if we've run out
	SUBS  R10, R10, #1
	MOVEQ R8, #STATE_STOPPED
	
1	; Return to the normal state, clocking finished
	BIC R7, R7, #IDLE_STATE_MASK
	ORR R7, R7, #IDLE_NORMAL
	
	B idle_process_return
	
	
	;- - - - - - - - - - - - - - - - - - - - - - - - - - -
	; The scan-path is being initialised
	;- - - - - - - - - - - - - - - - - - - - - - - - - - -
idle_scan_init	; Has the timer expired?
	BL  hold_time_elapsed
	BLT idle_process_return
	
	; Timer has expired, check which state the scan clock
	; is in
	LDRH LR, [R12, #FPGA_REG_SCAN_CONTROL]
	CMP  LR, #0
	BEQ  idle_scan_init_low
	CMP  LR, #SCAN_EN
	BEQ  idle_scan_init_enabled
	
	; The clock is high but we're not enabled, clear it
	BIC  LR, LR, #SCAN_CLK
	STRH LR, [R12, #FPGA_REG_SCAN_CONTROL]
	
	; Wait again
	BL reset_timer
	B  idle_process_return
	
idle_scan_init_low	; The clock is low so we've now finished cycling it.
	; Enable the scan-path.
	MOV  LR, #SCAN_EN
	STRH LR, [R12, #FPGA_REG_SCAN_CONTROL]
	BL  reset_timer
	
	B idle_process_return
	
idle_scan_init_enabled	; The scan path is enabled and the clock low, snatch
	; the first bit which will now be exposed on scan-out
	BL   scan_init
	BL   scan_bit
	
	; Set the flag if the last bit was just reached
	ORREQ R7, R7, #IDLE_FLAG_SCANNED_LAST
	
	; Start scanning
	BIC R7, R7, #IDLE_STATE_MASK
	ORR R7, R7, #IDLE_SCANNING
	BL   reset_timer
	
	B idle_process_return
	
	
	;- - - - - - - - - - - - - - - - - - - - - - - - - - -
	; The scan-path is being operated
	;- - - - - - - - - - - - - - - - - - - - - - - - - - -
idle_scanning	; Has the timer expired?
	BL  hold_time_elapsed
	BLT idle_process_return
	
	; Timer has expired, check which state the scan clock
	; is in
	LDRH LR, [R12, #FPGA_REG_SCAN_CONTROL]
	TST  LR, #SCAN_CLK
	BEQ  idle_scanning_low
	
	; The clock is high, clear it
	BIC  LR, LR, #SCAN_CLK
	STRH LR, [R12, #FPGA_REG_SCAN_CONTROL]
	
	; Did we previously scan the last bit? If so we've now
	; clocked it in so the scan is complete
	TST   R7, #IDLE_FLAG_SCANNED_LAST
	ORRNE R7, R7, #IDLE_FLAG_SCAN_COMPLETE
	BNE   %f0
	
	; Snatch the bit on this negative edge
	BL scan_bit
	; Set the flag if the last bit was just reached
	ORREQ R7, R7, #IDLE_FLAG_SCANNED_LAST
	
	; Wait again
0	BL reset_timer
	B  idle_process_return
	
idle_scanning_low	; The clock is low
	; Check to see if we're done scanning, if not, start
	; the next clock.
	TST R7, #IDLE_FLAG_SCAN_COMPLETE
	BEQ idle_scanning_next
	
	; If the scan path is disabled, we're done, if not,
	; disable it and wait.
	TST  LR, #SCAN_EN
	BNE  %f0
	
	; Finished scanning, indicate register values are valid
	ORR R7, R7, #IDLE_FLAG_REGS_AVAIL
	
	; Scan-path already disabled, Return to normal state
	BIC R7, R7, #(IDLE_REQ_REG_READ|IDLE_REQ_REG_WRITE)
	BIC R7, R7, #(IDLE_FLAG_SCAN_COMPLETE|IDLE_FLAG_SCANNED_LAST)
	BIC R7, R7, #IDLE_STATE_MASK
	ORR R7, R7, #IDLE_NORMAL
	B   idle_process_return
	
	; Scan-path not disabled, disable it and wait
0	BIC  LR, LR, #SCAN_EN
	STRH LR, [R12, #FPGA_REG_SCAN_CONTROL]
	BL reset_timer
	B  idle_process_return
	
idle_scanning_next	; Not finished counting, set the clock high to move
	; onto the next bit
	ORR  LR, LR, #SCAN_CLK
	STRH LR, [R12, #FPGA_REG_SCAN_CONTROL]
	BL reset_timer
	B  idle_process_return
	
	

idle_process_return	LDMFD SP!, {R1-R2, PC}
	
	
	
	;-----------------------------------------------------
	; Reset the registers in the register table to
	; zero/clean.
	;-----------------------------------------------------
zero_registers	STMFD SP!, {R0,R1, LR}
	
	; A mask of all non register-width fields
	MVN  R0, #0x00FF0000
	
	; The register table address
	MOVX LR, #REG_START
	
	; Zero out the value & dirty flag
0	LDR  R1, [LR]
	BICS R1, R1, R0
	STR  R1, [LR], #4
	; If only zeros remain we've reached the last (null)
	; element indicating the end of the table so stop
	BNE %b0
	
	LDMFD SP!, {R0,R1, PC}
	
	
	;-----------------------------------------------------
	; Reset the system timer
	;-----------------------------------------------------
reset_timer	STMFD SP!, {R0, LR}
	
	; Trigger (reset) the timer
	MOVX LR, #TC_BASE
	MOVX R0, #(1<<2)
	STR  R0, [LR, #TC_CCR]
	
	LDMFD SP!, {R0, PC}
	
	
	;-----------------------------------------------------
	; Has the hold time elapsed since the system timer was
	; elapsed? Sets the flags such that GE if the hold
	; time has elapsed and LT if not.
	;-----------------------------------------------------
hold_time_elapsed	STMFD SP!, {LR}
	
	; Trigger (reset) the timer
	MOVX LR, #TC_BASE
	LDR  LR, [LR, #TC_CV]
	CMP  LR, #HOLD_TIME
	
	LDMFD SP!, {PC}
	
	
	;-----------------------------------------------------
	; Act on any memory accesses initiated by the attached
	; CPU. Reads the address and control buses and responds
	; by either storing the value on the data bus or
	; sending out a value on the read bus.
	;-----------------------------------------------------
emulate_memory	STMFD SP!, {R0, R2, LR}
	
	; Read or write?
	LDRH LR, [R12, #FPGA_REG_DUT_RWEN]
	TST  LR, #DUT_RD
	BNE  emulate_read
	TST  LR, #DUT_WR
	BNE  emulate_write
	B    emulate_memory_return
	
emulate_write	; Get the address requested and the data and then store
	; it into memory.
	LDRH R2, [R12, #FPGA_REG_DUT_ADDR]
	LDRH R0, [R12, #FPGA_REG_DUT_DATA_OUT]
	BL   memory_write
	B    emulate_memory_return
	
emulate_read	; Get the address requested, do the read and send the
	; result to the device.
	LDRH R2, [R12, #FPGA_REG_DUT_ADDR]
	BL   memory_read
	STRH R0, [R12, #FPGA_REG_DUT_DATA_IN]
	; Fall through to return
	
	
emulate_memory_return	LDMFD SP!, {R0, R2, PC}
	
	
	;-----------------------------------------------------
	; Reset the counters used by scan_bit. Call this prior
	; to scanning data through the system.
	;-----------------------------------------------------
scan_init	STMFD SP!, {R0, LR}
	
	; Reset the counters
	MOV  R0, #0
	MOVX LR, #REG_CUR_NUM
	STR  R0, [LR]
	STR  R0, [LR, #4]
	
	LDMFD SP!, {R0, PC}
	
	
	;-----------------------------------------------------
	; Put the most recently scanned bit into a register
	; and either send it back into the scan path or send in
	; a new value if overwriting the register.
	;
	; Sets the flags. EQ if this is the last bit, NE
	; otherwise.
	;-----------------------------------------------------
scan_bit	STMFD SP!, {R0-R3, LR}
	
	; Get the current register/current bit
	MOVX LR, #REG_CUR_NUM
	LDR  R0, [LR]
	LDR  R1, [LR, #4]
	
	; Get the address of the register in the register
	; table
	MOVX LR, #REG_START
	ADD  LR, LR, R0, LSL #2
	
	; Load the register table entry
	LDRH R2, [LR]
	
	; Is the register dirty (i.e. to be written back?)
	LDRB R3, [LR, #3]
	TST R3, #SCAN_DIRTY<<16
	BNE scan_bit_dirty
	
	;- - - - - - - - - - - - - - - - - - - - - - - - - - -
	
	; This bit is clean, clear it and set it from the
	; scanpath
	MOV  R3, #1
	BIC  R2, R2, R3, LSL R1
	LDRH R3, [R12, #FPGA_REG_SCAN_OUT]
	ORR  R2, R2, R3, LSL R1
	
	; Set the value in the register table
	STRH R2, [LR]
	
	; Stuff the bit coming out of the scan path back in
	STRH R3, [R12, #FPGA_REG_SCAN_IN]
	
	B scan_bit_return
	
	;- - - - - - - - - - - - - - - - - - - - - - - - - - -
	
scan_bit_dirty	; This bit is dirty, send it to the scanpath
	MOV  R2, R2, LSR R1
	AND  R2, R2, #1
	STRH R3, [R12, #FPGA_REG_SCAN_IN]
	
	;- - - - - - - - - - - - - - - - - - - - - - - - - - -
	
scan_bit_return	; Increment the bit counter
	ADD R1, R1, #1
	
	; Check it isn't past the end of the field's length
	LDRB  R2, [LR, #2]
	CMP   R1, R2
	BLT   %f0
	
	; If it is, wrap around to next register
	MOV  R1, #0
	ADD  R0, R0, #1
	; And clear the dirty flag
	STRB R1, [LR, #3]
	
	; Set the flags depending on whether the next register
	; has any bits (if its 0 then we just scanned the last
	; bit
	LDRB R2, [LR, #4+2]
	CMP  R2, #0
	B %f1
	
0	; Set the flags indicating there are still more bits to
	; scan
	CMP R1, #0 ; Just anything that says != 0
	
	; Store the current register/current bit
1	MOVX LR, #REG_CUR_NUM
	STR  R0, [LR]
	STR  R1, [LR, #4]
	
	LDMFD SP!, {R0-R3, PC}
