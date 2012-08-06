	; ARM Host
	
	include header.s
	include usart.h
	include pio.h
	
	
	;------------------------------------------------------
	; Global Definitions
	;
	; The following registers are globals:
	;
	; R8:  CPU State
	; R9:  Steps executed since reset
	; R10: Steps remaining
	; R11: USART Base
	; R12: FPGA Base
	; R13: SP
	; R14: LR
	;------------------------------------------------------

maker	EQU	3
version	EQU	0
day	EQU	5
month	EQU	8
year	EQU	12
	
	; XXX: The location of the program in RAM
ROM_START	EQU 0x30000
	
	; Default MMU translation table included in ROM
TRANSLATION_TABLE	EQU 0x8000 | ROM_base
	
	; The serial port to use
USART_BASE	EQU USART1_BASE
	
	; The CPU type (3 = STUMP, 4 = MU0)
CPU_TYPE	EQU 3
CPU_SUBTYPE	EQU 0
	
	; The FPGA (feature) type
FPGA_TYPE	EQU 14
FPGA_SUBTYPE	EQU 0x0200
	
	;------------------------------------------------------
	; Simulation Definitions
	;------------------------------------------------------
	
	; Start address of the memory the STUMP uses (in the
	; ARM address space
MEMORY_START	EQU 0x00000000
	; STUMP Memory address (16-bit word wise address) mask
MEMORY_MASK	EQU 0xFFFF
	
	; Start address of the memory the STUMP registers use
REGISTERS_START	EQU (MEMORY_MASK<<1) + 1
	; Number of registers (8 registers + flags)
REGISTERS_NUM	EQU 9
	
	
	;------------------------------------------------------
	; Komodo Protocol Constants
	;------------------------------------------------------
	
	; Reponse to PINGs ("OK00" as ASCII)
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
FPGA_SCAN_CONTROL	EQU 0x00
FPGA_SCAN_IN	EQU 0x02
FPGA_SCAN_OUT	EQU 0x04
FPGA_STUMP_CONTROL	EQU 0x06
FPGA_STUMP_FETCH	EQU 0x08
FPGA_STUMP_ADDR	EQU 0x0A
FPGA_STUMP_RWEN	EQU 0x0C
FPGA_STUMP_DATA_OUT	EQU 0x0E
FPGA_STUMP_DATA_IN	EQU 0x10
	
STUMP_CLK	EQU 0x1
STUMP_RESET	EQU 0x2
STUMP_RD	EQU 0x1
STUMP_WR	EQU 0x2
	
SCAN_CLK	EQU 0x1
SCAN_EN	EQU 0x2
	
	
	;------------------------------------------------------
	; Start of ROM Image
	;------------------------------------------------------
	
	ORG ROM_START
	ENTRY
	
	B main
	
	; Pack version & date into 32 bits
Version_ID	DCD maker*&1000000 + version*&10000 + day*&800 + month*&80 + year
	; String describing the program
	DCB "FPGA CPU Debugger "
	DCB "Jonathan Heathcote, (c) University of Manchester "
	DCB "August 2012"
	
	ALIGN
	
	;------------------------------------------------------
	; Data
	;------------------------------------------------------
	
CPU_INFO	; Length of info
	DCW CPU_INFO_END-CPU_INFO_BODY
	
	; CPU type information
CPU_INFO_BODY	DCB CPU_TYPE
	DCW CPU_SUBTYPE
	
	; One feature: a Spartan-3 FPGA
	DCB 1
	DCB FPGA_TYPE
	DCW FPGA_SUBTYPE
	
	; One memory segment
	DCB 1
	DCD 0
	DCD (MEMORY_MASK+1)
CPU_INFO_END
	
	ALIGN
	
	
	;------------------------------------------------------
	; Host program system initialisation & mainloop
	;------------------------------------------------------
	
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
	
	; Set up global register values
	MOVX R8,  #STATE_RESET ; CPU State
	MOVX R9,  #0 ; Steps executed
	MOVX R10, #0 ; Steps remaining
	MOVX R11, #USART_BASE
	MOVX R12, #FPGA_BASE
	
	; XXX
	MOV R5, #0
	
	
	; Main-loop: handle new commands
main_loop	BL handle_command
	B main_loop
	
	; Shouldn't get here!
	B .
	
	
	
	
	;------------------------------------------------------
	; Read a byte from the serial port. This function will
	; block until a byte is read.
	;
	; Returns:
	;   R0: The byte read
	;------------------------------------------------------
	
serial_read	; Check the status register to see if data is available
	LDR R0, [R11, #US_CSR]
	TST R0, #US_RXRDY
	BEQ serial_read
	
	; Data is available, retrieve it
	LDR R0, [R11, #US_RHR]
	
	; Mask to just the byte
	AND R0, R0, #0xFF
	
	; Return
	MOV PC, LR
	
	
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
	; is ready. Busy-loop until it is.
0	LDR LR, [R11, #US_CSR]
	TST LR, #US_TXRDY
	BEQ %b0
	
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
	; Read from the memory space occupied by the CPU
	;
	; Argument:
	;   R2: Address
	; Returns:
	;   R0: Value
	;------------------------------------------------------
memory_read	STMFD SP!, {LR}
	
	; Mask off the address and convert into a byte address
	MOVX  R0, #(MEMORY_MASK<<1)
	AND   LR, R0, R2, LSL #1
	
	; Load the value
	LDRH  R0, [LR, #MEMORY_START]
	
	LDMFD SP!, {PC}
	
	
	;------------------------------------------------------
	; Write to the memory space occupied by the CPU
	;
	; Argument:
	;   R0: Value
	;   R2: Address
	;------------------------------------------------------
memory_write	STMFD SP!, {R2, LR}
	
	; Mask off the address and convert into a byte address
	MOVX  LR, #(MEMORY_MASK<<1)
	AND   R2, LR, R2, LSL #1
	
	; Store the value
	STRH  R0, [R2, #MEMORY_START]
	
	LDMFD SP!, {R2, PC}
	
	
	;------------------------------------------------------
	; Read register values from the CPU
	;
	; Argument:
	;   R2: Register 'Address'
	; Returns:
	;   R0: Value
	;------------------------------------------------------
register_read	STMFD SP!, {LR}
	
	; Do nothing if out of range
	CMP R2, #REGISTERS_NUM
	BHS %f0
	
	; Calculate offset (register addresses are for
	; half-words)
	MOVX LR, #REGISTERS_START
	ADD  LR, LR, R2, LSL #1
	
	; Load the register's value
	LDRH  R0, [LR]
	
0	LDMFD SP!, {PC}
	
	
	;------------------------------------------------------
	; Write to a register in the CPU.
	; TODO: Add register write support.
	;
	; Argument:
	;   R0: Value
	;   R2: Register 'Address'
	;------------------------------------------------------
register_write	; XXX: Not implemented!
	MOV PC, LR
	
	
	;------------------------------------------------------
	; Read and execute a command
	;------------------------------------------------------
	; XXX
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
	B cmd_nop;cmd_fr_get      ; 0x10 Feature get status
	B cmd_nop;cmd_fr_set      ; 0x11 Feature set status
	B cmd_nop;cmd_fr_write    ; 0x12 Feature send message
	B cmd_nop;cmd_fr_read     ; 0x13 Feature get message
	B cmd_nop;cmd_fr_file     ; 0x14 Feature download header
	B cmd_nop;cmd_fr_send     ; 0x15 Feature download data
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
cmd_wot_r_u	ADR  R1, CPU_INFO
	MOVX R2, #(CPU_INFO_END - CPU_INFO)
	BL   serial_write_data
	B    handle_command_return
	
	
	;- - - - - - - - - - - - - - - - - - - - - - - - - - -
	; Reset the CPU being tested
	;- - - - - - - - - - - - - - - - - - - - - - - - - - -
cmd_reset	MOV R8, #STATE_RESET ; Enter reset state
	MOV R9,  #0          ; Reset step counter
	MOV R10, #0          ; No more steps to execute
	
	B handle_command_return
	
	
	;- - - - - - - - - - - - - - - - - - - - - - - - - - -
	; Get CPU status
	;- - - - - - - - - - - - - - - - - - - - - - - - - - -
cmd_wot_u_do	; CPU State
	MOV R0, R8
	BL serial_write
	
	; Steps executed
	MOV R0, R9
	BL serial_write_word
	
	; Steps remaining
	MOV R0, R10
	BL serial_write_word
	
	B handle_command_return
	
	
	;- - - - - - - - - - - - - - - - - - - - - - - - - - -
	; Stop the CPU & clear remaining steps
	;- - - - - - - - - - - - - - - - - - - - - - - - - - -
cmd_stop	; Stop the CPU
	MOV R8, #STATE_STOPPED
	
	; Clear remaining steps
	MOV R10, #0
	
	B handle_command_return
	
	
	;- - - - - - - - - - - - - - - - - - - - - - - - - - -
	; Pause the CPU not clearning remaining steps
	;- - - - - - - - - - - - - - - - - - - - - - - - - - -
cmd_pause	; Stop the CPU
	MOV R8, #STATE_STOPPED
	
	B handle_command_return
	
	
	;- - - - - - - - - - - - - - - - - - - - - - - - - - -
	; Resume CPU execution
	;- - - - - - - - - - - - - - - - - - - - - - - - - - -
cmd_continue	; Don't resume if no steps remaining
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
cmd_reg	; Read or write? (flag is stored until branch)
	TST R1, #READ_BIT
	
	; Mask off the element size and take a copy in words
	; (R4) and in bytes (R1).
	AND R1, R1, #ELEMENT_SIZE_MASK
	SUB R4, R1, #1
	
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
	BLO %b0
	B   handle_command_return
	
1	; Byte access
	BL  serial_read
	BL  register_write
	; Advance to next address and loop
	ADD R2, R2, #1
	CMP R2, R4
	BLO %b0
	B   handle_command_return
	
	
	;- - - - - - - - - - - - - - - - - - - - - - - - - - -
	; Execution control command handler
	;
	; None of the flags included in the command are
	; currently supported and will be ignored.
	;- - - - - - - - - - - - - - - - - - - - - - - - - - -
handle_command_exec	; Read the number of steps
	BL  serial_read_word
	
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
	
	; Return XXX
handle_command_return	LDMFD SP!, {R0-R3, PC}
