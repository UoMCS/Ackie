;------------------------------------------------------------------------------
; Headers and definitions for AT91 basic set up.
; Last modified for SAM9  8/1/08

; General ARM headers

I_bit    EQU	&00000080		; Interrupt disable bit in status word
F_bit    EQU	&00000040		; FIQ disable bit in status word
T_bit    EQU	&00000020		; Thumb bit mask in status word

TRUE		EQU	-1
FALSE		EQU	 0

Mode_bits	EQU	&1F		; Bits considered as operating mode
User_mode	EQU	&10
FIQ_mode	EQU	&11
IRQ_mode	EQU	&12
Supervisor_mode	EQU	&13
Abort_mode	EQU	&17
Undefined_mode	EQU	&1B
System_mode	EQU	&1F

mode32		EQU	&10

;------------------------------------------------------------------------------
; MMU and CP15 parameters

CR1_base_value	equ	&0005_0078	; "SBO" bits in register 1
MMU_enable	equ	&0000_0001	; MMU enable
Align_enable	equ	&0000_0002	; Alignment fault enable
Dcache_enable	equ	&0000_0004	; D cache enable
Bigendian	equ	&0000_0080	; Big endian
SR_no_acc	equ	&0000_0000	; No access for AP=00
SR_sys_RO	equ	&0000_0100	; Privileged read-only for AP=00
SR_all_RO	equ	&0000_0200	; Read-only for AP=00
Icache_enable	equ	&0000_1000	; I cache enable bit
High_vectors	equ	&0000_2000	;
Round_robin	equ	&0000_4000	; Cache replacement (0=random)
L4		equ	&0000_8000	; Prevent Thumb switch when loading PC

;------------------------------------------------------------------------------
; HD44780 control codes

LCD_clear	equ	&01		; Clear screen
LCD_home	equ	&02		; Home cursor
LCD_entry_d	equ	&04		; Cursor decrement, no shift
LCD_entry_ds	equ	&05		; Cursor decrement, display shift
LCD_entry_i	equ	&06		; Cursor increment, no shift
LCD_entry_is	equ	&07		; Cursor increment, display shift
LCD_off		equ	&08		; Display off
LCD_on		equ	&0C		; Display on, no cursor
LCD_on_cursor	equ	&0E		; Display on with cursor
LCD_on_blink	equ	&0F		; Display on with blinking cursor
LCD_cursor_L	equ	&10		; Set: move cursor left
LCD_cursor_R	equ	&14		; Set: move cursor right
LCD_display_L	equ	&18		; Set: move display left
LCD_display_R	equ	&1C		; Set: move display right
LCD_fn_4_1_7	equ	&20		; 4 bit data; one line;   7 pixel font
LCD_fn_4_1_10	equ	&24		; 4 bit data; one line;  10 pixel font
LCD_fn_4_2_7	equ	&28		; 4 bit data; two lines;  7 pixel font
LCD_fn_4_2_10	equ	&2C		; 4 bit data; two lines; 10 pixel font
LCD_fn_8_1_7	equ	&30		; 8 bit data; one line;   7 pixel font
LCD_fn_8_1_10	equ	&34		; 8 bit data; one line;  10 pixel font
LCD_fn_8_2_7	equ	&38		; 8 bit data; two lines;  7 pixel font
LCD_fn_8_2_10	equ	&3C		; 8 bit data; two lines; 10 pixel font
LCD_CGRAM	equ	&40		; Set CGRAM address
LCD_DDRAM	equ	&80		; Set CGRAM address

; - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
; LCD print routine codes

cEOT		EQU	 4		; Basic ASCII characters
cLF		EQU	10
cFF		EQU	12
cCR		EQU	13

ttr		EQU	0		; String terminator

; - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
	
byte3		EQU	&FF000000	; Byte masks
byte2		EQU	&00FF0000
byte1		EQU	&0000FF00
byte0		EQU	&000000FF

;------------------------------------------------------------------------------
; Specific header for AT91 board
;

FASTRAM_base	EQU	&00000000	; Prescribed - run time address

RAM_base	EQU	&00300000	; At default address
ROM_base	EQU	&10000000	; Fixed in the architecture (CS0)

SPARTAN_base	EQU	&20000000	;  NCS1
VIRTEX_base	EQU	&30000000	;  NCS2 can also be used
; Note: remaining references to "Virtex" apply to second chip select of Spartan-3

		; Flags passed to application
LCD_present_flag	EQU	&00000001	; If LCD detected
Power_up_flag		EQU	&00000100	; If power-up reset
Watchdogged_flag	EQU	&00000200	; If watchdog reset

; - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

Serial_number_addr	EQU	&3FFC	;

boot_table_address	EQU	&4000	; 16Kbytes up to clear bottom block
					; Boot programme MUST be shorter
boot_table_shifts	EQU	8	; Log of boot table length (&100)
boot_table_entry_length	EQU	1 << boot_table_shifts
					; Can't define this the other way :-(

; Boot block offsets (don't really belong here)

Btab_flags		EQU	&04
Btab_RAM_start		EQU	&08
Btab_RAM_length		EQU	&0C
Btab_ROM_start		EQU	&10
Btab_ROM_length		EQU	&14
Btab_exec_offset	EQU	&18
Btab_exec_CPSR		EQU	&1C
Btab_spartan_data	EQU	&20
Btab_spartan_length	EQU	&24
Btab_virtex_data	EQU	&28
Btab_virtex_length	EQU	&2C
Btab_LCD_message	EQU	&30

; Boot flag definitions
BtFlg_LCD_message	EQU	&00000001
BtFlg_LCD_light		EQU	&00000002
BtFlg_LED_on		EQU	&00000004
BtFlg_RAM_boot		EQU	&00000008	; TBC
BtFlg_ROM_check		EQU	&00000010
BtFlg_RAM_clr_int	EQU	&00000100
BtFlg_RAM_clr_ext	EQU	&00000200
			; v. 3 additions
BtFlg_reset_dis		EQU	&00010000
BtFlg_wdog_en		EQU	&00020000
BtFlg_Icache_en		EQU	&00040000
BtFlg_Dcache_en		EQU	&00080000

UserFlg_LCD		EQU	&00000001
UserFlg_Spartan		EQU	&00000010	; TBC (NEW @@@)
UserFlg_PowerUp		EQU	&00000100
UserFlg_Watchdog	EQU	&00000200

; FPGA block offsets (don't really belong here either)

FPGA_offset		EQU	4
FPGA_length		EQU	8


; Default stack sizes (user gets remainder)

Supervisor_stack	EQU	&200
FIQ_stack		EQU	&80
IRQ_stack		EQU	&80
Abort_stack		EQU	&80
Undefined_stack		EQU	&80

;------------------------------------------------------------------------------
; AT91 on-board register definitions

DEV_TERMINATE	EQU	-1		; Not a peripheral address
					; used to terminate set-up tables

;------------------------------------------------------------------------------
; Local PIO (PIO C) bit definitions for AT91 board

DIP_shift	equ	23			; Position of DIP LSB
DIP_switches	equ	&F << DIP_shift		; Start up option switches

LED_bus		equ	&7F800000		; LED data bits
LED_En		equ	&00000040		; 

LCD_bus_shift	equ	23			;
LCD_bus		equ	&FF << LCD_bus_shift	; LCD data bus
LCD_busy	equ	&40000000		; MSB of data bus
LCD_RS		equ	&00010000		;
LCD_En		equ	&00000008		; 
LCD_RW		equ	&00020000		; 
LCD_light	equ	&00000080		; 
LCD_outputs	equ	LCD_RW | LCD_RS | LCD_light | LCD_En

FPGA_PROGB	equ	&00000001		;
FPGA_CCLK	equ	&00000002		;
FPGA_CS_B	equ	&00010000		;
FPGA_RDWR_B	equ	&00020000		;
FPGA_Init_B	equ	&00040000		;
FPGA_Busy	equ	&00080000		;
FPGA_bus_shift	equ	23			;
FPGA_bus	equ	&FF << FPGA_bus_shift	; FPGA programming bus
FPGA_X_0	equ	&00000400		; Uncommitted FPGA pin/baud clock
FPGA_X_1	equ	&00000800		; Uncommitted FPGA pin/FIQ/button
FPGA_outputs	equ	FPGA_CCLK | FPGA_PROGB | FPGA_CS_B | FPGA_RDWR_B

SWITCH_T	equ	&00000800		; Also may act as FIQ
SWITCH_M	equ	&00000010		;
SWITCH_B	equ	&00000020		;

UDP_PU		equ	&00004000		; Redundant! @@@@
UDP_POWER	equ	&00008000		;

TXD0		equ	&00000100		; Peripheral/PIO selects for port C
RXD0		equ	&00000200		; (Not all defined)
TXD1		equ	&00001000		;
RXD1		equ	&00002000		;
PCK1		equ	&80000000		; Here and below for periph. B
PCK2		equ	&00000100		;
PCK3		equ	&00000200		;
SCK0		equ	&00000400		;
FIQ		equ	&00000800		;
TIOA0		equ	&00080000		;
TIOB0		equ	&00100000		;
TIOA1		equ	&00200000		;
TIOB1		equ	&00400000		;
TIOA2		equ	&00800000		;
TIOB2		equ	&01000000		;

;PIO_used	equ	&7FFFCCFF		; PIO C bits in use
PIO_periph_A	equ	TXD0 | RXD0 | TXD1 | RXD1
PIO_periph_B	equ	PCK1
PIO_used	equ	~(PIO_periph_A | PIO_periph_B)	; PIO C bits in use
PIO_output	equ	LCD_outputs | UDP_PU | LED_En | FPGA_outputs	;  default outputs
PIO_output_H	equ	LED_En | UDP_PU | FPGA_PROGB |FPGA_CS_B	; Set high
PIO_output_L	equ	LCD_light | LCD_En | FPGA_CCLK	; Set low
PIO_PU		equ	PIO_used and ~PIO_output	; Pull up all inputs for now	@@@@

AT91_Spartan_IRQ EQU	&00000004		; 
AT91_FIQ	EQU	&00000800		; ??? Open drain (Also HDC signal)

;------------------------------------------------------------------------------
;
;prog_SA0_data	EQU	&0			; XPIO register offsets
;prog_SA0_ctrl	EQU	&1
;prog_SA1_data	EQU	&2
;prog_SA1_ctrl	EQU	&3
;prog_SB0_data	EQU	&4
;prog_SB0_ctrl	EQU	&5
;prog_SB1_data	EQU	&6
;prog_SB1_ctrl	EQU	&7
;prog_VS0_data	EQU	&8
;prog_VS0_ctrl	EQU	&9
;prog_VS1_data	EQU	&A
;prog_VS1_ctrl	EQU	&B

;------------------------------------------------------------------------------
; Codes for number files

Maker_MU	equ	1
Maker_NU	equ	4
Maker_PSU	equ	5

