; AT91SAM9261 Power Management Controller definitions

PMC_BASE	equ	&FFFFFC00		; PMC base address

PMC_SCER	equ	&00			; System clock enable register
PMC_SCDR	equ	&04			; System clock disable register
PMC_SCSR	equ	&08			; System clock status register
PMC_PCER	equ	&10			; Peripheral clock enable register
PMC_PCDR	equ	&14			; Peripheral clock disable register
PMC_PCSR	equ	&18			; Peripheral clock status register
CKGR_MOR	equ	&20			; Main oscillator register
CKGR_MCFR	equ	&24			; Main clock frequency register
CKGR_PLLAR	equ	&28			; PLL A register
CKGR_PLLBR	equ	&2C			; PLL B register
PMC_MCKR	equ	&30			; Master Clock Register

PMC_PCK0	equ	&40			; Programmable clock 0 register
PMC_PCK1	equ	&44			; Programmable clock 1 register
PMC_PCK2	equ	&48			; Programmable clock 2 register
PMC_PCK3	equ	&4C			; Programmable clock 3 register

PMC_IER		equ	&60			; Interrupt enable register
PMC_IDR		equ	&64			; Interrupt disable register
PMC_SR		equ	&68			; Status register
PMC_IMR		equ	&6C			; Interrupt mask register


PMC_EN_HCK1	equ	&0002_0000		; Sytem clock enable bits
PMC_EN_HCK0	equ	&0001_0000		;
PMC_EN_PCK3	equ	&0000_0800		;
PMC_EN_PCK2 	equ	&0000_0400		;
PMC_EN_PCK1	equ	&0000_0200		;
PMC_EN_PCK0	equ	&0000_0100		;
PMC_EN_UDP	equ	&0000_0080		;
PMC_EN_UHP	equ	&0000_0040		;

PLLA_ROOT	equ	&2000_0000		; Bit 29 must be set
PLLB_ROOT	equ	&0000_0000		; USBDIV set to PLLB output
PLL_OUT_SLOW	equ	&0000_0000		; Output  80 - 200 MHz
PLL_OUT_FAST	equ	&0000_8000		; Output 190 - 240 MHz
PLL_LOCK_COUNT	equ	&0000_3F00		; Maximum

PMC_MDIV_1	equ	&000
PMC_MDIV_2	equ	&100
PMC_MDIV_4	equ	&200

PMC_PRES_1	equ	&00
PMC_PRES_2	equ	&04
PMC_PRES_4	equ	&08
PMC_PRES_8	equ	&0C
PMC_PRES_16	equ	&10
PMC_PRES_32	equ	&14
PMC_PRES_64	equ	&18

PMC_CSS_slow	equ	0
PMC_CSS_main	equ	1
PMC_CSS_PLLA	equ	2
PMC_CSS_PLLB	equ	3

PMC_MOSCS	equ	&00000001		; Main OSCillator Stabilized
PLL_LOCKA	equ	&00000002
PLL_LOCKB	equ	&00000004

PMC_OSC_OFF	equ	&00000000		; Oscillators off
PMC_OSC_XTAL	equ	&00000001		; Main oscillator on
PMC_OSC_EXT	equ	&00000002		; Use external oscillator
PMC_OSC_START_SHIFT equ	8

		include	pid.h		; Peripheral identifiers

PMC_PIOA	equ	1 << PID_PIOA	; Peripheral bit fields
PMC_PIOB	equ	1 << PID_PIOB
PMC_PIOC	equ	1 << PID_PIOC
PMC_US0		equ	1 << PID_US0
PMC_US1		equ	1 << PID_US1
PMC_US2		equ	1 << PID_US2
PMC_MCI		equ	1 << PID_MCI
PMC_UDP		equ	1 << PID_UDP
PMC_TWI		equ	1 << PID_TWI
PMC_SPI0	equ	1 << PID_SPI0
PMC_SPI1	equ	1 << PID_SPI1
PMC_SSC0	equ	1 << PID_SSC0
PMC_SSC1	equ	1 << PID_SSC1
PMC_SSC2	equ	1 << PID_SSC2
PMC_TC0		equ	1 << PID_TC0
PMC_TC1		equ	1 << PID_TC1
PMC_TC2		equ	1 << PID_TC2
PMC_UHP		equ	1 << PID_UHP
PMC_LCDC	equ	1 << PID_LCDC
