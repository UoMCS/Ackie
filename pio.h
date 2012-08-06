; AT91SAM9261 PIO definitions

PIOA		equ	&FFFFF400		; PIO A base address
PIOB		equ	&FFFFF600		; PIO B base address
PIOC		equ	&FFFFF800		; PIO C base address

PIO_PER		equ	&00			; PIO Enable register
PIO_PDR		equ	&04			; PIO Disable register
PIO_PSR		equ	&08			; PIO Status register
PIO_OER		equ	&10			; Output enable register
PIO_ODR		equ	&14			; Output disable register
PIO_OSR		equ	&18			; Output status register
PIO_IFER	equ	&20			; Glitch input filter enable register
PIO_IFDR	equ	&24			; Glitch input filter disable register
PIO_IFSR	equ	&28			; Glitch input filter status register
PIO_SODR	equ	&30			; Set output data register
PIO_CODR	equ	&34			; Clear output data register
PIO_ODSR	equ	&38			; Output data status register
PIO_PDSR	equ	&3C			; Pin data status register
PIO_IER		equ	&40			; Interrupt enable register
PIO_IDR		equ	&44			; Interrupt disable register
PIO_IMR		equ	&48			; Interrupt mask register
PIO_ISR		equ	&4C			; Interrupt status register
PIO_MDER	equ	&50			; Multi-driver enable register
PIO_MDDR	equ	&54			; Multi-driver disable register
PIO_MDSR	equ	&58			; Multi-driver status register
PIO_PUDR	equ	&60			; Pull-up disable register
PIO_PUER	equ	&64			; Pull-up enable register
PIO_PUSR	equ	&68			; Pad pull-up status register
PIO_ASR		equ	&70			; Peripheral A select register
PIO_BSR		equ	&74			; Peripheral B select register
PIO_ABSR	equ	&78			; AB status register
PIO_OWER	equ	&A0			; Output write enable
PIO_OWDR	equ	&A4			; Output write disable
PIO_OWSR	equ	&A8			; Output write status register




