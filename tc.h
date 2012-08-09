; AT91SAM9261 TC definitions

TC_BASE		equ	&FFFA0000

TC_CHL0		equ	&00		; Base offset for TC0
TC_CHL1		equ	&40		; Base offset for TC1
TC_CHL2		equ	&80		; Base offset for TC2
TC_BCR		equ	&C0		; Block control register
TC_BMR		equ	&C4		; Block mode register

					; Register offsets within TC
TC_CCR		equ	&00		; Channel control register
TC_CMR		equ	&04		; Channel mode register
TC_CV		equ	&10		; Counter value
TC_RA		equ	&14		; Register A
TC_RB		equ	&18		; Register B
TC_RC		equ	&1C		; Register C
TC_SR		equ	&20		; Status register
TC_IER		equ	&24		; Interrupt enable register
TC_IDR		equ	&28		; Interrupt disable register
TC_IMR		equ	&2C		; Interrupt mask register

TC_COVFS	equ	&01		; Interrupt sources
TC_LOVRS	equ	&02
TC_CPAS 	equ	&04
TC_CPBS 	equ	&08
TC_CPCS 	equ	&10
TC_LDRAS	equ	&20
TC_LDRBS	equ	&40
TC_ETRGS	equ	&80
