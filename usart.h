; AT91SAM9261 USART definitions

USART0_BASE	equ	&FFFB0000	; USART 0 base address
USART1_BASE	equ	&FFFB4000	; USART 1 base address
USART2_BASE	equ	&FFFB8000	; USART 2 base address

US_CR		equ	&00		; Control Register
US_MR		equ	&04		; Mode Register
US_IER		equ	&08		; Interrupt Enable Register
US_IDR		equ     &0C		; Interrupt Disable Register
US_IMR		equ	&10		; Interrupt Mask Register
US_CSR		equ	&14		; Channel Status Register
US_RHR		equ	&18		; Receiver Holding Register
US_THR		equ	&1C		; Transmitter Holding Register
US_BRGR		equ	&20		; Baud Rate Generator Register
US_RTOR		equ	&24		; Receiver Time-out Register
US_TTGR		equ	&28		; Transmitter Timeguard Register
US_FIDI		equ	&40		; FI DI Ratio Register
US_NER		equ	&44		; Number of Errors Register
US_IF		equ	&4C		; IrDA Filter Register

; &100 - &128 is reserved for PDC registers

		; US_CR bits
US_RSTRX	equ	&00000004	; Reset receiver
US_RSTTX	equ	&00000008	; Reset transmitter
US_RXEN		equ	&00000010	; Enable receiver
US_RXDIS	equ	&00000020	; Disable receiver
US_TXEN		equ	&00000040	; Enable transmitter
US_TXDIS	equ	&00000080	; Disable transmitter
US_RSTSTA	equ	&00000100	; Reset status bits
US_STTBRK	equ	&00000200	; Start break
US_STPBRK	equ	&00000400	; Stop break
US_STTTO	equ	&00000800	; Start timeout
US_SENDA	equ	&00001000	; Send address (multidrop)
US_RSTIT	equ	&00002000	; Reset iterations
US_RSTNACK	equ	&00004000	; Reset non acknowledge
US_RETTO	equ	&00008000	; Rearm timeout
US_RTSEN	equ	&00020000	; Enable RTS
US_RTSDIS	equ	&00040000	; Disable RTS

		; US_MR bits
US_MODE_normal	equ	&00000000	; Normal
US_MODE_RS485	equ	&00000001	; RS485
US_MODE_HW_hnd	equ	&00000002	; Hardware handshaking
US_MODE_T0	equ	&00000004	; ISO7816: T=0
US_MODE_T1	equ	&00000006	; ISO7816: T=1
US_MODE_IrDA	equ	&00000008	; IrDA
US_CLKS_MCK	equ	&00000000	; Use MCK
US_CLKS_MCK_DIV	equ	&00000010	; Use MCK/8
US_CLKS_SCK	equ	&00000030	; Use SCK
US_CHRL_5	equ	&00000000	; Character length
US_CHRL_6	equ	&00000040	; ...
US_CHRL_7	equ	&00000080	; 
US_CHRL_8	equ	&000000C0	; 
US_CHRL_9	equ	&00020000	; 
US_ASYNC	equ	&00000000	; Async. mode
US_SYNC		equ	&00000100	; Sync. mode
US_PAR_even	equ	&00000000	; Even parity
US_PAR_odd	equ	&00000200	; Odd parity
US_PAR_0	equ	&00000400	; Zero parity
US_PAR_1	equ	&00000600	; One parity
US_PAR_none	equ	&00000800	; No parity
US_PAR_multi	equ	&00000C00	; Multidrop mode
US_NBSTOP_1	equ	&00000000	; 1 stop bit
US_NBSTOP_1_5	equ	&00001000	; 1.5 stop bits
US_NBSTOP_2	equ	&00002000	; 2 stop bits
US_CHMODE_normal equ	&00000000	; No loopback
US_CHMODE_echo	equ	&00004000	; Pin to receiver
US_CHMODE_local	equ	&00008000	; Loopback at unit
US_CHMODE_remote equ	&0000C000	; Loopback at pin
US_MSBF_LE	equ	&00000000	; Little endian
US_MSBF_BE	equ	&00010000	; Big endian
US_OVER_8	equ	&00080000	; Oversample 8 times
US_OVER_16	equ	&00000000	; Oversample 16 times
US_INACK	equ	&00100000	; Inhibit non acknowledge
US_DSNACK	equ	&00200000	; Disable successive NACK
US_MAX_ITERATION_0 equ	&00000000	; Max. iterations
US_MAX_ITERATION_1 equ	&01000000	;  for ISO7816: T=0
US_MAX_ITERATION_2 equ	&02000000	; 
US_MAX_ITERATION_3 equ	&03000000	; 
US_MAX_ITERATION_4 equ	&04000000	; 
US_MAX_ITERATION_5 equ	&05000000	; 
US_MAX_ITERATION_6 equ	&06000000	; 
US_MAX_ITERATION_7 equ	&07000000	; 
US_FILTER	equ	&10000000	; IrDA filter

		; US_... status bits
US_RXRDY	equ	&00000001	; RXRDY
US_TXRDY	equ	&00000002	; TXRDY
US_RXBRK	equ	&00000004	; Receiver Break
US_ENDRX	equ	&00000008	; End of Receive Transfer
US_ENDTX	equ	&00000010	; End of Transmit
US_OVRE		equ	&00000020	; Overrun Error
US_FRAME	equ	&00000040	; Framing Error
US_PARE		equ	&00000080	; Parity Error
US_TIMEOUT	equ	&00000100	; Time-out
US_TXEMPTY	equ	&00000200	; TXEMPTY
US_ITERATION	equ	&00000400	; Iteration
US_TXBUFE	equ	&00000800	; Buffer Empty
US_RXBUFF	equ	&00001000	; Buffer Full
US_NACK		equ	&00002000	; Non Acknowledge
US_CTSIC	equ	&00080000	; Clear to Send Input Change
US_CTS		equ	&00800000	; Clear to Send
