; AT91 boot_table template

		GET	header.s		; Register definitions etc.

		GET	link_addresses.s	; Addresses (etc.) of programmes

;ROM_loader_image_start	EQU	ROM_loader_image_position
;ROM_loader_image_end	EQU	ROM_loader_image_start + ROM_loader_image_length
;ROM_loader_ROM		EQU	ROM_loader_image_position - ROM_loader_branch_space
;Mon_RAM_start		EQU	Mon_ROM + Mon_RAM_image_position
;Mon_RAM_end		EQU	Mon_RAM_start + Mon_RAM_image_length

;Mon_RAM_start2		EQU	Mon_ROM2 + Mon_RAM_image_position

		origin	&4000

boot_table	DCB	"CODE"			; Identifier
		DCD	BtFlg_LCD_message or BtFlg_RAM_boot		; or BtFlg_Icache_en
		DCD	&1000, &454		; RAM start and length
		DCD	&1000, &454		; ROM start and length
		DCD	&00000000, &000000D3	; Start offset and CPSR
		DCD	0			; Spartan config. ROM offset
		DCD	0			; Spartan length
		DCD	0, 0			; Virtex config. & length
		DCB	cFF,"Flash loader", ttr
		DCB	"Flash loader", 0

	ALIGN	boot_table_entry_length

		DCB	"CODE"			; Identifier
		DCD	BtFlg_LCD_message or BtFlg_LED_on ;	or BtFlg_Icache_en
		DCD	0			; RAM start
		DCD	0			;  and length
		DCD	&0C000			; ROM start
		DCD	&2400			;  and length
		DCD	0			; Start offset
		DCD	&000000D3		;  and CPSR
		DCD	&10000			; Spartan config. ROM offset
		DCD	&20000			; Spartan length
		DCD	0, 0			; Virtex config. & length
		DCB	cFF, ttr
		DCB	"Komodo back-end", 0

	ALIGN	boot_table_entry_length

		DCB	"CODE"			; Identifier
		DCD	BtFlg_LCD_message or BtFlg_LED_on or BtFlg_Icache_en
		DCD	0			; RAM start
		DCD	0			;  and length
		DCD	&30000			; ROM start
		DCD	&10000			;  and length
		DCD	0			; Start offset
		DCD	&000000D3		;  and CPSR
		DCD	0, 0			; Spartan config. & length
		DCD	0, 0			; Virtex config. & length
		DCB	cFF, ttr		; No Message
		DCB	"Ackie: back-end ARM host program", 0

;	ALIGN	boot_table_entry_length
;
;		DCB	"CODE"			; Identifier
;		DCD	BtFlg_LCD_message or BtFlg_LED_on or BtFlg_Icache_en
;		DCD	Mon_RAM_start - Start	; RAM start
;		DCD	Mon_RAM_end - Mon_RAM_start ; and length
;		DCD	Mon_ROM, &2400		; ROM start and length
;		DCD	&00000000, &000000D3	; Start offset and CPSR
;		DCD	test_config - Start	; Spartan config. ROM offset
;		DCD	&4000			; Spartan length
;;	DCD	0,0
;		DCD	0, 0			; Virtex config. & length
;		DCB	cFF,"sptn ldr", ttr	; Show sptn ldr
;		DCB	"Spartan3 loader "	; User's text
;		DCB	"Version 0.1 "
;		DCB	0
;
;	ALIGN	boot_table_entry_length
;
;		DCB	"CODE"			; Identifier
;		DCD	BtFlg_LCD_message or BtFlg_LCD_light or BtFlg_LED_on
;		DCD	Mon_RAM_start - Start	; RAM start and length
;		DCD	0			
;		DCD	ROM_test, &64   	; ROM start and length
;		DCD	&00000000, &000000D3	; Start offset and CPSR
;		DCD	0,0			; Spartan config. ROM offset, length
;		DCD	0, 0			; Virtex config. & length
;		DCB	cFF,"ROM Tester ", ttr	; LCD message
;		DCB	"ROM tester (prototype)", 0
;
;	ALIGN	boot_table_entry_length
;
;		DCB	"CODE"			; Identifier
;		DCD	BtFlg_LCD_message or BtFlg_LCD_light or BtFlg_LED_on or BtFlg_RAM_boot
;		DCD	&2000			; RAM start
;		DCD	&100			;  and length
;		DCD	&2000			; ROM start
;		DCD	&100			;  and length
;		DCD	0			; Start offset
;		DCD	&000000D3		;  and CPSR
;		DCD	0, 0			; Spartan config. & length
;		DCD	0, 0			; Virtex config. & length
;		DCB	cFF,"From RAM ...", ttr
;		DCB	"Speed tester - from RAM", 0
;
;	ALIGN	boot_table_entry_length
;
;		DCB	"CODE"			; Identifier
;		DCD	BtFlg_LCD_message
;		DCD	0			; RAM start
;		DCD	0			; and length
;		DCD	&50000, &10000		; ROM start and length
;		DCD	&00000000, &000000D3	; Start offset and CPSR
;		DCD	0			; Spartan config. ROM offset
;		DCD	0			; Spartan length
;		DCD	0, 0			; Virtex config. & length
;		DCB	cFF, "Hello Michael", cLF, "and Lee", 0
;		DCB	"PIC/ARM test boot "	; User's text
;		DCB	"July 2007"
;		DCB	0
;
;	ALIGN	boot_table_entry_length
;
;		DCB	"CODE"			; Identifier
;		DCD	BtFlg_LCD_message
;		DCD	0			; RAM start
;		DCD	0			; and length
;		DCD	&30000, &10000		; ROM start and length
;		DCD	&00000000, &000000D3	; Start offset and CPSR
;		DCD	0			; Spartan config. ROM offset
;		DCD	0			; Spartan length
;		DCD	0, 0			; Virtex config. & length
;		DCB	cFF, "MMU experiments", 0
;		DCB	"Gash code to probe at "; User's text
;		DCB	"MMU/cache operation "	
;		DCB	"August 2007"
;		DCB	0
;
;;	ALIGN	boot_table_entry_length
;
;;		DCB	"CODE"			; Identifier
;;		DCD	BtFlg_LCD_message or BtFlg_LED_on
;;		DCD	Mon_RAM_start2 - Start	; RAM start
;;		DCD	Mon_RAM_end - Mon_RAM_start ; and length
;;		DCD	Mon_ROM2, &2400		; ROM start and length
;;		DCD	&00000000, &000000D3	; Start offset and CPSR
;;		DCD	XPIO_config - Start	; Spartan config. ROM offset
;;		DCD	&4000			; Spartan length
;;		DCD	0, 0			; Virtex config. & length
;;		DCB	cFF, 0			; Just clear screen
;;		DCB	"Komodo back end "	; User's text
;;		DCB	"Version 3 "
;;		DCB	0
;;
;;	ALIGN	boot_table_entry_length
;
;
;;------------------------------------------------------------------------------
;
