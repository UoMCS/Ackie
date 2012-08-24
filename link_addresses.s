;------------------------------------------------------------------------------
; Hand-built table for linking programmes in AT91 boot_table
; Last modified 11/3/07

; Items marked with ** are hand-linked and will need checking if code expands


Start			EQU	0

ROM_loader_image_position EQU	&2000	; ** Plenty of space above boot code
ROM_loader_image_length	  EQU	&500	; ** Overestimate
ROM_loader_branch_space	  EQU	&10	; I wish "ORIGIN" worked :-(

Mon_ROM			EQU	&10000	; **
Mon_RAM_image_position	EQU	&1000	; **	INVENTED @@@@
Mon_RAM_image_length	EQU	&280	; **	INVENTED @@@@

ROM_test		EQU	&18000	;

test_config		EQU	&20000	; **

;------------------------------------------------------------------------------
		END