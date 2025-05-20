;
; m328-nano-utile.asm
;
;  Created: 1/31/2016 7:30:50 AM
;  Author: lynf
;
;    uTile - A Programmable Logic Controller for the ATmega328P
;    Copyright (C) 2021  Francis Lyn
;
; The m328-uTile program is not intended for use in commercial, industrial,
; medical or safety-related applications.
;  
;  MIT License
;  
;  Permission is hereby granted, free of charge, to any person obtaining a copy
;  of this software and associated documentation files (the "Software"), to deal
;  in the Software without restriction, including without limitation the rights
;  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
;  copies of the Software, and to permit persons to whom the Software is
;  furnished to do so, subject to the following conditions:
;  
;  The above copyright notice and this permission notice shall be included in all
;  copies or substantial portions of the Software.
;  
;  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
;  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
;  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
;  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
;  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
;  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
;  SOFTWARE.
;  
;
;   email:  lynf.yyz@gmail.com
;
; Hardware platform:
; ==================
; Arduino Nano controller board with
; ATmega328P cpu, 16.0000 MHz external crystal
;
; Change log:
; ===========
; Record of most of the changes made over time up to and including ver. 1.0
; are listed below mainly to track internal code changes and development.
;
; 1. Change CLKA from 100 ms to 200 ms
;	Timer Tf changed to 6.66 min ticks
;
; 2. Change Tf from 6.66 min to 10 min tick
; 3. Remove T0...T3 from Timer1_COMPA
; 4. Add T8, T9 to Timer1_COMP
;
; ===========
; 5. Changed PY from PB to PD pins, PB reserved for future nRF24L01+ interface
; 6. Changed ^Z to Z. to clear timers in LDT command
; 7. Revised CLKx timers for slower rates.
; 8. Added input for selecting out_sense to invert all outputs if jumper present
;	 PB1/D9 input.
; 9. Added outputs Y6, Y7 (PB4/D12/MISO, PB5D/13/SCK).	(OK)
; 10. Added 8 new timers, T8...TF for a total of 16 timers. Reconfigured timers
;	 so TM0..7, 8..B are seconds timers, TMC..TMF are minutes timers.
; 11. Added new virtual byte PV as RAM buffer. Change PU from register to RAM buffer
;	 Change all variables PA, PY, PU and PV from register to ram storage.
;	 Modified pbyt and gbyt macros to RAM instead of register data transfer.
; 12. Add READ and WRITE commands to support file storage and retrieval from
;	 host PC of UF programs.	(OK)
; 13. Modified <sdfil2> to perform 16 bit decrement and test for zero
; 14. Change <zbuf> to clear proper area, was starting at linbuf.
; 14. Modify <autost:> to test if EEPROM 1st byte is erased.
; 15. Modify <ildp:/stp:> to change load/store buffer range.
;
; 2017-May-4
;
; 16. Add new alias keys INS and DEL for editor commands INS and DEL
; 17. Revise LDT command to fix extra characters in unused cursor, INS and DEL keys.
; 18. Fix 'andi	rga,0b00000011' mask error in <OUTY:>
;
;
; 2017-Aug-24
;
; 19. Timers TT8,9,a b changed from minutes to 102.4 ms timebase
;	 Define PRVms equate. Move swd macros from Timer1 to Timer0 handler
;
; 2018-Feb-09
;
; 20. <InitTimer1:> change t1800msf to t600msf in timrb routine
; 21. Change CLKA from 100 ms to 200 ms
; 22. <optn4:>
;
; 2018-June-14
;
; 23. Re-arranged timers T0..T3 are 100 ms timers, T4..Tb are sec, Tc...Tf min.
; 24. Revise TKTR timer trigger macros to use correct PRVx parameters
;
; 2018-Sept-22
;
; 25.	Revise Tf to count 6.666 min ticks, 3 counts = 20 min.
;		24 h = 24x60 = 1440 min, or 1440x3/20 = 216 counts
; 26.	Revise TKTR timer trigger macros to use PRVh prescaler parameter
;		for 6.660 min ticks. 24 h is about 216 counts. SWD macro uses t1800msf
;		for timing ticks.
; 27.	Modify <tcbfx:> to clear t1800msf
;
; 2019-May-31
; 28.	Modify <optn4a:> for Home string test, change ESC OH to ESC H.
;
; 2021-May-5
; 29.	Change Tf to 10 min tick. See 11). 24 h = 140 counts. Add tbased,
;       t4sf to <Timer1_COMPA> for 4 s tick timer.
;
; 2021/09/28
; 30. <ex0:> call <zbuf:> to clear MCU RAM except UFn
;    and TM reload buffers.
;
; 2021/10/07
; 31. ".CNT" gated counter, count 1 s ticks while input gate = 1
;   Ticks generated in Timer1_COMPA. At end of gate period
;	When gate goes to 0, wdbuf contains the 1 s ticks accumulated
;	during the period while the gate is high and drdyf flag is set.
;	The wdbuf Data display shows the number of 1 s ticks counted
;	while the gate is high. See <cnt:> and <tbase_1s:>.
;
; 32. 16 words, "LDTm", reload TRBm buffer if drdy flag = 1, then
;	clears drdy = 0. The 1 s tick count measured by "".CNT" is written
;	to the TRBm RAM buffer and then writted to the TRBm image address 
;	in EEPROM. See macro <ldtm>
;
;   Words:      .CNT, LDTm
;   Buffers:    CNTb, TM1sb
;   Flag:       t1sf, drdyf
;
; 33. "SWAP" (b2 b1 -- b1 b2) swaps contents of SKB0 and SKB1
; 34. "SET" revised to "1.", push a 1 to BSTK. Removed.
; 35. "CLR" revised to "0.", push a 0 to BSTK. Removed.
; 36. "2DUP" ( b1 b2 -- b1 b2 b1 b2 )
; 37. ".TGn" toggle TGQn on rising edge of TGn
;	  "TGQn." Get toggle output
; 38. Add paceb control to <ibs:>
;
;
; ====================================
;
; Fuse settings: 16 MHz external clock
; LFUSE:	0XFF
; HFUSE:	0XDA
; EFUSE:	0XFD
;
; ====================================
;
.list		; Listing on

.equ F_CPU = 16000000
;
; UART definitions
;
.equ	BAUD = 19200		; Baud rate
.equ	BAUD_PRE = 51		; Baud rate prescaler, 19200 baud
.equ	NULL = 0x0			; Null terminator
.equ	BELL = 0x07			; Bell
.equ	BS = 0x08			; Backspace
.equ	HT = 0x09			; Tab
.equ	LF = 0x0a			; Linefeed
.equ	CR = 0x0d			; Carriage return
.equ	ctlW = 0x17			; Control W
.equ	ctlX = 0x18			; Control X
.equ	ctlZ = 0x1a			; Control Z
.equ	SP = 0x20			; Space
.equ	ESC = 0x1b			; Escape
.equ	DEL = 0x7f			; Delete
.equ	CMA	= 0x2c			; Comma
;
.equ	ctlA = 0x01			; Control A, SOH
.equ	ctlS = 0x13			; Control S, DC3
;
; Output sense selection input pin
;
.equ	out_sense = PB1		; Jumper input (D9) to invert outputs
;
; Timer0, Timer1 parameters
;
.equ	OCR0Aload = 64		; OCR0A 8 bit register, 64 x 64 us = 4.096 ms
.equ	OCR1Aload = 6250	; OCR1A 16 bit register, 6250 x 16 us = 100 ms
;
;
; --- Register definitions ---
;
; Low registers
;
.def	count = R2			; Counter for line buffer
.def	asav = R3			; rga save register
.def	ufpch = R4			; UF program counter high
.def	ufpcl = R5			; UF program counter low
.def	BSTK = R6			; Boolean bit stack
.def	SRsav = R7			; SREG save
;
.def	timra = R8			; Software timera
.def	timrb = R9			; Software timerb
.def	timrc = R10			; Software timerc
.def	timrd = R11			; Software timerd
;
.def	res0 = R13			; result register 0
.def	res1 = R14			; result register 1
.def	res2 = R15			; result register 2
;
; High registers
;
.def	rmp = R16			; Multipurpose register
.def	rga = R17			; GP register RGA
.def	rgb = R18			; GP register RGB
.def	rgc = R19			; GP register RGC
.def	rgd = R20			; GP register RGD
.def	rge	= R21			; GP register RGE
.def	rgv	= R22			; Variable register
.def	flaga = R23			; Flag A register, 8 flags
.def	flagb = R24			; Flag B register, 8 flags
.def	flagc = R25			; Flag C register, 8 flags
;
;
; Flag register flaga
;
.equ	t100msf = 0			; 100 ms tick flag
.equ	t600msf = 1			; 600 ms tick flag
.equ	t1800msf = 2		; 1800 ms tick flag
.equ	numfl = 3			; Valid byte number flag
.equ	crf = 4				; Carriage return key flag
.equ	escf = 5			; Escape key flag
.equ	kyf	= 6				; Control key flag
.equ	xclinf = 7			; Delayed line clear flag
;
; Flag register flagb
;
.equ	clkaf = 0			; CLKA tick flagb
.equ	clkbf = 1			; CLKB tick flagb
.equ	clkcf = 2			; CLKc tick flagb
.equ	paceb = 3			; Pace flag for printing ports
.equ	t4msf = 4			; 4 ms timer flag
.equ	t4sf = 5			; 4 s timer flag
.equ	t1sf = 6			; 1 s tick flag
.equ    drdyf = 7           ; CNTb data ready flag
;
;
; Flag register flagc
;
;.equ	ufsel = 0			; User file space select flag
;
;
; --- Constants ---
;
.equ	timrav = 2			; timrb reload for 200 ms
.equ	timrbv = 6			; timrb reload for 600 ms
.equ	timrcv = 18			; timrc reload for 1800 ms
.equ	timrdv = 40			; timrc reload for 4 s
.equ	PRVms = 25			; 25 x 4.096 = 102.4 ms prescaler
.equ	PRVs = 10			; 10 x 0.1 s = 1.0 s prescaler
.equ	PRVm = 100			; 100 x 0.6s = 60 s prescaler
.equ	PRVh = 150			; 150 x 4s = 600 s prescaler
;
.equ	linsz = 8			; Line buffer size
.equ	opclen = 6			; Maximum opcode length + 1
.equ	nlines = 32			; Command lines per screen
.equ	lastpg = 224		; Basln number on last page
.equ	dlhzr = 19			; 19 spaces between columns
.equ	vtri = 9			; y = 0 position
.equ	hzri = 8			;  x = 0 position
;
; One-shot timer loader display
;
.equ	xinit = 15			; LDT initial x position
.equ	yinit = 11			; LDT initial y position
.equ	ntim = 16			; Number of timers
.equ	dx1 = 7				; Basic buffer screen spacing
.equ	dx2 = 4				; Extra spacing
.equ	dy = 5				; Row spacing
;
;
.equ	maxbits = 0xff		; High byte mask
.equ	nbits = 16			; Convert 16 bits
.equ	ndig = 3			; Digit pair bytes
.equ	ndec = 5			; Digits to display/convert
;
; Command word attributes
;
.equ	NORM = 0x0			; Default, interpreter and runtime command
.equ	EDIT = 0x0001		; Editor mode only
.equ	FALSE = 0x0			; Logical 0
.equ	TRUE = !FALSE		; Logical 1
;
.equ	uf0en = 0x60+0x1ff	; End of UF0 space - 2
;
; --- Addressable Bits for Input, Output and Variable storage ---
;
; Bit Stack, BSTK in register
;
.equ	SKB7 = 7
.equ	SKB6 = 6
.equ	SKB5 = 5
.equ	SKB4 = 4
.equ	SKB3 = 3
.equ	SKB2 = 2
.equ	SKB1 = 1
.equ	SKB0 = 0
;
; Input Port A bits, PA in register
;
.equ	A7 = 7
.equ	A6 = 6
.equ	A5 = 5
.equ	A4 = 4
.equ	A3 = 3
.equ	A2 = 2
.equ	A1 = 1
.equ	A0 = 0
;
; Port A leading edge bits, PAL in data space
;
.equ	AL7 = 7
.equ	AL6 = 6
.equ	AL5 = 5
.equ	AL4 = 4
.equ	AL3 = 3
.equ	AL2 = 2
.equ	AL1 = 1
.equ	AL0 = 0
;
; Port A trailing edge bits, PAT in data space
;
.equ	AT7 = 7
.equ	AT6 = 6
.equ	AT5 = 5
.equ	AT4 = 4
.equ	AT3 = 3
.equ	AT2 = 2
.equ	AT1 = 1
.equ	AT0 = 0
;
; Port A double action bits, PAD in data space
;
.equ	AD7 = 7
.equ	AD6 = 6
.equ	AD5 = 5
.equ	AD4 = 4
.equ	AD3 = 3
.equ	AD2 = 2
.equ	AD1 = 1
.equ	AD0 = 0
;
; Virtual bits, PU in register
;
.equ	U7 = 7			; Temporary storage bits
.equ	U6 = 6
.equ	U5 = 5
.equ	U4 = 4
.equ	U3 = 3
.equ	U2 = 2
.equ	U1 = 1
.equ	U0 = 0
;
; Virtual bits, PV in register
;
.equ	V7 = 7			; Temporary storage bits
.equ	V6 = 6
.equ	V5 = 5
.equ	V4 = 4
.equ	V3 = 3
.equ	V2 = 2
.equ	V1 = 1
.equ	V0 = 0
;
; Output Port Y bits, PY in register
;
.equ	Y7 = 7
.equ	Y6 = 6
.equ	Y5 = 5
.equ	Y4 = 4
.equ	Y3 = 3
.equ	Y2 = 2
.equ	Y1 = 1
.equ	Y0 = 0
;
; --- Flip/Flop control bits ---
;
.equ	QF7 = 7				; RS F/F output bits
.equ	QF6 = 6
.equ	QF5 = 5
.equ	QF4 = 4
.equ	QF3 = 3
.equ	QF2 = 2
.equ	QF1 = 1
.equ	QF0 = 0
;
.equ	QFF = 7				; RS F/F output bits
.equ	QFE = 6
.equ	QFD = 5
.equ	QFC = 4
.equ	QFB = 3
.equ	QFA = 2
.equ	QF9 = 1
.equ	QF8 = 0
;
.equ	RF7 = 7				; RS F/F reset bits
.equ	RF6 = 6
.equ	RF5 = 5
.equ	RF4 = 4
.equ	RF3 = 3
.equ	RF2 = 2
.equ	RF1 = 1
.equ	RF0 = 0
;
.equ	RFF = 7				; RS F/F reset bits
.equ	RFE = 6
.equ	RFD = 5
.equ	RFC = 4
.equ	RFB = 3
.equ	RFA = 2
.equ	RF9 = 1
.equ	RF8 = 0
;
.equ	SF7 = 7				; RS F/F set bits
.equ	SF6 = 6
.equ	SF5 = 5
.equ	SF4 = 4
.equ	SF3 = 3
.equ	SF2 = 2
.equ	SF1 = 1
.equ	SF0 = 0
;
.equ	SFF = 7				; RS F/F set bits
.equ	SFE = 6
.equ	SFD = 5
.equ	SFC = 4
.equ	SFB = 3
.equ	SFA = 2
.equ	SF9 = 1
.equ	SF8 = 0
;
; --- Software one-shot timer control flags ---
;
.equ	TQ0 = 0				; Timer_0 output flag
.equ	TR0 = 1				; Timer_0 reset flag
.equ	TM0 = 2				; Timer_0 enable flag
.equ	TK0 = 3				; Timer_0 trigger flag
;
.equ	TQ1 = 4				; Timer_1 output flag
.equ	TR1 = 5				; Timer_1 reset flag
.equ	TM1 = 6				; Timer_1 enable flag
.equ	TK1 = 7				; Timer_1 trigger flag
;
.equ	TQ2 = 0				; Timer_2 output flag
.equ	TR2 = 1				; Timer_2 reset flag
.equ	TM2 = 2				; Timer_2 enable flag
.equ	TK2 = 3				; Timer_2 trigger flag
;
.equ	TQ3 = 4				; Timer_3 output flag
.equ	TR3 = 5				; Timer_3 reset flag
.equ	TM3 = 6				; Timer_3 enable flag
.equ	TK3 = 7				; Timer_3 trigger flag
;
.equ	TQ4 = 0				; Timer_4 output flag
.equ	TR4 = 1				; Timer_4 reset flag
.equ	TM4 = 2				; Timer_4 enable flag
.equ	TK4 = 3				; Timer_4 trigger flag
;
.equ	TQ5 = 4				; Timer_5 output flag
.equ	TR5 = 5				; Timer_5 reset flag
.equ	TM5 = 6				; Timer_5 enable flag
.equ	TK5 = 7				; Timer_5 trigger flag
;
.equ	TQ6 = 0				; Timer_6 output flagF
.equ	TR6 = 1				; Timer_6 reset flag
.equ	TM6 = 2				; Timer_6 enable flag
.equ	TK6 = 3				; Timer_6 trigger flag
;
.equ	TQ7 = 4				; Timer_7 output flag
.equ	TR7 = 5				; Timer_7 reset flag
.equ	TM7 = 6				; Timer_7 enable flag
.equ	TK7 = 7				; Timer_7 trigger flag
;
.equ	TQ8 = 0				; Timer_8 output flag
.equ	TR8 = 1				; Timer_8 reset flag
.equ	TM8 = 2				; Timer_8 enable flag
.equ	TK8 = 3				; Timer_8 trigger flag
;
.equ	TQ9 = 4				; Timer_9 output flag
.equ	TR9 = 5				; Timer_9 reset flag
.equ	TM9 = 6				; Timer_9 enable flag
.equ	TK9 = 7				; Timer_9 trigger flag
;
.equ	TQA = 0				; Timer_A output flag
.equ	TRA = 1				; Timer_A reset flag
.equ	TMA = 2				; Timer_A enable flag
.equ	TKA = 3				; Timer_A trigger flag
;
.equ	TQB = 4				; Timer_B output flag
.equ	TRB = 5				; Timer_B reset flag
.equ	TMB = 6				; Timer_B enable flag
.equ	TKB = 7				; Timer_B trigger flag
;
.equ	TQC = 0				; Timer_C output flag
.equ	TRC = 1				; Timer_C reset flag
.equ	TMC = 2				; Timer_C enable flag
.equ	TKC = 3				; Timer_C trigger flag
;
.equ	TQD = 4				; Timer_D output flag
.equ	TRD = 5				; Timer_D reset flag
.equ	TMD = 6				; Timer_D enable flag
.equ	TKD = 7				; Timer_D trigger flag
;
.equ	TQE = 0				; Timer_E output flagF
.equ	TRE = 1				; Timer_E reset flag
.equ	TME = 2				; Timer_E enable flag
.equ	TKE = 3				; Timer_E trigger flag
;
.equ	TQF = 4				; Timer_F output flag
.equ	TRF = 5				; Timer_F reset flag
.equ	TMF = 6				; Timer_F enable flag
.equ	TKF = 7				; Timer_F trigger flag
;

;
; --- Macro definitions ---
;
.macro	ldzptr				; Load ZH:ZL pointer with address*2
		ldi		ZH,high(@0*2)
		ldi		ZL,low(@0*2)
.endm
;
.macro	ldxptr				; Load XH:XL pointer with address to access data memory
		ldi		XH,high(@0)
		ldi		XL,low(@0)
.endm
;
.macro	ldyptr					; Load YH:YL pointer with address to access data memory
		ldi		YH,high(@0)
		ldi		YL,low(@0)
.endm
;
; Exchange contents of registers
;
.macro	xchreg					; Exchange registers
		push	@0
		push	@1
		pop		@0
		pop		@1
.endm
;
; --- Tile macros ---
;
; Macros with '2' suffix are for namestrings with an even number of characters
; e.g. "AD0.". These macros add a null padding byte after the namestring, so that
; the complete string including the ^W terminator ends on a word boundary.
;
.LISTMAC
;
; Get direct bit (from register) and push to bit stack. C destroyed.
; Follow the form of macro exactly as shown. The offset to start of namestring is
; in 2's complement format. Adding this offset to PC at offset word's .dw location yields
; the address of the start of the namestring.
;
.macro	gbit					; @0 = bit, @1 = byte, @2 = namestring		(OK)
gbits:
		.db		@2,ctlW			; Namestring,ctlW
		.dw		gbite			; Link address to next word
		.dw		NORM			; Interpreter and runtime
gbitL:
		.dw		(gbits-gbitL)	; Offset to start of namestring
;
		bst		@1,@0			; T <-- Rd(b)
		sec						; Assume C = 1
		brts	gbit1			; Skip next if T = 1
		clc						; Clear C
gbit1:
		rol		BSTK			; Push C to TOS. BSTK bit 7 lost.
		ret
gbite:
.endm
;
; Get virtual bit (from data space byte) and push to bit stack. C destroyed.
; Follow the form of macro exactly as shown. The offset to start of namestring is
; in 2's complement format. Adding this offset to PC at offset word's .dw location yields
; the address of the start of the namestring.
;
.macro	gvbit2					; @0 = bit, @1 = data space byte, @2 = namestring		(OK)
gvbit2s:
		.db		@2,0,ctlW		; Namestring,ctlW
		.dw		gvbit2e			; Link address to next word
		.dw		NORM			; Interpreter and runtime
gvbit2L:
		.dw		(gvbit2s-gvbit2L)	; Offset to start of namestring
;
		lds		rmp,@1			; Get buffer byte
		bst		rmp,@0			; T <-- Rd(b)
		sec						; Assume C = 1
		brts	gvbit21			; Skip next if T = 1
		clc						; Clear C
gvbit21:
		rol		BSTK			; Push C to TOS. BSTK bit 7 lost.
		ret
gvbit2e:
.endm
;
.macro	gvbit					; @0 = bit, @1 = data space byte, @2 = namestring		(OK)
gvbits:
		.db		@2,ctlW		; Namestring,ctlW
		.dw		gvbite			; Link address to next word
		.dw		NORM			; Interpreter and runtime
gvbitL:
		.dw		(gvbits-gvbitL)	; Offset to start of namestring
;
		lds		rmp,@1			; Get buffer byte
		bst		rmp,@0			; T <-- Rd(b)
		sec						; Assume C = 1
		brts	gvbit1			; Skip next if T = 1
		clc						; Clear C
gvbit1:
		rol		BSTK			; Push C to TOS. BSTK bit 7 lost.
		ret
gvbite:
.endm
;
;
; Get an input transition bit (from data space byte) and push to the bit stack
; then clear the bit. C destroyed. Disable TCNT0 interrupt during the routine
; to prevent missing a bit transition.
; Consider the following. @0 (transition bit) is 0 when the gbitx2 routine is entered.
; A Timer0 interrupt occurs after the @0 bit is pushed to the BSTK and before the
; @0 bit is cleared. During the Timer0 interrupt routine the @0 bit is set to 1 by
; an external event.
; The new result is cleared, resulting in a miss. To prevent such misses, the
; Timer0 interrupt is disabled before the push to BSTK, and re-enabled after the @0 bit
; is cleared.
;
.macro	gbitx2					; @0 = bit, @1 = data space byte, @2 = namestring		(OK)
gbitx2s:
		.db		@2,0,ctlW		; Namestring,ctlW
		.dw		gbitx2e			; Link address to next word
		.dw		NORM			; Interpreter and runtime
gbitx2L:
		.dw		(gbitx2s-gbitx2L)	; Offset to start of namestring
;
		lds		rmp,TIMSK0		; Timer0 overflow interrupt
		cbr		rmp,1<<TOIE0	; Disable Timer/Counter0 Oveflow Interrupt
		sts		TIMSK0,rmp
;
		lds		rmp,@1			; Get buffer byte
		bst		rmp,@0			; T <-- Rd(b)
		sec						; Assume C = 1
		brts	gbitx21			; Skip next if T = 1
		clc						; Clear C
gbitx21:
		rol		BSTK			; Push C to TOS. BSTK bit 7 lost
		clt						; Clear T
		bld		rmp,@0			; Rd(b) <--T, clear transition bit
		sts		@1,rmp			; Save result
;
		lds		rmp,TIMSK0		; Timer0 overflow interrupt
		sbr		rmp,1<<TOIE0	; Enable Timer/Counter0 Oveflow Interrupt
		sts		TIMSK0,rmp
;
		ret
gbitx2e:
.endm
;
.macro	pbit					; @0 = bit, @1 = byte, @2 = namestring
pbits:
		.db		@2,ctlW			; Namestring,ctlW
		.dw		pbite			; Link address to next word
		.dw		NORM			; Interpreter and runtime
pbitL:
		.dw		(pbits-pbitL)	; Offset to start of namestring
;
		lsr		BSTK			; Pop TOS to C, 0 moved to BSTK bit 7
		bclr	SREG_T			; Clear T bit
		brcc	pbit1			; Set the Rd(b)
		bset	SREG_T			; Set T bit
pbit1:
		bld		@1,@0			; Rd(b) <-- T
		ret
pbite:
.endm
;
; Pop bit stack and put to virtual bit (in data space byte). C destroyed.		(OK)
;
.macro	pvbit					; @0 = bit, @1 = data space byte, @2 = namestring
pvbits:
		.db		@2,ctlW			; Namestring,ctlW
		.dw		pvbite			; Link address to next word
		.dw		NORM			; Interpreter and runtime
pvbitL:
		.dw		(pvbits-pvbitL)	; Offset to start of namestring
;
		lsr		BSTK			; Pop TOS to C, 0 moved to BSTK bit 7
		bclr	6				; Clear T bit
		brcc	pvbit1			; Set the Rd(b)
		bset	6				; Set T bit
pvbit1:
		lds		rmp,@1
		bld		rmp,@0			; Rd(b) <-- T
		sts		@1,rmp
		ret
pvbite:
.endm
;
; Pop bit stack and put to virtual bit (in data space byte). C destroyed.		(OK)
;
.macro	pvbit2					; @0 = bit, @1 = data space byte, @2 = namestring
pvbit2s:
		.db		@2,0,ctlW		; Namestring,ctlW
		.dw		pvbit2e			; Link address to next word
		.dw		NORM			; Interpreter and runtime
pvbit2L:
		.dw		(pvbit2s-pvbit2L)	; Offset to start of namestring
;
		lsr		BSTK			; Pop TOS to C, 0 moved to BSTK bit 7
		bclr	6				; Clear T bit
		brcc	pvbit21			; Set the Rd(b)
		bset	6				; Set T bit
pvbit21:
		lds		rmp,@1
		bld		rmp,@0			; Rd(b) <-- T
		sts		@1,rmp
		ret
pvbit2e:
.endm
;
;  Put bit stack to buffer
;  @0 = Destination buffer
;  @1 = namestring
;
.macro		pbyt
pbyts:
		.db		@1,ctlW
		.dw		pbytx
		.dw		NORM
pbytL:
		.dw		(pbyts-pbytL)
;
		sts		@0,BSTK
		ret
pbytx:
.endm
;
;  Get buffer to bit stack
;  @0 = Destination byte
;  @1 = namestring
;
.macro		gbyt
gbyts:
		.db		@1,ctlW
		.dw		gbytx
		.dw		NORM
gbytL:
		.dw		(gbyts-gbytL)
;
		lds		BSTK,@0
		ret
gbytx:
.endm
;
;
; --- One-shot timers ---
;
; Software one-shot timers T0 through TF are triggered by '.TKn' word,
; see TKTR routine for details on processing the trigger bit. When '.TKn'
; gets a 1 bit, and if the timer value is non-zero, the timer output TQn
; is set high, the TMn timer enable flag is set, and the one-shot timer
; starts timing.
;
; Timing is done by SWD timer routine which operate in the Timer1
; interrupt handler. If the TMn flag is set, the prescaler is decremented
; to zero. On zero rollover, the prescaler is reloaded and the timer count
; buffer is decremented. The timing cycle continues until the TCBn count
; reaches zero, at which time the TQn timer output flag is cleared and the
; TMn timer enable flag is cleared.
;
; The timer buffers are in internal data space. Timers can be be set to any
; decimal value between 0 and 255 counts of the timebase units.
; The timebase runs at 100 ms (t100msf), 1 timer tick every 100 ms.
; The prescalers divide down the timebase to seconds and minutes.
;
; Timer 0..B counts in seconds, timer C..E counts in minutes,
; Timer F counts in 10's of minutes. The timers are
; re-triggerable, meaning the timing cycle is restarted as long as high
; is sensed on TKn bit. Timing cycle starts when the TKn returns to 0.
;
.macro		swd
;
; @0 = TCBn, @1 = PRBn, @2 = TQn, @3 = TMn, @4 = PRVs/PRVm/PRVh
; @5 = tmf01
;
swds:
		lds		rga,@5			; Get timer flags
		sbrs	rga,@3			; TMn cleared?
		rjmp	swdx			;	Yes, exit
		lds		rmp,@1			; Get PRBn
		dec		rmp				; Count ticks
		sts		@1,rmp
		brne	swdx			; Not 0, exit
		ldi		rmp,@4			; Get PRVs/m/h
		sts		@1,rmp			; PRBn <-- PRVs/m/h
;
		lds		rmp,@0			; Get TCBn
		dec		rmp				; TCBn = TCBn-1
		sts		@0,rmp
		brne	swdx			; Not 0, exit
		clt						; T = 0
		bld		rga,@2			; rga(TQn) <-- T
		bld		rga,@3			; rga(TMn) <-- T
		sts		@5,rga			; Save tmf01
swdx:
.endm
;
;
; Get trigger bit for one-shot timer from TOS. If bit fetched by
; '.TK'n = 1, and if the one-shot timer is loaded with a value other
; than 0, enable timer and set timer output. If .TKn returns a 0 bit,
; do nothing and return.
;
.macro	tktr
;
; @0 = TCBn, @1 = PRBn, @2 = TQn, @3 = TMn, @4 = PRVs/m/h
; @5 = TRBn, @6 = tmf01, @7 = namestring
;
tkts:
		.db		@7,0,ctlW
		.dw		tkte
		.dw		NORM
tktL:
		.dw		(tkts-tktL)
;
itkt:
		lsr		BSTK			; Pop TOS to C
		brcc	tktx
		lds		rmp,@5			; Get TRBn
		tst		rmp
		breq	tktx			; Exit if 0
		ldi		rmp,@4			; Get PRVs/m
		sts		@1,rmp			; Store PRBn
		lds		rmp,@5			; Get TRBn
		sts		@0,rmp			; Store TCBn
		lds		rmp,@6			; Get timer flags
		set						; T=1
		bld		rmp,@2			; Set TQn bit
		bld		rmp,@3			; Set TMn bit
		sts		@6,rmp			; Store timer flags
tktx:
		ret
tkte:
.endm
;
;
; --- Reset Timers ---
;
.macro		rstd
;
; @0 = TQn, @1 = TMn, @2 = tmf01, @3 = namestring
;
rstds:
		.db		@3,0,ctlW		; Namestring,ctlW
		.dw		rstde			; Link address
		.dw		NORM
rstdL:
		.dw		(rstds-rstdL)
;
irstd:
		lsr		BSTK			; Pop TOS to C
		brcs	rstd1
		ret
;
rstd1:
		lds		rmp,@2			; Get timer flags
		clt						; T=0
		bld		rmp,@0			; Clear TQn output
		bld		rmp,@1			; Clear TMn bit
		sts		@2,rmp			; Store timer flags
		ret
rstde:
.endm
;
; Load TRBn timer reload buffer with (wdbuf) if drdy flag set
; Write (wdbuf+1) contents to TRBn EEPROM then clear drdyf.
;
;
.macro	ldtm
;
;	@0 = TRBn @1 = namestring
;
ldtms:
		.db		@1,0,ctlW
		.dw		ldtme
		.dw		NORM
ldtmL:
		.dw		(ldtms-ldtmL)
;
ildtm:
		sbrs	flagb,drdyf
		rjmp	ldtmx
		cbr		flagb,(1<<drdyf)	; Clear data ready flag
		lds		rga,wdbuf+1			; Get current wdbuf contents
		sts		@0,rga				; Update TRBn
;
; Write updated wdbuf to EEPROM. Note XL-1 offset correction
;
		ldxptr	(@0-uf0st-1)		; Offset from base address - 1
		call	wr_trb				; Write rga to EEPROM
;		
ldtmx:
		ret
ldtme:
.endm
;
; Input bit rising edge transitions toggle output bit
;
.macro	togg
;
; @0 =  n, @1 = tg_old, @2 = tg_out, @3 = namestring
toggs:
		.db		@3,0,ctlW
		.dw		togge
		.dw		NORM
toggL:
		.dw		(toggs-toggL)
itogg:
		lsr		BSTK			; Pop TOS to C
		brcs	togg1
;
; Input is 0
;
		lds		rmp,@1			; Get old result byte
		cbr		rmp,(1<<@0)		; Clear bit
		sts		@1,rmp			; Update old to current
		ret						
togg1:
;
; Input is 1. Check old input
;
		lds		rmp,@1			; Get old result byte
		sbrc	rmp,@0			; Old result = 0?
		ret						; Past rising edge
;
; Rising edge detected
;
		lds		rmp,@1			; Get old result byte
		sbr		rmp,(1<<@0)		; Set bit
		sts		@1,rmp			; Update old to current
;
; Complement the output bit
;
		lds		rmp,@2			; Get output byte
		ldi		rga,(1<<@0)		; Complement bit mask
		eor		rmp,rga			; Complement selected bit
		sts		@2,rmp			; Update output
		ret
togge:
.endm
;
;
; --- SRAM Data Segment ---
;
.DSEG
.ORG	0X0100		; 2 Kb SRAM space
;
; --- User File Area ---
;
filbeg:
;
uf0st:
.byte	0x200		; 256 words - Don't move this buffer, aligned to word boundary!
;
; One-shot timer data and flag buffers. 1 flag buffer required
; per two timers. timers require 28 bytes total.
;
TRB0:
.byte	1			; Timer_0 reload buffer
TRB1:
.byte	1			; Timer_1 reload buffer
TRB2:
.byte	1			; Timer_2 reload buffer
TRB3:
.byte	1			; Timer_3 reload buffer
TRB4:
.byte	1			; Timer_4 reload buffer
TRB5:
.byte	1			; Timer_5 reload buffer
TRB6:
.byte	1			; Timer_6 reload buffer
TRB7:
.byte	1			; Timer_7 reload buffer
;
TRB8:
.byte	1			; Timer_8 reload buffer
TRB9:
.byte	1			; Timer_9 reload buffer
TRBa:
.byte	1			; Timer_A reload buffer
TRBb:
.byte	1			; Timer_B reload buffer
TRBc:
.byte	1			; Timer_C reload buffer
TRBd:
.byte	1			; Timer_D reload buffer
TRBe:
.byte	1			; Timer_E reload buffer
TRBf:
.byte	1			; Timer_F reload buffer
;
;
;
filend:
;
;
CNTb:
.byte   1           ; Gated count buffer
;
TM1sb:
.byte   1           ; 1s timer count buffer
;
;
; Timer count buffers
;
TCB0:
.byte	1			; Timer_0 count buffer
TCB1:
.byte	1			; Timer_1 count buffer
TCB2:
.byte	1			; Timer_2 count buffer
TCB3:
.byte	1			; Timer_3 count buffer
TCB4:
.byte	1			; Timer_4 count buffer
TCB5:
.byte	1			; Timer_5 count buffer
TCB6:
.byte	1			; Timer_6 count buffer
TCB7:
.byte	1			; Timer_7 count buffer
;
TCB8:
.byte	1			; Timer_8 count buffer
TCB9:
.byte	1			; Timer_9 count buffer
TCBa:
.byte	1			; Timer_A count buffer
TCBb:
.byte	1			; Timer_B count buffer
TCBc:
.byte	1			; Timer_C count buffer
TCBd:
.byte	1			; Timer_D count buffer
TCBe:
.byte	1			; Timer_E count buffer
TCBf:
.byte	1			; Timer_F count buffer
;
PRB0:
.byte	1			; Timer_0 prescale count buffer
PRB1:
.byte	1			; Timer_1 prescale count buffer
PRB2:
.byte	1			; Timer_2 prescale count buffer
PRB3:
.byte	1			; Timer_3 prescale count buffer
PRB4:
.byte	1			; Timer_4 prescale count buffer
PRB5:
.byte	1			; Timer_5 prescale count buffer
PRB6:
.byte	1			; Timer_6 prescale count buffer
PRB7:
.byte	1			; Timer_7 prescale count buffer
;
PRB8:
.byte	1			; Timer_8 prescale count buffer
PRB9:
.byte	1			; Timer_9 prescale count buffer
PRBa:
.byte	1			; Timer_A prescale count buffer
PRBb:
.byte	1			; Timer_B prescale count buffer
PRBc:
.byte	1			; Timer_C prescale count buffer
PRBd:
.byte	1			; Timer_D prescale count buffer
PRBe:
.byte	1			; Timer_E prescale count buffer
PRBf:
.byte	1			; Timer_F prescale count buffer
;
tmf01:
.byte	1			; Timers 0 & 1 control flags
tmf23:
.byte	1			; Timers 2 & 3 control flags
tmf45:
.byte	1			; Timers 4 & 5 control flags
tmf67:
.byte	1			; Timers 6 & 7 control flags
;
tmf89:
.byte	1			; Timers 8 & 9 control flags
tmfab:
.byte	1			; Timers A & B control flags
tmfcd:
.byte	1			; Timers C & D control flags
tmfef:
.byte	1			; Timers E & F control flags
;
linbuf:
.byte	linsz		; Character input line buffer
;
basln:
.byte	2			;* Base line buffer, 16b wide
pointr:
.byte	2			;* Target address in UFx, 16b wide
lnctr:
.byte	2			;* Program line counter, 16b wide
attrb:
.byte	2			;* Attribute type buffer
;
offs:
.byte	1			; Offset buffer
cursor:
.byte	1			; Cursor offset position
hzr:
.byte	1			; Horizontal relative position
vtr:
.byte	1			; Vertical relative position
;
; Data buffer for word and byte
;
wdbuf:
.byte	2			; Word byte buffer
dba:
.byte	1			; Data byte buffer (don't move position relative to wdbuf)
;
; Port image buffers
;
pa:
.byte	1			; Port A image buffer
;
pb:
.byte	1			; Port A image buffer
;
py:
.byte	1			; Port A image buffer
;
pz:
.byte	1			; Port A image buffer
;
pu:
.byte	1			; Port U image buffer
;
pv:
.byte	1			; Port V image buffer
;
;
; Port A debounce buffers
;
paim:
.byte	1			; Previous port A scan image
pal:
.byte	1			; Port A leading edge bits
pat:
.byte	1			; Port A trailding edge bits
pad:
.byte	1			; Port A double action bits
padb:
.byte	1			; Port A debounced image
dbr1:
.byte	1			; Debounce result register 1
dbr2:
.byte	1			; Debounce result register 2
;
; Flip/Flop control registers
;
vsff_0:
.byte	1			; F/F Set byte
vrff_0:
.byte	1			; F/F Reset byte
vqff_0:
.byte	1			; F/F Output byte
;
vsff_1:
.byte	1			; F/F Set byte
vrff_1:
.byte	1			; F/F Reset byte
vqff_1:
.byte	1			; F/F Output byte
;
; Toggle output on input bit rising edge
;
tg_old:
.byte	1			; Input flag byte, old result
;
tg_out:
.byte	1			; Output flag byte
;
;
;
buffend:
;
;
; ============================================
;   R E S E T   A N D   I N T   V E C T O R S
; ============================================
;
;
; --- Code Segment ---
;
.CSEG
.ORG	$0000						; Interrupt vectors go here
;
		jmp		start				; Int vector 1 - Reset vector
;
.ORG	OC0Aaddr
		jmp			Timer0_COMPA	; Timer 0 Output Compare A handler
;
.ORG	OC1Aaddr
		jmp			Timer1_COMPA	; Timer 1 Output Compare A handler
;
;
; End of interrupt vectors, start of program code space
;
.ORG	0x0034						; Program begins here
;
;
;###########################################################################
;
verm:	.db	"*** Version 1.0 - 2025 Mar 17 ***",ctlZ
;
;###########################################################################
;
;
;
; ============================================
;     I N T E R R U P T   S E R V I C E S
; ============================================
;
;
; --- Timer 0 interrupt handler ---
;
; TCNT0 run in Output Compare mode, using OCR0A register to
; generate output compare interrupt every 4.096 ms. TCNT0 operates
; in Clear Timer on Compare Match (WGM02:0 = 2). OCR0A defines
; the counter's TOP value.
;
; On Compare Match, TCNT0 counter is cleared.
; Clk_t0 = 16 MHz/1024 = 15.625 kHz, 64 us period.
;
Timer0_COMPA:
		push	rmp					; Save registers
		push	rga
		push	rgb
		in		SRsav,SREG
;
; --- Timers using 4.096 ms tick, ms ---
;
		swd		TCB0,PRB0,TQ0,TM0,PRVs,tmf01
		swd		TCB1,PRB1,TQ1,TM1,PRVs,tmf01
		swd		TCB2,PRB2,TQ2,TM2,PRVs,tmf23
		swd		TCB3,PRB3,TQ3,TM3,PRVs,tmf23
;
;
		sbr		flagb,(1<<t4msf)	; Set timer 0 overflow flag
;
; --- Update Output Port PD7...PD2 ---
;
; PY5...0 bits are mapped to PD7...PD2 pins
;
outy:
		lds		rmp,PY
		sbis	PINB,out_sense		; Test if out_sense pin jumpered
		com		rmp					;	Jumpered, invert results
		andi	rmp,0b00111111		; Mask off bits 7,6
		lsl		rmp
		lsl		rmp					; Align data
		in		rga,PIND			; Read PIND
		andi	rga,0b00000011		; Clear output field
		or		rmp,rga				; Write results to port pins
		out		PORTD,rmp
;
; PB4, PB5 pins mapped to PY6,7 outputs
;
		lds		rmp,PY				; Get PY image from RAM buffer
		sbis	PINB,out_sense		; Test if out_sense pin jumpered
		rjmp	inv_py				;	Jumpered, invert results

; Normal sense
		sbi		PORTB,PB5			; Set bit
		sbrs	rmp,Y7				; If Y7 = 0
		cbi		PORTB,PB5			; Clear output pin
;
		sbi		PORTB,PB4			; Set bit
		sbrs	rmp,Y6				; If Y6 = 0
		cbi		PORTB,PB4			; Clear output pin
		rjmp	outex				; Exit
;
; Invert output values
;
inv_py:
		cbi		PORTB,PB5			; Clear bit
		sbrs	rmp,Y7				; If Y7 = 0
		sbi		PORTB,PB5			; Set output pin
;
		cbi		PORTB,PB4			; Clear bit
		sbrs	rmp,Y6				; If Y6 = 0
		sbi		PORTB,PB4			; Set output pin
outex:
;
	;	in		rga,PINB			; Read PINB
	;	andi	rga,0b1100000		; Clear output field
	;	or		rmp,rga				; Write results to port pins
	;	out		PORTB,rmp
;
;
; Read and debounce Input Port PC5...PC0 bits ---
;
inputa:
		in		rgb,PINC			; Read Port C
		andi	rgb,0b00111111		; Mask off bits 7,6
;
		lds		rga,paim			; Get previous scan image
		sts		paim,rgb			; Save this scan as previous image
		eor		rga,rgb				; 1 = changed bit, 0 = no change
		sts		dbr1,rga			; Save bit pattern of changes
		com		rga					; 0 = changed bit, 1 = no change
		and		rga,rgb				; Changed bits cleared to 0
		sts		dbr2,rga			; Save input with changed bits = 0
;
;  Merge in debounced bits to the old input image, keeping the old bits in
;  the positions that are not debounced. That is, an input bit is not
;  changed from its old state until it is debounced, since it could be
;  flipping on each scan cycle.
;
		lds		rga,pa				; Get old debounced input image
		lds		rmp,dbr1			; 1 = changed bit, 0 = no change
		and		rga,rmp				; 0 = debounced positions
		lds		rmp,dbr2
		or		rga,rmp				; Merge in new debounced bits
		lds		rmp,pa				; Save old input image
		sts		dbr2,rmp
		sts		pa,rga				; Update new debounced input image
;
;  Process -ve transitions for counter inputs ANDing the complemented
;  scan image with the changed bit mask in DBR1. Merge result with the
;  trailing edge trigger byte, pat.
;
		mov		rga,rgb				; Get input port reading
		com		rga					; Invert port reading
		lds		rmp,dbr1
		and		rga,rmp				; Isolate -ve transitions
		lds		rmp,pat
		or		rmp,rga				; Merge to trigger byte
		sts		pat,rmp				; Save result back to pat
;
;  Process +ve transition bits by ANDing bits in the scan image with the
;  changed bits mask in DBR1 to generate 1's in positions with 0 to 1
;  transitions in DBR1. Merge this resut to the transition image byte.
;
		mov		rga,rgb				; Get input port reading
		lds		rmp,dbr1
		and		rga,rmp				; Isolate +ve transitions
		lds		rmp,pal
		or		rmp,rga				; Merge to trigger byte
		sts		pal,rga				; Save result back to pat
;
;  Process double action bits. Each double action output bit acts as a
;  flip flop that change output state on each 0 to 1 transition of the
;  input bit. Each input bit has a corresponding double action output bit.
;
		lds		rmp,pad				; Process double action changes
		eor		rga,rmp
		sts		pad,rga				; Sace result back to pad
;
		out		SREG,SRsav			; Restore SREG
		pop		rgb
		pop		rga
		pop		rmp
		reti
;
;
; --- Timer 1 interrupt handler ---
;
; TCNT1 run in output compare mode, using OCR1A register to
; generate output compare interrupt every 100 ms. TCNT1 operates in mode 5
; CTC (WGM13:10 = 4). OCR1A define the counter's TOP value.
;
; TCNT1 is cleared on compare match with OCR1A channel.
; Clk_t1 = 16 MHz/256 = 62.5 kHz, 16 us period.
;
; Interrupt rate: 16 us x 6250 = 0.1 s
;
Timer1_COMPA:
		push	rmp					; Save registers
		push	rga
		in		SRsav,SREG
;
		sbr		flaga,(1<<t100msf)	; Set 100 ms flag
		sbr		flagb,(1<<paceb)	; Set 100 ms display pace flag
;
; --- Service software timers ---
;
; Timebase for CLKA count down timra to 0, reload counter
; and Complement CLKA every 200 ms
;
tbasea:
		dec		timra				; Count down timer
		brne	tbaseax
		ldi		rmp,timrav			; Reload timer b
		mov		timra,rmp
		ldi		rga,(1<<clkaf)
		eor		flagb,rga
tbaseax:
;
; Timebase for CLKB count down timrb to 0, reload counter
; and set timer flag t600ms
;
tbaseb:
		dec		timrb				; Count down timer
		brne	tbasebx
		ldi		rmp,timrbv			; Reload timer b
		mov		timrb,rmp
		sbr		flaga,(1<<t600msf)	; Set timer flag
;
; Complement CLKB every 600 ms
;
		ldi		rga,(1<<clkbf)
		eor		flagb,rga
tbasebx:
;
; Timebase for CLKBC count down timrc to 0, reload counter
; and set timer flag t1800msf
;
tbasec:
		dec		timrc				; Count down timer
		brne	tbasecx
		ldi		rmp,timrcv			; Reload timer c
		mov		timrc,rmp
		sbr		flaga,(1<<t1800msf)	; Set timer flag
;
; Complement CLKC every 1.8 s
;
		ldi		rga,(1<<clkcf)
		eor		flagb,rga
tbasecx:
;
; Timebase for 4 s tick count down timrd to 0, reload counter
; and set timer flag t4sf
;
tbased:
		dec		timrd				; Count down timer
		brne	tbasedx
		ldi		rmp,timrdv			; Reload timer d
		mov		timrd,rmp
		sbr		flagb,(1<<t4sf)	    ; Set timer flag
tbasedx:
;
; Generate 1 s tick flag
;
tbase_1s:

        lds     rga,TM1sb
        dec     rga
        sts     TM1sb,rga         ; Reload prescaler
        brne    tbase_1sx       	; Keep downcounting
        sbr     flagb,(1<<t1sf)     ; Set 1 s tick flag
        ldi     rga,10
        sts     TM1sb,rga         ; Reload prescaler
tbase_1sx:
;
; --- Timers using 100 ms tick, seconds ---
;
		swd		TCB4,PRB4,TQ4,TM4,PRVs,tmf45
		swd		TCB5,PRB5,TQ5,TM5,PRVs,tmf45
		swd		TCB6,PRB6,TQ6,TM6,PRVs,tmf67
		swd		TCB7,PRB7,TQ7,TM7,PRVs,tmf67
		swd		TCB8,PRB8,TQ8,TM8,PRVs,tmf89
		swd		TCB9,PRB9,TQ9,TM9,PRVs,tmf89
		swd		TCBA,PRBA,TQA,TMA,PRVs,tmfab
		swd		TCBB,PRBB,TQB,TMB,PRVs,tmfab
;
; --- Timers using 600 ms tick, minutes ---
;
		sbrs	flaga,t600msf
		rjmp	timf
;
		swd		TCBC,PRBC,TQC,TMC,PRVm,tmfcd
		swd		TCBD,PRBD,TQD,TMD,PRVm,tmfcd
		swd		TCBE,PRBE,TQE,TME,PRVm,tmfef
;
; --- Timer Tf using 4 s flag
;
timf:
		sbrs	flagb,t4sf
		rjmp	timfx
;
		swd		TCBF,PRBF,TQF,TMF,PRVh,tmfef
;
;  Clear tick flags after timer processing
;
timfx:
		cbr		flaga,(1<<t100msf)|(1<<t600msf)|(1<<t1800msf)
		cbr		flagb,(1<<t4sf)
;
; Exit service toutine
;
		out		SREG,SRsav
		pop		rga					; Restore registers
		pop		rmp
		reti
;
;
;###########################################################################
;
;
; --- Initialization Routines ---
;
; Initialize the Tile Engine
;
initz:
		call	zbuf			; Clear data space buffers, excluding uf0 space
		call	zregs			; Clear lower registers R0,..,R15
		clr		flaga			; Clear flags
		clr		flagb
;
; Initialize PB0/D8 as autostart, PB1/D9 as output_sense. Activate pull-ups
;
		sbi		PORTB,PB0		; Activate pull-up
		sbi		PORTB,PB1		; Activate pull-up
	;	sbi		PORTD,PD4		; Activate pull-up
;
; Initialize PB4, PB5 as outputs for Y6, Y7
;
		sbi		DDRB,PB4		; Y6 output
		sbi		DDRB,PB5		; Y7 output
;
; Initialize PC5...PC0 as inputs, pull-ups activated
;
		ldi		rmp,0b00111111
		out		PORTC,rmp		; Set PC bits high
;
; Initialize PD7...PD2 as outputs
;
		ldi		rmp,0b11111100
		out		DDRD,rmp		; Set PD as outputs
	;	ldi		rmp,0b00111111
	;	out		DDRB,rmp		; Set PB as outputs
;
;
; --- Timers Initialization ----
;
; === TCNT0 Initialization === (OK)
;
; Setup 8 bit Timer/Counter0 Compare A in CTC mode
; Setup TCNT0 prescaler = 1024, 64 us period
;
InitTimer0:
		ldi		rmp,(1<<CS02)|(1<<CS00)	; Divide by 1024 prescaler, Fclk = 15.625 kHz
		out		TCCR0B,rmp				; Timer/Counter0 control register B
;
; Setup TCNT0 for CTC mode
;
		ldi		rmp,(1<<WGM01)			; CTC mode
		out		TCCR0A,rmp				; Timer/Counter0 control register A
;
; Initialize OCR0A output compare register
;
		ldi		rmp,OCR0Aload			; Set OCR0A = 64 for 4.096 ms period
		out		OCR0A,rmp
;
; Enable Timer/Counter0 Compare A Match Interrput in TIMSK0
;
		lds		rmp,TIMSK0
		sbr		rmp,(1<<OCIE0A)			; Enable Timer/Counter0 Output Compare A Match Interrupt
		sts		TIMSK0,rmp
;
;
; === TCNT1 Initialization === (OK)
;
; Setup 16 bit Timer/Counter1 Compare A in CTC mode
; Setup TCNT1 prescaler for 256, clock period = 16 us and CTC mode
;
InitTimer1:
		ldi		rmp,(1<<CS12)|(1<<WGM12)	; Divide by 256 prescaler, Fclk = 62.5 kHz
		sts		TCCR1B,rmp					; Write to control register B
;
; Initialize OCR1A output compare register
;
		ldi		rmp,high(OCR1Aload)		; Set OCR1A = 100 ms period
		ldi		rga,low(OCR1Aload)
		sts		OCR1AH,rmp				; 16 bit write
		sts		OCR1AL,rga
;
; Enable TCNT1 Compare A Match Interrputs in TIMSK1
;
		lds		rmp,TIMSK1
		sbr		rmp,(1<<OCIE1A)		; Enable Output Compare A Match Interrupt
		sts		TIMSK1,rmp
;
;
; Load software timra, timrb and timrc counters with reload values
;
		ldi		rmp,timrav			; Reload timer
		mov		timra,rmp
;
		ldi		rmp,timrbv			; Reload timer
		mov		timrb,rmp
		cbr		flaga,(1<<t600msf)	; Clear t1800msf flag
;
		ldi		rmp,timrcv			; Reload timer
		mov		timrc,rmp
		cbr		flaga,(1<<t1800msf)	; Clear t1800msf flag
;
		ldi		rmp,timrdv			; Reload timer
		mov		timrd,rmp
		cbr		flagb,(1<<t4sf)		; Clear t4sf flag
		ret
;
; Initialize the UART for 19,200 baud asynchronous operation
;
inzuart:
		cli							; Clear global interrupts
		ldi		rmp,high(BAUD_PRE)
		sts		UBRR0H,rmp			; Load baud rate register high
		ldi		rmp,low(BAUD_PRE)
		sts		UBRR0L,rmp			; Load baud rate register low
;
; Setup frame for 1 start, 8 data, 1 stop and no parity
;
		ldi		rmp,(1<<UCSZ00)|(1<<UCSZ01)
		sts		UCSR0C,rmp
;
; Enable the UART
;
		ldi		rmp,(1<<RXEN0)|(1<<TXEN0)
		sts		UCSR0B,rmp			; Enable RX and TX
;
; --- Enable GLobal Interrupts
;
		sei							; Set global interrupts
		ret
;
; Initialize User File Space buffers
;
inzuf0:
		ldi		rmp,high(uf0st)
		mov		ufpch,rmp			; Setup UF program counter
		ldi		rmp,low(uf0st)
		mov		ufpcl,rmp
;
		clr		rmp
		sts		basln,rmp
		sts		basln+1,rmp			; Clear basln buffer
		sts		cursor,rmp			; Clear cursor buffer
;
		ret
;
;
;###########################################################################
;
;  UWORDS.ASM - UTL Dictionary
;
;  Copyright (c)   2013 Jan 25   Francis A. Lyn
;  Version 0
;
; --- The Word Dictionary ---
;
;  These subroutine modules form the WORDS that make up the DICTIONARY.
;  Each module consists of a header section, a main code body, and a RET
;  instruction at the end of the code body.
;
;  The header starts with a ^W terminated namestring, ['namestring',^W],
;  followed by a link address (LA) to next word in the dictionary. The LA
;  is followed by an offset byte. When this offset byte value is added to a
;  16 bit pointer to the offset byte location, the result will move the
;  pointer to the start of the WORD's namestring. The offset byte is in 2's
;  complement, as the namestring starts somewhere ahead of the the main code
;  body. The offset byte is used by the Editor to locate the beginning of
;  the namestring, given the vector to the start of the main code body, IWRD.
;
;  The offset byte is followed by the Attribute type byte. This byte
;  determines the execution environment for the word, i.e. whether the
;  word is interpreter only, interpreter and program run time (normal),
;  or run time only (not executed in interpreter mode).
;
;  The main code body for the WORD comes next, and is terminated by a
;  RET instruction.
;
;  The WORD structure is as follows:
;
;WORD1:	DB	'namestring',^W	Name of word			!!!MUST BE EVEN NUMBER OF BYTES!!!
;	DW	Link_Address	Address of next WORD2
;	DW	(WORD1-$)	Offset to start of namestring
;	DW	Attribute	Controls execution environment
;IWRD:	code		Subroutine code start for this word
;	.
;	.
;	code
;	RET
;
;WORD2: next Dictionary word, defined by the link_address field
;
;
; --- Attribute Type Definitions ---
;
;
;NORM	.dw		0	Default, Interpreter and Runtime command
;EDIT	.dw		1	Editor mode only
;
; ---
;
;  Bit Stack Push and Pop Operations
;
; ---
;
dict:							; Dictionary entry point
;
; ---
;
drop:
		.db		"DROP",0,ctlW
		.dw		drope			; Link address to next word
		.dw		NORM
dropL:
		.dw		(drop-dropL)
;
idrop:
		lsr		BSTK			; Pop TOS, discard bit, (n1 --   )
		ret
drope:
;
dup1:
		.db		"DUP",ctlW
		.dw		dupe
		.dw		NORM
dupL:
		.dw		(dup1-dupL)
;
idup:
		lsr		BSTK			; Duplicate TOS, (n1 -- n1,n1)
		brcc	dup2
		rol		BSTK			; TOS <-- C=1
		sec
		rol		BSTK			; TOS <-- C=1
		ret
;
dup2:
		rol		BSTK			; TOS <-- C=0
		clc
		rol		BSTK			; TOS <-- C=0
		ret
dupe:
;
; ( b1 b2 -- b1 b2 b1 b2)
;
dup2_1:
		.db		"2DUP",0,ctlW
		.dw		dup2_e
		.dw		NORM
dup2_L:
		.dw		(dup2_1-dup2_L)
;
idup2_:
		mov		rmp,BSTK		; Copy BSTK
		lsr		rmp
		lsr		rmp				; C = b1
		mov		rmp,BSTK		; Copy BSTK again
		rol		BSTK			; TOS <-- C = b1
		lsr		rmp				; C = b2
		rol		BSTK			; TOS <-- C = b2
		ret
dup2_e:
;
; Logical OR
;
or1:
		.db		"OR",0,ctlW
		.dw		ore
		.dw		NORM
orL:
		.dw		(or1-orL)
;
ior:
		mov		rmp,BSTK	; Copy BSTK to rmp
		lsr		rmp			; Operand bits in bit 0 position
		or		rmp,BSTK	; Perform logical OR
		lsr		BSTK		; Drop bit stack
		lsr		BSTK		; Drop bit stack
		lsr		rmp			; OR result in C
		rol		BSTK		; Push C to TOS
		ret
ore:
;
; Logical AND
;
and1:
		.db		"AND",ctlW
		.dw		ande
		.dw		NORM
andL:
		.dw		(and1-andL)
;
iand:
		mov		rmp,BSTK	; Copy BSTK to rmp
		lsr		rmp			; Operand bits in bit 0 position
		and		rmp,BSTK	; Perform logical AND
		lsr		BSTK		; Drop bit stack
		lsr		BSTK		; Drop bit stack
		lsr		rmp			; OR result in C
		rol		BSTK		; Push C to TOS
		ret
ande:
;
; Logical complement
;
not1:
		.db		"!",ctlW	; Logical negate TOS
		.dw		note
		.dw		NORM
notL:
		.dw		(not1-notL)
;
inot:
		lsr		BSTK		; Pop TOS
		brcc	not2
		clc					; TOS was 1
		rol		BSTK		; Push C to TOS
		ret
;
not2:
		sec					; TOS was 0
		rol		BSTK		; Push C to TOS
		ret
note:
;
; --- Complement the bit stack byte --
;
cplbs1:
		.db		"!!",0,ctlW	; Logical negate bit stack byte
		.dw		cplbse
		.dw		NORM
cplbsL:
		.dw		(cplbs1-cplbsL)
;
icplbs:
		sbr		rmp,0xff
		eor		BSTK,rmp		; Complement bit stack
		ret
;
cplbse:
;
; --- Swap (b2 b1 -- b1 b2)
;
swp:
		.db		"SWAP",0,ctlW	 
		.dw		swpe
		.dw		NORM
swpL:
		.dw		(swp-swpL)
;
		mov		rmp,BSTK		; Copy BSTK to rmp
		lsr		BSTK			; Pop b0 to C
		lsr		BSTK			; Pop b1 to C
		lsr		rmp				; Pop b0 to C from rmp copy
;
		rol		BSTK			; Push b0 to BSTK
		lsr		rmp				; Pop b1 to C from rmp copy
		rol		BSTK			; Push b0 to SKB0
		ret
swpe:
;
; Logical XOR 
;
xor:
		.db		"XOR",ctlW
		.dw		xore
		.dw		NORM
xorL:
		.dw		(xor-xorL)
;
ixor:
		mov		rmp,BSTK	; Copy BSTK to rmp
		lsr		rmp			; Operand bits in bit 0 position
		eor		rmp,BSTK	; Perform logical XOR
		lsr		BSTK		; Drop bit stack
		lsr		BSTK		; Drop bit stack
		lsr		rmp			; XOR result in C
		rol		BSTK		; Push C to TOS
		ret
xore:
;
; --- Byte transfers to bit stack ---		(OK)
;
		gbyt	PA,"PA."
		gbyt	PU,"PU."
		gbyt	PV,"PV."
		gbyt	PY,"PY."
;
; --- Bit stack transfers to bytes ---		(OK)
;
		pbyt	PA,".PA"
		pbyt	PU,".PU"
		pbyt	PV,".PV"
		pbyt	PY,".PY"
;
; --- Tile defined bits ---		(OK)
;
; Software internal clock bits
;
		gbit	clkaf,flagb,"CLKA."		; Get CLKA bit, 100 ms
		gbit	clkbf,flagb,"CLKB."		; Get CLKB bit, 600 ms
		gbit	clkcf,flagb,"CLKC."		; Get CLKC bit, 1200 ms
;
; Port A input bits
;
		gvbit	A0,PA,"A0."		; Get bit A0
		gvbit	A1,PA,"A1."		; Get bit A1
		gvbit	A2,PA,"A2."		; Get bit A2
		gvbit	A3,PA,"A3."		; Get bit A3
		gvbit	A4,PA,"A4."		; Get bit A4
		gvbit	A5,PA,"A5."		; Get bit A5
		gvbit	A6,PA,"A6."		; Get bit A6
		gvbit	A7,PA,"A7."		; Get bit A7
;
; Port A +ve edge transition bits
;
		gbitx2	AL0,PAL,"AL0."	; Get bit AL0
		gbitx2	AL1,PAL,"AL1."	; Get bit AL1
		gbitx2	AL2,PAL,"AL2."	; Get bit AL2
		gbitx2	AL3,PAL,"AL3."	; Get bit AL3
		gbitx2	AL4,PAL,"AL4."	; Get bit AL4
		gbitx2	AL5,PAL,"AL5."	; Get bit AL5
		gbitx2	AL6,PAL,"AL6."	; Get bit AL6
		gbitx2	AL7,PAL,"AL7."	; Get bit AL7
;
; Port A -ve edge transition bits
;
		gbitx2	AT0,PAT,"AT0."	; Get bit AT0
		gbitx2	AT1,PAT,"AT1."	; Get bit AT1
		gbitx2	AT2,PAT,"AT2."	; Get bit AT2
		gbitx2	AT3,PAT,"AT3."	; Get bit AT3
		gbitx2	AT4,PAT,"AT4."	; Get bit AT4
		gbitx2	AT5,PAT,"AT5."	; Get bit AT5
		gbitx2	AT6,PAT,"AT6."	; Get bit AT6
		gbitx2	AT7,PAT,"AT7."	; Get bit AT7
;
; Port A double action bits
;
		gvbit2	AD0,PAD,"AD0.",0	; Get bit AD0
		gvbit2	AD1,PAD,"AD1.",0	; Get bit AD1
		gvbit2	AD2,PAD,"AD2.",0	; Get bit AD2
		gvbit2	AD3,PAD,"AD3.",0	; Get bit AD3
		gvbit2	AD4,PAD,"AD4.",0	; Get bit AD4
		gvbit2	AD5,PAD,"AD5.",0	; Get bit AD5
		gvbit2	AD6,PAD,"AD6.",0	; Get bit AD6
		gvbit2	AD7,PAD,"AD7.",0	; Get bit AD7
		gvbit	U0,PU,"U0."		; Get bit U0
		gvbit	U1,PU,"U1."		; Get bit U1
		gvbit	U2,PU,"U2."		; Get bit U2
		gvbit	U3,PU,"U3."		; Get bit U3
		gvbit	U4,PU,"U4."		; Get bit U4
		gvbit	U5,PU,"U5."		; Get bit U5
		gvbit	U6,PU,"U6."		; Get bit U6
		gvbit	U7,PU,"U7."		; Get bit U7
;
		pvbit	U0,PU,".U0"		; Put bit U0
		pvbit	U1,PU,".U1"		; Put bit U1
		pvbit	U2,PU,".U2"		; Put bit U2
		pvbit	U3,PU,".U3"		; Put bit U3
		pvbit	U4,PU,".U4"		; Put bit U4
		pvbit	U5,PU,".U5"		; Put bit U5
		pvbit	U6,PU,".U6"		; Put bit U6
		pvbit	U7,PU,".U7"		; Put bit U7
;
; Port V internal storage bits
;
		gvbit	V0,PV,"V0."		; Get bit V0
		gvbit	V1,PV,"V1."		; Get bit V1
		gvbit	V2,PV,"V2."		; Get bit V2
		gvbit	V3,PV,"V3."		; Get bit V3
		gvbit	V4,PV,"V4."		; Get bit V4
		gvbit	V5,PV,"V5."		; Get bit V5
		gvbit	V6,PV,"V6."		; Get bit V6
		gvbit	V7,PV,"V7."		; Get bit V7
;
		pvbit	V0,PV,".V0"		; Put bit V0
		pvbit	V1,PV,".V1"		; Put bit V1
		pvbit	V2,PV,".V2"		; Put bit V2
		pvbit	V3,PV,".V3"		; Put bit V3
		pvbit	V4,PV,".V4"		; Put bit V4
		pvbit	V5,PV,".V5"		; Put bit V5
		pvbit	V6,PV,".V6"		; Put bit V6
		pvbit	V7,PV,".V7"		; Put bit V7
;
; Port Y output bits
;
		pvbit	Y0,PY,".Y0"		; Put bit Y0
		pvbit	Y1,PY,".Y1"		; Put bit Y1
		pvbit	Y2,PY,".Y2"		; Put bit Y2
		pvbit	Y3,PY,".Y3"		; Put bit Y3
		pvbit	Y4,PY,".Y4"		; Put bit Y4
		pvbit	Y5,PY,".Y5"		; Put bit Y5
		pvbit	Y6,PY,".Y6"		; Put bit Y6
		pvbit	Y7,PY,".Y7"		; Put bit Y7
;
; Flip/Flip control bits
;
		gvbit	QF0,VQFF_0,"Q0."	; Get bit QF0
		gvbit	QF1,VQFF_0,"Q1."	; Get bit QF1
		gvbit	QF2,VQFF_0,"Q2."	; Get bit QF2
		gvbit	QF3,VQFF_0,"Q3."	; Get bit QF3
		gvbit	QF4,VQFF_0,"Q4."	; Get bit QF4
		gvbit	QF5,VQFF_0,"Q5."	; Get bit QF5
		gvbit	QF6,VQFF_0,"Q6."	; Get bit QF6
		gvbit	QF7,VQFF_0,"Q7."	; Get bit QF7
;
		pvbit	SF0,VSFF_0,".S0"	; Put bit SF0
		pvbit	SF1,VSFF_0,".S1"	; Put bit SF1
		pvbit	SF2,VSFF_0,".S2"	; Put bit SF2
		pvbit	SF3,VSFF_0,".S3"	; Put bit SF3
		pvbit	SF4,VSFF_0,".S4"	; Put bit SF4
		pvbit	SF5,VSFF_0,".S5"	; Put bit SF5
		pvbit	SF6,VSFF_0,".S6"	; Put bit SF6
		pvbit	SF7,VSFF_0,".S7"	; Put bit SF7
;
		pvbit	RF0,VRFF_0,".R0"	; Put bit RF0
		pvbit	RF1,VRFF_0,".R1"	; Put bit RF1
		pvbit	RF2,VRFF_0,".R2"	; Put bit RF2
		pvbit	RF3,VRFF_0,".R3"	; Put bit RF3
		pvbit	RF4,VRFF_0,".R4"	; Put bit RF4
		pvbit	RF5,VRFF_0,".R5"	; Put bit RF5
		pvbit	RF6,VRFF_0,".R6"	; Put bit RF6
		pvbit	RF7,VRFF_0,".R7"	; Put bit RF7
;
		gvbit	QF8,VQFF_1,"Q8."	; Get bit QF8
		gvbit	QF9,VQFF_1,"Q9."	; Get bit QF9
		gvbit	QFA,VQFF_1,"QA."	; Get bit QFA
		gvbit	QFB,VQFF_1,"QB."	; Get bit QFB
		gvbit	QFC,VQFF_1,"QC."	; Get bit QFC
		gvbit	QFD,VQFF_1,"QD."	; Get bit QFD
		gvbit	QFE,VQFF_1,"QE."	; Get bit QFE
		gvbit	QFF,VQFF_1,"QF."	; Get bit QFF
;
		pvbit	SF8,VSFF_1,".S8"	; Put bit SF8
		pvbit	SF9,VSFF_1,".S9"	; Put bit SF9
		pvbit	SFA,VSFF_1,".SA"	; Put bit SFA
		pvbit	SFB,VSFF_1,".SB"	; Put bit SFB
		pvbit	SFC,VSFF_1,".SC"	; Put bit SFC
		pvbit	SFD,VSFF_1,".SD"	; Put bit SFD
		pvbit	SFE,VSFF_1,".SE"	; Put bit SFE
		pvbit	SFF,VSFF_1,".SF"	; Put bit SFF
;
		pvbit	RF8,VRFF_1,".R8"	; Put bit RF8
		pvbit	RF9,VRFF_1,".R9"	; Put bit RF9
		pvbit	RFA,VRFF_1,".RA"	; Put bit RFA
		pvbit	RFB,VRFF_1,".RB"	; Put bit RFB
		pvbit	RFC,VRFF_1,".RC"	; Put bit RFC
		pvbit	RFD,VRFF_1,".RD"	; Put bit RFD
		pvbit	RFE,VRFF_1,".RE"	; Put bit RFE
		pvbit	RFF,VRFF_1,".RF"	; Put bit RFF
;
; --- One-shot timer flags ---
;
		gvbit2	TQ0,tmf01,"TQ0."	; Timer0 output
		gvbit2	TQ1,tmf01,"TQ1."	; Timer1 output
		gvbit2	TQ2,tmf23,"TQ2."	; Timer2 output
		gvbit2	TQ3,tmf23,"TQ3."	; Timer3 output
		gvbit2	TQ4,tmf45,"TQ4."	; Timer4 output
		gvbit2	TQ5,tmf45,"TQ5."	; Timer5 output
		gvbit2	TQ6,tmf67,"TQ6."	; Timer6 output
		gvbit2	TQ7,tmf67,"TQ7."	; Timer7 output
;
		gvbit2	TQ8,tmf89,"TQ8."	; Timer_8 output
		gvbit2	TQ9,tmf89,"TQ9."	; Timer_9 output
		gvbit2	TQA,tmfab,"TQA."	; Timer_A output
		gvbit2	TQB,tmfab,"TQB."	; Timer_B output
		gvbit2	TQC,tmfcd,"TQC."	; Timer_C output
		gvbit2	TQD,tmfcd,"TQD."	; Timer_D output
		gvbit2	TQE,tmfef,"TQE."	; Timer_E output
		gvbit2	TQF,tmfef,"TQF."	; Timer_F output
;
		rstd	TQ0,TM0,tmf01,".TR0"	; Timer0 reset
		rstd	TQ1,TM1,tmf01,".TR1"	; Timer1 reset
		rstd	TQ2,TM2,tmf23,".TR2"	; Timer2 reset
		rstd	TQ3,TM3,tmf23,".TR3"	; Timer3 reset
		rstd	TQ4,TM4,tmf45,".TR4"	; Timer4 reset
		rstd	TQ5,TM5,tmf45,".TR5"	; Timer5 reset
		rstd	TQ6,TM6,tmf67,".TR6"	; Timer6 reset
		rstd	TQ7,TM7,tmf67,".TR7"	; Timer7 reset
;
		rstd	TQ8,TM8,tmf89,".TR8"	; Timer_8 reset
		rstd	TQ9,TM9,tmf89,".TR9"	; Timer_9 reset
		rstd	TQA,TMA,tmfab,".TRA"	; Timer_A reset
		rstd	TQB,TMB,tmfab,".TRB"	; Timer_B reset
		rstd	TQC,TMC,tmfcd,".TRC"	; Timer_C reset
		rstd	TQD,TMD,tmfcd,".TRD"	; Timer_D reset
		rstd	TQE,TME,tmfef,".TRE"	; Timer_E reset
		rstd	TQF,TMF,tmfef,".TRF"	; Timer_F reset
;
		tktr	TCB0,PRB0,TQ0,TM0,PRVms,TRB0,tmf01,".TK0"	; Timer0 trigger
		tktr	TCB1,PRB1,TQ1,TM1,PRVms,TRB1,tmf01,".TK1"	; Timer1 trigger
		tktr	TCB2,PRB2,TQ2,TM2,PRVms,TRB2,tmf23,".TK2"	; Timer2 trigger
		tktr	TCB3,PRB3,TQ3,TM3,PRVms,TRB3,tmf23,".TK3"	; Timer3 trigger
		tktr	TCB4,PRB4,TQ4,TM4,PRVs,TRB4,tmf45,".TK4"	; Timer4 trigger
		tktr	TCB5,PRB5,TQ5,TM5,PRVs,TRB5,tmf45,".TK5"	; Timer5 trigger
		tktr	TCB6,PRB6,TQ6,TM6,PRVs,TRB6,tmf67,".TK6"	; Timer6 trigger
		tktr	TCB7,PRB7,TQ7,TM7,PRVs,TRB7,tmf67,".TK7"	; Timer7 trigger
;
		tktr	TCB8,PRB8,TQ8,TM8,PRVs,TRB8,tmf89,".TK8"	; Timer_8 trigger
		tktr	TCB9,PRB9,TQ9,TM9,PRVs,TRB9,tmf89,".TK9"	; Timer_9 trigger
		tktr	TCBA,PRBA,TQA,TMA,PRVs,TRBA,tmfab,".TKA"	; Timer_A trigger
		tktr	TCBB,PRBB,TQB,TMB,PRVs,TRBB,tmfab,".TKB"	; Timer_B trigger
		tktr	TCBC,PRBC,TQC,TMC,PRVm,TRBC,tmfcd,".TKC"	; Timer_C trigger
		tktr	TCBD,PRBD,TQD,TMD,PRVm,TRBD,tmfcd,".TKD"	; Timer_D trigger
		tktr	TCBE,PRBE,TQE,TME,PRVm,TRBE,tmfef,".TKE"	; Timer_E trigger
;
		tktr	TCBF,PRBF,TQF,TMF,PRVh,TRBF,tmfef,".TKF"	; Timer_F trigger
;
		ldtm	TRB0,"LDT0"				; Timer_0 reload from wdbuf
		ldtm	TRB1,"LDT1"				; Timer_1 reload from wdbuf
		ldtm	TRB2,"LDT2"				; Timer_2 reload from wdbuf
		ldtm	TRB3,"LDT3"				; Timer_3 reload from wdbuf
		ldtm	TRB4,"LDT4"				; Timer_4 reload from wdbuf
		ldtm	TRB5,"LDT5"				; Timer_5 reload from wdbuf
		ldtm	TRB6,"LDT6"				; Timer_6 reload from wdbuf
		ldtm	TRB7,"LDT7"				; Timer_7 reload from wdbuf

		ldtm	TRB8,"LDT8"				; Timer_8 reload from wdbuf
		ldtm	TRB9,"LDT9"				; Timer_9 reload from wdbuf
		ldtm	TRBA,"LDTA"				; Timer_A reload from wdbuf
		ldtm	TRBB,"LDTB"				; Timer_B reload from wdbuf
		ldtm	TRBC,"LDTC"				; Timer_C reload from wdbuf
		ldtm	TRBD,"LDTD"				; Timer_D reload from wdbuf
		ldtm	TRBE,"LDTE"				; Timer_E reload from wdbuf
		ldtm	TRBF,"LDTF"				; Timer_F reload from wdbuf
;
; --- Fill and clear the bit stack ---		(OK)
;
fbs:
		.db		"F.",0,ctlW		; Fill bit stack
		.dw		fbse			; Link address to next word
		.dw		NORM			; Editor mode only
fbsL:
		.dw		(fbs - fbsL)	; Offset to namestring start
;
		ldi		rmp,0xff
		mov		BSTK,rmp
		ret
fbse:
;
zbs:
		.db		"Z.",0,ctlW		; Zero the bit stack
		.dw		zbse			; Link address to next word
		.dw		NORM			; Interpreter and runtime
zbsL:
		.dw		(zbs-zbsL)		; Offset to namestring start
;
		clr		BSTK
		ret
zbse:
;
; --- Print PA byte ---		(OK)
;
ppa:
		.db		"PPA",ctlW
		.dw		ppae
		.dw		NORM
ppaL:
		.dw		(ppa-ppaL)
;
ippa:
		sbrs	flagb,paceb			; Display every 100 ms
		ret
		cbr		flagb,(1<<paceb)	; Clear paceb flag
		call	pxy
		.db		6,57
		lds		rgb,pa
		rcall	prbyt				; Show 8 bits
		ret
ppae:
;
; --- Print PU byte ---		(OK)
;
ppu:
		.db		"PPU",ctlW
		.dw		ppue
		.dw		NORM
ppuL:
		.dw		(ppu-ppuL)
;
ippu:
		sbrs	flagb,paceb			; Display every 100 ms
		ret
		cbr		flagb,(1<<paceb)	; Clear paceb flag
		call	pxy
		.db		6,57
		lds		rgb,pu
		rcall	prbyt				; Show 8 bits
		ret
ppue:
;
; --- Print PV byte ---		(OK)
;
ppv:
		.db		"PPV",ctlW
		.dw		ppve
		.dw		NORM
ppvL:
		.dw		(ppv-ppvL)
;
ippv:
		sbrs	flagb,paceb			; Display every 100 ms
		ret
		cbr		flagb,(1<<paceb)	; Clear paceb flag
		call	pxy
		.db		6,57
		lds		rgb,pv
		rcall	prbyt				; Show 8 bits
		ret
ppve:
;
; --- Print PY byte ---		(OK)
;
ppy:
		.db		"PPY",ctlW
		.dw		ppye
		.dw		NORM
ppyL:
		.dw		(ppy-ppyL)
;
ippy:
		sbrs	flagb,paceb			; Display every 100 ms
		ret
		cbr		flagb,(1<<paceb)	; Clear paceb flag
		call	pxy
		.db		6,57
		lds		rgb,py
		rcall	prbyt				; Show 8 bits
		ret
ppye:
;
; --- No Operation ---		(OK)
;
nopw:	.db		"NOP",ctlW		; No operation
		.dw		nope			; Link address to next word
		.dw		NORM			; Interpreter and runtime
nopL:
		.dw		(nopw - nopL)	; Offset to namestring start
;
inop:
		ret
nope:
;
; --- Fill UFx space with NOPs ---		(OK)
;
fill:
		.db		"FILL",0,ctlW
		.dw		fille
		.dw		EDIT			; Editor mode only
fillL:
		.dw		(fill-fillL)	; Offset to namestring start
;
ifill:
		ldi		rgv,0x0			; UF0 is (0x100) 256 words
		ldi		YH,high(uf0st)
		ldi		YL,low(uf0st)	; Y <-- UF0ST
		sts		pointr,YH
		sts		pointr+1,YL		; (POINTR) <-- Y
		ldxptr	uf0st			; UF0 buffer start
		ldi		ZH,high(inop)	; NOP's IWRD address
		ldi		ZL,low(inop)
fill1:
		rcall	putwd			; Load NOP to UF0 @(PONITR)
		dec		rgv
		brne	fill1
		call	inzuf0			; Restart editor at line 0
		rcall	dpage
		ret
fille:
;
; --- Print out Bit Stack contents ---		(OK)
;
bss:	.db		"PBS",ctlW		; Non-destructive BSTK print
		.dw		bse				; Link address to next word
		.dw		NORM			; Interpreter and runtime
bssL:	.dw		(bss - bssL)	; Offset to namestring start
;
ibs:
		sbrs	flagb,paceb		; Display every 100 ms
		ret
		cbr		flagb,(1<<paceb)	; Clear paceb flag
		call	pxy
		.db		6,8
		mov		rgb,BSTK		; Get bit stack
prbyt:
		ldi		rgv,8			; Load counter, 8 bits/byte
bs2:
		rol		rgb				; Pop TOS
		call	space
		ldi		rga,'1'
		brcs	bs3				; C = 1
		ldi		rga,'0'
bs3:
		call	co
		dec		rgv				; Count the bits
		brne	bs2				; Repeat
		ret
bse:
;
; --- Breakout key ---		(OK)
;
key:
		.db		"KEY",ctlW
		.dw		keye
		.dw		NORM
keyl:
		.dw		(key-keyL)
;
ikey:
		call	getc			; Check for keyboard input
		tst		rga
		brne	key1
		ret
;
key1:
		cpi		rga,'/'
		breq	key2
		ret
;
key2:
		call	sak
		cli							; Disable GI
		ldi		rmp, HIGH(RAMEND)	; Init MSB stack
		out		SPH, rmp
		ldi		rmp, LOW(RAMEND)	; Init LSB stack
		out		SPL, rmp
		sei							; Enable GI
		rjmp	cml					; Restart command processor
keye:
;
; --- Run user file ---		(OK)
;
; The main interpreter's command to run the subroutines stored in
; the User File area in RAM. An iend vector is automatically loaded
; at the cursor position where the run command is entered. Run then
; enters a tight execution loop and begins executing IWRD vectors
; in UF0 space, beginning at the first vector at (ufpcH:ufpclo) = UF0ST.
; (ufpcH:ufpclo) is incremented to point to the next IWRD vector.
; Execution continues until the iend vewctor is encountered. The iend
; routine resets (ufpcH:ufpclo) = UF0ST, which causes execution to
; begin again at the first IWRD vector stored in UF0.
;
		.db		"RUN",ctlW
		.dw		rune
		.dw		EDIT			; Editor mode only
		.dw		0
;
irun:
		rcall	ufcsr			; (POINTR) <--- @(CURSOR)
		ldzptr	iend			; iend <-- Z
		lsr		ZH
		ror		ZL				; Z/2 for vector address
		rcall	putwd			; Insert end command
irun1:
		call	slinb			; Cursor to status line
		ldzptr	runm			; Show running message
		call	pptr
;
		cli							; Disable GI
		ldi		rmp, HIGH(RAMEND)	; Init MSB stack
		out		SPH, rmp
		ldi		rmp, LOW(RAMEND)	; Init LSB stack
		out		SPL, rmp
		sei							; Enable GI
;
; The execution loop is written as tight as possible for maximum
; speed when running UF program. ufpch:ufpcl is the UF program counter,
; incremented each pass through loop. ufpch:ufpcl can be modified by an
; IWRD. For example, the iend command, which is mandatory at the end
; of a user's program, reloads ufpch;ufpcl with UF0ST address. This causes
; the program to loop back to the start and repeat again.
;
irun2:
		mov		XH,ufpch	; Pointer to UF0 vector
		mov		XL,ufpcl
		ld		ZH,X+		; Vector to Z
		ld		ZL,X+
;
		mov		ufpch,XH	; Update to next vector
		mov		ufpcl,XL
;
		icall				; Execute IWRD @(ufpch:ufpcl)
		rjmp	irun2		; Loop again
;
rune:
;
; --- Execute UF0 program ---		(OK)
;
exec:
		.db		"EX0",ctlW
		.dw		ex0x
		.dw		EDIT			; Editor mode only
		.dw		0
;
iex0:
		rcall	iuf0			; Initialize UF0 buffers
		call	zbuf			; Initialize RAM buffers except UF
								; amd TRBn timer reload buffersmake
		rjmp	irun1
ex0x:
;
; --- Show Help Screen ---		(OK)
;
help:
		.db		"?",ctlW
		.dw		helpe
		.dw		EDIT			; Editor mode only
		.dw		0
;
ihelp:
		call	clean
		ldzptr	hlpm		; Print help screen
		call	pptr
		call	sak
		call	clean
		rcall	dpage
		ret
helpe:
;
; --- Restart in Edit mode ---
;
restr:
		.db		"/",ctlW
		.dw		restre
		.dw		EDIT			; Editor mode only
		.dw		0
;
irestr:
		cli							; Disable GI
		ldi		rmp, HIGH(RAMEND)	; Init MSB stack
		out		SPH, rmp
		ldi		rmp, LOW(RAMEND)	; Init LSB stack
		out		SPL, rmp
		sei							; Enable GI
		rjmp	cml
restre:
;
; --- Setup User File 0 ---
;
uf0:
		.db		"UF0",ctlW		; Setup User File 0 for execution
		.dw		uf0e
		.dw		EDIT			; Editor mode only
		.dw		0
;
iuf0:
		call	inzuf0
		call	pxy
		.db		24,2
		call	ceol
		ldzptr	uf0m
		call	pptr
		ret
uf0e:
;
; Insert a NOP at the current cursor position. Move all instructions	(OK)
; in the User File down 1 location from the current cursor position.
; The last instruction in file is lost.
;
; Registers: X, Y, Z, DE (YH:YL), count
;
; Source/Destination (S, S+1) <-- X
; (Z) = S+1 image, (Y) = S image
; count = words to move, (POINTR - UF0EN)
;
ins:
		.db		"INS",ctlW
		.dw		inse
		.dw		EDIT
		.dw		0
;
iins:
		rcall	ufcsr			; (POINTR) <-- UF0 @(cursor line), address in UF0
		lds		XH,pointr
		lds		XL,pointr+1		; Current Source location, S <-- X
;
		ldi		YH,high(uf0en)
		ldi		YL,low(uf0en)
		sub		YL,XL
		sbc		YH,XH
		lsr		YH
		ror		YL				; Y/2 for word count
		mov		count,YL		; Counter for words to move
;
; Preload Y buffer with IWRD at cursor, i.e. S location in UF0
;
		ld		YH,X+			; Preload (Y) <-- @(S)
		ld		YL,X+			; S+1 <-- X
;
ins1:
		ld		ZH,X+			; Z <-- @(S+1)
		ld		ZL,X			; Z = Z+1
;
		ldi		rmp,-1
		add		XL,rmp
		ldi		rmp,0xff
		adc		XH,rmp			; S+1 <-- X
;
		st		X+,YH			; S+1 <-- (Y)
		st		X+,YL			; Next word <-- X
;
		mov		YH,ZH
		mov		YL,ZL			; Copy Z to Y
;
		dec		count
		brne	ins1
;
		ldi		ZH,high(inop)	; NOP's IWRD address
		ldi		ZL,low(inop)
		rcall	putwd			; @(cursor) <-- inop
		rcall	dpage
		ret
inse:
;
; Delete the instruction at the current cursor position. Move all		(OK)
; instructions in the User File up 1 location to the current cursor
; position @POINTR. A NOP instruction is inserted in the last
; location of the current User File.
;
delet:
dele:
		.db		"DEL",ctlW
		.dw		delee
		.dw		EDIT
		.dw		0
idele:
		rcall	ufcsr			; (POINTR) <-- UF0 @(cursor line), address in UF0
		lds		XH,pointr
		lds		XL,pointr+1		; Current Source location, S <-- X
;
		ldi		YH,high(uf0en)
		ldi		YL,low(uf0en)
		sub		YL,XL
		sbc		YH,XH
		lsr		YH
		ror		YL				; Y/2 for word count
		mov		count,YL		; Counter for words to move
;
		mov		YH,XH			; Copy X to Y
		mov		YL,XL			; S <-- X
		adiw	YL,2				; S+1 <-- Y
;
dele1:
		ld		ZH,Y+			; Z <-- @(S+1)
		ld		ZL,Y+			; Z = Z+1
		st		X+,ZH
		st		X+,ZL
		dec		count
		brne	dele1
;
; Place a NOP at X
;
		ldi		ZH,high(inop)	; NOP's IWRD address
		ldi		ZL,low(inop)
		st		X+,ZH			; UFx <-- Y
		st		X+,ZL
		rcall	dpage
		ret
delee:
;
; Print current program version
;
pver:
		.db		"VER",ctlW
		.dw		pverx
		.dw		EDIT
pverL:
		.dw		(pver-pverL)
;
ipver:
		call	slinb
		ldzptr	verm
		call	pptr
		sbr		flaga,(1<<xclinf)	; Delayed line clear ###
		ret
pverx:
;
;************************************************
;
; Store and load UF and Timer settings to EEPROM
;
;************************************************
;
; Registers: rmp, rga, Y, X, Z
;
stp:
		.db		"STORE",ctlW
		.dw		stpx
		.dw		EDIT
stpL:
		.dw		(stp-stpL)
istp:
		call	slinb						; Show storing message
		ldzptr	storem
		call	pptr
;
		ldi		rmp,0xff
		out		EEARL,rmp					; EEPROM start address = -1 as
		out		EEARH,rmp					;  EEAR is pre-decremented
		ldi		rmp,high(filend-uf0st)		; Buffer size byte counter
		mov		XH,rmp
		ldi		rmp,low(filend-uf0st)		; Buffer size byte counter
		mov		XL,rmp						; X = bytes to move
;
		ldi		ZH,high(uf0st)				; UF0 <-- Z, source
		ldi		ZL,low(uf0st)
;
stp1:
		ld		rmp,Z+			; Get source byte from UF0
		call	EEWrSeq			; Write to EEPROM
;
		ldi		rga,1			; X = X-1
		sub		XL,rga
		clr		rga
		sbc		XH,rga
;
		tst		XH				; High counts 0?
		brne	stp1			;	No, loop
		tst		XL				; Low counts 0?
		brne	stp1			;	No, loop
;
		call	sak
		ret
stpx:
;
; Load EEPROM UF0 image to UF0 space
;
ldp:
		.db		"LOAD",0,ctlW
		.dw		ldpx
		.dw		EDIT
ldpL:
		.dw		(ldp-ldpL)
ildp:
		ldi		rmp,0xff
		out		EEARL,rmp					; EEPROM start address = -1
		out		EEARH,rmp
		ldi		rmp,high(filend-uf0st)		; Buffer size byte counter
		mov		XH,rmp
		ldi		rmp,low(filend-uf0st)		; Buffer size byte counter
		mov		XL,rmp						; X = bytes to move
;
		ldi		ZH,high(uf0st)				; UF0 <-- Z, destination
		ldi		ZL,low(uf0st)
;
ldp1:
		call	EERdSeq			; Get source byte from EEPROM
		st		Z+,rmp			; Write to UF0 buffer
;
		ldi		rga,1			; X = X-1
		sub		XL,rga
		clr		rga
		sbc		XH,rga
;
		tst		XH				; High counts 0?
		brne	ldp1			;	No, loop
		tst		XL				; Low counts 0?
		brne	ldp1			;	No, loop
;
		call	inzuf0			; Restart editor at line 0
		rcall	dpage
		ret
ldpx:
;
;###########################################################################
;
;
; Load one-shot timer re-load buffers TRB0..7 & TRB8..F
;
ldt:
		.db		"LDT",ctlW
		.dw		ldtx
		.dw		EDIT
ldtL:
		.dw		(ldt-ldtL)
;
ildt:
		rcall	slinb			; Position on status line
		ldzptr	ldtmm			; Show LDT command message
		call	pptr
;
		lds		rmp,offs		; Get editor's cursor
		push	rmp				; Save offs
		clr		rmp				; Start at timer 0
		sts		offs,rmp		; offs for timer settings
		call	clean			; Clean up display area
		call	prthdr			; Print headers
		call	prldt			; Print LDT command message, ceol
		clr		rmp
		sts		offs,rmp		; (offs) <-- TON0 buffer
		call	prtbs			; Display timer re-load buffer settings
		rcall	prthi			; Hilight the first buffer
ldt0:
		call	enter			; Enter prompt
ldt1:
		call	glin			; Get timer value
		sbrs	flaga,escf
		rjmp	ldt1a			; Skip arrow commands if not ESC
;
; Process Left/Right cursor keys
;
;       Left arrow        ESC [ D   -->   Previous timer
;       Right arrow       ESC [ C   -->   Next timer
;
ldtrt:
		call	ci				; Else get '[' character after the ESC
		cpi		rga,'['			; '['?
		brne	ldt0			; No, redo enter line
		call	ci				; Process cursor keys
		cpi		rga,'C'			; -> key?
		brne	ldtlf
		rcall	prtlo			; Hilight off
		rcall	nxbuf			; Next timer
		rcall	prthi			; Hilight on
		rjmp	ldt0
ldtlf:
		cpi		rga,'D'			; <- key?
		brne	ldt1b
		rcall	prtlo			; Hilight off
		rcall	pvbuf			; Previous timer
		rcall	prthi			; Hilight on
		rjmp	ldt0
;
ldt1b:
		cpi		rga,'A'
		brne	ldt1c
		rjmp	ldt0
;
ldt1c:
		cpi		rga,'B'
		brne	ldt1d
		rjmp	ldt0
;
ldt1d:
		call	ci				; eat the '~' on INS and DEL
		rjmp	ldt0
;
; Non-cursor entry, check for Z key by checking linbuf(0) character
;
ldt1a:
		ldxptr	linbuf				; Point to linbuf
		ld		rga,X				; Get a linbuf(0) character
		call	case				; Fold to UC
		cpi		rga,'Z'				; Z key?
		brne	ldt2				;	No, continue
		rcall	zdt					;	Yes, clear timer buffers
;
; Update buffer display
;
		rcall	prtbs				; Show timer buffers
		rcall	prthi				; Hilight buffer
		rjmp	ldt0				; Next entry
;
; Ready to check for new timer setting entry.
; Check for valid HH:MM input and load new timer setting if found
;
ldt2:
		call	gchr				; Process line input character, X+
		tst		rga					; Line empty?
		breq	ldte				;	Yes, exit
ldt3:
		call	gnum				; Get possible number, set numfl if good
		sbrs	flaga,numfl			; numfl set?
		rjmp	ldterr				;	No, input error, get new input
		cbr		flaga,(1<<numfl)	;	Yes, clear numfl, valid entry
;
ldt4:
		sbrs	flaga,xclinf		; Keep status line?
		rjmp	ldt5				; Yes, leave status line
		cbr		flaga,(1<<xclinf)
		call	prldt				; Show LDT command message, erase rest of line
;
; Got good entry, load timer and show new entry
;
ldt5:
		call	pdbuf				; Show new timer reload setting
		rcall	ldtb				; load timer buffer @(offs) <-- DBA
		rcall	prbuf				; Show new buffer entry
		rcall	nxbuf				; Move to next buffer
		rcall	prthi				; Show buffer highlight
		rjmp	ldt0				; Get next buffer entry
;
; Input error handler
;
ldterr:
		call	pxy					; Position cursor on status line
		.db		24,45
		ldzptr	err1				; Input error message
		call	pptr
		sbr		flaga,(1<<xclinf)	; Delayed line clear
		rjmp	ldt0				; Redo input
;
; Exit back to editor
;
ldte:
		call	pxy					; Blank line found, exit
		.db		22,6
		call	ceol				; Clear enter prompt line
		pop		rmp
		sts		offs,rmp			; Restore editor's offs
		call	clean				; Clean up display area
		call	dpage				; Refresh program page
		call	pruf0				;	UF1 selected, switch to UF1
		ret
;
ldtx:
;
;###########################################################################
;
; Gated 1 s tick counter CNTb enabled by input gate signal from BSTK
;
;   Enable <tbase_1s:> sets t1sf every 1 s 
;   Increment CNTb every 1 s to max value of 0xff

;   	If CNTb = 0, exit
;   	else
;       	clear t1sf
;       	TM1sb <== 10
;       	clear CNTb, exit
;
cnt:
		.db		".CNT",0,ctlW
		.dw		cntx
		.dw		NORM
cntL:
		.dw		(cnt-cntL)
;
icnt:
		lsr		BSTK			; Pop TOS to C, 0 moved to BSTK bit 7
		brcs	lab0
;
; Gate signal is low
;
        cbr     flagb,(1<<t1sf)  ; clear tick flag
		clr		rga
		sts		CNTb,rga
        ldi     rga,10			; Reload 1 s prescaler
		sts		TM1sb,rga
		ret
;
; Gate signal is high, count 1 s ticks
;
lab0:
; ---
		lds		rga,CNTb     	; Get counter value
		brne	lab1			; 
		rcall	lab2			; Show wdbuf if 0
lab1:
        sbrs    flagb,t1sf  	; Tick flag?
        ret       				;  no, exit
        cbr     flagb,(1<<t1sf)  ; Yes, clear tick flag
        lds     rga,CNTb     	; Get counter value
        inc     rga         	; Count 1 s
		sts		CNTb,rga		; Update counter
		sbr		flagb,(1<<drdyf) ; Data ready in wdbuf

;
; Show accumulated counts
;
lab2:
		clr		rga
		sts		wdbuf,rga	; Clear hi byte
		lds		rga,CNTb
		sts		wdbuf+1,rga
		call	pdbuf
; ---
		ret
cntx:
;
;
;###########################################################################
;
; Toggle on input bit rising edge
;
		togg	0,tg_old,tg_out,".TG0"		; Toggle 0
		togg	1,tg_old,tg_out,".TG1"		; Toggle 1
		togg	2,tg_old,tg_out,".TG2"		; Toggle 2
		togg	3,tg_old,tg_out,".TG3"		; Toggle 3
		togg	4,tg_old,tg_out,".TG4"		; Toggle 4
		togg	5,tg_old,tg_out,".TG5"		; Toggle 5
		togg	6,tg_old,tg_out,".TG6"		; Toggle 6
		togg	7,tg_old,tg_out,".TG7"		; Toggle 7
;
; Get Toggle output bits
;
		gvbit	0,tg_out,"TGQ0."	; Get TGQ0 output 
		gvbit	1,tg_out,"TGQ1."	; Get TGQ1 output 
		gvbit	2,tg_out,"TGQ2."	; Get TGQ2 output 
		gvbit	3,tg_out,"TGQ3."	; Get TGQ3 output 
		gvbit	4,tg_out,"TGQ4."	; Get TGQ4 output 
		gvbit	5,tg_out,"TGQ5."	; Get TGQ5 output 
		gvbit	6,tg_out,"TGQ6."	; Get TGQ6 output 
		gvbit	7,tg_out,"TGQ7."	; Get TGQ7 output 

;
;
;
;###########################################################################
;
; File Transfer Commands
;
;
.include		"file-transfer.asm"
;
;
;###########################################################################
;
.include		"list-uf0.asm"
;
;###########################################################################
;
;
; --- Last Word in Program ---
;
end:
		.db		"END",ctlW
		.dw		ende
		.dw		NORM
endL:
		.dw		(end-endL)
;
iend:
		ldi		rmp,high(uf0st)		; Reset ufpch:ufpcl to UF0 start
		mov		ufpch,rmp
		ldi		rmp,low(uf0st)
		mov		ufpcl,rmp
;
; --- Set/Reset Flip Flops ---
;
;  Set/Reset Flip Flops using three virtual bytes for 16 SR FFs.
;  Bytes SFF, RFF and QFF holds the Sn, Rn and Qn control bits
;  for the FFs. All 8 FFs are processsed in parallel. Control
;  words .Sn, .Rn and Qn. access individual FF bits. Reset takes
;  precedence over Set.
;
rsff:
		lds		rga,vqff_0				; Get current Q state
		lds		rmp,vsff_0				; Get current S state
		or		rga,rmp					; rga <-- Q + S
		sts		vqff_0,rga				; Q <-- (Q + S)
		lds		rmp,vrff_0				; Get current R state
		com		rmp						; rmp = !R
		and		rmp,rga					; rmp = !R & (Q + S)
		sts		vqff_0,rmp				; Q <-- (Q + S)
;
		lds		rga,vqff_1				; Get current Q state
		lds		rmp,vsff_1				; Get current S state
		or		rga,rmp					; rga <-- Q + S
		sts		vqff_1,rga				; Q <-- (Q + S)
		lds		rmp,vrff_1				; Get current R state
		com		rmp						; rmp = !R
		and		rmp,rga					; rmp = !R & (Q + S)
		sts		vqff_1,rmp				; Q <-- (Q + S)
;
		ret
ende:
;
; ---
;

dicte:
		.db		ctlZ,0			; Dictionary end marker
;
; ---
;
; --- Autostart routine ---
;
autost:
		sbic	PINB,PB0		; PB0 - 0?
		jmp		autostx			; Exit if PB0=1
;
		ldi		rmp,0xff
		out		EEARL,rmp		; EEPROM start address = -1
		out		EEARH,rmp
		rcall	EERdSeq			; Get first source byte from EEPROM in rmp
		cpi		rmp,0xff		; Is it erased?
		breq	autostx			;	Yes, exit
;
		call	ildp			;	No, load EEPROM buffer to UF0 space
		call	iuf0			;	No, execute UF0 program
		jmp		irun1
;
autostx:
		ret
;
;
;###########################################################################
;
;
;
; --- LDT Timer Setting sub-routines ---
;
; Go to next timer buffer
;
nxbuf:
		lds		rmp,offs		; Get offs
		inc		rmp				; Increment offs to next timer
		cpi		rmp,ntim		; offs = ntim?
		brne	nxbuf1			;	No, done
		clr		rmp				;	Yes, reset to 0
nxbuf1:
		sts		offs,rmp
		ret
;
; Go to previous timer buffer
;
pvbuf:
		lds		rmp,offs
		dec		rmp				; Decrement offs to previous timer
		cpi		rmp,0xff		; offs = -1?
		brne	pvbuf1			;	No
		ldi		rmp,(ntim-1)	;	Yes, go to buffer 7
pvbuf1:
		sts		offs,rmp
		ret
;
; Display 2 lines of 8 TRBn buffers per line
;
prtbs:
		ldi		rgv,ntim		; Show ntim re-load timer buffers
		clr		rmp
		sts		offs,rmp		; offs = 0, start at buffer 0
prtbs1:
		push	rgv				; Save counter
		rcall	prbuf			; Print a timer buffer
		rcall	nxbuf			; Do next buffer
		pop		rgv				; Get counter
		dec		rgv				; Count buffers shown
		brne	prtbs1			; Not done, do more
		clr		rmp
		sts		offs,rmp		; offs = 0, timer buffer base
		ret
;
; --- Show timer re-load buffer ---
;
;  Position cursor and show timer re-load buffer @(OFFS). OFFS contains
;  offset value that controls the position of the cursor as well as the
;  offset to the timer re-load buffer base address TRB0.
;
;  Entry:       OFFS <-- (0...15) offset address
;
;  Buffer base address set at TRB0, initial x,y position  XINIT
;  and YINIT.
;
;
prbuf:
		ldxptr	TRB0			; Base of timer re-load buffers
		ldi		YL,xinit		; Initial horizontal offset
		ldi		YH,yinit		; Initial vertical offset
		clr		count			; Column counter
		lds		rga,offs		; Get offset and process column offset
		tst		rga				; offs = 0?
		breq	prbuf1			;	Yes, use buffer base address
;
prbuf2:
		ldi		rmp,dx1			; C = 0, add buffer spacing
		add		YL,rmp
		adiw	XL,1				; Increment buffer pointer 1 bytes to next buffer
		inc		count			; Count columns moved
		mov		rmp,count
		cpi		rmp,8			; count = 8?
		brne	prbuf4			;	No, continue
		ldi		YL,xinit		; Reset X position
		ldi		rmp,dy			; Move to second row position
		add		YH,rmp
;
prbuf4:	dec		rga				; Count offs corrections
		brne	prbuf2
;
; Arrive here with XH:XL pointing to proper timer re-load buffer and cursor
; position in YH:YL. Show timer reload buffer contents.
;
prbuf1:
		call	gotoxy			; Move cursor to proper position
		ld		rga,X+			; Pick up buffer contents
		mov		YL,rga
		clr		YH				; YH = 0
		call	bn2bcd			; Convert to packed BCD
		call	p3dg			; Show timer setting
		ret
;
; Highlight current buffer pointed to by OFFS.
;
prthi:
		call	vrev			; Reverse attribute on
prthi1:
		call	prbuf			; Show timer buffer
		call	vlo				; Normal on
		ret
;
; Show current buffer as normal.
;
prtlo:
		call	vlo				; Normal on
		rjmp	prthi1
;
; Store timer re-load setting in DBA to buffesr TRB0@(offs)
;
ldtb:
		ldxptr	TRB0			; Base of timer re-load buffers
		lds		rmp,offs		; Get offs
		tst		rmp				; offs = 0?
		breq	ldtb2			;	Yes, store at TRB0
;
; Indexed store of TRBn@(offs) <-- (dba)
;
ldtb1:
		adiw	XL,1				;	No, increment to next timer re-load buffer
		dec		rmp				; Process offs count
		brne	ldtb1
ldtb2:
		lds		rmp,dba			; Get new setting
		st		X+,rmp			; TRBn buffer <-- (dba)
		ret
;
; Display timer re-load buffer headers
;
prthdr:
		call	pxy
		.db		9,15
		ldzptr	thdr1			; Buffers 0 to 7
		call	pptr
;
		call	pxy
		.db		10,15
		ldzptr	thdr3			; Separator line
		call	pptr
;
		call	pxy
		.db		14,15
		ldzptr	thdr2			; Buffers 8 to 15
		call	pptr
;
		call	pxy
		.db		15,15
		ldzptr	thdr3			; Separator line
		call	pptr
		ret
;
; Print LDT command message, clear rest of line
;
prldt:
		call	slina
		ldzptr	ldtmm			; Show LDT command message
		call	pptr
		call	ceol			; Clear to eol
		ret
;
; Clear delay timer reload buffers
;
zdt:
		ldxptr	TRB0				; Base address of timer reload buffers
		ldi		rgb,(TCB0-TRB0)		; Byte counter
		clr		rmp					; Fill byte
zdt0:
		st		X+,rmp				; Zero timer buffers
		dec		rgb
		brne	zdt0
		ret
;
;
;###########################################################################
;
;
; Controller startup and initialization
;
start:
;
; Initialize the stack pointer to end of SRAM
;
		ldi		rmp,high(RAMEND)	; Init MSB stack
		out		SPH,rmp
		ldi		rmp,low(RAMEND)	; Init LSB stack
		out		SPL,rmp
;
; Initialize the engine
;
		call	wdt_off			; Disable watchdog. Must be done soon after a reset
		call	initz			; Initialize engine
		call	inzuart			; Initialize the UART
;
; Check if autostart enabled D8/PB0 tied low).
;
		call	autost
;
;
;###########################################################################
;
;
; --- Command Line Input Processor ---
;
;  Process command line
;
cml:
		rcall	prscn			; Print command screen
		rcall	iuf0			; Setup control buffers for User File 0
		rcall	dpage			; Display page 0
;
;  Main processor
;
cml1:
		rcall	ibs				; Print bit stack
		rcall	pdbuf			; Print data entry
cml2:
		rcall	enter			; 'Enter: ' prompt
cml2a:
		rcall	glin			; User line input
		sbrs	flaga,escf
		rjmp	optn5			; Skip arrow commands if not ESC
;
;  Process escape sequences i.e. cursor and Home key commands
;  The cursor or arrow key sequences are generated when the
;  appropriate keys are pressed, and are processed by the following
;  modules. Note these are ANSI sequences.
;
;       Up arrow        ESC [ A   -->   Previous Line,	(^W key)
;       Down  "         ESC [ B   -->   Next Line,		(^Z key)
;       Left  "         ESC [ D   -->   Previous Page,	(^A key)
;       Right "         ESC [ C   -->   Next Page,		(^S key)
;       Home key        ESC [ 1 ~ -->   Jump to line #, ESC H for kermit
;		INS				ESC [ 2 ~ -->	INS command
;		DEL				ESC [ 3 ~ -->	DEL command
;
optn0:
		rcall	ci				; Else get '[' character after the ESC
		cpi		rga,'['			; '['?
		brne	optn4a			; No, Check if kermit Home string
		rcall	ci				; Process cursor keys
		cpi		rga,'B'			; DN key
		brne	optn1
		rcall	nextl			; Down a line
		rjmp	cml2
;
optn1:
		cpi		rga,'A'			; UP key
		brne	optn2
		rcall	pvln			; Up a line
		rjmp	cml2
optn2:
		cpi		rga,'C'			; --> key
		brne	optn3
		rcall	nxpg			; Next page
		rcall	dpage			; Dislay page
		rjmp	cml2
optn3:
		cpi		rga,'D'			; <-- key
		brne	optn4a
		rcall	pvpg			; Previous page
		rcall	dpage			; Dislay page
		rjmp	cml2
;
; Process Home key string from kermit
;
optn4a:
		cpi		rga,'H'			; 'H' ?
		brne	optn4			; No, check alternate string
		rcall	jump			; Jump to line number entered
		rjmp	cml2
;
; Process Home key string from non-kermit
;
optn4:
		cpi		rga,'1'			; Home key
		brne	optn7
		rcall	ci				; Eat extra '~' character
		rcall	jump			; Jump to line number entered
		rjmp	cml2
;
; Process aliases for INS and DEL commands
;
optn7:
		cpi		rga,'2'			; INS key
		brne	optn8
		rcall	ci				; Eat extra '~' character

		rcall	iins			; Execute INS command
		rjmp	cml2
optn8:
		cpi		rga,'3'			; DEL key
		brne	optn5
		rcall	ci				; Eat extra '~' character

		rcall	idele			; Execute DEL command
		rjmp	cml2
;
;  Arrive here if input was not a valid escape sequence.
;
optn5:
		sbrs	flaga,xclinf	; Keep status line?
		rjmp	optn6			; Yes, leave status line
		cbr		flaga,(1<<xclinf)
		rcall	pruf0			; Show File 0, erase rest of line
optn6:
		rcall	gchr			; Process line input
		tst		rga				; Line empty?
		breq	cml2			; Get new enter line
		rcall	decdg			; Test for 0....9
		brcc	pars			; No decimal digit, process line for a word entry
; ---
; Check next char in buffer to filter strings starting with a digit.
;
		ld		rga,x			; Get next character
		tst		rga				; nothing else?
		breq	optn6a			; yes, nothing there
		rcall	decdg			; check if another digit
		brcc	pars			; No, no digit
optn6a:
; ---
		rcall	gnum			; Get possible number, set numfl if good
;
		sbrs	flaga,numfl		; numfl set?
		rjmp	errin			;	No, input error
		cbr		flaga,(1<<numfl)	; Clear numfl
		rjmp	cml1
;
;  Search Dictionary for matching opcode words of length 1...5,
;  and return with C=1, A=0FFH and (opcode address) <-- Z if
;  string found. If no string found C=0, A=0 and ^Z <-- Z at end
;  of dictionary.
;
pars:
		mov		rmp,count		; String length
		cpi		rmp,opclen		; Length ok?
		brcc	errin			; No, unrecognized input
;
; --- Search Dictionary ---
;
		rcall	fndst			; Look for WORD string in dictionary
		brcs	errin			; String not found error
;
; --- Execute IWRD ---
;
; A vaild WORD from dictionary found, Z points to the WORD's IWRD vector.
; Execute the IWRD by indirect rcall operation.
;
cml4:
		push	ZH				; Preserve Z across icall, required by
		push	ZL				; rcall to putwd: IWRD <-- Z
		icall					; Execute IWRD vector
		pop		ZL				; Restore Z
		pop		ZH
		rcall	ufcsr			; (POINTR) <-- UF0 @(line #)
;
; --- Process IWRD attribute --
;
		lds		rmp,attrb		; Fetch WORD's low(attrb) - hi byte not used
		cpi		rmp,EDIT		; Edit word?
		brne	cml5			;	No, load IWRD to UFx
		rjmp	cml1			;	Yes, skip loading word to UFx
;
; --- Store IWRD to UFx space ---
;
cml5:
		rcall	putwd			; Load IWRD addr to UFx
		rcall	nextl			; Advance to next line
		rjmp	cml1
;
;  Error input handler		(OK)
;
errin:
		rcall	slinb			; Position cursor
		ldzptr	err1			; Unknown input error
		rcall	pptr
		sbr		flaga,(1<<xclinf)	; Delayed line clear ###
		rjmp	cml2
;
; Load POINTR <--  UF0 @(line #). POINTR is loaded with the address in		(OK)
; UF0 space corresponding to the current cursor position (the current
; absolute line no) stored in (CURSOR) buffer.
;
; llnctr uses offs buffer contents to calculate the program line number
; given by basln + offs. The decom routine decompiles the IWRD at this line
; number.
;
ufcsr:
		rcall	csxof			; Exchange offset buffers, CURSOR and OFFS
		rcall	llnctr			; Get absolute line #, the program counter
		rcall	lpntr			; Get UFx @(OFFS)
		rcall	csxof
		ret
;
; Exchange contents of CURSOR and OFFS buffers		(OK)
;
; The offs and cursor buffer contents are exchanged by csxof routine when
; the program line number for the screen cursor needs to be calculated. In
; this case, ufcsr is called prior to calling llnctr.
;
csxof:
		lds		rmp,cursor		; Get cursor contents
		lds		rga,offs		; Get offs contents
		sts		offs,rmp
		sts		cursor,rga
		ret
;
; --- Page display sub-processors ---
;
; On entry, if basln = 0, roll-under to page 7 (basln = 224).
; Otherwise, set basln = basln-32
;
pvpg:
		lds		rga,basln+1	; Get current basln+1
		cpi		rga,0		; Page 0?
		breq	pvpg1		;	Yes, roll-under to page 7
		ldi		rmp,-32		; rga = basln - 32
		add		rga,rmp
		sts		basln+1,rga
		ret
pvpg1:
		ldi		rga,224		; Page 7
		sts		basln+1,rga
		ret
;
; On entry, if basln = 224, roll over to page 0 (basln = 0).
; Otherwise, set basln = basln+32
nxpg:
		lds		rga,basln+1	; Get current basln+1
		cpi		rga,224		; Page 7?
		breq	nxpg1		;	Yes, roll-over to page 0
		ldi		rmp,32		; rga = basln + 32
		add		rga,rmp
		sts		basln+1,rga
		ret
nxpg1:
		clr		rga			; Page 0
		sts		basln+1,rga	; Update base page buffer
		ret
;
;  Move cursor to next line		(OK)
;
nextl:
		rcall	csrlo			; Restore cursor line normal
		lds		rmp,cursor
		inc		rmp				; Next line
		sts		cursor,rmp
;
		cpi		rmp,nlines		; Past last line 31?
		brne	nextl1			;	No, still same page

		clr		rmp
		sts		cursor,rmp		;	Yes, cursor = 0
		rcall	csrhi			; Hilite line
		rcall	nxpg			; Next page
		rcall	dpage			; Show page
		ret
nextl1:
		rcall	csrhi			; Hilite line
		ret
;
;  Move cursor to previous line		(OK)
;
pvln:
		rcall	csrlo			; Restore cursor line normal
		lds		rmp,cursor
		dec		rmp				; Previous line
		sts		cursor,rmp
;
		cpi		rmp,0xff		; Past last 0?
		brne	pvln1			;	No, still same page
		ldi		rmp,nlines-1
		sts		cursor,rmp		;	Yes, reset to line 0
		rcall	pvpg			; Previous page
		rcall	dpage			; Show page
pvln1:
		rcall	csrhi			; Hilite line
		ret
;
;  Jump to line number given in DBA		(OK)
;
jump:
		rcall	csrlo			; Restore cursor line
		lds		rga,dba			; Target line number
		ldi		rgb,nlines		; Divisor
		rcall	div8u			; rga/rgb = result in rga + remainder in rgb
		sts		cursor,rgb		; Remainder to cursor
		tst		rga
		breq	jump2			; Skip if page zero
		andi	rga,0b00000111	; 7 pages maximum
		ldi		YL,5			; Multiply by 32, 5 left shifts
jump1:
		lsl		rga				; Convert pages to lines
		dec		YL				; 1 page = 32 lines
		brne	jump1
;
jump2:
		mov		YL,rga			; Save page base line number
		lds		rmp,basln+1		; Get base line
		eor		rga,rmp			; Same as current page?
		breq	jump3			; Yes
		sts		basln+1,YL		; No, update to new page
		rcall	dpage			; Show page
		ret
jump3:
		rcall	csrhi			; Hilite cursor line
		ret
;
; Load (LNCTR) with absolute line number from BASLN + OFFS		(OK)
;
llnctr:
		lds		YH,basln
		lds		YL,basln+1		; Y <-- (basln) contents
		lds		rmp,offs		; rmp <-- (offs)
		add		YL,rmp
		clr		rmp
		adc		YH,rmp			; Y <-- (basln + offs)
		sts		lnctr,YH
		sts		lnctr+1,YL		; (lnctr) <-- Y
		ret
;
; Load (POINTR) with address of IWRD vector in UFx space from		(OK)
; the corresponding absolute line number. Call LLNCTR first to load
; LNCTR with absolute line number.
;
;	Entry:	(LNCTR) <-- program line number
;	Exit:	(POINTR) <-- (LNCTR)*2 + (ufpch:ufpcl)
;
lpntr:
		lds		YH,lnctr		; Y <-- (lnctr) contents
		lds		YL,lnctr+1
		lsl		YL				; Y*2 for word addressing
		rol		YH
;
		add		YL,ufpcl		; Add ufpcl low byte
		adc		YH,ufpch		; Y = lnctr*2 + ufpch:ufpcl
		sts		pointr,YH		; (pointr) <-- Y
		sts		pointr+1,YL
		ret
;
; Display one page of 32 lines of lines/code WORDS, hilite line.
;
dpage:
		call	ppage			; Show 32 lines of decompiled code
		call	csrhi			; Hilight on
		ret
;
csrhi:
		call	vrev			; Reverse on
csrhi1:
		call	csxof			; Exch OFFS and CURSOR buffers
		call	decom			; Show program line and code
		call	vlo				; Normal on
		call	csxof			; Exch OFFS and CURSOR buffers
		ret
;
csrlo:
		call	vlo
		rjmp	csrhi1
		ret
;
; Screen display routines - Line # / decompiled word namestring display		(?)
; is controlled by the base line # buffer BASLN and the offset buffer
; OFFS. The top-of-page line number is saved in BASLN buffer. Called
; by dpage.
; Print a page of 32 lines of decompiled code WORDs and line numbers
; Registers:
;
ppage:
		ldi		rgb,nlines		; 32 lines per page
		clr		rmp
		sts		offs,rmp		; offs = 0, line 0
ppage1:
		push	rgb
		rcall	decom			; Decompile a line
		lds		rmp,offs
		inc		rmp				; Next line
		sts		offs,rmp
		pop		rgb
		dec		rgb
		brne	ppage1
		clr		rmp
		sts		offs,rmp		; offs = 0
		ret
;
; Decompile and print one WORD namestring from OFFS and BASLN contents		(OK)
;
decom:
		rcall	vctran		; Position cursor
		rcall	llnctr		; Load LNCTR with absolute line number
		rcall	lpntr		; Load pointr with IWRD vector
		rcall	pgmln		; Show program line number
		rcall	valwd		; Test for valid word
		brcc	notwd		; Not valid, show dots
		rcall	ptnam		; Point to namestring
		rcall	prnam		; Show namestring
		ret
notwd:
		ldzptr	dots
		call	pptr
		ret
;
; Translate data in BASLN and OFFS to horiz and vert vector			(OK)
; and position cursor on screen. OFFS holds line count for the
; displayed 32 line page.
;
; Registers: rmp, rga, rgb, YH, YL
;
vctran:
		ldi		YL,hzri		; Initial horizontal offset
		ldi		YH,vtri		; Initial horizontal offset
		lds		rga,offs		; Get offset and process row info
		andi	rga,0b00000111	; Isolate bits 0..2, row 0..7 info
		add		YH,rga			; Add initial vertical offset
;
		lds		rga,offs		; Get offset and process column info
		andi	rga,0b00011000	; Bits 3,4 become...
		breq	vct1
		lsr		rga
		lsr		rga
		lsr		rga				; ...bits 0,1 (column 0..3 info)
		ldi		rmp,dlhzr		; Column offset
vct2:
		add		YL,rmp			; hzri + dlhzr
		dec		rga
		brne	vct2			; Add column offsets
vct1:
		call	gotoxy
		ret
;
; Check IWRD <-- (POINTR) in UFx is a valid word by looking for		(OK)
; ctrl W  namestring terminator, C=1 is valid, else C=0 if no ^W
; found. Searches backwards from IWRD to expected ^W location in
; dictionary WORD.
;
; Registers: Y, Z, rmp
;
; Entry:	IWRD <-- (POINTR), address to store IWRD in UFx
; Exit:		C=1 if ^W found in WORD vector, else C=0
;
valwd:
		lds		YH,pointr		; Y <-- (pointr) contents
		lds		YL,pointr+1		; Y has UF0 load address for IWRD vector
;
		ld		ZH,Y+			; Load Z with IWD vector from UF0
		ld		ZL,Y+
;
		ldi		rmp,4			; Z = Z - 4
		sub		ZL,rmp
		ldi		rmp,0
		sbc		ZH,rmp			; Point to namestring last word
;
		lsl		ZL
		rol		ZH				; Z*2 for memory access by Z
		ori		ZL,1			; Set ZL bit 0 for high byte selection
;
		lpm		rmp,Z			; ^W should be here if valid word
		cpi		rmp,ctlW
		sec
		breq	valwd1
		clc
valwd1:
		ret
;
; Point to 'namestring' of dictionary word.		(OK)
;
; Registers: Y, Z, rmp, rga
;
; Entry: IWRD <-- (POINTR), address of IWRD vector in UF0
; Exit:  "namestring" <-- Z*2
;
ptnam:
		lds		YH,pointr		; Y <-- (pointr) contents
		lds		YL,pointr+1		; Y has UF0 load address for IWRD vector
;
		ld		ZH,Y+			; Load Z with IWD vector from UF0
		ld		ZL,Y+
;
		ldi		rmp,1			; Z = Z-1
		sub		ZL,rmp
		ldi		rmp,0
		sbc		ZH,rmp			; Point to offset's address
;
		lsl		ZL				; Z*2 for memory access
		rol		ZH
;
		lpm		rgb,Z+			; Get offset, lo byte
		lpm		rga,Z			; Get offset, hi byte
;
		lsr		ZH				; Z/2 for real address
		ror		ZL				; offset word <-- Z
;
		add		ZL,rgb			; Z + offset
		adc		ZH,rga			; "namestring" address
;
		lsl		ZL				; Z*2 for memory access
		rol		ZH				; "namestring" <-- Z*2
;
		ret
;
; Print spaces/backspaces over old name, then newname		(OK)
;
prnam:
		push	ZH
		push	ZL
		ldzptr	spbs			; Blank-backspace string
		rcall	pptr
		pop		ZL
		pop		ZH
		rcall	pdptr			; Show "namestring",ctlW <-- Z
		ret
;
;  Print string pointed to by Z, ^W terminated		(OK)
;
pdptr:
		lpm		rga,Z+		; Get a character from memory
		cpi		rga,ctlW	; End of string marker?
		breq	pdptr1
		rcall	co
		rjmp	pdptr
pdptr1:
		ret
;
;
;###########################################################################
;
;
; --- Dictionary Search Routine ---		(OK)
;
;  Entry:	LINBUF <-- X,
;  			DICT <-- Z,
;  			count = string length of WORD
;  Exit: 	IWRD <-- Z, address of WORD's working code
;  			C=0 if string found, else C=1
;			(attrb) <-- Attribute type byte
;
;  The attribute type byte is stored in attrb buffer if the search finds
;  a matching dictionary word. The attribute byte is used to control in
;  which mode the dictionary word can be executed, i.e. interpreter, command
;  or run mode.
;
;  The input WORD string in the line buffer is compared to the WORDs
;  in the dictionary on a character-by-character basis. If a mismatch
;  is found, the input string pointer is reset and the dictionary
;  pointer is moved to the next word in the table. Searching continues
;  to the end of the dictionary.
;
fndst:
		ldzptr	dict			; Point to dictionary start
fndst1:
		ldxptr	linbuf			; Reset input string pointer
		mov		rgb,count		; Get string length
fndst2:
		lpm		rga,Z+			; Get a DICT namestring character
		mov		asav,rga		; Save character
;
; Check for '0' or ^W end markers to see if word is a short sub-string
;
		cpi		rga,NULL		; Is char a null byte?
		breq	fndst4			;	Yes, keep searching Dict
		cpi		rga,ctlW		; Is char a ^W?
		breq	fndst4a			;	Yes, keep searching Dict
;
; Check namestring character to see if it ctlZ end-of-dictionary marker
;
		ldi		rmp,ctlZ		; Dict end marker byte
		eor		rga,rmp			; Dictionary end?
		brne	fndst3			;	No, continue
		sec						;	Yes, exit, no word match
		ret						; C=0,rga=0
;
fndst3:
		ld		rga,X+			; Get linbuf char
		call	case			; Fold to UC
		eor		rga,asav		; linbuf char = Dict char?
		breq	fndst5			; Match, process next char
;
; Arrive here when mismatch occurs, move to ^W terminator
;
fndst4:
		lpm		rga,Z+			; Check for ^W terminator
		cpi		rga,ctlW		; ^W?
		brne	fndst4			;	No, keep looking until found
;
; Arrive here when ^W found. Pick up the link address of
; DICT word, put LA into Z, pointing to next DICT word.
; AVR 8 bit micros store data Little Endian style, lower order
; byte in lower order address.
;
fndst4a:
		lpm		rga,Z+			; rga <-- LA (link address) lo
		push	rga
		lpm		rga,Z+			; rga <-- LA (link address) hi
		push	rga
		pop		ZH				; ZH <-- hi(LA)
		pop		ZL				; ZL <-- lo(LA)
		lsl		ZL				; ZH:ZL*2 for memory address access
		rol		ZH
		rjmp	fndst1			; Search DICT for next word
;
; Arrive here when match found between DICT and linbuf char. Check if
; all characters in linbuf namestring processed
;
fndst5:
		dec		rgb				; Count linbuf chars processed
		brne	fndst2			; Continue matching characters

		lpm		rga,Z			; Test char after namestring
		cpi		rga,NULL		; Null byte?
		brne	fndst5a			;	No, Look for ^W
		adiw	ZL,1			;	Yes, advance past null

fndst5a:
		lpm		rga,Z+			; Check for ^W terminator
		cpi		rga,ctlW		; ^W?
		brne	fndst4			;	No, mismatch, next word
;
; Arrive here on successful match to dict namestring, Link_Address <-- Z.
;
; Exit:
;	(attrb)  <-- attribute word
;   IWRD     <-- Z
;	C=1
;
fndst6:
		adiw	ZH:ZL,2			; Advance to attribute word
		ldxptr	attrb
		lpm		rga,Z+			; Get attribute word, hi byte
		st		X+,rga			; Save it
		lpm		rga,Z+			; Get attribute word, lo byte
		st		X+,rga			; Save it, Z at offset word
;
		adiw	ZL,2				; Advance to IWRD

;
		lsr		ZH				; ZH:ZL/2 for code address
		ror		ZL				; Exit with Z = (IWRD)
		clc						; Success
		ret
;
; --- Interpreter Routines ---
;
; Z has the IWRD subroutine address to be stored in UFx.
; Load (ZH:XL) contents to UFx @(POINTR). This routine used to
; load the IWRD vector in Z to the UFx location @(POINTR), and
; incrementing POINTR to start of next storage location.
;
; Entry: Z = IWRD address
; Exit: (POINTR) = (POINTR+1)
;
putwd:
		lds		YH,pointr		; Hi then lo byte
		lds		YL,pointr+1
		st		Y+,ZH			; UFx <-- Y
		st		Y+,ZL
		sts		pointr,YH
		sts		pointr+1,YL
		ret
;
; --- Line input and initialization routines ---	(OK)
;
; An 8 byte line input buffer is supported. The buffer is initially
; cleared to zeroes, and pointed to by XH:XL. COUNT maintains a
; count of characters entered. Entry is terminated by <'CR'>, <^X>
; erases current line and starts over, and <BS> erases the previous
; character. XH:XL is reserved for use as LINBUF pointer to allow
; multiple GCHR calls.
;
;	Registers used:
;	rmp, rga, rgb, rgc, X
;
glin:
		call	inzln			; Zero the line buffer and count register
glin1:
		call	ci				; Get a character
		cpi		rga,CR			; Test if <CR>
		brne	glin2			;	No, look for next special key
		ldxptr	linbuf			;	Yes, reset linbuf pointer
		sbr		flaga,(1<<crf)	; And set CR flag
		ret
;
; Look for a ^X key, if so do a line delete
;
glin2:
		cpi		rga,ctlX		; Test if <^X>
		brne	glin3			;	No, look for next special key
		mov		rgb,count		; Load character counter
		tst		rgb				; Count = 0?
		breq	glin
glin2a:
		call	bksp			; Move cursor back one space
		dec		rgb
		brne	glin2a			; back to start
		rjmp	glin			; Restart
;
; Look for a BS key, if so do a delete character at cursor
;
glin3:
		cpi		rga,BS			; Test if backspace
		brne	glin5			;	No, look for next special key
glin3a:
		mov		rgb,count		; Load character counter
		tst		rgb				; Count = 0?
		breq	glin1			;	Yes, fetch another character
		dec		rgb
		mov		count,rgb
		call	bksp			; Move cursor back one space
		ldi		rmp,0			; Backup pointer and insert 0
		st		-X,rmp
		rjmp	glin1
;
; Look for a Tab key, if so expand tab to spaces
;
glin5:
		cpi		rga,HT			; Test if tab
		brne	glin6			;	No,  look for next special key
		ldi		rgc,7			; Temp counter
		ldi		rga,SP			; Space character
glin5a:
		call	ldlin
		dec		rgc
		brne	glin5a
		rjmp	glin1
;
; Look for a Escape key, if so set escf
;
glin6:
		cpi		rga,ESC			; Test if esc
		brne	glin7			;	No, look for other control key
		sbr		flaga,(1<<escf)	; Set esc flag
		ret
;
; Look for other control key.
;
glin7:
		rcall	fctl			; Test for other control key
		sbrs	flaga,kyf
		rjmp	glin8			;	kyf = 0
		ret						;	kyf = 1
;
; Arrive here is valid key entry
;
glin8:
		rcall	ldlin			; Load the input buffer and show
		rjmp	glin1
;
; Load character in rga to LINBUF, update pointer and character counter		(OK)
;
ldlin:
		mov		rgb,count		; Get current count
		cpi		rgb,linsz		; End of buffer?
		brne	ldlin1			;	No
		ret						;	Yes, exit
ldlin1:
		inc		rgb
		mov		count,rgb		; Update count
		st		X+,rga			; Store entered key to buffer
		call	co				; Show it
		ret
;
;  Get linbuf character, increment XH:XL pointer and set C if
;  not 'CR', else clear C, rga = 0.
;
gchr:
		ld		rga,X+			; Get character from line buffer, advance pointer
		cpi		rga,0			; Test for 0
		brne	gchr1			;	rga >= 0, means ascii printable character
		clc
		ret
gchr1:
		sec
		ret
;
; Clear input line buffer	(OK)
;
inzln:
		clr		rmp				; Fill byte
		clr		count			; Initialize count to 0
		ldi		rgb,linsz		; Buffer size
		ldxptr	linbuf			; Point to line buffer
inzln1:
		st		X+,rmp
		dec		rgb
		brne	inzln1
		ldxptr	linbuf			; Point to line buffer
		cbr		flaga,(1<<crf)|(1<<escf)	; Clear exit flags
		ret
;
; Zero RAM data space, excluding UF0 space and timer reload buffers
;
zbuf:
		ldi		rgb,(buffend-tcb0)
		ldxptr	tcb0			; Start at TCB0
		clr		rmp
zbuf1:
		st		X+,rmp
		dec		rgb
		brne	zbuf1
		ret
;
; Zero lower register buffers
;
zregs:
		clr		rmp
		mov		BSTK,rmp
		sts		PA,rmp			; RAM buffer
		sts		PU,rmp			; RAM buffer
		sts		PV,rmp			; RAM buffer
		sts		PY,rmp			; RAM buffer
		mov		res0,rmp
		mov		res1,rmp
		mov		res2,rmp
		ret
;
;  Test rga for control key, 0...19H, 7FH..FFH, and set KYF		(OK)
;  if true, else clear KYF. rga preserved
;
fctl:
		sbr		flaga,(1<<kyf)
		cpi		rga,SP				; rga < SP?
		brcs	fctl1				;	Yes
		cpi		rga,DEL				; rga >= SP?
		brcc	fctl1				;	No
		cbr		flaga,(1<<kyf)		; Clear kyf
fctl1:
		ret
;
;  Display LNCTR contents as decimal digits
;
pgmln:
		lds		YH,lnctr		; Get hi byte
		lds		YL,lnctr+1		; Get lo byte
		call	bn2bcd			; Convert to packed BCD
		call	p3dg
		call	dblsp
		ret
;
;  Show contents of Data Buffer		(OK)
;
pdbuf:
		rcall	pxy
		.db		6,39
		ldxptr	wdbuf			; Pointer to wdbuf
		ld		YH,X+			; Get hi byte
		ld		YL,X			; Get lo byte
		call	bn2bcd			; Convert to packed BCD
		call	p5dg			; Show the data word
		ret
;
; Print 5 digits in res2:res1:res0 stored as packed bcd		(OK)
;
p5dg:
		mov		rga,res2		; Fetch 1st of 3 bytes
		rcall	pdg				; Show digit 5 only
;
; Print 4 digits in res2:res1:res0 stored as packed bcd		(OK)
;
p4dg:
		mov		rga,res1		; Fetch 2nd of 3 bytes
		call	pacc			; Show digits 4,3
;
; Print 2 digits in res2:res1:res0 stored as packed bcd		(OK)
;
p2dg:
		mov		rga,res0		; Fetch 3rd of 3 bytes
		call	pacc
		ret
;
; Print 3 digits in res2:res1:res0 stored as packed bcd		(OK)
;
p3dg:
		mov		rga,res1		; Fetch 2nd of 3 bytes
		call	pdg				; Show digit 3 only
		rjmp	p2dg			; Show digits 2,1
;
; Convert packed BCD digits in rga to ascii and display		(OK)
;
pacc:
		mov		asav,rga		; Save the data
		swap	rga				; Hi nibble first
		call	pdg				; Convert to ascii and display
		mov		rga,asav
		call	pdg				; Show lo nibble
		ret
;
; Display bcd low nibble as ascii digit			(OK)
;
pdg:
		andi	rga,0x0f		; Mask hi nibble
		ldi		rmp,'0'			; Convert by adding '0'
		add		rga,rmp
		call	co				; Show the digit
		ret
;
;
;###########################################################################
;
; --- Conversion and testing routines ---
;
;  Convert rga to upper case
;
case:
		cpi		rga,0x61		; Ascii 'a'
		brcs	case1			; < a
		cpi		rga,0x7b		; Ascii 'z'
		brcc	case1			; > z
		clc
		sbci	rga,SP			; Subtract SP to convert to UC
case1:
		ret
;
; gnum used to process possible good ascii number from linbuf, convert it to		(OK)
; binary number in YH:YL using gdbn.
;
; Save data byte in DBA if number less than 256, or data word
; in WDBUF if number is 256 to 65,535. The numfl is set if the input
; data is good byte value. User to clear numfl after test.
;
; Registers:	flaga, rgv, YH, YL, rmp, XH:XL
;
gnum:
		cbr		flaga,(1<<numfl)	; Clear the good number flag
		mov		rgv,count			; Load counter with number of characters in linbuf
		cpi		rgv,ndec+1			; Compare to limit
		brcc	gnum2				; Too many digits error
		call	gdbn				; Convert to binary in YH:YL
		brcc	gnum2				; Error
		ldxptr	wdbuf				; Point to data word buffer
		st		X+,YH				; Save result high byte
		st		X+,YL				; Save result low byte
		st		X,YL				; Save result low to data byte buffer
		tst		YH					; Test result high byte if zero
		breq	gnum1				;	Yes, word value
		ldi		rmp,0xff			;	No, cap byte value to 0xff
		st		X,rmp				;   Save in dba
gnum1:
		sbr		flaga,(1<<numfl)	; Mark as byte value
gnum2:
		ret
;
; Convert ASCII decimal string in LINBUF to binary number in  YH:YL.		(OK)
; The number to be converted is maximum allowed 5 ascii digits long,
; set by ndec, equivalent to (0xffff) binary number.
;
; This routine called by gnum to convert ascii numbers for data entry
;
; Entry: rgv = ndec, number of ascii digits to convert
; Exit:  YH:YL <-- 16 bit result if ok, C = 1
;        YH:YL <-- 00, C = 0 if error
; Regs:  rga, rgb, rgc, YH, YL, rgv, XH:XL
;
gdbn:
		clr		YH			; Clear result registers
		clr		YL
		ldxptr	linbuf		; Setup line buffer pointer
gdbn1:
		ld		rga,X+		; Fetch a character
		call	decdg		; Convert to BCD
		brcc	gdbnx		; Error exit
		mov		asav,rga	; Save character
		call	dex10		; Value * 10, result in YH:YL
		brcs	gdbnov		; Overflow
;
		mov		rgc,asav	; Add original digit in
		clr		rgb
		call	adebc		; YH:YL = YH:YL + rgb:rgc
		brcs	gdbnov		; Overflow error
		dec		rgv			; All characters processed?
		brne	gdbn1		;	No, continue
		sec
		ret					;	Yes, normal exit
;
gdbnx:
		clc					; Error exit
		clr		YH
		clr		YL
		ret
;
gdbnov:
		sec					; Overflow condition
		ldi		YH,0xff
		ldi		YL,0xff	; Limit to 0xFFFF
		ret
;
; Convert ASCII 0.....9 to BCD, C = 1 if ok, else		(OK)
; C = 0 and rga unchanged
; Registers:	rga, asav
;
decdg:
		mov		asav,rga	; Save ascii digit
		call	case		; Fold to UC
		subi	rga,'0'		; Char less than char '0'?
		brcs	ddgx		;	Yes, error exit
		cpi		rga,LF		;  Char from 0...9?
		brcc	ddgx		;	No, error exit
		ret					; Is 0...9
ddgx:
		clc					; Not 0...9
		mov		rga,asav
		ret
;
; Convert 16 bit binary in YH:YL to packed bcd in res2:res1:res0		(OK)
;
; Registers: rgb, rgc, YH, YL, res0, res1, res2
;
bn2bcd:
		ser		rgc					; rgc = 0xff
		mov		res2,rgc
;
; Process 10,000's digit
;
cvde_L10k:
		inc		res2
		subi	YL,low(10000)
		sbci	YH,high(10000)
		brcc	cvde_L10k			; Loop until C set
		subi	YL,low(-10000)		; Correct last subtraction
		sbci	YH,high(-10000)
		ldi		rgb,(256-16)
;
; Process 1000's digit
;
cvde_L1k:
		subi	rgb,(-16)
		subi	YL,low(1000)
		sbci	YH,high(1000)
		brcc	cvde_L1k			; Loop until C set
		subi	YL,low(-1000)		; Correct last subtraction
		sbci	YH,high(-1000)
		mov		res1,rgc
;
; Process 100's digit
;
cvde_L100:
		inc		res1
		subi	YL,low(100)
		sbci	YH,high(100)
		brcc	cvde_L100			; Loop until C set
		subi	YL,low(-100)		; Correct last subtraction
		or		res1,rgb
		ldi		rgb,(256-16)
;
; Process 10's digit
;
cvde_L10:
		subi	rgb,(-16)
		subi	YL,10
		brcc	cvde_L10			; Loop until C set
		subi	YL,-10				; Correct last subtraction
		mov		res0,rgb
		or		res0,YL
		ret
;
;
; Convert rga to hex digit and set C, else clear C if character
; not an ascii hexadecimal digit. On error, return with character in rga
;
; Registers:	rga
;
hexdg:
		mov		asav,rga	; Save char
		rcall	case		; Fold to UC
		subi	rga,'0'		; rga < '0'?
		brcs	hexdg1		;	Yes, exit
		cpi		rga,LF		; rga from 0...9?
		brcs	hexdg2		;	Yes
		subi	rga,7		; Dump funny chars
		cpi		rga,LF		; Char from 9...A?
		brcs	hexdg1
		cpi		rga,0x10	; Char above F?
		brcc	hexdg1		;	Yes
hexdg2:
		ret					; Normal exit, C=1
hexdg1:
		mov		rga,asav	; Restore char
		clc
		ret
;
; --- General Tile Interpreter Screen Routines ---
;
;  Screen routines
;
slina:
		call	pxy
		.db		24,2			; Cursor to status line position A
		rjmp	slin
slinb:
		call	pxy				; Cursor to status line
		.db		24,22
slin:
		call	ceol			; Clear the line
		ret
;
clean:
		call	pxy				; Wipe screen clean
		.db		8,1
		call	clin			; Clear lines
		.dw		15
		call	pxy
		.db		8,1
		ret
;
; Clear lines specified immediately following rcall to clin
;
clin:
	pop		ZH				; Point to data word
	pop		ZL
	lsl		ZL				; Z*2 for word address
	rol		ZH
	andi	ZL,0xfe			; Fetch lower byte of word
	lpm		rgb,Z+			; Get word
	adiw	ZL,1
	lsr		ZH
	ror		ZL				; Z/2
	push	ZL				; Return address to stack
	push	ZH
clin1:
	call	ceol			; Clear lines
	call	cdown			; Move cursor down 1 row
	dec		rgb
	brne	clin1
	ret
;
; Display 'Enter:' prompt
;
enter:
		call	pxy
		.db		22,6
		call	ceol
		ldzptr	entm
		call	pptr
		ret
;
; Display 'File 0' message
;
pruf0:
		call	pxy
		.db		24,2
		call	ceol
		ldzptr	uf0m			; Show File 0
		call	pptr
		ret
;
; Print main screen
;
prscn:
		call	clrscn			; Home cursor and clear screen
		call	pxy
		.db		1,1
		ldzptr	scrnm			; Print Tile header
		call	pptr
		call	pxy
		.db		23,1
		ldzptr	statm			; Print status line
		call	pptr
		ret
;
sak:
		call	pxy
		.db		24,58
		ldzptr	sakm			; Strike any key
		call	pptr
		call	ci				; Wait for any key
		call	slinb
		ret
;
; --- Data Buffer control and Math routines ---
;
; Multiply YH:YL by 10, called by gdbn ascii to binary converter routine		(OK)
; YH:YL = YH:YL * 10, C = 0 if ok, C = 1 on error
; Registers:	rga, rgb, rgc, YH, YL
;
dex10:
		call	dex2		; YH:YL * 2
		brcs	dexx		; Error exit, overflow and C=1
		push	YH			; Copy YH:YL to rgb:rgc
		pop		rgb
		push	YL
		pop		rgc
;
		rcall	dex2		; * 4
		brcs	dexx
		rcall	dex2		; * 8
		brcs	dexx
		call	adebc		; YH:YL = YH:YL + rgb:rgc
dexx:
		ret
;
; YH:YL: = YH:YL * 2
;
dex2:
		clc
		rol		YL
		rol		YH
		ret
;
; YH:YL = YH:YL + rgb:rgc, C = 0 if ok else C = 1 on overflow
;
adebc:
		add		YL,rgc
		adc		YH,rgb
		ret
;
; Decrement XH:XL pointer by 1		(OK)
;
decxptr:
		push	rmp
		ldi		rmp,-1
		add		XL,rmp
		adc		XH,rmp
		pop		rmp
		ret
;
; "div8u" - 8/8 Bit Unsigned Division				(OK)
;
; This subroutine divides the two register variables "rga" (dividend) and
; "rgb" (divisor). The result is placed in "rga" and the remainder in "rgb".
;
; High registers used:	4 (rga,rgb,rgc,rgv)
;
;
; Register Variables:
;	rgc	remainder
;	rga	dividend & result
;	rgb divisor
;	rgv	loop counter
;
; Entry:	(rga) = dividend
;			(rgb) = divisor
; Exit:		(rga) = integer part of quotient
;			(rgb) = integer remainder
;
div8u:
		push	rgc
		push	rgv
		sub		rgc,rgc			; clear remainder and carry
        ldi		rgv,9			; init loop counter
d8u_1:	rol		rga				; shift left dividend
        dec		rgv				; decrement counter
        brne	d8u_2			; if done
		mov		rgb,rgc			; move remainder to rgb
		pop		rgv
		pop		rgc
        ret						;    return
;
d8u_2:	rol		rgc				; shift dividend into remainder
        sub		rgc,rgb			; remainder = remainder - divisor
        brcc	d8u_3			; if result negative
        add		rgc,rgb			;    restore remainder
        clc						;    clear carry to be shifted into result
        rjmp	d8u_1			; else
d8u_3:	sec						;    set carry to be shifted into result
        rjmp	d8u_1
;
;
; --- General Service Routines ---
;
;
; Turn off watchdog
;
wdt_off:
		cli							; Clear global interrupts
;
; Reset WD timer
;
		wdr
;
		in		rmp,MCUSR				; Clear WDRF bit
		andi	rmp,(0xff & (0<<WDRF))	; WDRF bit = 0
		out		MCUSR,rmp
;
; Set WDCE and WDE bits, keep old prescaler setting
;
		lds		rmp,WDTCSR
		ori		rmp,(1<<WDCE)|(1<<WDE)
		sts		WDTCSR,rmp
;
; Turn off WDT
;
		ldi		rmp,(0<<WDE)			; Clear WD system reset enable
		sts		WDTCSR,rmp
;
		sei								; Set global interrupts
		ret
;
;
;###########################################################################
;
; --- Video routines ---
;
;
; --- Low level video drivers ---
;
; Register rga used to pass data to console output routine
;
; Print rga data as two hexadecimal digits.			(OK)
;
pahex:
	push	rga
	swap	rga				; Show MSD nibble first
	rcall	pahex1
	pop		rga
pahex1:
	andi	rga, 0x0f		; Mask off higher nibble
	ldi		rgv, 0x30 		; Add ascii '0' to convert
	add		rga, rgv		; Convert to ascii
	cpi		rga, 0x3a		; Check if > 9
	brcs	pahex2			;  No, it is 0 ... 9
	ldi		rgv, 0x07		;  Yes, convert to A ... F
	add		rga, rgv
pahex2:
	call	co
	ret
;
; Print rga contents as decimal (0...255). Leading			(OK)
; zero suppression is provided only on the 100's
; digit, so at least two digits are always printed.
;
; Registers rga, rgb not saved
;
pdec:
	ldi		rgb,100			; Get 100's digit
	call	div8u
	tst		rga				; Do leading zero suppression
	breq	pdec1
	call	pnum
pdec1:
	ldi		rga,10			; Get 10's digit
	xchreg	rga,rgb
	call	div8u			; rgb has units
	call	pnum
	xchreg	rga,rgb
pnum:
	ori		rga,0x30		; Ascii "0"
	call	co				; Show ascii decimal
	ret
;
; Scan for keyboard input and return char in rga if any,
; else rga=0.
;
getc:
	lds		rmp,UCSR0A		; Get UART control status register
	sbrs	rmp,RXC0		; Test receiver complete flag
	rjmp	getc1
	lds		rga,UDR0		; rga <-- UDR0
	ret
getc1:
	clr	rga
	ret
;
; Load rga from UDR0 register. Waits until data byte is received.		(OK)
;
ci:
	lds		rmp,UCSR0A		; Get UART control status register
	sbrs	rmp,RXC0		; Test receiver complete flag
	rjmp	ci
;
; Fetch data
;
	lds		rga,UDR0		; rga <-- UDR0
	ret
;
; Load UDR0 from rga. Wait until transmitter is empty before loading.		(OK)
;
co:
	lds		rmp,UCSR0A		; Get UART control status register
	sbrs	rmp,UDRE0		; Test if UDR0 is empty
	rjmp	co
;
; Send data
;
	sts		UDR0,rga		; UDR0 <-- rga
	ret
;
; Print CR and LFs	(OK)
;
crllf:
	rcall	crlf			; Two CRLF
crlf:
	push	rga
	ldi		rga,CR			; Carriage return
	call	co
	ldi		rga,LF			; Linefeed
	call	co
	rjmp	cco
;
; Print spaces	(OK)
;
dblsp:
	call	space
space:
	push	rga
	ldi		rga,SP			; Space
	rjmp	cco
;
; Print tab	(OK)
;
prtab:
	push	rga
	ldi		rga,HT
	rjmp	cco
;
; Ring bell
;
beep:
	push	rga
	ldi		rga,BELL
cco:
	call	co
	pop		rga
	ret
;
; Print comma	(OK)
;
prcma:
	push	rga
	ldi		rga,cma
	rjmp	cco
;
; Print delete character at cursor	(OK)
;
bksp:
	push	rga
	call	cbak			; Delete character at cursor
	call	ceol			; Clear cursor to end of line
	pop		rga
	ret
;
; Print message string, ^Z terminated. Routine is called with		(OK)
; code address of string loaded in ZH:ZL.
;
pptr:
	push	rga
pptr1:
	lpm		rga,Z+			; String byte to rga, Z+
	cpi		rga,ctlZ		; byte ^Z?
	brne	pptr2			; Print if not ^Z
	pop		rga
	ret
pptr2:
	cpi		rga,NULL		; Skip any nulls in string
	breq	pptr1
	rcall	co
	rjmp	pptr1
;
; --- Video and Cursor control routines ---
;
; Clear screen	(OK)
;
clrscn:
	push	zh
	push	zl
	ldzptr	scrn		; Home cursor
	call	pptr
	ldzptr	clrs		; Clear entire screen
	rjmp	video
;
; --- Move cursor down ---
;
cdown:
	push	zh
	push	zl
	ldzptr	cudn		; Cursor down one row
	rjmp	video
;
; --- Clear to end of line ---
;
ceol:
	push	zh
	push	zl
	ldzptr	eol			; Clear to end of screen
	rjmp	video
;
; --- Cursor back one column ---
;
cbak:
	push	zh
	push	zl
	ldzptr	cubk			; Cursor back 1 column
	rjmp	video
;
; --- Highlight on ---
;
vhi:
	push	zh
	push	zl
	ldzptr	hi			; Highlight on
	rjmp	video
;
; --- Normal ---
;
vlo:
	push	zh
	push	zl
	ldzptr	lo			; Normal - attributes off
	rjmp	video
;
; --- Reverse ---	(OK)
;
vrev:
	push	zh
	push	zl
	ldzptr	rev			; Reverse on
video:
	rcall	pptr
	pop		zl
	pop		zh
	ret
;
; --- Video position cursor sequences ---
; Lead-in sequence
;
vpxy1:
	push	zh
	push	zl
	ldzptr	pxy1			; Lead-in sequence
	rjmp	video
;
; Middle sequence
;
vpxy2:
	push	zh
	push	zl
	ldzptr	pxy2			; Middle sequence
	rjmp	video
;
; End sequence
;
vpxy3:
	push	zh
	push	zl
	ldzptr	pxy3			; Trailing sequence
	rjmp	video
;
; --- Save cursor position ---
;
vscp:
	push	zh
	push	zl
	ldzptr	scp			; Save cursor position
	rjmp	video
;
; --- Restore cursor position ---
;
vrcp:
	push	zh
	push	zl
	ldzptr	rcp					; Restore cursor position
	rjmp	video
;
; --- Position cursor at row, column immediately following rcall to pxy ---
;
; Row & column values must be given as ascii decimal.			(OK)
;
pxy:
	call	vpxy1			; Lead-in sequence
	pop		ZH				; Point to string start address
	pop		ZL
	clc
	rol		ZL				; 16 bit multiply by 2 for word address
	rol		ZH
;
	lpm		rga,Z+			; Pick up row value
	call	pdec			; Print it and ..
	call	vpxy2			; Middle sequence		+++++ Uses Z pointer, must save Z +++
	lpm		rga,Z+			; Pick up column value
	call	pdec			; Print it and ..
	call	vpxy3			; End sequence
;
	clc
	ror		ZH
	ror		ZL
	push	ZL				; Return to caller
	push	ZH
	ret
;
; Position cursor at (YH)-->row, (YL)-->col		(OK)
;
gotoxy:
	call	vpxy1			; Send lead-in string
	mov		rga,YH			; Get row value
	call	pdec			; Send row
	call	vpxy2			; Send middle string
	mov		rga,YL			; Get col value
	call	pdec			; Send col
	call	vpxy3			; Send trailing string
	ret
;
;
; --- Message strings data area ---
;
; Terminal control sequences
;
cudn:	.db	ESC,"[B",ctlZ		; Move cursor down
cubk:	.db	ESC,"[D",ctlZ		; Cursor back one column
scrn:	.db	ESC,"[H",ctlZ		; Home cursor
eos:	.db	ESC,"[0J",ctlZ		; Clear from cursor to end of screen
clrs:	.db	ESC,"[J",ctlZ		; Clear entire screen
eol:	.db	ESC,"[K",ctlZ		; Erase to end of line
hi:		.db	ESC,"[1m",ctlZ		; Highlight on
lo:		.db	ESC,"[m",ctlZ		; Normal - attributes off
rev:	.db	ESC,"[7m",ctlZ		; Reverse on
pxy1:	.db	ESC,"[",ctlZ		; Lead-in sequence
pxy2:	.db	";",ctlZ			; Middle sequence
pxy3:	.db	"H",ctlZ			; Trailing sequence
dlc:	.db	ESC,"[1M",ctlZ		; Delete line at cursor
scp:	.db	ESC,"7",ctlZ		; Save cursor position
rcp:	.db	ESC,"8",ctlZ		; Restore cursor position
;
;
;###########################################################################
;
.include	"eeprom_module.asm"
;
;###########################################################################
;
; Write TRBm buffer in SRAM to EEPROM using sequential write
; 
; Entry:	rga has TRBm buffer data to write
;			XHL = TRBm offset from base address
;			YHL local temporary use

wr_trb:
		out		EEARL,XL
		out		EEARH,XH		; EEAR <== TRBm load address
;
; Write rga to specified EEPROM address in EEAR
;
		mov		rmp,rga			; EEWrite expects data in rmp
		rcall	EEWrSeq
		ret
;
;
; --- Screen displays ---
;
scrnm:
	.db	"              <<<<  m328-uTile Programmable Logic Controller >>>>",cr,lf
	.db	"--------------------------------------------------------------------------------",cr,lf
	.db	"           Bit Stack                                        Port Byte",cr,lf
	.db	"        7 6 5 4 3 2 1 0               Data               7 6 5 4 3 2 1 0",cr,lf
	.db	"        ===============               =====              ===============",cr,lf,lf
statm:
	.db	"--------------------------------------------------------------------------------",ctlZ
hlpm:
	.db	"  Logic   Bit Stack           I/O - Commands          General Commands    Timers",cr,lf
	.db	"  -----   ---------    --------------------------    ------------------   ------",cr,lf
	.db	"   and     dup 2dup      In:  Ai. ALi. ATi. ADi.       fill  ex0   run     TQm.",cr,lf
	.db	"   or       drop        Out: .Yn                       ins   key   end    .Tkm",cr,lf
	.db	"   xor      swap       Virt:  Un., .Un, Vn., .Vn       del    /    ver    .TRm",cr,lf
	.db	"   ! !!     f. z.    (i = 0..5, n = 0..7, m = 0..f)     ?   load   store   ldt",cr,lf
	.db	"   nop             .                                  list  read   write   LDTm",cr,lf,lf
	.db	"  ------- Byte Commands -------     -- Flip/Flop --    --- Clock Pulses ---",cr,lf
	.db	"   PA.  .PY   PU.  .PU  PV. .PV     .Sm  .Rm   Qm.     clka.   clkb.  clkc.",cr,lf
 	.db	"   PPA  PBS   PPU  PPV  PPY          -- Counter --     200 ms  600 ms 1.8 s",cr,lf
	.db	"  Ground D8 to enable Autostart          .CNT        Timers: T0..T3 (0.1 s)",cr,lf
	.db "  Ground D9 to invert outputs        -- Toggle --            T4..Tb (1 s)",cr,lf
	.db "  Date: 2025/03/17 Ver 1.0           .TGn   TGQn.    Tc..Te (min), Tf (10 min)",cr,lf,ctlZ
;
inerr:	.db	"   ?",0,ctlZ
runm:	.db	"*** Running User File Program ***",ctlZ
sakm:	.db	"  Strike any key --> ",ctlZ
uf0m:	.db	"*** User File 0 ***",0,ctlZ
spbs:	.db	"     ",BS,BS,BS,BS,BS,ctlZ
err1:	.db	"  *** Unrecognized Input! ***",ctlZ
entm:	.db	"Enter: ",ctlZ
dots:	.db	".....",ctlZ
storem:	.db "   Storing UF0, please wait ... ",ctlZ
ldtmm:	.db	"*** Load timers, Z clear all, <CR> exit ***",ctlZ
thdr1:  .db     "T0     T1     T2     T3     T4     T5     T6     T7",ctlZ
thdr2:  .db     "T8     T9     TA     TB     TC     TD     TE     TF",ctlZ
thdr3:  .db     "====================================================",ctlZ
;
;
;
;----
.exit
;----
;
; --- End of source code ---
;
