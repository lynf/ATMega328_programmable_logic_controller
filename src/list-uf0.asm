;
; list-uf0.asm - Dictionary command
;
;  Created: 4/10/2020
;  Author: lynf
;
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
;   email:  lynf.yyz@gmail.com
;
; INCLUDE list-UF0EN.asm in main.asm
;
; Dictionary command for listing UF0 as ascii text file
; via serial port. Listing includes the 16 Timer settings
;
; ---
;  List UF0 contents to serial output
; ---
;
; See WRITE command in file transfer module for example code.
;
list:
		.db		"LIST",0,ctlW
		.dw		liste
		.dw		EDIT			; Editor mode only
listL:
		.dw		(list-listL)	; Offset to namestring start
;
ilist:
;
; List routine prints 8 pages of UF0 program instructions on
; main screen as ascii text. Display format is similar to the
; editor's page display screen. The displayed program list is
; designed to be recorded on host PC using a Kermit log session
; script. See Kermit list.cmd take file
;
		call	clrscn		; Clear entire
		call	pxy			; Position to status line
		.db		3,17

		ldzptr	lsmm		; Start listing message
		call	pptr
;
;  Wait for console start or abort command '/'
;
list1:
		call	ci				; Scan for input 'CR' or '/'
		cpi		rga,cr			; Start listing?
		breq	list3			;   Yes
;
list2:
		cpi		rga,'/'			; '/' for editor screen?
		brne	list1			;	No, keep checking
		rjmp	listex			; Exit

list3:
		ldzptr	lshm			; UF0 listing
		call	pptr
		call	inzuf0			; UF0 line 000
		rcall	lspage			; Do UF0 listing
		rcall	lstmr			; Do Timer listing

lsend:
		call	ci				; Scan for 'CR' to exit
		cpi		rga,CR
		brne	lsend

listex:
		call	inzuf0			; Restore UF0 start
		rcall	prscn			; Print command screen
		call	pruf0			; UF0 selected
		rcall	dpage			; Display page 0
		ret
;
; ---
;
; List one page of 8 text lines
; rgc is page text line counter, 8 lines per page
;
lspage:
		ldi		rgd,8			; pages to list
lspage0:
		push	rgd
		ldi		rgc,8			; Text lines counter
lspage1:
		push	rgc
		rcall	lsline			; List one line
;
		pop		rgc
		dec		rgc				; Count text lines
		breq	lspage2			; Page done
;
; Subtract 23 from basln+1 buffer (low byte) to
; prepare for next text line
;
		ldi		rmp,-23
		lds		rga,basln+1		; Low byte
		add		rga,rmp			; Subtract 23
		sts		basln+1,rga		; (basln+1) - 23
		rjmp	lspage1
;
lspage2:
		rcall	crlf			; Page separator
;
		lds		rmp,basln+1		; Low byte
		inc		rmp				; New page
		sts		basln+1,rmp
;		
		pop		rgd				; All pages?
		dec		rgd
		brne	lspage0
		ret
;
; ---
;
; List 4 decomplied program line/namestring on text line
; The <dblsp:> and <prtab:> are required for even column
; spacing on text page due to tab action for nametrings
; of 1 and 2 characters.
;
lsline:
		call	prtab
		ldi		rgb,4			; Commands/line counter
lsline1:
		push	rgb
;
		lds		YH,basln
		lds		YL,basln+1		; Y <-- (basln) contents
		sts		lnctr,YH
		sts		lnctr+1,YL		; (lnctr) <-- Y

		rcall	l_decom			; Decompile a line
		call	dblsp			; Even columns
		call	prtab
;
		pop		rgb
		dec		rgb
		breq	lsline2			; Done
;
		lds		YL,basln+1
		ldi		rmp,8
		add		YL,rmp			; inc by 8
		sts		basln+1,YL		; Save new basln+1
		rjmp	lsline1
;
lsline2:
		ldi		rga,lf
		call	co
		ret
;
; Modified <decom:> for listing routine. <vctran:> not used, 
; <pdptr:> used instead of <prnam:> as print space-backspace
; string not used.
;
l_decom:
		rcall	llnctr		; Load LNCTR with absolute line number
		rcall	lpntr		; Load pointr with IWRD vector
		rcall	pgmln		; Show program line number
		rcall	valwd		; Test for valid word
		brcc	l_notwd		; Not valid, show dots
		rcall	ptnam		; Point Z to namestring
		rcall	pdptr		; Show namestring
		ret
l_notwd:
		ldzptr	dots
		call	pptr
		ret
;
; ---
;
; List timer settings
;
lstmr:
		ldzptr	ltmm		; Timer settings, 1st row
		call	pptr

		ldxptr	TRB0		; Base address of timer reload buffers
		ldi		rgv,ntim/2	; Timers per line
	
lstmr1:
		rcall	lstml		; List a line of 7 timers
		call	crlf		; Down 2 lines
;
		call	prtab
		ldzptr	lthm		; Timer settings, 2nd row
		call	pptr
		ldi		rgv,ntim/2	; Timers per line
		rcall	lstml		; List a line of 7 timers
		ldi		rga,lf
		call	co
		ret
;
; List (rgv) timer reload values on a line
;
lstml:
		ld		rga,X+			; Pick up buffer contents
		mov		YL,rga
		clr		YH				; YH = 0
		call	bn2bcd			; Convert to packed BCD
		call	p3dg			; Show timer setting
		call	prtab			; 2 Tabs to next col
		dec		rgv				; Count timers
		brne	lstml			; Continue
		ret
;
; ---
;
lsmm:	.db	"list .... 'CR' to send .... '/' to abort",cr,lf,lf,ctlZ
lshm:	.db "UF0 listing:",cr,lf,ctlZ
;
ltmm:	.db "Timer settings:",cr,lf,ht
		.db "T0      T1      T2      T3      T4      T5      T6      T7",cr,lf,ht,ctlZ
lthm:	.db "T8      T9      TA      TB      TC      TD      TE      TF",cr,lf,ht,ctlZ

;
liste:
;
;
;----
.exit
;----
;
;
