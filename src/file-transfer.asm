;
;
; file-transfer.asm - Dictionary command
;
;  Copyright (c)   2013 Jan 25   Francis A. Lyn
;  Version 0
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
; INCLUDE file-transfer.asm in main.asm
;
; Routines to read and write Intel hex file format data files between
; host PC and uTile. Hex file format:
;
;	:llaaaatt[dd...]	ascii characters
;	':' is start-of-record preamble
;	ll is length or record
;	aaaa is word length load address
;	tt is record type, 00 for data record used here
;	dd is data bytes in the record.
;
; --- File Transfer Words ---
;
;  read - File read routine.
;
;  read downloads a uTile User File Application from host that was previously
;  stored with the write command word.
;
;  A '/' key aborts download at start. Overrun error exits routine.
;
; Registers:
;  rga, rmp, general purpose	rgv = counter
;  rgc = checksum				rgd = counter for record read
;  XHL = RAM pointer			YHL = 16 bit buffer, YL has byte from <rdbyt>
; paceb flag used in <nochrs>
;
;  Since the <rdchr> routine occurs at several levels of subroutines,
;  error exits from <rdchr> back to the editor screen must restore the
;  SP to the initial state.
;
read:
		.db		"READ",0,ctlW
		.dw		readx
		.dw		EDIT					; Interpreter only
readL:
		.dw		(read-readL)
;
iread:
		call	clean					; Clear screen
		call	pxy						; Print file read message
		.db		9,25
		ldzptr	rdmsg					; Read file header message
		call	pptr
		call	slina					; Status line
;
;  Read a line of hex format data from serial port
;
read1:
		call	rdchr					; Wait for serial character input
		cpi		rga,'/'					; Abort character?
		brne	read2
		call	nochrs					;  Eat possible junk
		rjmp	rdabt					;  Yes, exit
read2:
		cpi		rga,':'					; Start of record?
		brne	read1					; No, wait for start character
		call	rdbyt					; Yes, get count byte, binary value returned in YL
		brcc	rderr					; Error exit
		tst		YL						; Count = 0?
		breq	rdend					; Yes, exit
;
		mov		rgd,YL					; Load bytes counter
		mov		rgc,YL					; Load initial checksum value to rgc
;
		call	rdwrd					; YHL <--load address
		brcc	rderr					; Error exit
		movw	XH:XL,YH:YL				; XHL <-- YHL
		add		rgc,YH					; Add address hi to checksum
		add		rgc,YL					; Add address lo to checksum
;
		call	rdbyt					; Get record type
		brcc	rderr					; Error exit
		tst		YL						; Type = 0?
		brne	rdend					; No, read end
;
read3:
		call	rdbyt					; Get data byte
		brcc	rderr					; Error exit
		st		X+,YL					; Store in RAM
		add		rgc,YL					; Update checksum
		dec		rgd
		brne	read3					; Continue till record done
;
		call	rdbyt					; Get sender's 2's compl checksum
		brcc	rderr					; Error exit
		add		rgc,YL					; Add to receiver's checksum
		tst		rgc
		breq	read1					; Continue reading if match
;
;  Return routines
;
rderr:
		call	nochrs					; Gobble input till pause
		ldzptr	errd					; Print error message
		rjmp	rdend1
;
;  Arrive here at end of successful file read
;
rdend:
		call	nochrs				; Gobble input till pause
		ldzptr	rendm				; Print end message
rdend1:
		call	pptr
;
mmnu:
		ldzptr	mmsg				; Print menu message
		call	pptr
mmnu1:
		call	ci					; Enter 'CR' for editor screen
		cpi		rga,CR
		brne	mmnu1				; Drop all other characters
;
rdabt:
		rcall	prscn				; Print command screen
		rcall	pruf0				; UF0 selected
		rcall	iuf0				; Setup control buffers for User File 0
		rcall	dpage				; Display page 0
		ret
;
;
;  Data read routines - <rdchr> uses the normal console serial port.
;  Scan for character, return with character in rga else rga = 0.
;
;	Exit:	Character in rga
;			If no character, rga = 0
;
rdchr:	call	getc				; Check for incoming character
		tst		rga
		breq	rdchr				; Wait for received character
		ret
;
;  Read 4 ASCII hex characters, convert to binary, YHL <-- word
;
rdwrd:
		ldi		rgv,4				; 4 char/word
		rjmp	nxchr
;
;  Read 2 ASCII hex characters, and leave binary byte in YHL
;
rdbyt:	
		ldi		rgv,2				; 2 char/byte
;
; Read in ASCII characters, convert to valid hex digit, and
; shift result into YHL. On error, exit to rderr. On exit
; YHL has binary data, byte or word.
;
nxchr:
		clr		YH					; Clear result regs
		clr		YL
nxch1:
		call	rdchr				; Get a serial char, rga = ascii char
		call	hexdg				; Convert to hex digit, result in rga
		brcc	nxchex				; Not a hex digit, error exit
;
		push	rga
		call	sl4y				; Shift YHL 4 left
		pop		rga
		or		YL,rga				; Merge in new nibble in rga to YL
		dec		rgv
		brne	nxch1				; Loop till done
		sec							; Routine successful, C = 1
nxchex:
		ret
;
; Shift YHL left 4 bits. YL low nibble = 0b0000
;
sl4y:
		push	rgd
		ldi		rgd,4				; Shift counter
		clc
sl4y1:
		rol		YL
		rol		YH
		dec		rgd
		brne	sl4y1
		pop		rgd
		ret
;
; Gobbles input stream until there is a quiet period. 
; Used to get rid of wrong file type transfers.
;
nochrs:
		cbr		flagb,(1<<paceb)	; Use 100 ms paceb flag
		sbrs	flagb,paceb			; Wait for paceb set
		rjmp	nochrs
		cbr		flagb,(1<<paceb)	; Use 100 ms paceb flag
;
nochr1:
		call	getc				; Scan input buffer
		tst		rga
		breq	nochrx				; Quiet, exit
nochr2:
		sbrs	flagb,paceb			; Wait for paceb set
		rjmp	nochr1				; Until timeout ends
nochrx:
		ret	
;
;
readx:
;
;
;  write - User File write routine.
;
;  Upload uTile User File Application to host for later retrieval
;  with the read command word.
;
;  Registers:
;		XHL = RAM buffer pointer, 16b
;		YHL = Filesize counter, 16b
;		rga, rmp general purpose registers
;		rgb = partial record counter
;		rgc = checksum
;		rgd = counter
;		count = character counter
;
;
; The following addresses define the RAM buffer area for the UF0 space
; as well as the one-shot timers reload buffers
;
;	 filbeg = uf0st:
;	 filend = TCB0:, next byte after TRBf
;
;
write:
		.db		"WRITE",ctlW
		.dw		writex
		.dw		EDIT					; Interpreter only
writeL:
		.dw		(write-writeL)
;
Iwrite:
		call	clean
		call	pxy						; Position to status line
		.db		9,17
		ldzptr	wrmsg					; File reader header message
		call	pptr
		call	slina
		ldzptr	xfrm					; Transfer message
		call	pptr
		call	pxy
		.db		11,1
;
;  Wait for console start or abort command '/'
;
write1:
		call	ci						; Scan for input 'CR' or '/'
		cpi		rga,cr					; Start upload?
		breq	write3					;   Start upload on'CR'
;
write2:
		cpi		rga,'/'					; '/' for editor screen?
		brne	write1					;	No, keep looking
		rjmp	wrex					; Exit
;
; Set up pointer XHL to source buffers
;
write3:	
		ldxptr	filbeg					; Start of buffer area to save
;
write4:
		ldi		YH,high(filend-filbeg)	; Filesize counter
		ldi		YL,low(filend-filbeg)	; Filesize counter
;
; Shift YHL right 4 times, LS nibble into rgb. This converts file size
; number in YHL into number ofrecords of 0x10 data bytes long, partial
; record remainder is left in rgb (after swapping nibbles).
; At the end of the right shift, YHL has number of full records count.
;
sr4y:
		clc
		ldi		rgd,4				; Shift counter
		clr		rgb					; Clear result rgb
sr4y1:
		ror		YH
		ror		YL
		ror		rgb
		dec		rgd
		brne	sr4y1
		swap	rgb					; Partial record count in rgb
;
		tst		YH
		brne	sdfil
		tst		YL					; Full records?
		breq	sdfil3				;	No, check for a partial record
;
;  Arrive here if whole record (rgc = 10) to be sent
;
sdfil:
		call	slhdr				; Send line header
sdfil1:
		call	sline				; Send a line of 16 characters
sdfil2:
		sbiw	YH:YL,1				; Decrement whole records counter
		brne	sdfil				; Keep sending data
;
; Arrive here to send a partial record if any. Number of data bytes
; in partial record is in rgb
;
sdfil3:
		tst		rgb					; Any data bytes in partial record?
		breq	seof				;	No, no data bytes left
;
		ldi		rga,':'				;	Yes, send start of record ':'
		call	co
		mov		rgd,rgb				; Load bytes counter with partial counts
		call	slhdr1				; Send line header
		call	sline				; Send the characters
seof:
		ldi		rga,':'				; Send EOF line
		call	co					; ':00000001FF'
		ldi		rgd,3				; Bytes to send
seof1:
		clr		rga
		call	pahex				; Send rga as ASCII hex digits
		dec		rgd
		brne	seof1
;
		ldi		rga,0x01			; Record type 01
		call	pahex
		ldi		rgc,0x01			; Checksum byte
		call	sline1
;
;  Arrive here at the end of the upload operation
;
wrend:
		call	ci					; Eat everything till 'CR'
		cpi		rga,CR
		brne	wrend				; Drop all other characters
wrex:
		rcall	prscn				; Print command screen
		call	pruf0				; UF0 selected
		rcall	dpage				; Display page 0
		ret							; Exit to main editor screen
;
;  Send line header
;
slhdr:
		ldi		rga,':'				; Start of record character
		call	co
		ldi		rgd,0x10			; Send 10H characters per line
slhdr1:
		mov		rga,rgd				; Get partial counts
		call	pahex				; Send count as ASCII hexadecimal
		clr		rgc					; Clear checksum
		add		rgc,rgd				; Add count to checksum
		add		rgc,XL				; Add load address low to checksum
		add		rgc,XH				; Add load address hi to checksum
		call	padr				; Send load adress as ASCII hexadecimal
		clr		rga					; Record type = 0
		call	pahex				; Send record type 0, data
		ret
;
;  Send a data bytes in record as ASCII hexadecimal
;
sline:
		ld		rga,X+				; Get byte to send
		push	rga
		call	pahex				; Send byte as ASCII hexadecimal
		pop		rga					; Restore byte
		add		rgc,rga				; Add data to checksum
		dec		rgd					; Loop until all data in record sent
		brne	sline
;
sline1:
		com		rgc					; Complement checksum
		inc		rgc					; Form two's complement
		mov		rga,rgc
		call	pahex				; Send 2's complement checksum
		call	crlf				; Send 'CR'/'LF'
		ret
;
;  Send pointer XHL contents (target address) as hexadecimal word
;
; Entry:
;		XHL points to RAM buffer address
; Exit:
;		XHL+
;
padr:
		mov		rga,XH				; Show high byte
		call	pahex
		mov		rga,XL				; Then low byte
		call	pahex
		ret
;
;
wrmsg:
	.db		"File write .... 'CR' to send .... '/' to abort",ctlZ
;
xfrm:
	.db		"*** Start/end host transfer procedure ***",ctlZ
;
rdmsg:
	.db		"Reading file ...... '/' to abort",ctlZ
;
rendm:
	.db		"File read completed -",ctlZ
;
errd:
	.db		"Read error! - ",ctlZ
;
mmsg:
	.db		" 'CR' for menu -->",ctlZ
;
;
writex:
;
;----
.exit
;----
;
;
