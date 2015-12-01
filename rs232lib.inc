;#define CAUTIOUS

;;; function-definition macros to "print" characters to a circular buffer in
;;; SRAM and optionally out the RS232 port as well (if RS232ECHO is defined)
defputc	macro	fsr,scratch	;void funcname(uint8_t fsr, char w, char* scratch) {
	;; put into next RAM location in circular buffer (using other FSR!)
	movwf	scratch		; scratch = w;
	movlw	0x20		;
#if (!fsr)==1
	movwf	FSR1H		;
#else
	movwf	FSR0H		;
#endif
	movf	scratch,w	;
#if (!fsr)==1
	movwi	FSR1++		; *(FSR[!fsr])++ = w;/* other FSR is advanced */
#else
	movwi	FSR0++		;
#endif
#ifdef RS232ECHO
	;; also put into USART buffer
	banksel PIR1		; extern uint8_t* FSR[2];
	clrwdt			; do
	btfss	PIR1,TXIF	;  clrwdt();
	bra	$-2		; while ((PIR1 & (1 << TXIF)) == 0); /* full */
	banksel	TXREG		;
	movwf	TXREG		; putchar(TXREG = w = scratch);
#endif
	endm			;}

defprnt	macro	fsr,putc	;void funcname(uint8_t fsr, (*putc)()) {
	banksel	TOSH		;
	movf	TOSH,w		;
#if (fsr)==1
	movwf	FSR1H		; extern uint8_t* FSR[2];
	bsf	FSR1H,7		;
#else
	movwf	FSR0H		;
	bsf	FSR0H,7		;
#endif
	movf	TOSL,w		;
#if (fsr)==1
	movwf	FSR1L		; for (FSR[fsr] = 0x8000 | *STKPTR; /* flash */
	moviw	FSR1++		;      (w = *(FSR[fsr])++)
#else
	movwf	FSR0L		;
	moviw	FSR0++		;
#endif
#ifdef __DEBUG	
 	andlw	0xff		;<- simulator bug: moviw doesn't set Z in STATUS
#endif
	btfsc	STATUS,Z	;      != '\0';)
	bra	$+4 ;/*btfss*/	;
	pagesel	putc		;
	call	putc		;  putc(w);
#ifdef __DEBUG
	bra	$-6 ;/*moviw*/	; 
#else
	bra	$-5 ;/*moviw*/	; 	
#endif
	btfss	INTCON,GIE	; if ((INTCON & (1 << GIE)) == 0)
#if (fsr)==1
	bcf	FSR1H,7		;  FSR[fsr] &= 0x7fff; /* interrupts were off */
#else
	bcf	FSR0H,7		;
#endif
	bcf	INTCON,GIE	; INTCON &= 0x7fff; /* disable all interrupts */
#if (fsr)==1
	movf	FSR1H,w		;
#else
	movf	FSR0H,w		;
#endif
	banksel	TOSH		;
	movwf	TOSH		;
#if (fsr)==1
	movf	FSR1L,w		;
#else
	movf	FSR0L,w		;
#endif
	movwf	TOSL		; *STKPTR = FSR[fsr] & 0x7fff;/* next command */
#if (fsr)==1
	btfsc	FSR1H,7		; if (FSR[fsr] & 0x8000)
#else
	btfsc	FSR0H,7		;
#endif
	bsf	INTCON,GIE	;  INTCON |= 0x8000; /* interrupts back on */
	endm			;}

hexdig	macro	offseta		;inline uint8_t hexdig(uint8_t offseta, char w) {
	andlw	0xf		;
	btfss	WREG,3		;
	bra	$+6 ; /* < 8 */	;
	btfsc	WREG,2		;
	bra	$+3 ; /* > 11 */;
	btfss	WREG,1		;
	bra	$+2 ; /* < 10 */; if ((w &= 0x0f) > 9)
	addlw	offseta-0xa-'0'	;  w += (offseta - '0')
	addlw	'0'		; return w += '0';
	endm			;}
	
printx	macro	file,putc   	;inline void printx(uint8_t file, (*putc)()) {
	swapf	file,w		;
	hexdig	'A'		;
	pagesel	putc		;
	call	putc		; putc(hexdig(offseta, file >> 4));
	movf	file,w		;
	hexdig	'A'		;
	pagesel	putc		;
	call	putc		; putc(hexdig(offseta, file));
	endm			;}

defprnx	macro	fsrnum,putc	;void funcname(uint8_t fsr, (*putc)(), uint8_t w) {
	local	fsrn		;
	if (fsrnum & 3)
fsrn set 1
	else
fsrn set 0
	endif
	movwf	FSR#v(fsrn)L	;
	printx	FSR#v(fsrn)L,putc;
	endm

defprns	macro	file,prnx,putc	;
	movlw	'+'
	btfsc	file,7
	movlw	'-'
	pagesel	putc
	call	putc
	movlw	'0'
	pagesel	putc
	call	putc
	movlw	'x'
	pagesel	putc
	call	putc
	movf	file,w
	btfsc	file,7
	sublw	0
	pagesel	prnx
	call	prnx
	endm			;
	
printw	macro	msb,num,fsrnum	;inline void printw(int8_t* msb, uint8_t num,
	local	fsrn		;                   int1_t fsrnum) {
	if (fsrnum & 3)
fsrn set 1
	else
fsrn set 0
	endif
	local i		; uint8_t i;
i = num
	while	i > 0		; for (i = num; i > 0; i--) {
	if (((msb+i-1) & 0x70) != 0x70)
	banksel	msb+i-1		;  int8_t w;
	endif
	movf	msb+i-1,w	;  w = msb[num+i-1];
	pagesel	print#v(fsrn)x	;  ( fsrnum&3 ? print1x : print0x )(w);
	call	print#v(fsrn)x	; }
i -= 1
	endw
	endm			;}

print0w	macro	msb,num		;inline void print0w(int8_t* msb, int8_t num) {
	printw	msb,num,FSR0	; printw(msb, num, FSR0);
	endm			;}
	
