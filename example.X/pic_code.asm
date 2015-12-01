;;; 3 Dec 2014
;;; 6 Jul 2015 resumed
;;; 4 Aug 2015 RS232 problems solved
;;; 19 Aug 2015 forked from multiply demo
;;; 25 Aug 2015 re-merged with multiply demo for the shell
;;;            __________ __________
;;;    _      |          U          |
;;;    |__VDD_|1                   N|___
;;;           |                     |   v
;;;        TX_|2 o (RA5) (AN0) o N-1|_ICSPDAT/DACVMAG
;;;           |                     |
;;;        Vy_|3 i (AN3) (AN1) i N-2|_ICSPCLK/RX
;;;           |                     |
;;;     /MCLR_|4 i       (AN2) i N-3|_Vx
;;;           |                     |
;;;   LED320D_|5 o (RC5) (RC1) o N-4|_LED0D
;;;           |                     |
;;;   LED256D_|6 o (RC4) (RC2) o N-5|_LED64D
;;;           |                     |
;;;   LED192D_|7 o (RC3) (RC2) o N-6|_LED128D
;;;           |                     |
;;;           .                     .
;;;           .                     .
;;; Vx and Vy sampled continuously at 8-bit resolution as signed integers about
;;; half-scale, roughly 100kHz
;;;
;;; DACVMAG calculated continuously as sqrt(Vx*Vx+Vy*Vy) and output to DAC pin
	
;	processor   16f1614	; // N = 14 pins, 4 kword, 512 byte
;	processor   16f1615	; // N = 14 pins, 8 kword, 1024 byte
;	processor   16f1618	; // N = 20 pins, 4 kword, 512 byte
	processor   16f1619 	; // N = 20 pins, 8 kword, 1024 byte
	__CONFIG	_CONFIG1,_FOSC_INTOSC & _WDTE_OFF & _PWRTE_OFF & _MCLRE_ON & _CP_OFF & _BOREN_OFF & _CLKOUTEN_OFF

#ifdef __16F1614 ;{
	include p16f1614.inc
#endif ;}
#ifdef __16F1615 ;{
	include p16f1615.inc
#endif ;}
#ifdef __16F1618 ;{
	include p16f1618.inc
#endif ;}
#ifdef __16F1619 ;{
	include p16f1619.inc
#endif ;}

#define VERBOSE
#define RS232ECHO

#include ../sqrt_acc.inc
#include ../atan_acc.inc
#include ../rs232lib.inc

;;; pin assignments
UART_TX equ (RA5)		; transmit output
AN3VY_B equ (RA4)		; analog input
NOTMCLR equ (RA3)		; reset input
AN2VX_B equ (RA2)		; analog input
UART_RX equ (RA1)		; digital input
DACVMAG equ (RA0)		; analog output

;;; variables in general-purpose (banked) SRAM
CIRC256	equ (0x000)		; 0x020~06f,0a0~0ef,120~16f,1a0~1ef,220~22f
SCRATCH equ (0x230)
OPP	equ (0x231)
ADJ	equ (0x232)
	
;;; variables in common (unbanked) SRAM
SAMPLES equ (0x70)
XSAMPLE equ (0x70)
XS equ XSAMPLE
DSTBASE equ (0x71)  ; // top 4 bits point to ARG, bottom 4 bits are base/2
YSAMPLE equ (0x72)
YS equ YSAMPLE
CHAR_IO equ (0x73)
ACCUMUL equ (0x74)
ACCUMUH equ (0x75)
RESULT	equ (0x76)
RESULTH	equ (0x79)
ARGSABC	equ (0x7a-2); // offset for LSB:MSB channels, which are are numbered:
ARG_A	equ (0x7a)  ; 1
ARG_AL	equ (ARG_A)
ARG_AH	equ (1+ARG_AL)
ARG_B	equ (0x7c)  ; through
ARG_BL	equ (ARG_B)
ARG_BH	equ (1+ARG_BL)
ARG_C	equ (0x7e)  ; 3 // simplifies check for FSR0==ARG_C into just incf + btf
ARG_CL	equ (ARG_C)
ARG_CH	equ (1+ARG_CL)

T0COUNT	equ (0xff-0x13-2) ;//0.125us*19=4.5us (cf. 4.37us Tacq + 0.25us margin)

initacc	macro	valreg,basereg	;inline void initacc(uint16_t* valreg, //ACCUMUL
	movf	basereg,w	;                    uint8_t* basereg) //DSTBASE
	andlw	0xf0		; //don't change to 0xe0 for 16-bit alignment!!!
	iorlw	0x0a>>1		; //bit4 contains subtract flag(?)
	movwf	basereg		; *basereg = (*basereg & 0xf0) | (10 >> 1);
	clrf	valreg+1	;
	clrf	valreg		; *valreg = 0;
	endm			;}
	
	org	0x000
rstvec
	pagesel	main		;
	goto	main		;

	org	0x004
irqvec	
	btfss	INTCON,T0IF	;void irqvec(void) {
	bra	testadc		; if ((INTCON & (1<<T0IF))
	btfss	INTCON,T0IE	;     &&
	bra	testadc		;     (INTCON & (1<<T0IE)) {//settling Tacq over
irqtmr0
	bcf	INTCON,T0IF	;  INTCON &= ~(1<<T0IF); // clear irq trigger,
	bcf	INTCON,T0IE	;  INTCON &= ~(1<<T0IE); // don't allow another
	banksel	PIR1		;
	bcf	PIR1,ADIF	;  PIR1 &= ~(1<<ADIF); // prevent false trigger
	banksel	ADCON0		;
	bsf	ADCON0,1	;  ADCON0 |= 1<<GO_NOT_DONE; // start conversion
	retfie			;

ODDCHAN equ (1<<CHS0)

testadc
	banksel	PIE1		;
	btfss	PIE1,ADIE	;
	bra	test232		;
	banksel	PIR1		;
	btfss	PIR1,ADIF	; } else if ((PIE1 & (1<<ADIE)) &&
	bra	test232		;            (PIR1 & (1<<ADIF))) { // conv ready
irqadc
	bcf	PIR1,ADIF	;  PIR1 &= ~(1<<ADIF); // clear irq trigger
	banksel	ADCON0		;
	rrf	ADCON0,w	;
	andlw	ODDCHAN>>1	;
	iorlw	SAMPLES		;
	movwf	FSR0L		;
	clrf	FSR0H		;  fsr[0] = SAMPLES + 2&(ADCON0>>(CHS0-1));

	movf	ADRESH,w	;  // copy high 8 bits only
	movwf	INDF0		;  if ( (*fsr[0] = *ADRESH) == 0x00 )
	btfsc	STATUS,Z	;   *fsr[0] = 0x01; /* 0x00,01 -> 0x81 */
	incf	INDF0,f		;                   /* 0x7f -> 0xff   */
	movlw	0x80		;                   /* 0x80 -> 0x00  */
	xorwf	INDF0,f		;  *fsr[0] ^= 0x80; /* 0xff -> 0x7f */
	movlw	ODDCHAN		;
	xorwf	ADCON0,f	;  ADCON0 ^= ODDCHAN;

	banksel	TMR0		;  
	movlw	T0COUNT		;  TMR0 = T0COUNT;
	movwf	TMR0		;  INTCON &= ~(1<<T0IF);// restart timer count

	bcf	INTCON,T0IF	;  INTCON |= 1<<T0IE;// for settling time Tacq
	bsf	INTCON,T0IE	; }
	retfie			;}

test232
#ifdef RS232ECHO
	banksel	PIR1		; } else /* if (PIR1 & (1<<RCIF)) */ {
	bcf	PIR1,RCIF	;  PIR1 &= (1<<RCIF);
	banksel	RCREG		;
	movf	RCREG,w		;  w = RCREG; // this read removes it from FIFO
echo_ch
	xorlw	0x20		;
	btfsc	STATUS,Z	;  if (w == ' ') // spaces confuse the syntax
	retfie			;   return;
	xorlw	0x20^0x2a	;
	btfsc	STATUS,Z	;  else if (w == '*')
	movlw	0x29^0x2a	;   w = ')'
	xorlw	0x2a		;   
	pagesel	putch0		;  
	call	putch0		;  CHAR_IO = w;
	btfsc	STATUS,Z     	;  if (!CHAR_IO) 
brkchr
	bra	brkchr		;   goto rstvec; // watchdog reset
	xorlw	0x3f		;
	btfss	STATUS,Z	;
	bra	polmode		;  else if (CHAR_IO = '?') { // dump registers

	movlw	'\b'		;
	pagesel	putch0		;
	call	putch0		;
	movf	XSAMPLE,w	;
	pagesel	print0s		;
	call	print0s		;
	movlw	','		;
	pagesel	putch0		;
	call	putch0		;
	movf	YSAMPLE,w	;
	pagesel	print0s		;
	call	print0s		;
	pagesel	print0		;
	call	print0		;
	dt	") = 0x",0	;
	banksel	DAC1CON1	;   printf("(%c0x%X,%c0x%X) = 0x%X",
	movf	DAC1CON1,w	;       (XS>=0) ? '+' : '-', abs(XS),
	pagesel	print0x		;       (YS>=0) ? '+' : '-', abs(YS),
	call	print0x		;       DAC1CON1);
	
	pagesel	print0		;
	call	print0		;
	dt	" @0x",0	;
	normal	YS,XS,RESULT	;
doatan
	pagesel	atantbl		;   doatan:
	callw			;
	movwf	RESULT		;   RESULT = atantbl[normal(&XS,&YS,&RESULT)];
	movlw	'1'		;
	pagesel	putch0		;
	btfsc	RESULT,7	;
	call	putch0		;
	lslf	RESULT,w	;
	pagesel	print0x		;   printf("@ 0x%c%X", (RESULT>=128)?'1':'\0',
	call	print0x		;          RESULT << 1);
#if 0
	pagesel	print0		;
	call	print0		;
	dt	"=atan(",0
	movf	YSAMPLE,w	;
	pagesel print0s		;
	call	print0s		;
	movlw	'/'		;
	pagesel	putch0		;
	call	putch0		;
	movf	XSAMPLE,w	;
	pagesel	print0s		;
	call	print0s		;   printf("=atan(%c0x%X/%c0x%X)\r\n"
	movlw	')'		;
	pagesel putch0		;
	call	putch0		;
#endif
	pagesel	print0		;       (YS>=0) ? '+' : '-', abs(YS),
	call	print0		;       (XS>=0) ? '+' : '-', abs(XS));
	dt	"\r\n",0
	bra	pprompt		;   goto pprompt;
polmode
	movf	CHAR_IO,w	;  } else if (CHAR_IO == ',') { // (x,y) coord
	xorlw	','		;
	btfsc	STATUS,Z	;
	bra	termsep		;   goto termsep;
	movf	CHAR_IO,w	;
	xorlw	'|'		;
	btfss	STATUS,Z	;
	bra	angmode		;  } else if (CHAR_IO == '|') { // magnitude
	pagesel	print0		;
	call	print0		;
	dt	" = 0x",0
	hypoten	ARG_AL,ARG_BL,sqrttbl
	pagesel print0x		;   // write a second magnitude bar before '('
	call	print0x		;   printf(" = 0x%X\rMathACC|\r\rn",
	pagesel	print0		;    w = hypoten(ARG_AL,ARG_BL,sqrttbl));
	call	print0		;   goto pprompt;
	dt	"\rMathACC|\r\n",0
	bra	pprompt		;
angmode
	movf	CHAR_IO,w	;
	xorlw	'@'		;
	btfss	STATUS,Z	;
	bra	escmode		;  } else if (CHAR_IO == '@') { // arctangent
	pagesel	print0		;   printf("0x");
	call	print0		;   w=normal(ARG_BL,ARG_CL,ACCUMUL /*scratch*/);
	dt	"0x",0
	normal	ARG_BL,ARG_AL,ACCUMUL
	bra	doatan		;   goto doatan;
escmode
	movf	CHAR_IO,w	;
	xorlw	0x7f		;
	btfss	STATUS,Z	;
	bra	entmode		;  } else if (CHAR_IO == '\b') { // clear line
pprompt
	pagesel	print0		;pprompt:
	call	print0		;   printf("\r\n");
	dt	"\r\n",0
	pagesel	printpr		;
	call	printpr		;   printpr();
	movlw	ARG_A<<4	;
	movwf	DSTBASE		;   DSTBASE = ARG_A<<4;
	initacc	ACCUMUL,DSTBASE	;   initacc(&ACCUMUL,&DSTBASE);
	retfie			;   return;
entmode
	movf	CHAR_IO,w	;
	xorlw	0x0d		;
	btfsc	STATUS,Z	;
	bra	equmode		;
	movf	CHAR_IO,w	;
	xorlw	0x3d		;
	btfss	STATUS,Z	; } else if ((CHAR_IO == '\r') ||
	bra	hexmode		;            (CHAR_IO == '=')) { // accept line
	movlw	0x0d		;  if (CHAR_IO == '=')
	pagesel	putch0		;   putchar('\r');
	call	putch0		;
equmode
	movlw	0x0a		;
	pagesel	putch0		;
	call	putch0		;  putchar('\n');
	movf	DSTBASE,w	;
	andlw	0xe0		;
	xorlw	ARG_A<<4	;
	btfss	STATUS,Z	;
	bra	nprompt		;
	movf	ACCUMUL,w	;
	btfss	STATUS,Z	;
	bra	nprompt		;
	movf	ACCUMUH,w	;  if ( (0x70|(DSTBASE>>4)) == ARG_A &&
	btfss	STATUS,Z	;       *((uint16_t*)ACCUMUL) == 0) { // no line
	bra	nprompt		;
	clrf	ARG_A		;   ARG_A = (uint16_t) 0;
	clrf	1+ARG_A		;
	clrf	ARG_B		;   ARG_B = (uint16_t) 0;
	clrf	1+ARG_B		;
	clrf	ARG_C		;   ARG_C = (uint16_t) 0;
	clrf	1+ARG_C		;   goto pprompt;
	bra	pprompt		;  }
nprompt
	movlw	0xe5		; // force ACCUMU[HL] into last arg (0x7e)
	movwf	DSTBASE		;  DSTBASE = (ARG_C << 4) | (/*base*/ 10 >> 1);
	bra	nextarg		;  goto nextarg;
hexmode
	movf	CHAR_IO,w	;
	xorlw	0x78		;
	btfss	STATUS,Z	;
	bra	delimit 	; } else if (CHAR_IO == 'x') { // hex mode
	movf	DSTBASE,w	;
	andlw	0x0f		;
	xorlw	0x08>>1		;
	movlw	0x18>>1		;
	btfsc	STATUS,Z	;  if ((DSTBASE << 1)&31 == 8) // base 8, not 10
	xorwf	DSTBASE,f	;   DSTBASE = (DSTBASE&0xf0) | (16>>1);
	retfie			;
delimit
	movf	CHAR_IO,w	; } else if (CHAR_IO == '-') { // negate ARG_B
	xorlw	'-'		;
	btfss	STATUS,Z	;
	bra	delplus		;// unused bit 4 indicates next quantity negated
	bsf	DSTBASE,4	;  DSTBASE |= 0x10;
	bra	termsep		;  goto termsep;
delplus
	movf	CHAR_IO,w	;
	xorlw	'+'		;
	btfss	STATUS,Z	;
	bra	dellast		; } else if (CHAR_IO == '+') {
termsep
	movf	DSTBASE,w	;  termsep: w = DSTBASE;
nextarg
	andlw	0xe0		;  nextarg:
	iorlw	0x07		;  // w assumed to contain (0x7_<<4) | (base>>1)
	movwf	FSR0L		;
	swapf	FSR0L,f		;
	clrf	FSR0H		;  FSR0 = (uint16_t*) (0x70 | (DSTBASE >> 4));

	btfss	DSTBASE,4	;  if ( (DSTBASE & (1<<4)) /* minus seen */ &&
	bra	nonneg		;
	btfsc	DSTBASE,5	;       ((0x70 | (DSTBASE>>4)) == (ARG_B|1)) ) {
	bra	nonneg		;   // negate this (ARG_B) argument
	comf	ACCUMUL,f	;
	movlw	1		;
	addwf	ACCUMUL,f	;
	comf	ACCUMUH,f	;
	movlw	0		;
	addwfc	ACCUMUH,f	;   (uint16_t*) ACCUMUL *= -1;
nonneg
	movf	ACCUMUL,w	;
	movwi	FSR0++		;
	movf	ACCUMUH,w	;
	movwi	FSR0++		;  *FSR0++ = (ACCUMUH << 8) | ACCUMUL;

	initacc	ACCUMUL,DSTBASE	;  initacc(&ACCUMUL,&DSTBASE);
	btfss	FSR0L,7		;
	retfie			;
	
	movlw	0xa5		;  if (FSR0 > ARG_C) {
	movwf   DSTBASE		;   DSTBASE = (ARG_A<<4) | (/*base*/ 10 >> 1);
	initacc	ACCUMUL,DSTBASE	;   initacc(&ACCUMUL,&DSTBASE);

	pagesel print0		;
	call	print0		;   printf("0x");
	dt	"0x",0

	sqsetup	PID1CON,0,0	;   sqsetup(PID1CON, 0/*nonacc.*/, 0/*signed*/);
	banksel	PID1INL		;

	movf	ARG_CH,w	;
	movwf	PID1K1H		;
	movf	ARG_CL,w	;
	btfsc	STATUS,Z	;
	movf	ARG_CH,f	;
	btfsc	STATUS,Z	;
	movlw	0x01		;
	movwf	ARG_CL		;
	movwf	PID1K1L		;

	movf	ARG_BH,w	;
	movwf	PID1SETH	;
	movf	ARG_BL,w	;
	movwf	PID1SETL	;
	
	movf	ARG_AH,w	;   if (ARG_C) // multiplying by zero is useless
	movwf	PID1INH		;    PID1ACC = ARG_C * (ARG_B + ARG_A);
	movf	ARG_AL,w	;   else // ...so change C to 1 if it had been 0
	movwf	PID1INL		;    PID1ACC = (ARG_C = 1) * (ARG_B + ARG_A);

	banksel	PID1CON		;
	clrwdt			;   do {
	btfsc	PID1CON,PID1BUSY;    clrwdt();
	bra	$-2		;   } while (PID1CON & (1<<PID1BUSY));
	
	movf	PID1ACCHH,w	;
	movwf	RESULT+3	;
	movf	PID1ACCHL,w	;
	movwf	RESULT+2	;
	movf	PID1ACCLH,w	;
	movwf	RESULT+1	;
	movf	PID1ACCLL,w	;
	movwf	RESULT+0	;  *((int32_t*)RESULT) = *((int32_t*)PID1ACC);

	print0w	RESULT,4	;   printf("%x%x%x%x%x"
	pagesel	print0		;          " = (0x%x%x + 0x%x%x) * 0x%x%x\r\n",
	call	print0		;        (int32_t) RESULT,
	dt	" = (0x",0
	print0w	ARG_A,2		;        (int16_t) ARG_A,
	pagesel	print0		;
	call	print0		;
	dt	" + 0x",0
	print0w	ARG_B,2		;        (int16_t) ARG_B,
	pagesel	print0		;
	call	print0		;
	dt	") * 0x",0
	print0w	ARG_C,2		;        (int16_t) ARG_C);

	bra	pprompt		;
	retfie			;  }

dellast
	movf	CHAR_IO,w	; } else if ((CHAR_IO == ')') { // stash in B
	xorlw	0x29		;
	btfss	STATUS,Z	;
	bra	sqrtmode	;
	movlw	0xe0	 	;
	andwf	DSTBASE,w	;
	xorlw	0xe0		;
	btfsc	STATUS,Z	;
	bra	bnonzero	;  if ((DSTBASE & 0xe0) != ARG_C)
	clrf	ARG_B		;
	clrf	ARG_B+1		;   ARG_B = 0;
bnonzero
	movlw	0xc5		;  if (DSTBASE & (1<<4))
	btfsc	DSTBASE,4	;   DSTBASE = (ARG_B<<4) | (1 << 4) | (10 >> 1);
	movlw	0xd5		;  else
	movwf	DSTBASE		;   DSTBASE = (ARG_B<<4) | (/* base */ 10 >> 1);
	bra	nextarg		;  goto nextarg;
	
sqrtmode
	movf	CHAR_IO,w	; } else if (CHAR_IO == '/') { // square root
	xorlw	'/'		;
	btfss	STATUS,Z	;
	bra	squamode	;
	pagesel	sqrt		;
	call	sqrt		;  uint8_t w = sqrt(*((uint16_t*) ACCUMUL));
	movwf	ACCUMUL		;
	clrf	ACCUMUH		;  *((uint16_t*)ACCUMUL) = (uint16_t) w;
	retfie			; //fixme: goto nextarg and stop accepting chars

squamode
	movf	CHAR_IO,w	;} else if (CHAR_IO == '^') { // square
	xorlw	'^'		;
	btfss	STATUS,Z	;
	bra	facmode		;
	pagesel	squareal	;
	call	squareal	; squareal((uint16_t*)ACCUMUL);
	retfie			; //fixme: goto nextarg and stop accepting chars

facmode
	movf	CHAR_IO,w	;
	xorlw	'!'		;
	btfss	STATUS,Z	;
	bra	digmode		; } else if (CHAR_IO == '!') {
	bra	facto		;  goto facto; // will retfie from there
digmode
	movlw	0-0x30		;
	addwf	CHAR_IO,f	;
	btfsc	CHAR_IO,7	;
	retfie			; } else if ((CHAR_IO -= '0') >= 0) { // digit
	movf	CHAR_IO,w	;
	btfsc	CHAR_IO,4	;  if (CHAR_IO & 0x10)
	addlw	0x09		;   CHAR_IO += 9; // digit A-F or a-f
	andlw	0x0f		;
	movwf	CHAR_IO		;  CHAR_IO &= 0x0f;  
	btfss	STATUS,Z	;
	bra	digit16		;  if ((CHAR_IO == 0) && // zero digit was
	movf	ACCUMUH,f	;
	btfss	STATUS,Z	;
	bra	digit16		;      (ACCUMUH == 0) &&
	movf	ACCUMUL,f	;
	btfss	STATUS,Z	;
	bra	digit16		;      (ACCUMUL == 0)) // a leading zero
	bcf	DSTBASE,0	;   DSTBASE &= ~(2>>1); // exit decimal mode
	retfie			;
digit16
	btfss	DSTBASE,3	;
	bra	digit10		;  else if (DSTBASE & (16>>1)) {
	swapf	ACCUMUH,w	;
	andlw	0xf0		;
	movwf	ACCUMUH		;   ACCUMUH <<= 4;
	swapf	ACCUMUL,w	;
	andlw	0x0f		;
	iorwf	ACCUMUH,f	;   ACCUMUH |= (ACCUMUL >> 4) & 0x0f;
	swapf	ACCUMUL,w	;
	andlw	0xf0		;   ACCUMUL <<= 4;
	iorwf	CHAR_IO,w	;
	movwf	ACCUMUL		;   ACCUMUL |= CHAR_IO;
	retfie			;  } else {
digit10
	lslf	ACCUMUL,w	;   uint16_t ACCUMU = (ACCUMUH << 8) | ACCUMUL;
	movwf	FSR0L		;
	rlf	ACCUMUH,w	;
	movwf	FSR0H		;   FSR0 = ACCUMU * 2;
	lslf	ACCUMUL,f	;
	rlf	ACCUMUH,f	;
	lslf	ACCUMUL,f	;
	rlf	ACCUMUH,f	;
	lslf	ACCUMUL,f	;
	rlf	ACCUMUH,f	;   ACCUMU *= 8;
	btfss	DSTBASE,0	;
	bra	digit8		;
	movf	FSR0L,w		;
	addwf	ACCUMUL,f	;
	movf	FSR0H,w		;   if (DSTBASE & (2>>1))
	addwfc	ACCUMUH,f	;    ACCUMU += FSR0;
digit8
	movf	CHAR_IO,w	;   ACCUMU += CHAR_IO;
	addwf	ACCUMUL,f	;
	clrw			;   ACCUMUL = ACCUMU & 0x00ff;
	addwfc	ACCUMUH,f	;   ACCUMUH = ACCUMU >> 8;
#endif ; // RS232ECHO		;  }
	retfie			;  retfie();

facto
	movlw	low ACCUMUL	;facto:
	movwf	FSR0L		;
	clrf	FSR0H		; fsr0 = (uint16t*) &ACCUMUL;
	movf	INDF0,w		;
	btfss	STATUS,Z	;
	decf	INDF0,w		;
	lut16w	facto,7,FSR0,1	; *fsr0 = factorial(*fsr ? (*fsr-1) : 0);
	variable i,x
i=0
x=1
	while	i<8
facto#v(i)
i+=1
x*=i
	 if (x>255)
	  movlw	high x		;
	  movwf	INDF0		;
	 endif
	 retlw	low x		; retfie;
	endw

	;; debug output subroutines (defined using macros in rs232lib.inc)
	;;
	;; the FSR0 ("0") pointer is used for reading characters out of program
	;; memory and is available for other purposes between calls
	;;
	;; the FSR1 ("1") pointer must stay pointing into the circular SRAM
	;; in order for the output to be decipherable, but if that ability isn't
	;; crucial FSR1 also may be used for other purposes (the routines make
	;; sure it can't point anywhere but linear data memory, e.g. at the
	;; critical system variables)
	;; 
	;; putch0 writes the ASCII character in w to a circular buffer in SRAM
	;; (and also spins until there is room in the RS232 write buffer if
	;; RS232ECHO is defined), such as:
	;;	movlw	'\n'
	;; 	call	putch0
	;;
	;; print0 and print1 each print the null-terminated string defined by
	;; the lower 8 bits of successive locations of program memory, returning
	;; control upon exiting to the instruction after the '\0' character:
	;;	call	print0
	;; 	dt	"hello world\n",0
	;; 	movlw	...
	;;
	;; print0x and print1x each print the two-character (0-9,A-F) hex
	;; representation of the byte value in w:
	;; 	movlw	0xff
	;; 	call	print0x
putch0
	defputc	0,CHAR_IO	;char putch0(char w); // also sets CHAR_IO to w
	return			;// puts a single character out on the TX buffer
print0
	defprnt	0,putch0	;void print0(char** STKPTR); // loops putch0 and
	return			;// prints \0-terminated string embedded in code
print0x
	defprnx	0,putch0	;void print0x(uint8_t w);
	return			;// prints as sign-agnostic hexadecimal 0..0xff
print0s
	movwf	RESULT		;void print0s(int8_t w);
	defprns	RESULT,print0x,putch0
	return			;// prints + or -, absolute hex value 0..0x80
printpr
	pagesel	print0		;void printpr(void) {
	call	print0		; printf("MathACC (");
	dt	"MathACC (",0
	return			;}

PINDIRA equ (0<<UART_TX)|(1<<AN3VY_B)|(1<<NOTMCLR)|(1<<AN2VX_B)|(1<<UART_RX)|0
ANALOGA equ (0<<UART_TX)|(1<<AN3VY_B)|(0<<NOTMCLR)|(1<<AN2VX_B)|(0<<UART_RX)|1
	
DAC1_ON	equ (1<<DAC1EN)|(1<<DAC1OE)|(0<<D1PSS1)
	
ADCHANY equ ((1<<ADON)|(3<<CHS0))
ADCFAST	equ (7<<ADCS0)
IN32MHZ	equ 0x80|(1<<IRCF3)|(1<<IRCF2)|(1<<IRCF1)|(1<<IRCF0)|(0<<SCS1)|(0<<SCS0)

T0PSMSK equ (0|0|(1<<TMR0CS)|0|(1<<PSA)|(7<<PS0_OPTION_REG))
T0PSOFF	equ (0|0|(0<<TMR0CS)|0|(0<<PSA)|(0<<PS0_OPTION_REG))

main
	banksel	TRISA		;void main(void) {
	movlw	PINDIRA		;
	movwf	TRISA		; TRISA = PINDIRA;
	clrf	TRISC		; TRISC = 0;

IOBANKA	equ (0)
IOBANKB	equ (1)
IOBANKC	equ (2)
A1RXPIN	equ ((IOBANKA<<RXPPS3)|1)
	
	banksel	RXPPS		;
	movlw	A1RXPIN		;
	movwf	RXPPS		; RXPPS = A1RXPIN; // UART RX on pin RA1

TXPINID equ (0x12) ; // Table 13-2
	
	banksel	RA5PPS	     	;
	movlw	TXPINID		;
	movwf	RA5PPS		; RA0PPS = TXPINID; // UART TX on pin RA0

	banksel	OSCCON		; // select and stabilize the oscillator
	movlw	IN32MHZ		; // no improvement in MATH ACC speed above 8MHz
	movwf	OSCCON		; OSCCON = IN32MHZ;
#ifndef __DEBUG
wait_hf
	btfsc	OSCSTAT,HFIOFR	; while ((OSCSTAT & (1<<HFIOFR) == 0) ||
	btfss	OSCSTAT,HFIOFS	;        (OSCSTAT & (1<<HFIOFS) == 0))
	bra	wait_hf		;  ; 
#endif

	;; wait out inevitable garbage TX (not sure why this delay is necessary)
	clrf	ACCUMUL
	clrf	ACCUMUH
delay0
	incfsz	ACCUMUH,f
	bra	delay0
	incfsz	ACCUMUL,f
	bra	delay0
	
	banksel	BAUD1CON	; // RS232 receive (following 25.1.2.8 setup):
	;; 
BAUDMSK	equ 0|0|0|0|(1<<BRG16)|0|0|0
BAUD16B	equ 0|0|0|0|(1<<BRG16)|0|0|0
TXSTMSK	equ 0|0|0|(1<<SYNC)|0|(1<<BRGH)|0|0
TXST16B	equ 0|0|0|(0<<SYNC)|0|(1<<BRGH)|0|0
	
#ifdef HALFSPEED
B115200	equ 0x0022		; // (16MHz/115200bps)/4 - 1, from Table 25-5
B9600	equ 0x01a0		; // (16MHz/9600bps)/4 - 1, from Table 25-5
#else
B115200	equ 0x0045		; // (32MHz/115200bps)/4 - 1 (Table 25-5)
B9600	equ 0x0340		; // (32MHz/9600bps)/4 - 1 (Table 25-5)
#endif
	bcf	RC1STA,SPEN	;
	bcf	RC1STA,CREN	; RC1STA &= ~((1<<SPEN)|(1<<CREN));
	bcf	TXSTA,TXEN	; TXSTA &= ~(1<<TXEN);
	movlw	~BAUDMSK	;
	andwf	BAUD1CON,w	; // (1) "Initialize..the desired baud rate"
	iorlw	BAUD16B		;
	movwf	BAUD1CON	; BAUD1CON = (BAUD1CON & ~BAUDMSK) | BAUD16B;
	movlw	~TXSTMSK	;
	andwf	TXSTA,w		;
	iorlw	TXST16B		;
	movwf	TXSTA		; TXSTA = (TXSTA & ~TXSTMSK) | TXST16B;
	movlw	low (B115200)	;
	movwf	SPBRGL		;
	movlw	high (B115200)	;
	movwf	SPBRGH		; SPBRG = B115200;
	bsf	RC1STA,SPEN	; // (3) "Enable..by setting..SPEN"
	bcf	RC1STA,RX9	; RC1STA &= ~(1<<RX9);  // (5) "9-bit..set..RX9"
	bsf	RC1STA,CREN	; RC1STA |= (1<<SPEN) | (1<<CREN); // (6) "CREN" 
	bcf	BAUD1CON,SCKP	; BAUDCON &= ~(1<<SCKP); // "SCKP..if inverted"

	bsf	TX1STA,TXEN	; // RS232 transmit (following 23.1.1.7 setup):
	banksel	PIE1		; TXSTA |= 1<<TXEN; // (5) "Enable..by..TXEN"
	bsf	PIE1,RCIE	; PIE1 |= 1<<RCIE; //(4) "Set..RCIE..and..PEIE"

	clrf	FSR1L		;
	movlw	0x20		;
	movwf	FSR1H		; fsr1 = 0x2000 | (fsr1 & 0x00ff); // circ. log
	
	btfsc	STATUS,NOT_TO	;
	bra	prinit		;
	pagesel	print0		;
	call	print0		;
	dt	"\r\nReset!",0
prinit
#ifdef VERBOSE
	pagesel	print0		;
	call	print0		; printf("\r\nInitializing hardware: ");
	dt	"\r\nInitializing hardware: ",0
#endif

	banksel	ADCON0		; // set up the ADC to start on chan 1 (Vy)
	movlw	ADCHANY    	;
	movwf	ADCON0		; ADCON0 = ADCHANY;
	movlw	ADCFAST		;
	movwf	ADCON1		; ADCON1 = 0x30; // left-justified, internal RC 
	bsf	PIE1,ADIE	; PIE1 |= 1<<ADIE; // enable ADC IRQ (same bank)
#ifdef VERBOSE
	pagesel	print0		;
	call	print0		; printf("ADC ");
	dt	"ADC ",0
#endif

	banksel	ANSELA		;
	movlw	ANALOGA		;
	movwf	ANSELA		; ANSELA = ANALOGA;
	clrf	ANSELC		; ANSELC = 0;
	banksel	DAC1CON0	;
	movlw	DAC1_ON		;
 	movwf	DAC1CON0	; DAC1CON0 = DAC1_ON;
	clrf	DAC1CON1	; DAC1CON1 = 0;
#ifdef VERBOSE
	pagesel	print0		;
	call	print0		; printf("DAC ");
	dt	"DAC ",0
#endif

	banksel	OPTION_REG	; // Timer0: Fosc/4 (8MHz), 125ns, 32us overflow
	movlw	~T0PSMSK	;
	andwf	OPTION_REG,w	;
	iorlw	T0PSOFF		;
	movwf	OPTION_REG	; OPTION_REG = (OPTION_REG & ~T0PSMSK)| T0PSOFF;
	bsf	INTCON,T0IE	; INTCON &= ~(1<<T0IE); // enable overflow IRQ
#ifdef VERBOSE
	pagesel	print0		;
	call	print0		; printf("TMR0\r\n");
	dt	"TMR0\r\n\n"
	dt	"PIC16F161x math acceleration console command summary\r\n"
	dt	" Positive integers:   MathACC (1+2)3      = 9 (0x09)\r\n"
	dt	" Negative integers:   MathACC (-3)3=(0-3)3=-9 (0xf7)\r\n"
	dt	" Mixed dec/oct/hex:   MathACC (15-0xe)011 = 9 (0x09)\r\n"
	dt	" Squared arguments:   MathACC (1^-2^)3    =-9 (0xf7)\r\n"
	dt	" Square root (<2^15): MathACC (1/+4/)9/   = 9 (0x09)\r\n"
	dt	" Factorials (<8):     MathACC (0!+1!)2!   = 4 (0x04)\r\n"
	dt	" Vector length:       MathACC (4,3)|      = 5 (0x05)\r\n"
	dt	" Vector angle:        MathACC (4,3)@      =36 (0x24)\r\n\r\n"
	dt	"Only blank lines clear memory (good for repeat operations)\r\n"
	dt	"Send a terminal BRK character to force a full device reset\r\n"
	dt	"Type '?' to display the polar-converted quadrature samples\r\n"
	dt	0
#endif
	pagesel	print0		;
	call	print0		;
	dt	"\r\n",0
	pagesel	printpr	    	;
	call	printpr		;
	
	bsf	INTCON,PEIE	; INTCON |= 1<<PEIE; // periph. interrupts on
	bsf	INTCON,GIE	; INTCON |= 1<<GIE; // all interrupts on
	
loop	clrwdt			; while (1) {
	hypoten	XS,YS,sqrttbl	;  clrwdt();
	banksel	DAC1CON1	;  DAC1CON1 = hypoten(XSAMPLE,YSAMPLE,sqrttbl);
	movwf	DAC1CON1	;

	banksel	SCRATCH		;
	movf	XS,w		;
	movwf	ADJ		;  ADJ = XSAMPLE;
	movf	YS,w		;
	movwf	OPP		;  OPP = YSAMPLE;
	normal	OPP,ADJ,SCRATCH	;
	pagesel	atantbl		;
	callw			;
	swapf	WREG,w		;
	rrf	WREG,w		;  w = atantbl[normal(&OPP, &ADJ, &SCRATCH)]>>5;

	pagesel	thermo		;
	call	thermo		;
	banksel	LATC		;  LATC = 1<<w; // w < 6
	movwf	LATC		; }
	bra	loop		;} // main()

thermo
	andlw	7		;uint8_t thermo(uint3_t w) {
	brw			;
	retlw	0x01		;
	retlw	0x02		;
	retlw	0x04		;
	retlw	0x08		;
	retlw	0x10		;
	retlw	0x20		;
	retlw	0x40		; return 1<<w;
	retlw	0x80		;}
	
sqrt
	sqsetup	PID1CON,+1,0	;uint8_t sqrt(uint16_t*ACCUMUL,uint16_t*RESULT){
	sqrt16f	ACCUMUL,sqrttbl	;  return w = (uint8_t) sqrt(*ACCUMUL);
	return			;}
	
squareal
	movf	ACCUMUH,f	;void squareal(uint16_t* ACCUMUL) {
	movlw	'2'		;
	btfss	STATUS,Z	;
	movlw	'V'		;
	pagesel	putch0		;
	call	putch0		; putchar((ACCUMUL > 255) ? 'V' : '2');

	sqsetup	PID1CON,0,0	;  // signed multiplications
	movf	ACCUMUL,w	;
	squarew	PID1INL		;
	banksel	PID1CON		;
nosquare
	clrwdt			;
	btfsc	PID1CON,PID1BUSY;
	bra	nosquare	;
	movf	PID1ACCLH,w	;
	movwf	ACCUMUH		;
	movf	PID1ACCLL,w	; 
	movwf	ACCUMUL		; *((uint16_t)ACCUMUL)=(ACCUMUL*ACCUMUL)&0xffff;
	return			;}
	
	org	0x700
atantbl
	include ../atan256.inc
	
	org	0x800
sqrttbl
	include ../sqrt2048.inc
	end
