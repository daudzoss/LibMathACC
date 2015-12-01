;;; sqrt_acc.inc
;;; 
;;; square root macros for 15-bit (with 2k lookup table) or 16-bit (with 4k one)
;;;  integer support, accepting a little-endian file register pair and returning
;;;  truncated square root after with a single multiply/verify after the lookup
;;;
;;; the basic principle is that for an N-bit positive integer, N can be right-
;;;  shifted by one nybble and the resulting N-4 bits used as an index into a
;;;  lookup table with an ambiguity of only 1
;;; 
;;; this holds true since 4^2=16 and thus for y>4 all squares of y will be
;;;  separated by at least 2^4 and adjacent values in the table differ by 1 at
;;;  most
;;; 
;;; a quick comparison with the lookup value squared (using the PIC16F1619 PID
;;;  block) confirms whether the lookup value is the floor or ceiling of the
;;;  real value; this library selects the floor by testing whether the square
;;;  when added to the negative of the input minus one (the bitwise complement)
;;;  using the PID's accumulation mode is still negative or not

;;; example invocation code with 32399, the square root of which is 179.997:
;
;	include sqrt_acc.inc
;
;	movlw	0x8f
;	movwf	file
;	movlw	0x7e
;	movwf	1+file
;	sqsetup	PID1CON,1,1; first 1 means accumulate, second 1 means unsigned
;	sqrt16f	file,table ; w now holds (uint8_t)sqrt(180*180-1) == 179
;
;	org	0x800
; table
;	include tbl.inc
;	end 

;;; sqsetup(pidcon,acc,unsgn)
;;; 
;;; should be called to set the mode of the PID block to one of four possible
;;; settings before calling squarew(), sqrt16f(), sqrt16u(), hypoten() etc.:
;;; 
;;;  acc=0,unsgn=0 non-accumulating signed multiplication
;;;  acc=0,unsgn=1 non-accumulating unsigned multiplication
;;;  acc=1,unsgn=0 accumulating signed multiplication (<-used by sqrt functions)
;;;  acc=1,unsgn=1 accumulating unsigned multiplication
;;;
;;; it spins on the PIDxBUSY bit until the peripheral is done with any previous
;;;  calculation, then sets the mode and cycles the PIDxEN bit in order to clear
;;;  the 33-bit PIDxACC result field to all zeroes
;;;
;;; upon exit the BSR is still set to the bank of the PIDxCON file register
	
sqsetup	macro	pidcon,acc,unsgn;inline void sqsetup(uint8_t* pidcon,
	banksel	pidcon		;                  uint1_t acc, uint1_t unsgn) {
	clrwdt			; while ((*pidcon) & (1<<PIDxBUSY))
	btfsc	pidcon,PID1BUSY	;  clrwdt();
	bra	$-2		; if (!acc)
	clrf	pidcon		;  *pidcon = 0; // mode 000: unsigned non-accum.
	if acc
	 bsf	pidcon,0	; else *pidcon = 1; // mode 001: unsigned accum.
	endif
	if unsgn == 0
	 bsf	pidcon,1	; if (!unsgn) *pidcon |= 2; // signed is 010/011
	endif
	bsf	pidcon,PID1EN	; *pidcon |= 1<<PID1EN; 
	endm			;}
	
;;; squarew(pidparm)
;;;
;;; uses the PID block to square the 8-bit integer in WREG the range
;;;  -128 to 127 (if sqsetup() was invoked with unsgn=0) or
;;;     0 to 255 (if sqsetup() was invoked with unsgn=1)
;;;  and places the result in either of the 16-bit fields:
;;; 
;;;    PIDxACCLH<<8 | PIDxACCLL (if pidparm is odd)
;;;  or
;;;    PIDxACCHH<<8 | PIDxACCHL (if pidparm is odd)
;;; 
;;; it is the responsibility of the caller to wait until the PIDxBUSY bit is
;;;  clear before fetching the result from the above file register(s)
;;; 
;;; upon exit the BSR is still set to the bank of the pidparm file register

squarew	macro	pidparm		;inline void squarew(uint8_t* pidparm, int8_t w)
	banksel	pidparm		;{ // unused input bytes all cleared
	clrf	PID1SETH	; if (pidcon & 1) { // square 256*w instead of w
	clrf	PID1SETL	;  if ((*pidcon) & 1) // note asterisk this time
	if (pidparm & 1)
	 movwf	PID1K1H		;   *((uint16_t*)PIDxACC) += (w * w) << 16;
	 movwf	PID1INH		;  else 
	 clrf	PID1K1L		;   *((uint16_t*)PIDxACC) = (w * w) << 16;
	 clrf	PID1INL		; } else
	else
	 clrf	PID1K1H		; if ((*pidcon) & 1)
	 clrf	PID1INH		;  *((uint16_t*)PIDxACC) += w * w;
	 movwf	PID1K1L		;  else 
	 movwf	PID1INL		;   *((uint16_t*)PIDxACC) = (w * w) << 16;
	endif
	endm			;}

;;; com16f(src,dst)
;;;
;;; flips all bits in the 16-bit word at src, placing the result in the 16-bit
;;;  word at dst
;;;
;;; sqrt16f(x) uses com16f(x,PIDyACCz) to place -x-1 into the accumulator before
;;;  checking the squared lookup value for the square root against x itself
;;;  since if the result is not negative the actual root is known to be one less
;;; WREG incidentally contains the upper 8 bits of the result upon exit
;;; STATUS Z bit incidentally is set if WREG is zero upon exit
;;;
;;; caller should set the BSR for either src or dsr as needed, after which
;;;  bank selection is avoided if unnecessary, but if src and dst are not
;;;  accessible from the same bank the BSR remains set to the dst bank upon exit

com16f	macro	src,dst		;inline int8_t com16f(uint16_t* src,
	local	needbsr		;                     uint16_t* dst) {
needbsr set 0
	if ( ((src&0x70)!=0x70) && ((src&0xf80)!=(dst&0xf80)) && src && dst )
needbsr set 1
	banksel src		; // BSR only needs to be ping-ponged if:
	endif
	comf	src,w	       	; //  1) neither src nor dst is in common RAM,
	if (needbsr)
	banksel dst		; //  2) src and dst aren't in the same bank
	endif
	movwf	dst		; //  and
	if (needbsr)
	banksel src		; //  3) neither src nor dst is the INDF pair
	endif
	comf	1+src,w      	;
	if (needbsr)
	banksel 1+dst		;
	endif
	movwf	1+dst		; return *w = ((*dst = ~*src) >> 8); // b7 sign
	endm			;}

;;; ror16f(bits,arg16)
;;;
;;; rotates the 16-bit little-endian word in two file registers located at arg16
;;;  rightward in place by the indicated number of bits
;;;
;;; as distinct from alternating rrf instructions, which would achieve a 17-bit
;;;  rotation through the carry bit, this wraps the LSB directly into the MSB
;;;
;;; unlike the rrf machine instruction, Z in STATUS is set if all bits of arg16
;;;  are clear
;;; 
;;; sqrt16f(x) uses ror16f(4,x) to separate the 16 bits of its input x into:
;;;  (top nybble) the 4 LSBs of x, used only to distinguish among roots < 4
;;;  (2nd nybble) the 4 MSBs of x, added to the page of the table to set PCLATCH
;;;  (low byte) the middle bits of x, used in a callw instruction to do a lookup
;;; 
;;; WREG is essentially clobbered in the process, as is arg16, but arg16 can be
;;;  restored by a subsequent call to rol16f() by the same number of bits
	
ror16f	macro	bits,arg16	;inline void ror16f(int bits, uint16_t* arg16) {
        if bits == 4
	 swapf	1+arg16,f	;
	 swapf	arg16,f		;
	 movf	1+arg16,w	;
	 xorwf	arg16,w		;
	 andlw	0xf0		;
	 xorwf	1+arg16,f	;
	 xorwf	arg16,f		;
	else
 	 local i
i = 0
	 while i < bits
	  rrf	arg16,w		;
	  rrf	1+arg16,f	;
	  rrf	arg16,f		;
i += 1
	 endw
	endif
	movf	arg16,f		; *arg16=((*arg16)>>bits)|((*arg16)<<(16-bits));
	btfsc	STATUS,Z	;
	movf	1+arg16,f	; return; // Z set appropriately for 16-bit word
	endm			;}

;;; rol16f(bits,arg16)
;;;
;;; rotates the 16-bit little-endian word in two file registers located at arg16
;;;  leftward in place by the indicated number of bits
;;;
;;; as distinct from alternating rlf instructions, which would achieve a 17-bit
;;;  rotation through the carry bit, this wraps the MSB directly into the LSB
;;;
;;; no macros in this file use rol16f() but the caller of sqrt16f() or sqrt16u()
;;;  may wish to call rol16f(4,x) afterward to restore the value x that was sent
;;; 
;;; unlike the rlf machine instruction, Z in STATUS is set if all bits of arg16
;;;  are clear
;;; 
;;; WREG is essentially clobbered in the process, as is arg16, but arg16 can be
;;;  restored by a subsequent call to ror16f() by the same number of bits
	
rol16f	macro	bits,arg16	;inline void rol16f(int bits, uint16_t* arg16) {
        if bits == 4
	 swapf	1+arg16,f	;
	 swapf	arg16,f		;
	 movf	1+arg16,w	;
	 xorwf	arg16,w		;
	 andlw	0x0f		;
	 xorwf	1+arg16,f	;
	 xorwf	arg16		;
	else
	 local i
i = 0
	 while i < bits
	  rlf	1+arg16,w	;
	  rlf	arg16,f		;
	  rlf	1+arg16,f	;
i += 1
	 endw
	endif
	movf	arg16,f		; *arg16=((*arg16)<<bits)|((*arg16)>>(16-bits));
	btfsc	STATUS,Z	;
	movf	1+arg16,f	; return; // Z set appropriately for 16-bit word
	endm			;}

;;; sqrtp4f(arg4,bit0)
;;;
;;; returns in the low 3 bits of WREG the square root of the 4-bit *positive*
;;;  field of the file register arg4, aligned to bit0
;;;
;;; sqrt16f() calls this instead of performing a lookup into the leading entry
;;;  of the root table, since for all radicands other than 0 the value will be
;;;  either the table entry or one less (but there are 4 roots between 0 and 15)
;;; 
;;; note that arg4 must not be zero: sqrtp4f(0,0) will return 1 instead of 0;
;;;  instead call sqrt4f(x,0) if x is not known to be nonzero

sqrtp4f	macro	arg4,bit0	;inline uint3_t sqrtp4f(uint8_t* arg4/*nonzero*/,
	local	maybe2,is2,done	;                       uint3_t bit0/*shift*/) {
	btfss	arg4,3+bit0	; uint4_t field = ((*arg4) >> bit0) & 0xf;
	bra	maybe2		; if (field >= 8) {
	movlw	7<<bit0		;
	andwf	arg4,w		;  if (field == 8)
	btfsc	STATUS,Z	;   return w = 2;
	bra	is2		;  else
	movlw	3		;   return w = 3;
	bra	done		;
maybe2
	movlw	1		; } elseif (field >= 4)
	btfsc	arg4,2+bit0	;  return w = 2;
is2
	movlw	2		; else return w = 1;
done
	endm			;}
	
;;; sqrtf(arg4,bit0)
;;;
;;; returns in the low 3 bits of WREG the square root of the 4-bit non-negative
;;;  field of the file register arg4, aligned to bit0

sqrt4f	macro	arg4,bit0 	;inline uint3_t sqrt4f(uint8_t* arg4/*non-neg.*/,
	local	is0		;                      uint3_t bit0/*shift*/) {
	movlw	0x0f<<bit0	; uint4_t field = ((*arg4) >> bit0) & 0xf;
	andwf	arg4,w		; if (field > 0)
	btfsc	STATUS,Z	;  return sqrtp4f(arg4, bit0);
	bra	is0		; else
	sqrtp4f	arg4,bit0	;  return 0; // Z also set
is0
	endm			;}

;;; sqrt16u(arg16,tbl4k)
;;;
;;; returns in WREG the square root of the 16-bit non-negative little-endian
;;;  integer in the two file registers beginning at arg16, using the 2k-aligned
;;;  2048-lookup table starting at tbl4k
;;;
;;; each entry N in the table contains the square root of (N<<4)|0x000f so after
;;;  looking up the square root of (arg16&0xfff0)|0x000f this value is squared
;;;   and compared against arg16 (in the same step, thanks to the accumulate
;;;   mode of the PID and a pre-load of the complement of arg16) compared to
;;;   arg16
;;;
;;; if larger, the guess (still in WREG) is decremented in order to implement a
;;;  floor(sqrt(x))
;;;
;;; arg16 is clobbered by ror16f(arg16); it is the caller's responsibility to
;;;   restore it if desired with a call to rol16f(arg16)
	
sqrt16u macro	arg16,tbl4k	;inline uint8_t sqrt16u(uint16_t* arg16, const
	local	not0,busy,done	;                       uint8_t const* tbl4k) {
	banksel	PID1ACCLL	; //first do: sqsetup(PID1CON, 1/*accumulate*/,)
	com16f	arg16,PID1ACCLL	; *((uint16_t*)PID1ACC) = ~(*arg16);// init'ed 
	ror16f	4,arg16		; arg16 = (arg16 << 12) | (arg16 >> 4);
	btfsc	STATUS,Z	;
	bra	done		; if (arg16) {
	movf	1+arg16,w	;  uint8_t w = arg16 >> 8;
	andlw	0x0f		;  w &= 0x0f; // original arg16>>12
	btfss	STATUS,Z	;
	bra	not0		;
	movf	arg16,f		;
	btfss	STATUS,Z	;
	bra	not0		;  if ((w == 0) && (arg16 & 0x00ff == 0)) // <16
	sqrtp4f	1+arg16,4	;   return sqrtp4f(1+arg16,4); // orig arg16&0xf
	bra	done		;
not0
	addlw	high tbl4k	;  w += (tbl4k & 0xf800) >> 8;
	movwf	PCLATH		;  const uint8_t* PCLATH = w << 8;
	movf	arg16,w		; 
	callw			;  w = PCLATH[arg16 & 0x00ff]; // look up higher
	squarew	PID1INL		;  *PID1ACC += w*w;// of two possibilities, test
	banksel	PID1CON		;  // back to other (control/result) PID1 bank
busy
	clrwdt			;  while (PID1CON & (1<<PID1BUSY))
	btfsc	PID1CON,PID1BUSY;   clrwdt();
	bra	busy		; } else w = 0; // input was zero or negative
	btfss	PID1ACCLH,7	; w += (PID1ACC & (1<<15)) ? 0 : -1; // is lower
	decf	WREG,w		; PID1CON &= ~(1<<PID1EN);
done
	bcf	PID1CON,PID1EN	; return w;
	endm			;}

;;; sqrt16f(arg15,tbl2k)
;;;
;;; returns in WREG the square root of the 15-bit non-negative little-endian
;;;  integer in the two file registers beginning at arg16, using the 2k-aligned
;;;  2048-entry lookup table starting at tbl4k
;;;
;;; if the MSB of arg15 isn't zero, before invoking sqrt16u() arg15 is set to 0
;;;  (from which a zero is quickly returned with Z in STATUS set)
;;;
;;; in this way, the validity of a result can be tested by examining the Z flag;
;;;  if it is set and the input was not zero, the input was too large for the
;;;  2k table and the unsigned version sqrt16u() should have been used instead
;;;
;;; arg16 is clobbered by ror16f(arg16); it is the caller's responsibility to
;;;   restore it if desired with a call to rol16f(arg16)

sqrt16f	macro	arg15,tbl2k	;inline int sqrt16f(int16_t* arg15,
	btfsc	1+arg15,7	;                   const uint8_t const* tbl2k){
	clrf	arg15		; if (arg15 & 0x8000)
	btfsc	1+arg15,7	;  return w = 0; // clip imaginary root to real
	clrf	1+arg15		; else
	sqrt16u	arg15,tbl2k	;  return w = sqrt16u(arg15, tbl2k); // half tbl
	endm			;}

;;; hypoten(side1,side2,tbl)
;;;
;;; returns in WREG the floor of the square root of the sum of the squares of
;;;  the signed 8-bit file registers side1 and side2:
;;;   floor(sqrt(side1*side1 + side2*side2))
;;;
;;; instead of using additional file registers for the intermediate result,
;;;  the actual calculation takes advantage of the 16-bit size of the PID block
;;;  and uses the 16-bit values side1*256 and side2*256 squared in accumulation
;;;  mode; with the lower 16 bits of the accumulator known to be zero the square
;;;  root operation is called on the word in the high 15 bits of the accumulator
;;;  itself without adverse effect
;;; 
;;; the 2k-aligned lookup table tbl only has to be 2048 entries long, since the
;;;  maximum value of the sum is 32768, just barely a 16-bit value and thus too
;;;  large (only in this case when side1=side2=-128) to invoke sqrt16u() so this
;;;  special situation is detected and the answer 181 is returned directly

hypoten macro	side1,side2,tbl	;inline uint8_t hypoten(int8_t side1, int8_t
	sqsetup	PID1CON,+1,0	;             side2, const uint8_t const* tbl) {

	movf	side1,w		;
	squarew	PID1INH		; PID1ACCH = side1*side1; // signed multiply
	banksel	PID1CON		;
	clrwdt			; while ((*PIDxCON) & (1<<PIDxBUSY))
	btfsc	PID1CON,PID1BUSY;  clrwdt();
	bra	$-2		;

	movf	side2,w		;
	squarew	PID1INH		; PID1ACCH += side2*side2; // signed multiply
	banksel	PID1CON		;
	clrwdt			; while ((*PIDxCON) & (1<<PIDxBUSY))
	btfsc	PID1CON,PID1BUSY;  clrwdt();
	bra	$-2		;

	local	exit
	banksel	PID1ACCHH	; // squaring, doubling -128 gives 16-bit result
	movlw	0xb5		; // so skip calculation and return sqrt(32768)
	btfsc	PID1ACCHH,7	; if ((PIDxACC>>8) & 0x08000)
	bra	exit		;  return sqrt(0x08000);

	sqrt16u	PID1ACCHL,tbl	; return (uint8_t) sqrt(PID1ACCH);
exit
	endm			;}
