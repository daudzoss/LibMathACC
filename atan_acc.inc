;;; normal(macro)
;;;
;;; converts two signed 8-bit integers ys and xs into a ratio of two 4-bit ints
;;;  by normalizing them i.e. they are both right-shifted until one of them
;;;  exhibits bit 7 (sign) different from bit 6 (magnitude MSB)
;;;
;;; the ratio is returned in w with the ys portion in the high nybble and the
;;;  xs portion in the low nybble, suitable for an index for looking up a trig
;;;  value for example in a 256-word lookup table
;;; 
normal	macro	ys,xs,scratch	;inline int8_t normal(int8_t *ys, int8_t *xs,
	local	for,cleany,found
	movlw	6		;                     int8_t *scratch) {
	movwf	scratch		; for (*scratch = 6; *scratch > 0; *scratch--) {
for
	rlf	ys,w		;  if (( (*ys)&(1<<6) )!=(( (*ys)&(1<<7) )>>1))
	xorwf	ys,w		;   break; // *ys sign bit differs from next MSB
	btfsc	WREG,7		;
	bra	found		;  else
	lslf	ys,f		;  if (( (*xs)&(1<<6) )!=(( (*xs)&(1<<7) )>>1))

	rlf	xs,w		;   break; // *xs sign bit differs from next MSB
	xorwf	xs,w		;  else {
	btfsc	WREG,7		;   *ys <<= 1;
	bra	cleany		;   *xs <<=1;
	lslf	xs,f		;  }

	decfsz	scratch,f	;
	bra	for		;
	bra	found		;
cleany
	rlf	ys,w		;
	rrf	ys,f		; }

found
	swapf	xs,w		;
	andlw	0x0f		;
	movwf	scratch		;
	movlw	0xf0		;
	andwf	ys,w		;
	iorwf	scratch,w	; return w = ((*ys)&0xf0) | (((*xs)&0xf0) >> 4);
	endm			;}

;;; lut16w(macro)
;;;
;;; lookup into a 16-bit table
;;; 
;;; takes a prefix fname and an unsigned mask (2^N)-1 and generates a call to
;;;  to one of a series of 2^N address labels fname0 through fname[mask] (which
;;;  may overlap to avoid generating redundant table entries)
;;;
;;; each call is expected to return and 8-bit signed value in WREG
;;; 
;;; if the i flag is nonzero, a retfie instead of a return to the caller is
;;;  generated as the 4th instruction
lut16w	macro	fname,mask,fsr,i;inline int8_t lut16w(((uint_8t)(void))*fname[],
	local	j,indf		;         uint8_t mask, uint8_t* fsr, uint1_t i,
indf = ((fsr - FSR0)>>1)
	movwi	++fsr		;         uint8_t w) {
	clrf	indf		; *((uint16_t)fsr) &= 0x00ff;
	rlf	WREG,w		;
	rlf	WREG,w		;
	andlw	mask<<2		;
	brw			; switch (mask & w) {
j = 0
	while	(j<=mask)
	pagesel fname#v(j)	; case j:
	 call	fname#v(j)	;  *fsr = (uint16_t) (fname[j])())
	 movwi	--fsr		;  return j; // to normal function
	 if (i)
	  retfie		;  return; // from interrupt context
	 else
	  retlw	j		; }
	 endif
j += 1
	endw
	endm			;}
