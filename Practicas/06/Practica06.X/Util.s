.ifndef __UTIL_S__
    .NOLIST
    .equ __UTIL_S__,1
    .include "p30F4013.inc"
    .global delay_ms    
   
    .text
	delay_ms:
	    MOV #615, W1
	    PUSH W1	    	
	    C2:		    
		DEC W1,W1
		BRA NZ, C2
	    POP W1
	    DEC W0,W0
	    CP0 W0
	    BRA NZ, delay_ms
	    RETURN		    	    
    .LIST
.endif