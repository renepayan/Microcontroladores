.ifndef __INTERRUPCIONES_S__   
    .NOLIST    
    .equ __INTERRUPCIONES_S__,1
    .include "p30F4013.inc"
    ;---------------------------------------------------------------------------
    ;Variables globales
    ;---------------------------------------------------------------------------    
    .global __U1RXInterrupt
    .global __T1Interrupt
    
    .text
    /*
     * __U1RXInterrupt:
     *	Esta interrupcion se activa cuando se recibe un paquete via UART1     
     */ 
    __U1RXInterrupt:
	BCLR    IFS0,	    #U1RXIF	
	RETFIE
    __T1Interrupt:   
	COM	    PORTB	
	BCLR	    IFS0,	#T1IF	    ; Clear the Timer1 interrupt status flag
	RETFIE
    .LIST
.endif