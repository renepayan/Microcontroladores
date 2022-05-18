/*
 * Aqui van las funcion main del programa
 */
.ifndef __MAIN_S__
    .NOLIST
    .equ __MAIN_S__,1
    .include "p30F4013.inc"
    ;..............................................................................
    ;Funciones globales
    ;..............................................................................
    /*
     * funcion_Principal:
     *	El codigo principal del microcontrolador, programado por el usuario
     */ 
    .global funcion_Principal    
    .text
	La:
	    BCLR    T1CON,	#TON	
	    CLR	    TMR1		    ; Clear contents of the timer register
	    MOV	    #2094,	w0	    ; Load the Period register
	    MOV	    w0,		PR1	   
	    BSET    T1CON,	#TON	
	    RETURN
	Mi:
	    BCLR    T1CON,	#TON	
	    CLR	    TMR1		    ; Clear contents of the timer register
	    MOV	    #3144,	w0	    ; Load the Period register
	    MOV	    w0,		PR1	   
	    BSET    T1CON,	#TON
	    RETURN
	Re:
	    BCLR    T1CON,	#TON	
	    CLR	    TMR1		    ; Clear contents of the timer register
	    MOV	    #2800,	w0	    ; Load the Period register
	    MOV	    w0,		PR1	    ; with the value 14394
	    BSET    T1CON,	#TON
	    RETURN
	    
	/*
	 * funcion_Principal:
	 *	    Aqui va el ciclo principal, esta funcion se va a ejecutar eternamente
	 *	    mientras el microcontrolador este prendido
	 */	
	    
	funcion_Principal:	    
	    PUSH W6
	    MOV PORTD, W6
	    CP W6, W7
	    BRA Z, funcion_Principal
	    MOV W6, W7
	    CP0 W6
	    BRA Z,La
	    CP W6, #1 ;Caso [01]
	    BRA Z, Mi
	    CP W6, #2 ;Caso [10]
	    BRA Z, Re
	    bra funcion_Principal
    .LIST
.endif
    