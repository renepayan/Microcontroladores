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
	MOV     #15, W0
	CALL    delay_ms	
	MOV	U1RXREG,    W8	
	MOV     W8,         U1TXREG			
	PUSH W6	  
	PUSH W0
	MOV W8, W6
	CP W6, W7
	BRA Z, salte		
	MOV W6, W7
	CP0 W6    ;Caso [000]  
	BRA Z, Doo
	CP W6, #1 ;Caso [001]
	BRA Z, Re
	CP W6, #2 ;Caso [010]
	BRA Z, Mi		
	CP W6, #3 ;Caso [011]
	BRA Z, Fa
	CP W6, #4 ;Caso [100]
	BRA Z, So	
	CP W6, #5 ;Caso [101]
	BRA Z, La
	CP W6, #6 ;Caso [110]
	BRA Z, Si	
	CP W6, #7 ;Caso [111]
	BRA Z, prueba_1s
	salte:	  
	    POP W0
	    POP W6	
	    BCLR    IFS0,	    #U1RXIF
	    RETFIE
	Doo:
	    MOV	    #3529,	w0	    ; Load the Period register
	    BCLR    T1CON,	#TCKPS0	    ; Sin preescala
	    BCLR    T1CON,	#TCKPS1	   
	    MOV	    w0,		PR1	
	    GOTO salte
	Re:
	    MOV	    #3144,	w0	    ; Load the Period register
	    BCLR    T1CON,	#TCKPS0	    ; Sin preescala
	    BCLR    T1CON,	#TCKPS1	   
	    MOV	    w0,		PR1	
	    GOTO salte
	Mi:	    
	    MOV	    #2800,	w0	    ; Load the Period register
	    BCLR    T1CON,	#TCKPS0	    ; Sin preescala
	    BCLR    T1CON,	#TCKPS1	   
	    MOV	    w0,		PR1	
	    GOTO salte
	Fa:	    
	    MOV	    #2639,	w0	    ; Load the Period register
	    BCLR    T1CON,	#TCKPS0	    ; Sin preescala
	    BCLR    T1CON,	#TCKPS1	   
	    MOV	    w0,		PR1	
	    GOTO salte
	So:	    
	    MOV	    #2350,	w0	    ; Load the Period register
	    BCLR    T1CON,	#TCKPS0	    ; Sin preescala
	    BCLR    T1CON,	#TCKPS1	   
	    MOV	    w0,		PR1	
	    GOTO salte
	La:	    
	    MOV	    #2093,	w0	    ; Load the Period register
	    BCLR    T1CON,	#TCKPS0	    ; Sin preescala
	    BCLR    T1CON,	#TCKPS1	   
	    MOV	    w0,		PR1	
	    GOTO salte
	Si:	    
	    MOV	    #1868,	w0	    ; Load the Period register
	    BCLR    T1CON,	#TCKPS0	    ; Sin preescala
	    BCLR    T1CON,	#TCKPS1	   
	    MOV	    w0,		PR1	
	    GOTO salte
	prueba_1s:
	    MOV	    #3598,	w0	    ; Load the Period register
	    BSET    T1CON,	#TCKPS0	    ; Sin preescala
	    BSET    T1CON,	#TCKPS1	   
	    MOV	    w0,		PR1	
	    GOTO salte  
    __T1Interrupt:   	
	COM	    PORTB	
	BCLR	    IFS0,	#T1IF	    ; Clear the Timer1 interrupt status flag
	RETFIE
    .LIST
.endif