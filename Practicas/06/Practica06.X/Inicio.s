/*
    Aqui va el codigo de inicio del microcontrolador
 */
.ifndef __INICIO_S__
    .NOLIST
    .equ __INICIO_S__,1
    .include "p30F4013.inc"    
    ;..............................................................................
    ;Funciones globales
    ;..............................................................................
    /*
     * _wreg_init:
     *	Provide global scope to _wreg_init routine
     *	In order to call this routine from a C file,
     *	place "wreg_init" in an "extern" declaration
     *	in the C file.
     */ 
    .global _wreg_init
    /*
     * __reset:
     *	The label for the first line of code.
     */
    .global __reset         
    .text
	/*
	 * Esta funcion inicializa los 14 registros en 0
	 */
	_wreg_init:
	    CLR W0
	    MOV W0, W14
	    REPEAT #12
	    MOV W0, [++W14]
	    CLR W14
	    RETURN

	/*
	 * Esta funcion prepara todos los puertos del microcontrolador para que sean de entrada o salida
	 */
	CONF_PERIPHERALS:
	    SETM	ADPCFG		    ;Disable analogic inputs	
	    /*
	     *  Configuracion del puerto A, como entrada de datos
	     */
	    CLR         PORTA
	    NOP
	    CLR         LATA
	    NOP
	    SETM        TRISA		    
	    NOP       			

	    /*
	     *  Configuracion del puerto B, como salida de datos
	     */
	    CLR         PORTB
	    NOP
	    CLR         LATB
	    NOP
	    CLR         TRISB		   
	    NOP       			

	    /*
	     *  Configuracion del puerto C, como entrada de datos
	     */	
	    CLR         PORTC
	    NOP
	    CLR         LATC
	    NOP
	    SETM        TRISC
	    NOP       

	    /*
	     *  Configuracion del puerto D, como entrada de datos
	     */
	    CLR         PORTD
	    NOP
	    CLR         LATD
	    NOP 
	    SETM        TRISD
	    NOP

	    /*
	     *  Configuracion del puerto F, como entrada de datos
	     */
	    CLR         PORTF
	    NOP
	    CLR         LATF
	    NOP
	    SETM        TRISF		    ;PORTF AS INPUT
	    NOP       		    
	    RETURN      

	/*
	  *	Esta funcion configura el UART1
	  */
	CONF_UART1:
	    MOV	#11,	W0
	    MOV	W0,	U1BRG

	    BSET	IPC2,	#U1TXIP2 ; Set UART TX interrupt priority
	    BCLR	IPC2,	#U1TXIP1 ;
	    BCLR	IPC2,	#U1TXIP0 ;

	    BSET	IPC2,	#U1RXIP2 ; Set UART RX interrupt priority
	    BCLR	IPC2,	#U1RXIP1 ;
	    BCLR	IPC2,	#U1RXIP0 ;
	    CLR	U1STA

	    MOV	#0x8000,    W0	; Enable UART for 8-bit data,
				    ; no parity, 1 STOP bit,
				    ; no wakeup
	    BCLR	 IFS0,	#U1RXIF
	    MOV		 W0,	 U1MODE
	    BSET	 U1STA,	#UTXEN ; Enable transmit
	    BSET	 IEC0,	#U1RXIE ; Enable receive interrupts	    
	    RETURN
	
	CONF_T1:
	    PUSH    W0
	    CLR	    T1CON		    ; Stops the Timer1 and reset control reg.
	    BCLR    T1CON,	#TCKPS0	    ; Sin preescala
	    BCLR    T1CON,	#TCKPS1	    	    

	    CLR	    TMR1		    ; Clear contents of the timer register
	    MOV	    #3529,	w0	    ; Load the Period register
	    MOV	    w0,		PR1	    ; with the value 14394

	    BCLR    IPC0,	#T1IP0	    ; Setup Timer1 interrupt for
	    BCLR    IPC0,	#T1IP1	    ; desired priority level
	    BSET    IPC0,	#T1IP2	    ; (this example assigns level 4 priority)

	    BCLR    IFS0,	#T1IF	    ; Clear the Timer1 interrupt status flag
	    BSET    IEC0,	#T1IE	    ; Enable Timer1 interrupts
	    BSET    T1CON,	#TON	    ; Start Timer1 with clock source set to

	    POP	    W0
	    ; the internal instruction cycle
	    RETURN

	/*
	 *	Esta funcion es la primer linea de codigo del microcontrolador
	 */
	__reset:
	    MOV #__SP_init, W15       ;Initalize the Stack Pointer
	    MOV #__SPLIM_init, W0     ;Initialize the Stack Pointer Limit Register
	    MOV W0, SPLIM
	    NOP                       ;Add NOP to follow SPLIM initialization	    
	    CALL _wreg_init           ;Call _wreg_init subroutine, Optionally use RCALL instead of CALL	
	    CALL CONF_PERIPHERALS
	    CALL CONF_UART1		    	    	    
	    CALL CONF_T1	    
	    MOV #10, W7	
	    ciclo_eterno:
		CALL funcion_Principal
		GOTO ciclo_eterno
    .LIST
.endif