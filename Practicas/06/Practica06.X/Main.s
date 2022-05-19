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
	
	    
	/*
	 * funcion_Principal:
	 *	    Aqui va el ciclo principal, esta funcion se va a ejecutar eternamente
	 *	    mientras el microcontrolador este prendido
	 */	
	    
	funcion_Principal:
	    MOV W4, U1TXREG	    
	    MOV #1000, W0
	    CALL delay_ms	    
	    INC W4, W4   	    
	    GOTO funcion_Principal	    	    
	    RETURN
    .LIST
.endif
    