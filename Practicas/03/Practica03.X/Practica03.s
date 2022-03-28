/*
 * File:   This file is the first example of the course.     
	    You can use it as a template if necessary  
	    The progam makes blink a light emiter diode (LED) at 1Hz
     
 * Author: Garcia Alvarez Corinne Angelica
 *         Payán Téllez René
 *
 * Created on March, 2022
 */

    .include "p30F4013.inc"
;---------------------------------------------------------------------------    
    
    ;Clock Switching Operation and 
    ;the Fail-Safe Clock Monitor (FSCM) are disabled.
    ;FSCM allows the device to continue to operate even in the event of 
    ;an oscillator failure.    
    ;FRC 7.37 MHz internal Fast RC oscillator. Enabled
    
    #pragma config __FOSC, CSW_FSCM_OFF & FRC
    
;---------------------------------------------------------------------------    
    
    ;Watchdog Timer is disabled
    ;The primary function of the Watchdog Timer (WDT) is to reset the processor
    ;in the event of a software malfunction
    #pragma config __FWDT, WDT_OFF 
    
;---------------------------------------------------------------------------    
    
    ;The BOR and POR Configuration bits found in the FBORPOR Configuration 
    ;register are used to set the Brown-out Reset voltage for the device, 
    ;enable the Brown-out Reset circuit, and set the Power-up Timer delay time.
    ;For more information on these Configuration bits, please refer to 
    ;Section 8. "Reset?.
    
;    POR: Power-on Reset
;   There are two threshold voltages associated with a Power-on Reset (POR). 
;    The first voltage is the device threshold voltage, V POR . The device 
;    threshold voltage is the voltage at which the device logic circuits become 
;    operable. The second voltage associated with a POR event is the POR circuit 
;    threshold voltage which is nominally 1.85V.
    
;    Brown-out Reset (BOR) module is based on an internal voltage reference 
    ;circuit. The main purpose of the BOR module is to generate a device Reset
    ;when a brown-out condition occurs. Brown-out conditions are generally 
    ;caused by glitches on the AC mains (i.e., missing waveform portions of the 
    ;AC cycles due to bad power transmission lines), or voltage sags due to 
    ;excessive current draw when a large load is energized.
    
;    TPWRT = Additional ?power-up? delay as determined by the FPWRT<1:0>
;   configuration bits. This delay is 0 ms, 4 ms, 16 ms or 64 ms nominal.
    
;    EXTR: External Reset (MCLR) Pin bit enabled
    ;RCON: Reset Control Register
    
    #pragma config __FBORPOR, PBOR_ON & BORV27 & PWRT_16 & MCLR_EN
    
;---------------------------------------------------------------------------      
    
;    General Code Segment Configuration Bits
;The general code segment Configuration bits in the FGS Configuration register 
;    are used to code-protect or write-protect the user program memory space. 
;    The general code segment includes all user program memory with the exception
;    of the interrupt vector table space (0x000000-0x0000FE).
;If the general code segment is code-protected by programming the GCP 
;    Configuration bit (FGS<1>) to a ?0?, the device program memory cannot be 
;    read from the device using In-Circuit Serial Programming (ICSP), or the 
;    device programmer. Additionally, further code cannot be programmed into the 
;    device without first erasing the entire general code segment.
;    When the general segment is code-protected, user code can still access the 
;    program memory data via table read instructions, or Program Space Visibility
;    (PSV) accesses from data space. 
;    If the GWRP (FGS<0>) Configuration bit is programmed, all writes to the 
;    user program memory space are disabled.    
    
    #pragma config __FGS, CODE_PROT_OFF & GWRP_OFF

;..............................................................................
;Program Specific Constants (literals used in code)
;..............................................................................

    .equ SAMPLES, 64         ;Number of samples



;..............................................................................
;Global Declarations:
;..............................................................................

    .global _wreg_init       ;Provide global scope to _wreg_init routine
                                 ;In order to call this routine from a C file,
                                 ;place "wreg_init" in an "extern" declaration
                                 ;in the C file.

    .global __reset          ;The label for the first line of code.

;..............................................................................
;Constants stored in Program space
;..............................................................................

    .section .myconstbuffer, code
    .palign 2                ;Align next word stored in Program space to an
                                 ;address that is a multiple of 2
ps_coeff:
    .hword   0x0002, 0x0003, 0x0005, 0x000A
KNIGHT_RIDER_PM:
    .WORD 0X00E0, 0X0070, 0X0038, 0X001C, 0X000E, 0X0007
    .WORD 0X0007, 0X000E, 0X001C, 0X0038, 0X0070, 0X00E0, 0X0000
	
BLINK_PM:
    .WORD 0X8FFF, 0X8000, 0X0000
	
ROTATE_LEFT_PM:
    .WORD 0X0001, 0X0002, 0X0004, 0X0008, 0X0010, 0X0020, 0X0040, 0X0080, 0X0000
	
ROTATE_RIGHT_PM:
    .WORD 0X0080, 0X0040, 0X0020, 0X0010, 0X0008, 0X0004, 0X0002, 0X0001, 0X0000
	
CENTER_PM:
    .WORD 0X0018, 0X0024, 0X0042, 0X0081, 0X00000
    ;.WORD 0X0081, 0X0042, 0X0024, 0X0018
	
LETTER_AU_PM:
    .WORD 0X0077, 0X004F, 0X0006, 0X007E, 0X003E, 0X0000
	
LETTER_UA_PM:
    .WORD 0X003E, 0X007E, 0X0006, 0X004F, 0X0077, 0X0000



;..............................................................................
;Uninitialized variables in X-space in data memory
;..............................................................................

    .section .xbss, bss, xmemory
x_input: .space 2*SAMPLES        ;Allocating space (in bytes) to variable.



;..............................................................................
;Uninitialized variables in Y-space in data memory
;..............................................................................

    .section .ybss, bss, ymemory
y_input:  .space 2*SAMPLES




;..............................................................................
;Uninitialized variables in Near data memory (Lower 8Kb of RAM)
;..............................................................................

    .section .nbss, bss, near
var1:     .space 2               ;Example of allocating 1 word of space for
                                 ;variable "var1".




;..............................................................................
;Code Section in Program Memory
;..............................................................................

.text                             ;Start of Code section
__reset:
    MOV #__SP_init, W15       ;Initalize the Stack Pointer
    MOV #__SPLIM_init, W0     ;Initialize the Stack Pointer Limit Register
    MOV W0, SPLIM
    NOP                       ;Add NOP to follow SPLIM initialization

    CALL _wreg_init           ;Call _wreg_init subroutine
			      ;Optionally use RCALL instead of CALL
			      ;<<insert more user code here>>
    CALL INI_PERIPHERALS	
    CALL done
    RETURN
    
delay_10us:
    MOV #6, W1
    PUSH W1	    	
    C1:		    
	DEC W1,W1
	BRA NZ, C1
    POP W1
    DEC W0,W0
    CP0 W0
    BRA NZ, delay_10us
    RETURN
    
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

obtenerValoresDeMemoriaEImprimirEnB:    
    MOV	W0, W5							;Salvamos el valor del sleep
    A1_obtenerValoresDeMemoriaEImprimirEnB:
	TBLRDL [W3++], W4					;Leemos el registro en memoria (Posicion W3) dentro de W4 y aumentamos en 1 W3
	CP0	    W4						;Comparamos W4 con 0
	BRA	    NZ, A2_obtenerValoresDeMemoriaEImprimirEnB	;Si no es cero, entonces nos vamos a la siguiente parte del codigo
	RETURN							;Caso contrario, regresamos al ciclo principal
    A2_obtenerValoresDeMemoriaEImprimirEnB:
	MOV	    W4, PORTB					;si no es cero, entonces cargamos el valor de W4 al puerto B		
	MOV	    W5, W0					;Le regresamos a W0 el valor del sleep (despues del primer ciclo se pierde)
	CALL	    delay_ms					;Detener n*1ms donde n esta en W0
	BRA	    A1_obtenerValoresDeMemoriaEImprimirEnB	;Ejecutamos de nuevo el ciclo de lectura

obtenerValoresDeMemoriaEImprimirEnF:    
    MOV	W0, W5							;Salvamos el valor del sleep
    A1_obtenerValoresDeMemoriaEImprimirEnF:
	TBLRDL [W3++], W4					;Leemos el registro en memoria (Posicion W3) dentro de W4 y aumentamos en 1 W3
	CP0	    W4						;Comparamos W4 con 0
	BRA	    NZ, A2_obtenerValoresDeMemoriaEImprimirEnF	;Si no es cero, entonces nos vamos a la siguiente parte del codigo
	RETURN							;Caso contrario, regresamos al ciclo principal
    A2_obtenerValoresDeMemoriaEImprimirEnF:
	MOV	    W4, PORTB					;si no es cero, entonces cargamos el valor de W4 al puerto F		
	MOV	    W5, W0					;Le regresamos a W0 el valor del sleep (despues del primer ciclo se pierde)
	CALL	    delay_ms					;Detener n*1ms donde n esta en W0
	BRA	    A1_obtenerValoresDeMemoriaEImprimirEnB	;Ejecutamos de nuevo el ciclo de lectura
	
	
knightRider:    
    MOV	    #tblpage(KNIGHT_RIDER_PM), W2	;Cargamos la tabla de paginacion a W2
    MOV	    W2, TBLPAG				;Cargamos TBLPAG con nuestra tabla de paginación 
    MOV	    #tbloffset(KNIGHT_RIDER_PM), W3	;Movemos el offset a W3
    MOV	    #100, W0				;Cargamos en el registro 0 el tiempo de espera
    CALL obtenerValoresDeMemoriaEImprimirEnB	;Llamamos a la funcion generica    
    RETURN	
    
parpadeo200msRetradoT400ms:
    MOV	    #tblpage(BLINK_PM), W2		;Cargamos la tabla de paginacion a W2
    MOV	    W2, TBLPAG				;Cargamos TBLPAG con nuestra tabla de paginación 
    MOV	    #tbloffset(BLINK_PM), W3		;Movemos el offset a W3
    MOV	    #200, W0				;Cargamos en el registro 0 el tiempo de espera
    CALL obtenerValoresDeMemoriaEImprimirEnB	;Llamamos a la funcion generica    
    RETURN
    
parpadeo500msRetradoT1s:
    MOV	    #tblpage(BLINK_PM), W2		;Cargamos la tabla de paginacion a W2
    MOV	    W2, TBLPAG				;Cargamos TBLPAG con nuestra tabla de paginación 
    MOV	    #tbloffset(BLINK_PM), W3		;Movemos el offset a W3
    MOV	    #500, W0				;Cargamos en el registro 0 el tiempo de espera
    CALL obtenerValoresDeMemoriaEImprimirEnB	;Llamamos a la funcion generica    
    RETURN
    
giraIzquierdaRetraso100ms:    
    MOV	    #tblpage(ROTATE_LEFT_PM), W2	;Cargamos la tabla de paginacion a W2
    MOV	    W2, TBLPAG				;Cargamos TBLPAG con nuestra tabla de paginación 
    MOV	    #tbloffset(ROTATE_LEFT_PM), W3	;Movemos el offset a W3
    MOV	    #100, W0				;Cargamos en el registro 0 el tiempo de espera
    CALL obtenerValoresDeMemoriaEImprimirEnB	;Llamamos a la funcion generica    
    RETURN
    
giraDerechaRetraso100ms:
    MOV	    #tblpage(ROTATE_RIGHT_PM), W2	;Cargamos la tabla de paginacion a W2
    MOV	    W2, TBLPAG				;Cargamos TBLPAG con nuestra tabla de paginación 
    MOV	    #tbloffset(ROTATE_RIGHT_PM), W3	;Movemos el offset a W3
    MOV	    #100, W0				;Cargamos en el registro 0 el tiempo de espera
    CALL obtenerValoresDeMemoriaEImprimirEnB	;Llamamos a la funcion generica    
    RETURN

partirDelCentroEIrEsquinas:
    MOV	    #tblpage(CENTER_PM), W2		;Cargamos la tabla de paginacion a W2
    MOV	    W2, TBLPAG				;Cargamos TBLPAG con nuestra tabla de paginación 
    MOV	    #tbloffset(CENTER_PM), W3		;Movemos el offset a W3
    MOV	    #100, W0				;Cargamos en el registro 0 el tiempo de espera
    CALL obtenerValoresDeMemoriaEImprimirEnB	;Llamamos a la funcion generica    
    RETURN
    
imprimirLetrasAU:
    MOV	    #tblpage(LETTER_AU_PM), W2		;Cargamos la tabla de paginacion a W2
    MOV	    W2, TBLPAG				;Cargamos TBLPAG con nuestra tabla de paginación 
    MOV	    #tbloffset(LETTER_AU_PM), W3		;Movemos el offset a W3
    MOV	    #100, W0				;Cargamos en el registro 0 el tiempo de espera
    CALL obtenerValoresDeMemoriaEImprimirEnF	;Llamamos a la funcion generica    
    RETURN
    
imprimirLetrasUA:    
    MOV	    #tblpage(LETTER_UA_PM), W2		;Cargamos la tabla de paginacion a W2
    MOV	    W2, TBLPAG				;Cargamos TBLPAG con nuestra tabla de paginación 
    MOV	    #tbloffset(LETTER_UA_PM), W3		;Movemos el offset a W3
    MOV	    #100, W0				;Cargamos en el registro 0 el tiempo de espera
    CALL obtenerValoresDeMemoriaEImprimirEnF	;Llamamos a la funcion generica    
    RETURN
done:      
    PUSH    W0
    PUSH    W1    
    PUSH    W2
    PUSH    W3
    PUSH    W4
    PUSH    W5
    PUSH    W6   
    PUSH    W7
    CLR	    W0
    CLR     W2
    CLR     W3
    CLR     W4
    CLR     W5
    CLR	    W6
    CLR     W7
    CLR	    PORTB
    CLR	    LATB
    MOV PORTD, W6
    CP0 W6    ;Caso [000]
    BRA Z,knightRider
    CP W6, #1 ;Caso [001]
    BRA Z, parpadeo200msRetradoT400ms
    CP W6, #2 ;Caso [010]
    BRA Z, parpadeo500msRetradoT1s
    CP W6, #3 ;Caso [011]
    BRA Z, giraIzquierdaRetraso100ms
    CP W6, #4 ;Caso [100]
    BRA Z, giraDerechaRetraso100ms
    CP W6, #5 ;Caso [101]
    BRA Z, partirDelCentroEIrEsquinas
    CP W6, #6 ;Caso [110] 
    BRA Z, imprimirLetrasAU
    CP W6, #7 ;Caso [111] 
    BRA Z, imprimirLetrasUA    
    POP    W0
    POP    W1    
    POP    W2
    POP    W3
    POP    W4
    POP    W5
    POP    W6
    POP    W7
    BRA     done				;Place holder for last line of executed code



;..............................................................................
;Subroutine: Initialization of W registers to 0x0000
;..............................................................................

_wreg_init:
    CLR W0
    MOV W0, W14
    REPEAT #12
    MOV W0, [++W14]
    CLR W14
    RETURN
    
    
;******************************************************************************
;DESCRIPTION:	Initialize peripherals and determine whether each pin associated
		;with the I/O port is an input or an output
;PARAMETER: 	NINGUNO
;RETURN: 	NINGUNO
;******************************************************************************		
INI_PERIPHERALS:
    CLR         PORTB
    NOP
    CLR         LATB
    NOP
    CLR         TRISB		    ;PORTB AS OUTPUT
    NOP       			
    SETM	ADPCFG		    ;Disable analogic inputs
	
    CLR         PORTC
    NOP
    CLR         LATC
    NOP
    SETM        TRISC		    ;PORTC AS INPUT
    NOP       
	
    CLR         PORTD
    NOP
    CLR         LATD
    NOP 
    SETM        TRISD		    ;PORTD AS INPUT
    NOP

    CLR         PORTF
    NOP
    CLR         LATF
    NOP
    CLR        TRISF		    ;PORTF AS OUTPUT
    NOP       		
    
    RETURN    

;--------End of All Code Sections ---------------------------------------------   

.end                               ;End of program code in this file
