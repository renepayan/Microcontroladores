/*
 * File:   This file is the first example of the course.     
	    You can use it as a template if necessary  
	    The progam makes blink a light emiter diode (LED) at 1Hz
     
 * Author: Garcia Alvarez Corinne Angelica
 *         Pay�n T�llez Ren�
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
    
F_Es_0_Y_C_Es_0:		;Este es el caso donde ambos dip switch estan apagados, aqui llamamos a un solo delay de 10us
    PUSH W0
    MOV #1, W0 
    CALL delay_10us
    RETURN
    
F_No_Es_0_Y_C_Es_0:		;Este es el caso donde un dip switch esta apagado y el otro no, aqui llamamos a 10 delay de 10us
    PUSH W0
    MOV #10, W0 
    CALL delay_10us
    RETURN
    
F_Es_0_Y_C_No_Es_0:		;Este es el caso donde un dip switch esta apagado y el otro no, aqui llamamos a 10 bloques de delay de 1ms
    PUSH W0
    MOV #10, W0 
    CALL delay_ms
    RETURN
    
F_No_Es_0_Y_C_No_Es_0:		;Este es el caso donde ambos dip switch estan prendidos, aqui llamamos a 200 bloques de delay de 1ms
    PUSH W0
    MOV #500, W0 
    CALL delay_ms
    RETURN
    
C_Es_0:
    CP0	    PORTF		    ;Comparamos el puerto F con 0
    BRA	    NZ, F_No_Es_0_Y_C_Es_0  ;Si no es 0 nos vamos a F_No_Es_0_Y_C_Es_0
    BRA	    Z,  F_Es_0_Y_C_Es_0	    ;Si es 0 nos vamos a F_Es_0_Y_C_Es_0
    RETURN
    
C_No_Es_0:
    CP0	    PORTF			;Comparamos el puerto F con 0
    BRA	    NZ, F_No_Es_0_Y_C_No_Es_0	;Si no es 0 nos vamos a F_No_Es_0_Y_C_No_Es_0
    BRA	    Z,  F_Es_0_Y_C_No_Es_0	;Si es 0 nos vamos a F_Es_0_Y_C_No_Es_0
    RETURN
    
done:
    COM	    PORTB	    ;Invertimos el puerto B
    CP0	    PORTC	    ;Comparamos el puerto C con 0
    BRA	    NZ, C_No_Es_0   ;Si el puerto C no es 0, nos vamos a la funcion C_No_Es_0
    BRA	    Z,  C_Es_0	    ;Si el puerto C es 0, nos vamos a la funcion C_Es_0    
    

    BRA     done              ;Place holder for last line of executed code



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
    SETM        TRISF		    ;PORTF AS INPUT
    NOP       		
    
    RETURN    

;--------End of All Code Sections ---------------------------------------------   

.end                               ;End of program code in this file
