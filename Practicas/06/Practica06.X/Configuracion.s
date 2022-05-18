/*
 *  En este archivo va toda la configuracion necesaria para hacer funcionar el microncontrolador
 */
.ifndef __CONFIGURACION_S__
    .NOLIST
    .equ __CONFIGURACION_S__,1
    .include "p30F4013.inc"
    ;---------------------------------------------------------------------------    
    /*
     *  Clock Switching Operation and the Fail-Safe Clock Monitor (FSCM) are disabled.
     *  FSCM allows the device to continue to operate even in the event of an oscillator failure.
     *  FRC 7.37 MHz internal Fast RC oscillator. Enabled    
     */        
    #pragma config __FOSC, CSW_FSCM_OFF & FRC

    ;---------------------------------------------------------------------------    
    /*
     *  The primary function of the Watchdog Timer (WDT) is to reset the processor
     *  in the event of a software malfunction.
     *  Watchdog Timer is disabled
     */    
    #pragma config __FWDT, WDT_OFF 

    ;---------------------------------------------------------------------------        
    /*     
     *  The BOR and POR Configuration bits found in the FBORPOR Configuration 
     *  register are used to set the Brown-out Reset voltage for the device, 
     *  enable the Brown-out Reset circuit, and set the Power-up Timer delay time.
     *  For more information on these Configuration bits, please refer to Section 8. "Reset?.
     *  POR: Power-on Reset:
     *	There are two threshold voltages associated with a Power-on Reset (POR). 
     *	The first voltage is the device threshold voltage, V POR . The device 
     *      threshold voltage is the voltage at which the device logic circuits become 
     *      operable. The second voltage associated with a POR event is the POR circuit 
     *	threshold voltage which is nominally 1.85V.
     *  BOR: Brown-out Reset:
     *	Module is based on an internal voltage reference
     *	circuit. The main purpose of the BOR module is to generate a device Reset
     *	when a brown-out condition occurs. Brown-out conditions are generally 
     *	caused by glitches on the AC mains (i.e., missing waveform portions of the 
     *	AC cycles due to bad power transmission lines), or voltage sags due to 
     *	excessive current draw when a large load is energized.
     *  TPWRT:
     *	Additional "power-up" delay as determined by the FPWRT<1:0>
     * 	configuration bits. This delay is 0 ms, 4 ms, 16 ms or 64 ms nominal.
     *  EXTR: 
     *	External Reset (MCLR) Pin bit enabled
     *  RCON: 
     *	Reset Control Register
     */       
     #pragma config __FBORPOR, PBOR_ON & BORV27 & PWRT_16 & MCLR_EN

    ;---------------------------------------------------------------------------      
    /*
     *  General Code Segment Configuration Bits
     *  The general code segment Configuration bits in the FGS Configuration register 
     *  are used to code-protect or write-protect the user program memory space. 
     *  The general code segment includes all user program memory with the exception
     *  of the interrupt vector table space (0x000000-0x0000FE).
     *  If the general code segment is code-protected by programming the GCP 
     *  Configuration bit (FGS<1>) to a ?0?, the device program memory cannot be 
     *  read from the device using In-Circuit Serial Programming (ICSP), or the 
     *  device programmer. Additionally, further code cannot be programmed into the 
     *  device without first erasing the entire general code segment.
     *  When the general segment is code-protected, user code can still access the 
     *  program memory data via table read instructions, or Program Space Visibility
     *  (PSV) accesses from data space.
     *  If the GWRP (FGS<0>) Configuration bit is programmed, all writes to the 
     *  user program memory space are disabled.    
     */
     #pragma config __FGS, CODE_PROT_OFF & GWRP_OFF
    ;..............................................................................
    ;Constantes del programa, se usaran en el codigo
    ;..............................................................................
    .equ SAMPLES, 64         ;Number of samples
    
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
				     ;variable "var1"
    .LIST
.endif			
		     