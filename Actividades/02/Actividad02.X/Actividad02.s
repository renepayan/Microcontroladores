/*
 * File:   This file is the first example using UART.     
	    You can use it as a template if necessary  
	    The progam sends a counter through Tx which is
	    W2. The parameters are 9600, 8, N, 1
	    Besides the counter, an echo was programmed on the
	    Rx ISR. Just take a look.
     
	    Advices: 
	    >>	Use picocom on linux as a serial monitor
		type: sudo picocom -b 9600 /dev/tty/USB0
     	    >>	To know what USB are present, just type:
		dmesg | grep tty
	    >>	DO NOT SHARE NEITHER TX NOR RX PINS WITH OTHER MODULES
		OTHERWISE, YOU WILL NOT SEE ANYTHING
	    
	    Sincerely, hope this will be helpful
     
 * Author: Ren� Jim�nez
 *
 * Created on November 29th
 */

    .include "p30F4013.inc"
;---------------------------------------------------------------------------    
    
    ;Clock Switching Operation and 
    ;the Fail-Safe Clock Monitor (FSCM) are disabled.
    ;FSCM allows the device to continue to operate even in the event of 
    ;an oscillator failure.    
    ;FRC 7.37 MHz internal Fast RC oscillator. Enabled
    
    config __FOSC, CSW_FSCM_OFF & FRC
    
;---------------------------------------------------------------------------    
    
    ;Watchdog Timer is disabled
    ;The primary function of the Watchdog Timer (WDT) is to reset the processor
    ;in the event of a software malfunction
    config __FWDT, WDT_OFF 
    
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
    
    config __FBORPOR, PBOR_ON & BORV27 & PWRT_16 & MCLR_EN
    
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
    
    config __FGS, CODE_PROT_OFF & GWRP_OFF

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
    .global __U1RXInterrupt
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

CALL CONF_PERIPHERALS
CALL CONF_UART1	
CLR	W2		;THIS IS THE COUNTER
done:
    INC	    W2,	    W2
    MOV	    W2,	    PORTB
    NOP
    MOV	    W2,	    U1TXREG
    ;CALL    DELAY_250ms
    NOP
    MOV     #1000, W0
    CALL    delay_ms 
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
    
delay_ms:
    MOV #615, W1
    PUSH W1	    	
    C2_delay_ms:		    
	DEC W1,W1
	BRA NZ, C2_delay_ms
    POP W1
    DEC W0,W0
    CP0 W0
    BRA NZ, delay_ms
    RETURN
;******************************************************************************
;DESCRIPTION:	SECTION OF CODE FOR A 1s DELAY
;PARAMETER: 	NINGUNO
;RETURN: 	NINGUNO
;******************************************************************************		    
    
;FOSC = 7.3725 MHz -> this is the internal clock source (FRC) and we are using it. 
    ;If you use another clock source you have to modify "config __FOSC" section 
    
;FCY = FOSC/4 = 1.8432 MHz or cycles p/sec.
;T(FCY) = 542.53 ns. This is the time for an internal instruction cycle clock (FCY).	

;"CYCLE1" will repeat 2^16 times = 65536 times
;DEC(1) + BRA(2) = 3 pulses in total
;(BRA uses 2 CLK pulses when it jumps and just one if it does not)

;65536 * 3 cycles * 542 ns = 0.1066s
;Thus, "CYCLE1" must be repeated 10 times to delay 1s
DELAY_1s:
	PUSH	    W0
	PUSH	    W1	
	
	MOV	    #10,	    W1
    CYCLE2:	
	CLR	    W0
	
    CYCLE1:		
	DEC	    W0,		    W0
	BRA	    NZ,		    CYCLE1
	
	DEC	    W1,		    W1
	BRA	    NZ,		    CYCLE2
	
	POP	    W1
	POP	    W0
RETURN
	
;******************************************************************************
;DESCRIPTION:	SECTION OF CODE FOR A 25ms DELAY
;PARAMETER: 	NINGUNO
;RETURN: 	NINGUNO
;******************************************************************************		    
DELAY_250ms:
	PUSH	    W0
	PUSH	    W1	
	
	MOV	    #3,	    W1
    C2:	
	CLR	    W0
	
    C1:		
	DEC	    W0,		    W0
	BRA	    NZ,		    C1
	
	DEC	    W1,		    W1
	BRA	    NZ,		    C2
	
	POP	    W1
	POP	    W0
RETURN	
	
;******************************************************************************
;DESCRIPTION:	SECTION OF CODE FOR A 25ms DELAY
;PARAMETER: 	NINGUNO
;RETURN: 	NINGUNO
;******************************************************************************		    
DELAY_15ms:
	PUSH	    W0
		
	MOV	    #9225,	    W0   	
	
    CC1:		
	DEC	    W0,		    W0
	BRA	    NZ,		    CC1
				
	POP	    W0
RETURN		
	
;******************************************************************************
;	Same as a function in C	
;	VOID CONF_PERIPHERALS ( VOID )
;******************************************************************************		
CONF_PERIPHERALS:
    CLR         PORTA
    NOP
    CLR         LATA
    NOP
    SETM        TRISA		    ;PORTA AS INPUT
    NOP       			
    
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
	MOV	 W0,	U1MODE
	BSET	 U1STA,	#UTXEN ; Enable transmit
	BSET	 IEC0,	#U1RXIE ; Enable receive interrupts
    RETURN
    
    __U1RXInterrupt:
    MOV    #15, W0	
	CALL   delay_ms		
	SETM    PORTB
	MOV	#1000,	    W0
	CALL    delay_ms    	
	MOV	U1RXREG,    W2	
	MOV	W2,	    PORTB
	MOV     W2,         U1TXREG
	MOV	#1000,	    W0
	CALL    delay_ms    	
	BCLR    IFS0,	    #U1RXIF
	RETFIE    

;--------End of All Code Sections ---------------------------------------------   

.end                               ;End of program code in this file
