;***********************************************
;                                        
;    Filename:	    ftb173_174.asm          
;    Date:          9/11/2004           
;    File Version:  0.9                  
;                                        
;    Author:        jeff bachiochi         
;    Company:       imagine that            
;                   www.imaginethatnow.com   
;                                            
;***********************************************
;                                       
;    Files required:            
;                               
;                                        
;                                        
;***********************************************
;                                         
;    Notes:                                 
;                                            
;   The flags2 bits can be set/cleared to enable and disable                                          
;	various serial output see more flags bits0-4 
;
;	PB1 - alternately choose RED/IR LED
;	PB2 - alternately choose the sensitivity of TSL230R
;			(proper sensitivity automatically chosen by program)
;	PB3 - alternately choose the divisor of TLS230R	
;			(proper divisor automatically chosen by program) 
;	PB4 - alternately chooses BPM or O2 output
;			(changing ratios are based on both LEDs
;				obviously only one is active at a time in this implementation)
;
;	Future Needs:
;	
;	Alternating LEDs will allow real time O2 results
;	Present program only designed for experimentation
;	Most interactive when used with Microchip's ICD2
;	careful with expectations as PORTB used for ICD2 functions                                         
;                                           
;***********************************************

DEBUG		EQU	0

	list		p=16f873A	; list directive to define processor
	#include	<p16f873A.inc>	; processor specific variable definitions
	
	__CONFIG _CP_OFF & _WDT_OFF & _BODEN_OFF & _PWRTE_ON & _XT_OSC & _WRT_OFF & _LVP_OFF & _CPD_OFF

;		list		p=12f629						; list directive to define processor
;		#include	<P12F629.inc>					; processor specific variable definitions
	
;		__CONFIG    _CP_OFF & _CPD_OFF & _PWRTE_ON & _WDT_OFF & _INTRC_OSC_NOCLKOUT & _MCLRE_OFF

; '__CONFIG' directive is used to embed configuration data within .asm file.
; The labels following the directive are located in the respective .inc file.
; See respective data sheet for additional information on configuration word.
;                                           
;***********************************************
;
;   	PORT DEFINITIONS
;                                           
;***********************************************
;
;		PORTA
S0				EQU	0
S1				EQU	1
S2				EQU	2	
S3				EQU	3	
;	
;	
;		PORTB
FREQ			EQU	0
;
;
;
SW1				EQU	4
SW2				EQU	5
SW3				EQU	6
SW4				EQU	7
;		PORTC
IR				EQU	0
RED				EQU	1
T1TRIG			EQU 2
T1SAMP			EQU 3
T1ERR			EQU 4
BEEPER			EQU	5
TX				EQU	6
;
;                                           
;***********************************************
;
;		CONSTANT DEFINITIONS
;                                           
;***********************************************
;
T				EQU		D'31220'
T1				EQU		T ^ 0xFFFF
T1L				EQU		LOW T1
T1H				EQU		HIGH T1
LEGALMAXL		EQU		LOW (T/2)
LEGALMAXH		EQU		HIGH (T/2)
LEGALMINL		EQU		D'60'
LEGALMINH		EQU		0
MAXCNTBPM		EQU		D'1920'/D'30'					; 30PBM where 1920 = 60BPM/.03125mS
MINCNTBPM		EQU		D'1920'/D'180'					; 180PBM where 1920 = 60BPM/.03125mS

; flags
TS				EQU		0						; xxxxx110 = got a new sample
SAMP1			EQU		1						; xxxxx111 = have read the new sample
SAMP2			EQU		2						; xxxxx001 = get a new sample
GOTAGOODSAMPLE	EQU		3						; 1=YES
LEGAL			EQU		4						; 1=YES
SLOPE			EQU		5						; 1=last change was new Max
TBIGGER			EQU		6						; 1=TEMPH/L is BIGGER
T1OVF			EQU		7						; 1=ERROR

; more flags
TimerTic		EQU		0						; TImer 1 overflow 31.25uS
Sample			EQU		1						; Sample Value
Beats			EQU		2					; 1 = BPM Value 0 = Saturation Value
MaxMin			EQU		3						; New Mean Value	
Saturation		EQU		4						; New Slope Direction	
DoSomething		EQU		5						; 	
COS_SLOPE0		EQU		6						; 1=slope just changed from 0 to 1	
COS_SLOPE1		EQU		7						; 1=slope just changed from 1 to 0	
;                                           
;***********************************************
;
;		VARIABLE DEFINITIONS
;                                           
;***********************************************
;
w_temp			EQU	0x20						; variable used for context saving
status_temp		EQU	0x21						; variable used for context saving
fsr_temp		EQU	0x22						; variable used for context saving
pclath_temp		EQU	0x23
OldSample		EQU	0x24
NewSample		EQU 0x25
MaxH			EQU	0x26
MaxL			EQU	0x27
MinH			EQU	0x28
MinL			EQU	0x29
T1SH			EQU	0x2A				
T1SL			EQU	0x2B
TIC				EQU	0x2C
MODE			EQU 0x2D						; xxx-LED-DIV-SEN							; LED 1=IR 0=RED
												; DIV 11=/100 10=/10 01=/2 00=/1
												; SEN 11=100X 10=10X 01=1X 00=PD																								
DIVISOR			EQU	0x2E					
LASTDIVISOR		EQU	0x2F
BUFFERHEAD		EQU 0x30
BUFFERTAIL		EQU 0x31
BUFFERSTART		EQU 0x32
;
BUFFEREND		EQU 0x51
BPMH			EQU	0x52
BPML			EQU	0x53
IR_RatioH		EQU	0x54
IR_RatioL		EQU	0x55
RED_RatioH		EQU	0x56
RED_RatioL		EQU	0x57
SAT				EQU	0x58
LASTMODE		EQU 0x59
CNT				EQU 0x5A
TEMP			EQU 0x5B
TOC				EQU 0x5C
MaxCNT			EQU	0x5D
AVG				EQU	0x5F
NUMOFSAMPS		EQU	0x60
TEMPH			EQU	0x61
TEMPL			EQU	0x62
TEMPM			EQU	0x63
flags1			EQU	0x64
flags2			EQU	0x65
LastMinH		EQU	0x66
LastMinL		EQU	0x67
LastMaxH		EQU	0x68
LastMaxL		EQU	0x69

w_temp_h	EQU	0xA0						; variable used for context saving
;
AARGB3		equ	0xB0
AARGB2		equ	0xB1
AARGB1		equ	0xB2
AARGB0		equ	0xB3
BARGB3		equ	0xB4
BARGB2		equ	0xB5
BARGB1		equ	0xB6
BARGB0		equ	0xB7
REMB3		equ	0xB8
REMB2		equ	0xB9
REMB1		equ	0xBA
REMB0		equ	0xBB
LOOPCOUNT	equ	0xBC
CHARCNT		EQU	0xBD
MATHTEMP	EQU	0xDE

M1_1H		EQU	0xC0
M1_1L		EQU	0xC1
M1_2H		EQU	0xC2
M1_2L		EQU	0xC3
M1_10H		EQU	0xC4
M1_10L		EQU	0xC5
M1_100H		EQU	0xC6
M1_100L		EQU	0xC7
M10_1H		EQU	0xC8
M10_1L		EQU	0xC9
M10_2H		EQU	0xCA
M10_2L		EQU	0xCB
M10_10H		EQU	0xCC
M10_10L		EQU	0xCD
M10_100H	EQU	0xCE
M10_100L	EQU	0xCF
M100_1H		EQU	0xD0
M100_1L		EQU	0xD1
M100_2H		EQU	0xD2
M100_2L		EQU	0xD3
M100_10H	EQU	0xD4
M100_10L	EQU	0xD5
M100_100H	EQU	0xD6
M100_100L	EQU	0xD7

;                                           
;***********************************************
;
;		RESET VECTOR
;                                           
;***********************************************
;
	ORG			0x000								; processor reset vector
  	goto		INIT							; go to beginning of program
;                                           
;***********************************************
;
;		INTERRUPT VECTOR
;                                           
;***********************************************
;
	ORG			0x004								; interrupt vector location
INT_VEC
	movwf		w_temp							; save off current W register contents
	movf		STATUS,W						; move status register into W register
	banksel		status_temp	
	movwf		status_temp						; save off contents of STATUS register
	movf		FSR,W
	movwf		fsr_temp
	movf		PCLATH,W
	movwf		pclath_temp
	
	pagesel		INT_VEC

	banksel		PIE1
	btfss		PIE1,TMR1IE
	goto		INT_CHKT0

	banksel		PIR1
	btfsc		PIR1,TMR1IF
	goto		INT_TMR1
				
INT_CHKT0	
	btfss		INTCON,T0IE
	goto		INT_CHKEX
	
	btfsc		INTCON,T0IF	
	goto		INT_TMR0

INT_CHKEX	
	btfss		INTCON,INTE	
	goto		INT_CHKTX
		
	btfsc		INTCON,INTF
	goto		INT_EXT	
	
INT_CHKTX
	banksel		PIE1	
	btfss		PIE1,TXIE
	goto		INT_CHKCOS
	
	banksel		PIR1
	btfsc		PIR1,TXIF
	goto		INT_TX	

INT_CHKCOS	
	btfss		INTCON,RBIE	
	goto		INT_EXIT
		
	btfsc		INTCON,RBIF
	goto		INT_COSB	

INT_EXIT
	banksel		pclath_temp
	movf		pclath_temp,W	
	movwf		PCLATH
	movf		fsr_temp,W
	movwf		FSR
	movf		status_temp,w					; retrieve copy of STATUS register
	movwf		STATUS							; restore pre-isr STATUS register contents
	movf		w_temp,W						; restore pre-isr W register contents
	retfie										; return from interrupt
;                                           
;***********************************************
;
;		INT_TX
;                                           
;***********************************************
;
INT_TX
	banksel		BUFFERTAIL
	movf		BUFFERTAIL,W
	movwf		FSR
	movf		INDF,W
	movwf		TXREG
	
	movf		FSR,W
	sublw		BUFFEREND
	btfss		STATUS,Z						; skip next if end of buffer 
	goto		INT_TX1
	
	movlw		BUFFERSTART-1
	movwf		FSR	
	
INT_TX1		
	incf		FSR	
	movf		FSR,W
	movwf		BUFFERTAIL
	bcf			PIR1,TXIF	

	movf		FSR,W
	subwf		BUFFERHEAD,W
	btfss		STATUS,Z						; skip next if at BUFFERHEAD
	goto		INT_TX_EXIT	
	
	banksel		PIE1
	bcf			PIE1,TXIE
	
INT_TX_EXIT	
	goto		INT_EXIT			
;                                           
;***********************************************
;
;		INT_TMR0
;                                           
;***********************************************
;
INT_TMR0
		banksel		TIC
		bcf			INTCON,T0IF
		movlw		1 << BEEPER
		xorwf		PORTC
		decfsz		TIC
		goto		INT_TMR0_EXIT

		bcf			INTCON,T0IE

INT_TMR0_EXIT	
		goto		INT_EXIT			
;                                           
;***********************************************
;
;		INT_TMR1 - 0.03125S timer 
;					set T1OVF flag if complete
;					sample has not been taken
;                                           
;***********************************************
;
INT_TMR1
		banksel		TMR1L
		bsf			PORTC,T1TRIG
		bcf			PORTC,T1TRIG
		bcf			T1CON,TMR1ON	
		movlw		T1L
		movwf		TMR1L
		movlw		T1H
		movwf		TMR1H
		bsf			T1CON,TMR1ON
		bcf			flags1,T1OVF	
		btfss		flags1,2					; skip next if  (no T1OVF error)	
		bsf			flags1,T1OVF		

		bsf			flags1,0
		bcf			flags1,1
		bcf			flags1,2
	
		btfss		flags2,TimerTic
		goto		INT_TMR1_1
		movlw		A'+'
		movwf		TXREG
	

INT_TMR1_1
		bcf			INTCON,INTF		
		bsf			INTCON,INTE
		
INT_TMR1_EXIT	
		banksel		PIR1
		bcf			PIR1,TMR1IF
		goto		INT_EXIT

;                                           
;***********************************************
;
;	INT_COSB
;                                           
;***********************************************
;	
INT_COSB	
	banksel		NewSample
	movf		PORTB,W
	movwf		NewSample
	bcf			INTCON,RBIF
	
INT_COSB_CHK1
	btfss		NewSample,SW1
	goto		INT_COSB_CHK2
	
INT_COSB_SW1
	btfsc		OldSample,SW1
	goto		INT_COSB_CHK2

	movlw		0x10
	xorwf		MODE
	btfss		MODE,4						; skip next if MODE.4=IR
	goto		INT_COSB_RED

INT_COSB_IR
	bcf			PORTC,RED	
	bsf			PORTC,IR
	bsf			MODE,4
	goto		INT_COSB_CHK2
	
INT_COSB_RED
	bsf			PORTC,RED
	bcf			PORTC,IR
	bcf			MODE,4
	goto		INT_COSB_CHK2

INT_COSB_CHK2
	btfss		NewSample,SW2
	goto		INT_COSB_CHK3
	
INT_COSB_SW2
	btfsc		OldSample,SW2
	goto		INT_COSB_CHK3
	
	incf		MODE
	movlw		0x03
	andwf		MODE,W
	btfss		STATUS,Z						; skip next if SEN = 0
	goto		INT_COSB_SW2_xx
	
	movlw		0x04
	xorwf		MODE

INT_COSB_SW2_xx
	btfss		MODE,0						; skip next if MODE.1/0=1X
	goto		INT_COSB_SW2_x0	
		
INT_COSB_SW2_x1
	btfss		MODE,1						; skip next if MODE.1/0=1X
	goto		INT_COSB_1X
	
INT_COSB_100X
	bsf			PORTA,S0
	bsf			PORTA,S1
	bsf			MODE,0
	bsf			MODE,1	
	goto		INT_COSB_CHK3

INT_COSB_SW2_x0
	btfss		MODE,1						; skip next if MODE.1/0=1X
	goto		INT_COSB_PD
	
INT_COSB_10X
	bcf			PORTA,S0
	bsf			PORTA,S1
	bcf			MODE,0
	bsf			MODE,1	
	goto		INT_COSB_CHK3

INT_COSB_PD
	bcf			PORTA,S0
	bcf			PORTA,S1
	bcf			MODE,0
	bcf			MODE,1	
	goto		INT_COSB_CHK3
	
INT_COSB_1X
	bsf			PORTA,S0
	bcf			PORTA,S1
	bsf			MODE,0
	bcf			MODE,1
	goto		INT_COSB_CHK3	

INT_COSB_CHK3
	btfss		NewSample,SW3
	goto		INT_COSB_CHK4			
	
INT_COSB_SW3	
	btfsc		OldSample,SW3
	goto		INT_COSB_CHK4

	movlw		0x04
	addwf		MODE
	movlw		0x0C
	andwf		MODE,W
	btfss		STATUS,Z						; skip next if SEN = 0
	goto		INT_COSB_SW3_xx
	
	movlw		0x01
	xorwf		MODE

INT_COSB_SW3_xx
	btfss		MODE,2						; skip next if MODE.1/0=1X
	goto		INT_COSB_SW3_x0	
		
INT_COSB_SW3_x1
	btfss		MODE,3						; skip next if MODE.1/0=1X
	goto		INT_COSB_2
	
INT_COS_100
	bsf			PORTA,S2
	bsf			PORTA,S3
	bsf			MODE,2
	bsf			MODE,3		
	goto		INT_COSB_CHK4

INT_COSB_SW3_x0
	btfss		MODE,3						; skip next if MODE.1/0=1X
	goto		INT_COSB_1
	
INT_COSB_10
	bcf			PORTA,S2
	bsf			PORTA,S3
	bcf			MODE,2
	bsf			MODE,3		
	goto		INT_COSB_CHK4

INT_COSB_1
	bcf			PORTA,S2
	bcf			PORTA,S3
	bcf			MODE,2
	bcf			MODE,3		
	goto		INT_COSB_CHK4
	
INT_COSB_2
	bsf			PORTA,S2
	bcf			PORTA,S3
	bsf			MODE,2
	bcf			MODE,3		
	goto		INT_COSB_CHK4	
	
INT_COSB_CHK4
	btfss		NewSample,SW4
	goto		INT_COSB_EXIT			
	
INT_COSB_SW4	
		btfsc		OldSample,SW4
		goto		INT_COSB_EXIT
		
		movlw		1<<Saturation
		xorwf		flags2
		movlw		1<<Beats
		xorwf		flags2		
		
		goto		INT_COSB_EXIT	
	
INT_COSB_EXIT
	movf		NewSample,W
	movwf		OldSample
	movf		MODE
	goto		INT_EXIT
;                                           
;***********************************************
;
;	INT_EXT
;		flags1 = xxxxx1xx - Time2 Sample taken
;		flags1 = xxxxxx1x - Time1 Sample taken
;		flags1 = xxxxx001 - Ready to take Sample
;		flags1 = xxxxx110 - Got a Sample
;                                           
;***********************************************
;
INT_EXT
	banksel		flags1
	btfsc		flags1,2							; skip next if Time2 flag NOT set
	goto		INT_EXT_EXIT
	
	btfsc		flags1,1							; skip next if Time1 flag NOT set
	goto		INT_EXT_T2
	
INT_EXT_T1
	banksel		TMR1L
	bsf			PORTC,T1SAMP
	bcf			PORTC,T1SAMP
	bcf			T1CON,TMR1ON
	movf		TMR1L,W
	xorlw		0xFF
	movwf		T1SL
	movf		TMR1H,W
	xorlw		0xFF
	movwf		T1SH	
	bsf			T1CON,TMR1ON
	bsf			flags1,1
	goto		INT_EXT_EXIT
	
INT_EXT_T2	
	bsf			PORTC,T1SAMP
	bcf			PORTC,T1SAMP
	bcf			T1CON,TMR1ON
	movf		TMR1L,W
	xorlw		0xFF
	subwf		T1SL
	btfss		STATUS,C						; skip next if NO borrow
	decf		T1SH
	movf		TMR1H,W
	xorlw		0xFF
	subwf		T1SH
	bsf			T1CON,TMR1ON	
	bsf			flags1,2
	bcf			flags1,0	
	bcf			INTCON,INTE
	
INT_EXT_EXIT
	bcf			INTCON,INTF
	goto		INT_EXIT		
;                                           
;***********************************************
;
;		INITIALIZATION
;                                           
;***********************************************
;
INIT
		pagesel		INIT
		banksel		PORTA
		movlw		b'11111111'					; xxHHLLLL ?, ?, ?,	?, S3, S2, S1, S0
		movwf		PORTA
		
		banksel		TRISA
		movlw		b'11110000'					; IIIIOOOO ?, ?, ?,	?, S3, S2, S1, S0
		movwf		TRISA
	
		banksel		PORTB
		movlw		b'11111111'					; HHHHHHHH ?, ?, SW4, SW3, ?, SW2, SW1, FREQ
		movwf		PORTB	

		banksel		TRISB
		movlw		b'11111111'					; IIIIIIII ?, ?, SW4, SW3, ?, SW2, SW1, FREQ
		movwf		TRISB
		
		banksel		PORTC
		movlw		b'11000010'					; HHHHHHLL RX, TX, BEEPER, ?, ?, ?, RED, IR
		movwf		PORTC
				
		banksel		TRISC
		movlw		b'10000000'					; IOOOOOOO RX, TX, BEEPER ?, ?, ?, RED, IR
		movwf		TRISC		

		banksel		OPTION_REG
		movlw		b'00001000'					; EnPU, EXINT Falling, Int Clk, EXCLK rising, tmr0 1:1
		movwf		OPTION_REG	

		banksel		ADCON1
		movlw		b'00000110'					; all digital inputs		
		movwf		ADCON1
		
		banksel		SPBRG
		movlw		D'12'							; 19.2k
		movwf		SPBRG
		bcf			TXSTA,SYNC
		bsf			TXSTA,TXEN
		bsf			TXSTA,BRGH						; HIGH SPEED		
		
		banksel		RCSTA
		bsf			RCSTA,SPEN
		
		banksel		CMCON
		movlw		b'00000111'					; all digital inputs		
		movwf		CMCON					

		banksel		TMR1L
		movlw		T1L
		movwf		TMR1L
		movlw		T1H
		movwf		TMR1H
		bsf			T1CON,TMR1ON
		bcf			PIR1,TMR1IF
		
		clrf		MODE
		movlw		BUFFERSTART
		movwf		BUFFERHEAD
		movwf		BUFFERTAIL
		
		banksel		PIE1
		bsf			PIE1,TMR1IE
		
		banksel		NewSample
		movf		PORTB,W
		movwf		NewSample
		movwf		OldSample				
		
		bcf			INTCON,RBIF
		bsf			INTCON,RBIE
		bcf			INTCON,INTF
		bsf			INTCON,INTE			
	
		bsf			INTCON,PEIE
		bsf			INTCON,GIE

		banksel		flags2		
		movlw		b'00000100'
		movwf		flags2
		
		movlw		0x04
		movwf		DIVISOR
		
		banksel		flags1		
		clrf		flags1
		clrf		IR_RatioH		
		clrf		IR_RatioL
		clrf		RED_RatioH
		clrf		RED_RatioL				
		
REINIT
		pagesel		RANGE				
		call		RANGE
		
		pagesel		BEST		
		call		BEST
		
		pagesel		SENDMODE		
		call		SENDMODE
		
		pagesel		SENDLEAK		
		call		SENDLEAK
		
		pagesel		SAMP_READY		
		call		SAMP_READY

		pagesel		REINIT
		banksel		T1SH
		movf		T1SH,W
		movwf		MaxH
		movwf		MinH
		movf		T1SL,W
		movwf		MaxL
		movwf		MinL
	
		bsf			flags1,SLOPE
		clrf		CNT	
		clrf		NUMOFSAMPS	
;                                           
;***********************************************
;
;		MAIN
;                                           
;***********************************************
;
MAIN
		pagesel		SAMP_READY	
		call		SAMP_READY
		
		pagesel		MAIN
		banksel		T1SH		
		movlw		0xFF
		subwf		T1SH,W
		btfsc		STATUS,Z					; skip next if NO error
		goto		MAIN_ERROR
		
		movf		LASTMODE,W
		subwf		MODE,W
		pagesel		SENDMODE		
		btfss		STATUS,Z						; skip next if MODE NOT changed
		call		SENDMODE

;		pagesel		MAIN
;		banksel		LASTDIVISOR
;		movf		LASTDIVISOR,W
;		subwf		DIVISOR,W
;		pagesel		SENDLEAK
;		btfss		STATUS,Z						; skip next if DIVISOR NOT changed
;		call		SENDLEAK		
				
		pagesel		MAIN				
		banksel		PIE1
		btfsc		PIE1,TXIE
		goto		$-3

		banksel		flags2				
		btfss		flags2,Sample
		goto		MAIN1
			
DISPLAY_SAMPLE_DATA
		banksel		T1SH
		movf		T1SH,W
		
		banksel		REMB0
		movwf		REMB0
		
		banksel		T1SL		
		movf		T1SL,W
		
		banksel		REMB1		
		movwf		REMB1
		
		pagesel		SENDDIGITS				
		call		SENDDIGITS		

MAIN1
		pagesel		MAIN1
		banksel		CNT
		incf		CNT
		btfsc		STATUS,Z					; skip next if NOT 0
		goto		MAIN_ERROR

MAIN_LOOK4MAX		
		bcf			flags2,DoSomething			; start off with nothing to do
		movf		T1SH,W
		subwf		MaxH,W
		btfss		STATUS,C					; skip next if SampleH <= MaxH	
		goto		MAIN_NEWMAX					; SampleH > MaxH
		
		btfss		STATUS,Z					; skip next if SampleH = MaxH
		goto		MAIN2						; SampleH < MaxH

		movf		T1SL,W
		subwf		MaxL,W
		btfsc		STATUS,C					; skip next if SampleL > MaxL	
		goto		MAIN2						; SampleL <= MaxL

MAIN_NEWMAX
		movf		T1SH,W						; save new Max
		movwf		MaxH
		movf		T1SL,W
		movwf		MaxL
		movf		CNT,W
		movwf		MaxCNT
		btfss		flags1,SLOPE				; skip next if SLOPE=1
		bsf			flags2,COS_SLOPE1			; SLOPE was 0 so set COS_SLOPE1
		bsf			flags1,SLOPE				; now (or still) SLOPE 1
		
MAIN2
		btfss		flags1,SLOPE				; skip next if SLOPE=1 (leaking MIN)
		goto		MAIN_MAX_LEAK				; SLOPE=0 go leak off MAX
		
		btfss		flags2,COS_SLOPE0			; skip next if SLOPE just changed to 0 
		goto		MAIN_LOOK4MIN				; slope did not just changed to 0 				
		
MAIN_MAXDO	
		movf		MinH,W
		movwf		LastMinH
		movf		MinL,W
		movwf		LastMinL
		
		movf		MaxCNT,W
		subwf		CNT
		
		bcf			flags2,COS_SLOPE0
		bsf			flags2,DoSomething		
			
		banksel		LastMaxH		
		movf		LastMaxH,W			
			
		banksel		AARGB0		
		movwf		AARGB0
		
		banksel		LastMaxL		
		movf		LastMaxL,W
		
		banksel		AARGB1		
		movwf		AARGB1
		
		banksel		LastMinH		
		movf		LastMinH,W
		
		banksel		BARGB0		
		movwf		BARGB0
		
		banksel		LastMinL		
		movf		LastMinL,W
		
		banksel		BARGB1		
		movwf		BARGB1
		
		pagesel		DIFFERENCE		
		call		DIFFERENCE
		
		pagesel		MAIN_MAXDO	
		banksel		AARGB2
		clrf		AARGB2
		clrf		AARGB3
				
		banksel		LastMaxH		
		movf		LastMaxH,W
		
		banksel		BARGB2		
		movwf		BARGB2
		
		banksel		LastMaxL		
		movf		LastMaxL,W
		
		banksel		BARGB3		
		movwf		BARGB3
		clrf		BARGB0
		clrf		BARGB1

MAIN_MAXDO_1		
		pagesel		FXD3232U		
		call		FXD3232U

		pagesel		MAIN_MAXDO			
		banksel		MODE		
		btfss		MODE,4						; skip next if IR MODE
		goto		RATIO_RED

RATIO_IR
		banksel		AARGB2		
		movf		AARGB2,W
		
		banksel		IR_RatioH		
		movwf		IR_RatioH
		
		banksel		AARGB3		
		movf		AARGB3,W

		banksel		IR_RatioL		
		movwf		IR_RatioL		
		goto		DISPLAY_SATURATION				
		
RATIO_RED	
		banksel		AARGB2		
		movf		AARGB2,W
		
		banksel		RED_RatioH		
		movwf		RED_RatioH
		
		banksel		AARGB3		
		movf		AARGB3,W
		
		banksel		RED_RatioL		
		movwf		RED_RatioL				

DISPLAY_SATURATION		
		pagesel		SATURATION
		banksel		flags2
		btfsc		flags2,Saturation
		call		SATURATION	
		
		pagesel		MAIN_LOOK4MIN
		goto		MAIN_LOOK4MIN		

MAIN_MAX_LEAK		
		banksel		MaxH		
		movf		MaxH,W
		
		banksel		AARGB0		
		movwf		AARGB0
		
		banksel		MaxL		
		movf		MaxL,W
		
		banksel		AARGB1		
		movwf		AARGB1
		
		banksel		MinH		
		movf		MinH,W
		
		banksel		BARGB0		
		movwf		BARGB0
		
		banksel		MinL		
		movf		MinL,W
		
		banksel		BARGB1		
		movwf		BARGB1
		
		pagesel		DIFFERENCE		
		call		DIFFERENCE
		
		pagesel		MAIN_MAX_LEAK		
		banksel		DIVISOR		
		movf		DIVISOR,W
		
		banksel		BARGB1		
		movwf		BARGB1
		clrf		BARGB0
		
		pagesel		FXD1616U		
		call		FXD1616U
		
		pagesel		MAIN_MAX_LEAK		
		movf		AARGB1,W
		
		banksel		MaxL		
		subwf		MaxL
		btfss		STATUS,C
		decf		MaxH
		
		banksel		AARGB0		
		movf		AARGB0,W
		
		banksel		MaxH		
		subwf		MaxH
		
MAIN_LOOK4MIN
		banksel		T1SH
		movf		T1SH,W
		subwf		MinH,W
		btfss		STATUS,C					; skip next if SampleH <= MinH	
		goto		MAIN3						; SampleH > MinH		
		
		btfss		STATUS,Z					; skip next if SampleH = MinH
		goto		MAIN_NEWMIN					; SampleH < MinH	
		
		movf		T1SL,W
		subwf		MinL,W
		btfss		STATUS,C					; skip next if SampleL <= MinL	
		goto		MAIN3						; SampleL > MinL
		
		btfsc		STATUS,Z					; skip next if SampleL NOT MinL
		goto		MAIN3						; SampleL = MinL			

MAIN_NEWMIN		
		movf		T1SH,W						; save new Min
		movwf		MinH
		movf		T1SL,W
		movwf		MinL
		btfsc		flags1,SLOPE				; skip next if no change in SLOPE
		bsf			flags2,COS_SLOPE0			; SLOPE has changed so set COS_SLOPE0
		bcf			flags1,SLOPE
		
MAIN3
		pagesel		MAIN3
		banksel		flags1
		btfsc		flags1,SLOPE				; skip next if SLOPE=0 (leaking MAX)
		goto		MAIN_MIN_LEAK				; SLOPE=1 go leak off MIN
		
		btfsc		flags2,COS_SLOPE1			; skip next if SLOPE did not just change to 1 
		goto		MAIN_MINDO					; slope just changed to 1 (found new maximum)
		goto		MAIN_EXIT
		
MAIN_MIN_LEAK
		movf		MaxH,W
		
		banksel		AARGB0		
		movwf		AARGB0
		
		banksel		MaxL		
		movf		MaxL,W
		
		banksel		AARGB1		
		movwf		AARGB1
		
		banksel		MinH		
		movf		MinH,W
		
		banksel		BARGB0		
		movwf		BARGB0
		
		banksel		MinL		
		movf		MinL,W
		
		banksel		BARGB1		
		movwf		BARGB1
		
		pagesel		DIFFERENCE		
		call		DIFFERENCE
		
		pagesel		MAIN_MIN_LEAK		
		banksel		DIVISOR		
		movf		DIVISOR,W
		
		banksel		BARGB1		
		movwf		BARGB1
		clrf		BARGB0
		
		pagesel		FXD1616U		
		call		FXD1616U	
		
		pagesel		MAIN_MIN_LEAK		
		movf		AARGB1,W
		
		banksel		MinL		
		addwf		MinL
		btfsc		STATUS,C					; skip next if no borrow
		incf		MinH
		
		banksel		AARGB0		
		movf		AARGB0,W
		
		banksel		MinH		
		addwf		MinH				
		goto		MAIN_EXIT
		
MAIN_MINDO	
		movf		MaxH,W
		movwf		LastMaxH
		movf		MaxL,W
		movwf		LastMaxL
		bcf			flags2,COS_SLOPE1		
		goto		MAIN_EXIT
		
MAIN_ERROR
		pagesel		RESTART
		call		RESTART	
		
		pagesel		REINIT					
		goto		REINIT
		
MAIN_EXIT
		banksel		flags2
		pagesel		MEAN				
		btfsc		flags2,MaxMin
		
DISPLAY_MAX_MIN		
		call		MEAN
		
MAIN_EXIT1
		pagesel		DO_SOMETHING
		btfsc		flags2,DoSomething
		call		DO_SOMETHING
		
		banksel		BUFFERHEAD
		pagesel		CR	
		movf		BUFFERHEAD,W	
		subwf		BUFFERTAIL,W			
		btfss		STATUS,Z					; skip next if nothing to display
		
DISPLAY_CR
		call		CR

		pagesel		MAIN	
		goto		MAIN
;                                           
;***********************************************
;
;		DO SOMETHING
;                                           
;***********************************************
;		
DO_SOMETHING
;		pagesel		SENDDIGITS
;		banksel		MaxCNT	
;		movf		MaxCNT,W
		
;		banksel		REMB1
;		movwf		REMB1
;		clrf		REMB0
;		call		SENDDIGITS
		
		pagesel		DO_SOMETHING
		banksel		MINCNTBPM					
		movlw		MINCNTBPM
		subwf		MaxCNT,W
		btfss		STATUS,C					; skip next if MaxCNT >= D'7'
		goto		TOOLOW
		
		movlw		MAXCNTBPM
		subwf		MaxCNT,W
		btfsc		STATUS,C					; skip next if MaxCNT < D'46'
		goto		TOOHIGH
				
		pagesel		BEEP				
		call		BEEP
		
		pagesel		DO_SOMETHING
		goto		DO_SOMETHING1
		
TOOHIGH											; TOO FEW PEAKS - use smaller DIVISOR
		banksel		DIVISOR
		rrf			DIVISOR,W
		rrf			DIVISOR		
		movf		AVG,W
		movwf		MaxCNT
		goto		DO_SOMETHING1
				
TOOLOW											; TOO MANY PEAKS - use bigger DIVISOR
		banksel		DIVISOR
		rlf			DIVISOR,W
		rlf			DIVISOR
		movf		AVG,W
		movwf		MaxCNT				


DO_SOMETHING1	
		pagesel		AVERAGE	
		call		AVERAGE

		pagesel		BPM		
		banksel		flags2
		btfsc		flags2,Beats
		call		BPM
		
DO_SOMETHING_EXIT
		return		
;                                           
;***********************************************
;
;		AVERAGE
;                                           
;***********************************************
;
AVERAGE
		pagesel		AVERAGE
		banksel		NUMOFSAMPS
		movf		NUMOFSAMPS,W
		btfsc		STATUS,Z					; skip next if NUMOFSAMPLES NOT 0
		goto		AVERAGE_INC

AVERAGE_1		
		movlw		0x01
		subwf		NUMOFSAMPS,W
		btfss		STATUS,Z					; skip next if NUMOFSAMPLES NOT 1
		goto		AVERAGE_2
		
		movf		MaxCNT,W
		movwf		AVG
		goto		AVERAGE_INC
		
AVERAGE_2
		banksel		AARGB0
		clrf		AARGB0
		clrf		AARGB1
		
		banksel		NUMOFSAMPS		
		movf		NUMOFSAMPS,W
		
		banksel		BARGB1		
		movwf		BARGB1
		decf		BARGB1
		
AVERAGE_LOOP
		banksel		AVG
		movf		AVG,W		
		
		banksel		AARGB1		
		addwf		AARGB1
		btfsc		STATUS,C
		incf		AARGB0
		decfsz		BARGB1
		goto		AVERAGE_LOOP
				
		banksel		MaxCNT				
		movf		MaxCNT,W
		
		banksel		AARGB1		
		addwf		AARGB1
		btfsc		STATUS,C
		incf		AARGB0
		movf		AARGB0,W
		
		banksel		BPMH		
		movwf		BPMH
		
		banksel		AARGB1		
		movf		AARGB1,W
		
		banksel		BPML		
		movwf		BPML
				
		movf		NUMOFSAMPS,W		
		
		banksel		BARGB1		
		movwf		BARGB1
		clrf		BARGB0
		
		pagesel		FXD1616U		
		call		FXD1616U
		
		pagesel		AVERAGE_LOOP		
		movf		AARGB1,W
		
		banksel		AVG		
		movwf		AVG
		
		movlw		0x08
		subwf		NUMOFSAMPS,W
		btfss		STATUS,Z					; skip next if NUMSAMPS = 8
		goto		AVERAGE_INC
		
		movf		BPMH,W
		
		banksel		BARGB0		
		movwf		BARGB0
		
		banksel		BPML		
		movf		BPML,W
		
		banksel		BARGB1		
		movwf		BARGB1
		movlw		0x3C
		movwf		AARGB0
		clrf		AARGB1
		
		pagesel		FXD1616U		
		call		FXD1616U
				
		pagesel		AVERAGE_LOOP
		movf		AARGB1,W
		
		banksel		BPML		
		movwf		BPML
		
		banksel		REMB0		
		btfss		REMB0,7						; skip next if remainder > 1/2
		goto		AVERAGE_3
		
		banksel		BPML		
		incf		BPML
		
AVERAGE_3	
		banksel		BPMH	
		clrf		BPMH		
		goto		AVERAGE_EXIT

AVERAGE_INC		
		incf		NUMOFSAMPS
		
AVERAGE_EXIT		
		return		
;                                           
;***********************************************
;
;		SENDDIGITS
;                                           
;***********************************************
;
SENDDIGITS
		pagesel		SENDDIGITS
		banksel		BUFFERHEAD
		movf		BUFFERHEAD,W
		movwf		FSR
		
		banksel		BARGB0		
		movlw		HIGH D'10000'
		movwf		BARGB0
		movlw		LOW D'10000'
		movwf		BARGB1	
		
		pagesel		CONV2DEC		
		call		CONV2DEC
		
		pagesel		SENDDIGITS		
		movlw		HIGH D'1000'
		movwf		BARGB0
		movlw		LOW D'1000'
		movwf		BARGB1	
		
		pagesel		CONV2DEC		
		call		CONV2DEC
		
		pagesel		SENDDIGITS		
		movlw		HIGH D'100'
		movwf		BARGB0
		movlw		LOW D'100'
		movwf		BARGB1	
		
		pagesel		CONV2DEC		
		call		CONV2DEC
		
		pagesel		SENDDIGITS		
		movlw		HIGH D'10'
		movwf		BARGB0
		movlw		LOW D'10'
		movwf		BARGB1	
		
		pagesel		CONV2DEC		
		call		CONV2DEC						
		
		pagesel		SENDDIGITS		
		movf		REMB1,W
		addlw		0x30
		movwf		INDF
		
		pagesel		CHK4END		
		call 		CHK4END	
		
		pagesel		SENDDIGITS		
		incf		FSR	
		movlw		A','
		movwf		INDF
		
		pagesel		CHK4END		
		call 		CHK4END	
		
		pagesel		SENDDIGITS			
		incf		FSR
		movf		FSR,W
		
		banksel		BUFFERHEAD		
		movwf		BUFFERHEAD

		return
;                                           
;***********************************************
;
;		CR - Adds a CR to the buffer and starts
;				serial output
;
;                                           
;***********************************************
;
CR	
		pagesel		CR
		banksel		BUFFERHEAD
		movf		BUFFERHEAD,W
		movwf		FSR			
		movlw		0x0D
		movwf		INDF
		
		pagesel		CHK4END		
		call 		CHK4END
		
		pagesel		CR		
		incf		FSR	
		movf		FSR,W
		movwf		BUFFERHEAD
		banksel		PIE1
		bsf			PIE1,TXIE


		return		
;                                           
;***********************************************
;
;		RANGE - select all mode ranges and save samples
;
;
;                                           
;***********************************************
;
RANGE
		pagesel		CALIBRATE
		call		CALIBRATE
		
RANGE1_1
		pagesel		RANGE1_1
		banksel		T1SH
		bsf			PORTA,S0
		bcf			PORTA,S1
		bcf			PORTA,S2
		bcf			PORTA,S3
		
		pagesel		SAMP_READY		
		call		SAMP_READY

		pagesel		SAMP_READY		
		call		SAMP_READY
				
		pagesel		RANGE1_1
		banksel		T1SH						
		movf		T1SH,W
		
		banksel		M1_1H
		movwf		M1_1H

		banksel		T1SL		
		movf		T1SL,W
		
		banksel		M1_1L		
		movwf		M1_1L
		
		banksel		PIE1
		btfsc		PIE1,TXIE
		goto		$-3
		
		banksel		T1SH		
		movf		T1SH,W
		
		banksel		REMB0		
		movwf		REMB0
		
		banksel		T1SL		
		movf		T1SL,W
		
		banksel		REMB1		
		movwf		REMB1
		
		pagesel		SENDDIGITS					
		call		SENDDIGITS
		
		pagesel		CR		
		call		CR						

RANGE1_2
		pagesel		RANGE1_2
		banksel		T1SH
		bsf			PORTA,S0
		bcf			PORTA,S1
		bsf			PORTA,S2
		bsf			PORTA,S3
		
		pagesel		SAMP_READY		
		call		SAMP_READY
		
		pagesel		SAMP_READY		
		call		SAMP_READY
				
		pagesel		RANGE1_2
		banksel		T1SH						
		movf		T1SH,W
		
		banksel		M1_2H		
		movwf		M1_2H
		
		banksel		T1SL		
		movf		T1SL,W
		
		banksel		M1_2L		
		movwf		M1_2L
				
		banksel		PIE1
		btfsc		PIE1,TXIE
		goto		$-3
		
		banksel		T1SH		
		movf		T1SH,W
		
		banksel		REMB0		
		movwf		REMB0
		
		banksel		T1SL		
		movf		T1SL,W
		
		banksel		REMB1		
		movwf		REMB1
		
		pagesel		SENDDIGITS				
		call		SENDDIGITS
		
		pagesel		CR		
		call		CR
				
RANGE1_10
		pagesel		RANGE1_10
		banksel		T1SH
		bsf			PORTA,S0
		bcf			PORTA,S1
		bcf			PORTA,S2
		bsf			PORTA,S3
		
		pagesel		SAMP_READY		
		call		SAMP_READY
		
		pagesel		SAMP_READY		
		call		SAMP_READY
				
		pagesel		RANGE1_10
		banksel		T1SH						
		movf		T1SH,W
		
		banksel		M1_10H		
		movwf		M1_10H
		
		banksel		T1SL		
		movf		T1SL,W
		
		banksel		M1_10L		
		movwf		M1_10L
				
		banksel		PIE1
		btfsc		PIE1,TXIE
		goto		$-3
		
		banksel		T1SH		
		movf		T1SH,W
		
		banksel		REMB0		
		movwf		REMB0
		
		banksel		T1SL		
		movf		T1SL,W
		
		banksel		REMB1		
		movwf		REMB1
		
		pagesel		SENDDIGITS			
		call		SENDDIGITS
		
		pagesel		CR		
		call		CR		
				
RANGE1_100
		pagesel		RANGE1_100
		banksel		T1SH
		bsf			PORTA,S0
		bcf			PORTA,S1
		bsf			PORTA,S2
		bsf			PORTA,S3
		
		pagesel		SAMP_READY		
		call		SAMP_READY
		
		pagesel		SAMP_READY		
		call		SAMP_READY
				
		pagesel		RANGE1_100
		banksel		T1SH						
		movf		T1SH,W
		
		banksel		M1_100H		
		movwf		M1_100H
		
		banksel		T1SL		
		movf		T1SL,W
		
		banksel		M1_100L		
		movwf		M1_100L
			
		banksel		PIE1
		btfsc		PIE1,TXIE
		goto		$-3
		
		banksel		T1SH		
		movf		T1SH,W
		
		banksel		REMB0		
		movwf		REMB0
		
		banksel		T1SL		
		movf		T1SL,W
		
		banksel		REMB1		
		movwf		REMB1
		
		pagesel		SENDDIGITS				
		call		SENDDIGITS
		
		pagesel		CR		
		call		CR
		
RANGE10_1
		pagesel		RANGE10_1
		banksel		T1SH
		bcf			PORTA,S0
		bsf			PORTA,S1
		bcf			PORTA,S2
		bcf			PORTA,S3
		
		pagesel		SAMP_READY		
		call		SAMP_READY
		
		pagesel		SAMP_READY		
		call		SAMP_READY
				
		pagesel		RANGE10_1
		banksel		T1SH						
		movf		T1SH,W
		
		banksel		M10_1H		
		movwf		M10_1H
		
		banksel		T1SL		
		movf		T1SL,W
		
		banksel		M10_1L		
		movwf		M10_1L
		
		banksel		PIE1
		btfsc		PIE1,TXIE
		goto		$-3
		
		banksel		T1SH		
		movf		T1SH,W
		
		banksel		REMB0		
		movwf		REMB0
		
		banksel		T1SL		
		movf		T1SL,W
		
		banksel		REMB1		
		movwf		REMB1
		
		pagesel		SENDDIGITS					
		call		SENDDIGITS
		
		pagesel		CR		
		call		CR	
				
RANGE10_2
		pagesel		RANGE10_2
		banksel		T1SH
		bcf			PORTA,S0
		bsf			PORTA,S1
		bsf			PORTA,S2
		bcf			PORTA,S3
		
		pagesel		SAMP_READY
		call		SAMP_READY

		pagesel		SAMP_READY			
		call		SAMP_READY
				
		pagesel		RANGE10_2
		banksel		T1SH						
		movf		T1SH,W
		
		banksel		M10_2H		
		movwf		M10_2H
		
		banksel		T1SL		
		movf		T1SL,W
		
		banksel		M10_2L		
		movwf		M10_2L
		
		banksel		PIE1
		btfsc		PIE1,TXIE
		goto		$-3
		
		banksel		T1SH		
		movf		T1SH,W
		
		banksel		REMB0		
		movwf		REMB0
		
		banksel		T1SL		
		movf		T1SL,W
		
		banksel		REMB1		
		movwf		REMB1
		
		pagesel		SENDDIGITS					
		call		SENDDIGITS
		
		pagesel		CR	
		call		CR		
		
RANGE10_10
		pagesel		RANGE10_10
		banksel		T1SH
		bcf			PORTA,S0
		bsf			PORTA,S1
		bcf			PORTA,S2
		bsf			PORTA,S3
		
		pagesel		SAMP_READY		
		call		SAMP_READY
	
		pagesel		SAMP_READY	
		call		SAMP_READY
				
		pagesel		RANGE10_10	
		banksel		T1SH			
		movf		T1SH,W
		
		banksel		M10_10H		
		movwf		M10_10H
		
		banksel		T1SL		
		movf		T1SL,W
		
		banksel		M10_10L		
		movwf		M10_10L	
		
		banksel		PIE1
		btfsc		PIE1,TXIE
		goto		$-3
		
		banksel		T1SH		
		movf		T1SH,W
		
		banksel		REMB0		
		movwf		REMB0
		
		banksel		T1SL		
		movf		T1SL,W
		
		banksel		REMB1		
		movwf		REMB1
		
		pagesel		SENDDIGITS					
		call		SENDDIGITS
		
		pagesel		CR		
		call		CR		
				
RANGE10_100
		pagesel		RANGE10_100
		banksel		T1SH
		bcf			PORTA,S0
		bsf			PORTA,S1
		bsf			PORTA,S2
		bsf			PORTA,S3
		
		pagesel		SAMP_READY		
		call		SAMP_READY

		pagesel		SAMP_READY
		call		SAMP_READY
				
		pagesel		RANGE10_100	
		banksel		T1SH					
		movf		T1SH,W
		
		banksel		M10_100H		
		movwf		M10_100H
		
		banksel		T1SL		
		movf		T1SL,W
		
		banksel		M10_100L		
		movwf		M10_100L
		
		banksel		PIE1
		btfsc		PIE1,TXIE
		goto		$-3
		
		banksel		T1SH		
		movf		T1SH,W
		
		banksel		REMB0		
		movwf		REMB0
		
		banksel		T1SL		
		movf		T1SL,W
		
		banksel		REMB1		
		movwf		REMB1
		
		pagesel		SENDDIGITS					
		call		SENDDIGITS
		
		pagesel		CR		
		call		CR		
				
RANGE100_1
		pagesel		RANGE100_1
		banksel		T1SH
		bsf			PORTA,S0
		bsf			PORTA,S1
		bcf			PORTA,S2
		bcf			PORTA,S3
		
		pagesel		SAMP_READY		
		call		SAMP_READY

		pagesel		SAMP_READY	
		call		SAMP_READY
				
		pagesel		RANGE100_1	
		banksel		T1SH					
		movf		T1SH,W
		
		banksel		M100_1H		
		movwf		M100_1H
		
		banksel		T1SL		
		movf		T1SL,W
		
		banksel		M100_1L		
		movwf		M100_1L	
		
		banksel		PIE1
		btfsc		PIE1,TXIE
		goto		$-3

		banksel		T1SH		
		movf		T1SH,W
		
		banksel		REMB0		
		movwf		REMB0
		
		banksel		T1SL		
		movf		T1SL,W
		
		banksel		REMB1		
		movwf		REMB1
		
		pagesel		SENDDIGITS					
		call		SENDDIGITS
		
		pagesel		CR		
		call		CR		
				
RANGE100_2
		pagesel		RANGE100_2
		banksel		T1SH
		bsf			PORTA,S0
		bsf			PORTA,S1
		bsf			PORTA,S2
		bcf			PORTA,S3
		
		pagesel		SAMP_READY		
		call		SAMP_READY
	
		pagesel		SAMP_READY	
		call		SAMP_READY
				
		pagesel		RANGE100_2	
		banksel		T1SH					
		movf		T1SH,W
		
		banksel		M100_2H		
		movwf		M100_2H
		
		banksel		T1SL		
		movf		T1SL,W
		
		banksel		M100_2L		
		movwf		M100_2L
		
		banksel		PIE1
		btfsc		PIE1,TXIE
		goto		$-3
		
		banksel		T1SH
		movf		T1SH,W
		
		banksel		REMB0
		movwf		REMB0

		banksel		T1SL		
		movf		T1SL,W
		
		banksel		REMB1		
		movwf		REMB1
		
		pagesel		SENDDIGITS					
		call		SENDDIGITS
		
		pagesel		CR		
		call		CR		
		
RANGE100_10
		pagesel		RANGE100_10
		banksel		T1SH
		bsf			PORTA,S0
		bsf			PORTA,S1
		bcf			PORTA,S2
		bsf			PORTA,S3
		
		pagesel		SAMP_READY		
		call		SAMP_READY

		pagesel		SAMP_READY
		call		SAMP_READY

		pagesel		RANGE100_10	
		banksel		T1SH					
		movf		T1SH,W
		
		banksel		M100_10H		
		movwf		M100_10H
		
		banksel		T1SL		
		movf		T1SL,W
		
		banksel		M100_10L		
		movwf		M100_10L
		
		banksel		PIE1
		btfsc		PIE1,TXIE
		goto		$-3
		
		banksel		T1SH		
		movf		T1SH,W
		
		banksel		REMB0		
		movwf		REMB0
		
		banksel		T1SL		
		movf		T1SL,W
		
		banksel		REMB1		
		movwf		REMB1
		
		pagesel		SENDDIGITS					
		call		SENDDIGITS
		
		pagesel		CR		
		call		CR		
				
RANGE100_100
		pagesel		RANGE100_100
		banksel		T1SH
		bsf			PORTA,S0
		bsf			PORTA,S1
		bsf			PORTA,S2
		bsf			PORTA,S3
		
		pagesel		SAMP_READY		
		call		SAMP_READY

		pagesel		SAMP_READY
		call		SAMP_READY

		pagesel		RANGE100_100
		banksel		T1SH						
		movf		T1SH,W
		
		banksel		M100_100H		
		movwf		M100_100H
		
		banksel		T1SL		
		movf		T1SL,W
		
		banksel		M100_100L		
		movwf		M100_100L
		
		banksel		PIE1
		btfsc		PIE1,TXIE
		goto		$-3
		
		banksel		T1SH		
		movf		T1SH,W
		
		banksel		REMB0		
		movwf		REMB0
		
		banksel		T1SL		
		movf		T1SL,W
		
		banksel		REMB1		
		movwf		REMB1
		
		pagesel		SENDDIGITS					
		call		SENDDIGITS
		
		pagesel		CR		
		call		CR		
			
		return
;                                           
;***********************************************
;
;		BEST_MODE - choose the best mode for the signal present
;                                           
;***********************************************
;
BEST
		pagesel		BEST
		banksel		TEMPM
		movlw		0xFF
		movwf		TEMPM
		bcf			flags1,GOTAGOODSAMPLE
		
BEST_CHK1_1
		banksel		M1_1H
		movf		M1_1H,W
		movwf		AARGB0
		movf		M1_1L,W
		movwf		AARGB1
		
		pagesel		CHK4LEGAL		
		CALL		CHK4LEGAL
		
		pagesel		BEST_CHK1_1		
		banksel		flags1			
		btfss		flags1,LEGAL					; skip next if LEGAL
		goto		BEST_CHK1_2
		
BEST_1_1		
		movlw		0x01						; LLLH 1X1
		movwf		TEMPM
		
		banksel		M1_1H		
		movf		M1_1H,W
		
		banksel		TEMPH
		movwf		TEMPH
		
		banksel		M1_1L		
		movf		M1_1L,W
		
		banksel		TEMPL		
		movwf		TEMPL
		bsf			flags1,GOTAGOODSAMPLE
		
BEST_CHK1_2	
		banksel		M1_2H
		movf		M1_2H,W
		movwf		AARGB0
		movf		M1_2L,W
		movwf		AARGB1
		
		pagesel		CHK4LEGAL		
		CALL		CHK4LEGAL	
		
		pagesel		BEST_CHK1_2		
		banksel		flags1		
		btfss		flags1,LEGAL					; skip next if LEGAL
		goto		BEST_CHK1_10
		
		btfss		flags1,GOTAGOODSAMPLE		; skip next if we've got a good sample
		goto		BEST1_2

		pagesel		CHK4TBIGGER
		CALL		CHK4TBIGGER

		pagesel		BEST_CHK1_2			
		banksel		flags1		
		btfsc		flags1,TBIGGER				; skip next if TEMPH/L NOT BIGGER than ARGB0/1
		goto		BEST_CHK1_10
	
BEST1_2	
		movlw		0x05						; LHLH 1X2
		banksel		TEMPM		
		movwf		TEMPM
		
		banksel		M1_2H		
		movf		M1_2H,W
		
		banksel		TEMPH		
		movwf		TEMPH
		
		banksel		M1_2L		
		movf		M1_2L,W
		
		banksel		TEMPL		
		movwf		TEMPL
		bsf			flags1,GOTAGOODSAMPLE
				
BEST_CHK1_10		
		banksel		M1_10H
		movf		M1_10H,W
		movwf		AARGB0
		movf		M1_10L,W
		movwf		AARGB1
		
		pagesel		CHK4LEGAL			
		CALL		CHK4LEGAL	
		
		pagesel		BEST_CHK1_10		
		banksel		flags1		
		btfss		flags1,LEGAL					; skip next if LEGAL
		goto		BEST_CHK1_100
		
		btfss		flags1,GOTAGOODSAMPLE		; skip next if we've got a good sample
		goto		BEST1_10

		pagesel		CHK4TBIGGER
		CALL		CHK4TBIGGER
		
		pagesel		BEST_CHK1_10		
		banksel		flags1
		btfsc		flags1,TBIGGER				; skip next if TEMPH/L NOT BIGGER than ARGB0/1
		goto		BEST_CHK1_100	
		
BEST1_10	
		movlw		0x09						; HLLH 1X10
		banksel		TEMPM		
		movwf		TEMPM
		
		banksel		M1_10H		
		movf		M1_10H,W
		
		banksel		TEMPH		
		movwf		TEMPH
		
		banksel		M1_10L		
		movf		M1_10L,W
		
		banksel		TEMPL		
		movwf		TEMPL		
		bsf			flags1,GOTAGOODSAMPLE
		
BEST_CHK1_100		
		banksel		M1_100H
		movf		M1_100H,W
		movwf		AARGB0
		movf		M1_100L,W
		movwf		AARGB1
		
		pagesel		CHK4LEGAL		
		CALL		CHK4LEGAL
		
		pagesel		BEST_CHK1_100		
		banksel		flags1			
		btfss		flags1,LEGAL					; skip next if LEGAL
		goto		BEST_CHK10_1
		
		btfss		flags1,GOTAGOODSAMPLE		; skip next if we've got a good sample
		goto		BEST1_100

		pagesel		CHK4TBIGGER
		CALL		CHK4TBIGGER

		pagesel		BEST_CHK1_100
		banksel		flags1		
		btfsc		flags1,TBIGGER				; skip next if TEMPH/L NOT BIGGER than ARGB0/1
		goto		BEST_CHK10_1	
		
BEST1_100	
		movlw		0x0D						; HHLH 1X100
		banksel		TEMPM		
		movwf		TEMPM
		
		banksel		M1_100H		
		movf		M1_100H,W
		
		banksel		TEMPH		
		movwf		TEMPH
		
		banksel		M1_100L		
		movf		M1_100L,W
		
		banksel		TEMPL		
		movwf		TEMPL
		bsf			flags1,GOTAGOODSAMPLE
				
BEST_CHK10_1		
		banksel		M10_1H
		movf		M10_1H,W
		movwf		AARGB0
		movf		M10_1L,W
		movwf		AARGB1
		
		pagesel		CHK4LEGAL		
		CALL		CHK4LEGAL	
		
		pagesel		BEST_CHK10_1		
		banksel		flags1		
		btfss		flags1,LEGAL					; skip next if LEGAL
		goto		BEST_CHK10_2
		
		btfss		flags1,GOTAGOODSAMPLE		; skip next if we've got a good sample
		goto		BEST10_1

		pagesel		CHK4TBIGGER
		CALL		CHK4TBIGGER

		pagesel		BEST_CHK10_1
		banksel		flags1		
		btfsc		flags1,TBIGGER				; skip next if TEMPH/L NOT BIGGER than ARGB0/1
		goto		BEST_CHK10_2	
		
BEST10_1	
		movlw		0x02						; LLHL 10X1
		banksel		TEMPM		
		movwf		TEMPM
		
		banksel		M10_1H		
		movf		M10_1H,W
		
		banksel		TEMPH		
		movwf		TEMPH
		
		banksel		M10_1L		
		movf		M10_1L,W
		
		banksel		TEMPL		
		movwf		TEMPL
		bsf			flags1,GOTAGOODSAMPLE
				
BEST_CHK10_2		
		banksel		M10_2H
		movf		M10_2H,W
		movwf		AARGB0
		movf		M10_2L,W
		movwf		AARGB1
		
		pagesel		CHK4LEGAL			
		CALL		CHK4LEGAL	
		
		pagesel		BEST_CHK10_2		
		banksel		flags1		
		btfss		flags1,LEGAL					; skip next if LEGAL
		goto		BEST_CHK10_10
		
		btfss		flags1,GOTAGOODSAMPLE		; skip next if we've got a good sample
		goto		BEST10_2

		pagesel		CHK4TBIGGER
		CALL		CHK4TBIGGER

		pagesel		BEST_CHK10_2
		banksel		flags1		
		btfsc		flags1,TBIGGER				; skip next if TEMPH/L NOT BIGGER than ARGB0/1
		goto		BEST_CHK10_10			
		
BEST10_2	
		movlw		0x06						; LHHL 10X2
		banksel		TEMPM		
		movwf		TEMPM
		
		banksel		M10_2H		
		movf		M10_2H,W
		
		banksel		TEMPH		
		movwf		TEMPH
		
		banksel		M10_2L		
		movf		M10_2L,W
		
		banksel		TEMPL		
		movwf		TEMPL		
		bsf			flags1,GOTAGOODSAMPLE
		
BEST_CHK10_10		
		banksel		M10_10H
		movf		M10_10H,W
		movwf		AARGB0
		movf		M10_10L,W
		movwf		AARGB1
		
		pagesel		CHK4LEGAL			
		CALL		CHK4LEGAL	
		
		pagesel		BEST_CHK10_10		
		banksel		flags1		
		btfss		flags1,LEGAL					; skip next if LEGAL
		goto		BEST_CHK10_100
		
		btfss		flags1,GOTAGOODSAMPLE		; skip next if we've got a good sample
		goto		BEST10_10

		pagesel		CHK4TBIGGER	
		CALL		CHK4TBIGGER

		pagesel		BEST_CHK10_10
		banksel		flags1		
		btfsc		flags1,TBIGGER				; skip next if TEMPH/L NOT BIGGER than ARGB0/1
		goto		BEST_CHK10_100

BEST10_10	
		movlw		0x0A						; HLHL 10X2
		banksel		TEMPM		
		movwf		TEMPM
		
		banksel		M10_10H		
		movf		M10_10H,W
		
		banksel		TEMPH		
		movwf		TEMPH
		
		banksel		M10_10L		
		movf		M10_10L,W
		
		banksel		TEMPL		
		movwf		TEMPL
		bsf			flags1,GOTAGOODSAMPLE
				
BEST_CHK10_100
		banksel		M10_100H		
		movf		M10_100H,W
		movwf		AARGB0
		movf		M10_100L,W
		movwf		AARGB1
		
		pagesel		CHK4LEGAL					
		CALL		CHK4LEGAL	
		
		pagesel		BEST_CHK10_100		
		banksel		flags1		
		btfss		flags1,LEGAL					; skip next if LEGAL
		goto		BEST_CHK100_1
		
		btfss		flags1,GOTAGOODSAMPLE		; skip next if we've got a good sample
		goto		BEST10_100

		pagesel		CHK4TBIGGER
		CALL		CHK4TBIGGER

		pagesel		BEST_CHK10_100
		banksel		flags1		
		btfsc		flags1,TBIGGER				; skip next if TEMPH/L NOT BIGGER than ARGB0/1
		goto		BEST_CHK100_1
		
BEST10_100	
		movlw		0x0E						; HHHL 10X100
		banksel		TEMPM		
		movwf		TEMPM
		
		banksel		M10_100H		
		movf		M10_100H,W
		
		banksel		TEMPH		
		movwf		TEMPH
		
		banksel		M10_100L		
		movf		M10_100L,W
		
		banksel		TEMPL		
		movwf		TEMPL
		bsf			flags1,GOTAGOODSAMPLE
				
BEST_CHK100_1		
		banksel		M100_1H
		movf		M100_1H,W
		movwf		AARGB0
		movf		M100_1L,W
		movwf		AARGB1
		
		pagesel		CHK4LEGAL		
		CALL		CHK4LEGAL	
		
		pagesel		BEST_CHK100_1		
		banksel		flags1		
		btfss		flags1,LEGAL					; skip next if LEGAL
		goto		BEST_CHK100_2
		
		btfss		flags1,GOTAGOODSAMPLE		; skip next if we've got a good sample
		goto		BEST100_1

		pagesel		CHK4TBIGGER
		CALL		CHK4TBIGGER
		
		pagesel		BEST_CHK100_1	
		banksel		flags1		
		btfsc		flags1,TBIGGER				; skip next if TEMPH/L NOT BIGGER than ARGB0/1
		goto		BEST_CHK100_2
		
BEST100_1	
		movlw		0x03						; LLHH 100X1
		banksel		TEMPM		
		movwf		TEMPM
		
		banksel		M100_1H		
		movf		M100_1H,W
		
		banksel		TEMPH		
		movwf		TEMPH
		
		banksel		M100_1L		
		movf		M100_1L,W
		
		banksel		TEMPL		
		movwf		TEMPL
		bsf			flags1,GOTAGOODSAMPLE
				
BEST_CHK100_2		
		banksel		M100_2H
		movf		M100_2H,W
		movwf		AARGB0
		movf		M100_2L,W
		movwf		AARGB1
		
		pagesel		CHK4LEGAL		
		CALL		CHK4LEGAL	
		
		pagesel		BEST_CHK100_2		
		banksel		flags1		
		btfss		flags1,LEGAL					; skip next if LEGAL
		goto		BEST_CHK100_10
		
		btfss		flags1,GOTAGOODSAMPLE		; skip next if we've got a good sample
		goto		BEST100_2

		pagesel		CHK4TBIGGER
		CALL		CHK4TBIGGER

		pagesel		BEST_CHK100_2
		banksel		flags1		
		btfsc		flags1,TBIGGER				; skip next if TEMPH/L NOT BIGGER than ARGB0/1
		goto		BEST_CHK100_10
		
BEST100_2	
		movlw		0x07						; LHHH 100X2
		banksel		TEMPM		
		movwf		TEMPM
		
		banksel		M100_2H		
		movf		M100_2H,W
		
		banksel		TEMPH		
		movwf		TEMPH
		
		banksel		M100_2L		
		movf		M100_2L,W
		
		banksel		TEMPL		
		movwf		TEMPL		
		bsf			flags1,GOTAGOODSAMPLE
		
BEST_CHK100_10		
		banksel		M100_10H
		movf		M100_10H,W
		movwf		AARGB0
		movf		M100_10L,W
		movwf		AARGB1
		
		pagesel		CHK4LEGAL		
		CALL		CHK4LEGAL	
		
		pagesel		BEST_CHK100_10		
		banksel		flags1		
		btfss		flags1,LEGAL					; skip next if LEGAL
		goto		BEST_CHK100_100
		
		btfss		flags1,GOTAGOODSAMPLE		; skip next if we've got a good sample
		goto		BEST100_10

		pagesel		CHK4TBIGGER
		CALL		CHK4TBIGGER

		pagesel		BEST_CHK100_10
		banksel		flags1		
		btfsc		flags1,TBIGGER				; skip next if TEMPH/L NOT BIGGER than ARGB0/1
		goto		BEST_CHK100_100
		
BEST100_10	
		movlw		0x0B						; HLHH 100X10
		banksel		TEMPM		
		movwf		TEMPM
		
		banksel		M100_10H		
		movf		M100_10H,W
		
		banksel		TEMPH		
		movwf		TEMPH
		
		banksel		M100_10L		
		movf		M100_10L,W
		
		banksel		TEMPL		
		movwf		TEMPL
		bsf			flags1,GOTAGOODSAMPLE
				
BEST_CHK100_100		
		banksel		M100_100H
		movf		M100_100H,W
		movwf		AARGB0
		movf		M100_100L,W
		movwf		AARGB1
		
		pagesel		CHK4LEGAL		
		CALL		CHK4LEGAL	
		
		pagesel		BEST_CHK100_100		
		banksel		flags1		
		btfss		flags1,LEGAL					; skip next if LEGAL
		goto		BEST_EXIT
		
		btfss		flags1,GOTAGOODSAMPLE		; skip next if we've got a good sample
		goto		BEST100_100

		pagesel		CHK4TBIGGER
		CALL		CHK4TBIGGER

		pagesel		BEST_CHK100_100
		banksel		flags1		
		btfsc		flags1,TBIGGER				; skip next if TEMPH/L NOT BIGGER than ARGB0/1
		goto		BEST_EXIT
		
BEST100_100	
		movlw		0x0F						; HHHH 100X100
		banksel		TEMPM		
		movwf		TEMPM
		
		banksel		M100_10H		
		movf		M100_10H,W
		
		banksel		TEMPH		
		movwf		TEMPH

		banksel		M100_10L		
		movf		M100_10L,W
		
		banksel		TEMPL		
		movwf		TEMPL
		bsf			flags1,GOTAGOODSAMPLE		
				
BEST_EXIT
		btfsc		flags1,GOTAGOODSAMPLE		; skip next if we've got NO good sample
		goto		BEST_EXIT1

		pagesel		BEEP
		CALL		BEEP
		
		pagesel		BEST_EXIT		
		banksel		PORTC		
		movlw		0x03
		xorwf		PORTC
		nop
		xorwf		PORTC
		goto		BEST
		
BEST_EXIT1
		banksel		MODE
		movlw		0xF0
		andwf		MODE
		movf		TEMPM,W
		iorwf		MODE
		movf		PORTA,W
		andlw		0xF0
		iorwf		TEMPM,W
		movwf		PORTA		
		return
;                                           
;***********************************************
;
;		TBIGGER - Is TEMPH/L Larger than AARGB0/1?
;			flags1.TBIGGER - 1=YES 0=NO
;                                           
;***********************************************
;
CHK4TBIGGER
		pagesel		CHK4TBIGGER
		banksel		flags1
		bcf			flags1,TBIGGER				; start out not BIGGER		
		movf		TEMPH,W
		
		banksel		AARGB0		
		subwf		AARGB0,W
		btfss		STATUS,C					; skip next if TEMPH <= AARGB0
		goto		CHK4TBIGGER_IS				; TEMPL > AARGB1
		
		btfss		STATUS,Z					; skip next if TEMPH = AARGB0
		goto		CHK4TBIGGER_EXIT		

		banksel		TEMPL
		movf		TEMPL,W
		
		banksel		AARGB1		
		subwf		AARGB1,W
		btfss		STATUS,C					; skip next if TEMPL <= AARGB1
		goto		CHK4TBIGGER_IS			    ; TEMPH <= AARGB0

		btfss		STATUS,Z		
		goto		CHK4TBIGGER_EXIT		    ; TEMPH < AARGB0
		
CHK4TBIGGER_IS
		banksel		flags1
		bsf			flags1,TBIGGER				; is BIGGER	
				
CHK4TBIGGER_EXIT		
		return		
;                                           
;***********************************************
;
;		CHK4LEGAL - Is ARGB0/1 Between LEGALMAXH/L and LEGALMINH/L?
;			flags1.LEGAL - 1=YES 0=NO 
;                                           
;***********************************************
;
CHK4LEGAL
		pagesel		CHK4LEGAL
		banksel		flags1
CHK4LEGAL_MAX
		bcf			flags1,LEGAL					; start out not LEGAL		
		movlw		LEGALMAXH
		
		banksel		AARGB0		
		subwf		AARGB0,W
		btfss		STATUS,C					; skip next if AARGB0 >= LEGALMAXH
		goto		CHK4LEGAL_MIN				; AARGB0 < LEGALMAXH
		
		btfss		STATUS,Z					; skip next if AARGB0 = LEGALMAXH
		goto		CHK4LEGAL_EXIT				; AARGB0 > LEGALMAXH		

		banksel		LEGALMAXL
		movlw		LEGALMAXL
		
		banksel		AARGB1		
		subwf		AARGB1,W
		btfss		STATUS,C					; skip next if AARGB1 >= LEGALMAXL
		goto		CHK4LEGAL_MIN				; AARGB1 < LEGALMAXL

		btfss		STATUS,Z					; skip next if AARGB1 = LEGALMAXL
		goto		CHK4LEGAL_EXIT				; AARGB1 > LEGALMAXL

CHK4LEGAL_MIN		
		banksel		LEGALMINH
		movlw		LEGALMINH
		
		banksel		AARGB0		
		subwf		AARGB0,W
		btfss		STATUS,C					; skip next if AARGB0 >= LEGALMINH
		goto		CHK4LEGAL_EXIT				; AARGB0 < LEGALMINH
		
		btfss		STATUS,Z					; skip next if AARGB0 = LEGALMINH
		goto		CHK4LEGAL_GOOD				; AARGB0 > LEGALMINH		

		banksel		LEGALMINL
		movlw		LEGALMINL
		
		banksel		AARGB1		
		subwf		AARGB1,W
		btfss		STATUS,C					; skip next if AARGB1 >= LEGALMINL
		goto		CHK4LEGAL_EXIT				; AARGB1 < LEGALMINL

CHK4LEGAL_GOOD		
		banksel		flags1					
		bsf			flags1,LEGAL
		
CHK4LEGAL_EXIT		
		return
;
;
		ORG		0x800
;		
;                                           
;***********************************************
;
;		DIFFERENCE - AARGBx-BARGBx
;                                           
;***********************************************
;
DIFFERENCE
		pagesel		DIFFERENCE
		banksel		BARGB1
		movf		BARGB1,W
		subwf		AARGB1
		btfss		STATUS,C					; skip next if no borrow
		decf		AARGB0
		movf		BARGB0,W
		subwf		AARGB0
		return	
;                                           
;***********************************************
;
;		SEND LEAKAGE
;                                           
;***********************************************
;				
SENDLEAK
		pagesel		SENDLEAK
		banksel		PIE1
		btfsc		PIE1,TXIE
		goto		$-3
		banksel		BUFFERHEAD				
		movf		BUFFERHEAD,W
		movwf		FSR
		movlw		A'L'
		movwf		INDF
		
		pagesel		CHK4END			
		call 		CHK4END	
		
		pagesel		SENDLEAK			
		incf		FSR
		movlw		A'E'
		movwf		INDF
		
		pagesel		CHK4END		
		call 		CHK4END	
		
		pagesel		SENDLEAK			
		incf		FSR
		movlw		A'A'
		movwf		INDF
		
		pagesel		CHK4END		
		call 		CHK4END
		
		pagesel		SENDLEAK			
		incf		FSR
		movlw		A'K'
		movwf		INDF
		
		pagesel		CHK4END		
		call 		CHK4END	
		
		pagesel		SENDLEAK			
		incf		FSR
		movlw		A'-'
		movwf		INDF
		
		pagesel		CHK4END		
		call 		CHK4END	
		
		pagesel		SENDLEAK			
		incf		FSR
		btfsc		DIVISOR,0
		goto		SENDLEAK_1
		btfsc		DIVISOR,1
		goto		SENDLEAK_2
		btfsc		DIVISOR,2
		goto		SENDLEAK_4
		btfsc		DIVISOR,3
		goto		SENDLEAK_8
		btfsc		DIVISOR,4
		goto		SENDLEAK_16
		btfsc		DIVISOR,5
		goto		SENDLEAK_32
		btfsc		DIVISOR,6
		goto		SENDLEAK_64
	
SENDLEAK_128
		movlw		A'/'
		movwf		INDF
		
		pagesel		CHK4END		
		call 		CHK4END	
		
		pagesel		SENDLEAK_128			
		incf		FSR
		movlw		A'1'
		movwf		INDF
		
		pagesel		CHK4END
		call 		CHK4END	
		
		pagesel		SENDLEAK_128			
		incf		FSR		
		movlw		A'2'
		movwf		INDF
		
		pagesel		CHK4END		
		call 		CHK4END
		
		pagesel		SENDLEAK_128		
		incf		FSR	
		movlw		A'8'
		movwf		INDF
		
		pagesel		CHK4END		
		call 		CHK4END	
		
		pagesel		SENDLEAK_128			
		incf		FSR
		goto		SENDLEAK_EXIT
		
SENDLEAK_64
		movlw		A'/'
		movwf		INDF
		
		pagesel		CHK4END		
		call 		CHK4END	
		
		pagesel		SENDLEAK_64				
		incf		FSR
		movlw		A'6'
		movwf		INDF
		
		pagesel		CHK4END		
		call 		CHK4END	
		
		pagesel		SENDLEAK_64				
		incf		FSR		
		movlw		A'4'
		movwf		INDF
		
		pagesel		CHK4END		
		call 		CHK4END	
		
		pagesel		SENDLEAK_64			
		incf		FSR
		goto		SENDLEAK_EXIT
		
SENDLEAK_32
		movlw		A'/'
		movwf		INDF
		
		pagesel		CHK4END		
		call 		CHK4END	
		
		pagesel		SENDLEAK_32			
		incf		FSR
		movlw		A'3'
		movwf		INDF
		
		pagesel		CHK4END		
		call 		CHK4END	
		
		pagesel		SENDLEAK_32				
		incf		FSR		
		movlw		A'2'
		movwf		INDF
		
		pagesel		CHK4END		
		call 		CHK4END		
		
		pagesel		SENDLEAK_32			
		incf		FSR
		goto		SENDLEAK_EXIT

SENDLEAK_16
		movlw		A'/'
		movwf		INDF
		
		pagesel		CHK4END
		call 		CHK4END		
		
		pagesel		SENDLEAK_16		
		incf		FSR
		movlw		A'1'
		movwf		INDF
		
		pagesel		CHK4END		
		call 		CHK4END	
		
		pagesel		SENDLEAK_16				
		incf		FSR		
		movlw		A'6'
		movwf		INDF
		
		pagesel		CHK4END		
		call 		CHK4END		
		
		pagesel		SENDLEAK_16			
		incf		FSR
		goto		SENDLEAK_EXIT
		
SENDLEAK_8
		movlw		A'/'
		movwf		INDF
				
		pagesel		CHK4END
		call 		CHK4END		
		
		pagesel		SENDLEAK_8		
		incf		FSR
		movlw		A'8'
		movwf		INDF
				
		pagesel		CHK4END
		call 		CHK4END	
		
		pagesel		SENDLEAK_8			
		incf		FSR
		goto		SENDLEAK_EXIT
		
SENDLEAK_4
		movlw		A'/'
		movwf		INDF
				
		pagesel		CHK4END
		call 		CHK4END	
		
		pagesel		SENDLEAK_4			
		incf		FSR
		movlw		A'4'
		movwf		INDF
				
		pagesel		CHK4END
		call 		CHK4END	
		
		pagesel		SENDLEAK_4			
		incf		FSR
		goto		SENDLEAK_EXIT
		
SENDLEAK_2
		movlw		A'/'
		movwf		INDF
				
		pagesel		CHK4END
		call 		CHK4END	
		
		pagesel		SENDLEAK_2			
		incf		FSR
		movlw		A'2'
		movwf		INDF
				
		pagesel		CHK4END
		call 		CHK4END	
		
		pagesel		SENDLEAK_2			
		incf		FSR
		goto		SENDLEAK_EXIT
		
SENDLEAK_1
		movlw		A'/'
		movwf		INDF
				
		pagesel		CHK4END
		call 		CHK4END	
		
		pagesel		SENDLEAK_1			
		incf		FSR
		movlw		A'1'
		movwf		INDF
				
		pagesel		CHK4END
		call 		CHK4END	
		
		pagesel		SENDLEAK_1			
		incf		FSR
		goto		SENDLEAK_EXIT														
		
SENDLEAK_EXIT
		movlw		0x0D
		movwf		INDF
				
		pagesel		CHK4END
		call 		CHK4END
		
		pagesel		SENDLEAK_EXIT		
		incf		FSR			
		movf		FSR,W
		movwf		BUFFERHEAD
		banksel		PIE1
		bsf			PIE1,TXIE
		banksel		DIVISOR
		movf		DIVISOR,W
		movwf		LASTDIVISOR			
		return						
;                                           
;***********************************************
;
;		SEND MODE
;                                           
;***********************************************
;				
SENDMODE
		
		pagesel		SENDMODE
		banksel		PIE1
		btfsc		PIE1,TXIE
		goto		$-3
		banksel		BUFFERHEAD				
		movf		BUFFERHEAD,W
		movwf		FSR
		movlw		A'M'
		movwf		INDF
				
		pagesel		CHK4END		
		call 		CHK4END	
		
		pagesel		SENDMODE			
		incf		FSR
		movlw		A'O'
		movwf		INDF
				
		pagesel		CHK4END
		call 		CHK4END	
		
		pagesel		SENDMODE			
		incf		FSR
		movlw		A'D'
		movwf		INDF
				
		pagesel		CHK4END
		call 		CHK4END	
		
		pagesel		SENDMODE			
		incf		FSR
		movlw		A'E'
		movwf		INDF
		
		pagesel		CHK4END
		call 		CHK4END	

		pagesel		SENDMODE	
		incf		FSR
		movlw		A'-'
		movwf		INDF
				
		pagesel		CHK4END
		call 		CHK4END	
		
		pagesel		SENDMODE			
		incf		FSR
		btfss		MODE,4
		goto		SENDMODE_RED

SENDMODE_IR
		movlw		A'I'
		movwf		INDF
				
		pagesel		CHK4END
		call 		CHK4END	
		
		pagesel		SENDMODE_IR			
		incf		FSR
		movlw		A'R'
		movwf		INDF
				
		pagesel		CHK4END
		call 		CHK4END
		
		pagesel		SENDMODE_IR		
		incf		FSR	
		movlw		A'-'
		movwf		INDF
				
		pagesel		CHK4END
		call 		CHK4END	
		
		pagesel		SENDMODE_IR			
		incf		FSR
		goto		SENDMODE_SENxx
		
SENDMODE_RED
		movlw		A'R'
		movwf		INDF
				
		pagesel		CHK4END
		call 		CHK4END	
		
		pagesel		SENDMODE_RED			
		incf		FSR
		movlw		A'E'
		movwf		INDF
				
		pagesel		CHK4END
		call 		CHK4END	
		
		pagesel		SENDMODE_RED
		incf		FSR	
		movlw		A'D'
		movwf		INDF
				
		pagesel		CHK4END
		call 		CHK4END		
		
		pagesel		SENDMODE_RED	
		incf		FSR	
		movlw		A'-'
		movwf		INDF
				
		pagesel		CHK4END
		call 		CHK4END		
		
		pagesel		SENDMODE_RED	
		incf		FSR				
		goto		SENDMODE_SENxx
		
SENDMODE_SENxx
		btfss		MODE,1
		goto		SENDMODE_SEN0x

SENDMODE_SEN1x
		btfss		MODE,0
		goto		SENDMODE_SEN10
		
SENDMODE_SEN11
		movlw		A'1'
		movwf		INDF
				
		pagesel		CHK4END
		call 		CHK4END		
		
		pagesel		SENDMODE_SEN11	
		incf		FSR	
		movlw		A'0'
		movwf		INDF
				
		pagesel		CHK4END
		call 		CHK4END		
		
		pagesel		SENDMODE_SEN11	
		incf		FSR	
		movlw		A'0'
		movwf		INDF
				
		pagesel		CHK4END
		call 		CHK4END			
		
		pagesel		SENDMODE_SEN11
		incf		FSR	
		movlw		A'X'
		movwf		INDF
				
		pagesel		CHK4END
		call 		CHK4END			
		
		pagesel		SENDMODE_SEN11
		incf		FSR	
		movlw		A'-'
		movwf		INDF
				
		pagesel		CHK4END
		call 		CHK4END	
		
		pagesel		SENDMODE_SEN11		
		incf		FSR						
		goto		SENDMODE_DIVxx
		
SENDMODE_SEN10
		movlw		A'1'
		movwf		INDF
				
		pagesel		CHK4END
		call 		CHK4END	
		
		pagesel		SENDMODE_SEN10		
		incf		FSR	
		movlw		A'0'
		movwf		INDF
				
		pagesel		CHK4END
		call 		CHK4END	
		
		pagesel		SENDMODE_SEN10		
		incf		FSR	
		movlw		A'X'
		movwf		INDF
				
		pagesel		CHK4END
		call 		CHK4END	
		
		pagesel		SENDMODE_SEN10		
		incf		FSR	
		movlw		A'-'
		movwf		INDF
				
		pagesel		CHK4END
		call 		CHK4END	
		
		pagesel		SENDMODE_SEN10		
		incf		FSR						
		goto		SENDMODE_DIVxx
				
SENDMODE_SEN0x		
		btfss		MODE,0
		goto		SENDMODE_SEN00
		
SENDMODE_SEN01
		movlw		A'1'
		movwf		INDF
				
		pagesel		CHK4END
		call 		CHK4END	
		
		pagesel		SENDMODE_SEN01		
		incf		FSR	
		movlw		A'X'
		movwf		INDF
				
		pagesel		CHK4END
		call 		CHK4END		
		
		pagesel		SENDMODE_SEN01	
		incf		FSR	
		movlw		A'-'
		movwf		INDF
				
		pagesel		CHK4END
		call 		CHK4END		
		
		pagesel		SENDMODE_SEN01	
		incf		FSR						
		goto		SENDMODE_DIVxx
		
SENDMODE_SEN00	
		movlw		A'P'
		movwf		INDF
				
		pagesel		CHK4END
		call 		CHK4END	
		
		pagesel		SENDMODE_SEN00		
		incf		FSR	
		movlw		A'D'
		movwf		INDF
				
		pagesel		CHK4END
		call 		CHK4END			
		
		pagesel		SENDMODE_SEN00
		incf		FSR	
		movlw		A'-'
		movwf		INDF
				
		pagesel		CHK4END
		call 		CHK4END			
		
		pagesel		SENDMODE_SEN00
		incf		FSR				
		goto		SENDMODE_DIVxx		
		
SENDMODE_DIVxx
		btfss		MODE,3
		goto		SENDMODE_DIV0x

SENDMODE_DIV1x
		btfss		MODE,2
		goto		SENDMODE_DIV10
		
SENDMODE_DIV11
		movlw		A'/'
		movwf		INDF
				
		pagesel		CHK4END
		call 		CHK4END	
		
		pagesel		SENDMODE_DIV11		
		incf		FSR	
		movlw		A'1'
		movwf		INDF
				
		pagesel		CHK4END
		call 		CHK4END		
		
		pagesel		SENDMODE_DIV11	
		incf		FSR	
		movlw		A'0'
		movwf		INDF
				
		pagesel		CHK4END
		call 		CHK4END	
		
		pagesel		SENDMODE_DIV11		
		incf		FSR	
		movlw		A'0'
		movwf		INDF
				
		pagesel		CHK4END
		call 		CHK4END	
		
		pagesel		SENDMODE_DIV11		
		incf		FSR					
		goto		SENDMODE_EXIT
SENDMODE_DIV10	
		movlw		A'/'
		movwf		INDF
				
		pagesel		CHK4END
		call 		CHK4END	
		
		pagesel		SENDMODE_DIV10		
		incf		FSR	
		movlw		A'1'
		movwf		INDF
				
		pagesel		CHK4END
		call 		CHK4END	
		
		pagesel		SENDMODE_DIV10		
		incf		FSR	
		movlw		A'0'
		movwf		INDF
				
		pagesel		CHK4END
		call 		CHK4END	
		
		pagesel		SENDMODE_DIV10		
		incf		FSR				
		goto		SENDMODE_EXIT
				
SENDMODE_DIV0x		
		btfss		MODE,2
		goto		SENDMODE_DIV00
		
SENDMODE_DIV01
		movlw		A'/'
		movwf		INDF
				
		pagesel		CHK4END
		call 		CHK4END		
		
		pagesel		SENDMODE_DIV01	
		incf		FSR	
		movlw		A'2'
		movwf		INDF
				
		pagesel		CHK4END
		call 		CHK4END	
		
		pagesel		SENDMODE_DIV01		
		incf		FSR			
		goto		SENDMODE_EXIT
		
SENDMODE_DIV00	
		movlw		A'/'
		movwf		INDF
				
		pagesel		CHK4END
		call 		CHK4END	
		
		pagesel		SENDMODE_DIV00		
		incf		FSR	
		movlw		A'1'
		movwf		INDF
				
		pagesel		CHK4END
		call 		CHK4END		
		
		pagesel		SENDMODE_DIV00	
		incf		FSR												
		goto		SENDMODE_EXIT		
		
SENDMODE_EXIT
		movlw		0x0D
		movwf		INDF
				
		pagesel		CHK4END
		call 		CHK4END	
		
		pagesel		SENDMODE_EXIT
		incf		FSR			
		movf		FSR,W
		movwf		BUFFERHEAD
		banksel		PIE1
		bsf			PIE1,TXIE
		banksel		MODE
		movf		MODE,W
		movwf		LASTMODE			
		return		
;                                           
;***********************************************
;
;		SAMP_READY - wait here for a new sample
;						(flags1 = xxxxx110) or
;						timer1 has overflowed
;						(flags1 = 1xxxxxxx)
;					overflow = error andis marked by
;						T2SH/L = 0xFFFF 
;                                           
;***********************************************
;
SAMP_READY
		pagesel		SAMP_READY
		banksel		flags1
		bsf			PORTC,T1ERR
		
SAMP_READY_WAIT		
		btfsc		flags1,T1OVF					; skip next if no error
		goto		SAMP_READYERR
		movlw		0x07
		andwf		flags1,W
		sublw		0x06
		btfss		STATUS,Z					; skip next if a new sample	
		goto		SAMP_READY_WAIT
		
SAMP_READY_EXIT		
		bsf			flags1,0						; Got the Sample (flags1=xxxxx111)
		bcf			PORTC,T1ERR	
		return	
		
SAMP_READYERR
		movlw		0xFF
		movwf		T1SH
		movwf		T1SL
		bcf			flags1,T1OVF
		goto		SAMP_READY_EXIT					
;                                           
;***********************************************
;
;		CONV2DEC
;                                           
;***********************************************
;
CONV2DEC
		pagesel		CONV2DEC
		banksel		REMB0
		movf		REMB0,W
		movwf		AARGB0
		movf		REMB1,W
		movwf		AARGB1
		
		pagesel		FXD1616U		
		call		FXD1616U
		
		pagesel		CONV2DEC		
		movf		AARGB1,W
		addlw		0x30
		movwf		INDF
		
		pagesel		CHK4END		
		call 		CHK4END	
		
		pagesel		CONV2DEC		
		incf		FSR	
		return	
;                                           
;***********************************************
;
;		BEEP
;                                           
;***********************************************
;
BEEP
		pagesel		BEEP
		bsf			INTCON,T0IE	

		return
;
;***********************************************
;
; 16/16 bit unsigned fixed point divide 16/16 -> 16.16
;
;	Input:	16 bit unsigned fixed point dividend in AARGB0, AARGB1
;		16 bit unsigned fixed point divisor in BARGB0, BARGB1
;
;	Use:	CALL	FXD1616U
;
;	Output:	16 bit unsigned fixed point quotient in AARGB0, AARGB1
;		16 bit unsigned fixed point remainder in REMB0, REMB1
;
;	Result:	AARG, REM <-- AARG/BARG
;
;***********************************************
;
FXD1616U
		pagesel	FXD1616U
		banksel	REMB0	
		clrf	REMB0
		clrf	REMB1
		movlw	0x10
		movwf	LOOPCOUNT

LOOPU1616
		rlf		AARGB0,w
		rlf		REMB1
		rlf		REMB0
		movf	BARGB1,w
		subwf	REMB1
		movf	BARGB0,w
		btfss	STATUS,C
		incfsz	BARGB0,w
		subwf	REMB0

		btfsc	STATUS,C
		goto	UOK66LL
		movf	BARGB1,w
		addwf	REMB1
		movf	BARGB0,w
		btfsc	STATUS,C
		incfsz	BARGB0,w
		addwf	REMB0

		bcf		STATUS,C

UOK66LL
		rlf		AARGB1
		rlf		AARGB0
		
		decfsz	LOOPCOUNT
		goto	LOOPU1616

		return
;*************************************
;       32/32 Bit Unsigned Fixed Point Divide 32/32 -> 32.32
;       Input:  32 bit unsigned fixed point dividend in AARGB0, AARGB1,AARGB2,AARGB3
;               32 bit unsigned fixed point divisor in BARGB0, BARGB1, BARGB2, BARGB3
;       Use:    CALL    FXD3232U
;       Output: 32 bit unsigned fixed point quotient in AARGB0, AARGB1,AARGB2,AARGB3
;               32 bit unsigned fixed point remainder in REMB0, REMB1, REMB2, REMB3
;       Result: AARG, REM  <--  AARG / BARG
;       Max Timing:     4+1025+2 = 1031 clks
;       Max Timing:     4+981+2 = 987 clks
;       PM: 4+359+1 = 364               DM: 13

FXD3232U
		pagesel	FXD3232U
		banksel	REMB0
		CLRF	REMB0
		CLRF	REMB1
		CLRF	REMB2
		CLRF	REMB3

		CLRF	MATHTEMP

		RLF		AARGB0,W
		RLF		REMB3
		MOVF	BARGB3,W
		SUBWF	REMB3
 		MOVF	BARGB2,W
 		BTFSS	STATUS,C
 		INCFSZ	BARGB2,W
 		SUBWF	REMB2
 		MOVF	BARGB1,W
 		BTFSS	STATUS,C
 		INCFSZ	BARGB1,W
 		SUBWF	REMB1
 		MOVF	BARGB0,W
 		BTFSS	STATUS,C
 		INCFSZ	BARGB0,W
 		SUBWF	REMB0

		CLRW
		BTFSS	STATUS,C
		MOVLW	1
		SUBWF	MATHTEMP
		RLF		AARGB0
		MOVLW	7
		MOVWF	LOOPCOUNT

LOOPU3232A
		RLF		AARGB0,W
		RLF		REMB3
		RLF		REMB2
		RLF		REMB1
		RLF		REMB0
		RLF		MATHTEMP
		MOVF	BARGB3,W
		BTFSS	AARGB0,0
		GOTO	UADD22LA

		SUBWF	REMB3
		MOVF	BARGB2,W
		BTFSS	STATUS,C
		INCFSZ	BARGB2,W
		SUBWF	REMB2
		MOVF	BARGB1,W
		BTFSS	STATUS,C
		INCFSZ	BARGB1,W
		SUBWF	REMB1
		MOVF	BARGB0,W
		BTFSS	STATUS,C
		INCFSZ	BARGB0,W
		SUBWF	REMB0
		CLRW
		BTFSS	STATUS,C
		MOVLW	1
		SUBWF	MATHTEMP
		GOTO	UOK22LA

UADD22LA 
		ADDWF	REMB3
		MOVF	BARGB2,W
		BTFSC	STATUS,C
		INCFSZ	BARGB2,W
		ADDWF	REMB2
		MOVF	BARGB1,W
		BTFSC	STATUS,C
		INCFSZ	BARGB1,W
		ADDWF	REMB1
		MOVF	BARGB0,W
		BTFSC	STATUS,C
		INCFSZ	BARGB0,W
		ADDWF	REMB0
		CLRW
		BTFSC	STATUS,C
		MOVLW	1
		ADDWF	MATHTEMP
		        
UOK22LA
		RLF		AARGB0, F

		DECFSZ	LOOPCOUNT
		GOTO	LOOPU3232A

		RLF		AARGB1,W
		RLF		REMB3
		RLF		REMB2
		RLF		REMB1
		RLF		REMB0
		RLF		MATHTEMP
		MOVF	BARGB3,W
		BTFSS	AARGB0,0
		GOTO	UADD22L8

		SUBWF	REMB3
		MOVF	BARGB2,W
		BTFSS	STATUS,C
		INCFSZ	BARGB2,W
		SUBWF	REMB2
		MOVF	BARGB1,W
		BTFSS	STATUS,C
		INCFSZ	BARGB1,W
		SUBWF	REMB1
		MOVF	BARGB0,W
		BTFSS	STATUS,C
		INCFSZ	BARGB0,W
		SUBWF	REMB0
		CLRW
		BTFSS	STATUS,C
		MOVLW	1
		SUBWF	MATHTEMP
		GOTO	UOK22L8

UADD22L8
        ADDWF	REMB3
        MOVF	BARGB2,W
        BTFSC	STATUS,C
        INCFSZ	BARGB2,W
        ADDWF	REMB2
        MOVF	BARGB1,W
        BTFSC	STATUS,C
        INCFSZ	BARGB1,W
        ADDWF	REMB1
        MOVF	BARGB0,W
        BTFSC	STATUS,C
        INCFSZ	BARGB0,W
        ADDWF	REMB0
        CLRW
        BTFSC	STATUS,C
        MOVLW	1
        ADDWF	MATHTEMP, F
        
UOK22L8
		RLF		AARGB1, F

		MOVLW	7
		MOVWF	LOOPCOUNT

LOOPU3232B
		RLF		AARGB1,W
		RLF		REMB3
		RLF		REMB2
		RLF		REMB1
		RLF		REMB0
		RLF		MATHTEMP
		MOVF	BARGB3,W
		BTFSS	AARGB1,0
		GOTO	UADD22LB

		SUBWF	REMB3
		MOVF	BARGB2,W
		BTFSS	STATUS,C
		INCFSZ	BARGB2,W
		SUBWF	REMB2
		MOVF	BARGB1,W
		BTFSS	STATUS,C
		INCFSZ	BARGB1,W
		SUBWF	REMB1
		MOVF	BARGB0,W
		BTFSS	STATUS,C
		INCFSZ	BARGB0,W
		SUBWF	REMB0
		CLRW
		BTFSS	STATUS,C
		MOVLW	1
		SUBWF	MATHTEMP
		GOTO	UOK22LB

UADD22LB
        ADDWF	REMB3
        MOVF	BARGB2,W
        BTFSC	STATUS,C
        INCFSZ	BARGB2,W
        ADDWF	REMB2
        MOVF	BARGB1,W
        BTFSC	STATUS,C
        INCFSZ	BARGB1,W
        ADDWF	REMB1
        MOVF	BARGB0,W
        BTFSC	STATUS,C
        INCFSZ	BARGB0,W
        ADDWF	REMB0
        CLRW
        BTFSC	STATUS,C
        MOVLW	1
        ADDWF	MATHTEMP
        
UOK22LB
		RLF		AARGB1

		DECFSZ	LOOPCOUNT
		GOTO	LOOPU3232B
		
		RLF		AARGB2,W
		RLF		REMB3
		RLF		REMB2
		RLF		REMB1
		RLF		REMB0
		RLF		MATHTEMP
		MOVF	BARGB3,W
		BTFSS	AARGB1,0
		GOTO	UADD22L16
		
		SUBWF	REMB3
		MOVF	BARGB2,W
		BTFSS	STATUS,C
		INCFSZ	BARGB2,W
		SUBWF	REMB2
		MOVF	BARGB1,W
		BTFSS	STATUS,C
		INCFSZ	BARGB1,W
		SUBWF	REMB1
		MOVF	BARGB0,W
		BTFSS	STATUS,C
		INCFSZ	BARGB0,W
		SUBWF	REMB0
		CLRW
		BTFSS	STATUS,C
		MOVLW	1
		SUBWF	MATHTEMP
		GOTO	UOK22L16

UADD22L16
		ADDWF	REMB3
		MOVF	BARGB2,W
		BTFSC	STATUS,C
		INCFSZ	BARGB2,W
		ADDWF	REMB2
		MOVF	BARGB1,W
		BTFSC	STATUS,C
		INCFSZ	BARGB1,W
		ADDWF	REMB1
		MOVF	BARGB0,W
		BTFSC	STATUS,C
		INCFSZ	BARGB0,W
		ADDWF	REMB0
		CLRW
		BTFSC	STATUS,C
		MOVLW	1
		ADDWF	MATHTEMP
        
UOK22L16        
		RLF		AARGB2

		MOVLW	7
		MOVWF	LOOPCOUNT

LOOPU3232C      
		RLF		AARGB2,W
		RLF		REMB3
		RLF		REMB2
		RLF		REMB1
		RLF		REMB0
		RLF		MATHTEMP
		MOVF	BARGB3,W
		BTFSS	AARGB2,0
		GOTO	UADD22LC
		
		SUBWF	REMB3
		MOVF	BARGB2,W
		BTFSS	STATUS,C
		INCFSZ	BARGB2,W
		SUBWF	REMB2
		MOVF	BARGB1,W
		BTFSS	STATUS,C
		INCFSZ	BARGB1,W
		SUBWF	REMB1
		MOVF	BARGB0,W
		BTFSS	STATUS,C
		INCFSZ	BARGB0,W
		SUBWF	REMB0
		CLRW
		 BTFSS	STATUS,C
		 MOVLW	1
		 SUBWF	MATHTEMP
		 GOTO	UOK22LC

UADD22LC
		ADDWF	REMB3
		MOVF	BARGB2,W
		BTFSC	STATUS,C
		INCFSZ	BARGB2,W
		ADDWF	REMB2
		MOVF	BARGB1,W
		BTFSC	STATUS,C
		INCFSZ	BARGB1,W
		ADDWF	REMB1
		MOVF	BARGB0,W
		BTFSC	STATUS,C
		INCFSZ	BARGB0,W
		ADDWF	REMB0
		CLRW
		BTFSC	STATUS,C
		MOVLW	1
		ADDWF	MATHTEMP
        
UOK22LC
		RLF		AARGB2

		DECFSZ	LOOPCOUNT
		GOTO	LOOPU3232C

		RLF		AARGB3,W
		RLF		REMB3
		RLF		REMB2
		RLF		REMB1
		RLF		REMB0
		RLF		MATHTEMP
		MOVF	BARGB3,W
		BTFSS	AARGB2,0
		GOTO	UADD22L24

		SUBWF	REMB3
		MOVF	BARGB2,W
		BTFSS	STATUS,C
		INCFSZ	BARGB2,W
		SUBWF	REMB2
		MOVF	BARGB1,W
		BTFSS	STATUS,C
		INCFSZ	BARGB1,W
		SUBWF	REMB1
		MOVF	BARGB0,W
		BTFSS	STATUS,C
		INCFSZ	BARGB0,W
		SUBWF	REMB0
		CLRW
		
		BTFSS	STATUS,C
		MOVLW	1
		SUBWF	MATHTEMP
		GOTO	UOK22L24

UADD22L24       
		ADDWF	REMB3
		MOVF	BARGB2,W
		BTFSC	STATUS,C
		INCFSZ	BARGB2,W
		ADDWF	REMB2
		MOVF	BARGB1,W
		BTFSC	STATUS,C
		INCFSZ	BARGB1,W
		ADDWF	REMB1
		MOVF	BARGB0,W
		BTFSC	STATUS,C
		INCFSZ	BARGB0,W
		ADDWF	REMB0
		CLRW
		BTFSC	STATUS,C
		MOVLW	1
		ADDWF	MATHTEMP
        
UOK22L24 
		RLF		AARGB3

		MOVLW	7
		MOVWF	LOOPCOUNT

LOOPU3232D 
		RLF		AARGB3,W
		RLF		REMB3
		RLF		REMB2
		RLF		REMB1
		RLF		REMB0
		RLF		MATHTEMP
		MOVF	BARGB3,W
		BTFSS	AARGB3,0
		GOTO	UADD22LD

		SUBWF	REMB3, F
		MOVF	BARGB2,W
		BTFSS	STATUS,C
		INCFSZ	BARGB2,W
		SUBWF	REMB2, F
		MOVF	BARGB1,W
		BTFSS	STATUS,C
		INCFSZ	BARGB1,W
		SUBWF	REMB1, F
		MOVF	BARGB0,W
		BTFSS	STATUS,C
		INCFSZ	BARGB0,W
		SUBWF	REMB0, F
		CLRW
		BTFSS	STATUS,C
		MOVLW	1
		SUBWF	MATHTEMP, F
		GOTO	UOK22LD

UADD22LD
		ADDWF	REMB3, F
		MOVF	BARGB2,W
		BTFSC	STATUS,C
		INCFSZ	BARGB2,W
		ADDWF	REMB2, F
		MOVF	BARGB1,W
		BTFSC	STATUS,C
		INCFSZ	BARGB1,W
		ADDWF	REMB1, F
		MOVF	BARGB0,W
		BTFSC	STATUS,C
		INCFSZ	BARGB0,W
		ADDWF	REMB0, F
		CLRW
		BTFSC	STATUS,C
		MOVLW	1
		ADDWF	MATHTEMP, F
        
UOK22LD
		RLF		AARGB3, F
		DECFSZ	LOOPCOUNT, F
		GOTO	LOOPU3232D

		BTFSC	AARGB3,0
		GOTO	UOK22L
		MOVF	BARGB3,W
		ADDWF	REMB3, F
		MOVF	BARGB2,W
		BTFSC	STATUS,C
		INCFSZ	BARGB2,W
		ADDWF	REMB2, F
		MOVF	BARGB1,W
		BTFSC	STATUS,C
		INCFSZ	BARGB1,W
		ADDWF	REMB1, F
		MOVF	BARGB0,W
		BTFSC	STATUS,C
		INCFSZ	BARGB0,W
		ADDWF	REMB0, F

UOK22L
		return			
;                                           
;***********************************************
;
;		CHK for end of ring buffer
;                                           
;***********************************************
;
CHK4END	
	pagesel		CHK4END
	movf		FSR,W
	sublw		BUFFEREND
	btfss		STATUS,Z						; skip next if end of buffer 
	goto		CHK4END_EXIT
	
	movlw		BUFFERSTART-1
	movwf		FSR	
	
CHK4END_EXIT
	return
;                                           
;***********************************************
;
;		CALIBRATE MESSAGE
;                                           
;***********************************************
;
CALIBRATE 
		pagesel		CALIBRATE
		 	
		banksel		PIE1
		btfsc		PIE1,TXIE
		goto		$-3
		
		banksel		BUFFERHEAD				
		movf		BUFFERHEAD,W
		movwf		FSR
		movlw		A'C'
		movwf		INDF
		
		pagesel		CHK4END			
		call 		CHK4END	
		
		pagesel		CALIBRATE			
		incf		FSR
		movlw		A'A'
		movwf		INDF
		
		pagesel		CHK4END			
		call 		CHK4END		
		
		pagesel		CALIBRATE		
		incf		FSR
		movlw		A'L'
		movwf		INDF
		
		pagesel		CHK4END			
		call 		CHK4END	
		
		pagesel		CALIBRATE			
		incf		FSR
		movlw		A'I'
		movwf		INDF
		
		pagesel		CHK4END			
		call 		CHK4END	
		
		pagesel		CALIBRATE			
		incf		FSR
		movlw		A'B'
		movwf		INDF
		
		pagesel		CHK4END			
		call 		CHK4END	
		
		pagesel		CALIBRATE			
		incf		FSR
		movlw		A'R'
		movwf		INDF
		
		pagesel		CHK4END			
		call 		CHK4END	
		
		pagesel		CALIBRATE			
		incf		FSR
		movlw		A'A'
		movwf		INDF
		
		pagesel		CHK4END			
		call 		CHK4END	
		
		pagesel		CALIBRATE			
		incf		FSR		
		movlw		A'T'
		movwf		INDF
		
		pagesel		CHK4END			
		call 		CHK4END	
		
		pagesel		CALIBRATE			
		incf		FSR
		movlw		A'E'
		movwf		INDF
		
		pagesel		CHK4END			
		call 		CHK4END	
		
		pagesel		CALIBRATE			
		incf		FSR				
		movlw		0x0D
		movwf		INDF
		
		pagesel		CHK4END			
		call 		CHK4END
		
		pagesel		CALIBRATE		
		incf		FSR			
		movf		FSR,W
		banksel		BUFFERHEAD
		movwf		BUFFERHEAD
		banksel		PIE1
		bsf			PIE1,TXIE
		return					
;                                           
;***********************************************
;
;		RESTART MESSAGE
;                                           
;***********************************************
;
RESTART	
		pagesel		RESTART
		banksel		PIE1
		btfsc		PIE1,TXIE
		goto		$-3
		banksel		BUFFERHEAD				
		movf		BUFFERHEAD,W
		movwf		FSR
		movlw		A'R'
		movwf		INDF
		
		pagesel		CHK4END			
		call 		CHK4END
		
		pagesel		RESTART				
		incf		FSR
		movlw		A'E'
		movwf		INDF
		
		pagesel		CHK4END			
		call 		CHK4END
		
		pagesel		RESTART				
		incf		FSR
		movlw		A'S'
		movwf		INDF
		
		pagesel		CHK4END			
		call 		CHK4END
		
		pagesel		RESTART				
		incf		FSR
		movlw		A'T'
		movwf		INDF
		
		pagesel		CHK4END			
		call 		CHK4END
		
		pagesel		RESTART				
		incf		FSR
		movlw		A'A'
		movwf		INDF
		
		pagesel		CHK4END			
		call 		CHK4END
		
		pagesel		RESTART				
		incf		FSR
		movlw		A'R'
		movwf		INDF
		
		pagesel		CHK4END			
		call 		CHK4END
		
		pagesel		RESTART				
		incf		FSR
		movlw		A'T'
		movwf		INDF
		
		pagesel		CHK4END			
		call 		CHK4END
		
		pagesel		RESTART				
		incf		FSR		
		movlw		0x0D
		movwf		INDF
		
		pagesel		CHK4END		
		call 		CHK4END
		
		pagesel		RESTART		
		incf		FSR			
		movf		FSR,W
		movwf		BUFFERHEAD
		banksel		PIE1
		bsf			PIE1,TXIE
		return
;                                           
;***********************************************
;
;		BPM MESSAGE
;                                           
;***********************************************
;
BPM	
		pagesel		BPM	
		banksel		BUFFERHEAD				
		movf		BUFFERHEAD,W
		movwf		FSR
		movlw		A'B'
		movwf		INDF
		
		pagesel		CHK4END		
		call 		CHK4END
		
		pagesel		BPM					
		incf		FSR
		movlw		A'P'
		movwf		INDF
		
		pagesel		CHK4END			
		call 		CHK4END	
		
		pagesel		BPM				
		incf		FSR
		movlw		A'M'
		movwf		INDF
		
		pagesel		CHK4END			
		call 		CHK4END
		
		pagesel		BPM				
		incf		FSR	
		movf		FSR,W
		
		banksel		BUFFERHEAD		
		movwf		BUFFERHEAD

		banksel		REMB0
		clrf		REMB0
		
		banksel		BPML		
		movf		BPML,W
		
		banksel		REMB1		
		movwf		REMB1	
		
		pagesel		SENDDIGITS			
		call		SENDDIGITS
		
		return
;                                           
;***********************************************
;
;		BPM O2 SATURATION
;                                           
;***********************************************
;
SATURATION	
		pagesel		SATURATION
		banksel		IR_RatioH
		movf		IR_RatioH
		btfss		STATUS,Z					; skip next if IR_RatioH = 0
		goto		SATURATION_RED
		
		movf		IR_RatioL
		btfss		STATUS,Z					; skip next if IR_RatioL = 0
		goto		SATURATION_RED
		
		movlw		0x10
		xorwf		MODE
		pagesel		MAIN_ERROR
		goto		MAIN_ERROR		
		
SATURATION_RED
		movf		RED_RatioH
		btfss		STATUS,Z					; skip next if RED_RatioH = 0
		goto		SATURATION_OK
		
		movf		RED_RatioL
		btfss		STATUS,Z					; skip next if RED_RatioL = 0
		goto		SATURATION_OK
		
		movlw		0x10
		xorwf		MODE
		pagesel		MAIN_ERROR
		goto		MAIN_ERROR	
				
SATURATION_OK
		movf		RED_RatioH,W
		
		banksel		AARGB0
		movwf		AARGB0
		
		banksel		RED_RatioL
		movf		RED_RatioL,W
		
		banksel		AARGB1
		movwf		AARGB1
		clrf		AARGB2
		clrf		AARGB3
		
		banksel		IR_RatioH		
		movf		IR_RatioH,W
		
		banksel		BARGB0
		movwf		BARGB2
		
		banksel		IR_RatioL
		movf		IR_RatioL,W
		
		banksel		BARGB1
		movwf		BARGB3
		clrf		BARGB0	
		clrf		BARGB0
		
		
		pagesel		FXD3232U
		call		FXD3232U	

		banksel		AARGB1		
		movf		AARGB1,W
		movwf		AARGB0
		movf		AARGB2,W
		movwf		AARGB1
		movlw		0x66
		movwf		BARGB1
		clrf		BARGB0
		
		pagesel		DIFFERENCE
		call		DIFFERENCE
		
		pagesel		SATURATION_OK
		banksel		AARGB2
		clrf		AARGB2
		clrf		AARGB3
		movlw		0x0A
		movwf		BARGB3
		clrf		BARGB2
		clrf		BARGB1
		clrf		BARGB0
		
		pagesel		FXD3232U
		call		FXD3232U
		
		pagesel		SATURATION_OK
		banksel		AARGB1	
		movf		AARGB1,W
		
		banksel		SAT
		movwf		SAT
		
		banksel		AARGB2
		btfss		AARGB2,7
		goto		SATURATION_OK1
		
		banksel		SAT
		incf		SAT
				
SATURATION_OK1
		banksel		SAT		
		movf		SAT,W
		sublw		D'100'
		movwf		SAT
		btfss		STATUS,C
		goto		SATURATION_EXIT
		movf		SAT,W
		
		banksel		REMB1		
		movwf		REMB1	
		clrf		REMB0
		
		pagesel		SATURATION_OK1
		banksel		BUFFERHEAD				
		movf		BUFFERHEAD,W
		movwf		FSR
		movlw		A'O'
		movwf		INDF
		
		pagesel		CHK4END		
		call 		CHK4END
		
		pagesel		SATURATION_OK1				
		incf		FSR
		movlw		A'2'
		movwf		INDF
		
		pagesel		CHK4END			
		call 		CHK4END	
		
		pagesel		SATURATION_OK1				
		incf		FSR
		movlw		A'%'
		movwf		INDF
		
		pagesel		CHK4END			
		call 		CHK4END
		
		pagesel		SATURATION_OK1			
		incf		FSR	
		movf		FSR,W
		
		banksel		BUFFERHEAD		
		movwf		BUFFERHEAD		
		
		pagesel		SENDDIGITS			
		call		SENDDIGITS

SATURATION_EXIT		
		pagesel		SATURATION_OK1		
		return		
;                                           
;***********************************************
;
;		MEAN MESSAGE
;                                           
;***********************************************
;
MEAN	
		banksel		MaxH
		movf		MaxH,W
		
		banksel		REMB0		
		movwf		REMB0
		
		banksel		MaxL		
		movf		MaxL,W
		
		banksel		REMB1		
		movwf		REMB1	
		
		pagesel		SENDDIGITS			
		call		SENDDIGITS
		
		pagesel		MEAN	
		banksel		MinH		
		movf		MinH,W
		
		banksel		REMB0		
		movwf		REMB0
		
		banksel		MinL		
		movf		MinL,W
		
		banksel		REMB1		
		movwf		REMB1
		
		pagesel		SENDDIGITS				
		call		SENDDIGITS
					
		return
;                                           
;***********************************************
;
;		BPM TABLE
;                                           
;***********************************************
;		
TABLE_BPM
;		DATA																			
;                                           
;***********************************************
;
;		EEPROM
;                                           
;***********************************************
;
	ORG	0x2100									; data EEPROM location
;	DE	1,2,3,4									; define first four EEPROM locations as 1, 2, 3, and 4
;                                           
;***********************************************
;
;		END
;                                           
;***********************************************
;
	END                       					; directive 'end of program'

