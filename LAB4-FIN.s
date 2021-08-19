; Archivo: Lab 4
; Dispositivo: PIC16F887
; Autor: José Santizo 
; Compilador: pic-as (v2.32), MPLAB X v5.50
    
; Programa: Interrupciones
; Hardware: LEDs en el puerto A y pushbuttons en el puerto B
    
; Creado: 17 de Agosto, 2021
; Última modificación: 17 de agosto de 2021

PROCESSOR 16F887
#include <xc.inc>

; CONFIG1
  CONFIG  FOSC = INTRC_NOCLKOUT ; Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
  CONFIG  WDTE = OFF            ; Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
  CONFIG  PWRTE = ON            ; Power-up Timer Enable bit (PWRT enabled)
  CONFIG  MCLRE = OFF           ; RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
  CONFIG  CP = OFF              ; Code Protection bit (Program memory code protection is disabled)
  CONFIG  CPD = OFF             ; Data Code Protection bit (Data memory code protection is disabled)
  CONFIG  BOREN = OFF           ; Brown Out Reset Selection bits (BOR disabled)
  CONFIG  IESO = OFF            ; Internal External Switchover bit (Internal/External Switchover mode is disabled)
  CONFIG  FCMEN = OFF           ; Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
  CONFIG  LVP = ON              ; Low Voltage Programming Enable bit (RB3/PGM pin has PGM function, low voltage programming enabled)

; CONFIG2
  CONFIG  BOR4V = BOR40V        ; Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
  CONFIG  WRT = OFF             ; Flash Program Memory Self Write Enable bits (Write protection off)

;---------------Macros-------------------------  
 REINICIAR_TMR0 MACRO
    BANKSEL	PORTD		; Selección del puerto D
    MOVLW	178		; Timer 0 reinicia cada 20 ms
    MOVWF	TMR0		; Mover este valor al timer 0
    BCF		T0IF		; Limpiar la bandera del Timer 0
    ENDM
 
;-----------Valores globales------------------
UP	EQU 0			; Literal up = bit 0
DOWN	EQU 5			; Literal down = bit 5
;-----------Variables a utilizar---------------
PSECT udata_bank0	    ; common memory
    CONT:	    DS 2	    ; 2 byte
    CONT1:	    DS 2	    ; 2 byte
    PORT:	    DS 1	    ; 1 byte
    PORT1:	    DS 1	    ; 1 byte
    PORT2:	    DS 2	    ; 1 byte
    PORT3:	    DS 1	    ; 1 byte
    PORTC1:	    DS 1	    ; 1 byte
    
PSECT udata_shr	    ; common memory
    W_TEMP:	    DS 1	    ; 1 byte
    STATUS_TEMP:    DS 1	    ; 1 byte
    
PSECT resVect, class=CODE, abs, delta=2
 ;------------vector reset-----------------
 ORG 00h			    ; posición 0000h para el reset
 resetVec:  
    PAGESEL MAIN
    goto MAIN

 PSECT intVect, class=CODE, abs, delta=2
 ;------------vector interrupciones-----------------
 ORG 04h			    ; posición 0000h para interrupciones
 
 PUSH:
    MOVWF	W_TEMP
    SWAPF	STATUS, W
    MOVWF	STATUS_TEMP
 
 ISR:
    
    BTFSC	T0IF		    ; Chequear si la bandera T0IF está en 0
    CALL	INT_T0		    ; Subrutina INT_T0
    
    BTFSC	RBIF		    ; Chequear si la bandera RBIF está en 0
    CALL	INT_IOCB	    ; Subrutina INT_IOCB

 POP:
    SWAPF	STATUS_TEMP, W
    MOVWF	STATUS
    SWAPF	W_TEMP, F
    SWAPF	W_TEMP, W
    RETFIE
   
 ;------------Sub rutinas de interrupción--------------
 INT_IOCB:
    BANKSEL	PORTA		    ; Selección del puerto A
    BTFSS	PORTB, UP	    ; Chequear si el pin 0 del puerto B está en 0
    INCF	PORT		    ; Incrementar contador PORT
    BTFSS	PORTB, DOWN	    ; Chequear si el pin 5 del puerto B está en 0
    DECF	PORT		    ; Decrementar el contador PORT
    MOVWF	PORT, W		    ; Mover el valor del PORT a w
    ANDLW	00001111B	    ; Realizar una máscara por medio de un And con el valor binario 00001111
    MOVWF	PORTA		    ; Mover el valor resultante de la máscara a PORTA
    BCF		RBIF		    ; Limpiar la bandera RBIF
    RETURN
    
 INT_T0:
    REINICIAR_TMR0		    ; Reiniciar el Timer 0
    INCF	CONT		    ; Incrementar la variable CONT
    MOVF	CONT, W		    ; Mover el valor de CONT a W
    SUBLW	50		    ; Multiplicar el valor de 20 ms por 50
    BTFSS	ZERO		    ; Chequear si el valor es 0
    GOTO	RETURN_T0	    
    CLRF	CONT		    ; Limpiar el CONT	
    INCF	PORT1		    ; Incrementar el contador en variable PORT1
    MOVWF	PORT1, W	    ; Mover el valor de PORT1 a W
    CALL	TABLA		    ; Traducir el valor por medio de la Tabla
    MOVWF	PORTD		    ; Mover el valor traducido a PORTD
    
    MOVF	PORT1, W	    ; Mover el valor de PORT1 a w
    SUBLW	10		    ; Realizar la resta 10 - valor en PORT1
    BTFSC	STATUS, 2	    ; Chequear si el bit 2 del registro status es 1
    CALL	INCREMENTO	    ; Llamar a la rutina incremento
    
    MOVWF	PORT3, W	    ; Mover el valor de PORT3 a w
    CALL	TABLA		    ; Traducir el valor de PORT3 por una tabla
    MOVWF	PORTC		    ; Mover el valor traducido de la tabla al puerto C
    RETURN
 
 INCREMENTO:
    INCF	PORT3		    ; Incrementar en 1 el valor del PORT3
    CLRF	PORT1		    ; Colocar en 0 el valor del contador PORT1
    MOVWF	PORT1,W		    ; Mover el valor de PORT1 a w
    CALL	TABLA		    ; Traducir el valor de PORT1 por una tabla
    MOVWF	PORTD		    ; Mover el valor traducido de la tabla al puerto D
    
    MOVF	PORT3, W	    ; Mover el valor de PORT3 a w
    SUBLW	6		    ; Restar 6 - valor en PORT3
    BTFSC	STATUS, 2	    ; Chequear si el bit 2 del registro status es 1
    CLRF	PORT3		    ; Limpiar el contador PORT3 para reiniciar el mismo
    RETURN
    
 RETURN_T0:   
    RETURN
 
    
 PSECT CODE, DELTA=2, ABS
 ORG 100H		    ;Posición para el codigo
 ;------------Tablas------------------------
 
 TABLA:
    CLRF	PCLATH
    BSF		PCLATH, 0   ;PCLATH = 01    PCL = 02
    ANDLW	0x0f
    ADDWF	PCL	    ;PC = PCLATH + PCL + W
    RETLW	00111111B   ;0
    RETLW	00000110B   ;1
    RETLW	01011011B   ;2
    RETLW	01001111B   ;3
    RETLW	01100110B   ;4
    RETLW	01101101B   ;5
    RETLW	01111101B   ;6
    RETLW	00000111B   ;7
    RETLW	01111111B   ;8
    RETLW	01101111B   ;9
    RETLW	01110111B   ;A
    RETLW	01111100B   ;B
    RETLW	00111001B   ;C
    RETLW	01011110B   ;D
    RETLW	01111001B   ;E
    RETLW	01110001B   ;F
 ;-----------Configuración----------------
 MAIN:
    CALL	CONFIG_IO	    ;Configuraciones de entradas y salidas
    CALL	CONFIG_RELOJ	    ;Configuración del oscilador
    CALL	CONFIG_TMR0	    ;Configuración del Timer 0
    CALL	CONFIG_INT_ENABLE   ;Configuración de interrupciones	
    CALL	CONFIG_IOCB	    ;Configuración de resistencias pull up internas en puerto B
    
 LOOP:
    
    GOTO	LOOP
 
 ;-----------SUBRUTINAS------------------
    
 CONFIG_IOCB:
    BANKSEL	TRISA		    ;Selección del banco trisA
    BSF		IOCB, UP	    ;Limpiar las resistencias pull up en puerto 0 y 5
    BSF		IOCB, DOWN
    
    BANKSEL	PORTA		    ;SELECCIONAR PUERTO A
    MOVF	PORTB, W	    ;TERMINA CONDICIÓN MISMATCH AL LEER
    BCF		RBIF
    RETURN
    
 CONFIG_TMR0:
    BANKSEL	TRISD
    BCF		T0CS		    ;Reloj interno
    BCF		PSA		    ;PRESCALER
    BSF		PS2 
    BSF		PS1
    BSF		PS0		    ;Prescaler = 110 = 256
    REINICIAR_TMR0
    RETURN
    
 CONFIG_INT_ENABLE:
    BSF		GIE		    ;Configuración de las interrupciones
    BSF		T0IE
    BSF		RBIE
    BCF		T0IF
    BCF		RBIF
    RETURN
 
 CONFIG_RELOJ:
    BANKSEL	OSCCON
    BSF		IRCF2		    ;IRCF = 110 = 4 MHz
    BSF		IRCF1
    BCF		IRCF0
    BSF		SCS		    ;Reloj interno
    RETURN
 
 CONFIG_IO:
    BSF		STATUS, 5	    ;BANCO 11
    BSF		STATUS, 6
    CLRF	ANSEL		    ;PINES DIGITALES
    CLRF	ANSELH
    
    BSF		STATUS, 5	    ;BANCO 01
    BCF		STATUS, 6
    CLRF	TRISA		    ;PORT A COMO SALIDA
    CLRF	TRISC		    ;PORT C COMO SALIDA
    CLRF	TRISD		    ;PORT D COMO SALIDA
    
    BSF		TRISB, UP	    ;PINES 0 Y 7 DE PORTB COMO ENTRADA
    BSF		TRISB, DOWN
    
    BCF		OPTION_REG, 7	    ;HABILITAR PULL UPS
    BSF		WPUB, UP
    BSF		WPUB, DOWN
    
    BCF		STATUS, 5	    ;BANCO 00
    BCF		STATUS, 6
    CLRF	PORTA
    CLRF	PORTC
    CLRF	PORTD
    RETURN

    
END
    


