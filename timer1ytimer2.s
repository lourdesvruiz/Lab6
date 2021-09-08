 ; Archivo:	timer1ytimer2.s
 ; Dispositivo:	PIC16F887
 ; Autor:	Lourdes Ruiz
 ; Compilador:	pic-as (v2.32), MPLABX V5.50
 ;                
 ; Programa:    LED se enciende de manera intermitente, 500ms encendida y 500ms	
 ;              apagada, display muestra el valor de la variable "segundos".
 ; Hardware:	LED en el puerto C, "segundos" en puerto A, display en puerto D
 ;              y transistores en Puerto B. 
 ;                       
 ; Creado: 29 ago, 2021
 ; Última modificación: 7 sept, 2021
 
 PROCESSOR 16F887
 #include <xc.inc>
 
 ;configuration word 1
  CONFIG FOSC=INTRC_NOCLKOUT	// Oscillador Interno sin salidas, XT
  CONFIG WDTE=OFF   // WDT disabled (reinicio repetitivo del pic)
  CONFIG PWRTE=OFF   // PWRT enabled  (espera de 72ms al iniciar)
  CONFIG MCLRE=OFF  // El pin de MCLR se utiliza como I/O
  CONFIG CP=OFF	    // Sin protección de código
  CONFIG CPD=OFF    // Sin protección de datos
  
  CONFIG BOREN=OFF  // Sin reinicio cuándo el voltaje de alimentación baja de 4V
  CONFIG IESO=OFF   // Reinicio sin cambio de reloj de interno a externo
  CONFIG FCMEN=OFF  // Cambio de reloj externo a interno en caso de fallo
  CONFIG LVP=OFF        // programación en bajo voltaje permitida
 
;configuration word 2
  CONFIG WRT=OFF    // Protección de autoescritura por el programa desactivada
  CONFIG BOR4V=BOR40V // Reinicio abajo de 4V, (BOR21V=2.1V)
  
;------------------macros------------------- 
reinicio_timer1   macro  ;macro para reiniciar el contador del timer1
   banksel PORTA
   movlw   0xB
   movwf  TMR1H    ;valores iniciales para 0.5s
   movlw   0x47
   movwf  TMR1L
   bcf    TMR1IF 
   endm 

reinicio_timer2  macro  
  movlw     246    ;Valor para obtener un reinicio cada 500ms 
  movwf     PR2    ;Valor inicial para el registro de comparación
  bcf       TMR2IF ;Se limpia la bandera del timer para no sobre poner valores 
  endm
   
;--------calculos de temporizador--------
;temporizador = 4*TOSC*TMR0*Prescaler 
;TOSC = 1/FOSC 
;TMR0 = 256 - N (el cual indica el valor a cargar en TMR0)
;¿valor necesario para 0.005s? 
;(4*(1/500kHz))*TMR0*256 = 0.002s
;TMR0 = 1
;256-1 =  / N=255
reinicio_timer0   macro  ;macro para reiniciar el contador del timer0
    banksel  PORTA 
    movlw    254
    movwf    TMR0
    bcf      T0IF    ;se apaga la bandera luego del reinicio
    endm   
    
wdivl   macro   divisor     ;macro para división 
   movwf       var_dos
   clrf        var_dos+1
   
   incf        var_dos+1    ;revisar cuantas veces se ha restado 
   movlw       divisor      ;se quiere restar el "divisor"
   
   subwf       var_dos, f   ;se le resta el divisor 
   btfsc       CARRY        ;revisar si hubo acarreo. si si, ya se paso la resta (ej. 9/10)
   goto        $-4          ;si no, se vuelve a repetir la resta
                            ;
   decf        var_dos+1, W ;guardar los resultados en W
   movwf       cociente     ;resultado de la división
   
   movlw       divisor
   addwf       var_dos, W   ;debido a que quedo "negativo" se le suma el divisor 
   movwf       residuo      ;el resultado sería el residuo  
   endm
    
;---------------variables--------------------    
 PSECT udata_bank0 ;common memory
    contador:    DS  1
    segundos:    DS  1
    bandera:     DS  1
    flag:        DS  1 
    display_var: DS  2
    nibble:      DS  2
    var:         DS  1
    decena:      DS  1
    unidad:      DS  1 
    var_dos:     DS  1
    cociente:    DS  1
    residuo:     DS  1
    banderatmr:  DS  1
 PSECT udata_shr ;common memory
    W_TEMP:	 DS  1 ;1 byte
    STATUS_TEMP: DS  1 ;1 byte
        
 PSECT resVect, class=CODE, abs, delta=2
 ;--------------vector reset------------------
 ORG 00h	;posición 0000h para el reset
 resetVec:
     PAGESEL main
     goto main
 
 PSECT intVect, class=CODE, abs, delta=2
 ;--------------interrupt vector------------------
 ORG 04h	;posición 0004h para las interrupciones
 push:
    movwf   W_TEMP
    swapf   STATUS, W
    movwf   STATUS_TEMP
    
 isr:
    btfsc    T0IF 
    call     interrupt_tmr0
    
    btfsc    TMR1IF          ;si la bandera esta prendida entra a la siguiente instruccion
    call     interrupt_tmr1
    
    btfsc    TMR2IF          ;si la bandera esta prendida entra a la siguiente instruccion
    call     interrupt_tmr2
      
 pop:
    swapf   STATUS_TEMP, W
    movwf   STATUS
    swapf   W_TEMP, F
    swapf   W_TEMP, W
    retfie
 
;-------------subrutinas de interrupcion-----
interrupt_tmr0:                ;subrutina para la interrupción en el contador del timer0
    reinicio_timer0    
    clrf     PORTB 
    btfsc    flag, 0
    goto     display1      ;si "banderas" está en 1 se va a display1
    ;goto     display0     ;si está en 2 se va directo a display0
    
 display0:
    movf    display_var, W
    movwf   PORTD
    bsf     PORTB, 0
    goto    siguiente_display

display1:
    movf    display_var+1, W
    movwf   PORTD    
    bsf     PORTB, 1
    goto    siguiente_display
    
 ;seleccionar el siguiente display
 siguiente_display:
    movlw   1
    xorwf   flag, F   ;se niega el estado para que pase al siguiente display    
    return
    
interrupt_tmr1:
    reinicio_timer1
    incf    contador  
    movf    contador, W    ;variable incrementa cada 500ms
    sublw   2
    btfsc   ZERO 
    call    segundo
    return
    
interrupt_tmr2:
    bcf      TMR2IF 
    btfsc    bandera, 0
    goto     set1
    goto     clear1
       
set1:  
    bsf      banderatmr,0
    bsf      PORTC, 0           ;se prende el LED
    goto     ledchange

clear1:
    bcf      banderatmr, 0
    bcf      PORTC, 0           ;se apaga el LED
    clrf     banderatmr 
    goto     ledchange 
    
ledchange:
    movlw    1
    xorwf    bandera, F    ;constantemente alternando 
    
    return
    
 PSECT code, delta=2, abs
 ORG 100h	; posición para el código
 
 ;configuración de tablas de 7 segmentos
 seg7_tabla:
    clrf   PCLATH
    bsf    PCLATH, 0   ; PCLATH = 01 PCL = 02
    andlw  0x0f        ; limitar a numero "f", me va a poner en 0 todo lo superior y lo inferior, lo deja pasar (cualquier numero < 16)
    addwf  PCL         ; PC = PCLATH + PCL + W (PCL apunta a linea 103) (PC apunta a la siguiente linea + el valor que se sumo)
    retlw  00111111B   ;return que tambien me devuelve una literal (cuando esta en 0, me debe de devolver ese valor)
    retlw  00000110B   ;1
    retlw  01011011B   ;2
    retlw  01001111B   ;3
    retlw  01100110B   ;4
    retlw  01101101B   ;5
    retlw  01111101B   ;6
    retlw  00000111B   ;7
    retlw  01111111B   ;8
    retlw  01101111B   ;9
    retlw  01110111B   ;A
    retlw  01111100B   ;B
    retlw  00111001B   ;C
    retlw  01011110B   ;D
    retlw  01111001B   ;E
    retlw  01110001B   ;F
 
;-------------configuración------------------
 main:
    call    config_io
    call    config_clock
    call    config_timer0
    call    config_timer1
    call    config_timer2
    call    config_int_enable
    banksel PORTA
    
;------------loop principal---------          
 loop:
     
    movf    segundos, W 
    call    tenths
    call    preparar_displays 
    goto    loop        ; loop forever

 ;------------sub rutinas------------
    
preparar_displays:
    movf    unidad, W
    call    seg7_tabla
    movwf   display_var 
    movf    decena, W
    call    seg7_tabla
    movwf   display_var+1
 
    return
     
config_clock:
    banksel OSCCON 
    bcf     IRCF2   ;IRCF = 011 500 kHz 
    bsf     IRCF1
    bsf     IRCF0
    bsf     SCS     ;configurar reloj interno
    return

config_timer0:
    banksel TRISA 
    ;configurar OPTION_REG
    bcf     T0CS   ;reloj interno (utlizar ciclo de reloj)
    bcf     PSA    ;asignar el Prescaler a TMR0
    bsf     PS2
    bsf     PS1 
    bsf     PS0    ;PS = 111 (1:256)
    reinicio_timer0
    return
    
config_timer1:
    banksel PORTA
    bcf    TMR1GE     ;Timer1 siempre cuenta
    bcf    T1CKPS1   
    bcf    T1CKPS0    ;PS de 1:1 (00)
    bcf    T1OSCEN    ;oscilador de bajo voltaje apagado
    bcf    TMR1CS     ;reloj interno
    bsf    TMR1ON     ;prender tmr1
    reinicio_timer1
    return
    
config_timer2:
    banksel   PORTA
    bsf       TOUTPS3
    bsf       TOUTPS2
    bsf       TOUTPS1
    bsf       TOUTPS0    ;postcaler en 1:16 (1111)
    
    bsf       TMR2ON     ;prender el TMR2
    bsf       T2CKPS1
    bsf       T2CKPS0    ;prescaler en 16  (11)  
    banksel   TRISA
    reinicio_timer2
    return
    
config_io:
    banksel ANSEL   ;nos lleva a banco 3 (11)
    clrf    ANSEL   ;configuración de pines digitales 
    clrf    ANSELH
    
    banksel TRISA    ;nos lleva a banco 1 (01)
    clrf    TRISA    ;salida para LEDs (contador)
    clrf    TRISC
    clrf    TRISB
    clrf    TRISD
    
    ;---------------------valores iniciales en banco 00--------------------------
    banksel PORTA   ;nos lleva a banco 0 (00)
    clrf    PORTA 
    clrf    PORTC 
    clrf    PORTB
    clrf    PORTD
    return 
    
config_int_enable: 
    banksel TRISA
    bsf     TMR1IE   ;interrupcion de TIMER1
    bsf     TMR2IE   ;interrupcion de TIMER2
    banksel PORTA 
    bsf     T0IE     ;interrupcion TMR0
    bcf     T0IF     ;bandera TMR0
    bcf     TMR1IF   ;bandera TMR1
    bcf     TMR2IF   ;bandera TMR2
    bsf     PEIE     ;interrupciones perifericas 
    bsf     GIE      ;Intcon (interrupciones globales habilitadas)
    
    return 
                  
 segundo:
    clrf  contador 
    incf  segundos
    movf  segundos, W           ;contador de 1 segundo
    sublw 100                   ;limita a que solo cuenta hasta 99
    btfsc ZERO
    clrf  segundos
    movf  segundos, W
    movwf PORTA                 ;se muestra el contador en Puerto A
    return

tenths:                         ;obtener decenas y unidades (decimal) 
   wdivl  10
   movf   cociente, W 
   movwf  decena  
   movf   residuo, W           ;el residuo se guarda en la variable "unidad" (no hay necesidad de dividir dentro de 1)
   
units:
   movwf  unidad 
   return    
   

    
END 
  


