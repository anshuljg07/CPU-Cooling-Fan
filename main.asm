//code by Anshul Gowda and Rafael Rangel

.cseg

//interupts
.org 0x0000	
jmp reset ;skip interrupts

.org 0x000A ;PCINT2  
	jmp RPGInterrupt
	

.org 0x0006 ;PCINT0 pushbutton
	jmp buttonInterrupt

; end interrupts
.org INT_VECTORS_SIZE

reset:

	ldi tmp2, 0b00000101
	sts PCICR, tmp2

	ldi tmp2, 0b00110000; enable the PCINT23 to do pin change interrupt requests
	sts PCMSK2, tmp2

	ldi tmp2, 0b00000001; enable the PCINT23 to do pin change interrupt requests
	sts PCMSK0, tmp2


;set up input and output pins
;__________________________________________________________________________________

;setting B ports to output, not C ports
sbi DDRB,3 ; output the RS signal for the LCD
sbi DDRB,5 ; output the enable signal for the LCD

sbi DDRD, 3 ;pwm

cbi DDRD, 2

;configure C ports to output
sbi DDRC, 0
sbi DDRC, 1
sbi DDRC, 2
sbi DDRC, 3


cbi DDRB, 0
 
 ;Register aliases     
;__________________________________________________________________________________
.def    character           = r16	; this will hold the characters soon to be displayed on the LCD
.def    tmp1                = r18	; temporary registers. Should be push/pop if you are going to use in routine
.def    tmp2                = r19
.def	tmp3                = r25
.def	divider				= r26
.def	dutyCycle			= r27
.def    loopingVar          = r23
.def    commandBitsForLCD   = r17	; contains commnad bits to write to LCD
.def    result			    = r31
.def    remainder           = r30 
.def    currentCycle        = r24
.def    rotaryBits          = r22
.def    previousRotaryBits  = r21
.def	fanStatus		= r29
.def	previousButtonBits = r28
.def	PBSstatus = r20  


;init register values
ldi currentCycle, 150; divide 2, store 
ldi dutyCycle, 25; (inverted bullshit ~ stands for a DC of 75)
ldi previousRotaryBits, 0
ldi PBSstatus, 0
ldi fanStatus, 0xFF

;enable interrupts
sei

;initialize timer
ldi tmp2, 2
out TCCR0B, tmp2  ; stop timer 0

ldi tmp2, 0
out TCCR0A, tmp2

start:
	rcall PWMInit;initialize the PWM for the fan

	rcall LCDInitialization ; set up the LCD to start outputing

	rcall setLCDtoCommandMode
	
	ldi commandBitsForLCD, 0b00000010; shift the cursor back to home in prep for "DC = "  
	rcall writeToLCD

	rcall displayToLCD ;display initial values to the LCD

;main loop to keep program running
main:
	rjmp main

;interrupts(RPG)
;____________________________________________________________________________________
RPGInterrupt:
	cli
	in rotaryBits, PIND
	andi rotaryBits, 0b00110000
	

	lsr rotaryBits
	lsr rotaryBits
	lsr rotaryBits
	lsr rotaryBits

	lsl previousRotaryBits
	lsl previousRotaryBits
	or previousRotaryBits, rotaryBits 
	
	cpi previousRotaryBits, 0b01001011
	breq ccwTurn
	cpi previousRotaryBits, 0b10000111
	breq cwTurn

	reti 
cwTurn:
	cpi dutyCycle, 0
	breq rollover
	//mess with duty cycle
	//CW rotation: increase
	rcall displayToLCD
	ldi previousRotaryBits, 0b11111111
	cpi currentCycle, 200
	inc currentCycle
	inc currentCycle
	dec dutyCycle
	
	sts OCR2B, currentCycle
	reti
	
rollover:
	reti

ccwTurn:
	//mess with duty cycle
	//CCW rotation: decrease
	cpi dutyCycle, 100
	breq rollover

	rcall displayToLCD

	ldi previousRotaryBits, 0b11111111
	
	dec currentCycle
	dec currentCycle
	inc dutyCycle
	
	sts OCR2B, currentCycle
	
	reti

	
;interrupts(Button)
;____________________________________________________________________________________
buttonInterrupt:
	cli
	
	in PBSstatus, PINB
	ldi loopingVar,200
	rcall delayNms
	andi PBSstatus, 1
	

	lsl previousButtonBits
	or previousButtonBits, PBSstatus
	andi previousButtonBits, 3
	cpi previousButtonBits, 1
	breq rising

	reti 
	; Code here will execute when rising edge is detected

rising: 
	cpi fanStatus, 0xFF
	breq off
	cpi fanStatus, 0x00
	breq on

off:
	ldi fanStatus,0x00
	ldi tmp1,0x00;turn fan off
	sts TCCR2A,tmp1
	rcall displayToLCD

	jmp done
on:
	ldi fanStatus,0xFF
	ldi tmp1, 0b00100011; turn fan on
	sts TCCR2A, tmp1



	rcall displayToLCD
	


done:
	reti

	;display method 
;_________________________________________________________________________________________________
displayToLCD:
	
	ldi commandBitsForLCD, 0b00000001 ; clear display
	rcall writeToLCD

	ldi commandBitsForLCD, 0b00000010; shift the cursor back to home in prep for "DC = "  
	rcall writeToLCD

	ldi ZL, LOW(2*dc_str) ; Load Z register low of "DC ="
	ldi ZH, HIGH(2*dc_str) ; Load Z register high of "DC ="
	rcall displayString

	ldi commandBitsForLCD, 0b10000101; set the position on the LCD by sending the address for the cursor 
	rcall writeToLCD


	ldi remainder, 0x00
	ldi divider, 0x00
	rcall divideDutyCycle

	mov character, result ;display the tenths part of the duty cycle 
	rcall displayCharacter

	mov character, remainder;display the ones part of the duty cycle 
	rcall displayCharacter

	ldi character, 0x25
	rcall displayCharacter

	ldi commandBitsForLCD, 0b11000000; set the position on the LCD by sending the address for the cursor 
	rcall writeToLCD

	ldi ZL, LOW(2*fan_str) ; Load Z register low of "DC ="
	ldi ZH, HIGH(2*fan_str) ; Load Z register high of "DC ="
	rcall displayString

	ldi commandBitsForLCD, 0b11000101; set the position on the LCD by sending the address for the cursor 
	rcall writeToLCD

	cpi fanStatus, 0xFF
	breq displayON

	cpi fanStatus, 0x00
	breq displayOFF

displayON:
	ldi ZL, LOW(2*fanOn) 
	ldi ZH, HIGH(2*fanOn) 
	rjmp displayString

displayOFF:
	ldi ZL, LOW(2*fanOff) 
	ldi ZH, HIGH(2*fanOff) 
	rjmp displayString
	
	reti


divideDutyCycle:
	ldi divider, 10
	mov tmp1, dutyCycle ; the duty cycle stored in tmp1
	push tmp1 ; duty cycle saved

	divInit:
		sub remainder, remainder
		ldi tmp2, 9; tmp2 serves as counter for algo

	divA:
		rol result
		rol dutyCycle
		dec tmp2
		brne divB

		ldi tmp1, 0x30; hex converter
		add remainder, tmp1; ASCII one's place
		add result, tmp1 ; ASCII ten's place

		pop tmp1
		mov dutyCycle, tmp1

		ret

	divB:
		rol remainder
		sub remainder, divider
		brcc divC
		add remainder, divider
		clc
		rjmp divA
	
	divC:
		sec
		rjmp divA

	
;_________________________________________________________________________________________________


;init LCD
;_____________________________________________________________________________________
LCDInitialization: 

	ldi loopingVar, 100
	rcall delayNms

	;set to command mode 
	rcall setLCDtoCommandMode

	
	ldi commandBitsForLCD, 0b00110011 ; set function 8 bit, 1 line
	rcall writeToLCD

	ldi commandBitsForLCD, 0b00110010 ; set function 8 bit, 1 line
	rcall writeToLCD
	

	ldi commandBitsForLCD, 0b00101000 ; set function 4 bit, 2 line
	rcall writeToLCD

	ldi commandBitsForLCD, 0b00000001 ; clear display
	rcall writeToLCD

	ldi commandBitsForLCD, 0b00001100 ; Display on/off display on, no underline, no blink 
	rcall writeToLCD


	ldi commandBitsForLCD, 0b00000110 ; Character entry mode, increment, and no display shift  
	rcall writeToLCD

	ret
;LCD related functions
;______________________________________________________________________________________
displayTestMessage:
	rcall setLCDtoCommandMode; ensure that it is in command mode
	
	ldi commandBitsForLCD, 0b00000001 ; clear display
	rcall writeToLCD

	rcall displayString

	ret
setLCDtoCommandMode:
	cbi PORTB,5
	ret
setLCDtoCharacterMode:
	sbi PORTB,5
	ret
writeToLCD:
   rcall setLCDtoCommandMode
   swap commandBitsForLCD             
   out PORTC, commandBitsForLCD
   rcall LCDStrobe
   ldi loopingVar, 50; 5 ms delay
   rcall delayNms
   swap commandBitsForLCD             
   out PORTC, commandBitsForLCD
   rcall LCDStrobe
   ldi loopingVar, 50; 5 ms delay
   rcall delayNms
   
   ret
LCDStrobe: 
	sbi PORTB, 3
	rcall testdelay_100u
	cbi PORTB, 3 
	ret
displayString:
	rcall setLCDtoCharacterMode; set LCD to receive character data

	lpm character, Z+ ; load the nth char then inc to the next n+1 char
	tst character ; check the SREG to see if flag raised
	breq endDisplay
	rcall displayCharacter
	rjmp displayString ;loop until all chars displayed

	ret  
endDisplay:
	ret

;------------------------------------------------------------------------------------
; send the nth character as data to the LCD display 
;------------------------------------------------------------------------------------
displayCharacter:
	rcall setLCDtoCharacterMode
	swap character
	out PORTC, character ; output the character to the LCD
	rcall LCDStrobe
	rcall testdelay_100u; delay 100 microseconds
	
	swap character
	out PORTC, character ; output the character to the LCD
	rcall LCDStrobe
	rcall testdelay_100u; delay 100 microseconds
	
	
	ret
;initialize PWN
;______________________________________________________________________________________
PWMInit:
	
	ldi tmp1, 0b00100011
	sts TCCR2A, tmp1

	ldi tmp1, 0b00001001 ; set the timer control register B with WGM02 to 1 and the last 3 bits to 001 for preescalar of 1
	sts TCCR2B, tmp1

	ldi tmp1, 199
	sts OCR2A, tmp1

	sts OCR2B, currentCycle
	
	ret



;Delay related functions
;_____________________________________________________________________________________
delayNms:
	rcall testdelay_100u
	dec loopingVar
	brne delayNms
	ret 

testdelay_100u: ; (WRONG COMMENT) 5 ms timer
   rcall testset_100u
   rjmp loopTimer

loopTimer:
   in tmp2, TIFR0
   sbrs tmp2, TOV0   ; Wait until timer done
   rjmp loopTimer
   ret

testset_100u:          ; 100us timer

   in tmp2, TIFR0
   sbr tmp2, 1<<TOV0 ; TOV0 is # of bit its stored at. So we shift 1 that many bytes
   out TIFR0, tmp2   ; clear overflow flag

   ldi tmp2, 50      ; 256 - (100E-6/(6.25E-8*8))
   ldi tmp1, 0b010   ; load configuration
   out TCNT0, tmp2   ; load to 5ms start point
   out TCCR0B, tmp1  ; Load config (starts timer)

   ret


; Display strings
;______________________________________________________________________________________________
dc_str:           .db "DC: ",0
fan_str:          .db "Fan: ",0
fanRPM_ok:        .db "RPM OK ",0
fanRPM_low:       .db "low RPM",0
fanStopped:       .db "stopped",0
fanOff:           .db "OFF  ", 0
fanON:			.db "ON " ,0


