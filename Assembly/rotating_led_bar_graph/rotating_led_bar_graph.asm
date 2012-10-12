/*
 * rotating_led_bar_graph.asm
 * 
 * Continuously rotates a 10 segment LED bar graph (https://www.sparkfun.com/products/9937)
 * Pins 2 - 11 are connected to LED bar graph
 *
 * Created: 10/4/2012
 * Last modified: 10/11/2012
 * Author: Eric Ouyang
 */ 

.def tmp = R16 ; used as a general temporary storage

.def outputD = R17 ; used as a temporary storage for PORTD
.def outputB = R18 ; used as a temporary storage for PORTB

; counters used in implementing delay
.def counter1 = R19 
.def counter2 = R20
.def counter3 = R21

ldi tmp, 0b11111100 ; pins 2-7
out DDRD, tmp ; set pins 2-7 as output

ldi tmp, 0b00001111 ; pins 8-11
out DDRB, tmp ; set pins 8-11 as output

loop: ; main loop of program

	ldi outputD, 0b00000100 ; turn LED on pin 2 to HIGH
	out PORTD, outputD ; actually set the appropriate register

	ldi outputB, 0 ; turn off all LEDs on B
	out PORTB, outputB ; actually set the appropriate register
	
	leftD: ; shifts the LEDs on D to the left
		rcall delay ; relative call to delay subroutine

		lsl outputD ; shifts all bits to the left & sets carry (C) flag to bit 7
		out PORTD, outputD ; actually set the appropriate register
		
		brcc leftD ; branch if carry cleared (ie. if carry flag == 0, go back up to leftD, otherwise continue the program)

	ldi outputB, 1 ; turn LED on pin 8 to HIGH
	out PORTB, outputB ; actually set the appropriate register

	leftB: ; shifts the LEDs on B to the left
		rcall delay ; relative call to delay subroutine

		lsl outputB ; shifts all bits to the left & sets carry (C) flag to bit 7
		out PORTB, outputB ; actually set the appropriate register
		
		cpi outputB, 0b00010000 ; compares outputB with 0b00010000 - checks if pin 12 is set as an output (not actually connected to anything -- this means that the code as reached the end of the bar graph)
		brne leftB ; branch if not equal (ie. if outputB does not equal 0b00010000, go back up to leftB, otherwise continue the program)

	rjmp loop ; jump back to loop

delay: ; subroutine to delay for approximately one second
	
	ldi counter1, 0xFF ; set counter1 to 255 - 1 cycle 
	delay1: ; delay loop 1 for counter 1

		ldi counter2, 0xFF ; set counter2 255 - 1 cycle 
		delay2: ; delay loop 2 for counter 2
		 
			ldi counter3, 0x48 ; set counter3 to 72 - 1 cycle 
			delay3: ; delay loop 3 for counter 3

				dec counter3 ; decrease counter3 by 1 - 1 cycle 
				brne delay3 ; branch to delay3 if counter2 != 0 - 1 cycle if false / 2 cycles if true

			dec counter2 ; decrease counter2 by 1 - 1 cycle 
			brne delay2 ; branch to delay2 if counter2 != 0 - 1 cycle if false / 2 cycles if true

		dec counter1 ; decrease counter1 by 1 - 1 cycle 
		brne delay1 ; branch to delay1 if counter1 != 0 - 1 cycle if false / 2 cycles if true
	
	ret ; return to previous location