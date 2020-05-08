/*
 * ECE 3613 Lab 10 Activity 5 (debounced).c
 *
 * Created: 5/6/2020 10:57:28 PM
 * Author : Leomar Duran <https://github.com/lduran2>
 * Board  : ATmega324PB Xplained Pro - 2505
 * For    : ECE 3612, Spring 2020
 * Self   : <https://github.com/lduran2/ece3612-lab10/blob/master/activity5-debounced.c>
 *
 * Increments and decrements the count displayed on PORTA LEDs every
 * time that the sensors on PB0 and PB1 respectively go low.  And sets
 * PA7 whenever the counter goes higher than 10, or clears when it is
 * at most 10.
 */
#include <avr/io.h>
#include <avr/interrupt.h>
/* CPU at 16 [MHz] */
#define	F_CPU	((unsigned long)(16e+6))

/* mask to clear clock select */
#define	CLEAR_CS	(0b11111000)

/* clock select modes for timers */
typedef enum {
	CS_STOP,
	CS_NO_PRESCALING,
	CS_PRESCALING_8,
	CS_PRESCALING_64,
	CS_PRESCALING_256,
	CS_PRESCALING_1024,
	CS_EXTERNAL_SOURCE_FALLING,
	CS_EXTERNAL_SOURCE_RISING
} CLOCK_SELECT_T;

/* interrupt sense control modes for external interrupt requests */
typedef enum {
	ISC_LOW,
	ISC_CHANGE,
	ISC_NEGEDGE,
	ISC_POSEDGE
} INTERRUPT_SENSE_CONTROL_t;

/* the maximum amount that makes PORTA not a large number, */
/* thereby clearing PALF */
#define	PORTA_LARGE	(10)
/* mask for flag for PORTA is large */
#define	PALF_MASK	(1 << 07)

/* the sensors at PORTB */
#define	SENSOR_MASK	0b00000011
/* the offset of INT0 in PORTD */
#define PD_INT0	(2)
/* the offset of INT1 in PORTD */
#define PD_INT1	(3)
/* the interrupts in PORTD */
#define	PD_INT_MASK	((1 << PD_INT0) | (1 << PD_INT1))

/* number of ticks for [(1/4) s] of debouncing */
#define	COUNT_DEBOUNCE	(62500)
/* the respective pr	escaling */
#define	PRESCALING_DEBOUNCE	(CS_PRESCALING_64)

/* debounce states:
 *   standby	= waiting for interrupt or timer (depends on EIMSK)
 *                e.g. if ((int0_debounce == DBNC_STANDBY) && (EIMSK & (1 << INT0)))
 *                     then waiting on interrupt
 *                e.g. if ((int0_debounce == DBNC_STANDBY) && (!(EIMSK & (1 << INT0))))
 *                     then waiting on timer
 *   timing 	= ready to start counting on the timer
 *   success	= successfully read a low signal
 *   failure	= the low signal was just a fluke
 */
volatile enum { DBNC_STANDBY, DBNC_TIMING, DBNC_SUCCESS, DBNC_FAILURE }
	int0_debounce = DBNC_STANDBY,
	int1_debounce = DBNC_STANDBY;

int main(void)
{
	/* set up PORTA */
	PORTA = 0x00;	/* clear PORTA */
	DDRA = 0xFF;	/* set PORTA to all outputs */

	/* set up PORTB */
	PORTB |= SENSOR_MASK;	/* activate the pull switches */
	DDRB &= (~SENSOR_MASK);	/* set PORTB to inputs on sensors */

	/* set up PORTD */
	PORTD |= PD_INT_MASK;	/* activate the pull switches */
	DDRD &= PD_INT_MASK;	/* set PORTD to inputs on interrupts */

	cli();	/* stop listening to interrupts (CLear Interrupt flag) */

	/* set up the external interrupts */
	EIMSK |= ((1 << INT0) | (1 << INT1));	/* set the mask for INT0, INT1 */
	/* set sensor control for low signal at INT0, INT1 */
	/* in External Interrupt Control Register */
	EICRA |= ((ISC_LOW << ISC00) | (ISC_LOW << ISC10));

	/* set up the pin change mask */
	PCICR |= (1 << PCIE0);	/* set interrupt control for PCMSK0 */
	PCMSK0 |= (1 << PCINT0);	/* set the mask for PCINT0 */

	/* set up the timers */
	/* timer1 is used to debounce INT0 */
	TCNT1 = 0;	/* reset the timer */
	OCR1A = COUNT_DEBOUNCE;	/* set output compare A for debounce */
	TIMSK1 |= (1 << OCIE1A);	/* timer1 interrupt on match to compare A */
	/* timer3 is used to debounce INT1 */
	TCNT3 = 0;	/* reset the timer */
	OCR3A = COUNT_DEBOUNCE;	/* set output compare A for debounce */
	TIMSK3 |= (1 << OCIE3A);	/* timer3 interrupt on match to compare A */
	
	/* keep the timers stopped, but set the wave generation mode */
	/* timer1 */
	TCCR1B = (CS_STOP | 0b1000);
	TCCR1A = 0;
	/* timer3 */
	TCCR3B = (CS_STOP | 0b1000);
	TCCR3A = 0;

	sei();	/* start listening to interrupts (SEt Interrupt flag) */

	while (1)
	{
		/* check the int0 debounce */
		switch (int0_debounce) {
			/* if the debounce state is timing */
			case DBNC_TIMING:
				TCCR1B &= CLEAR_CS;	/* clear the clock select */
				TCCR1B |= PRESCALING_DEBOUNCE;	/* start the clock with prescaling */
				int0_debounce = DBNC_STANDBY;	/* signal standby for timer */
			break; /* case DBNC_TIMING */
			/* if the debounce succeeded */
			case DBNC_SUCCESS:
				++PORTA;	/* increment PORTA */
			case DBNC_FAILURE:	/* either way, at the end of debouncing */
				int0_debounce = DBNC_STANDBY;	/* back to standby for interrupt */
				EIMSK |= (1 << INT0);	/* start listening on INT0 again */
			break; /* case DBNC_SUCCESS */
			/* otherwise */
			default:
				/* hold if on standby */
			break; /* default */
		} /* end switch (int0_debounce) */

		/* check the int1 debounce */
		switch (int1_debounce) {
			/* if the debounce state is timing */
			case DBNC_TIMING:
				TCCR3B &= CLEAR_CS;	/* clear the clock select */
				TCCR3B |= PRESCALING_DEBOUNCE;	/* start the clock with prescaling */
				int1_debounce = DBNC_STANDBY;	/* signal standby for timer */
			break; /* case DBNC_TIMING */
			/* if the debounce succeeded */
			case DBNC_SUCCESS:
				--PORTA;	/* decrement PORTA */
			case DBNC_FAILURE:	/* either way, at the end of debouncing */
				int1_debounce = DBNC_STANDBY;	/* back to standby for interrupt */
				EIMSK |= (1 << INT1);	/* start listening on INT1 again */
			break; /* case DBNC_SUCCESS */
			/* otherwise */
			default:
				/* hold if on standby */
			break; /* default */
		} /* end switch (int1_debounce) */
	} /* end while (1) */
} /* end main(void) */

/* Interrupt service routine for external interrupt 0:
 * Starts the debouncer if ready.
 */
ISR(INT0_vect, ISR_BLOCK)
{
	/* stop if not ready to debounce */
	if (int0_debounce != DBNC_STANDBY) return;
	/* otherwise */
	EIMSK &= (~(1 << INT0));	/* ignore further INT0s while debouncing */
	int0_debounce = DBNC_TIMING;	/* start the debouncer */
} /* end ISR(INT0_vect, ISR_BLOCK) */

/* Interrupt service routine for external interrupt 1:
 * Starts the debouncer if ready.
 */
ISR(INT1_vect, ISR_BLOCK)
{
	/* stop if not ready to debounce */
	if (int1_debounce != DBNC_STANDBY) return;
	/* otherwise */
	EIMSK &= (~(1 << INT1));	/* ignore further INT1s while debouncing */
	int1_debounce = DBNC_TIMING;	/* start the debouncer */
} /* end ISR(INT1_vect, ISR_BLOCK) */

/* Interrupt service routine for timer1 match on compare A:
 * Stop the timer, and check if INT0 is still low.
 */
ISR(TIMER1_COMPA_vect, ISR_BLOCK)
{
	int0_debounce = DBNC_FAILURE;	/* fail by default */
	TCCR1B &= CLEAR_CS;	/* stop timer1 */
	TCNT1 = 0;	/* reset the timer for the next cycle */

	/* stay on standby if not still low bcs the low was just a fluke */
	if (PIND & (1 << PD_INT0)) return;

	/* otherwise, flag the success */
	int0_debounce = DBNC_SUCCESS;
} /* end ISR(TIMER1_COMPA_vect, ISR_BLOCK) */

/* Interrupt service routine for timer3 match on compare A:
 * Stop the timer, and check if INT1 is still low.
 */
ISR(TIMER3_COMPA_vect, ISR_BLOCK)
{
	int1_debounce = DBNC_FAILURE;	/* fail by default */
	TCCR3B &= CLEAR_CS;	/* stop timer1 */
	TCNT3 = 0;	/* reset the timer for the next cycle */

	/* stay on standby if not still low bcs the low was just a fluke */
	if (PIND & (1 << PD_INT1)) return;

	/* otherwise, flag the success */
	int1_debounce = DBNC_SUCCESS;
} /* end ISR(TIMER3_COMPA_vect, ISR_BLOCK) */

/* Interrupt service routine for pin change 0:
 * PA0 will change every time that PORTA is incremented or decremented.
 */
ISR(PCINT0_vect, ISR_BLOCK)
{
	PORTA &= (~PALF_MASK);	/* clear the PORTA is large flag */
	/* if PORTA is large */
	if (PORTA > PORTA_LARGE)
	{
		PORTA |= PALF_MASK;	/* set the PORTA is large flag */
	} /* end if (PORTA > PORTA_LARGE) */
} /* end ISR(PCINT0_vect, ISR_BLOCK) */
