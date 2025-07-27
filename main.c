/*
 * function-genrator.c
 *
 * Created: 05/07/2025 10:49:19
 * Author : User
 */ 
//----------------------------------------------------------------------------------------------------------------
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/delay.h>
//----------------------------------------------------------------------------------------------------------------
// Pin definitions
#define BUTTON_PIN   PA0
#define MODE_SWITCH_PIN   PA1

#define PIEZO1_PIN    PB0
#define PIEZO2_PIN    PB1
#define LED_PIN      PB2

// Timer0 overflow frequency (~16Hz at 8MHz with 1024 prescaler)
#define TIMER0_RELOAD   131 //131  // 256 - 125 = 131 (for ~16Hz)
//----------------------------------------------------------------------------------------------------------------
// Function prototypes
void setupPins(void);
void setupTimer1(void);
void setupTimer0(void);
//----------------------------------------------------------------------------------------------------------------
// Global variables
volatile uint8_t pwm_active = 0;
volatile uint8_t led_mode = 0;
//----------------------------------------------------------------------------------------------------------------
// Initialize I/O pins
void setupPins(void) {
	
	// Set LED pin as outputs
	DDRB |= (1 << LED_PIN);
	// Set piezo_pins as outputs
	// PWM output (PB1/OC1A)(PB0/!(OC1A)) as output 
	DDRB |= (1 << PIEZO1_PIN) | (1 << PIEZO2_PIN);
	// Set button pins as inputs with pull-ups
	DDRA &= ~((1 << BUTTON_PIN) | (1 << MODE_SWITCH_PIN));
	PORTA |= (1 << BUTTON_PIN) | (1 << MODE_SWITCH_PIN);
}
//----------------------------------------------------------------------------------------------------------------
// Configure Timer1 for PWM toggle mode (~24.5kHz)
void setupTimer1(void) {
	TCCR1A = 0;             // Reset control register A
	TCCR1B = 0;             // Reset control register B
	TCNT1 = 0;              // Reset counter

	OCR1A = OCR1C/2;             // 50% duty cycle (~24.5kHz at 8MHz)
	OCR1C = 40;            // Top value for frequency

	// Configure for PWM, toggle OC1A (PB0)
	TCCR1A |= (1 << COM1A0); // Toggle OC1A on compare match
	TCCR1A |= (1 << PWM1A);  // Enable PWM mode

	// Prescaler 1/64 (CS12 + CS11 + CS10)
	//TCCR1B |= (1 << CS12) | (1 << CS11) | (1 << CS10);
}
//----------------------------------------------------------------------------------------------------------------
// Configure Timer0 for overflow interrupts (~16Hz)
void setupTimer0(void) {
	TCCR0 = 0;              // Normal mode (no CTC)
	TCNT0 = TIMER0_RELOAD;  // Preload_timer for ~16Hz
	
	// Prescaler 1024 (CS02 + CS00)
	TCCR0 |= (1 << CS02) | (1 << CS00);
	
	// Enable Timer0 Overflow Interrupt (TOIE0)
	TIMSK |= (1 << TOIE0);
}
//----------------------------------------------------------------------------------------------------------------
// Toggle LED states
void toggle_leds(void) {
	PORTB ^= (1 << LED_PIN);
}
//----------------------------------------------------------------------------------------------------------------
// Start PWM generation
void start_pwm(void) {
	// Start Timer1 
	// Prescaler 1/64 (CS12 + CS11 + CS10)
	TCCR1B |= (1 << CS12) | (1 << CS11) | (1 << CS10);
	pwm_active = 1;
}
//----------------------------------------------------------------------------------------------------------------
// Stop PWM generation
void stop_pwm(void) {
	// Stop Timer1
	TCCR1B &= ~((1 << CS12) | (1 << CS11) | (1 << CS10));
	// Make sure piezo_pins are low
	PORTB &= ~((1 << PIEZO1_PIN) | (1 << PIEZO2_PIN));
	pwm_active = 0;
}
//----------------------------------------------------------------------------------------------------------------
// ISR triggered on Timer0 overflow (~16Hz)
ISR(TIMER0_OVF_vect) {
	TCNT0 = TIMER0_RELOAD;  // Reload timer value
	// Check button 2 (LED mode)
	if(!(PINA & (1 << MODE_SWITCH_PIN))) {
		_delay_ms(50);  // Debounce
		if(!(PINA & (1 << MODE_SWITCH_PIN))) {
			toggle_leds();
			//toggle_pwm
			start_pwm();
			// Wait for button release
			while(!(PINA & (1 << MODE_SWITCH_PIN)));
		}
	}

}
//----------------------------------------------------------------------------------------------------------------
int main(void) {
	cli(); // Disable interrupts during setup
	setupPins();
	setupTimer1();
	setupTimer0();
	sei(); // Enable interrupts

	while (1) {
		// Main loop - all logic handled in ISR
		        // Check button 1 (start/stop PWM)
		        if(!(PINA & (1 << BUTTON_PIN))) {
			        _delay_ms(50);  // Debounce
			        if(!(PINA & (1 << BUTTON_PIN))) {
					        start_pwm();
							toggle_leds();
				        }
				        // Wait for button release
				        while(!(PINA & (1 << BUTTON_PIN)));
			        }
		        
		        
		        // Check button 2 (LED mode)
		        if(!(PINA & (1 << MODE_SWITCH_PIN))) {
			        _delay_ms(50);  // Debounce
			        if(!(PINA & (1 << MODE_SWITCH_PIN))) {
				        led_mode = 1;  // Toggle LED mode
						PORTB ^= (1 << LED_PIN);
				        // Wait for button release
				        while(!(PINA & (1 << MODE_SWITCH_PIN)));
			        }
		        }
	}
}
//----------------------------------------------------------------------------------------------------------------