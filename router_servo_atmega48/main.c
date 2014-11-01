/* main.c
 * Author:  Klas löfstedt 
 * Created: ‎16 ‎sep ‎2014, ‏‎00:19:31
 * Webpage: klaslofstedt.se
 * Description: This program reads ZigBee packages from an Xbee router
 *              connected to USART0. Status LED are conencted to PD7.
 *              The program is written for an ATmega48.           
 */ 
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <avr/interrupt.h>
//----------------------------------------------------------------------------//
#define USART_BAUDRATE 9600
#define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)
//----------------------------------------------------------------------------//
//	frame + id + 8byte adr + 2 byte adr + radius + option + data
#define SIZE_OF_XBEE_PACKAGE 14	// 0 to 14 gives 15 bytes
//----------------------------------------------------------------------------//
volatile uint8_t g_idx = 0;
volatile uint8_t g_xbee_data;
//----------------------------------------------------------------------------//
enum XbeeDataStatus { IDLE, READING, READY };
volatile enum XbeeDataStatus g_xbee_status = IDLE;
//----------------------------------------------------------------------------//
void usart_init(void)
{
	UBRR0L = BAUD_PRESCALE;
	UBRR0H = (BAUD_PRESCALE >> 8);
	UCSR0B = ((1<<RXEN0) | (1<<RXCIE0));
	UCSR0C = ((1 << USBS0) | (3 << UCSZ00)); // Use 8-bit char
}
//----------------------------------------------------------------------------//
void pwm_init()
{
   	TCCR1A = ((1<<WGM11) | (1<<COM1A1));  // set pwm mode 14 & non-inverted output
   	TCCR1B = ((1<<WGM12) | (1<<WGM13) | (1<<CS11)); // set prescaler to 8 
	// set period = 20 ms   
   	// ICR1 = (F_CPU[MHz] * period[us]) / prescaler) - 1;
   	ICR1 = 19999;  // ICR1 = (8 * 20000 / 8) - 1;	
}
//----------------------------------------------------------------------------//
void led_init(void)
{
	DDRD |= (1 << PD7);
	PORTD |= ~(1 << PD7);
}
//----------------------------------------------------------------------------//
inline uint8_t read_xbee_byte()
{
    return UDR0;
}
//----------------------------------------------------------------------------//
inline void read_xbee_data()
{
    if(g_idx == SIZE_OF_XBEE_PACKAGE) {
       	g_xbee_data = read_xbee_byte();
        g_xbee_status = READY;
        g_idx = 0;
    } else if(g_idx > SIZE_OF_XBEE_PACKAGE){
		g_idx = 0;
		g_xbee_status = IDLE;
	}else{
    	g_idx++;
    	read_xbee_byte();
    }
}
//----------------------------------------------------------------------------//
void turn_servo(unsigned int desired_position)
{
	DDRB |= (1 << PB1); // PWM pin
	OCR1A = desired_position;
}
//----------------------------------------------------------------------------//
ISR(USART_RX_vect)
{
	PORTD = (1 << PD7);
	if(g_xbee_status == IDLE){
		if (read_xbee_byte() == 0x7E){
			g_xbee_status = READING;
		}
	}else if(g_xbee_status == READING){
		read_xbee_data();
	}
}
//----------------------------------------------------------------------------//
void activate_servo()
{
	switch ( g_xbee_data ) {
	case 0x00:
		turn_servo(750);
  		break;
	case 0x01:
		turn_servo(2150);
  		break;
  	case 0xFF:
  		PORTD ^= (1 << PD7);
  		break;
	default:
  		//code
  	break;
	}
}
//----------------------------------------------------------------------------//
int main(void)
{
	usart_init();  
	sei();         // enable all interrupts
	led_init();    // init LEDs for testing
	pwm_init();
	for(;;)
	{
		if(g_xbee_status == READY){
			activate_servo();
			g_xbee_status = IDLE;
		}
		_delay_ms(1); // to be able to se the led flash up
		PORTD = ~(1 << PD7);
	}
}
//----------------------------------------------------------------------------//
