/* main.c
 * Author:  Klas löfstedt 
 * Created: ‎10 ‎sep ‎2014, ‏‎21:19
 * Webpage: klaslofstedt.se
 * Description: This program reads ZigBee packages from an Raspberry Pi
 *              connected to USART0, and transmits them to an Xbee coordinator
 *              connected to USART1. Status LEDs are conencted to PC0 and PC1.
 *              The program is written for an ATmega162.        
 */ 
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <stdio.h>
//----------------------------------------------------------------------------//
#define USART_BAUDRATE 9600
#define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)
//----------------------------------------------------------------------------//
#define XBEE_EXT_ADDRESS_SIZE 8
#define XBEE_NETWORK_ADDRESS_SIZE 2
#define XBEE_VALUE_IDX XBEE_EXT_ADDRESS_SIZE + XBEE_NETWORK_ADDRESS_SIZE
typedef struct {
    uint8_t extAddress[XBEE_EXT_ADDRESS_SIZE];
    uint8_t networkAddress[XBEE_NETWORK_ADDRESS_SIZE];
    uint8_t value;
    uint8_t idx;
} RaspberryData;
RaspberryData g_raspberry_data = {{0,0,0,0,0,0,0,0},{0,0},0,0};
enum RaspberryDataStatus { IDLE, READING, READY };
volatile enum RaspberryDataStatus g_raspberry_status = IDLE;
//----------------------------------------------------------------------------//
inline uint8_t read_raspberry_byte()
{
    return UDR0;
}
//----------------------------------------------------------------------------//
void send_xbee_byte(uint8_t send_byte)
{
	while((UCSR1A &(1<<UDRE1)) == 0);
	UDR1 = send_byte; // Transmit data
}
//----------------------------------------------------------------------------//
inline void read_raspberry_data()
{
    if(g_raspberry_data.idx < XBEE_EXT_ADDRESS_SIZE) {
        g_raspberry_data.extAddress[g_raspberry_data.idx] = read_raspberry_byte();
        ++g_raspberry_data.idx;
    } else if (g_raspberry_data.idx >=XBEE_EXT_ADDRESS_SIZE &&
               g_raspberry_data.idx < XBEE_VALUE_IDX) {
        const uint8_t idx = g_raspberry_data.idx - XBEE_EXT_ADDRESS_SIZE;
        g_raspberry_data.networkAddress[idx] = read_raspberry_byte();
        ++g_raspberry_data.idx;  
    } else if (g_raspberry_data.idx == XBEE_VALUE_IDX) {
        g_raspberry_data.value = read_raspberry_byte();
        g_raspberry_status = READY;
        g_raspberry_data.idx = 0;
    }
}
//----------------------------------------------------------------------------//
ISR(USART0_RXC_vect)
{
    PORTC |= (1 << PC0);
    if(g_raspberry_status == IDLE) {
        g_raspberry_status = READING;
    } else if(g_raspberry_status == READING) {
        read_raspberry_data();
    }
}
//----------------------------------------------------------------------------//
void led_init(void)
{
	DDRC = ((1 << PC0) | (1 << PC1));
    PORTC = (1 << PC0);
}
//----------------------------------------------------------------------------//
void usart_init()
{
    UCSR1B = (1<<TXEN1); // transmitter
    UCSR0B = ((1<< RXEN0) | (1<<RXCIE0));// receiver + rx interrupt + transmitter
    UCSR0C = (1 << URSEL0) | (1 << UCSZ00) | (1 << UCSZ01); // Use 8-bit char
	UCSR1C = (1 << URSEL1) | (1 << UCSZ10) | (1 << UCSZ11); // Use 8-bit char
    UBRR0L = UBRR1L = BAUD_PRESCALE; // Set baud rate, lower 8 bits in low byte
	UBRR0H = UBRR1H = (BAUD_PRESCALE >> 8); 
}
//----------------------------------------------------------------------------//
void send_xbee_package()
{
	send_xbee_byte(0x7E);	// start byte
    send_xbee_byte(0x00);	// lenghtbyte 1
    send_xbee_byte(0x0F);	// lenghtbyte 2
    send_xbee_byte(0x10);	// msg type
    send_xbee_byte(0x01);	// frame ID
    int i;
    for(i = 0; i < XBEE_EXT_ADDRESS_SIZE; i++){
        send_xbee_byte(g_raspberry_data.extAddress[i]);
        }
    send_xbee_byte(g_raspberry_data.networkAddress[0]);
    send_xbee_byte(g_raspberry_data.networkAddress[1]);
    send_xbee_byte(0x00);	// radius
    send_xbee_byte(0x00);	// option
    send_xbee_byte(g_raspberry_data.value);
    long temp = 0x10 + 0x01 + g_raspberry_data.extAddress[0] + g_raspberry_data.extAddress[1] + 
    	g_raspberry_data.extAddress[2] + g_raspberry_data.extAddress[3] + g_raspberry_data.extAddress[4] +
    	g_raspberry_data.extAddress[5] + g_raspberry_data.extAddress[6] + g_raspberry_data.extAddress[7] + 
    	g_raspberry_data.networkAddress[0] + g_raspberry_data.networkAddress[1] + g_raspberry_data.value;
    send_xbee_byte(0xFF - (temp & 0xFF));
}
//----------------------------------------------------------------------------//
void wait_for_raspberry()
{
    int i;
    for(i = 0; i < 3; i++){
        _delay_ms(5000);
    }
}
//----------------------------------------------------------------------------//
int main(void)
{
    led_init();             // init LEDs for testing
    wait_for_raspberry();   // wait for raspberry pi start-up
    PORTC = (1 << PC1); 
	sei();                  // enable interrupt
	usart_init();           // init USART

	for(;;){        
		if(g_raspberry_status == READY){
			ATOMIC_BLOCK ( ATOMIC_RESTORESTATE ){
				send_xbee_package();
                g_raspberry_status = IDLE;
                PORTC ^= (1 << PC0);
			}
		}
	}
}
//----------------------------------------------------------------------------//
