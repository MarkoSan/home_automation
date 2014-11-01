/* make all
 * MSP430Flasher.exe -n msp430g2xx3 -w "main.hex" -v -z [VCC] -m SBW2 -std=c99
 * main.c
 * Author:  Klas löfstedt 
 * Created: ‎31 ‎oct ‎2014, ‏‎14:25
 * Webpage: klaslofstedt.se
 * Description: This program reads ZigBee packages from an Xbee router
 *              connected to UART. P1.5 controls the relay and a status
 *				led is connected to P1.0.
 *              The program is written for an MSP430g2553.           
 */ 
#include <msp430g2553.h>
#include <stdint.h>
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
void pin_init()
{
	P1DIR = (BIT0 | BIT5);
	P1OUT =~ (BIT0 | BIT5);
}
//----------------------------------------------------------------------------//
void freq_init()
{
	DCOCTL = 0;                               
	BCSCTL1 = CALBC1_1MHZ;                    
	DCOCTL = CALDCO_1MHZ;
}
//----------------------------------------------------------------------------//
void uart_init()
{
	P1SEL = (BIT1 | BIT2);
	P1SEL2 = (BIT1 | BIT2); 
	UCA0CTL1 = UCSWRST; 
    UCA0CTL1 |= UCSSEL_2;                     // SMCLK
    UCA0BR0 = 104;                            // 1MHz 9600
    UCA0BR1 = 0;                              // 1MHz 9600
    UCA0MCTL = UCBRS0;                        // Modulation UCBRSx = 1 
    UCA0CTL1 &= ~UCSWRST; 
    IE2 |= UCA0RXIE;						//Enable USCI_A0 RX interrupt
    __enable_interrupt();
}
//----------------------------------------------------------------------------// 
#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR(void)
{
	P1OUT = BIT0;
	if(g_xbee_status == IDLE){
		uint8_t uart_data = UCA0RXBUF;
		if (uart_data == 0x7E){
			g_xbee_status = READING;
		}
	}else if(g_xbee_status == READING){
		if(g_idx >= SIZE_OF_XBEE_PACKAGE) {
			g_xbee_data = UCA0RXBUF;
			g_xbee_status = READY;
			g_idx = 0;
		}else{
			g_idx++;
			uint8_t temp = UCA0RXBUF;
		}
	}
}
//----------------------------------------------------------------------------//
void toggle_relay()
{
	switch ( g_xbee_data ) {
		case 0x00:
			P1OUT ^= (BIT5);
        	__delay_cycles(500000L);
        	P1OUT ^= (BIT5);
		break;
		case 0x01:
			P1OUT ^= (BIT5);
        	__delay_cycles(500000L);
        	P1OUT ^= (BIT5);
		break;

		default:
  				// code
		break;
	}
}
//----------------------------------------------------------------------------//
int main(void) 
{
	WDTCTL = WDTPW | WDTHOLD;		// Stop watchdog timer
	freq_init();
	pin_init();
	uart_init();
	
	for(;;) 
	{
		if(g_xbee_status == READY){
			__disable_interrupt();
			toggle_relay();
			g_xbee_status = IDLE;
			__enable_interrupt();
		}
	}
}
//----------------------------------------------------------------------------//