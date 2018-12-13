/*******************************************************
Project : Tuner
Version : 1
Date    : 05.05.2016
Author  : Kurt Moraw
Company : Helitron


Chip type               : ATmega88PA
Program type            : Application
AVR Core Clock frequency: 8,000000 MHz 
Memory model            : Small
External RAM size       : 0
Data Stack size         : 256
*******************************************************/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <utils/interrupt_avr8.h>
#include <stdio.h>
#include <stdbool.h>


#define DATA_REGISTER_EMPTY (1<<UDRE0)
#define RX_COMPLETE (1<<RXC0)
#define FRAMING_ERROR (1<<FE0)
#define PARITY_ERROR (1<<UPE0)
#define DATA_OVERRUN (1<<DOR0)

// USART Receiver buffer
#define RX_BUFFER_SIZE0 128
char rx_buffer0[RX_BUFFER_SIZE0];

#if RX_BUFFER_SIZE0 <= 256
volatile unsigned char rx_wr_index0=0,rx_rd_index0=0;
#else
volatile unsigned int rx_wr_index0=0,rx_rd_index0=0;
#endif

#if RX_BUFFER_SIZE0 < 256
volatile unsigned char rx_counter0=0;
#else
volatile unsigned int rx_counter0=0;
#endif

// This flag is set on USART Receiver buffer overflow
bool rx_buffer_overflow0;

// USART Receiver interrupt service routine
ISR(USART_RX_vect)
{
	unsigned char status;
	char data;

	status=UCSR0A;
	data=UDR0;
	if (!rx_buffer_overflow0 && (status & (FRAMING_ERROR | PARITY_ERROR | DATA_OVERRUN))==0)
	{
		rx_buffer0[rx_wr_index0++]=data;
#if RX_BUFFER_SIZE0 == 256
		// special case for receiver buffer size=256
		if (++rx_counter0 == 0)
			rx_buffer_overflow0=1;
#else
		if (rx_wr_index0 == RX_BUFFER_SIZE0)
			rx_wr_index0=0;

		if (++rx_counter0 == RX_BUFFER_SIZE0)
		{
			rx_counter0=0;
			rx_buffer_overflow0=1;
		}
#endif
   }
}


// Set a port bit
void portSet(uint16_t port, char bitPos)
{
	uint8_t *const adr = (uint8_t*) port;

	*adr |= (1 << bitPos);
}

// Clear a port bit
void portClr(uint16_t port, char bitPos)
{
	uint8_t *const adr = (uint8_t*) port;

	*adr &= ~(1 << bitPos);
}


// Get a character from the USART Receiver buffer
#define _ALTERNATE_GETCHAR_
//#pragma used+

char dataAvailable()
{
    return rx_counter0;
}

char tuner_getchar(void)
{
    char data;

    while (rx_counter0==0)
		;

    data=rx_buffer0[rx_rd_index0++];
	// echo
	putchar(data);

    #if RX_BUFFER_SIZE0 != 256
	if (rx_rd_index0 == RX_BUFFER_SIZE0) 
		rx_rd_index0=0;
    #endif

	cpu_irq_disable();
    --rx_counter0;
	cpu_irq_enable();

    return data;
}
//#pragma used-

// Standard Input/Output functions
#include <stdio.h>
#include "tuner.h"

void tuner_main(void)
{
    // Crystal Oscillator division factor: 1
//  #pragma optsize-
    CLKPR=(1<<CLKPCE);
    CLKPR=(0<<CLKPCE) | (0<<CLKPS3) | (0<<CLKPS2) | (0<<CLKPS1) | (0<<CLKPS0);
//  #ifdef _OPTIMIZE_SIZE_
//  #pragma optsize+
//  #endif

    // Input/Output Ports initialization
    // Port B initialization
    // Function: Bit5=Out Bit4=Out Bit3=Out Bit2=Out Bit1=Out Bit0=Out 
    DDRB= (1<<DDB5) | (1<<DDB4) | (1<<DDB3) | (1<<DDB2) | (1<<DDB1) | (1<<DDB0);
    // State: Bit7=0 Bit6=0 Bit5=0 Bit4=0 Bit3=0 Bit2=0 Bit1=0 Bit0=0 
    PORTB= (0<<PORTB5) | (0<<PORTB4) | (0<<PORTB3) | (0<<PORTB2) | (0<<PORTB1) | (0<<PORTB0);

    // Port C initialization
    // Function: Bit5=Out Bit4=Out Bit3=Out Bit2=Out Bit1=Out Bit0=Out 
    DDRC= (1<<DDC5) | (1<<DDC4) | (1<<DDC3) | (1<<DDC2) | (1<<DDC1) | (1<<DDC0);
    // State: Bit6=0 Bit5=0 Bit4=0 Bit3=0 Bit2=0 Bit1=1 Bit0=0 
    PORTC= (0<<PORTC5) | (0<<PORTC4) | (0<<PORTC3) | (0<<PORTC2) | (1<<PORTC1) | (0<<PORTC0);

    // Port D initialization
    // Function: Bit7=Out Bit6=Out Bit5=Out Bit4=Out Bit3=Out Bit2=Out Bit1=Out Bit0=In 
    DDRD=(1<<DDD7) | (1<<DDD6) | (1<<DDD5) | (1<<DDD4) | (1<<DDD3) | (1<<DDD2) | (1<<DDD1) | (0<<DDD0);
    // State: Bit7=0 Bit6=0 Bit5=0 Bit4=0 Bit3=0 Bit2=0 Bit1=0 Bit0=T 
    PORTD=(0<<PORTD7) | (0<<PORTD6) | (0<<PORTD5) | (0<<PORTD4) | (0<<PORTD3) | (0<<PORTD2) | (0<<PORTD1) | (0<<PORTD0);

    // Timer/Counter 0 initialization
    // Clock source: System Clock
    // Clock value: Timer 0 Stopped
    // Mode: Normal top=0xFF
    // OC0A output: Disconnected
    // OC0B output: Disconnected
    TCCR0A=(0<<COM0A1) | (0<<COM0A0) | (0<<COM0B1) | (0<<COM0B0) | (0<<WGM01) | (0<<WGM00);
    TCCR0B=(0<<WGM02) | (0<<CS02) | (0<<CS01) | (0<<CS00);
    TCNT0=0x00;
    OCR0A=0x00;
    OCR0B=0x00;

    // Timer/Counter 1 initialization
    // Clock source: System Clock
    // Clock value: Timer1 Stopped
    // Mode: Normal top=0xFFFF
    // OC1A output: Disconnected
    // OC1B output: Disconnected
    // Noise Canceler: Off
    // Input Capture on Falling Edge
    // Timer1 Overflow Interrupt: Off
    // Input Capture Interrupt: Off
    // Compare A Match Interrupt: Off
    // Compare B Match Interrupt: Off
    TCCR1A=(0<<COM1A1) | (0<<COM1A0) | (0<<COM1B1) | (0<<COM1B0) | (0<<WGM11) | (0<<WGM10);
    TCCR1B=(0<<ICNC1) | (0<<ICES1) | (0<<WGM13) | (0<<WGM12) | (0<<CS12) | (0<<CS11) | (0<<CS10);
    TCNT1H=0x00;
    TCNT1L=0x00;
    ICR1H=0x00;
    ICR1L=0x00;
    OCR1AH=0x00;
    OCR1AL=0x00;
    OCR1BH=0x00;
    OCR1BL=0x00;

    // Timer/Counter 2 initialization
    // Clock source: System Clock
    // Clock value: Timer2 Stopped
    // Mode: Normal top=0xFF
    // OC2A output: Disconnected
    // OC2B output: Disconnected
    ASSR=(0<<EXCLK) | (0<<AS2);
    TCCR2A=(0<<COM2A1) | (0<<COM2A0) | (0<<COM2B1) | (0<<COM2B0) | (0<<WGM21) | (0<<WGM20);
    TCCR2B=(0<<WGM22) | (0<<CS22) | (0<<CS21) | (0<<CS20);
    TCNT2=0x00;
    OCR2A=0x00;
    OCR2B=0x00;

    // Timer/Counter 0 Interrupt(s) initialization
    TIMSK0=(0<<OCIE0B) | (0<<OCIE0A) | (0<<TOIE0);

    // Timer/Counter 1 Interrupt(s) initialization
    TIMSK1=(0<<ICIE1) | (0<<OCIE1B) | (0<<OCIE1A) | (0<<TOIE1);

    // Timer/Counter 2 Interrupt(s) initialization
    TIMSK2=(0<<OCIE2B) | (0<<OCIE2A) | (0<<TOIE2);

    // External Interrupt(s) initialization
    // INT0: Off
    // INT1: Off
    // Interrupt on any change on pins PCINT0-7: Off
    // Interrupt on any change on pins PCINT8-14: Off
    // Interrupt on any change on pins PCINT16-23: Off
    EICRA=(0<<ISC11) | (0<<ISC10) | (0<<ISC01) | (0<<ISC00);
    EIMSK=(0<<INT1) | (0<<INT0);
    PCICR=(0<<PCIE2) | (0<<PCIE1) | (0<<PCIE0);

    // USART initialization
    // Communication Parameters: 8 Data, 1 Stop, No Parity
    // USART Receiver: On
    // USART Transmitter: On
    // USART0 Mode: Asynchronous
    // USART Baud Rate: 9600
    UCSR0A=(0<<RXC0) | (0<<TXC0) | (0<<UDRE0) | (0<<FE0) | (0<<DOR0) | (0<<UPE0) | (0<<U2X0) | (0<<MPCM0);
    UCSR0B=(1<<RXCIE0) | (0<<TXCIE0) | (0<<UDRIE0) | (1<<RXEN0) | (1<<TXEN0) | (0<<UCSZ02) | (0<<RXB80) | (0<<TXB80);
    UCSR0C=(0<<UMSEL01) | (0<<UMSEL00) | (0<<UPM01) | (0<<UPM00) | (0<<USBS0) | (1<<UCSZ01) | (1<<UCSZ00) | (0<<UCPOL0);
    /* 9600:
	UBRR0H=0x00;
    UBRR0L=0x33;*/

	// 1200:
	UBRR0H=0x01;
	UBRR0L=0xA0;

    // Analog Comparator initialization
    // Analog Comparator: Off
    // The Analog Comparator's positive input is
    // connected to the AIN0 pin
    // The Analog Comparator's negative input is
    // connected to the AIN1 pin
    ACSR=(1<<ACD) | (0<<ACBG) | (0<<ACO) | (0<<ACI) | (0<<ACIE) | (0<<ACIC) | (0<<ACIS1) | (0<<ACIS0);
    ADCSRB=(0<<ACME);
    // Digital input buffer on AIN0: On
    // Digital input buffer on AIN1: On
    DIDR1=(0<<AIN0D) | (0<<AIN1D);

    // ADC initialization
    // ADC disabled
    ADCSRA=(0<<ADEN) | (0<<ADSC) | (0<<ADATE) | (0<<ADIF) | (0<<ADIE) | (0<<ADPS2) | (0<<ADPS1) | (0<<ADPS0);

    // SPI initialization
    // SPI disabled
    SPCR=(0<<SPIE) | (0<<SPE) | (0<<DORD) | (0<<MSTR) | (0<<CPOL) | (0<<CPHA) | (0<<SPR1) | (0<<SPR0);

    // TWI initialization
    // TWI disabled
    TWCR=(0<<TWEA) | (0<<TWSTA) | (0<<TWSTO) | (0<<TWEN) | (0<<TWIE);

    // Globally enable interrupts
	cpu_irq_enable();

    Grundstellung();    // alle C-Relais offen, alle L-Relais zu

    while (1)
    {
        readSerialCommand();
        switchRelais();
    }
}
