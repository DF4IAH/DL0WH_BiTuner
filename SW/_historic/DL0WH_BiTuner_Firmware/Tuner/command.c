#include <string.h>
#include <atmel_start.h>

#include "tuner.h"

extern volatile uint8_t oldrelais[18];

char command;
char parameter;
char action;
char valid = 0;

unsigned long shortval = 0;
char shortvalid = 0;

void readSerialCommand()
{
	static char status = 0;
	char c;
	unsigned long ul;

    if(dataAvailable() == 0)
		return;    // nichts zu tun

    c = tuner_getchar();
    
    // Daten wurden empfangen
    // Format: L1v bis L8v oder C1v bis C8v oder Vv bzw Hv (Cs vorne oder hinten) gefolgt von CR oder LF
    // v= 0=aus-, 1=einschalten
	// Kurzformate:
	// K gefolgt von 3 Byte:
	// Bit 0..7 = C0..7
	// Bit 8..15 = L0..7
	// Bit 16 = V
	// Bit 17 = H

	// FSM
	switch(status)
	{
		case 0:
				// erlaubte Bytes: L, C, V oder R, alles andere ignorieren
				c &= ~0x20U; // wandle in Großbuchstaben
				if(c == 'L' || c == 'C' || c == 'V' || c == 'H')
				{
					command = c;
					status = 1;
				}

				// Kurzformat
				if( c == 'K')
				{
					command = c;
					status = 10;
				}

				/* Send help */
				if (!status) {
					status = 99;
				}
				break;
    
		case 1:
				// erlaubte Werte L/C: 1-8 oder V/R: cr oder lf
				if(command == 'L' || command == 'C')
				{
					if(c>='1' && c<='8')
					{
						parameter = c - 0x30 -1;
						status = 2;
					}
					else
						status = 99;
				}
				else if(command == 'V' || command == 'H')
				{
					if(c == '0' || c == '1') 
					{                
						action = c - 0x30;
						status = 3;
					}
				}
				else
					status = 99;
				break;
    
		case 2:
				if(c == '0' || c == '1')
				{
					action = c - 0x30;
					status = 3;
				} 
				else
					status = 99;
				break;
    
		case 3:
				// cr oder lf Abschluss von L oder C Kommando
				if(c == '\r' || c == '\n') {
					valid = 1;
					status = 0;

				} else {
					status = 99;
				}

				break;   
	
		case 10: 
				shortval = c;
				status++;
				break;

		case 11:
				ul = c;
				ul <<= 8;
				shortval |= ul;
				status++;
				break;

		case 12:
				ul = c;
				ul <<= 16;
				shortval |= ul;
				shortvalid = 1;
				status=0;
				break;
				
		case 99:
				printHelp();
				status = 0;
	}
}

void printString(const char* str)
{
	size_t len = strlen(str);

	if (len > 254) {
		return;
	}

	for (uint8_t idx = 0; idx < len; idx++) {
		USART_0_write(*(str + idx));
	}
}

void printCrLf(void)
{
	printString("\r\n");
}

void printState(void)
{
	char buf[4] = { ' ', ' ', '?' };

	printString("\r\n## C1 C2 C3 C4 C5 C6 C7 C8  L1 L2 L3 L4 L5 L6 L7 L8  CV CH\r\n ");

	for (int idx = 0; idx < 18; idx++) {
		if (idx == 8 || idx == 16) {
			USART_0_write(' ');
		}
		buf[2] = *(oldrelais + idx) ?  '1' : '0';
		printString(buf);
	}
	printString("\r\n");
}

void printWelcome(void)
{
	printCrLf();
	USART_0_write(12U);
	printCrLf();
	printCrLf();
	printString("* Welcome to the DL0WH modified BiTuner (ATmega88) *");
	printCrLf();
}

void printHelp(void)
{
	printCrLf();
	printString("Commands: Cxy or Lxy or Vy or Hy - where: x=1..8, y=0 (off) or 1 (on)");
	printCrLf();
	printCrLf();
}
