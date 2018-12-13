#include "tuner.h"

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
				c &= ~0x20; // wandle in Großbuchstaben
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
						status = 0;
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
					status = 0;
				break;
    
		case 2:
				if(c == '0' || c == '1')
				{
					action = c - 0x30;
					status = 3;
				} 
				else
					status = 0;
				break;
    
		case 3:
				// cr oder lf Abschluss von L oder C Kommando
				if(c == '\r' || c == '\n')
					valid = 1;
            
				status = 0;
				break;   
	
		case 10: 
				shortval = c;
				status ++;
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
	}   
}
