#include <avr/io.h>
#include <util/delay.h>
#include "tuner.h"

/* 
set/reset Time dieser Relais: 15ms
es gibt 8 L Relais, 8 C Relais und zwei V/H Relais

L-Relais: 
K10-K17 und K18 bis K25
angeschlossen an:
REL17/18 bis REL31/32 (set/reset) und auf der Ctrl-Platine an:
RELAIS14/15 bis RELAIS28/29
in Array ist das Index 8..15

C-Relais:
K1 bis K8 angeschlossen an:
REL1/2 bis REL15/16 und auf der Ctrl-Platine an:
RELAIS0/1 bis RELAIS8/9 weiter mit RELAIS34/35 und dann RELAIS10/11 und 12/13
in Array ist das Index 0..7

V-Relais (C am Eingang oder Ausgang, je nachdem wie man es sieht):
K9 an REL33/34 und auf der Ctrl-Platine an:
RELAIS30/31
in Array ist das Index 16 

H-Relais
K26 an REL35/36 und auf der Ctrl-Platine an:
RELAIS32/33
in Array ist das Index 17
*/

char relais[18]		= {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
char oldrelais[18]	= {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

void pulsedirectport(char port, unsigned char portnum)
{
    if(port == 'B')
    {
        PORTB = PORTB | (1<<portnum);
        _delay_ms(20);
        PORTB = PORTB & ~(1<<portnum);
        return;
    }
    
    if(port == 'C')
    {
        PORTC = PORTC | (1<<portnum);
        _delay_ms(20);
        PORTC = PORTC & ~(1<<portnum);
        return;
    }
    
    if(port == 'D')
    {
        PORTD = PORTD | (1<<portnum);
        _delay_ms(20);
        PORTD = PORTD & ~(1<<portnum);
        return;
    }
} 

/*
74HC595:
SCL-10(/MR) : Master Reset setzt SRs auf 0, wenn gleichzeitig STCP (steigende Flanke) dann auch die Ausgänge auf 0
SCK-11(SHCP): SR Clock lädt neuen Wert mit steigender Flanke ins SR 
RCK-12(STCP): Storage Reg Clock steigende Flanke lädt SR in die Ausgänge
G-13(/OE)   : Output Enable muss immer 0 sein
*/
void setShiftRegs(unsigned long val)
{
	int i;

    if(val == 0)
    {
        // setze alle Ausgänge auf 0
		portClr(PORTC, PORTC1);		// ziehe Reset
		portSet(PORTC, PORTC2);		// auf die Ausgänge
		portSet(PORTC, PORTC1);		// und zurück in die Grundstellung
		portClr(PORTC, PORTC2);
        return;
    }
    
    for(i=(24-1); i>=0; i--)
    {
        if(val & (1L << i))
			portSet(PORTB, PORTB2);
        else  
			portClr(PORTB, PORTB2);
        
		portSet(PORTC, PORTC0);		// reinschieben
		portClr(PORTC, PORTC0);
    }
    
	portSet(PORTC, PORTC2);			// auf die Ausgänge
	portClr(PORTC, PORTC2);
}

// relnum: 0..35 entsprechend Relaisanschluss RELAIS0-35
void pulserelais(char relnum)
{
	unsigned long sr, tmp;

    // direkt angeschlossene Relais
    switch (relnum)
    {
        case 24: pulsedirectport('B',0); return;
        case 25: pulsedirectport('B',1); return;
        case 26: pulsedirectport('D',2); return;
        case 27: pulsedirectport('D',3); return;
        case 28: pulsedirectport('D',4); return;
        case 29: pulsedirectport('D',5); return;
        case 30: pulsedirectport('D',6); return;
        case 31: pulsedirectport('D',7); return;        
        case 32: pulsedirectport('C',4); return;
        case 33: pulsedirectport('C',5); return;
        case 34: pulsedirectport('B',4); return;
        case 35: pulsedirectport('B',5); return;
    }
    
    // Relais an Schieberegistern
    // Vorbelegen des Wertes
    tmp = relnum;
    sr = (1L<<tmp);
    setShiftRegs(sr);
    _delay_ms(20);
    setShiftRegs(0);    
}

void pulserelaisindex(unsigned char r)
{
	unsigned char rn;

    switch (r)
    { 
        // Cs
        case 0 :  rn = 0; break;
        case 1 :  rn = 2; break;
        case 2 :  rn = 4; break;
        case 3 :  rn = 6; break;
        case 4 :  rn = 8; break;
        case 5 :  rn = 34; break;
        case 6 :  rn = 10; break;
        case 7 :  rn = 12; break;
             
        // Ls
        case 8 :  rn = 14; break;
        case 9 :  rn = 16; break;
        case 10:  rn = 18; break;
        case 11:  rn = 20; break;
        case 12:  rn = 22; break;
        case 13:  rn = 24; break;
        case 14:  rn = 26; break;
        case 15:  rn = 28; break;
             
        // V oder H
        case 16:  rn = 30; break;
        case 17:  rn = 32; break;
		
		default:
				  rn = 0;
    }
    
    if(relais[r] == 0)
        pulserelais(rn+1);			// Relais abfallen lassen
    else
        pulserelais(rn);			// Relais anziehen lassen
}

void moveRelais()
{
	unsigned char i;

    for(i=0; i<18; i++)
    {
        if(relais[i] != oldrelais[i])
        {
            pulserelaisindex(i);
        }
    }

    for(i=0; i<18; i++) 
        oldrelais[i] = relais[i];
}

void switchRelais()
{
	unsigned long i;

	if(valid == 0 && shortvalid == 0)	// kein Kommando
		return;

	// Textkommandos
    if(valid != 0)
	{ 
		// Belege die Relaisvariable vor
		if(command == 'C')
		{          
			relais[(int)parameter] = action;                        
		}
     
		if(command == 'L')
		{          
			relais[(int)parameter + 8] = action;                       
		}
    
		if(command == 'V')
		{          
			relais[16] = action;

			if(action == 1)
				relais[17] = 0;
		} 

		if(command == 'H')
		{          
			relais[17] = action;                       

			if(action == 1)
				relais[16] = 0;
		} 
	}  

	// Kommando in Kurzform
	if(shortvalid != 0)
	{
		/*
		 shortval:
		 Bit 0..7 = C0..7
		 Bit 8..15 = L0..7
		 Bit 16 = V
		 Bit 17 = H
		*/
		for(i=0; i<18; i++)
		{
			relais[i] = (shortval & (1UL<<i)) ? 1 : 0;
		}
	}
    
    // steuere geänderte Relais an
    moveRelais();
    
    valid = 0;
	shortvalid = 0;
}

void forceRelais(char cmd, char num, char act)
{
	command = cmd;
	parameter = num;
	action = act;
	valid = 1; 
	switchRelais();
}

void Grundstellung()
{
	char i;

	#ifdef INIT_ON_OFF

	/* Last state flags */
	for(i = 0; i < 18; i++) {
		oldrelais[(int)i] = 0x00;
	}

	for(i = 1; i <= 8; i++)	{
		forceRelais('C', i - 1, 1);
	}

	forceRelais('V', 0, 1);
	forceRelais('H', 0, 1);

	for(i = 1; i <= 8; i++) {
		forceRelais('L', i-1, 0);
	}

	#endif


	/* Last state flags */
	for(i = 0; i < 18; i++) {
		oldrelais[(int)i] = 0xff;
	}

	for(i = 1; i <= 8; i++)
	{
		forceRelais('C', i - 1, 0);
	}

	forceRelais('V', 0, 0);
	forceRelais('H', 0, 0);

	for(i = 1; i <= 8; i++)
	{
		forceRelais('L', i-1, 1);
	}
}
