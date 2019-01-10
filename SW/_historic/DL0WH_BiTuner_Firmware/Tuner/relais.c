#include <atmel_start.h>
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


extern bool isReady;

volatile uint8_t relais[18]		= { 0U };
volatile uint8_t oldrelais[18]	= { 0U };

void pulsedirectport(char port, uint8_t portnum)
{
    if(port == 'B')
    {
		PORTB_set_pin_level(portnum, true);
        _delay_ms(20);
		PORTB_set_pin_level(portnum, false);
        return;
    }

    if(port == 'C')
    {
		PORTC_set_pin_level(portnum, true);
        _delay_ms(20);
		PORTC_set_pin_level(portnum, false);
        return;
    }

    if(port == 'D')
    {
		PORTD_set_pin_level(portnum, true);
        _delay_ms(20);
		PORTD_set_pin_level(portnum, false);
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

    if (!val) {
		/* Reset shift registers */
		PORTC_set_pin_level(PORTC1, false);
		PORTC_set_pin_level(PORTC1, true);
    } else {

		/* Push out data stream */
		for (i = 24-1; i >= 0; i--)
		{
			/* Content */
			const uint8_t flag = (val & (1UL << i)) ?  1U : 0U;
			PORTB_set_pin_level(PORTB2, flag);

			/* Clock */
			PORTC_set_pin_level(PORTC0, true);
			PORTC_set_pin_level(PORTC0, false);
		}
	}

	/* Latch new result */
	PORTC_set_pin_level(PORTC2, true);
	PORTC_set_pin_level(PORTC2, false);
}

// relnum: 0..35 entsprechend Relaisanschluss RELAIS0-35
void pulserelais(uint8_t relnum)
{
    /* Relays driven by GPIOs */
    switch (relnum)
    {
        case 24U: pulsedirectport('B', 0U);  return;
        case 25U: pulsedirectport('B', 1U);  return;
        case 26U: pulsedirectport('D', 2U);  return;
        case 27U: pulsedirectport('D', 3U);  return;
        case 28U: pulsedirectport('D', 4U);  return;
        case 29U: pulsedirectport('D', 5U);  return;
        case 30U: pulsedirectport('D', 6U);  return;
        case 31U: pulsedirectport('D', 7U);  return;        
        case 32U: pulsedirectport('C', 4U);  return;
        case 33U: pulsedirectport('C', 5U);  return;
        case 34U: pulsedirectport('B', 4U);  return;
        case 35U: pulsedirectport('B', 5U);  return;
    }

	/* Relays driven by the shift registers */
	if (relnum < 24U) {
		setShiftRegs(1UL << relnum);
		_delay_ms(20);
		setShiftRegs(0UL);
	}
}

void pulserelaisindex(uint8_t r)
{
	uint8_t rn;

    switch (r)
    { 
        // Cs
        case  0U:  rn =  0U; break;
        case  1U:  rn =  2U; break;
        case  2U:  rn =  4U; break;
        case  3U:  rn =  6U; break;
        case  4U:  rn =  8U; break;
        case  5U:  rn = 34U; break;
        case  6U:  rn = 10U; break;
        case  7U:  rn = 12U; break;

        // Ls
        case  8U:  rn = 14U; break;
        case  9U:  rn = 16U; break;
        case 10U:  rn = 18U; break;
        case 11U:  rn = 20U; break;
        case 12U:  rn = 22U; break;
        case 13U:  rn = 24U; break;
        case 14U:  rn = 26U; break;
        case 15U:  rn = 28U; break;

        // V oder H
        case 16U:  rn = 30U; break;
        case 17U:  rn = 32U; break;

		default:
				   rn =  0U;
    }

    if(relais[r])
        pulserelais(rn);			// Relais anziehen lassen
    else
        pulserelais(rn + 1U);		// Relais abfallen lassen
}

void moveRelais()
{
	uint8_t i;

    for(i = 0U; i < 18U; i++)
    {
        if(oldrelais[i] != relais[i])
        {
            pulserelaisindex(i);
			oldrelais[i] = relais[i];
        }
    }
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
		for(i = 0UL; i < 18UL; i++)
		{
			relais[i] = (shortval & (1UL << i)) ? 1U : 0U;
		}
	}

    // steuere geänderte Relais an
    moveRelais();

	/* Print current state of relays */
	if (isReady) {
		printState();
	}

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
	uint8_t i;

	#ifdef INIT_ON_OFF

	/* Last state flags */
	for(i = 0U; i < 18U; i++) {
		oldrelais[i] = 0U;
	}

	for(i = 1U; i <= 8U; i++)	{
		forceRelais('C', i-1, 1);
	}

	forceRelais('V', 0, 1);
	forceRelais('H', 0, 1);

	for(i = 1U; i <= 8U; i++) {
		forceRelais('L', i-1, 0);
	}

	#endif

	/* Last state flags */
	for(i = 0U; i < 18U; i++) {
		oldrelais[i] = 1U;
	}

	for(i = 1U; i <= 8U; i++)
	{
		forceRelais('C', i-1, 0);
	}

	forceRelais('V', 0, 0);
	forceRelais('H', 0, 0);

	for(i = 1U; i <= 8U; i++)
	{
		forceRelais('L', i-1, 1);
	}


#if 0
	// TODO: remove me!
	while (1) {
		for (i = 0U; i < 18U; i++) {
			relais[i] = 1U;
		}
		moveRelais();
		_delay_ms(100);

		for (i = 0U; i < 18U; i++) {
			relais[i] = 0U;
		}
		moveRelais();
		_delay_ms(100);
	}
#endif
}
