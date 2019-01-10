#include <atmel_start.h>
#include "Tuner/tuner.h"

extern bool isReady;

/* Source code modified by DF4IAH  2019-01-10 */


int main(void)
{
#ifdef OLD_CODE

	/* Call Tuner */
	tuner_main();

#else

	/* Initializes MCU, drivers and middleware */
	atmel_start_init();

    /* Globally enable interrupts */
    cpu_irq_enable();

	/* RESET all C relays, SET all L relays */
    Grundstellung();
	USART_0_enable();

	isReady = true;
	printWelcome();
	printHelp();
	printState();

	/* Work loop */
	while (1) {
		readSerialCommand();
		switchRelais();
	}

#endif
}
