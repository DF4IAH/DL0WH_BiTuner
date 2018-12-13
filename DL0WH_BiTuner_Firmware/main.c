#include <atmel_start.h>
#include "Tuner/tuner.h"

int main(void)
{
	#if 0
	/* Initializes MCU, drivers and middleware */
	atmel_start_init();

	/* Replace with your application code */
	while (1) {
	}
	#endif
	
	/* Call Tuner */
	tuner_main();
}
