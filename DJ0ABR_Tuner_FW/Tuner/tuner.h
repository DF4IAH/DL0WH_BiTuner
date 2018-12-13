#include <util/delay.h>

void portSet(uint16_t port, char bitPos);
void portClr(uint16_t port, char bitPos);
void readSerialCommand();
char dataAvailable();
char tuner_getchar(void);
void switchRelais();
void Grundstellung();
void pulsedirectport(char port, unsigned char portnum);
void pulserelais(char relnum);
void tuner_main(void);

extern char valid;
extern char shortvalid;
extern unsigned long shortval;
extern char command;
extern char parameter;
extern char action;
