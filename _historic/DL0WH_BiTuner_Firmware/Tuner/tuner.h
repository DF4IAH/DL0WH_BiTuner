#ifndef TUNER_H_
#define TUNER_H_


#include <util/delay.h>

extern char valid;
extern char shortvalid;
extern unsigned long shortval;
extern char command;
extern char parameter;
extern char action;


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


#endif // TUNER_H_
