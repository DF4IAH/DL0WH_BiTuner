#ifndef TUNER_H_
#define TUNER_H_


#include <util/delay.h>

extern char valid;
extern char shortvalid;
extern unsigned long shortval;
extern char command;
extern char parameter;
extern char action;


#ifdef OLD_CODE
void portSet(uint16_t port, char bitPos);
void portClr(uint16_t port, char bitPos);
#endif

void readSerialCommand();
void printString(const char* str);
void printCrLf(void);
void printState(void);
void printWelcome(void);
void printHelp(void);
char dataAvailable();
char tuner_getchar(void);
void switchRelais();
void Grundstellung();
void pulsedirectport(char port, uint8_t portnum);
void pulserelais(uint8_t relnum);

#ifdef OLD_CODE
void tuner_main(void);
#endif

#endif // TUNER_H_
