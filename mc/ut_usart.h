
#ifndef __UT_USART_H__
#define __UT_USART_H__

#include "eos_type.h"

#ifdef __cplusplus
extern "C" {
#endif

//#define UT_USART_DEFAULT_TTY "/dev/ttyS1"

//115200, 0, 8, 1, 'N'
int Ut_UsartDefaultInit(char* devName);

//baud, 0, 8, 1, 'N',baud=9600,57600,115200
int Ut_UsartInit(char* devName, unsigned int baud);

//baud, 0, 8, 1, 'N',9600 < baud < 115200
int Ut_UsartSpecialInit(char* devName, unsigned int baud);

int Ut_UsartExit(int devfd);
int Ut_UsartWrite(int devfd, char* pData, unsigned int length);
int Ut_UsartRead(int devfd, char* pData, unsigned int size);

void UtDelayMs(unsigned int msval);
unsigned int UtGetRunTimeMs(void);

#ifdef __cplusplus
}
#endif

#endif

