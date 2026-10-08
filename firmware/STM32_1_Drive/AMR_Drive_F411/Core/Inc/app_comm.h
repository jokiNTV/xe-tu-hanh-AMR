#ifndef APP_COMM_H
#define APP_COMM_H

#include <stdint.h>

void AppComm_Run(void);

uint32_t AppComm_GetLastHostActivityMs(void);
uint8_t AppComm_GetMode(void);

#endif
