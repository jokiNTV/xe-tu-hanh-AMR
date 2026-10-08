#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <stdint.h>

void Communication_Init(void);
void Communication_Task(void);

/* Được gọi từ USB CDC Receive callback */
void Communication_USBReceive(uint8_t *data, uint32_t len);

/* Reset only the USB command parser/session state.
 * Safe to call when CDC host opens/closes the virtual COM port.
 * Does not change mechanism state, fault latch, or motor outputs.
 */
void Communication_USBSessionReset(void);

#endif
