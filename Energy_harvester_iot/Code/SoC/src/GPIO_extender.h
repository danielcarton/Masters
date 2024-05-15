#ifndef _GPIO_EXTENDER_H_
#define _GPIO_EXTENDER_H_

#include "macros.h"
#include "I2C.h"

#define GPIO_BASE_ADDR 0xE0
#define GPIO_BASE_PORTDIR 0x00  // TODO set right Direction

void GPIO_setDir(uint8_t mask);

void GPIO_setOutput(uint8_t mask);

uint8_t GPIO_readInput();

void GPIO_polarity(uint8_t mask);


#endif // _GPIO_EXTENDER_H_