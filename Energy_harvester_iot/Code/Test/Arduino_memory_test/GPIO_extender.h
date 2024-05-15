#ifndef _GPIO_EXTENDER_H_
#define _GPIO_EXTENDER_H_

#include "macros.h"
#include "I2C.h"

#define GPIO_BASE_ADDR 0xE0
#define GPIO_BASE_PORTDIR 0x00  // TODO set right Direction

void GPIO_setDir(u8 mask);

void GPIO_setOutput(u8 mask);

u8 GPIO_readInput();

void GPIO_polarity(u8 mask);


#endif // _GPIO_EXTENDER_H_