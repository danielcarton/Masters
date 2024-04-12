#include "macros.h"
#include "I2C.h"

#define GPIO_BASE_ADDR 0xE0

void GPIO_setDir(u8 mask);

void GPIO_setOutput(u8 mask);

u8 GPIO_readInput();

void GPIO_polarity(u8 mask);