#ifndef _GPIO_EXTENDER_H_
#define _GPIO_EXTENDER_H_

#define GPIO_BASE_PORTDIR 0x00 

void GPIO_init(void);

void GPIO_setDir(uint8_t mask);

void GPIO_setOutput(uint8_t mask);

uint8_t GPIO_readInput();

void GPIO_polarity(uint8_t mask);


#endif // _GPIO_EXTENDER_H_