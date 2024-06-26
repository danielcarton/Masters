#ifndef _I2C_SWTICH_H_
#define _I2C_SWTICH_H_

#include "macros.h"
#include "I2C.h"

#define I2C_SWITCH_BASE_ADDR 0xEE

void I2C_switch_setChannel(u8 mask);

u8 I2C_switch_readChannels(void);


#endif // !_I2C_SWTICH_H_