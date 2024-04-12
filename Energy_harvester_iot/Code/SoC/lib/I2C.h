#include "macros.h"

#define TWIM_SDA_PIN 
#define TWIM_SCL_PIN

enum TWIM_freqs
{
    freq_100k = 0x01980000,
    freq_250k = 0x04000000,
    freq_400k = 0x06400000
};

void i2c_init(void);

void i2c_write(u8 addr, u8 *data, u8 len);

void i2c_read(u8 addr, u8 *data, u8 len);

