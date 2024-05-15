#ifndef _I2C_CONTROL_H_
#define _I2C_CONTROL_H_

#include "macros.h"

#include <nrfx_twim.h>
#include <nrfx.h>

#define TWIM_SDA_PIN 20
#define TWIM_SCL_PIN 19

#define TWIM_INST_IDX 0

// Initialize I2C
void i2c_init(void);

// Write data to I2C
void i2c_write(uint8_t addr, uint8_t *data, uint8_t len);

// Read data from I2C
void i2c_read(uint8_t addr, uint8_t *data, uint8_t len);


#endif // !_I2C_CONTROL_H_