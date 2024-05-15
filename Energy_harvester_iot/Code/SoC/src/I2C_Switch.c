#include "I2C_Switch.h"

// Enables/disables the I2C channels corresponding to the mask, a 1 means the channel is enabled, a 0 means the channel is disabled. Reset = 0x00
void I2C_switch_setChannel(uint8_t mask){
    uint8_t data[1] = {mask};
    i2c_write(I2C_SWITCH_BASE_ADDR, data, 1);
}

// Reads the state of all I2C channels. 
uint8_t I2C_switch_readChannels(void){
    uint8_t data[1];
    i2c_read(I2C_SWITCH_BASE_ADDR, data, 1);
    return data[0];
}