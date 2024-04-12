#include "I2C.h"

// Initialize I2C
void i2c_init(void){
    // Enable TWIM by writing 0x06 to ENABLE register
    // Set the SCL and SDA pins by writing to the PSEL registers
    // set the freq using the TWIM_freqs enum in the FREQUENCY register

}

// Write data to I2C
void i2c_write(u8 addr, u8 *data, u8 len){
    // Set the value of TXD.PTR to equal the pointer value *data
    // Set the number of bytes to transmit (including address, basically len + 1) to TXD.MAXCNT
    // write the device address to ADDRESS  register (maybe right-shifted by 1?)
}

// Read data from I2C
void i2c_read(u8 addr, u8 *data, u8 len){
    // set the value of RXD.PTR to equal the pointer value of *data
    // set the number of bytes to read to RXD.MAXCNT
    // write the device address to ADDRESS  register (maybe right-shifted ny 1?)
}
