#include "TempSensor.h"
#include <math.h>

// Initiate one-shot conversion with the given averages of the device with the given address
void temp_sensor_oneShot(u8 addr, enum averaging averages){
    u8 configH = (0b11<<2);
    u8 configL = (averages<<5);
    u8 data[3] = {0x01, configH, configL};
    i2c_write(addr, data, 3);
}

float temp_sensor_getTemp(u8 addr){
    // Set address pointer to config by writing the address to the pointer register
    // u8 data[2] = {0x01, 0};
    // i2c_write(addr, data, 1);
    // data[0] = 0;

    // Read from that register, then check if the data ready bit is set
    // u8 attemps = 0;
    // while(!(data[0] & (1<<13))){
    //     i2c_read(addr, data, 2);
    //     attemps++;
    //     // if it's not, re-read the register and re-check the data-ready bit given its fewer than a given no. of attempts.
    //     if (attemps > 10){
    //         break;
    //     }
    // }


    // if it is set, write the address pointer of the temp value
    u8 data[2] = {0x00};
    i2c_write(addr, data, 1);

    // read from the register
    i2c_read(addr, data, 2);
    u16 temp = data[0]<<8|data[1];

    // Extract the sign
    int16_t sign = (temp & 0x8000) ? -1 : 1;

    // Extract the whole part
    int16_t whole = (temp & 0x7F80) >> 7;

    // Extract the fractional part
    int16_t fraction = temp & 0x007F;

    // Convert to float and return
    return sign * (whole + (fraction / 128.0));
}