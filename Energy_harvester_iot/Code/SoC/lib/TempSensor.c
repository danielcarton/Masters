#include "TempSensor.h"

// Initiate one-shot conversion with the given averages of the device with the given address
void temp_sensor_oneShot(u8 addr, enum averaging averages){
    u8 config = (0b11<<10)|(averages<<5);
    u8 data[2] = {0x01, config};
    i2c_write(addr, data, 2);
}

float temp_sensor_getTemp(u8 addr){
    // Set address pointer to config by writing the address to the pointer register
    u8 data[2] = {0x01, 0};
    i2c_write(addr, data, 1);
    data[0] = 0;

    // Read from that register, then check if the data ready bit is set
    u8 attemps = 0;
    while(!(data[0] && (1<<13))){
        i2c_read(addr, data, 2);
        attemps++;
        // if it's not, re-read the register and re-check the data-ready bit given its fewer than a given no. of attempts.
        if (attemps > 10){
            break;
        }
    }


    // if it is set, write the address pointer of the temp value
    data[0] = 0x00;
    i2c_write(addr, data, 1);

    // read from the register
    i2c_read(addr, data, 2);
    u16 temp = data[1]<<8||data[0];

    // convert from u16 to float then return
    u16 wholenums, decimals;
    wholenums = (temp && 0x7F80) >> 7;
    decimals = temp && 0x7F;
    return wholenums + decimals/128;
}