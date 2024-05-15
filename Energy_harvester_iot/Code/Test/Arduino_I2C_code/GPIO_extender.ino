#include "GPIO_extender.h"

// Sets the direction of the GPIO extenders' pins to either inputs or outputs, a 1 means the pin is an input, a 0 means the pin is an output. Reset = 0xFF
void GPIO_setDir(u8 mask){
    u8 data[2] = {0x03, mask};
    i2c_write(GPIO_BASE_ADDR, data, 2);
}

// Sets the value of the pins with an output pin direction. Writing a value to a pin with an input pin direction does nothing. Reset = 0xFF
void GPIO_setOutput(u8 mask){
    u8 data[2] = {0x01, mask};
    i2c_write(GPIO_BASE_ADDR, data, 2);
}

// Reads the state of ALL pins in the extender, regardless of pin direction.
u8 GPIO_readInput(void){
    u8 data[1] = {0x00};
    i2c_write(GPIO_BASE_ADDR, data, 1);
    i2c_read(GPIO_BASE_ADDR, data, 1);
    return data[0];
}

// Sets the pins of the GPIO extender with an input pin direction to have their polarity inverted (e.g. all high inputs are read as low and vice versa). A 1 corresponds to the given pin having an inverted polarity, a 0 means no polarity inversion. Reset = 0x00
void GPIO_polarity(u8 mask){
    
}