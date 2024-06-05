#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>

#include "GPIO_extender.h"

#define I2C0_GPIO_EXTENDER DT_NODELABEL(pinextender)
#define GPIO0_GPIO_EXTENDER DT_NODELABEL(pinextenderreset)

static const struct i2c_dt_spec i2c_dev_GPIO_ext = I2C_DT_SPEC_GET(I2C0_GPIO_EXTENDER);
static const struct gpio_dt_spec GPIO_reset = GPIO_DT_SPEC_GET(GPIO0_GPIO_EXTENDER, gpios);

void GPIO_init(void){
    // sets reset pin high TODO
    if(GPIO_reset.port){
        gpio_pin_configure_dt(&GPIO_reset, GPIO_OUTPUT_INACTIVE);
        gpio_pin_set_dt(&GPIO_reset, 0);
    }
}   

// Sets the direction of the GPIO extenders' pins to either inputs or outputs, a 1 means the pin is an input, a 0 means the pin is an output. Reset = 0xFF
void GPIO_setDir(uint8_t mask){
    uint8_t data[2] = {0x03, mask};
    while(!i2c_is_ready_dt(&i2c_dev_GPIO_ext));
    i2c_write_dt(&i2c_dev_GPIO_ext, data, 2);
}

// Sets the value of the pins with an output pin direction. Writing a value to a pin with an input pin direction does nothing. Reset = 0xFF
void GPIO_setOutput(uint8_t mask){
    uint8_t data[2] = {0x01, mask};
    while(!i2c_is_ready_dt(&i2c_dev_GPIO_ext));
    i2c_write_dt(&i2c_dev_GPIO_ext, data, 2);
}

// Reads the state of ALL pins in the extender, regardless of pin direction.
uint8_t GPIO_readInput(void){
    uint8_t data[1] = {0x00};
    while(!i2c_is_ready_dt(&i2c_dev_GPIO_ext));
    i2c_write_dt(&i2c_dev_GPIO_ext, data, 1);
    while(!i2c_is_ready_dt(&i2c_dev_GPIO_ext));
    i2c_read_dt(&i2c_dev_GPIO_ext, data, 1);
    return data[0];
}

// Sets the pins of the GPIO extender with an input pin direction to have their polarity inverted (e.g. all high inputs are read as low and vice versa). A 1 corresponds to the given pin having an inverted polarity, a 0 means no polarity inversion. Reset = 0x00
void GPIO_polarity(uint8_t mask){
    
}