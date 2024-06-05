#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/devicetree.h>

#include "I2C_Switch.h"

#define I2C0_I2C_SWITCH DT_NODELABEL(busswitch)
#define GPIO0_SWITCH_EXTENDER DT_NODELABEL(pinextenderreset)

static const struct i2c_dt_spec i2c_dev_bus_switch = I2C_DT_SPEC_GET(I2C0_I2C_SWITCH);
static const struct gpio_dt_spec switch_reset = GPIO_DT_SPEC_GET(GPIO0_SWITCH_EXTENDER, gpios);

void I2C_switch_init(void){
    // sets reset pin high TODO
    if(switch_reset.port){
        gpio_pin_configure_dt(&switch_reset, GPIO_OUTPUT_INACTIVE);
        gpio_pin_set_dt(&switch_reset, 0);
    }
}

// Enables/disables the I2C channels corresponding to the mask, a 1 means the channel is enabled, a 0 means the channel is disabled. Reset = 0x00
void I2C_switch_setChannel(uint8_t mask){
    uint8_t data[1] = {mask};
    while(!i2c_is_ready_dt(&i2c_dev_bus_switch));
    i2c_write_dt(&i2c_dev_bus_switch, data, 1);
}

// Reads the state of all I2C channels. 
uint8_t I2C_switch_readChannels(void){
    uint8_t data[1];
    while(!i2c_is_ready_dt(&i2c_dev_bus_switch)); 
    i2c_read_dt(&i2c_dev_bus_switch, data, 1);
    return data[0];
}