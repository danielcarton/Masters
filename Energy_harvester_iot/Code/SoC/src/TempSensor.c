#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

#include "TempSensor.h"
#include "GPIO_extender.h"
#include "I2C_Switch.h"

#define I2C0_TEMP1 DT_NODELABEL(temp1)
#define I2C0_TEMP2 DT_NODELABEL(temp2)
#define I2C0_TEMP3 DT_NODELABEL(temp3)
#define I2C0_TEMP4 DT_NODELABEL(temp4)
#define I2C0_TEMP5 DT_NODELABEL(temp5)
#define I2C0_TEMP6 DT_NODELABEL(temp6)

static const struct i2c_dt_spec i2c_dev_temp1 = I2C_DT_SPEC_GET(I2C0_TEMP1);
static const struct i2c_dt_spec i2c_dev_temp2 = I2C_DT_SPEC_GET(I2C0_TEMP2);
static const struct i2c_dt_spec i2c_dev_temp3 = I2C_DT_SPEC_GET(I2C0_TEMP3);
static const struct i2c_dt_spec i2c_dev_temp4 = I2C_DT_SPEC_GET(I2C0_TEMP4);
static const struct i2c_dt_spec i2c_dev_temp5 = I2C_DT_SPEC_GET(I2C0_TEMP5);
static const struct i2c_dt_spec i2c_dev_temp6 = I2C_DT_SPEC_GET(I2C0_TEMP6);

LOG_MODULE_REGISTER(Temp_sensors,LOG_LEVEL_DBG);

extern uint8_t gpio_portMask;
extern uint8_t switch_portMask;

// Initiate one-shot conversion with the given averages of the device with the given address
void temp_sensor_oneShot(struct i2c_dt_spec dev, enum averaging averages){
    uint8_t configH = (0b11<<2);
    uint8_t configL = (averages<<5);
    uint8_t data[3] = {0x01, configH, configL};
    while(!i2c_is_ready_dt(&dev)); 
    i2c_write_dt(&dev, data, 3);
}

float temp_sensor_getTemp(struct i2c_dt_spec dev){
    // Read from config register, then check if the data ready bit is set
    uint8_t data[2], attempts = 0;
    data[0] = 0x01;
    while(!i2c_is_ready_dt(&dev)); 
    i2c_write_read_dt(&dev, data, 1, data, 2);
    while(!(data[0] & (1<<5))){
        k_msleep(8);
        data[0] = 0x01;
        while(!i2c_is_ready_dt(&dev)); 
        i2c_write_read_dt(&dev, data, 1, data, 2);
        attempts++;
        // if it's not, re-read the register and re-check the data-ready bit given its fewer than a given no. of attempts.
        if (attempts > 30){
            return -255.0;
        }
    }

    // if it is set, write the address pointer of the temp value
    data[0] = 0x00;
    while(!i2c_is_ready_dt(&dev)); 
    i2c_write_read_dt(&dev, data, 1, data, 2);
    uint16_t temp = data[0]<<8|data[1];

    int16_t sign = (temp & 0x8000) ? -1 : 1;
    int16_t whole = (temp & 0x7F80) >> 7;
    int16_t fraction = temp & 0x007F;

    // Convert to float and return
    return sign * (whole + (fraction / 128.0));
}

void temp_Measure_Read(float *output, enum averaging avgs, uint8_t numSensors){
    switch (numSensors)
    {
    case 1:
        // disable all temp I2C buses
        switch_portMask = switch_portMask & 0b11110001;
        I2C_switch_setChannel(switch_portMask);

        // enable only power group for first temp sensor group
        gpio_portMask = (gpio_portMask & 0b11110001)|(1<<1);
        GPIO_setOutput(gpio_portMask);

        // enable only i2c bus for first temp sensor group 
        switch_portMask |= 1<<1;
        I2C_switch_setChannel(switch_portMask);
        k_msleep(2);

        // take single measurement
        temp_sensor_oneShot(i2c_dev_temp1, avgs);
        
        // read data from measurement
        output[0] = temp_sensor_getTemp(i2c_dev_temp1);
        break;
    
    case 2:
        // disable all temp I2C buses
        switch_portMask = switch_portMask & 0b11110001;
        I2C_switch_setChannel(switch_portMask);

        // enable only power group for first temp sensor group
        gpio_portMask = (gpio_portMask & 0b11110001)|(1<<2);
        GPIO_setOutput(gpio_portMask);

        // enable only i2c bus for first temp sensor group 
        switch_portMask |= 1<<2;
        I2C_switch_setChannel(switch_portMask);
        k_msleep(2);

        // take single measurement
        temp_sensor_oneShot(i2c_dev_temp2, avgs);
        temp_sensor_oneShot(i2c_dev_temp3, avgs);
        
        // read data from measurement
        output[0] = temp_sensor_getTemp(i2c_dev_temp2);
        output[1] = temp_sensor_getTemp(i2c_dev_temp3);
        break;
    
    case 3:
        // disable all temp I2C buses
        switch_portMask = switch_portMask & 0b11110001;
        I2C_switch_setChannel(switch_portMask);

        // enable only power group for first temp sensor group
        gpio_portMask = (gpio_portMask & 0b11110001)|(1<<3);
        GPIO_setOutput(gpio_portMask);

        // enable only i2c bus for first temp sensor group 
        switch_portMask |= 1<<3;
        I2C_switch_setChannel(switch_portMask);
        k_msleep(2);

        // take single measurement
        temp_sensor_oneShot(i2c_dev_temp4, avgs);
        temp_sensor_oneShot(i2c_dev_temp5, avgs);
        temp_sensor_oneShot(i2c_dev_temp6, avgs);
        
        // read data from measurement
        output[0] = temp_sensor_getTemp(i2c_dev_temp4);
        output[1] = temp_sensor_getTemp(i2c_dev_temp5);
        output[2] = temp_sensor_getTemp(i2c_dev_temp6);
        break;
    
    case 4:
        // disable all temp I2C buses
        switch_portMask = switch_portMask & 0b11110001;
        I2C_switch_setChannel(switch_portMask);

        // enable only power group for first temp sensor group
        gpio_portMask = (gpio_portMask & 0b11110001)|(1<<1)|(1<<3);
        GPIO_setOutput(gpio_portMask);

        // enable only i2c bus for first temp sensor group 
        switch_portMask |= (1<<1)|(1<<3);
        I2C_switch_setChannel(switch_portMask);
        k_msleep(2);

        // take single measurement
        temp_sensor_oneShot(i2c_dev_temp1, avgs);
        temp_sensor_oneShot(i2c_dev_temp4, avgs);
        temp_sensor_oneShot(i2c_dev_temp5, avgs);
        temp_sensor_oneShot(i2c_dev_temp6, avgs);
        
        // read data from measurement
        output[0] = temp_sensor_getTemp(i2c_dev_temp1);
        output[1] = temp_sensor_getTemp(i2c_dev_temp4);
        output[2] = temp_sensor_getTemp(i2c_dev_temp5);
        output[3] = temp_sensor_getTemp(i2c_dev_temp6);
        break;
    
    case 5:
        // disable all temp I2C buses
        switch_portMask = switch_portMask & 0b11110001;
        I2C_switch_setChannel(switch_portMask);

        // enable only power group for first temp sensor group
        gpio_portMask = (gpio_portMask & 0b11110001)|(1<<2);
        GPIO_setOutput(gpio_portMask);

        // enable only i2c bus for first temp sensor group 
        switch_portMask |= 1<<2;
        I2C_switch_setChannel(switch_portMask);
        k_msleep(2);

        // take single measurement
        temp_sensor_oneShot(i2c_dev_temp2, avgs);
        temp_sensor_oneShot(i2c_dev_temp3, avgs);
        
        // read data from measurement
        output[0] = temp_sensor_getTemp(i2c_dev_temp2);
        output[1] = temp_sensor_getTemp(i2c_dev_temp3);
        printf("2: %.4f, 3: %.4f\n\r", output[0], output[1]);

        // disable all temp I2C buses
        switch_portMask = switch_portMask & 0b11110001;
        I2C_switch_setChannel(switch_portMask);

        // enable only power group for first temp sensor group
        gpio_portMask = (gpio_portMask & 0b11110001)|(1<<3);
        GPIO_setOutput(gpio_portMask);

        // enable only i2c bus for first temp sensor group 
        switch_portMask |= 1<<3;
        I2C_switch_setChannel(switch_portMask);
        k_msleep(2);

        // take single measurement
        temp_sensor_oneShot(i2c_dev_temp4, avgs);
        temp_sensor_oneShot(i2c_dev_temp5, avgs);
        temp_sensor_oneShot(i2c_dev_temp6, avgs);
        
        // read data from measurement
        output[2] = temp_sensor_getTemp(i2c_dev_temp4);
        output[3] = temp_sensor_getTemp(i2c_dev_temp5);
        output[4] = temp_sensor_getTemp(i2c_dev_temp6);
        break;

    case 6:
        // disable all temp I2C buses
        switch_portMask = switch_portMask & 0b11110001;
        I2C_switch_setChannel(switch_portMask);

        // enable only power group for first temp sensor group
        gpio_portMask = (gpio_portMask & 0b11110001)|(1<<2)|(1<<1);
        GPIO_setOutput(gpio_portMask);

        // enable only i2c bus for first temp sensor group 
        switch_portMask |= (1<<2)|(1<<1);
        I2C_switch_setChannel(switch_portMask);
        k_msleep(2);

        // take single measurement
        temp_sensor_oneShot(i2c_dev_temp1, avgs);
        temp_sensor_oneShot(i2c_dev_temp2, avgs);
        temp_sensor_oneShot(i2c_dev_temp3, avgs);
        
        // read data from measurement
        output[0] = temp_sensor_getTemp(i2c_dev_temp1);
        output[1] = temp_sensor_getTemp(i2c_dev_temp2);
        output[2] = temp_sensor_getTemp(i2c_dev_temp3);

        // disable all temp I2C buses
        switch_portMask = switch_portMask & 0b11110001;
        I2C_switch_setChannel(switch_portMask);

        // enable only power group for first temp sensor group
        gpio_portMask = (gpio_portMask & 0b11110001)|(1<<3);
        GPIO_setOutput(gpio_portMask);

        // enable only i2c bus for first temp sensor group 
        switch_portMask |= 1<<3;
        I2C_switch_setChannel(switch_portMask);
        k_msleep(2);

        // take single measurement
        temp_sensor_oneShot(i2c_dev_temp4, avgs);
        temp_sensor_oneShot(i2c_dev_temp5, avgs);
        temp_sensor_oneShot(i2c_dev_temp6, avgs);
        
        // read data from measurement
        output[3] = temp_sensor_getTemp(i2c_dev_temp4);
        output[4] = temp_sensor_getTemp(i2c_dev_temp5);
        output[5] = temp_sensor_getTemp(i2c_dev_temp6);
        break;

    default:
        break;
    }
}