#include <stdio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/devicetree.h>

#include "memory.h"
#include "GPIO_extender.h"
#include "I2C_Switch.h"

#define I2C0_EXT_MEMORY DT_NODELABEL(ext_memory)

extern uint8_t gpio_portMask;
extern uint8_t switch_portMask;

struct i2c_dt_spec i2c_dev_ext_memory = I2C_DT_SPEC_GET(I2C0_EXT_MEMORY);

uint8_t memory_buffer[MEMORY_BUFFER_SIZE];       // buffer containting data not yet sent to memory; 
uint16_t buffer_index = 0;   // index to position within the buffer
uint32_t write_address = 0;
uint16_t block_index = 0;

// Write a block of data to memory. Block size defined in memory.h
void memory_write_block(uint32_t memAddr, uint8_t *data, uint16_t startPos){
    memAddr = memAddr & 0x7FFFF;
    i2c_dev_ext_memory.addr |= memAddr >> 16;

    uint8_t addr[2];
    addr[0] = memAddr >> 8;
    addr[1] = memAddr & 0xFF;

    uint8_t transmitData[MEMORY_BLOCK_SIZE+2];
    transmitData[0] = addr[0];
    transmitData[1] = addr[1];
    for (int i = 0; i < MEMORY_BLOCK_SIZE; i++)
    {
        transmitData[i+2] = data[i+startPos];
    }
    
    while(!i2c_is_ready_dt(&i2c_dev_ext_memory)); 
    uint8_t attempts = 0;
    while (i2c_write_dt(&i2c_dev_ext_memory, transmitData, MEMORY_BLOCK_SIZE + 2) == EIO){
        attempts += 1;
        k_msleep(5);
        if(attempts == 20){
            break;
        }
    }
    // i2c_write_dt(&i2c_dev_ext_memory, transmitData, MEMORY_BLOCK_SIZE + 2);
    i2c_dev_ext_memory.addr &= 0xF8;
}

// Read a block of data from memory. Block size is defined in memory.h
void memory_read_block(uint32_t memAddr, uint8_t *data){

    

    // only use 19 bits
    memAddr = memAddr & 0x7FFFF;
    // modify the external memory's device address
    i2c_dev_ext_memory.addr |= memAddr >> 16;

    uint8_t addr[2];
    addr[0] = (memAddr >> 8) & 0xff;
    addr[1] = memAddr & 0xff;
    while(!i2c_is_ready_dt(&i2c_dev_ext_memory)); 
    i2c_write_read_dt(&i2c_dev_ext_memory, addr, 2, data, MEMORY_BLOCK_SIZE);
    i2c_dev_ext_memory.addr &= 0xF8;    // TODO ensure this is right
}

void memory_add(uint8_t *data, uint8_t len){
    switch_portMask |= 0x01;
    gpio_portMask |= 0x01;

    I2C_switch_setChannel(switch_portMask);
    GPIO_setOutput(gpio_portMask);

    for (uint16_t i = 0; i < len; i++)
    {
        // Increment buffer index, wrap around if exceeding buffer size
        buffer_index = (buffer_index + 1) % MEMORY_BUFFER_SIZE;

        // Write data to buffer
        memory_buffer[buffer_index] = data[i];


        // If the buffer index has exceeded a multiple of the block size, write to external memory.
        if (buffer_index / MEMORY_BLOCK_SIZE != block_index){
            memory_write_block(write_address, memory_buffer, block_index*MEMORY_BLOCK_SIZE);
            write_address += MEMORY_BLOCK_SIZE;
        }

        // If the buffer has wrapped around, e.g. buffer_index == 0, increment the page index by the buffer size


        block_index = buffer_index/MEMORY_BLOCK_SIZE;

    }
}

void floatToByteArray(float val, uint8_t *array){
    union convert_u
    {
        float fl;
        uint8_t bytes[4];
    } u;
    u.fl = val;
    for (int i = 0; i < 4; i++)
    {
        array[i] = u.bytes[3 - i];
    }
}