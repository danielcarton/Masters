#include "memory.h"
#include <stdio.h>


uint8_t memory_buffer[MEMORY_BUFFER_SIZE];       // buffer containting data not yet sent to memory; 
uint16_t block_index = 0;    // index to block within buffer
uint16_t buffer_index = 0;   // index to position within the buffer
uint32_t page_index = 0;     // index to position of page
// Write a block of data to memory. Block size defined in memory.h
void memory_write_block(uint32_t memAddr, uint8_t *data){
    memAddr = memAddr & 0x7FFFF;
    uint8_t addr[3];
    addr[0] = EXT_MEMORY_BASE_ADDR | ( (memAddr & 0x70000) >> 16 );
    addr[1] = ( (memAddr & 0xFF00) >> 8 );
    addr[2] = memAddr & 0xFF;

    uint8_t transmitData[MEMORY_BLOCK_SIZE+2];
    transmitData[0] = addr[1];
    transmitData[1] = addr[2];

    memcpy(transmitData + 2, data, MEMORY_BLOCK_SIZE);

    i2c_write(addr[0], transmitData, MEMORY_BLOCK_SIZE + 2);
}

// Read a block of data from memory. Block size is defined in memory.h
void memory_read_block(uint32_t memAddr, uint8_t *data){
    memAddr = memAddr & 0x7FFFF;
    uint8_t addr[3];
    addr[0] = EXT_MEMORY_BASE_ADDR | ( (memAddr & 0x70000) >> 16 );
    addr[1] = ( (memAddr & 0xFF00) >> 8 );
    addr[2] = memAddr & 0xFF;
    i2c_write(addr[0], addr + 1, 2);
    i2c_read(addr[0], data, MEMORY_BLOCK_SIZE);
}

void memory_add(uint8_t *data, uint8_t len){
    for (uint16_t i = 0; i < len; i++)
    {
        // Increment buffer index, wrap around if exceeding buffer size
        buffer_index = (buffer_index + 1) % MEMORY_BUFFER_SIZE;

        // Write data to buffer
        memory_buffer[buffer_index] = data[i];

        // If the buffer has wrapped around, e.g. buffer_index == 0, increment the page index by the buffer size
        if (buffer_index == 0){
            page_index += MEMORY_BUFFER_SIZE;
        }

        // If the buffer index has exceeded a multiple of the block size, write to external memory.
        if (buffer_index / MEMORY_BLOCK_SIZE != block_index){
            memory_write_block( (page_index * MEMORY_BUFFER_SIZE + buffer_index) - MEMORY_BLOCK_SIZE, *(memory_buffer + block_index*MEMORY_BLOCK_SIZE));
        }

        // Update block index
        block_index = buffer_index / MEMORY_BLOCK_SIZE;
        
    }
    
}