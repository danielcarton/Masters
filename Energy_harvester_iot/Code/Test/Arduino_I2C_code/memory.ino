#include "memory.h"
#include <stdio.h>


u8 memory_buffer[MEMORY_BUFFER_SIZE];       // buffer containting data not yet sent to memory; 
u16 block_index = 0;    // index to block within buffer
u16 buffer_index = 0;   // index to position within the buffer
u32 page_index = 0;     // index to position of page
// Write a block of data to memory. Block size defined in memory.h
void memory_write_block(u32 memAddr, u8 *data){
    memAddr = memAddr & 0x7FFFF;
    u8 addr[3];
    addr[0] = EXT_MEMORY_BASE_ADDR | ( (memAddr & 0x70000) >> 16 );
    addr[1] = ( (memAddr & 0xFF00) >> 8 );
    addr[2] = memAddr & 0xFF;

    u8 transmitData[MEMORY_BLOCK_SIZE+2];
    transmitData[0] = addr[1];
    transmitData[1] = addr[2];

    memcpy(transmitData + 2, data, MEMORY_BLOCK_SIZE);

    i2c_write(addr[0], transmitData, MEMORY_BLOCK_SIZE + 2);
}

// Read a block of data from memory. Block size is defined in memory.h
void memory_read_block(u32 memAddr, u8 *data){
    memAddr = memAddr & 0x7FFFF;
    u8 addr[3];
    addr[0] = EXT_MEMORY_BASE_ADDR | ( (memAddr & 0x70000) >> 16 );
    addr[1] = ( (memAddr & 0xFF00) >> 8 );
    addr[2] = memAddr & 0xFF;
    i2c_write(addr[0], addr + 1, 2);
    i2c_read(addr[0], data, MEMORY_BLOCK_SIZE);
}

void memory_add(u8 *data, u8 len){
    for (u16 i = 0; i < len; i++)
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