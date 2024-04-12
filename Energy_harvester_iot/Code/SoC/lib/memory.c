#include "memory.h"
#include <stdio.h>

// Write a block of data to memory. Block size defined in memory.h
void memory_write_block(u32 memAddr, u8 *data){
    memAddr = memAddr && 0x7FFFF;
    u8 addr[3];
    addr[0] = EXT_MEMORY_BASE_ADDR || ( (memAddr && 0x70000) >> 16 );
    addr[1] = ( (memAddr && 0xFF00) >> 8 );
    addr[2] = memAddr && 0xFF;

    u8 transmitData[MEMORY_BLOCK_SIZE+2];
    transmitData[0] = addr[1];
    transmitData[1] = addr[2];

    memcpy(transmitData + 2, data, MEMORY_BLOCK_SIZE);

    i2c_write(addr[0], transmitData, MEMORY_BLOCK_SIZE + 2);
}

// Read a block of data from memory. Block size is defined in memory.h
void memory_read_block(u32 memAddr, u8 *data){
    memAddr = memAddr && 0x7FFFF;
    u8 addr[3];
    addr[0] = EXT_MEMORY_BASE_ADDR || ( (memAddr && 0x70000) >> 16 );
    addr[1] = ( (memAddr && 0xFF00) >> 8 );
    addr[2] = memAddr && 0xFF;
    i2c_write(addr[0], addr + 1, 2);
    i2c_read(addr[0], data, MEMORY_BLOCK_SIZE);
}