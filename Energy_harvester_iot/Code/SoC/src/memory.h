#ifndef _MEMORY_H_
#define _MEMORY_H_

#include <stdio.h>


// base address of memory modules, dont need to 
#define EXT_MEMORY_BASE_ADDR 0xA0
// size of blocks of memory, must be factors of 2 and less than 256 to fit into pages neatly
#define MEMORY_BLOCK_SIZE 32
// size of buffer to contain memory blocks, should be large enough to fit at least two periods of samples and two blocks, 
#define MEMORY_BUFFER_SIZE 256
#define MEMORY_PAGE_SIZE 256

void memory_write_block(uint32_t memAddr, uint8_t *data, uint16_t startPos);

void memory_read_block(uint32_t memAddr, uint8_t *data);

void memory_add(uint8_t *data, uint8_t len);

void floatToByteArray(float val, uint8_t *array);


#endif // !_MEMORY_H_