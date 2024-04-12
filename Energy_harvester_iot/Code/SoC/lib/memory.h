#include "macros.h"
#include "I2C.h"
#include <stdio.h>


// base address of memory modules, dont need to 
#define EXT_MEMORY_BASE_ADDR 0xA0
// size of blocks of memory, must be factors of 2 and less than 256 to fit into pages neatly
#define MEMORY_BLOCK_SIZE 32
// size of buffer to contain memory blocks, should be large enough to fit at least two periods of samples and two blocks, 
#define MEMORY_BUFFER_SIZE 128

void memory_write_block(u32 memAddr, u8 *data);

void memory_read_block(u32 memAddr, u8 *data);