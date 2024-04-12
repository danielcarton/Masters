#include "I2C.h"
#include "macros.h"

// base address of all the sensors regardless of address pin. Sensors in SoC are grouped as: {0x92}, {0x90, 0x96}, {0x90, 0x92, 0x96}
#define TEMP_SENSOR_BASE_ADDR0x90

// defines the number of samples averaged before the sensor gives the result.
// Delays:
// AVG_1  = 15.5ms
// AVG_8  = 125ms
// AVG_32 = 500ms
// AVG_64 = 1s
enum averaging {AVG_1 = 0b00, AVG_8 = 0b01, AVG_32 = 0b10, AVG_64 = 0b11};

void temp_sensor_oneShot(u8 addr, enum averaging averages);

float temp_sensor_getTemp(u8 addr);