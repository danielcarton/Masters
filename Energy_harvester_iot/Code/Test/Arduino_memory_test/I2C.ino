#include "I2C.h"
#include <Wire.h>


void i2c_init(void){
  Wire.begin();
  Wire.setClock(400000);

}

// Write data to I2C
void i2c_write(u8 addr, u8 *data, u8 len){
  Wire.beginTransmission(addr>>1);
  for(int i = 0; i < len; i++){
    Wire.write(*(data + i));
  }
  Wire.endTransmission();
}

// Read data from I2C
void i2c_read(u8 addr, u8 *data, u8 len){
  Wire.requestFrom(addr>>1, len);
  for (int i = 0; i < len; i++){
    if (Wire.available()){
      data[i] = Wire.read();
    }
  }
}
