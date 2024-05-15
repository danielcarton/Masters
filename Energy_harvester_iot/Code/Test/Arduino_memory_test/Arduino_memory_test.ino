#include "inc.h"
#include <Wire.h>

#define BYTES_TO_STORE 16

u8 data[MEMORY_BUFFER_SIZE];
u32 STARTADDR = 1<<17;

void setup() {
  // put your setup code here, to run once
  Serial.begin(9600);
  Wire.begin();
  Wire.setClock(400000);

  u8 data[2] = {0x03, 0x00};
  Wire.beginTransmission(GPIO_BASE_ADDR>>1);
  for(int i = 0; i < 2; i++){
    Wire.write(*(data + i));
  }
  Wire.endTransmission();

  data[0] = 0x01;
  data[1] = 0x01;
  Wire.beginTransmission(GPIO_BASE_ADDR>>1);
  for(int i = 0; i < 2; i++){
    Wire.write(*(data + i));
  }
  Wire.endTransmission();

  Wire.beginTransmission(I2C_SWITCH_BASE_ADDR>>1);
  Wire.write(0x01);
  Wire.endTransmission();

}

void loop() {
  Serial.println("Press to write to memory");
  while(Serial.read() == -1);
  for(int i = 0; i < BYTES_TO_STORE; i++){
     data[i] = i;
  }

  Wire.beginTransmission((u8)(EXT_MEMORY_BASE_ADDR >> 1| 0x2));
  // Wire.beginTransmission(EXT_MEMORY_BASE_ADDR >> 1);
  Wire.write((STARTADDR & (0xFF<< 8)) >> 8);
  Wire.write((u8)(STARTADDR & 0xFF));
  for (int i = 0; i < BYTES_TO_STORE; i++) {
    Wire.write(data[i]);
  }
  Wire.endTransmission();

  Serial.println("Press to read from memory");
  while(Serial.read() == -1);
  Wire.beginTransmission((u8)(EXT_MEMORY_BASE_ADDR >> 1| 0x2));
  Wire.write((STARTADDR & (0xFF<< 8)) >> 8);
  Wire.write(STARTADDR & 0xFF);
  Wire.endTransmission();

  u8 recievedData[BYTES_TO_STORE];
  Wire.requestFrom((u8)(EXT_MEMORY_BASE_ADDR >> 1| 0x2), BYTES_TO_STORE);
  for (int i = 0; i < BYTES_TO_STORE; i++) {
    recievedData[i] = Wire.read();
    Serial.print(recievedData[i], HEX);
    Serial.print(" ");
    if(i % 4 == 0){
      Serial.println(" ");
    }
  }
  Serial.println("Write complete, comparing result");
  for (int i = 0; i < BYTES_TO_STORE; i++) {
    if(recievedData[i] != i){
      Serial.print("Write error at address: ");
      Serial.println(i + STARTADDR, HEX);
    }
  }
  STARTADDR += BYTES_TO_STORE;
}
