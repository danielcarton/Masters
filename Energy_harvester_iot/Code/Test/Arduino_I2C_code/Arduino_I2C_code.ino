#include "inc.h"

u8 GPIO_portmask = 0x00;
u8 I2C_switch_portmask = 0x00;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  i2c_init();

  Serial.println("Setting GPIO extender output pin directions");
  Serial.println(" ");
  GPIO_setDir(0x00);

  Serial.println("Setting GPIO extender output pin states");
  Serial.println(" ");
  GPIO_setOutput(0x00);

  Serial.println("Disabling all I2C Switch ports");
  I2C_switch_setChannel(I2C_switch_portmask);
  Serial.println(" ");

}

void loop() {
  // put your main code here, to run repeatedly:
  GPIO_portmask = 1<<1;
  GPIO_setOutput(GPIO_portmask);

  I2C_switch_portmask = 1<<1;
  I2C_switch_setChannel(I2C_switch_portmask);

  Serial.println("Taking a temperature measurement of sensor 1");
  temp_sensor_oneShot(0x94, AVG_64);
  delay(10);
  while(Serial.read() == -1);
  float temp = temp_sensor_getTemp(0x94);
  Serial.print("Result: ");
  Serial.println(temp, 5);
  Serial.println(" ");

  GPIO_portmask = 1<<2;
  GPIO_setOutput(GPIO_portmask);

  I2C_switch_portmask = 1<<2;
  I2C_switch_setChannel(I2C_switch_portmask);

  Serial.println("Taking a temperature measurement of sensor 2");
  temp_sensor_oneShot(0x90, AVG_64);
  delay(10);
  while(Serial.read() == -1);
  temp = temp_sensor_getTemp(0x90);
  Serial.print("Result: ");
  Serial.println(temp, 5);
  Serial.println(" ");

  Serial.println("Taking a temperature measurement of sensor 3");
  temp_sensor_oneShot(0x96, AVG_64);
  delay(10);
  while(Serial.read() == -1);
  temp = temp_sensor_getTemp(0x96);
  Serial.print("Result: ");
  Serial.println(temp, 5);
  Serial.println(" ");

  GPIO_portmask = 1<<3;
  GPIO_setOutput(GPIO_portmask);

  I2C_switch_portmask = 1<<3;
  I2C_switch_setChannel(I2C_switch_portmask);

  Serial.println("Taking a temperature measurement of sensor 4");
  temp_sensor_oneShot(0x92, AVG_64);
  delay(10);
  while(Serial.read() == -1);
  temp = temp_sensor_getTemp(0x92);
  Serial.print("Result: ");
  Serial.println(temp, 5);
  Serial.println(" ");

  Serial.println("Taking a temperature measurement of sensor 5");
  temp_sensor_oneShot(0x90, AVG_64);
  delay(10);
  while(Serial.read() == -1);
  temp = temp_sensor_getTemp(0x90);
  Serial.print("Result: ");
  Serial.println(temp, 5);
  Serial.println(" ");

  Serial.println("Taking a temperature measurement of sensor 6");
  temp_sensor_oneShot(0x96, AVG_64);
  delay(10);
  while(Serial.read() == -1);
  temp = temp_sensor_getTemp(0x96);
  Serial.print("Result: ");
  Serial.println(temp, 5);
  Serial.println(" ");
}
