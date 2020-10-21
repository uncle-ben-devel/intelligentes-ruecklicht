/* Get all possible data from MPU6050
 * Accelerometer values are given as multiple of the gravity [1g = 9.81 m/s²]
 * Gyro values are given in deg/s
 * Angles are given in degrees
 * Note that X and Y are tilt angles and not pitch/roll.
 *
 * License: MIT
 */

#include "Wire.h"
#include <MPU6050_light.h>

MPU6050 mpu(Wire);

long timer = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  mpu.begin();
  Serial.println(F("Calculating gyro offset, do not move MPU6050"));
  delay(1000);
  mpu.calcGyroOffsets();
  Serial.println("Done!\n");
  
}

void loop() {
  mpu.update();

  if(millis() - timer > 1000){ // print data every second
    Serial.print(F("TEMPERATURE  : "));Serial.println(mpu.getTemp());
    Serial.print(F("ACCELERO   X : "));Serial.print(mpu.getAccX());
    Serial.print("\tY : ");Serial.print(mpu.getAccY());
    Serial.print("\tZ : ");Serial.println(mpu.getAccZ());
  
    Serial.print(F("GYRO       X : "));Serial.print(mpu.getGyroX());
    Serial.print("\tY : ");Serial.print(mpu.getGyroY());
    Serial.print("\tZ : ");Serial.println(mpu.getGyroZ());
  
    Serial.print(F("ACC ANGLE  X : "));Serial.print(mpu.getAccAngleX());
    Serial.print("\tY : ");Serial.println(mpu.getAccAngleY());
    
    Serial.print(F("ANGLE      X : "));Serial.print(mpu.getAngleX());
    Serial.print("\tY : ");Serial.print(mpu.getAngleY());
    Serial.print("\tZ : ");Serial.println(mpu.getAngleZ());
    Serial.println(F("=======================================================\n"));
    timer = millis();
  }


  //########################## I2C Scan ####################################
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for(address = 1; address < 127; address++ ) 
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error==4) 
    {
      Serial.print("Unknown error at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");

  delay(2000);           // wait 2 seconds for next scan

}
