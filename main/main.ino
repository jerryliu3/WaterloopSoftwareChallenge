//Library Imports
#include <Wire.h>
#include <math.h>

const int slave_address = 0x5A; // I2C address

//Variables needed for signal reading and analysis
long data;
long first, second, third, fourth, fifth, sixth, seventh, eighth, ninth, tenth, last;
float acceleration [10];
float brakeTemp [10];
float propTemp [10];
float boardTemp [10];
float timeReading;
int state;
byte readings[32];
int index = 0;
//Arduino setup
void setup() {
  Serial.begin(9600);
  Wire.begin(1);
  Wire.beginTransmission(slave_address);
  Wire.write(0x56);  // start byte
  Wire.write(0x00);     // make idle
  Wire.write(0x23);
  Wire.endTransmission(true);
  Wire.beginTransmission(slave_address);
  Wire.write(0x56);  // start byte
  Wire.write(0x01);     // make ready
  Wire.write(0x23);
  Wire.endTransmission(true);
  //start acceleration
  Wire.beginTransmission(slave_address);
  Wire.write(0x56);  // start byte
  Wire.write(0x02);     // make ready
  Wire.write(0x23);
  Wire.endTransmission(true);
}

//Continuous loop
void loop() {
  index=0;
  Wire.requestFrom(slave_address, 32, true);
  while (Wire.available())
  {
    readings[index] = Wire.read();
    //Serial.print(readings[index] & 0xFF);
    //Serial.print("\t");
    index++;
  }
  state = readings[2];
  Serial.print(state);
  Serial.print(" ");
  byte data[4];
  data[0] = readings[3];
  data[1] = readings[4];
  data[2] = readings[5];
  data[3] = readings[6];
  timeReading = *((float*)(data));  
  data[0] = readings[7];
  data[1] = readings[8];
  data[2] = readings[9];
  data[3] = readings[10];
  float acc1 = *((float*)(data));
  data[0] = readings[11];
  data[1] = readings[12];
  data[2] = readings[13];
  data[3] = readings[14];
  float acc2 = *((float*)(data));
  data[0] = readings[15];
  data[1] = readings[16];
  data[2] = readings[17];
  data[3] = readings[18];
  float acc3 = *((float*)(data));
  float accAvg = (acc1 + acc2 + acc3)/3;
  data[0] = readings[19];
  data[1] = readings[20];
  data[2] = readings[21];
  data[3] = readings[22];
  float newBrakeTemp = *((float*)(data));
  data[0] = readings[23];
  data[1] = readings[24];
  data[2] = readings[25];
  data[3] = readings[26];
  float newPropTemp = *((float*)(data));  
  data[0] = readings[27];
  data[1] = readings[28];
  data[2] = readings[29];
  data[3] = readings[30];
  float newBoardTemp = *((float*)(data));  
  Serial.print(timeReading);
  Serial.print(" ");
  Serial.print(accAvg);
  Serial.print(" ");
  Serial.print(newBrakeTemp);
  Serial.print(" ");  
  Serial.print(newPropTemp);
  Serial.print(" ");
  Serial.println(newBoardTemp);
    memcpy(acceleration, &acceleration[1], sizeof(acceleration) - sizeof(int));
    acceleration[9] =  accAvg;
    memcpy(brakeTemp, &brakeTemp[1], sizeof(brakeTemp) - sizeof(int));
    brakeTemp[9] =  newBrakeTemp;
    memcpy(propTemp, &propTemp[1], sizeof(propTemp) - sizeof(int));
    propTemp[9] =  newPropTemp;
    memcpy(boardTemp, &boardTemp[1], sizeof(boardTemp) - sizeof(int));
    boardTemp[9] =  newBoardTemp;  

  //get all three acceleration
  //get average acceleration
  //calculate velocity
  //calculate distance
  
  delay(20);
}
