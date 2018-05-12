#include <ArduinoJson.h>
#include <SimpleKalmanFilter.h>
#include <Wire.h>
#include <math.h>
#include <Filters.h>

const int slave_address = 0x5A; // I2C address

//Moving average
const int RunningAverageCount = 3;
float RunningAverageBuffer[RunningAverageCount];
int NextRunningAverage;

//Variables needed for signal reading and analysis
long data;
float acceleration [10];
float brakeTemp [10];
float propTemp [10];
float boardTemp [10];
float timeReading;
int state;
byte readings[32];
int index = 0;

//filters out changes faster that 5 Hz.
float filterFrequency = 1.0;

// create a one pole (RC) lowpass filter
FilterOnePole lowpassFilter( LOWPASS, filterFrequency );   
    
float oldAcc = 0;
float newAcc = 0;
float oldVel = 0;
float newVel = 0;
float newDist = 0;
float time_since_start_new = 0;
float time_since_start_old = 0;
const float totalDistance = 30000;
bool breaksToggled = false;
float oldTime = 0;
float oldPrintTime = 0;

void setup() {
  // put your setup code here, to run once:
  // Initialize Serial port
  Serial.begin(9600);
  while (!Serial) continue; //wait for serial port to connect. 
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
  Wire.write(0x02);     // make accelerate
  Wire.write(0x23);
  Wire.endTransmission(true);
}

void loop() {
  if (millis() - oldTime > 20){
    index=0;
    Wire.requestFrom(slave_address, 32, true);
    while (Wire.available())
    {
      readings[index] = Wire.read();
      index++;
    }
    state = readings[2];
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
     
    //Shift values over
    memcpy(acceleration, &acceleration[1], sizeof(acceleration) - sizeof(int));
    acceleration[9] =  accAvg;
    memcpy(brakeTemp, &brakeTemp[1], sizeof(brakeTemp) - sizeof(int));
    brakeTemp[9] =  newBrakeTemp;
    memcpy(propTemp, &propTemp[1], sizeof(propTemp) - sizeof(int));
    propTemp[9] =  newPropTemp;
    memcpy(boardTemp, &boardTemp[1], sizeof(boardTemp) - sizeof(int));
    boardTemp[9] =  newBoardTemp;  
    
    time_since_start_new = timeReading;
    newAcc = acceleration[9];
    lowpassFilter.input(newAcc);
    float newAcc2 = newAcc;
    newAcc = lowpassFilter.output();
    
    float dT= time_since_start_new-time_since_start_old;
    //Integrate for the velocity
    newVel += (newAcc-oldAcc)*(dT)/2 + oldAcc * dT; //Add a triangle on top for better approx.     
    //Integrate for distance
    newDist += (newVel-oldVel)*(dT)/2 + oldVel * dT;
    oldAcc = newAcc;
    oldVel = newVel;
    time_since_start_old = time_since_start_new;
    //Consider applying high pass filter. 
  
    //Brake if 5000 km left.
    if (!breaksToggled && totalDistance - newDist < 3600){
      Wire.beginTransmission(slave_address);
      Wire.write(0x56);  // start byte
      Wire.write(0x04);     // make brake
      Wire.write(0x23);
      Wire.endTransmission(true);
      breaksToggled = true;
    }
  
//    const size_t bufferSize = JSON_ARRAY_SIZE(8) + JSON_OBJECT_SIZE(1) + 70;
//    DynamicJsonBuffer jsonBuffer(bufferSize);
//  
//  
//    JsonObject& root = jsonBuffer.createObject();
//    JsonArray& data2 = root.createNestedArray("data");
//    data2.add(newVel);
//    data2.add(newDist);
//    data2.add(newAcc);
//    data2.add(newPropTemp);
//    data2.add(newBrakeTemp);
//    data2.add(newBoardTemp);
//    data2.add(state);
//    data2.add(timeReading);
    //String output;
    //root.printTo(output); //print as a string. May prevent errors with parsing. 
    //Serial.println(output);

    //Byte array. serial.write(Arr)
    //byte valueArr[] = {newVel, newDist, newAcc, newPropTemp, newBrakeTemp, newBoardTemp, state, timeReading};
    oldTime = millis();
    
  }
  if(millis() - oldPrintTime >= 1000){
    Serial.print(state);
    Serial.print("    ");
    Serial.print(newVel);
    Serial.print("    ");
    Serial.print(newDist);
    Serial.println();
    oldPrintTime = millis();
  }

}

