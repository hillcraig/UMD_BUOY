/*
Author: Reagan Nguyen
This sketch tests the runtimes and different aspects
of the temperature sensors on its own
*/
\

#include <Wire.h>
#include "TSYS01.h"

const int MUX_ADDR = 0x70;
const int TSYS01_ADDRESS = 0x76;
const int WATER_CHANNEL   = 1; //mux water temp channel
const int AIR_CHANNEL = 0; //mux air temp channel
TSYS01 waterSensor;
TSYS01 airSensor;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Wire.begin();

  //intialize sensors
  enableMuxPort(WATER_CHANNEL);
  waterSensor.init();
  disableMuxPort(WATER_CHANNEL);
  enableMuxPort(AIR_CHANNEL);
  airSensor.init();
  disableMuxPort(AIR_CHANNEL);
  delay(3000);


}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("Getting temperature readings...");
  int airTemp = readTemperature(0);

  Serial.print("air temp = ");
  Serial.println(airTemp);

  delay(1000); // wait one sec

  int waterTemp = readTemperature(1);
   Serial.print("water temp = ");
    Serial.println(waterTemp);

  delay(1000); //wait one sec  
}

/*
case read_temp:
      Serial.println(F("Getting temperature readings..."));
      int airAVG[10];
      
      int waterAVG[10];
      
      setAGTWirePullups(1); //not sure if temp needs pull ups

      //take measurements from air temp every 6 seconds for a miniute
      for(int i=0; i<10; ++i){
        enableMuxPort(AIR_CHANNEL);
        airSensor.read();
        airAVG[i]= airSensor.temperature();
        delay(400);
        disableMuxPort(AIR_CHANNEL);
        enableMuxPort(WATER_CHANNEL);
        waterSensor.read();
        waterAVG[i] = waterSensor.temperature();
        delay(400)
        disableMuxPort(WATER_CHANNEL);
        delay(6000);
      }
      //average out temps
      airTemp = average(airAVG);
      waterTemp = average(waterAVG);
      

       loop_step = start_LTC3225; // Move on, start the super capacitor charger

      break; // End of case read_temp
*/
      //Enables a specific port number
boolean enableMuxPort(byte portNumber)
{
  if(portNumber > 7) portNumber = 7;

  //Read the current mux settings
  Wire.requestFrom(MUX_ADDR, 1);
  if(!Wire.available()) return(false); //Error
  byte settings = Wire.read();

  //Set the wanted bit to enable the port
  settings |= (1 << portNumber);
 
  Wire.beginTransmission(MUX_ADDR);
  Wire.write(settings);
  Wire.endTransmission();

  return(true);
}

//Disables a specific port number
boolean disableMuxPort(byte portNumber)
{
  if(portNumber > 7) portNumber = 7;

  //Read the current mux settings
  Wire.requestFrom(MUX_ADDR, 1);
  if(!Wire.available()) return(false); //Error
  byte settings = Wire.read();

  //Clear the wanted bit to disable the port
  settings &= ~(1 << portNumber);

  Wire.beginTransmission(MUX_ADDR);
  Wire.write(settings);
  Wire.endTransmission();

  return(true);
}


int readTemperature(int channel){
  Wire.beginTransmission(MUX_ADDR);// starts transmission to TSYS01

  enableMuxPort(channel); // send a command to initiate the proper channel
  Wire.endTransmission(); //end the transmission

  Wire.beginTransmission(TSYS01_ADDRESS); //start a conversion

  Wire.write(0x48); //send a commmand to initiate temp conversion

  Wire.endTransmission(); //end the transmission

  delay(10); //wait for conversion

  Wire.requestFrom(TSYS01_ADDRESS, 3); //request 3 bytes from sensor
  int temperature = Wire.read() << 8; // read the first byte and shift it left by 8 bits
  temperature |= Wire.read(); // OR the second byte with the first
  Wire.read(); // discard the third byte
  temperature = (temperature / 32768) * 165 - 40; // convert the raw temperature value to degrees Celsius
  return temperature;
}
int getAverage(int temp[]){
        int avg = 0.0;
          for(int i = 0; i<10; ++i){
        avg += (int)temp[i];
      }
        return (avg/10);
      }
