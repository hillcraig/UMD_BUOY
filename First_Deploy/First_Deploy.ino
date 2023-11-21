/*
 *  LTE_Buoy.ino
 *  Liam Gaeuman @ University of Minnesota Duluth
 *  5/9/23
 */

#include "Notecard.h" 
#include "TSYS01.h" 
#include "SparkFun_MS5803_I2C.h" 
#include "ICM_20948.h" 
#include "Ezo_i2c.h" //include the EZO I2C library from https://github.com/Atlas-Scientific/Ezo_I2c_lib
//#include "WaveProcessor.h"
#include <time.h> 
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Servo.h>

#define serialDebug Serial
#define productUID "com.gmail.liamgaeuman:environmental_monitoring_buoy" 
#define NDEBUG 1

#ifndef ARDUINO_SWAN_R5
#error "This program was designed to run on the Blues Wireless Swan"
#endif

//- - - - - declare sensors - - - - - - - - - 
Notecard notecard;            //notecard object
TSYS01 airTempSensor;         //sesnor used to collect air temperature
TSYS01 waterTempSensor;       //sesnor used to collect water temperature
ICM_20948_I2C myICM;          //sensor for motion 
//Ezo_board EC = Ezo_board(100, "EC");
//Ezo_board PH = Ezo_board(99, "PH");
//Ezo_board DO = Ezo_board(98, "DO");
//- - - - - - other variables - - - - - - - - 
const int chipSelect = A5;     //selector for SD
const byte indicatorPin = A4;  //selecter for LED                 
File myFile;
size_t mainLoopTime;           //stores time for main loop
int IMU_SAMPLE_SIZE = 4608;
int array_index;

//- - - - - init data fields - - - - - - - - - 
char yyyy[5][2];   //year   yyyy
char mM[3][2];     //month  mm
char dd[3][2];     //day    dd
char hh[3][2];     //hour   hh
char mm[3][2];     //minute mm
char ss[3][2];     //second ss
double lat[2];     //lat    ##.#####
double lon[2];     //lon    ##.#####
int sats[2];       //number of GPS satellites
float airTemp[2];        
float waterTemp[2];        
//float pH;              
//float conductivity;    
//float dissolvedOxygen;

// - - - - set ports for multiplexer - - - - -
const int MUX_ADDR = 0x70;
const int IMU_PORT = 0;
const int TSYS01_AIR_PORT = 1;
const int TSYS01_WATER_PORT = 4;

void setup() {
  int error = 0;
  delay(5000);

  pinMode(indicatorPin, OUTPUT); //setup LED
  serialDebug.begin(115200); 
  notecard.begin();
  Wire.begin();

  configureSensors();                  
  error += setUpSD(); 

  if (error < 0) 
    error += setUpSD();

  if (error < 0) 
    error += setUpSD();

  #if NDEBUG
    notecard.setDebugOutputStream(serialDebug);
  #endif
 
  J *req1 = notecard.newRequest("hub.set");          //init JSON object for request
  JAddStringToObject(req1, "product", productUID);   //set productUID
  JAddStringToObject(req1, "mode", "periodic");      //set mode to a continuous sensor
  if (!notecard.sendRequest(req1)) {                 //send off request for connection
    notecard.logDebug("FATAL: Failed to configure Notecard!\n");
    JDelete(req1);
    error -= 1; 
  }

  J *req2 = notecard.newRequest("card.location.mode");
  JAddStringToObject(req2, "mode", "periodic");         //set gps mode to periodic 
  if (!notecard.sendRequest(req2)) {
    JDelete(req2);
    error -= 1; 
  }

  if(error < 0){
    serialDebug.println("error in setup");
    blinkLED(-1);
    delay(500);
    blinkLED(1); //state 1
  }
}

/*
 * time: 30 minutes
 */
void loop() { 
  mainLoopTime = millis();
  serialDebug.println("main loop start"); 
  array_index = 0;

  serialDebug.println("obtaing GPS Connectin");
  blinkLED(100);
  if(connectToGPS() < 0){
    serialDebug.println("error obtaining GPS connection");
    blinkLED(-1);
    delay(500);
    blinkLED(2); //state 2
  }

  blinkLED(100);
  serialDebug.println("collecting data");
  if(collectData() < 0){
    serialDebug.println("error collecting data");
    blinkLED(-1);
    delay(500);
    blinkLED(3); //state 3
  }

  blinkLED(100);
  serialDebug.println("collecting IMU data");
  if(sample_IMU() < 0){
    serialDebug.println("error sampling IMU");
    blinkLED(-1);
    delay(500);
    blinkLED(4); //state 4
  }

  array_index = 1;
 
  blinkLED(100);
  serialDebug.println("obtaing 2nd GPS connectin");
  if(connectToGPS() < 0){
    serialDebug.println("error obtaining 2nd GPS connection");
    blinkLED(-1);
    delay(500);
    blinkLED(5); //state 5
  }

  blinkLED(100);
  serialDebug.println("collecting data");
  if(collectData() < 0){
    serialDebug.println("error collecting data");
    blinkLED(-1);
    delay(500);
    blinkLED(6); //state 6
  }

  blinkLED(100);
  serialDebug.println("sending data");
  if(sendData() < 0){
    serialDebug.println("error sending data");
    blinkLED(-1);
    delay(500);
    blinkLED(7); //state 7
  }

  blinkLED(100);
  serialDebug.println("writing to SD");
  if(writeToMainFile() < 0){
    serialDebug.println("error writing to SD");
    blinkLED(-1);
    delay(500);
    blinkLED(8); //state 8
  }
    
  while(millis() < mainLoopTime + 30 * 60000); //must take exactly 30 minutes before it restarts
}

void blinkLED(int x){
  int i;
  if(x == 100){
    for(i = 0; i < 10; i++){
      digitalWrite(indicatorPin, HIGH);  
      delay(100);
      digitalWrite(indicatorPin, LOW);
      delay(100);
    }
  }
  else if (x >= 0 ){
    for(i=0; i < x; i++){
      digitalWrite(indicatorPin, HIGH);  
      delay(500);
      digitalWrite(indicatorPin, LOW);
      delay(250);
    }
  }
  else{
    digitalWrite(indicatorPin, HIGH);  
    delay(2000);
    digitalWrite(indicatorPin, LOW);
  }
}

int collectData(){
    getAirTemp();
    getWaterTemp();
    //get_pH();
    //getConductivity();
    //getDissolvedOxygen();
    return 0;
}

int connectToGPS(){
  
  // Save the time from the last location reading.
  J *rsp = notecard.requestAndResponse(notecard.newRequest("card.location"));
  size_t gps_time_s = JGetInt(rsp, "time");
  notecard.deleteResponse(rsp);
  
  // Set the location mode to "continuous" mode to force the
  // Notecard to take an immediate GPS/GNSS reading.
  J *req = notecard.newRequest("card.location.mode");
  JAddStringToObject(req, "mode", "continuous");
  notecard.sendRequest(req);

  // How many seconds to wait for a location before you stop looking
  size_t timeout_s = 150; //2.5 min

  // Block while resolving GPS/GNSS location
  for (const size_t start_ms = ::millis();;) {
    // Check for a timeout, and if enough time has passed, break out of the loop
    // to avoid looping forever    
    if (::millis() >= (start_ms + (timeout_s * 1000))) {
      serialDebug.println("Timed out looking for a location\n");
      //what to do if location can't be found
      break;
    }
  
    // Check if GPS/GNSS has acquired location information
    J *rsp = notecard.requestAndResponse(notecard.newRequest("card.location"));
    if (JGetInt(rsp, "time") != gps_time_s) {   //once updated, location info will have a new time
      setLocationVars(rsp);        
      notecard.deleteResponse(rsp);
      setTimeVars();

      // Restore previous configuration
      J *req = notecard.newRequest("card.location.mode");
      JAddStringToObject(req, "mode", "periodic");
      notecard.sendRequest(req);
      serialDebug.println("GPS connection succesful :)\n");
      return 0; //gps connection success
    }
  
    // If a "stop" field is on the card.location response, it means the Notecard
    // cannot locate a GPS/GNSS signal, so we break out of the loop to avoid looping
    // endlessly
    if (JGetObjectItem(rsp, "stop")) {
      notecard.deleteResponse(rsp);
      serialDebug.println("Found a stop flag, cannot find location\n");
      break;
    }
    // Wait 2 seconds before tryring again 
    delay(2000);
  }

  noGPS();
  setTimeVars();
  serialDebug.println("GPS connection unsuccesful :(\n");
  return -1; //gps connection failed
}

int sendData(){
  //send to cloud
  J *req = notecard.newRequest("note.add");  //create note request 
  if (req != NULL)
  {
    JAddStringToObject(req, "file", "buoy.qo");     
    JAddBoolToObject(req, "sync", true);
    J *body = JAddObjectToObject(req, "body");
    if (body)
    {
      int i = 1;
      if (lon[1] == 0 && lat[1] == 0)
        i = 0;
      //add all date/time objects to note 
      JAddStringToObject(body, "YYYY", yyyy[i]);
      JAddStringToObject(body, "MM", mM[i]);
      JAddStringToObject(body, "DD", dd[i]);  
      JAddStringToObject(body, "hh", hh[i]);
      JAddStringToObject(body, "mm", mm[i]);
      JAddStringToObject(body, "ss", ss[i]);
      JAddNumberToObject(body, "sats", sats[i]);
      JAddNumberToObject(body, "lat", lat[i]);            
      JAddNumberToObject(body, "lon", lon[i]);   
      //JAddNumberToObject(body, "pH", getAverage(pH,COLUMNS));
      //JAddNumberToObject(body, "conductivity", getAverage(conductivity,COLUMNS));    
      //JAddNumberToObject(body, "dissolved oxygen", getAverage(dissolvedOxygen,COLUMNS)); 
      JAddNumberToObject(body, "air temperature", airTemp[i]);  
      JAddNumberToObject(body, "water temperature", waterTemp[i]);
    }
    if (!notecard.sendRequest(req)) {
      JDelete(req);
      return -1;
    }                         
  }
  return 0;
}

/*
 * time: 2 sec
 * samples: 20
 */
void getAirTemp(){

  enableMuxPort(TSYS01_AIR_PORT);
  
  float sumOfTemps = 0;
  int count = 0;
  size_t start_ms = millis();

  while(count < 20){
    if(millis() >= start_ms + count * 100){ //read every 100 milliseconds
      airTempSensor.read();
      sumOfTemps += airTempSensor.temperature();
      count++;
    } 
  }
  
  airTemp[array_index] = sumOfTemps/20;

  disableMuxPort(TSYS01_AIR_PORT);
}

/*
 * time: 2 sec
 * samples: 20
 */
void getWaterTemp(){
  
  enableMuxPort(TSYS01_WATER_PORT);
  
  float sumOfTemps = 0;
  int count = 0;
  size_t start_ms = millis();
  
  while(count < 20){
    if(millis() >= start_ms + count * 100){ //read every 100 milliseconds
      waterTempSensor.read();
      sumOfTemps += waterTempSensor.temperature();
      count++;
    } 
  }
  
  waterTemp[array_index] = sumOfTemps/20;

  disableMuxPort(TSYS01_WATER_PORT);
}

/*
 * time: tbd
 * samples: tbd
 */
int get_pH(){
  return 0;
}

/*
 * time: tbd
 * samples: tbd
 */
int getDissolvedOxygen(){
  return 0;
}

/*
 * time: tbd
 * samples: tbd
 */
int getConductivity(){
  return 0;
}

/*
 * time: tbd
 * samples: tbd
 */
int checkVoltage(){
  return 0;
}


//- - - - - - - access data from location request - - - - -> 
int setLocationVars(J *rsp){  

  serialDebug.println("start location func");
  if (rsp == NULL) 
    return -1;

  lon[array_index] = JGetNumber(rsp, "lon"); 
  lon[array_index] = (floor(100000*lon[array_index])/100000); 
  lat[array_index] = JGetNumber(rsp, "lat");
  lat[array_index] = (floor(100000*lat[array_index])/100000);
  sats[array_index] = find_number_after_substring(JGetString(rsp, "status"), "R, ");

  return 0;
}

//if GPS can't acquire location info, set all fields to 0
void noGPS(){
  lat[array_index]=0;
  lon[array_index]=0;
  sats[array_index]=0;
}

//- - - - - access data from time request - - - - -
int setTimeVars(){
  serialDebug.println("start time func");
  time_t rawtime;//time object
  //request epoch time from notecard
  J *rsp = notecard.requestAndResponse(notecard.newRequest("card.time")); 
  if(rsp == NULL){

    return -1;
  }

  rawtime = JGetNumber(rsp, "time");  //store epoch time 
  notecard.deleteResponse(rsp);

  struct tm  ts;  

  // Format time, "yyyymmdd and hhmmss zzz" zzz? GMT
  ts = *localtime(&rawtime);
  strftime(yyyy[array_index], sizeof(yyyy[0]), "%Y", &ts); 
  strftime(mM[array_index], sizeof(mM[0]), "%m", &ts); 
  strftime(dd[array_index], sizeof(dd[0]), "%d", &ts); 
  strftime(hh[array_index], sizeof(hh[0]), "%H", &ts); 
  strftime(mm[array_index], sizeof(mm[0]), "%M", &ts); 
  strftime(ss[array_index], sizeof(ss[0]), "%S", &ts); 

  return 0;
}

//  - - - - - - - - - - parse # satellites - - - - - - - - - -
int find_number_after_substring(char* str, char* substring) {
  char* p = strstr(str, substring);
  if (p == NULL) {
    return -1;  // substring not found
  }
  p += strlen(substring); 

  // find the first digit after the substring
  while (*p != '\0' && !isdigit(*p)) {
    p++;
  }
  if (*p == '\0') {
    return -1;  // no number found after the substring
  }
  char* end = p;
  while (*end != '\0' && *end != '/') {
    end++;
  }
  if (*end == '\0') {
    return -1;  // number not followed by '/'
  }
  *end = '\0';
  return atoi(p);
}

float getAverage(float arr[], int n) {
    float sum = 0;
    
    for (int i = 0; i < n; i++) {
        sum += arr[i];
    }
    
    return sum / n;
}


// - - - functions for computing wave statistics - - - 
int sample_IMU(){
  
  enableMuxPort(IMU_PORT);

  /*har a[5];   //year   yyyy 
  char b[3];     //month  mm 
  char c[3];     //day    dd 
  char d[3];     //hour   hh   
  char e[3];     //minute mm 
  char f[3];     //second ss*/
  
  size_t start_ms = ::millis();

  /*time_t rawtime;//time object
  //request epoch time from notecard
  J *rsp = notecard.requestAndResponse(notecard.newRequest("card.time")); 
  if(rsp == NULL) 
    return -1;

  rawtime = JGetNumber(rsp, "time");  //store epoch time 
  notecard.deleteResponse(rsp);

  struct tm  ts;  
  
  ts = *localtime(&rawtime);
  
  strftime(a, 5, "%Y", &ts); 
  strftime(b, 3, "%m", &ts); 
  strftime(c, 3, "%d", &ts); 
  strftime(d, 3, "%H", &ts); 
  strftime(e, 3, "%M", &ts); 
  strftime(f, 3, "%S", &ts); 

  String fileName = String(ts.tm_yday + 1) + d + b + "I.txt";*/
  myFile = SD.open("waves.txt", FILE_WRITE);   
  
  if(myFile){
    
    serialDebug.println("file was made");

    /*serialDebug.print(a);
    serialDebug.print(",");
    serialDebug.print(b);
    serialDebug.print(",");
    serialDebug.print(c);
    serialDebug.print(",");
    serialDebug.print(d);
    serialDebug.print(",");
    serialDebug.print(e);
    serialDebug.print(",");
    serialDebug.print(f);
    serialDebug.print(",");
    serialDebug.print(lat[0]);
    serialDebug.print(",");
    serialDebug.print(lon[0]);
    serialDebug.print(",");
    serialDebug.println("4 Hz");

    myFile.println("YYYY, MM, DD, hh, mm, ss, lat, lon, sample rate");

    myFile.print(a);
    myFile.print(",");
    myFile.print(b);
    myFile.print(",");
    myFile.print(c);
    myFile.print(",");
    myFile.print(d);
    myFile.print(",");
    myFile.print(e);
    myFile.print(",");
    myFile.print(f);
    myFile.print(",");
    myFile.print(lat[0]);
    myFile.print(",");
    myFile.print(lon[0]);
    myFile.print(",");
    myFile.println("4 Hz");*/

    myFile.println("Ax (m/s^2),Ay (m/s^2),Az (m/s^2),Gx (deg/sec),Gy (deg/sec),Gz (deg/sec),Mx (mTesla),My (mTesla),Mz (mTesla)");

    int count = 0;
    
    while(count < IMU_SAMPLE_SIZE){
      if(millis() >= start_ms + 250 * count && myICM.dataReady()){
      
        myICM.getAGMT();  
  
        myFile.print(myICM.agmt.acc.axes.x/1000);
        myFile.print(",");
        myFile.print(myICM.agmt.acc.axes.y/1000);
        myFile.print(",");
        myFile.print(myICM.agmt.acc.axes.z/1000);
        myFile.print(",");
        myFile.print(myICM.agmt.gyr.axes.x);
        myFile.print(",");
        myFile.print(myICM.agmt.gyr.axes.y);
        myFile.print(",");
        myFile.print(myICM.agmt.gyr.axes.z);
        myFile.print(",");
        myFile.print(myICM.agmt.mag.axes.x);
        myFile.print(",");
        myFile.print(myICM.agmt.mag.axes.y);
        myFile.print(",");
        myFile.println(myICM.agmt.mag.axes.z);
  
        count++;
      }

    }
    
    myFile.close();
    serialDebug.println("IMU data collection complete");
    
  } else {
    serialDebug.println("ERROR: file was not made");
    return -1;
  }

  disableMuxPort(IMU_PORT);
  return 0;
}

//  - - - - - - - - - - - - - - - - - - - - - 
//     function to configure sensors  
//  - - - - - - - - - - - - - - - - - - - - - 

int configureSensors(){
  enableMuxPort(IMU_PORT);
  while(true){
    myICM.begin(Wire, 1);
    if (myICM.status == ICM_20948_Stat_Ok)
      break;
    delay(500);
  }
  disableMuxPort(IMU_PORT);

  enableMuxPort(TSYS01_AIR_PORT);
  airTempSensor.init();
  disableMuxPort(TSYS01_AIR_PORT);

  enableMuxPort(TSYS01_WATER_PORT);
  waterTempSensor.init();
  disableMuxPort(TSYS01_WATER_PORT);

  return 0;
}

//  - - - - - - - - - - - - - - - - - - - - - 
//     functions to access mux devices  
//  - - - - - - - - - - - - - - - - - - - - - 

//Enables a specific port number
int enableMuxPort(byte portNumber)
{
  if(portNumber > 7) portNumber = 7;

  //Read the current mux settings
  Wire.requestFrom(MUX_ADDR, 1);
  if(!Wire.available()) 
    return -1; //Error
  byte settings = Wire.read();

  //Set the wanted bit to enable the port
  settings |= (1 << portNumber);
 
  Wire.beginTransmission(MUX_ADDR);
  Wire.write(settings);
  Wire.endTransmission();

  return 0;
}

//Disables a specific port number
int disableMuxPort(byte portNumber)
{
  if(portNumber > 7) portNumber = 7;

  //Read the current mux settings
  Wire.requestFrom(MUX_ADDR, 1);
  if(!Wire.available())
   return -1; //Error
  byte settings = Wire.read();

  //Clear the wanted bit to disable the port
  settings &= ~(1 << portNumber);

  Wire.beginTransmission(MUX_ADDR);
  Wire.write(settings);
  Wire.endTransmission();

  return 0;
}

//  - - - - - - - - - - - - - - - - - - - 
//              SD card stuff 
//  - - - - - - - - - - - - - - - - - - -  

int setUpSD(){

  Serial.print("Initializing SD card...");
  if (SD.begin(chipSelect)) {

    Serial.println("initialization done.");
    myFile = SD.open("buoy.txt", FILE_WRITE);

    if(myFile){
      myFile.close();
      return 0;
    }
  }
  
  serialDebug.println("initialization failed!");
  return -1;
}

int writeToMainFile(){

  myFile = SD.open("buoy.txt", FILE_WRITE);
  if (myFile) {
    for(int i = 0; i < 2; i++){
      myFile.print(yyyy[i]);
      myFile.print(",");
      myFile.print(mM[i]);
      myFile.print(",");
      myFile.print(dd[i]);
      myFile.print(",");
      myFile.print(hh[i]);
      myFile.print(",");
      myFile.print(mm[i]);
      myFile.print(",");
      myFile.print(ss[i]);
      myFile.print(",");
      myFile.print(sats[i]);
      myFile.print(",");
      myFile.print(lat[i]);
      myFile.print(",");
      myFile.print(lon[i]);
      myFile.print(",");
      myFile.print(airTemp[i]);
      myFile.print(",");
      myFile.println(waterTemp[i]);
      myFile.close();
    }
    return 0;
  }

  serialDebug.println("opening file issue. You need to cry and stop everything");
  return -1;
}
