/*
 *  LTE_Buoy.ino
 *  Liam Gaeuman @ University of Minnesota Duluth
 *  1/31/24
 */

#include "Notecard.h" 
#include "TSYS01.h" 
#include "ICM_20948.h"
#include "Ezo_i2c.h" 
#include "WaveProcessor.h"
#include <time.h> 
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Servo.h>

#define serialDebug Serial
#define productUID "com.gmail.liamgaeuman:environmental_monitoring_buoy" 
#ifndef ARDUINO_SWAN_R5
#error "This program was designed to run on the Blues Wireless Swan"
#endif

//toggle steps: 1 on, 0 off
//only use 0 for testing purposes 
#define NDEBUG 1                  
#define SAMPLE_IMU 1              
#define TEMP_SENSORS 1            
#define WATER_QUALITY_SENSORS 1   
#define VOLTAGE 1                
#define GPS_CONNECT 1            
#define SEND_DATA 1               
#define WRITE_MAIN 1              
#define WAVE_STATS 1              
#define WAIT 1    

//- - - - - declare I2C addresses - - - - - - - - -
const int MUX_ADDR = 0x70;
const int EC_ADDR = 100;
const int DO_ADDR = 97;
const int PH_ADDR = 99;

// - - - - - set ports for multiplexer - - - - - - - 
const int IMU_PORT = 0;
const int TSYS01_AIR_PORT = 1;
const int TSYS01_WATER_PORT = 4;
const int EC_PORT = 5;
const int PH_PORT = 3;
const int DO_PORT = 2;

//- - - - - - declare sensors - - - - - - - - - - -
Notecard notecard;        //notecard object
TSYS01 airTempSensor;     //sesnor used to collect air temperature
TSYS01 waterTempSensor;   //sesnor used to collect water temperature
ICM_20948_I2C myICM;      //sensor for motion 

Ezo_board EC = Ezo_board(EC_ADDR, "EC");
Ezo_board DO = Ezo_board(DO_ADDR, "DO");  
Ezo_board PH = Ezo_board(PH_ADDR, "PH"); 

//- - - - - - init data fields - - - - - - - - - - - 
char yyyy[2][5];   //year   yyyy
char mM[2][3];     //month  mm
char dd[2][3];     //day    dd
char hh[2][3];     //hour   hh
char mm[2][3];     //minute mm
char ss[2][3];     //second ss
double lat[2];     //lat    ##.#####
double lon[2];     //lon    ##.#####
int sats[2];       //number of GPS satellites
char doy[4];       //day of year

float airTemp[2];          
float waterTemp[2];
float conductivity[2];
float dissolvedOxygen[2];
float pH[2];  
float voltage[2];

//- - - - - - other variables - - - - - - - - 
const byte SD_PIN = A5;             //selector for SD
const byte LED_PIN = A4;            //selecter for LED    
const int WQS = 10;                 //number of samples to take for each water quality sensor
const float GRAVITY = 9.8;          //
const int IMU_SAMPLE_SIZE = 4608;   //            
File myFile;                        //
size_t mainLoopTime;                //stores time for main loop
int array_index;                    // 

//error state blink enum -> 
enum states {SETUP_STATE, GPS_STATE, COLLECT_DATA_STATE, IMU_STATE, SEND_STATE, WRITE_STATE};

void setup()
{
  int error = 0;
  delay(5000);

  pinMode(LED_PIN, OUTPUT); //setup LED
  serialDebug.begin(115200); 
  notecard.begin();
  Wire.begin();

  configureSensors();
  #if WRITE_MAIN || SAMPLE_IMU                  
  error += setUpSD(); 
  #endif

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
    blinkLED(SETUP_STATE); 
  }
}

/*
 * time: 30 minutes
 */
void loop()
{ 
  mainLoopTime = millis();
  serialDebug.println("main loop start"); 
  array_index = 0;

  #if GPS_CONNECT
  serialDebug.println("obtaing GPS Connectin");
  blinkLED(100);
  if(connectToGPS() < 0){
    noGPS();
    setTimeVars();
    serialDebug.println("error obtaining GPS connection");
    blinkLED(-1);
    delay(500);
    blinkLED(GPS_STATE); 
  }
  #else
  noGPS();
  setTimeVars();
  #endif

  blinkLED(100);
  serialDebug.println("collecting data");
  if(collectOtherData() < 0){
    serialDebug.println("error collecting data");
    blinkLED(-1);
    delay(500);
    blinkLED(COLLECT_DATA_STATE); 
  }

  array_index = 1;

  #if SAMPLE_IMU
  blinkLED(100);
  serialDebug.println("collecting IMU data");
  if(sample_IMU() < 0){
    serialDebug.println("error sampling IMU");
    blinkLED(-1);
    delay(500);
    blinkLED(IMU_STATE); 
  }
  #endif

  #if GPS_CONNECT
  blinkLED(100);
  serialDebug.println("obtaing 2nd GPS connectiin");
  if(connectToGPS() < 0){
    noGPS();
    setTimeVars();
    serialDebug.println("error obtaining 2nd GPS connection");
    blinkLED(-1);
    delay(500);
    blinkLED(GPS_STATE); 
  }
  #else
  noGPS();
  setTimeVars();
  #endif
  
  blinkLED(100);
  serialDebug.println("collecting data");
  if(collectOtherData() < 0){
    serialDebug.println("error collecting data");
    blinkLED(-1);
    delay(500);
    blinkLED(COLLECT_DATA_STATE); 
  }

  #if SEND_DATA
  blinkLED(100);
  serialDebug.println("sending data");
  if(sendData() < 0){
    serialDebug.println("error sending data");
    blinkLED(-1);
    delay(500);
    blinkLED(SEND_STATE); 
  }
  #endif

  #if WRITE_MAIN
  blinkLED(100);
  serialDebug.println("writing to SD");
  if(writeToMainFile() < 0){
    serialDebug.println("error writing to SD");
    blinkLED(-1);
    delay(500);
    blinkLED(WRITE_STATE); 
  }
  #endif
  
  #if WAIT
  serialDebug.println("waiting");
  while(millis() < mainLoopTime + 30 * 60000){ //must take exactly 30 minutes before it restarts 
  }
  #endif
}

void blinkLED(int x) //rewrite this, ewwwwwwwwwwww
{
  int i;
  if(x == 100){
    for(i = 0; i < 10; i++){
      digitalWrite(LED_PIN, HIGH);  
      delay(100);
      digitalWrite(LED_PIN, LOW);
      delay(100);
    }
  }
  else if (x >= 0 ){
    for(i=0; i < x+1; i++){
      digitalWrite(LED_PIN, HIGH);  
      delay(500);
      digitalWrite(LED_PIN, LOW);
      delay(250);
    }
  }
  else{
    digitalWrite(LED_PIN, HIGH);  
    delay(4000);
    digitalWrite(LED_PIN, LOW);
  }
}

int collectOtherData()
{
    #if TEMP_SENSORS
    getTemp(TSYS01_AIR_PORT, airTempSensor, airTemp);
    getTemp(TSYS01_WATER_PORT, waterTempSensor, waterTemp);
    #endif

    #if WATER_QUALITY_SENSORS
    getWaterQualitySesnorReadings(EC_PORT, EC, conductivity, WQS);
    getWaterQualitySesnorReadings(DO_PORT, DO, dissolvedOxygen, WQS);
    getWaterQualitySesnorReadings(PH_PORT, PH, pH, WQS);
    #endif

    #if VOLTAGE
    getVoltage();
    #endif

    return 0;
}

#if GPS_CONNECT
int connectToGPS()
{  
  // Save the time from the last location reading.
  J *rsp = notecard.requestAndResponse(notecard.newRequest("card.location"));
  size_t previous_gps_time_s = JGetInt(rsp, "time");
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
    if (JGetInt(rsp, "time") != previous_gps_time_s) {   //once updated, location info will have a new time
      
      setTimeVars();
      setLocationVars(rsp);        
      notecard.deleteResponse(rsp);
      
      // Restore previous configuration
      J *req = notecard.newRequest("card.location.mode");
      JAddStringToObject(req, "mode", "periodic");
      notecard.sendRequest(req);
      serialDebug.println("GPS connection successful :)\n");
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
  return -1; //gps connection failed
}
#endif

#if SEND_DATA
int sendData()
{
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
      JAddNumberToObject(body, "air temperature", airTemp[i]);  
      JAddNumberToObject(body, "water temperature", waterTemp[i]);
      JAddNumberToObject(body, "dissolved oxygen", dissolvedOxygen[i]);
      JAddNumberToObject(body, "conductivity", conductivity[i]);
      JAddNumberToObject(body, "pH", pH[i]);
      JAddNumberToObject(body, "voltage", voltage[i]);
    }
    if (!notecard.sendRequest(req)) {
      JDelete(req);
      return -1;
    }                         
  }
  return 0;
}
#endif

#if TEMP_SENSORS
/*
 * time: 2 sec
 * samples: 20
 */
void getTemp(int port, TSYS01 tempSensor, float* arr)
{

  enableMuxPort(port);
  
  float sumOfTemps = 0;
  int count = 0;
  int NUM_READINGS = 20;
  size_t start_ms = millis();

  while(count < NUM_READINGS){
    if(millis() >= start_ms + count * 100){ //read every 100 milliseconds
      tempSensor.read();
      delay(80);
      sumOfTemps += tempSensor.temperature();
      count++;
    } 
  }
  
  arr[array_index] = sumOfTemps/NUM_READINGS;

  disableMuxPort(port);
}
#endif

/*
 * time: 2 sec
 * samples: 20
 */
#if WATER_QUALITY_SENSORS
int getWaterQualitySesnorReadings(int port, Ezo_board sensor, float* arr, int samples)
{  
  enableMuxPort(port);
  
  float sumOfReadings = 0;
  int NUM_READINGS = samples;
  
  for(int i=0; i<NUM_READINGS; i++){
    sensor.send_read_cmd(); 
    delay(1000);
    if(sensor.receive_read_cmd() == Ezo_board::NOT_READ_CMD)
      return -1;
    sumOfReadings += sensor.get_last_received_reading();
  }

  arr[array_index] = sumOfReadings/NUM_READINGS;

  disableMuxPort(port);
}
#endif

void getVoltage()
{
  J *req = NoteNewRequest("card.voltage");
  JAddStringToObject(req, "mode", "?");
  
  J *rsp = notecard.requestAndResponse(req);
  voltage[array_index] = JGetNumber(rsp, "value");
  notecard.deleteResponse(rsp);
}

//- - - - - - - access data from location request - - - - -> 
#if GPS_CONNECT
int setLocationVars(J *rsp)
{  
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
#endif

//if GPS can't acquire location info, set all fields to 0
void noGPS()
{
  lat[array_index]=0;
  lon[array_index]=0;
  sats[array_index]=0;
}

//- - - - - access data from time request - - - - -
int setTimeVars()
{
  serialDebug.println("start time func");
  time_t rawtime;//time object

  //request epoch time from notecard
  J *rsp = notecard.requestAndResponse(notecard.newRequest("card.time")); 
  if(rsp == NULL){
    return -1;
  }

  rawtime = JGetNumber(rsp, "time");  //store epoch time 

  lon[array_index] = JGetNumber(rsp, "lon"); 
  lon[array_index] = (floor(100000*lon[array_index])/100000); 
  lat[array_index] = JGetNumber(rsp, "lat");
  lat[array_index] = (floor(100000*lat[array_index])/100000);

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

  snprintf(doy, sizeof(doy), "%03d", ts.tm_yday);

  return 0;
}

int find_number_after_substring(char* str, char* substring) {
  char* p = strstr(str, substring);
  if (p == NULL) {
    return -1;  
  }
  p += strlen(substring); 

  while (*p != '\0' && !isdigit(*p)) {
    p++;
  }
  if (*p == '\0') {
    return -1;  
  }
  char* end = p;
  while (*end != '\0' && *end != '/') {
    end++;
  }
  if (*end == '\0') {
    return -1;  
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
#if SAMPLE_IMU
int sample_IMU()
{
  setTimeVars();

  char filename[13];
  strcpy(filename, doy);
  strcat(filename, hh[array_index]);
  strcat(filename, mm[array_index]);
  strcat(filename, "I.txt");
   
  enableMuxPort(IMU_PORT);

  
  size_t start_ms = ::millis();

  myFile = SD.open(filename, FILE_WRITE);   
  
  if(myFile){
    
    serialDebug.println("file was made");

    myFile.print(yyyy[array_index]);
    myFile.print(",");
    myFile.print(mM[array_index]);
    myFile.print(",");
    myFile.print(dd[array_index]);
    myFile.print(",");
    myFile.print(hh[array_index]);
    myFile.print(",");
    myFile.print(mm[array_index]);
    myFile.print(",");
    myFile.print(ss[array_index]);
    myFile.print(",");
    myFile.print(sats[array_index]);
    myFile.print(",");
    myFile.print(lat[array_index]);
    myFile.print(",");
    myFile.println(lon[array_index]);

    myFile.println("Ax (m/s^2),Ay (m/s^2),Az (m/s^2),Gx (deg/sec),Gy (deg/sec),Gz (deg/sec),Mx (mTesla),My (mTesla),Mz (mTesla)");

    int count = 0;

    float **array;
    array = new float *[9];
    for(int i = 0; i<9; i++)
      array[i] = new float[IMU_SAMPLE_SIZE];
    
    //add some light action

    while(count < IMU_SAMPLE_SIZE){
      if(millis() >= start_ms + 250 * count && myICM.dataReady()){
        myICM.getAGMT();  

        array[0][count] = myICM.accX()/1000 * GRAVITY;
        array[1][count] = myICM.accY()/1000 * GRAVITY;
        array[2][count] = myICM.accZ()/1000 * GRAVITY;
        array[3][count] = myICM.gyrX();
        array[4][count] = myICM.gyrY();
        array[5][count] = myICM.gyrZ();
        array[6][count] = myICM.magX();
        array[7][count] = myICM.magY();
        array[8][count] = myICM.magZ();

        myFile.print(array[0][count]); // m/s
        myFile.print(",");
        myFile.print(array[1][count]); // m/s
        myFile.print(",");
        myFile.print(array[2][count]); // m/s
        myFile.print(",");
        myFile.print(array[3][count]);   // degrees per second
        myFile.print(",");
        myFile.print(array[4][count]);   // degrees per second
        myFile.print(",");
        myFile.print(array[5][count]);   // degrees per second
        myFile.print(",");
        myFile.print(array[6][count]);   // micro teslas
        myFile.print(",");
        myFile.print(array[7][count]);   // micro teslas
        myFile.print(",");
        myFile.println(array[8][count]); // micro teslas
  
        count++;
      }
    }

    WaveProcessor waveProcessor = WaveProcessor(array);
    float x = waveProcessor.getHeight();
    serialDebug.println(x);

    for(int i = 0; i<9; i++)
      delete[] array[i];
    delete[] array;
    
    myFile.close();
    serialDebug.println("IMU data collection complete");
    
  } else {
    serialDebug.println("ERROR: file was not made");
    return -1;
  }
  disableMuxPort(IMU_PORT);
  return 0;
}
#endif

//  - - - - - - - - - - - - - - - - - - - - - 
//     function to configure sensors  
//  - - - - - - - - - - - - - - - - - - - - - 
int configureSensors()
{
  #if SAMPLE_IMU
  enableMuxPort(IMU_PORT);
  while(true){
    myICM.begin(Wire, 1);
    if (myICM.status == ICM_20948_Stat_Ok)
      break;
    delay(500);
  }
  disableMuxPort(IMU_PORT);
  #endif

  #if TEMP_SENSORS
  enableMuxPort(TSYS01_AIR_PORT);
  airTempSensor.init();
  disableMuxPort(TSYS01_AIR_PORT);

  enableMuxPort(TSYS01_WATER_PORT);
  waterTempSensor.init();
  disableMuxPort(TSYS01_WATER_PORT);
  #endif

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
#if WRITE_MAIN || SAMPLE_IMU
int setUpSD()
{
  Serial.print("Initializing SD card...");
  if (SD.begin(SD_PIN)) {

    Serial.println("initialization done.");
    myFile = SD.open("test.txt", FILE_WRITE);

    if(myFile){
      myFile.println("Meow");
      myFile.close();
      return 0;
    }
  }
  
  serialDebug.println("initialization failed!");
  return -1;
}
#endif

#if WRITE_MAIN
int writeToMainFile()
{
  char filename[13];
  strcpy(filename, "BUOY");
  strcat(filename, yyyy[array_index]);
  strcat(filename, ".txt");

  myFile = SD.open(filename, FILE_WRITE);
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
      myFile.print(waterTemp[i]);
      myFile.print(",");
      myFile.print(conductivity[i]);
      myFile.print(",");
      myFile.print(dissolvedOxygen[i]);
      myFile.print(",");
      myFile.print(pH[i]);
      myFile.print(",");
      myFile.println(voltage[i]);
    }
    myFile.close();
    return 0;
  }

  return -1;
}
#endif