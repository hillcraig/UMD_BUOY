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
Ezo_board EC = Ezo_board(100, "EC");
//Ezo_board PH = Ezo_board(99, "PH");
//Ezo_board DO = Ezo_board(98, "DO");

//- - - - - - other variables - - - - - - - - 
int cycleCount = 0;            //cycle count
const int chipSelect = A5;     //selector for SD
const byte indicatorPin = A4;  //selecter for LED
const int COLUMNS = 3;         //the number entries gathered before a send
int column;                    //stores the current column
File myFile;
size_t mainLoopTime;           //stores time for main loop
bool successful_connection;    //stores whether or not a GPS connection was succesful

//- - - - - init data fields - - - - - - - - - 
char yyyy[5];   //year   yyyy
char mM[3];     //month  mm
char dd[3];     //day    dd
char hh[3];     //hour   hh
char mm[3];     //minute mm
char ss[3];     //second ss
double lat;     //lat    ##.#####
double lon;     //lon    ##.#####
int sats;       //number of GPS satellites
float airTemp[COLUMNS];        
float waterTemp[COLUMNS];        
float pH[COLUMNS];              
float conductivity[COLUMNS];    
float dissolvedOxygen[COLUMNS];

// - - - - set ports for multiplexer - - - - -
const int MUX_ADDR = 0x70;
const int TSYS01_AIR_PORT = 0;
const int TSYS01_WATER_PORT = 1;
const int IMU_PORT = 2;  
const int PH_PORT = 5; 
const int DISSOLVED_OXYGEN_PORT = 6;   
const int CONDUCTIVITY_PORT = 7;    

void setup() {
  delay(5000);
  serialDebug.begin(115200); //maybe 9600 instead
  notecard.begin();
  Wire.begin();
  configureSensors();
  setUpSD();                    
  pinMode(indicatorPin,OUTPUT); //for LED

  #if NDEBUG
    notecard.setDebugOutputStream(serialDebug);
  #endif
 
  J *req1 = notecard.newRequest("hub.set");          //init JSON object for request
  JAddStringToObject(req1, "product", productUID);   //set productUID
  JAddStringToObject(req1, "mode", "periodic");      //set mode to a continuous sensor
  if (!notecard.sendRequest(req1)) {                 //send off request for connection
    JDelete(req1);
  }

  J *req2 = notecard.newRequest("card.location.mode");
  JAddStringToObject(req2, "mode", "periodic");         //set gps mode to periodic 
  if (!notecard.sendRequest(req2)) {
    JDelete(req2);
  }
}

/*
 * time: 10 minutes
 */
void loop() { 
  serialDebug.println("main loop start");                    
  mainLoopTime = millis();
  //float voltage = checkVoltage();
  
  blinkLED(true);
  successful_connection = connectToGPS();
  blinkLED(successful_connection);
  
  getWaveStats();
  serialDebug.println("exited wave processing");
  
  for(column=0;column<COLUMNS;column++){
    serialDebug.println("Entering sequence " + String(column+1));
    sequenceData();
    //write to the SD 
  }

  sendData(); //get something back to tell you if it was a success? then write that to SD
//  writeToSD();

  serialDebug.println("- - - - - - - - - - - - - - -");
  serialDebug.print("minutes to spare: ");
  serialDebug.println(((mainLoopTime + 10 * 60000) - millis())/60000);
  
    
  while(millis() < mainLoopTime + 10 * 60000); //must take exactly 10 minutes before it restarts
  
  cycleCount++;
}

/*
 * time: tbd
 */
void sequenceData(){
    getAirTemp();
    getWaterTemp();
    get_pH();
    getConductivity();
    getDissolvedOxygen();
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
      notecard.logDebug("Timed out looking for a location\n");
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

      return true; //gps connection success
    }
  
    // If a "stop" field is on the card.location response, it means the Notecard
    // cannot locate a GPS/GNSS signal, so we break out of the loop to avoid looping
    // endlessly
    if (JGetObjectItem(rsp, "stop")) {
      NoteDeleteResponse(rsp);
      notecard.logDebug("Found a stop flag, cannot find location\n");
      break;
    }
    // Wait 2 seconds before tryring again 
    delay(2000);
  }
  noGPS();
  return false;
}

void sendData(){
  //send to cloud
  J *req = notecard.newRequest("note.add");   //create note request 
  if (req != NULL)
  {
    JAddStringToObject(req, "file", "buoy.qo");     
    JAddBoolToObject(req, "sync", true);
    J *body = JAddObjectToObject(req, "body");
    if (body)
    {
      //add all date/time objects to note 
      JAddStringToObject(body, "YYYY", yyyy);
      JAddStringToObject(body, "MM", mM);
      JAddStringToObject(body, "DD", dd);  
      JAddStringToObject(body, "hh", hh);
      JAddStringToObject(body, "mm", mm);
      JAddStringToObject(body, "ss", ss);
      JAddNumberToObject(body, "# of Satellites", sats);
      JAddNumberToObject(body, "lat", lat);            
      JAddNumberToObject(body, "lon", lon);   
      JAddNumberToObject(body, "pH", getAverage(pH,COLUMNS));
      JAddNumberToObject(body, "conductivity", getAverage(conductivity,COLUMNS));    
      JAddNumberToObject(body, "dissolved oxygen", getAverage(dissolvedOxygen,COLUMNS)); 
      JAddNumberToObject(body, "air temperature", getAverage(airTemp,COLUMNS));  
      JAddNumberToObject(body, "water temperature", getAverage(waterTemp,COLUMNS));
    }
    if (!notecard.sendRequest(req)) {
      JDelete(req);
    }                         
  }
  serialDebug.println("data sent");
}

/*
 * time: unknown
 * 
 */
void blinkLED(bool x){
  if(x){
    serialDebug.println("success");
    for(int i=0;i<10;i++){
      digitalWrite(indicatorPin,HIGH);  
      delay(150);
      digitalWrite(indicatorPin,LOW);
      delay(150);
    }
  }else{
     serialDebug.print("connection failed");
     digitalWrite(indicatorPin,HIGH);  
     delay(3000);
     digitalWrite(indicatorPin,LOW);
  }
  
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
    if(millis() >= start_ms + count * 100){//read every 100 milliseconds
      airTempSensor.read();
      sumOfTemps += airTempSensor.temperature();
      count++;
    } 
  }
  
  airTemp[column] = sumOfTemps/20;

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
    if(millis() >= start_ms + count * 100){//read every 100 milliseconds
      waterTempSensor.read();
      sumOfTemps += waterTempSensor.temperature();
      count++;
    } 
  }
  
  waterTemp[column] = sumOfTemps/20;

  disableMuxPort(TSYS01_WATER_PORT);
}

/*
 * time: tbd
 * samples: tbd
 */
void get_pH(){
  /*PH.send_read_cmd(); 
  delay(1000);
  PH.receive_read_cmd();
  delay(1000);
  pH[column] = PH.get_last_received_reading();*/
  pH[column] = 0;
}

/*
 * time: tbd
 * samples: tbd
 */
void getDissolvedOxygen(){
  /*DO.send_read_cmd(); 
  delay(1000);
  DO.receive_read_cmd();
  delay(1000);
  dissolvedOxygen[column] = DO.get_last_received_reading();*/
  dissolvedOxygen[column] = 0;
}

/*
 * time: tbd
 * samples: tbd
 */
void getConductivity(){
  enableMuxPort(CONDUCTIVITY_PORT);
  serialDebug.println("HERE WE GO");
  EC.send_read_cmd(); 
  delay(1000); //600ms? see pic add loop stuff
  Ezo_board::errors x = EC.receive_read_cmd();
  delay(1000);
  serialDebug.println(x);
  conductivity[column] = EC.get_last_received_reading();
  serialDebug.println(conductivity[column]);
  disableMuxPort(CONDUCTIVITY_PORT);
}

/*
 * time: tbd
 * samples: tbd
 */
void checkVoltage(){
  serialDebug.println("checking voltage!!");
  J *req = NoteNewRequest("card.voltage");
  JAddStringToObject(req, "mode", "?");
  J* rsp = notecard.requestAndResponse(req);
  //do something with the repsonse
  notecard.deleteResponse(rsp);
}

/*
    char status[20];
    bool connected = JGetBool(rsp, "connected");
    char *tempStatus = JGetString(rsp, "status");
    strlcpy(status, tempStatus, sizeof(status));
    int storage = JGetInt(rsp, "storage");
    int time = JGetInt(rsp, "time");
    bool cell = JGetBool(rsp, "cell");
 */

//- - - - - - - access data from location request - - - - -> 
void setLocationVars(J *rsp){  //make this take the req as a param 
  serialDebug.println("start location func");
  if (rsp != NULL) {
    lon = JGetNumber(rsp, "lon"); 
    lon = (floor(100000*lon)/100000); 
    lat = JGetNumber(rsp, "lat");
    lat = (floor(100000*lat)/100000);
    sats = find_number_after_substring(JGetString(rsp, "status"), "R, ");
  }  
}

//if GPS can't acquire location info, set all fields to 0
void noGPS(){
  lat=0;
  lon=0;
  sats=0;
}

//- - - - - access data from time request - - - - -
void setTimeVars(){
  serialDebug.println("start time func");
  time_t rawtime;//time object
  //request epoch time from notecard
  J *rsp = notecard.requestAndResponse(notecard.newRequest("card.time")); 
  if (rsp != NULL) {
     rawtime = JGetNumber(rsp, "time");  //store epoch time 
     notecard.deleteResponse(rsp);
  }

  struct tm  ts;  

  // Format time, "yyyymmdd and hhmmss zzz" zzz? GMT
  ts = *localtime(&rawtime);
  strftime(yyyy, sizeof(yyyy), ",Y", &ts); 
  strftime(mM, sizeof(mM), ",m", &ts); 
  strftime(dd, sizeof(dd), ",d", &ts); 
  strftime(hh, sizeof(hh), ",H", &ts); 
  strftime(mm, sizeof(mm), ",M", &ts); 
  strftime(ss, sizeof(ss), ",S", &ts); 

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
void getWaveStats(){
  
  serialDebug.println("getting wave stats");
  
  enableMuxPort(IMU_PORT);
  
  float **array;
  array = new float *[9];
  for(int i = 0; i<9; i++)
    array[i] = new float[2048];   
  
  size_t someTime = millis();


  //this willl be an int
  String fileName = String(cycleCount) + ".txt";
  myFile = SD.open(fileName, FILE_WRITE);   
  
  if(myFile){
    
    serialDebug.println("file was made");

    myFile.println("Ax (m/s^2),Ay (m/s^2),Az (m/s^2),Gx (deg/sec),Gy (deg/sec),Gz (deg/sec),Mx (mTesla),My (mTesla),Mz (mTesla)");

    int count = 0;
    
    while(count < 2048){
      if(millis() >= someTime + 62.5 * count && myICM.dataReady()){
      
        myICM.getAGMT();  // The values are only updated when you call 'getAGMT'
  
        array[0][count] = myICM.accX()/1000;
        array[1][count] = myICM.agmt.acc.axes.y/1000;
        array[2][count] = myICM.agmt.acc.axes.z/1000;
        array[3][count] = myICM.agmt.gyr.axes.x;
        array[4][count] = myICM.agmt.gyr.axes.y;
        array[5][count] = myICM.agmt.gyr.axes.z;
        array[6][count] = myICM.agmt.mag.axes.x;
        array[7][count] = myICM.agmt.mag.axes.y;
        array[8][count] = myICM.agmt.mag.axes.z;
  
        myFile.print(array[0][count]);
        myFile.print(",");
        myFile.print(array[1][count]);
        myFile.print(",");
        myFile.print(array[2][count]);
        myFile.print(",");
        myFile.print(array[3][count]);
        myFile.print(",");
        myFile.print(array[4][count]);
        myFile.print(",");
        myFile.print(array[5][count]);
        myFile.print(",");
        myFile.print(array[6][count]);
        myFile.print(",");
        myFile.print(array[7][count]);
        myFile.print(",");
        myFile.println(array[8][count]);
  
        count++;
      }
    }
    
    myFile.close();
    serialDebug.println("IMU collecting complete");
    
  } else {
    serialDebug.println("ERROR: file was not made");
    digitalWrite(indicatorPin,HIGH);
    while(true);
  }
    
     
  //instanstiate wave data class WaveStatistics waveMachine(array);
  //call methods waveMachine.getHeight(); waveMachine.getPeriod(); waveMachine.getDirection();

  for(int i = 0; i<9; i++)
    delete[] array[i];
    
  delete[] array;

  disableMuxPort(IMU_PORT);
}

//  - - - - - - - - - - - - - - - - - - - - - 
//     function to configure sensors  
//  - - - - - - - - - - - - - - - - - - - - - 

void configureSensors(){
  
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
}

//  - - - - - - - - - - - - - - - - - - - - - 
//     functions to access mux devices  
//  - - - - - - - - - - - - - - - - - - - - - 

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

//  - - - - - - - - - - - - - - - - - - - 
//              SD card stuff 
//  - - - - - - - - - - - - - - - - - - -  

void setUpSD(){
  Serial.print("Initializing SD card...");
  if (!SD.begin(chipSelect)) {
    Serial.println("initialization failed!");
    while (1);
  }
  Serial.println("initialization done.");
  
  if (SD.exists("buoy.txt")) {
    serialDebug.println("buoy.txt exists.");
    //do something
  } else {
    serialDebug.println("buoy.txt doesn't exist.");
    //open a new file and immediately close it:
    serialDebug.println("Creating imu.txt...");
    myFile = SD.open("buoy.txt", FILE_WRITE);
    myFile.close();
  } 
  
  myFile = SD.open("buoy.txt", FILE_WRITE);
  if(myFile){
    myFile.println("id,");
    myFile.close();
  }else{
    serialDebug.println("opening file issue. You need to cry and stop everything");
    digitalWrite(indicatorPin,HIGH);
    while(true);
  }
}

void writeToMainFile(){
  myFile = SD.open("buoy.txt", FILE_WRITE);
  if (myFile) {
      myFile.print("");
    myFile.close();
  }
}
