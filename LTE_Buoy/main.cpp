/*
*  LTE_Buoy.ino
*  Liam Gaeuman @ University of Minnesota Duluth
*  5/24/24
*/

#include <Arduino.h>
#include "Notecard.h" 
#include "TSYS01.h" 
#include "ICM_20948.h"
#include "Ezo_i2c.h" 
#include "SparkFun_MS5803_I2C.h"
#include "SparkFun_I2C_Mux_Arduino_Library.h"
#include "SD.h"
#include "wave_processor.h" 
#include <ctype.h>   
#include <stdlib.h> 
#include <time.h> 
#include <Wire.h>
#include <SPI.h>
#include <Servo.h>
#include <math.h> 

#define serialDebug Serial
#define productUID "com.gmail.liamgaeuman:environmental_monitoring_buoy" 
#ifndef ARDUINO_SWAN_R5
#error "This program was designed to run on the Blues Wireless Swan"
#endif

//toggle steps: 1 on, 0 off
//only use 0 for testing purposes except for NDBUG which should be 0 for deployment 
#define DEBUG_PRINT 1                  
#define SAMPLE_IMU 1                           
#define GPS_CONNECT 0 
#define OTHER_DATA 1          
#define SEND_DATA 1               
#define WRITE_MAIN 1                           
#define WAIT 1    

// --- wave-processing constants ― samples @ 16 Hz for 19.2 min ---
#define WAVE_N_SAMPLES   4608 // input to wave procesor lib (really collecting 4608 * 4 samples)

// - - - Function Prototypes - - - 
void  blink(int error, int state);
int   otherData();
void  noData();
int   getTemp(byte port, TSYS01 &tempSensor, float* arr);
int   getWaterQualitySesnorReadings(byte port, Ezo_board sensor, float* arr, int samples);
int   get_voltage();
int   get_signal_metrics();
int   connectToGPS();
int   sendData();
int   setLocationVars(J *rsp);
void  noGPS();
int   setTimeVars();
int   find_number_after_substring(const char *str, const char *substring);
int   sample_IMU();
void  noIMU();
int   configureSensors();
int   setUpSD();
int   writeToMainFile();

//static accel buffer for wave algorithm (no dynamic memory)
float accel_data[WAVE_N_SAMPLES][3];

//- - - - - declare I2C addresses - - - - - - - - -
const byte EC_ADDR = 100;
const byte PH_ADDR = 99;
const byte ORP_ADDR = 98;
const byte DO_ADDR = 97;
const byte RTD_ADDR = 102;

// - - - - - set ports for multiplexer - - - - - - - 
const byte IMU_PORT = 0;
const byte TSYS01_AIR_PORT = 1;
const byte EC_PORT = 5;
const byte PH_PORT = 3;
const byte DO_PORT = 2;
const byte ORP_PORT = 7;
const byte RTD_PORT = 4;
const byte PRESSURE_PORT = 6;

//- - - - - - declare sensors - - - - - - - - - - -
Notecard notecard;        //notecard object
QWIICMUX myMux;           //multiplexer
TSYS01 airTempSensor;     //sesnor used to collect air temperature
ICM_20948_I2C myICM;      //sensor for motion 
//MS5803 pressureSensor(ADDRESS_HIGH);  //pressure
Ezo_board EC = Ezo_board(EC_ADDR, "EC");
Ezo_board DO = Ezo_board(DO_ADDR, "DO");  
Ezo_board PH = Ezo_board(PH_ADDR, "PH"); 
Ezo_board ORP = Ezo_board(ORP_ADDR, "ORP"); 
Ezo_board RTD = Ezo_board(RTD_ADDR, "RTD"); 

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
float sinr[2];
float rssi[2];
float pressure[2]; 
float orp[2];

float wave_height = NAN;   //Hs
float wave_period = NAN;


//- - - - - - other variables - - - - - - - - - -
const byte SD_PIN = A5;             // selector for SD
const byte LED_PIN = A4;            // selecter for LED    
const int WQS = 10;                 // number of samples to take for each water quality sensor
const float GRAVITY = 9.8;          //
const int IMU_SAMPLE_SIZE = WAVE_N_SAMPLES;   //  
const byte LOOP_DURATION = 30;      // 30 minutes 
File myFile;                        //
size_t mainLoopTime;                // stores time for main loop
int array_index;                    // 

// - - - - - - - - - - - - - - - - - - - - - - - - - - - 
enum states {SETUP_STATE, GPS_STATE, OTHER_DATA_STATE, IMU_STATE, SEND_STATE, WRITE_STATE};

/* -------------------------------------------------------------------------- */
/*                                SETUP & LOOP                                */
/* -------------------------------------------------------------------------- */

void setup()
{
  #if DEBUG_PRINT
  delay(5000); // delays for 5 seconds
  #endif

  int error = 0;
  
  pinMode(LED_PIN, OUTPUT); //setup LED
  serialDebug.begin(9600); 
  Wire.begin();
  myMux.begin();
  notecard.begin();

  #if DEBUG_PRINT
  notecard.setDebugOutputStream(serialDebug);
  #endif

  configureSensors();

  #if WRITE_MAIN || SAMPLE_IMU                  
  error += setUpSD(); 
  #endif

  J *req = notecard.newRequest("hub.set");
  if (req) {
    JAddStringToObject(req, "product", productUID);
    JAddStringToObject(req, "mode", "periodic");
    JAddBoolToObject(req, "sync", true);
    if (!notecard.sendRequest(req)) {
      serialDebug.println("FATAL: Failed to configure Notecard!\n");
      error -= 1;
    }
  }

  J *req2 = notecard.newRequest("card.location.mode");
  JAddStringToObject(req2, "mode", "periodic");          //set gps mode to periodic 
  if (!notecard.sendRequest(req2)) 
    error -= 1; 

  blink(error, SETUP_STATE);
}

/*
* time: 30 minutes
*/
void loop()
{ 
  mainLoopTime = millis(); //get loop start time 
  array_index = 0;

  #if GPS_CONNECT
  blink(connectToGPS(),GPS_STATE);
  #else
  noGPS();
  setTimeVars();
  #endif

  #if OTHER_DATA
  blink(otherData(),OTHER_DATA_STATE);
  #else
  noData();
  #endif

  array_index = 1;

  #if SAMPLE_IMU
  blink(sample_IMU(),IMU_STATE);
  #else
  noIMU();
  #endif

  #if GPS_CONNECT
  blink(connectToGPS(),GPS_STATE);
  #else
  noGPS();
  setTimeVars();
  #endif
  
  #if OTHER_DATA
  blink(otherData(),OTHER_DATA_STATE);
  #endif

  #if SEND_DATA
  blink(sendData(),SEND_STATE);
  #endif

  #if WRITE_MAIN
  blink(writeToMainFile(),WRITE_STATE);
  #endif
  
  #if WAIT
  size_t x = millis();
  while(millis() < mainLoopTime + LOOP_DURATION * 60000){  // wait for remaining time to fulfill 30 minute loop cycle
    /*  every 20 seconds */
    if(millis() > x + 20000){
      x = millis();
      serialDebug.println("waiting");
    }
  }
  #endif
}

/* -------------------------------------------------------------------------- */
/*                                BLINK HANDLER                               */
/* -------------------------------------------------------------------------- */

void blink(int error, int state) 
{
  int i;
  if (error >= 0){
    for(i = 0; i < 10; i++){
      digitalWrite(LED_PIN, HIGH);  
      delay(100);
      digitalWrite(LED_PIN, LOW);
      delay(100);
    }
  }
  else{ /* in event of error */
    serialDebug.println("ERROR");
    digitalWrite(LED_PIN, HIGH);  
    delay(4000);
    digitalWrite(LED_PIN, LOW);
    delay(1000);
    for(i = 0; i < state+1; i++){
      digitalWrite(LED_PIN, HIGH);  
      delay(500);
      digitalWrite(LED_PIN, LOW);
      delay(250);
    }
    myFile = SD.open("ERROR_LOG.txt", FILE_WRITE);
    if (myFile) {

      serialDebug.println(state); //error code
      serialDebug.println(array_index); // current position in loop

      myFile.println(state); //error code
      myFile.println(array_index); // current position in loop

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
        myFile.print(orp[i]);
        myFile.print(",");
        myFile.print(sinr[i]);
        myFile.print(",");
        myFile.println(rssi[i]);
      }
      myFile.close();
    }
    //delay(35000); //let watchdog reset microcontroller 
  }
}

/* -------------------------------------------------------------------------- */
/*                           WATER-QUALITY SEQUENCE                           */
/* -------------------------------------------------------------------------- */

#if OTHER_DATA
int otherData()
{
    serialDebug.println("--- Starting Subsequence");
    int err = 0;                                    // 0 → success, −1 → something failed

    if (getTemp(TSYS01_AIR_PORT, airTempSensor, airTemp) != 0)             err = -1;
    if (getWaterQualitySesnorReadings(EC_PORT,  EC,  conductivity,   WQS) != 0) err = -1;
    if (getWaterQualitySesnorReadings(DO_PORT,  DO,  dissolvedOxygen, WQS) != 0) err = -1;
    if (getWaterQualitySesnorReadings(PH_PORT,  PH,  pH,             WQS) != 0) err = -1;
    if (getWaterQualitySesnorReadings(ORP_PORT, ORP, orp,            WQS) != 0) err = -1;
    if (getWaterQualitySesnorReadings(RTD_PORT, RTD, waterTemp,      WQS) != 0) err = -1;
    myMux.disablePort(RTD_PORT);
    if (get_voltage() != 0)                                                    err = -1;
    if (get_signal_metrics() != 0)                                             err = -1;

    //myMux.setPort(PRESSURE_PORT);
    //pressure[array_index] = pressureSensor.getPressure(ADC_4096);

    if (err == 0)
        serialDebug.println("--- Subsequence Complete (OK)");
    else
        serialDebug.println("--- Subsequence Complete with ERRORS");

    return err;                          // blink() will flash error pattern if < 0
}
#else
void noData()
{
  for(int i=0;i<2;i++){
    airTemp[i] = NAN;          
    waterTemp[i] = NAN;
    conductivity[i] = NAN;
    dissolvedOxygen[i] = NAN;
    pH[i] = NAN;  
    voltage[i] = NAN;
    sinr[i] = NAN;
    rssi[i] = NAN;
    //pressure[i] = NAN; 
  }
}
#endif

/* -------------------------------------------------------------------------- */
/*                               GPS CONNECTION                               */
/* -------------------------------------------------------------------------- */

#if GPS_CONNECT
int connectToGPS()
{  
  serialDebug.println("--- Starting attempt to obtain GPS connection");

  J *rsp = notecard.requestAndResponse(notecard.newRequest("card.location"));
  if (rsp == nullptr) {
    serialDebug.println("Error: No response from Notecard.");
    return -1;
  }
  size_t previous_gps_time_s = JGetInt(rsp, "time");
  notecard.deleteResponse(rsp);
  
  J *req = notecard.newRequest("card.location.mode");
  JAddStringToObject(req, "mode", "continuous");
  notecard.sendRequest(req);

  size_t timeout_s = 150; //2.5 min

  for (const size_t start_ms = ::millis();;) {

    if (::millis() >= (start_ms + (timeout_s * 1000))) {
      serialDebug.println("Timed out looking for a location\n");
      break;
    }
  
    rsp = notecard.requestAndResponse(notecard.newRequest("card.location"));
    if (JGetInt(rsp, "time") != previous_gps_time_s) {   //once updated, location info will have a new time
      
      setTimeVars();
      setLocationVars(rsp);        
      notecard.deleteResponse(rsp);
      
      req = notecard.newRequest("card.location.mode");
      JAddStringToObject(req, "mode", "periodic");
      notecard.sendRequest(req);
      serialDebug.println("--- GPS connection successful :)\n");
      return 0;
    }
  
    if (JGetObjectItem(rsp, "stop")) {
      notecard.deleteResponse(rsp);
      serialDebug.println("Found a stop flag, cannot find location\n");
      break;
    }
    delay(2000);
  }
  serialDebug.println("failed to obtain GPS connection");
  noGPS();
  return 0;
}
#endif

/* -------------------------------------------------------------------------- */
/*                               DATA SEND-OUT                                */
/* -------------------------------------------------------------------------- */

#if SEND_DATA
int sendData()
{
  serialDebug.println("--- Sending Data");

  J *req = notecard.newRequest("note.add");  //create note request 
  if (req != NULL)
  {
    JAddStringToObject(req, "file", "buoy.qo");     
    JAddBoolToObject(req, "sync", true);
    J *body = JAddObjectToObject(req, "body");
    if (body)
    {
      int i = (lon[1]==lon[1]) ? 1 : 0;   // crude isnan() without math.h macro
      if (!i) { i = 0; }                  // keep original style

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
      JAddNumberToObject(body, "air_temp", airTemp[i]);  
      JAddNumberToObject(body, "water_temp", waterTemp[i]);
      JAddNumberToObject(body, "dissolved_oxygen", dissolvedOxygen[i]);
      JAddNumberToObject(body, "conductivity", conductivity[i]);
      JAddNumberToObject(body, "pH", pH[i]);
      JAddNumberToObject(body, "orp", orp[i]);
      JAddNumberToObject(body, "rssi", rssi[i]);
      JAddNumberToObject(body, "sinr", sinr[i]);
      //JAddNumberToObject(body, "pressure", pressure[i]);
      JAddNumberToObject(body, "voltage", voltage[i]);
      JAddNumberToObject(body, "Hs", wave_height);  
    }
    if (!notecard.sendRequest(req)){
      serialDebug.println("Failed to send data");
      return -1;
    }
  }
  serialDebug.println("--- Data Sent");
  return 0;
}
#endif

/* -------------------------------------------------------------------------- */
/*                         SENSOR-UTILITY FUNCTIONS                           */
/* -------------------------------------------------------------------------- */

#if OTHER_DATA
int getTemp(byte port, TSYS01 &tempSensor, float *arr)
{
    serialDebug.println(F("--- Getting Temperature"));
    myMux.setPort(port);

    float sum = 0.0f;
    int   samples = 0;
    const uint32_t t0 = millis();

    while (samples < WQS) {
        if (millis() - t0 >= samples * 100UL) {   // every 100 ms
            tempSensor.read();                    // returns void

            float T = tempSensor.temperature();   // pull last reading
            if (isnan(T)) {                       // sensor/I²C fault
                serialDebug.println(F("Temp read failed"));
                return -1;
            }

            sum += T;
            ++samples;
            delay(80);                            // sensor’s conversion time
        }
    }

    arr[array_index] = sum / WQS;
    serialDebug.println(F("--- Temperature obtained"));
    return 0;
}
#endif


#if OTHER_DATA
int getWaterQualitySesnorReadings(byte port, Ezo_board sensor, float* arr, int samples)
{  
  serialDebug.println("--- Reading Water Quality Sensor");
  myMux.setPort(port);
  
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
  serialDebug.println("--- Water Quality Sensor Read Complete!");
  return 0;
}
#endif

#if OTHER_DATA
int get_voltage()                         // 0 = OK, −1 = error
{
    J *req = NoteNewRequest("card.voltage");
    JAddStringToObject(req, "mode", "?");

    J *rsp = notecard.requestAndResponse(req);
    if (rsp == nullptr) return -1;

    voltage[array_index] = JGetNumber(rsp, "value");
    notecard.deleteResponse(rsp);
    return 0;
}
#endif

#if OTHER_DATA
int get_signal_metrics()
{
  J *rsp = notecard.requestAndResponse(NoteNewRequest("card.wireless"));

  if(rsp == nullptr){
    return -1;
  }

  J *info = JGetObject(rsp, "net"); 
  sinr[array_index] = JGetInt(info, "sinr");
  rssi[array_index] = JGetInt(info, "rssi");
  notecard.deleteResponse(rsp);
  return 0;  
}
#endif

/* -------------------------------------------------------------------------- */
/*                          LOCATION / TIME HELPERS                           */
/* -------------------------------------------------------------------------- */

#if GPS_CONNECT
int setLocationVars(J *rsp)
{  
  serialDebug.println("--- Starting location func");
  if (rsp == nullptr) 
    return -1;

  lon[array_index] = JGetNumber(rsp, "lon"); 
  lon[array_index] = (floor(100000*lon[array_index])/100000); 
  lat[array_index] = JGetNumber(rsp, "lat");
  lat[array_index] = (floor(100000*lat[array_index])/100000);
  sats[array_index] = find_number_after_substring(JGetString(rsp, "status"), "R, ");

  return 0;
}
#endif

void noGPS()
{
  lat[array_index] = NAN;
  lon[array_index] = NAN;
  sats[array_index] = -1;
}

int setTimeVars()
{
  serialDebug.println("start time func");
  time_t rawtime;//time object

  J *rsp = notecard.requestAndResponse(notecard.newRequest("card.time")); 
  if(rsp == nullptr){
    return -1;
  }

  rawtime = JGetNumber(rsp, "time");  //store epoch time 

  lon[array_index] = JGetNumber(rsp, "lon"); 
  lon[array_index] = (floor(100000*lon[array_index])/100000); 
  lat[array_index] = JGetNumber(rsp, "lat");
  lat[array_index] = (floor(100000*lat[array_index])/100000);

  notecard.deleteResponse(rsp);

  struct tm  ts;  

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

int find_number_after_substring(const char *str, const char *substring)
{
    if (!str || !substring) return -1;

    const char *p = strstr(str, substring);
    if (!p) return -1;

    p += strlen(substring);             // advance past the substring

    /* skip anything that isn’t a digit */
    while (*p && !isdigit(static_cast<unsigned char>(*p)))
        ++p;

    if (!isdigit(static_cast<unsigned char>(*p)))   // no digits found
        return -1;

    /* strtol stops as soon as it hits a non-digit, so “15/6” → 15 */
    char *endptr;
    long value = strtol(p, &endptr, 10);

    return static_cast<int>(value);
}

/* -------------------------------------------------------------------------- */
/*                               IMU SAMPLING                                */
/* -------------------------------------------------------------------------- */

#if SAMPLE_IMU
int sample_IMU()
{
  serialDebug.println("--- Starting IMU Sampling\n");

  setTimeVars();

  char filename[13];
  strcpy(filename, doy);
  strcat(filename, hh[array_index]);
  strcat(filename, mm[array_index]);
  strcat(filename, "I.txt");
  
  myMux.setPort(IMU_PORT);

  myFile = SD.open(filename, FILE_WRITE);   
  
  if(myFile){
    
    serialDebug.println("file was made");

    myFile.println("yyyy,mM,dd,hh,mm,ss,sats,lat,lon");

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

    const uint32_t BASE_MS = 62;          
    bool extra_ms = false;                
    uint32_t next_ms = millis();          

    int count = 0;
    while (count < IMU_SAMPLE_SIZE * 4) {

        if ((int32_t)(millis() - next_ms) >= 0 && myICM.dataReady()) {

            myICM.getAGMT();

            float ax = myICM.accX() / 1000 * GRAVITY;
            float ay = myICM.accY() / 1000 * GRAVITY;
            float az = myICM.accZ() / 1000 * GRAVITY;

            if (count % 4 == 0) {
                accel_data[count / 4][0] = ax;
                accel_data[count / 4][1] = ay;
                accel_data[count / 4][2] = az;
            }

            myFile.print(ax);              myFile.print(",");
            myFile.print(ay);              myFile.print(",");
            myFile.print(az);              myFile.print(",");
            myFile.print(myICM.gyrX());    myFile.print(",");
            myFile.print(myICM.gyrY());    myFile.print(",");
            myFile.print(myICM.gyrZ());    myFile.print(",");
            myFile.print(myICM.magX());    myFile.print(",");
            myFile.print(myICM.magY());    myFile.print(",");
            myFile.println(myICM.magZ());

            count++;
            
            next_ms += BASE_MS + (extra_ms ? 1 : 0);
            extra_ms = !extra_ms;          
        }
    }

    myFile.close();
    serialDebug.println("--- IMU sampling complete");

    /* ---------- actual processing ---------- */
    WaveProcessorContext ctx;
    wave_processor_init(&ctx);
    wave_processor_compute_spectrum(&ctx, accel_data);

    float Hs, Tm01, Tm02;
    wave_processor_get_stats(&ctx, &Hs, &Tm01, &Tm02);

    wave_height = Hs;
    wave_period = Tm01;   
    serialDebug.println("Hs: ");
    serialDebug.println(Hs);

  } else {
    noIMU();
    return -1;
  }
  return 0;
}
#endif

void noIMU(){
  wave_height = NAN;
  wave_period = NAN;
}

/* -------------------------------------------------------------------------- */
/*                           SENSOR CONFIG / SD SETUP                         */
/* -------------------------------------------------------------------------- */

int configureSensors()
{
  #if SAMPLE_IMU
  myMux.setPort(IMU_PORT);
  while(true){
    myICM.begin(Wire, 1);
    if (myICM.status == ICM_20948_Stat_Ok)
      break;
    delay(500);
  }
  #endif

  #if OTHER_DATA
  myMux.setPort(TSYS01_AIR_PORT);
  airTempSensor.init();

  //myMux.setPort(PRESSURE_PORT);
  //pressureSensor.begin();
  #endif

  return 0;
}

#if WRITE_MAIN || SAMPLE_IMU
int setUpSD()
{
  Serial.print("Initializing SD card...  ");
  if (SD.begin(SD_PIN)) {

    Serial.println("initialization done");

    myFile = SD.open("start.txt", FILE_WRITE);
    if(myFile){
      myFile.println("system start");
      Serial.println("system start");
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

  serialDebug.println("--- Writing To Main File");

  char filename[13];
  strcpy(filename, "BUOY");
  strcat(filename, yyyy[array_index]);
  strcat(filename, ".txt");

  myFile = SD.open(filename, FILE_WRITE);
  if (myFile) {

    for(int i = 0; i < 2; i++){
      myFile.print(yyyy[i]); myFile.print(",");
      myFile.print(mM[i]); myFile.print(",");
      myFile.print(dd[i]); myFile.print(",");
      myFile.print(hh[i]); myFile.print(",");
      myFile.print(mm[i]); myFile.print(",");
      myFile.print(ss[i]); myFile.print(",");
      myFile.print(sats[i]); myFile.print(",");
      myFile.print(lat[i]); myFile.print(",");
      myFile.print(lon[i]); myFile.print(",");
      myFile.print(airTemp[i]); myFile.print(",");
      myFile.print(waterTemp[i]); myFile.print(",");
      myFile.print(conductivity[i]); myFile.print(",");
      myFile.print(dissolvedOxygen[i]); myFile.print(",");
      myFile.print(pH[i]); myFile.print(",");
      myFile.print(orp[i]); myFile.print(",");
      myFile.print(sinr[i]); myFile.print(",");
      myFile.print(rssi[i]); myFile.print(",");
      //myFile.print(pressure[i]); myFile.print(",");
      myFile.print(voltage[i]); myFile.print(",");
      myFile.print(wave_height); myFile.print(",");
      myFile.println(wave_period);
    }
    myFile.close();
    serialDebug.println("--- Main File Write Complete");
    return 0;
  }

  return -1;
}
#endif