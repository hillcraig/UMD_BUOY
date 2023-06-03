/* 
 *  LTE_Buoy.ino
 *  Liam Gaeuman @ University of Minnesota Duluth
 *  1/29/23
 *  
 *  This program collects the following every 5 min 
 *  and sends it to the cloud:
 *  location data
 *  time data
 *  9-axis motion data
 *  
 *
 *  To collect the motion data, a SparkFun 9DoF IMU Breakout - ICM-20948 (Qwiic)
 *  is used.
 *  https://www.sparkfun.com/products/15335
 *  sensor is connected through a multiplexer
 */

#include "Notecard.h"
#include "ICM_20948.h" 
#include <Wire.h>
#include <time.h> 
#define serialDebug Serial
#define productUID "com.gmail.liamgaeuman:environmental_monitoring_buoy" //insert your product UID here
#define NDEBUG 0

#ifndef ARDUINO_SWAN_R5
#error "This program was designed to run on the Blues Wireless Swan"
#endif

const int AD0_VAL = 1;
bool dataSent;  
size_t gps_time_s; //gps loc time
size_t old_time = 0; //stores last time data was sent
Notecard notecard; //notecard object
ICM_20948_I2C myICM;   //orientation sensor

// set ports for multiplexer
const int MUX_ADDR = 0x70;
const int IMU_port = 2; //IMU plugged into port 2 of mux


//- - - - - init data fields - - - - -
char yyyy[5];   //year   yyyy
char mM[3];     //month  mm
char dd[3];     //day    dd
char hh[3];     //hour   hh
char mm[3];     //minute mm
char ss[3];     //second ss
double lat;     //lat    ##.#####
double lon;     //lon    ##.#####
int sats;       //number of GPS satellites
double Ax, Ay, Az, Gx, Gy, Gz, Mx, My, Mz;

void setup() { 
  delay(2500); 
  serialDebug.begin(115200);
  notecard.begin();
  Wire.begin();
  Wire.setClock(400000);
  configureAllSensors();

  #if !NDEBUG
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

  notecard.logDebug("Setup complete\n");
}

void processData() {
  //get time fields
  setTimeVars();
  //collect sensor data
  collectSensorData();
  
  //stall until the difference of the old time 
  //and the new time is at least 5 minutes 
  J *rsp = notecard.requestAndResponse(notecard.newRequest("card.time"));
  size_t new_time = JGetInt(rsp, "time");
  NoteDeleteResponse(rsp);
    
  if(new_time - old_time < 300){
    notecard.logDebug("Waiting to send\n");        //calculate and stall the remaining
    delay(300*1000 - (new_time - old_time)*1000);  //time to make sure 5 min has passed
  }                                                
  
  rsp = notecard.requestAndResponse(notecard.newRequest("card.time"));
  old_time = JGetInt(rsp, "time");
  NoteDeleteResponse(rsp);

  //output to serial
  output();

  //send to cloud
  J *req = notecard.newRequest("note.add");   //create note request 
  if (req != NULL)
  {
    JAddStringToObject(req, "file", "sensors.qo");     
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
      JAddNumberToObject(body, "Ax", Ax); 
      JAddNumberToObject(body, "Ay", Ay);
      JAddNumberToObject(body, "Az", Az);
      JAddNumberToObject(body, "Gx", Gx);
      JAddNumberToObject(body, "Gy", Gy);
      JAddNumberToObject(body, "Gz", Gz);
      JAddNumberToObject(body, "Mx", Mx);
      JAddNumberToObject(body, "My", My);
      JAddNumberToObject(body, "Mz", Mz);              
    }
    if (!notecard.sendRequest(req)) {
      JDelete(req);
    }                         
  }
  
  serialDebug.println("data process complete.\n");
}

void loop() {
  dataSent = false; //send function has not been called
  {
    // Save the time from the last location reading.
    J *rsp = notecard.requestAndResponse(notecard.newRequest("card.location"));
    gps_time_s = JGetInt(rsp, "time");
    NoteDeleteResponse(rsp);
  }

  {
    // Set the location mode to "continuous" mode to force the
    // Notecard to take an immediate GPS/GNSS reading.
    J *req = notecard.newRequest("card.location.mode");
    JAddStringToObject(req, "mode", "continuous");
    notecard.sendRequest(req);
  }

  // How many seconds to wait for a location before you stop looking
  size_t timeout_s = 270; //4.5 min
  
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
      setLocationVars();
      NoteDeleteResponse(rsp);
      // Restore previous configuration
      {
        J *req = notecard.newRequest("card.location.mode");
        JAddStringToObject(req, "mode", "periodic");
        notecard.sendRequest(req);
      }
      //with location info acquired. go to phase 2
      notecard.logDebug("gps success, gathering data\n");
      dataSent = true;
      processData();
      break;
    }
  
    // If a "stop" field is on the card.location response, it means the Notecard
    // cannot locate a GPS/GNSS signal, so we break out of the loop to avoid looping
    // endlessly
    if (JGetObjectItem(rsp, "stop")) {
      NoteDeleteResponse(rsp);
      serialDebug.println("Found a stop flag, cannot find location\n");
      break;
    }
    // Wait 2 seconds before tryring again 
    delay(2000);
  }
  //if data was never sent, set gps fields to 0 and send
  if (!dataSent){
    noGPS();   
    processData();
  }
}

//- - - - - access data from time request - - - - -
void setTimeVars(){
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
  strftime(yyyy, sizeof(yyyy), "%Y", &ts); 
  strftime(mM, sizeof(mM), "%m", &ts); 
  strftime(dd, sizeof(dd), "%d", &ts); 
  strftime(hh, sizeof(hh), "%H", &ts); 
  strftime(mm, sizeof(mm), "%M", &ts); 
  strftime(ss, sizeof(ss), "%S", &ts); 

}

//- - - - - - - access data from location request - - - - -> 
void setLocationVars(){
  J *rsp = notecard.requestAndResponse(notecard.newRequest("card.location")); 
  if (rsp != NULL) {
    lon = JGetNumber(rsp, "lon"); 
    lon = (floor(100000*lon)/100000); 
    lat = JGetNumber(rsp, "lat");
    lat = (floor(100000*lat)/100000);
    sats = find_number_after_substring(JGetString(rsp, "status"), "R, ");
    notecard.deleteResponse(rsp);
  }  
}

//if GPS can't acquire location info, set all fields to 0
void noGPS(){
  lat=0;
  lon=0;
  sats=0;
}

//  - - - - - - - - - - - - - - - - - - - - - - - - -
//       functions to configure and use sensors  
//  - - - - - - - - - - - - - - - - - - - - - - - - -
void collectSensorData(){
  enableMuxPort(IMU_port);
  if (myICM.dataReady()){
    myICM.getAGMT();         // The values are only updated when you call 'getAGMT'
    printRawAGMT( myICM.agmt );     // Uncomment this to see the raw values, taken directly from the agmt structure
    //printScaledAGMT(&myICM); // This function takes into account the scale settings from when the measurement was made to calculate the values with units
    delay(30);
    
  }
  disableMuxPort(IMU_port);
}

void configureAllSensors(){
  enableMuxPort(IMU_port);
  while(true){
    myICM.begin(Wire, AD0_VAL);
    if (myICM.status == ICM_20948_Stat_Ok)
      break;
    delay(500);
  }
  disableMuxPort(IMU_port);
}

//  - - - - - - - - - - - - - - - - - - - 
//       functions to display output  
//  - - - - - - - - - - - - - - - - - - - 

void output(){
  serialDebug.println("- - - - - - - - - -");
  
  serialDebug.print("The date is: ");
  serialDebug.print(yyyy);
  serialDebug.print('/');
  serialDebug.print(mM);
  serialDebug.print('/');
  serialDebug.println(dd);
    
  serialDebug.print("The time is: ");
  serialDebug.print(hh);
  serialDebug.print(':');
  serialDebug.print(mm);
  serialDebug.print(':');
  serialDebug.println(ss);
    
  serialDebug.print("lat: ");
  serialDebug.println(lat); //output lat
  serialDebug.print("lon: ");
  serialDebug.println(lon); //output lon

  serialDebug.print("# of satellites: ");
  serialDebug.println(sats); //output number of satellites

  serialDebug.print("Ax: ");
  serialDebug.print(Ax); 
  serialDebug.print(", Ay: ");
  serialDebug.print(Ay); 
  serialDebug.print(", Az: ");
  serialDebug.println(Az); 

  serialDebug.print("Gx: ");
  serialDebug.print(Gx); 
  serialDebug.print(", Gy: ");
  serialDebug.print(Gy); 
  serialDebug.print(", Gz: ");
  serialDebug.println(Gz); 

  serialDebug.print("Mx: ");
  serialDebug.print(Mx); 
  serialDebug.print(", My: ");
  serialDebug.print(My); 
  serialDebug.print(", Mz: ");
  serialDebug.println(Mz); 
  
     
  
  serialDebug.println("- - - - - - - - - -"); 
}



void printRawAGMT(ICM_20948_AGMT_t agmt)
{
  Ax = (agmt.acc.axes.x);
  Ay = (agmt.acc.axes.y);
  Az = (agmt.acc.axes.z);
  Gx = (agmt.gyr.axes.x);
  Gy = (agmt.gyr.axes.y);
  Gz = (agmt.gyr.axes.z);
  Mx = (agmt.mag.axes.x);
  My = (agmt.mag.axes.y);
  Mz = (agmt.mag.axes.z);
}


//  - - - - - - - - - - parse # satellites - - - - - - - - - -

int find_number_after_substring(char* str, char* substring) {
  char* p = strstr(str, substring);
  if (p == NULL) {
    return -1;  // substring not found
  }
  p += strlen(substring);  // move past the substring

  // find the first digit after the substring
  while (*p != '\0' && !isdigit(*p)) {
    p++;
  }
  if (*p == '\0') {
    return -1;  // no number found after the substring
  }

  // find the end of the number
  char* end = p;
  while (*end != '\0' && *end != '/') {
    end++;
  }
  if (*end == '\0') {
    return -1;  // number not followed by '/'
  }

  // null-terminate the number string and convert it to an int
  *end = '\0';
  return atoi(p);
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
