/* 
 *  LTE_Buoy.ino
 *  Liam Gaeuman @ University of Minnesota Duluth
 *  1/29/23
 *  
 *  This program collects the follwing every 5 min 
 *  and sends it to the cloud:
 *  location data
 *  time data
 *  water temperature
 *  air temperature
 *  
 *  To collect the temperatures, two BlueRobitics Celuis
 *  Fast-Repsonse I2C sensors are used.
 *  https://bluerobotics.com/store/sensors-sonars-cameras/sensors/celsius-sensor-r1/
 */

#include "Notecard.h"
#include "TSYS01.h"
#include <time.h> 
#include <Wire.h>
#define serialDebug Serial
#define productUID "com.gmail.liamgaeuman:environmental_monitoring_buoy" //insert your product UID here
#define NDEBUG 0


#ifndef ARDUINO_SWAN_R5
#error "This program was designed to run on the Blues Wireless Swan"
#endif

bool dataSent;  
size_t gps_time_s; //gps loc time
size_t old_time = 0; //stores last time data was sent
Notecard notecard; //notecard object
TSYS01 airTempSensor;     //
TSYS01 waterTempSensor;     //

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
float temp_air; //air temperature 
float temp_wat; //water temperature 

// set ports for multiplexer
const int MUX_ADDR = 0x70;
const int TSYS01Port_1 = 0;
const int TSYS01Port_2 = 1;

void setup() { 
  delay(2500); 
  serialDebug.begin(115200);
  notecard.begin();
  Wire.begin();
  configureTempSensors(); //configure sensors

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
    }
    if (!notecard.sendRequest(req)) {
      JDelete(req);
    }                         
  }
  
  notecard.logDebug("data process complete.\n");
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
      notecard.logDebug("Timed out looking for a location\n");
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
      notecard.logDebug("Found a stop flag, cannot find location\n");
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

  serialDebug.print("Air Temp (C): ");
  serialDebug.println(temp_air); //output air temp
  serialDebug.print("Water Temp (C): ");
  serialDebug.println(temp_wat); //output water temp
  
  serialDebug.println("- - - - - - - - - -"); 
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

//  - - - - - - - - - - - - - - - - - - - - - - - - -
//       functions to configure and use sensors  
//  - - - - - - - - - - - - - - - - - - - - - - - - -

void configureTempSensors(){
  enableMuxPort(TSYS01Port_1);
  airTempSensor.init();
  disableMuxPort(TSYS01Port_1);

  enableMuxPort(TSYS01Port_2);
  waterTempSensor.init();
  disableMuxPort(TSYS01Port_2);
}

void collectSensorData(){
  enableMuxPort(TSYS01Port_1);
  airTempSensor.read();
  delay(40);
  temp_air = airTempSensor.temperature();  //temperature air
  disableMuxPort(TSYS01Port_1);

  enableMuxPort(TSYS01Port_2);
  waterTempSensor.read();
  delay(40);
  temp_wat = waterTempSensor.temperature();  //temperature water
  disableMuxPort(TSYS01Port_2);
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
