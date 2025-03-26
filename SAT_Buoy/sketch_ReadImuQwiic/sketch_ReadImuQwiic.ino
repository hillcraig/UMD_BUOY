/**
 * Qwiic Mux Demo
 *
 */
#include "ICM_20948.h" // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU
#include <Wire.h>
#include <string.h>
#include <SD.h>
#define SERIAL_PORT Serial


// Parameters
const int MUX_ADDR = 0x70;
const int IMU_CHANNEL = 2;
const int NIR_CHANNEL = 1;

File imuFileRaw;

//USED FOR TESTING SD CARD
  // We didn't get a 3D fix so
        // set the lat, long etc. to default values
float agtVbat = 5.0; // Battery voltage
float agtLatitude = 0.0; // Latitude in degrees
float agtLongitude = 0.0; // Longitude in degrees
long  agtAltitude = 0; // Altitude above Median Seal Level in m
float agtSpeed = 0.0; // Ground speed in m/s
byte  agtSatellites = 0; // Number of satellites (SVs) used in the solution
long  agtCourse = 0; // Course (heading) in degrees
int   agtPDOP = 0;  // Positional Dilution of Precision in m
int   agtYear = 1970; // GNSS Year
byte  agtMonth = 1; // GNSS month
byte  agtDay = 1; // GNSS day
byte  agtHour = 0; // GNSS hours
byte  agtMinute = 0; // GNSS minutes
byte  agtSecond = 0; // GNSS seconds

// Pins
const int PIN_RESET = 9;
const int DC_JUMPER = 1;

#define WIRE_PORT Wire // Your desired Wire port.      Used when "USE_SPI" is not defined
#define AD0_VAL 1      // The value of the last bit of the I2C address.                \
                       // On the SparkFun 9DoF IMU breakout the default is 1, and when \
                       // the ADR jumper is closed the value becomes 0
// Constants

// Globals
ICM_20948_I2C myICM;
//int clock[2048];
 float accX[2048];
  float accY[2048];
  float accZ[2048];

  float magX[2048];
  float magY[2048];
  float magZ[2048];

  float gyrX[2048];
  float gyrY[2048];
  float gyrZ[2048];

void setup() {

  // Initialize communcation ports
  Serial.begin(115200);
  Wire.begin();

 




  // Initialize visible spectral sensor
  enableMuxPort(IMU_CHANNEL); 
  myICM.begin();
  disableMuxPort(IMU_CHANNEL);

 

  bool initialized = false;
  while (!initialized)
  {
    enableMuxPort(2);
/*#ifdef USE_SPI
    myICM.begin(CS_PIN, SPI_PORT, SPI_FREQ); // Here we are using the user-defined SPI_FREQ as the clock speed of the SPI bus
#else
    myICM.begin(WIRE_PORT, AD0_VAL);
#endif*/
    myICM.begin(Wire, 1);
    Serial.print(F("Initialization of the sensor returned: "));
    Serial.println(myICM.statusString());
    if (myICM.status != ICM_20948_Stat_Ok)
    {
      Serial.println("Trying again...");
      delay(500);
    
    }
    else
    {
    
      initialized = true;
      Serial.println("Device Connected!");
    }
    disableMuxPort(IMU_CHANNEL);
  }
  int i = 0;
}



void loop() {
  int i = 0;
  while(i<2048){
  enableMuxPort(IMU_CHANNEL);
  if (myICM.dataReady())
  {
    myICM.getAGMT();         // The values are only updated when you call 'getAGMT'
    convertRaw(myICM.agmt);
                                 //printRawAGMT( myICM.agmt );     // Uncomment this to see the raw values, taken directly from the agmt structure
    printScaledAGMT(&myICM); // This function takes into account the scale settings from when the measurement was made to calculate the values with units
    delay(30);
  }
  else
  {
    SERIAL_PORT.println("Waiting for data");
    delay(500);
  }



  
    fillMatrix(i, myICM.agmt);
    i++;
    Serial.println(i);
  
  }//end while
        disableMuxPort(IMU_CHANNEL);
          //save to SD card
        //"YYYYMMDD_HHMMSS_IMUraw"
        String fileName = String(agtYear) + String(agtMonth) + String(agtDay) + "_" + String(agtHour) + String(agtMinute) + String(agtSecond) + "_imuRaw.txt";
        Serial.println(fileName);  //Filename for raw imu values
        bool initialized;
        Serial.print("Initializing SD card...");

        if (!SD.begin(4)) {
        Serial.println("initialization failed!");
        initialized = false;
        while (1);
       }
       Serial.println("initialization done.");
       initialized = true;

          // open the file. note that only one file can be open at a time,
          // so you have to close this one before opening another.
          imuFileRaw = SD.open("fileName.txt", FILE_WRITE);

          // if the file opened okay, write to it:
          if (imuFileRaw) {
          Serial.print("Writing to file...");
          imuFileRaw.println(fileName + ":");
          printMatrix(imuFileRaw);
          imuFileRaw.println();
          // close the file:
          imuFileRaw.close();
          Serial.println("done.");
          } else {
          // if the file didn't open, print an error:
          Serial.println("error opening file");
        }

      // re-open the file for reading:
      imuFileRaw = SD.open("fileName.txt");
      if (imuFileRaw) {
        Serial.println(fileName + ":");

    // read from the file until there's nothing else in it:
    while (imuFileRaw.available()) {
      Serial.write(imuFileRaw.read());
    }
    // close the file:
    imuFileRaw.close();
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening file");
  }
  
delay(100000);
  
}



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

// Below here are some helper functions to print the data nicely!

void printPaddedInt16b(int16_t val)
{
  if (val > 0)
  {
    SERIAL_PORT.print(" ");
    if (val < 10000)
    {
      SERIAL_PORT.print("0");
    }
    if (val < 1000)
    {
      SERIAL_PORT.print("0");
    }
    if (val < 100)
    {
      SERIAL_PORT.print("0");
    }
    if (val < 10)
    {
      SERIAL_PORT.print("0");
    }
  }
  else
  {
    SERIAL_PORT.print("-");
    if (abs(val) < 10000)
    {
      SERIAL_PORT.print("0");
    }
    if (abs(val) < 1000)
    {
      SERIAL_PORT.print("0");
    }
    if (abs(val) < 100)
    {
      SERIAL_PORT.print("0");
    }
    if (abs(val) < 10)
    {
      SERIAL_PORT.print("0");
    }
  }
  SERIAL_PORT.print(abs(val));
}

void printRawAGMT(ICM_20948_AGMT_t agmt)
{
  SERIAL_PORT.print("RAW. Acc [ ");
  printPaddedInt16b(agmt.acc.axes.x);
  SERIAL_PORT.print(", ");
  printPaddedInt16b(agmt.acc.axes.y);
  SERIAL_PORT.print(", ");
  printPaddedInt16b(agmt.acc.axes.z);
  SERIAL_PORT.print(" ], Gyr [ ");
  printPaddedInt16b(agmt.gyr.axes.x);
  SERIAL_PORT.print(", ");
  printPaddedInt16b(agmt.gyr.axes.y);
  SERIAL_PORT.print(", ");
  printPaddedInt16b(agmt.gyr.axes.z);
  SERIAL_PORT.print(" ], Mag [ ");
  printPaddedInt16b(agmt.mag.axes.x);
  SERIAL_PORT.print(", ");
  printPaddedInt16b(agmt.mag.axes.y);
  SERIAL_PORT.print(", ");
  printPaddedInt16b(agmt.mag.axes.z);
  SERIAL_PORT.print(" ], Tmp [ ");
  printPaddedInt16b(agmt.tmp.val);
  SERIAL_PORT.print(" ]");
  SERIAL_PORT.println();
}
void convertRaw(ICM_20948_AGMT_t agmt){
  //constants
  int s2g = 16384;
  int s4g = 8192;
  int s16g = 2048;
  float gScale = 131.072;
  float mScale = 0.15;
  float g = 0.00981;  //milli g to m/s ^2 

  //acc
  agmt.acc.axes.x /= s2g;
  agmt.acc.axes.y /= s2g;
  agmt.acc.axes.z /= s2g;
  agmt.gyr.axes.x /= gScale;
  agmt.gyr.axes.y /= gScale;
  agmt.gyr.axes.z /= gScale;
  agmt.mag.axes.x *= mScale;
  agmt.mag.axes.y *= mScale;
  agmt.mag.axes.z *= mScale;
  
  
}


void printFormattedFloat(float val, uint8_t leading, uint8_t decimals)
{
  float aval = abs(val);
  if (val < 0)
  {
    SERIAL_PORT.print("-");
  }
  else
  {
    SERIAL_PORT.print(" ");
  }
  for (uint8_t indi = 0; indi < leading; indi++)
  {
    uint32_t tenpow = 0;
    if (indi < (leading - 1))
    {
      tenpow = 1;
    }
    for (uint8_t c = 0; c < (leading - 1 - indi); c++)
    {
      tenpow *= 10;
    }
    if (aval < tenpow)
    {
      SERIAL_PORT.print("0");
    }
    else
    {
      break;
    }
  }
  if (val < 0)
  {
    SERIAL_PORT.print(-val, decimals);
  }
  else
  {
    SERIAL_PORT.print(val, decimals);
  }
}

#ifdef USE_SPI
void printScaledAGMT(ICM_20948_SPI *sensor)
{
#else
void printScaledAGMT(ICM_20948_I2C *sensor)
{
#endif
  SERIAL_PORT.print("Scaled. Acc (mg) [ ");
  printFormattedFloat(sensor->accX(), 5, 2);
  SERIAL_PORT.print(", ");
  printFormattedFloat(sensor->accY(), 5, 2);
  SERIAL_PORT.print(", ");
  printFormattedFloat(sensor->accZ(), 5, 2);
  SERIAL_PORT.print(" ], Gyr (DPS) [ ");
  printFormattedFloat(sensor->gyrX(), 5, 2);
  SERIAL_PORT.print(", ");
  printFormattedFloat(sensor->gyrY(), 5, 2);
  SERIAL_PORT.print(", ");
  printFormattedFloat(sensor->gyrZ(), 5, 2);
  SERIAL_PORT.print(" ], Mag (uT) [ ");
  printFormattedFloat(sensor->magX(), 5, 2);
  SERIAL_PORT.print(", ");
  printFormattedFloat(sensor->magY(), 5, 2);
  SERIAL_PORT.print(", ");
  printFormattedFloat(sensor->magZ(), 5, 2);
  SERIAL_PORT.print(" ], Tmp (C) [ ");
  printFormattedFloat(sensor->temp(), 5, 2);
  SERIAL_PORT.print(" ]");
  SERIAL_PORT.println();
}

  void fillMatrix(int i, ICM_20948_AGMT_t agmt){
  
    //clock[i] = i;

    accX[i] = (agmt.acc.axes.x);
    accY[i] = (agmt.acc.axes.y);
    accZ[i] = (agmt.acc.axes.z);

    gyrX[i] = (agmt.gyr.axes.x);
    gyrY[i] = (agmt.gyr.axes.y);
    gyrZ[i] = (agmt.gyr.axes.z);

    magX[i] = (agmt.mag.axes.x);
    magY[i] = (agmt.mag.axes.y);
    magZ[i] = (agmt.mag.axes.z);
  
}

//prints the matrix within the imu file for raw values
void printMatrix(File& imuFileRaw){
  for(int y = 0; y < 2048; y++){
    imuFileRaw.print(accX[y]);
    imuFileRaw.print(", ");
    imuFileRaw.print(accY[y]);
    imuFileRaw.print(", ");
    imuFileRaw.print(accZ[y]);
    imuFileRaw.print(", ");
    imuFileRaw.print(gyrX[y]);
    imuFileRaw.print(", ");
    imuFileRaw.print(gyrY[y]);
    imuFileRaw.print(", ");
    imuFileRaw.print(gyrZ[y]);
    imuFileRaw.print(", ");
    imuFileRaw.print(magX[y]);
    imuFileRaw.print(", ");
    imuFileRaw.print(magY[y]);
    imuFileRaw.print(", ");
    imuFileRaw.print(magZ[y]);
    imuFileRaw.println();
    
  }
  return;
}



