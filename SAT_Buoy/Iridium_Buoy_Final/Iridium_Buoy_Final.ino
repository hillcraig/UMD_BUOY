/*
  Artemis Global Tracker
  This code was adapted from the AGT Example 14 "Simple Tracker," written by Paul Clark. 
  
  Originally written by Paul Clark (PaulZC)
  September 7th 2021

  Modified by Chase Pheifer (using some functions and code created by Liam)
  Spring 2024
  Email: chasedpheifer@gmail.com

  ** Updated for v2.1.0 of the Apollo3 core / Artemis board package **
  ** (At the time of writing, v2.1.1 of the core conatins a feature which makes communication with the u-blox GNSS problematic. Be sure to use v2.1.0) **

  ** Set the Board to "RedBoard Artemis ATP" **
  ** (The Artemis Module does not have a Wire port defined, which prevents the GNSS library from compiling) **

  This example configures the Artemis Iridium Tracker as a simple
  GNSS + Iridium tracker. A 3D fix is taken from the ZOE-M8Q.
  Readings are taken from the sensors connected via the MUX, including the IMU. The bus voltage
  is measured. The raw IMU data is written to a file on the SD card.
  The other sensor readings are written to a file on the SD card. The data is transmitted every INTERVAL minutes
  via Iridium SBD. The Artemis goes into low power mode between transmissions.

  ** Fyi, I [Chase] cut the power LED on the AGT and on the IMU I was using to conserve power. This could be done on the Ezo circuits to also save some power.

  ** Make sure to connect the sensors to the correct MUX ports as defined near the beginning of this sketch or redefine which ports you connected the sensors to.

  ** If you get hardfaults when running a sketch on the AGT, it's likely the case that something or some pin is being put to sleep that shouldn't be. Also, the SD card would sometimes slightly slide out of its slot, causing no SD files to be written.

  ** Since we are using a solar charge controller and not connecting directly into the AGT's solar power ports, I am pretty sure we don't technically need the supercapacitors. 
    Also, I believe the 10F supercapacitors are currently connected to the AGT, but the jumper that switches from 150 mA (default) to 60 mA operation (for when using the AGT's solar power port and supercapacitors) has not been cut.
    I'm not sure if/what the 10F supercapacitors are doing without that jumper being severed.
    Additionally, I'm not certain what low battery voltage will do in our case of using the solar charge controller connected to the AGT 5V input. I don't know how the solar controller behaves when the battery voltage gets low.
    I currently still check the voltage of the input power to the AGT to ensure it's high enough for the super capacitor charger, though the solar controller might cut off power to the AGT when it can't supply 5V.

  ** I tried to make this sketch as low power as possible, i.e., putting most things to sleep and shutting down pins between each loop, following the original "Simple Tracker" example and incorporating our additional sensors
    That being said, no promises that this is the least power possible approach. But from our brief test, it (qualitatively) didn't seem like running out of battery before solar could recharge it was going to be a big issue. 
  
  The message is transmitted in text and is formatted for display on MathWork's ThingSpeak:
    time,air pressure (in Pa),relative humidity,air temp,water temp,EC,DO,pH,ORP,lat,lon,elevation,battery voltage

  The non-IMU data that is written to the SD card has the form of:
    Year,Month,Day,Hour,Minute,Second,Num of Satellites,Latitude,Longitude,Altitude,Speed,Course,Postional Dilution of Precision,Air Pressure,Relative Humidity,airTemp,waterTemp,conductivity,dissolvedOxygen,pH,ORP,Battery Voltage
  
  You will need to install version 3.0.5 of the Iridium SBD I2C library
  before this code will run successfully:
  https://github.com/sparkfun/SparkFun_IridiumSBD_I2C_Arduino_Library
  (Available through the Arduino Library Manager: search for IridiumSBDi2c)
  
  You will also need to install the Qwiic_PHT_MS8607_Library:
  https://github.com/sparkfun/SparkFun_PHT_MS8607_Arduino_Library
  (Available through the Arduino Library Manager: search for SparkFun MS8607)
  
  You will need to install the SparkFun u-blox library before this code will run successfully:
  https://github.com/sparkfun/SparkFun_u-blox_GNSS_Arduino_Library
  (Available through the Arduino Library Manager: search for SparkFun u-blox GNSS)
  
  SparkFun (and Chase :) ) labored with love to create this code. Feel like supporting open source hardware?
  Buy a board from SparkFun!

  Version history:
  Spring 2024
    Modifications made by Chase P. for Craig Hill's MN DNR project
  August 25th 2021
    Added a fix for https://github.com/sparkfun/Arduino_Apollo3/issues/423
  August 7th 2021
    Updated for v2.1 of the Apollo3 core
  December 15th, 2020:
    Adding the deep sleep code from OpenLog Artemis.
    Keep RAM powered up during sleep to prevent corruption above 64K.
    Restore busVoltagePin (Analog in) after deep sleep.

*/

// Artemis Tracker pin definitions
#define spiCS1 4             // D4 can be used as an SPI chip select or as a general purpose IO pin
#define geofencePin 10       // Input for the ZOE-M8Q's PIO14 (geofence) pin
#define busVoltagePin 13     // Bus voltage divided by 3 (Analog in)
#define iridiumSleep 17      // Iridium 9603N ON/OFF (sleep) pin: pull high to enable the 9603N
#define iridiumNA 18         // Input for the Iridium 9603N Network Available
#define LED 19               // White LED
#define iridiumPwrEN 22      // ADM4210 ON: pull high to enable power for the Iridium 9603N
#define gnssEN 26            // GNSS Enable: pull low to enable power for the GNSS (via Q2)
#define gnssBckpBatChgEN 44  // GNSS backup battery charge enable; when set as INPUT = disabled, when OUTPUT+LOW = charging.
#define superCapChgEN 27     // LTC3225 super capacitor charger: pull high to enable the super capacitor charger
#define superCapPGOOD 28     // Input for the LTC3225 super capacitor charger PGOOD signal
#define busVoltageMonEN 34   // Bus voltage monitor enable: pull high to enable bus voltage monitoring (via Q4 and Q3)
#define spiCS2 35            // D35 can be used as an SPI chip select or as a general purpose IO pin
#define iridiumRI 41         // Input for the Iridium 9603N Ring Indicator
// Make sure you do not have gnssEN and iridiumPwrEN enabled at the same time!
// If you do, bad things might happen to the AS179 RF switch!

// We use Serial1 to communicate with the Iridium modem. Serial1 on the ATP uses pin 24 for TX and 25 for RX. AGT uses the same pins.

#include <IridiumSBD.h>    //http://librarymanager/All#IridiumSBDI2C
#define DIAGNOSTICS false  // Change this to true to see IridiumSBD diagnostics
// Declare the IridiumSBD object (including the sleep (ON/OFF) and Ring Indicator pins)
IridiumSBD modem(Serial1, iridiumSleep, iridiumRI);

#include <Wire.h>  // Needed for I2C
const byte PIN_AGTWIRE_SCL = 8;
const byte PIN_AGTWIRE_SDA = 9;
TwoWire agtWire(PIN_AGTWIRE_SDA, PIN_AGTWIRE_SCL);  //Create an I2C port using pads 8 (SCL) and 9 (SDA)

#include "SparkFun_u-blox_GNSS_Arduino_Library.h"  //http://librarymanager/All#SparkFun_u-blox_GNSS
SFE_UBLOX_GNSS myGNSS;

#include <SparkFun_PHT_MS8607_Arduino_Library.h>  //http://librarymanager/All#SparkFun_MS8607
MS8607 barometricSensor;                          //Create an instance of the MS8607 object

#include "RTC.h"  //Include RTC library included with the Arduino_Apollo3 core

#include "TSYS01.h"
#include "ICM_20948.h"
// #include "Ezo_i2c.h" // using this library's functions to read from the Ezo sensors caused some hardfaults on the AGT, likely having to do with shutting down and starting up i2c pins. I get around using this library by manually doing what these functions do.
// #include "WaveProcessor.h" // For use when Liam's functions that process the raw IMU data and compute wave parameters
#include <SD.h>
// #include "am_mcu_apollo.h"  // for resetting the AGT when 'reset' is received via Iridium communications

//- - - - - declare I2C addresses - - - - - - - - -
const int MUX_ADDR = 0x70; // these should all be the default I2C addresses for each device
const int EC_ADDR = 100; 
const int DO_ADDR = 97;
const int PH_ADDR = 99;
const int ORP_ADDR = 98;
const int RTD_ADDR = 102;
const int ICM_address = 105;

// - - - - - set ports for multiplexer - - - - - - -
const int IMU_PORT = 7;
const int TSYS01_AIR_PORT = 0;
const int RTD_PORT = 1;
const int EC_PORT = 2; 
const int DO_PORT = 3;
const int PH_PORT = 4; 
const int ORP_PORT = 5;

//- - - - - - declare sensors - - - - - - - - - - -
TSYS01 airTempSensor;  //sensor used to collect air temperature
ICM_20948_I2C myICM;  //sensor for motion

//- - - - - - init data fields - - - - - - - - - - -
char yyyy[5];  //year   yyyy
char mM[3];    //month  mm
char dd[3];    //day    dd
char hh[3];    //hour   hh
char mm[3];    //minute mm
char ss[3];    //second ss
char doy[4];   //day of year
int doyInt;

float airTemp[2];
float waterTemp[2];
float conductivity[2];
float dissolvedOxygen[2];
float pH[2];
float ORP[2];

// More global variables
float agtVbat = 5.0;                   // Battery voltage
float reportVbat[2] = { 5.0, 5.0 };    // Battery voltage to report on SD card
float agtLatitude[2] = { 0.0, 0.0 };   // Latitude in degrees
float agtLongitude[2] = { 0.0, 0.0 };  // Longitude in degrees
long agtAltitude[2] = { 0, 0 };        // Altitude above Median Seal Level in m
float agtSpeed[2] = { 0.0, 0.0 };      // Ground speed in m/s
byte agtSatellites[2] = { 0, 0 };      // Number of satellites (SVs) used in the GNSS fix
long agtCourse[2] = { 0, 0 };          // Course (heading) in degrees
int agtPDOP[2] = { 0, 0 };             // Positional Dilution of Precision in m
int agtYear[2] = { 1970, 1970 };       // GNSS Year
byte agtMonth[2] = { 1, 1 };           // GNSS month
byte agtDay[2] = { 1, 1 };             // GNSS day
byte agtHour[2] = { 0, 0 };            // GNSS hours
byte agtMinute[2] = { 0, 0 };          // GNSS minutes
byte agtSecond[2] = { 0, 0 };          // GNSS seconds
int agtMilliseconds[2] = { 0, 0 };     // GNSS milliseconds
float agtPascals[2] = {0.0,0.0};       // Atmospheric pressure in Pascals
float agtHumidity[2] = {0.0,0.0};      // Relative Humidity [%]
byte agtFixType = 0;                   // GNSS fix type: 0=No fix, 1=Dead reckoning, 2=2D, 3=3D, 4=GNSS+Dead reckoning
bool agtPGOOD = false;                 // Flag to indicate if LTC3225 PGOOD is HIGH
int agtErr;                            // Error value returned by IridiumSBD.begin
unsigned long gnssMillis;              // millisecond timestamp when first GNSS datetime is recorded

struct DateTime {
  int year;
  int month;
  int day;
  int hour;
  int minute;
  int second;
};

//- - - - - - other variables - - - - - - - -
const byte SD_PIN = spiCS1;             // set pin SD card is connected to
const byte LED_PIN = 43;                // set pin that external LED is connected to LED
const int Ezo_samples = 10;             // number of samples to take for each Atlas Ezo sensor
const float GRAVITY = 9.8;              //
const int IMU_SAMPLE_SIZE = 4608;       // number of samples to take from the IMU, which is set to read with a freq of 4 Hz.
File myFile;                            //
int reading_num;                        // variable that is set based on whether it's the 1st or 2nd time taking sensor readings in a given loop
byte lowVBatCount = 0;                  // variable for counting number of times in a row we have a low battery. It is reset if a loop goes to sleep for any other reason than a low battery, as that would mean a low battery did not occur twice in a row.
const byte lowVBatCountLimit = 2;       // if a low battery is encountered lowVBatCountLimit times in a row, go into survival mode unto the battery voltage is high enough
byte Ezo_code;                          // variable to hold error code from Atlas Ezo sensor readings
int powered9603;                        // variable to store whether the 9603 powers up correctly
// uint8_t rxBuffer[20];                   // define an array to store received messages from Iridium satellite
// size_t bufferSize = sizeof(rxBuffer);   // Get the size of the buffer

//error state blink enum ->
enum states { VOLT_STATE,
              SETUP_STATE,
              GPS_STATE,
              COLLECT_DATA_STATE,
              IMU_STATE,
              WRITE_STATE,
              CAPACITOR_STATE,
              SEND_STATE };
// --------------------------------------------

// Define how often messages are sent in SECONDS
// This is the _quickest_ messages will be sent. Could be much slower than this depending on:
// capacitor charge time; gnss fix time; Iridium timeout; etc.
unsigned long INTERVAL = 30 * 60;       // 30 minutes
unsigned long SURVIVAL = 6 * 60 * 60;  // 6 hours

// Use this to keep a count of the second alarms from the rtc
volatile unsigned long secondsCount = 0;

// This flag indicates an interval alarm has occurred
volatile bool intervalAlarm = false;
volatile bool survivalAlarm = false;

// sendAttemptCounter is incremented each time a transmission is attempted.
// It helps keep track of whether messages are being sent successfully.
// It also indicates if the tracker has been reset (the count will go back to zero).
long sendAttemptCounter = 0;
long loopCounter = 0;

#define VBAT_LOW 2.8  // Minimum voltage for LTC3225

// Timeout after this many _minutes_ when waiting for a 3D GNSS fix
// (UL = unsigned long)
#define GNSS_timeout 5UL

// Timeout after this many _minutes_ when waiting for the super capacitors to charge
// 1 min should be OK for 1F capacitors at 150mA.
// Charging 10F capacitors at 60mA can take a long time! Could be as much as 10 mins.
#define CHG_timeout 2UL

// Top up the super capacitors for this many _seconds_.
// 10 seconds should be OK for 1F capacitors at 150mA.
// Increase the value for 10F capacitors.
#define TOPUP_timeout 10UL

// Loop Steps - these are used by the switch/case in the main loop
// This structure makes it easy to go from any of the steps directly to zzz when (e.g.) the batteries are low
typedef enum {
  loop_init = 0,   // Send the welcome message, check the battery voltage
  check_messages,  // Check if a reset message has been sent from land
  start_GNSS1,     // Enable the ZOE-M8Q, check the battery voltage
  read_GNSS1,      // Wait for up to GNSS_timeout minutes for a valid 3D fix, check the battery voltage
  read_sensors1,   // Read the pressure and temperature from the MS8607
  start_GNSS2,     // Enable the ZOE-M8Q, check the battery voltage
  read_GNSS2,      // Wait for up to GNSS_timeout minutes for a valid 3D fix, check the battery voltage
  read_sensors2,   // Read the pressure and temperature from the MS8607
  writeToSD,       // Write GPS and measurements to SD card
  start_LTC3225,   // Enable the LTC3225 super capacitor charger and wait for up to CHG_timeout minutes for PGOOD to go high
  wait_LTC3225,    // Wait TOPUP_timeout seconds to make sure the capacitors are fully charged
  start_9603,      // Power on the 9603N, send the message, check the battery voltage
  zzz,             // Turn everything off and put the processor into deep sleep
  wakeUp,          // Wake from deep sleep, restore the processor clock speed
  survival_mode    // If a low battery is encountered lowVBatCountLimit times in a row, go into deep sleep survival mode until the battery voltage is high enough
} loop_steps;
loop_steps loop_step = loop_init;  // Make sure loop_step is set to loop_init

// RTC alarm Interrupt Service Routine
// Clear the interrupt flag and increment seconds_count
// If INTERVAL has been reached, set the interval_alarm flag and reset seconds_count
// (Always keep ISRs as short as possible, don't do anything clever in them,
//  and always use volatile variables if the main loop needs to access them too.)
extern "C" void am_rtc_isr(void) {
  // Clear the RTC alarm interrupt
  rtc.clearInterrupt();

  // Increment seconds_count
  secondsCount = secondsCount + 1;

  if (lowVBatCount == lowVBatCountLimit) {
    // Check if intervalAlarm should be set
    if (secondsCount >= SURVIVAL) {
      survivalAlarm = true;
      secondsCount = 0;
    }
  } else {
    // Check if intervalAlarm should be set
    if (secondsCount >= INTERVAL) {
      intervalAlarm = true;
      secondsCount = 0;
    }
  }
}

// Get the battery (bus) voltage
// Enable the bus voltage monitor
// Read the bus voltage and store it in agtVbat
// Disable the bus voltage monitor to save power
// Converts the analogread into Volts, compensating for
// the voltage divider (/3) and the Apollo voltage reference (2.0V)
// Include a correction factor of 1.09 to correct for the divider impedance
void getVbat() {
  digitalWrite(busVoltageMonEN, HIGH);  // Enable the bus voltage monitor
  //analogReadResolution(14); //Set resolution to 14 bit
  delay(1);  // Let the voltage settle
  agtVbat = ((float)analogRead(busVoltagePin)) * 3.0 * 1.09 * 2.0 / 16384.0;
  digitalWrite(busVoltageMonEN, LOW);  // Disable the bus voltage monitor
}

// IridiumSBD Callback - this code is called while the 9603N is trying to transmit
bool ISBDCallback() {

  // Check the battery voltage now we are drawing current for the 9603
  // If voltage is low, stop Iridium send
  getVbat();  // Read the battery (bus) voltage
  if (agtVbat < VBAT_LOW) {
    Serial.print(F("*** LOW VOLTAGE (ISBDCallback) "));
    Serial.print(agtVbat, 2);
    Serial.println(F("V ***"));
    blinkLED(-1);
    delay(500);
    blinkLED(VOLT_STATE);
    lowVBatCount++;
    return false;  // Returning false causes IridiumSBD to terminate
  } else {
    return true;
  }
}

void gnssON(void)  // Enable power for the GNSS
{
  am_hal_gpio_pincfg_t pinCfg = g_AM_HAL_GPIO_OUTPUT;  // Begin by making the gnssEN pin an open-drain output
  pinCfg.eGPOutcfg = AM_HAL_GPIO_PIN_OUTCFG_OPENDRAIN;
  pin_config(PinName(gnssEN), pinCfg);
  delay(1);

  digitalWrite(gnssEN, LOW);  // Enable GNSS power (HIGH = disable; LOW = enable)
}

void gnssOFF(void)  // Disable power for the GNSS
{
  am_hal_gpio_pincfg_t pinCfg = g_AM_HAL_GPIO_OUTPUT;  // Begin by making the gnssEN pin an open-drain output
  pinCfg.eGPOutcfg = AM_HAL_GPIO_PIN_OUTCFG_OPENDRAIN;
  pin_config(PinName(gnssEN), pinCfg);
  delay(1);

  digitalWrite(gnssEN, HIGH);  // Disable GNSS power (HIGH = disable; LOW = enable)
}

// Overwrite the IridiumSBD beginSerialPort function - a fix for https://github.com/sparkfun/Arduino_Apollo3/issues/423
void IridiumSBD::beginSerialPort()  // Start the serial port connected to the satellite modem
{
  diagprint(F("custom IridiumSBD::beginSerialPort\r\n"));

  // Configure the standard ATP pins for UART1 TX and RX - endSerialPort may have disabled the RX pin

  am_hal_gpio_pincfg_t pinConfigTx = g_AM_BSP_GPIO_COM_UART_TX;
  pinConfigTx.uFuncSel = AM_HAL_PIN_24_UART1TX;
  pin_config(D24, pinConfigTx);

  am_hal_gpio_pincfg_t pinConfigRx = g_AM_BSP_GPIO_COM_UART_RX;
  pinConfigRx.uFuncSel = AM_HAL_PIN_25_UART1RX;
  pinConfigRx.ePullup = AM_HAL_GPIO_PIN_PULLUP_WEAK;  // Put a weak pull-up on the Rx pin
  pin_config(D25, pinConfigRx);

  Serial1.begin(19200);
}

// Overwrite the IridiumSBD endSerialPort function - a fix for https://github.com/sparkfun/Arduino_Apollo3/issues/423
void IridiumSBD::endSerialPort() {
  diagprint(F("custom IridiumSBD::endSerialPort\r\n"));

  // Disable the Serial1 RX pin to avoid the code hang
  am_hal_gpio_pinconfig(PinName(D25), g_AM_HAL_GPIO_DISABLE);
}

void setup() {
  // Let's begin by setting up the I/O pins

  pinMode(LED, OUTPUT);  // Make the LED pin an output

  gnssOFF();                         // Disable power for the GNSS
  pinMode(gnssBckpBatChgEN, INPUT);  // GNSS backup batttery charge control; input = disable charging; output+low=charging.
  pinMode(geofencePin, INPUT);       // Configure the geofence pin as an input

  pinMode(iridiumPwrEN, OUTPUT);     // Configure the Iridium Power Pin (connected to the ADM4210 ON pin)
  digitalWrite(iridiumPwrEN, LOW);   // Disable Iridium Power (HIGH = enable; LOW = disable)
  pinMode(superCapChgEN, OUTPUT);    // Configure the super capacitor charger enable pin (connected to LTC3225 !SHDN)
  digitalWrite(superCapChgEN, LOW);  // Disable the super capacitor charger (HIGH = enable; LOW = disable)
  pinMode(iridiumSleep, OUTPUT);     // Iridium 9603N On/Off (Sleep) pin
  digitalWrite(iridiumSleep, LOW);   // Put the Iridium 9603N to sleep (HIGH = on; LOW = off/sleep)
  pinMode(iridiumRI, INPUT);         // Configure the Iridium Ring Indicator as an input
  pinMode(iridiumNA, INPUT);         // Configure the Iridium Network Available as an input
  pinMode(superCapPGOOD, INPUT);     // Configure the super capacitor charger PGOOD input

  // Make sure the Serial1 RX pin is disabled to prevent the power-on glitch on the modem's RX(OUT) pin
  // causing problems with v2.1.0 of the Apollo3 core. Requires v3.0.5 of IridiumSBDi2c.
  modem.endSerialPort();

  pinMode(busVoltageMonEN, OUTPUT);    // Make the Bus Voltage Monitor Enable an output
  digitalWrite(busVoltageMonEN, LOW);  // Set it low to disable the measurement to save power
  analogReadResolution(14);            //Set resolution to 14 bit

  // Initialise the globals
  sendAttemptCounter = 0;  // Make sure sendAttemptCounter is set to zero (indicating a reset)
  loopCounter = 0;
  loop_step = loop_init;  // Make sure loop_step is set to loop_init
  secondsCount = 0;       // Make sure seconds_count is reset
  intervalAlarm = false;  // Make sure the interval alarm flag is clear
  survivalAlarm = false;

  // Set up the RTC for 1 second interrupts
  /*
    0: Alarm interrupt disabled
    1: Alarm match every year   (hundredths, seconds, minutes, hour, day, month)
    2: Alarm match every month  (hundredths, seconds, minutes, hours, day)
    3: Alarm match every week   (hundredths, seconds, minutes, hours, weekday)
    4: Alarm match every day    (hundredths, seconds, minute, hours)
    5: Alarm match every hour   (hundredths, seconds, minutes)
    6: Alarm match every minute (hundredths, seconds)
    7: Alarm match every second (hundredths)
  */
  rtc.setAlarmMode(7);    // Set the RTC alarm mode
  rtc.attachInterrupt();  // Attach RTC alarm interrupt
}

void loop() {
  // loop is one large switch/case that controls the sequencing of the code
  switch (loop_step) {

    // ************************************************************************************************
    // Initialise things
    case loop_init:

      // Start the console serial port and send the welcome message
      Serial.begin(115200);
      delay(1000);  // Wait for the user to open the serial monitor (extend this delay if you need more time)
      Serial.println();
      Serial.println();
      Serial.println(F("Artemis Global Tracker"));
      Serial.println(F("Iridium Buoy"));
      Serial.print(F("Send count:"));
      Serial.print(sendAttemptCounter);
      Serial.println();
      loopCounter = loopCounter + 1;
      Wire.begin();

      for (byte x = 0 ; x <= 7 ; x++)
      {
        disableMuxPort(x);
      }

      configureSensors();
      setUpSD();

      Serial.print(F("Using an INTERVAL of "));
      Serial.print(INTERVAL);
      Serial.println(F(" seconds"));

      // Setup the IridiumSBD
      modem.setPowerProfile(IridiumSBD::USB_POWER_PROFILE);  // Change power profile to "low current"
      // modem.setPowerProfile(IridiumSBD::DEFAULT_POWER_PROFILE); // this is the other option. I think our application would be defined as low current, but I haven't found a clear definition of that.

      // Check battery voltage
      // If voltage is low, go to sleep
      getVbat();  // Get the battery (bus) voltage
      if (agtVbat < VBAT_LOW) {
        Serial.print(F("*** LOW VOLTAGE (init) "));
        Serial.print(agtVbat, 2);
        Serial.println(F(" ***"));
        blinkLED(-1);
        delay(500);
        blinkLED(VOLT_STATE);
        lowVBatCount++;
        if (lowVBatCount == lowVBatCountLimit){
          loop_step = survival_mode;
        } else {
          loop_step = zzz;  // Go to sleep
        }
      } else {
        loop_step = start_GNSS1;
      }

      // Initialize values to 'NaN,' i.e., -9999; 
      for (int z = 0; z < 2; z++) {
        agtPascals[z] = -9999; 
        agtHumidity[z] = -9999; 
        airTemp[z] = -9999; 
        waterTemp[z] = -9999; 
        conductivity[z] = -9999; 
        dissolvedOxygen[z] = -9999; 
        pH[z] = -9999; 
        ORP[z] = -9999; 
    }

      break;  // End of case loop_init

    // ************************************************************************************************
    // Power up the GNSS (ZOE-M8Q)
    case start_GNSS1:
      blinkLED(100);
      reading_num = 0; // first time through GPS and sensors gets put into first index

      Serial.println(F("Powering up the GNSS..."));

      gnssON();  // Enable power for the GNSS

      pinMode(gnssBckpBatChgEN, OUTPUT);  // GNSS backup batttery charge control; output + low = enable charging
      digitalWrite(gnssBckpBatChgEN, LOW);
      //pinMode(gnssBckpBatChgEN, INPUT); // GNSS backup batttery charge control; input = disable charging

      delay(2000);  // Give it time to power up

      agtWire.begin();           // Set up the I2C pins
      agtWire.setClock(100000);  // Use 100kHz for best performance
      setAGTWirePullups(0);      // Remove the pull-ups from the I2C pins (internal to the Artemis) for best performance

      // Check battery voltage now we are drawing current for the GNSS
      getVbat();  // Get the battery (bus) voltage
      if (agtVbat < VBAT_LOW) {
        // If voltage is low, turn off the GNSS and go to sleep
        Serial.print(F("*** LOW VOLTAGE (start_GNSS) "));
        Serial.print(agtVbat, 2);
        Serial.println(F("V ***"));
        blinkLED(-1);
        delay(500);
        blinkLED(VOLT_STATE);
        gnssOFF();  // Disable power for the GNSS
        lowVBatCount++;
        if (lowVBatCount == lowVBatCountLimit) {
          loop_step = survival_mode;
        } else {
          loop_step = zzz;  // Go to sleep
        }
      }

      else {  // If the battery voltage is OK

        if (myGNSS.begin(agtWire) == false)  //Connect to the u-blox module using agtWire port
        {
          // If we were unable to connect to the ZOE-M8Q:

          // Send a warning message
          Serial.println(F("*** Ublox GNSS not detected at default I2C address ***"));
          blinkLED(-1);
          delay(500);
          blinkLED(GPS_STATE);

          // Set the lat, long etc. to default values
          agtLatitude[reading_num] = 0.0;    // Latitude in degrees
          agtLongitude[reading_num] = 0.0;   // Longitude in degrees
          agtAltitude[reading_num] = 0;      // Altitude above Median Seal Level in m
          agtSpeed[reading_num] = 0.0;       // Ground speed in m/s
          agtSatellites[reading_num] = 0;    // Number of satellites (SVs) used in the solution
          agtCourse[reading_num] = 0;        // Course (heading) in degrees
          agtPDOP[reading_num] = 0;          // Positional Dilution of Precision in m
          agtYear[reading_num] = 1970;       // GNSS Year
          agtMonth[reading_num] = 1;         // GNSS month
          agtDay[reading_num] = 1;           // GNSS day
          agtHour[reading_num] = 0;          // GNSS hours
          agtMinute[reading_num] = 0;        // GNSS minutes
          agtSecond[reading_num] = 0;        // GNSS seconds
          agtMilliseconds[reading_num] = 0;  // GNSS milliseconds

          // Power down the GNSS
          gnssOFF();  // Disable power for the GNSS

          loop_step = read_sensors1;  // Move on, skip reading the GNSS fix
        }

        else {  // If the GNSS started up OK

          //myGNSS.enableDebugging(); // Enable debug messages
          myGNSS.setI2COutput(COM_TYPE_UBX);  // Limit I2C output to UBX (disable the NMEA noise)

          // If we are going to change the dynamic platform model, let's do it here.
          // Possible values are:
          // PORTABLE, STATIONARY, PEDESTRIAN, AUTOMOTIVE, SEA, AIRBORNE1g, AIRBORNE2g, AIRBORNE4g, WRIST, BIKE
          if (myGNSS.setDynamicModel(DYN_MODEL_SEA) == false) {
            Serial.println(F("*** Warning: setDynamicModel may have failed ***"));
          } else {
            Serial.println(F("Dynamic Model updated"));
          }

          loop_step = read_GNSS1;  // Move on, read the GNSS fix
        }
      }

      break;  // End of case start_GNSS1

    // ************************************************************************************************
    // Read a fix from the ZOE-M8Q. Could maybe make this a fxn.
    case read_GNSS1:

      Serial.println(F("Waiting for a 3D GNSS fix..."));

      agtFixType = 0;  // Clear the fix type

      // Look for GNSS signal for up to GNSS_timeout minutes
      for (unsigned long tnow = millis(); (agtFixType != 3) && ((millis() - tnow) < (GNSS_timeout * 60UL * 1000UL));) {

        agtFixType = myGNSS.getFixType();  // Get the GNSS fix type

        // Check battery voltage now we are drawing current for the GNSS
        // If voltage is low, stop looking for GNSS and go to sleep
        getVbat();
        if (agtVbat < VBAT_LOW) {
          break;  // Exit the for loop now
        }

        delay(100);  // Don't pound the I2C bus too hard!
      }

      // If voltage is low then go straight to sleep
      if (agtVbat < VBAT_LOW) {
        Serial.print(F("*** LOW VOLTAGE (read_GNSS) "));
        Serial.print(agtVbat, 2);
        Serial.println(F("V ***"));
        blinkLED(-1);
        delay(500);
        blinkLED(VOLT_STATE);
        lowVBatCount++;
        if (lowVBatCount == lowVBatCountLimit) {
          loop_step = survival_mode;
        } else {
          loop_step = zzz;  // Go to sleep
        }
      }

      else if (agtFixType == 3)  // Check if we got a valid 3D fix
      {
        gnssMillis = millis();  // needed for IMU timestamp timing
        // Get the time and position etc.
        // Get the time first to hopefully avoid second roll-over problems
        agtMilliseconds[reading_num] = myGNSS.getMillisecond();
        agtSecond[reading_num] = myGNSS.getSecond();
        agtMinute[reading_num] = myGNSS.getMinute();
        agtHour[reading_num] = myGNSS.getHour();
        agtDay[reading_num] = myGNSS.getDay();
        agtMonth[reading_num] = myGNSS.getMonth();
        agtYear[reading_num] = myGNSS.getYear();                                // Get the year
        agtLatitude[reading_num] = (float)myGNSS.getLatitude() / 10000000.0;    // Get the latitude in degrees
        agtLongitude[reading_num] = (float)myGNSS.getLongitude() / 10000000.0;  // Get the longitude in degrees
        agtAltitude[reading_num] = myGNSS.getAltitudeMSL() / 1000;              // Get the altitude above Mean Sea Level in m
        agtSpeed[reading_num] = (float)myGNSS.getGroundSpeed() / 1000.0;        // Get the ground speed in m/s
        agtSatellites[reading_num] = myGNSS.getSIV();                           // Get the number of satellites used in the fix
        agtCourse[reading_num] = myGNSS.getHeading() / 10000000;                // Get the heading in degrees
        agtPDOP[reading_num] = myGNSS.getPDOP() / 100;                          // Get the PDOP in m

        Serial.println(F("A 3D fix was found!"));
        Serial.print(F("Latitude (degrees): "));
        Serial.println(agtLatitude[reading_num], 6);
        Serial.print(F("Longitude (degrees): "));
        Serial.println(agtLongitude[reading_num], 6);
        Serial.print(F("Altitude (m): "));
        Serial.println(agtAltitude[reading_num]);

        loop_step = read_sensors1;  // Move on, read the pressure and temperature
      }

      else {
        // We didn't get a 3D fix so
        // set the lat, long etc. to default values
        agtLatitude[reading_num] = 0.0;    // Latitude in degrees
        agtLongitude[reading_num] = 0.0;   // Longitude in degrees
        agtAltitude[reading_num] = 0;      // Altitude above Median Seal Level in m
        agtSpeed[reading_num] = 0.0;       // Ground speed in m/s
        agtSatellites[reading_num] = 0;    // Number of satellites (SVs) used in the solution
        agtCourse[reading_num] = 0;        // Course (heading) in degrees
        agtPDOP[reading_num] = 0;          // Positional Dilution of Precision in m
        agtYear[reading_num] = 1970;       // GNSS Year
        agtMonth[reading_num] = 1;         // GNSS month
        agtDay[reading_num] = 1;           // GNSS day
        agtHour[reading_num] = 0;          // GNSS hours
        agtMinute[reading_num] = 0;        // GNSS minutes
        agtSecond[reading_num] = 0;        // GNSS seconds
        agtMilliseconds[reading_num] = 0;  // GNSS milliseconds

        Serial.println(F("A 3D fix was NOT found!"));
        Serial.println(F("Using default values..."));
        blinkLED(-1);
        delay(500);
        blinkLED(GPS_STATE);

        loop_step = read_sensors1;  // Move on, read the pressure and temperature
      }

      // Power down the GNSS
      Serial.println(F("Powering down the GNSS..."));
      gnssOFF();  // Disable power for the GNSS

      break;  // End of case read_GNSS1

    // ************************************************************************************************
    // Read the pressure and temperature from the MS8607
    case read_sensors1:

      setAGTWirePullups(1); // MS8607 needs pull-ups

      bool barometricSensorOK;

      barometricSensorOK = barometricSensor.begin(agtWire); // Begin the PHT sensor
      if (barometricSensorOK == false)
      {
        // Send a warning message if we were unable to connect to the MS8607:
        Serial.println(F("*** Could not detect the MS8607 sensor. Trying again... ***"));
        barometricSensorOK = barometricSensor.begin(agtWire); // Re-begin the PHT sensor
        if (barometricSensorOK == false)
        {
          // Send a warning message if we were unable to connect to the MS8607:
          Serial.println(F("*** MS8607 sensor not detected at default I2C address ***"));

          blinkLED(-1);
          delay(500);
          blinkLED(COLLECT_DATA_STATE);
        }
      }

      agtPascals[reading_num] = barometricSensor.getPressure() * 100.0; // Convert pressure from hPa to Pascals
      agtHumidity[reading_num] = barometricSensor.getHumidity();

      Serial.print(F("Pressure="));
      Serial.print(agtPascals[reading_num], 1);
      Serial.println(F("(Pa)"));

      Serial.print("Humidity=");
      Serial.print(agtHumidity[reading_num], 1);
      Serial.print("(%RH)");

      setAGTWirePullups(0); // Disable pull-ups

      blinkLED(100);
      // Record battery voltage
      getVbat();
      reportVbat[reading_num] = agtVbat;

      // Grab data from temp sensors and/or WQ sensors
      Serial.println(F("Getting the temp and WQ readings..."));
      if (collectOtherData() == -1){
        blinkLED(-1);
        delay(500);
        blinkLED(COLLECT_DATA_STATE);
      }
      Serial.print(F("TSYS Air Temp="));
      Serial.print(airTemp[reading_num], 1);
      Serial.println(F("(C)"));

      blinkLED(100);
      Serial.println(F("Sampling the IMU..."));
      if (sample_IMU() == -1){
        blinkLED(-1);
        delay(500);
        blinkLED(IMU_STATE);
      }

      loop_step = start_GNSS2; // Move on, get GNSS fix

      break;  // End of case read_sensors1

      // ************************************************************************************************
    // Power up the GNSS (ZOE-M8Q)
    case start_GNSS2:
      blinkLED(100);
      reading_num = 1;  // second time through GPS and sensors gets put into second index
      Serial.println(F("Powering up the GNSS..."));

      gnssON();  // Enable power for the GNSS

      pinMode(gnssBckpBatChgEN, OUTPUT);  // GNSS backup batttery charge control; output + low = enable charging
      digitalWrite(gnssBckpBatChgEN, LOW);
      //pinMode(gnssBckpBatChgEN, INPUT); // GNSS backup batttery charge control; input = disable charging

      delay(2000);  // Give it time to power up

      agtWire.begin();           // Set up the I2C pins
      agtWire.setClock(100000);  // Use 100kHz for best performance
      setAGTWirePullups(0);      // Remove the pull-ups from the I2C pins (internal to the Artemis) for best performance

      // Check battery voltage now we are drawing current for the GNSS
      getVbat();  // Get the battery (bus) voltage
      if (agtVbat < VBAT_LOW) {
        // If voltage is low, turn off the GNSS and go to sleep
        Serial.print(F("*** LOW VOLTAGE (start_GNSS) "));
        Serial.print(agtVbat, 2);
        Serial.println(F("V ***"));
        gnssOFF();  // Disable power for the GNSS
        blinkLED(-1);
        delay(500);
        blinkLED(VOLT_STATE);
        lowVBatCount++;
        if (lowVBatCount == lowVBatCountLimit) {
          loop_step = survival_mode;
        } else {
          loop_step = zzz;  // Go to sleep
        }
      }

      else {  // If the battery voltage is OK

        if (myGNSS.begin(agtWire) == false)  //Connect to the u-blox module using agtWire port
        {
          // If we were unable to connect to the ZOE-M8Q:

          // Send a warning message
          Serial.println(F("*** Ublox GNSS not detected at default I2C address ***"));
          blinkLED(-1);
          delay(500);
          blinkLED(GPS_STATE);

          // Set the lat, long etc. to default values
          agtLatitude[reading_num] = 0.0;    // Latitude in degrees
          agtLongitude[reading_num] = 0.0;   // Longitude in degrees
          agtAltitude[reading_num] = 0;      // Altitude above Median Seal Level in m
          agtSpeed[reading_num] = 0.0;       // Ground speed in m/s
          agtSatellites[reading_num] = 0;    // Number of satellites (SVs) used in the solution
          agtCourse[reading_num] = 0;        // Course (heading) in degrees
          agtPDOP[reading_num] = 0;          // Positional Dilution of Precision in m
          agtYear[reading_num] = 1970;       // GNSS Year
          agtMonth[reading_num] = 1;         // GNSS month
          agtDay[reading_num] = 1;           // GNSS day
          agtHour[reading_num] = 0;          // GNSS hours
          agtMinute[reading_num] = 0;        // GNSS minutes
          agtSecond[reading_num] = 0;        // GNSS seconds
          agtMilliseconds[reading_num] = 0;  // GNSS milliseconds

          // Power down the GNSS
          gnssOFF();  // Disable power for the GNSS

          loop_step = read_sensors2;  // Move on, skip reading the GNSS fix
        }

        else {  // If the GNSS started up OK

          //myGNSS.enableDebugging(); // Enable debug messages
          myGNSS.setI2COutput(COM_TYPE_UBX);  // Limit I2C output to UBX (disable the NMEA noise)

          // If we are going to change the dynamic platform model, let's do it here.
          // Possible values are:
          // PORTABLE, STATIONARY, PEDESTRIAN, AUTOMOTIVE, SEA, AIRBORNE1g, AIRBORNE2g, AIRBORNE4g, WRIST, BIKE
          if (myGNSS.setDynamicModel(DYN_MODEL_SEA) == false) {
            Serial.println(F("*** Warning: setDynamicModel may have failed ***"));
          } else {
            Serial.println(F("Dynamic Model updated"));
          }

          loop_step = read_GNSS2;  // Move on, read the GNSS fix
        }
      }

      break;  // End of case start_GNSS2

    // ************************************************************************************************
    // Read a fix from the ZOE-M8Q. Could maybe make this a fxn.
    case read_GNSS2:

      Serial.println(F("Waiting for a 3D GNSS fix..."));

      agtFixType = 0;  // Clear the fix type

      // Look for GNSS signal for up to GNSS_timeout minutes
      for (unsigned long tnow = millis(); (agtFixType != 3) && ((millis() - tnow) < (GNSS_timeout * 60UL * 1000UL));) {

        agtFixType = myGNSS.getFixType();  // Get the GNSS fix type

        // Check battery voltage now we are drawing current for the GNSS
        // If voltage is low, stop looking for GNSS and go to sleep
        getVbat();
        if (agtVbat < VBAT_LOW) {
          break;  // Exit the for loop now
        }

        delay(100);  // Don't pound the I2C bus too hard!
      }

      // If voltage is low then go straight to sleep
      if (agtVbat < VBAT_LOW) {
        Serial.print(F("*** LOW VOLTAGE (read_GNSS) "));
        Serial.print(agtVbat, 2);
        Serial.println(F("V ***"));
        blinkLED(-1);
        delay(500);
        blinkLED(VOLT_STATE);
        lowVBatCount++;
        if (lowVBatCount == lowVBatCountLimit) {
          loop_step = survival_mode;
        } else {
          loop_step = zzz;  // Go to sleep
        }
      }

      else if (agtFixType == 3)  // Check if we got a valid 3D fix
      {
        // Get the time and position etc.
        // Get the time first to hopefully avoid second roll-over problems
        agtMilliseconds[reading_num] = myGNSS.getMillisecond();
        agtSecond[reading_num] = myGNSS.getSecond();
        agtMinute[reading_num] = myGNSS.getMinute();
        agtHour[reading_num] = myGNSS.getHour();
        agtDay[reading_num] = myGNSS.getDay();
        agtMonth[reading_num] = myGNSS.getMonth();
        agtYear[reading_num] = myGNSS.getYear();                                // Get the year
        agtLatitude[reading_num] = (float)myGNSS.getLatitude() / 10000000.0;    // Get the latitude in degrees
        agtLongitude[reading_num] = (float)myGNSS.getLongitude() / 10000000.0;  // Get the longitude in degrees
        agtAltitude[reading_num] = myGNSS.getAltitudeMSL() / 1000;              // Get the altitude above Mean Sea Level in m
        agtSpeed[reading_num] = (float)myGNSS.getGroundSpeed() / 1000.0;        // Get the ground speed in m/s
        agtSatellites[reading_num] = myGNSS.getSIV();                           // Get the number of satellites used in the fix
        agtCourse[reading_num] = myGNSS.getHeading() / 10000000;                // Get the heading in degrees
        agtPDOP[reading_num] = myGNSS.getPDOP() / 100;                          // Get the PDOP in m

        Serial.println(F("A 3D fix was found!"));
        Serial.print(F("Latitude (degrees): "));
        Serial.println(agtLatitude[reading_num], 6);
        Serial.print(F("Longitude (degrees): "));
        Serial.println(agtLongitude[reading_num], 6);
        Serial.print(F("Altitude (m): "));
        Serial.println(agtAltitude[reading_num]);

        loop_step = read_sensors2;  // Move on, read the pressure and temperature
      }

      else {
        // We didn't get a 3D fix so
        // set the lat, long etc. to default values
        agtLatitude[reading_num] = 0.0;    // Latitude in degrees
        agtLongitude[reading_num] = 0.0;   // Longitude in degrees
        agtAltitude[reading_num] = 0;      // Altitude above Median Seal Level in m
        agtSpeed[reading_num] = 0.0;       // Ground speed in m/s
        agtSatellites[reading_num] = 0;    // Number of satellites (SVs) used in the solution
        agtCourse[reading_num] = 0;        // Course (heading) in degrees
        agtPDOP[reading_num] = 0;          // Positional Dilution of Precision in m
        agtYear[reading_num] = 1970;       // GNSS Year
        agtMonth[reading_num] = 1;         // GNSS month
        agtDay[reading_num] = 1;           // GNSS day
        agtHour[reading_num] = 0;          // GNSS hours
        agtMinute[reading_num] = 0;        // GNSS minutes
        agtSecond[reading_num] = 0;        // GNSS seconds
        agtMilliseconds[reading_num] = 0;  // GNSS milliseconds

        Serial.println(F("A 3D fix was NOT found!"));
        Serial.println(F("Using default values..."));
        blinkLED(-1);
        delay(500);
        blinkLED(GPS_STATE);

        loop_step = read_sensors2;  // Move on, read the pressure and temperature
      }

      // Power down the GNSS
      Serial.println(F("Powering down the GNSS..."));
      gnssOFF();  // Disable power for the GNSS

      break;  // End of case read_GNSS2
    // ************************************************************************************************
    // Read the pressure and temperature from the MS8607
    case read_sensors2:

      setAGTWirePullups(1); // MS8607 needs pull-ups

      barometricSensorOK = barometricSensor.begin(agtWire); // Begin the PHT sensor
      if (barometricSensorOK == false)
      {
        // Send a warning message if we were unable to connect to the MS8607:
        Serial.println(F("*** Could not detect the MS8607 sensor. Trying again... ***"));
        barometricSensorOK = barometricSensor.begin(agtWire); // Re-begin the PHT sensor
        if (barometricSensorOK == false)
        {
          // Send a warning message if we were unable to connect to the MS8607:
          Serial.println(F("*** MS8607 sensor not detected at default I2C address ***"));
          blinkLED(-1);
          delay(500);
          blinkLED(COLLECT_DATA_STATE);
        }
      }

      agtPascals[reading_num] = barometricSensor.getPressure() * 100.0; // Convert pressure from hPa to Pascals
      agtHumidity[reading_num] = barometricSensor.getHumidity();

      Serial.print(F("Pressure="));
      Serial.print(agtPascals[reading_num], 1);
      Serial.println(F("(Pa)"));

      Serial.print("Humidity=");
      Serial.print(agtHumidity[reading_num], 1);
      Serial.print("(%RH)");

      setAGTWirePullups(0); // Disable pull-ups

      blinkLED(100);
      // Record battery voltage
      getVbat();
      reportVbat[reading_num] = agtVbat;

      // Grab data from temp sensors and/or WQ sensors
      Serial.println(F("Getting the temp and WQ readings..."));
      if (collectOtherData() == -1){
        blinkLED(-1);
        delay(500);
        blinkLED(COLLECT_DATA_STATE);
      }
      Serial.print(F("TSYS Air Temp="));
      Serial.print(airTemp[reading_num], 1);
      Serial.println(F("(C)"));

      loop_step = writeToSD;  // Move on, write to SD card

      break;  // End of case read_sensors2
    // ************************************************************************************************
    // Write data to the SD card
    case writeToSD:
      blinkLED(100);
      if (writeToMainFile() == -1){
        blinkLED(-1);
        delay(500);
        blinkLED(WRITE_STATE);
      }
      
      loop_step = start_LTC3225; // Move on, start the super capacitor charger
      break;  // End of case writeToSD

    // ************************************************************************************************
    // Start the LTC3225 supercapacitor charger
    case start_LTC3225:
      if (chargeCapacitors() == 0)
        loop_step = wait_LTC3225;  // Move on and give the capacitors extra charging time
      break;                       // End of case start_LTC3225

    // ************************************************************************************************
    // Give the super capacitors some extra time to charge
    case wait_LTC3225:
      if (waitForCapacitors() == 0)
        loop_step = start_9603;  // Move on and start the 9603N
      break;                     // End of case wait_LTC3225

    // ************************************************************************************************
    // Enable the 9603N and attempt to send a message
    case start_9603:
      if (powerUp9603() != 0)
        break;

      // The modem started OK so let's try to send the message
      char outBuffer[120];  // Use outBuffer to store the message. Always try to keep message short to avoid wasting credits

      // Apollo3 v2.1 does not support printf or sprintf correctly. We need to manually add preceeding zeros
      // and convert floats to strings

      // Convert the floating point values into strings
      char latStr[15];  // latitude string
      ftoa(agtLatitude[reading_num], latStr, 3, 15); // lat and lon readings seem to only be consistent out to 3 decimals. NDBC also only reports lat-lon with a precision of 3
      char lonStr[15];  // longitude string
      ftoa(agtLongitude[reading_num], lonStr, 3, 15);
      char altStr[15];  // altitude string
      ftoa(agtAltitude[reading_num], altStr, 2, 15);
      char vbatStr[6];  // battery voltage string
      ftoa(reportVbat[reading_num], vbatStr, 2, 6);
      char speedStr[8];  // speed string
      ftoa(agtSpeed[reading_num], speedStr, 2, 8);

      char pressureStr[9]; // pressure string from AGT onboard sensor
      ftoa(agtPascals[reading_num],pressureStr,0,9);
      char humidityStr[9]; // humidity string from AGT onboard sensor
      ftoa(agtHumidity[reading_num],humidityStr,1,10);

      char temperatureAirStr[10];  // temperature string from TSYS01
      ftoa(airTemp[reading_num], temperatureAirStr, 1, 10);
      char temperatureWaterStr[10]; // temperature string from EZO-RTD
      ftoa(waterTemp[reading_num],temperatureWaterStr,1,10);

      char ECStr[10]; // conductivity string from ezo
      ftoa(conductivity[reading_num],ECStr,1,10);
      char DOStr[10]; // dissolved oxygen string from ezo
      ftoa(dissolvedOxygen[reading_num],DOStr,1,10);
      char PHStr[10]; // pH string from ezo
      ftoa(pH[reading_num],PHStr,1,10);
      char ORPStr[10]; // ORP string from ezo
      ftoa(ORP[reading_num],ORPStr,1,10);

      // Convert the date and time into strings
      char gnssDay[3];
      char gnssMonth[3];
      if (agtDay[reading_num] < 10)
        sprintf(gnssDay, "0%d", agtDay[reading_num]);
      else
        sprintf(gnssDay, "%d", agtDay[reading_num]);
      if (agtMonth[reading_num] < 10)
        sprintf(gnssMonth, "0%d", agtMonth[reading_num]);
      else
        sprintf(gnssMonth, "%d", agtMonth[reading_num]);

      char gnssHour[3];
      char gnssMin[3];
      char gnssSec[3];
      if (agtHour[reading_num] < 10)
        sprintf(gnssHour, "0%d", agtHour[reading_num]);
      else
        sprintf(gnssHour, "%d", agtHour[reading_num]);
      if (agtMinute[reading_num] < 10)
        sprintf(gnssMin, "0%d", agtMinute[reading_num]);
      else
        sprintf(gnssMin, "%d", agtMinute[reading_num]);
      if (agtSecond[reading_num] < 10)
        sprintf(gnssSec, "0%d", agtSecond[reading_num]);
      else
        sprintf(gnssSec, "%d", agtSecond[reading_num]);

      // Assemble the message using sprintf
      sprintf(outBuffer, "%d-%s-%s %s:%s:%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s", agtYear[reading_num], gnssMonth, gnssDay, gnssHour, gnssMin, gnssSec,
              pressureStr, humidityStr, temperatureAirStr, temperatureWaterStr, ECStr, DOStr, PHStr, ORPStr,
              latStr, lonStr, altStr, vbatStr); // for ThingSpeak, the fields are: time, air pressure, RH, air temp, water temp, EC, DO, pH, ORP, lat, lon, elevation, and the status is the battery voltage

      // Send the message
      Serial.print(F("Transmitting message '"));
      Serial.print(outBuffer);
      Serial.println(F("'"));

#ifndef noTX
      agtErr = modem.sendSBDText(outBuffer);  // This could take many seconds to complete and will call ISBDCallback() periodically

      // If trying to also receive messages like a reset command from land:
      // memset(rxBuffer, 0, sizeof(rxBuffer)); // make sure receive buffer is clear before checking messages
      // agtErr = modem.sendReceiveSBDText(outBuffer, rxBuffer, bufferSize); // This could take many seconds to complete and will call ISBDCallback() periodically
#else
      agtErr = ISBD_SUCCESS;
#endif
      if (lowVBatCount == lowVBatCountLimit){
        loop_step = survival_mode;
        break;
      }

      // Check if the message sent OK
      if (agtErr != ISBD_SUCCESS) {
        Serial.print(F("Transmission failed with error code "));
        Serial.println(agtErr);
        blinkLED(-1);
        delay(500);
        blinkLED(SEND_STATE);
      } else {
        Serial.println(F(">>> Message sent! <<<"));

        // I wasn't able to get the below to actually receive a reset message sent from land and reset the AGT. 
        // It could be that I am not parsing the received message correctly...
        // // Convert the received bytes to an integer if they represent a hexadecimal value (from ChatGPT)
        // uint64_t receivedValue = strtoull(reinterpret_cast<const char*>(rxBuffer), nullptr, 16);
        // Serial.println("Reset message:");
        // Serial.println(receivedValue);

        // // if (strcmp((char *)rxBuffer, "reset") == 0) {
        // if (receivedValue == 0x7265736574) { // 7265736574 is the hexadecimal code for 'reset'
        //     Serial.println(F("*** 'reset' hexadecimal code received. RESETTING AGT "));
        //     am_hal_reset_control(AM_HAL_RESET_CONTROL_SWPOR,0); // 'SWPOR' is software power on reset
        //     while (1)
        //       ;  // Wait for the reset to take effect
        // }
      }

      sendAttemptCounter = sendAttemptCounter + 1;  // Increment the sendAttemptCounter
      lowVBatCount = 0;                             // since we made it all the way through without a low battery, reset counter to zero.

      loop_step = zzz;  // Now go to sleep

      break;  // End of case start_9603

    // ************************************************************************************************
    // Go to sleep
    case zzz:
      {
        Serial.println(F("Getting ready to put the Apollo3 into deep sleep..."));

        // Power down the GNSS
        Serial.println(F("Powering down the GNSS..."));
        gnssOFF();  // Disable power for the GNSS

        powerDown9603(); // function defined below that does a shutdown routine for the 9603

        // Disable the supercapacitor charger
        Serial.println(F("Disabling the supercapacitor charger..."));
        digitalWrite(superCapChgEN, LOW);  // Disable the super capacitor charger

        // Close the Iridium serial port
        Serial1.end();

        // Close the I2C port
        //agtWire.end(); // DO NOT Power down I2C - causes badness with v2.1 of the core: https://github.com/sparkfun/Arduino_Apollo3/issues/412

        digitalWrite(busVoltageMonEN, LOW);  // Disable the bus voltage monitor

        digitalWrite(LED, LOW);  // Disable the LED

        // Close and detach the serial console
        Serial.println(F("Going into deep sleep until next INTERVAL..."));
        Serial.flush();  //Finish any prints

        Serial.end();  // Close the serial console

        // Code taken (mostly) from Apollo3 Example6_Low_Power_Alarm (and modified by Chase)

        // Disable ADC
        powerControlADC(false);

        // Force the peripherals off
        am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_IOM0);  // SPI. I think SD card is connected here, so shut it down.
        // am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_IOM1); // agtWire I2C
        am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_IOM2);
        am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_IOM3);
        // am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_IOM4); // Qwiic I2C. Fails if you disable, even if you try to reenable
        am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_IOM5);
        am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_ADC);
        //am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_UART0); // Leave UART0 on to avoid printing erroneous characters to Serial
        am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_UART1);  // Serial1

        // Disable all unused pins - including: SCL (8), SDA (9), UART0 TX (48) and RX (49), UART1 TX (24) and RX (25), and Qwiic I2C pins (39 and 40)
        const int pinsToDisable[] = { 0, 1, 2, 8, 9, 11, 12, 14, 15, 16, 20, 21, 24, 25, 29, 31, 32, 33, 36, 37, 38, 39, 40, 42, 43, 44, 45, 48, 49, -1 };
        for (int x = 0; pinsToDisable[x] >= 0; x++) {
          pin_config(PinName(pinsToDisable[x]), g_AM_HAL_GPIO_DISABLE);
        }

        //Power down CACHE, flashand SRAM
        am_hal_pwrctrl_memory_deepsleep_powerdown(AM_HAL_PWRCTRL_MEM_ALL);     // Power down all flash and cache
        am_hal_pwrctrl_memory_deepsleep_retain(AM_HAL_PWRCTRL_MEM_SRAM_384K);  // Retain all SRAM (0.6 uA)

        // Keep the 32kHz clock running for RTC
        am_hal_stimer_config(AM_HAL_STIMER_CFG_CLEAR | AM_HAL_STIMER_CFG_FREEZE);
        am_hal_stimer_config(AM_HAL_STIMER_XTAL_32KHZ);

        if (lowVBatCount == lowVBatCountLimit) {
          // This while loop keeps the processor asleep until SURVIVAL seconds have passed
          while (!survivalAlarm)  // Wake up every SURVIVAL seconds
          {
            // Go to Deep Sleep.
            am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);
          }
          survivalAlarm = false;  // Clear the alarm flag now

          // Wake up! --- Not sure if I need to do this if I am not able to send a location message.
          // But can I check Voltage without waking up?
          loop_step = wakeUp;
          // If I *am* planning on sending a message even with low voltage, I might need to include some stuff from case loop_init before I do...
        } else {
          // This while loop keeps the processor asleep until INTERVAL seconds have passed
          while (!intervalAlarm)  // Wake up every INTERVAL seconds
          {
            // Go to Deep Sleep.
            am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);
          }
          intervalAlarm = false;  // Clear the alarm flag now

          // Wake up!
          loop_step = wakeUp;
        }
      }
      break;  // End of case zzz

    // ************************************************************************************************
    // Wake from sleep
    case wakeUp:

      // Code taken (mostly) from Apollo3 Example6_Low_Power_Alarm

      // Go back to using the main clock
      am_hal_stimer_config(AM_HAL_STIMER_CFG_CLEAR | AM_HAL_STIMER_CFG_FREEZE);
      am_hal_stimer_config(AM_HAL_STIMER_HFRC_3MHZ);

      // Power up SRAM, turn on entire Flash
      am_hal_pwrctrl_memory_deepsleep_powerdown(AM_HAL_PWRCTRL_MEM_MAX);

      // Renable UART0 pins: TX (48) and RX (49)
      pin_config(PinName(48), g_AM_BSP_GPIO_COM_UART_TX);
      pin_config(PinName(49), g_AM_BSP_GPIO_COM_UART_RX);

      // Renable Qwiic I2C pins if disabled
      pin_config(PinName(39), g_AM_BSP_GPIO_IOM4_SCL);
      pin_config(PinName(40), g_AM_BSP_GPIO_IOM4_SDA);

      am_hal_pwrctrl_periph_enable(AM_HAL_PWRCTRL_PERIPH_IOM0); // reenable SD card periphal
      // am_hal_pwrctrl_periph_enable(AM_HAL_PWRCTRL_PERIPH_IOM4);

      // Do not renable the UART1 pins here as the modem is still powered off. Let modem.begin do it via beginSerialPort.

      // Enable ADC
      powerControlADC(true);

      if (lowVBatCount == lowVBatCountLimit) {
        getVbat();
        loop_step = survival_mode;
      } else {
        // Do it all again!
        loop_step = loop_init;
      }

      break;  // End of case wake

    // ************************************************************************************************
    // If low battery occurs twice in a row, go into survival mode
    case survival_mode:

      if (agtVbat < VBAT_LOW) {
        // Get GNSS reading
        // Send location to shore --- How can I do this if my voltage isn't high enough to charge the super capacitors, which supply the necessary current to send a message?

        loop_step = zzz;  // Go into deep sleep for six hours
      } else {
        lowVBatCount = 0;       // reset lowVBatCount
        loop_step = loop_init;  // Do it all again!
      }
      break;  // end of case survival_mode
  }           // End of switch (loop_step)
}  // End of loop()

#if DIAGNOSTICS
void ISBDConsoleCallback(IridiumSBD *device, char c) {
  Serial.write(c);
}

void ISBDDiagsCallback(IridiumSBD *device, char c) {
  Serial.write(c);
}
#endif

void setAGTWirePullups(uint32_t i2cBusPullUps) {
  //Change SCL and SDA pull-ups manually using pin_config
  am_hal_gpio_pincfg_t sclPinCfg = g_AM_BSP_GPIO_IOM1_SCL;
  am_hal_gpio_pincfg_t sdaPinCfg = g_AM_BSP_GPIO_IOM1_SDA;

  if (i2cBusPullUps == 0) {
    sclPinCfg.ePullup = AM_HAL_GPIO_PIN_PULLUP_NONE;  // No pull-ups
    sdaPinCfg.ePullup = AM_HAL_GPIO_PIN_PULLUP_NONE;
  } else if (i2cBusPullUps == 1) {
    sclPinCfg.ePullup = AM_HAL_GPIO_PIN_PULLUP_1_5K;  // Use 1K5 pull-ups
    sdaPinCfg.ePullup = AM_HAL_GPIO_PIN_PULLUP_1_5K;
  } else if (i2cBusPullUps == 6) {
    sclPinCfg.ePullup = AM_HAL_GPIO_PIN_PULLUP_6K;  // Use 6K pull-ups
    sdaPinCfg.ePullup = AM_HAL_GPIO_PIN_PULLUP_6K;
  } else if (i2cBusPullUps == 12) {
    sclPinCfg.ePullup = AM_HAL_GPIO_PIN_PULLUP_12K;  // Use 12K pull-ups
    sdaPinCfg.ePullup = AM_HAL_GPIO_PIN_PULLUP_12K;
  } else {
    sclPinCfg.ePullup = AM_HAL_GPIO_PIN_PULLUP_24K;  // Use 24K pull-ups
    sdaPinCfg.ePullup = AM_HAL_GPIO_PIN_PULLUP_24K;
  }

  pin_config(PinName(PIN_AGTWIRE_SCL), sclPinCfg);
  pin_config(PinName(PIN_AGTWIRE_SDA), sdaPinCfg);
}

//*****************************************************************************
//
//  Divide an unsigned 32-bit value by 10.
//
//  Note: Adapted from Ch10 of Hackers Delight (hackersdelight.org).
//
//*****************************************************************************
static uint64_t divu64_10(uint64_t ui64Val) {
  uint64_t q64, r64;
  uint32_t q32, r32, ui32Val;

  //
  // If a 32-bit value, use the more optimal 32-bit routine.
  //
  if (ui64Val >> 32) {
    q64 = (ui64Val >> 1) + (ui64Val >> 2);
    q64 += (q64 >> 4);
    q64 += (q64 >> 8);
    q64 += (q64 >> 16);
    q64 += (q64 >> 32);
    q64 >>= 3;
    r64 = ui64Val - q64 * 10;
    return q64 + ((r64 + 6) >> 4);
  } else {
    ui32Val = (uint32_t)(ui64Val & 0xffffffff);
    q32 = (ui32Val >> 1) + (ui32Val >> 2);
    q32 += (q32 >> 4);
    q32 += (q32 >> 8);
    q32 += (q32 >> 16);
    q32 >>= 3;
    r32 = ui32Val - q32 * 10;
    return (uint64_t)(q32 + ((r32 + 6) >> 4));
  }
}

//*****************************************************************************
//
// Converts ui64Val to a string.
// Note: pcBuf[] must be sized for a minimum of 21 characters.
//
// Returns the number of decimal digits in the string.
//
// NOTE: If pcBuf is NULL, will compute a return ui64Val only (no chars
// written).
//
//*****************************************************************************
static int uint64_to_str(uint64_t ui64Val, char *pcBuf) {
  char tbuf[25];
  int ix = 0, iNumDig = 0;
  unsigned uMod;
  uint64_t u64Tmp;

  do {
    //
    // Divide by 10
    //
    u64Tmp = divu64_10(ui64Val);

    //
    // Get modulus
    //
    uMod = ui64Val - (u64Tmp * 10);

    tbuf[ix++] = uMod + '0';
    ui64Val = u64Tmp;
  } while (ui64Val);

  //
  // Save the total number of digits
  //
  iNumDig = ix;

  //
  // Now, reverse the buffer when saving to the caller's buffer.
  //
  if (pcBuf) {
    while (ix--) {
      *pcBuf++ = tbuf[ix];
    }

    //
    // Terminate the caller's buffer
    //
    *pcBuf = 0x00;
  }

  return iNumDig;
}

//*****************************************************************************
//
//  Float to ASCII text. A basic implementation for providing support for
//  single-precision %f.
//
//  param
//      fValue     = Float value to be converted.
//      pcBuf      = Buffer to place string AND input of buffer size.
//      iPrecision = Desired number of decimal places.
//      bufSize    = The size (in bytes) of the buffer.
//                   The recommended size is at least 16 bytes.
//
//  This function performs a basic translation of a floating point single
//  precision value to a string.
//
//  return Number of chars printed to the buffer.
//
//*****************************************************************************
#define OLA_FTOA_ERR_VAL_TOO_SMALL -1
#define OLA_FTOA_ERR_VAL_TOO_LARGE -2
#define OLA_FTOA_ERR_BUFSIZE -3

typedef union {
  int32_t I32;
  float F;
} ola_i32fl_t;

static int ftoa(float fValue, char *pcBuf, int iPrecision, int bufSize) {
  ola_i32fl_t unFloatValue;
  int iExp2, iBufSize;
  int32_t i32Significand, i32IntPart, i32FracPart;
  char *pcBufInitial, *pcBuftmp;

  iBufSize = bufSize;  // *(uint32_t*)pcBuf;
  if (iBufSize < 4) {
    return OLA_FTOA_ERR_BUFSIZE;
  }

  if (fValue == 0.0f) {
    // "0.0"
    *(uint32_t *)pcBuf = 0x00 << 24 | ('0' << 16) | ('.' << 8) | ('0' << 0);
    return 3;
  }

  pcBufInitial = pcBuf;

  unFloatValue.F = fValue;

  iExp2 = ((unFloatValue.I32 >> 23) & 0x000000FF) - 127;
  i32Significand = (unFloatValue.I32 & 0x00FFFFFF) | 0x00800000;
  i32FracPart = 0;
  i32IntPart = 0;

  if (iExp2 >= 31) {
    return OLA_FTOA_ERR_VAL_TOO_LARGE;
  } else if (iExp2 < -23) {
    return OLA_FTOA_ERR_VAL_TOO_SMALL;
  } else if (iExp2 >= 23) {
    i32IntPart = i32Significand << (iExp2 - 23);
  } else if (iExp2 >= 0) {
    i32IntPart = i32Significand >> (23 - iExp2);
    i32FracPart = (i32Significand << (iExp2 + 1)) & 0x00FFFFFF;
  } else  // if (iExp2 < 0)
  {
    i32FracPart = (i32Significand & 0x00FFFFFF) >> -(iExp2 + 1);
  }

  if (unFloatValue.I32 < 0) {
    *pcBuf++ = '-';
  }

  if (i32IntPart == 0) {
    *pcBuf++ = '0';
  } else {
    if (i32IntPart > 0) {
      uint64_to_str(i32IntPart, pcBuf);
    } else {
      *pcBuf++ = '-';
      uint64_to_str(-i32IntPart, pcBuf);
    }
    while (*pcBuf)  // Get to end of new string
    {
      pcBuf++;
    }
  }

  //
  // Now, begin the fractional part
  //
  *pcBuf++ = '.';

  if (i32FracPart == 0) {
    *pcBuf++ = '0';
  } else {
    int jx, iMax;

    iMax = iBufSize - (pcBuf - pcBufInitial) - 1;
    iMax = (iMax > iPrecision) ? iPrecision : iMax;

    for (jx = 0; jx < iMax; jx++) {
      i32FracPart *= 10;
      *pcBuf++ = (i32FracPart >> 24) + '0';
      i32FracPart &= 0x00FFFFFF;
    }

    //
    // Per the printf spec, the number of digits printed to the right of the
    // decimal point (i.e. iPrecision) should be rounded.
    // Some examples:
    // Value        iPrecision          Formatted value
    // 1.36399      Unspecified (6)     1.363990
    // 1.36399      3                   1.364
    // 1.36399      4                   1.3640
    // 1.36399      5                   1.36399
    // 1.363994     Unspecified (6)     1.363994
    // 1.363994     3                   1.364
    // 1.363994     4                   1.3640
    // 1.363994     5                   1.36399
    // 1.363995     Unspecified (6)     1.363995
    // 1.363995     3                   1.364
    // 1.363995     4                   1.3640
    // 1.363995     5                   1.36400
    // 1.996        Unspecified (6)     1.996000
    // 1.996        2                   2.00
    // 1.996        3                   1.996
    // 1.996        4                   1.9960
    //
    // To determine whether to round up, we'll look at what the next
    // decimal value would have been.
    //
    if (((i32FracPart * 10) >> 24) >= 5) {
      //
      // Yes, we need to round up.
      // Go back through the string and make adjustments as necessary.
      //
      pcBuftmp = pcBuf - 1;
      while (pcBuftmp >= pcBufInitial) {
        if (*pcBuftmp == '.') {
        } else if (*pcBuftmp == '9') {
          *pcBuftmp = '0';
        } else {
          *pcBuftmp += 1;
          break;
        }
        pcBuftmp--;
      }
    }
  }

  //
  // Terminate the string and we're done
  //
  *pcBuf = 0x00;

  return (pcBuf - pcBufInitial);
}  // ftoa()

// Charge the supercapacitors
int chargeCapacitors() {
  blinkLED(100);
  // Enable the supercapacitor charger
  Serial.println(F("Enabling the supercapacitor charger..."));
  digitalWrite(superCapChgEN, HIGH);  // Enable the super capacitor charger

  Serial.println(F("Waiting for supercapacitors to charge..."));
  delay(2000);

  agtPGOOD = false;  // Flag to show if PGOOD is HIGH

  // Wait for PGOOD to go HIGH for up to CHG_timeout minutes
  for (unsigned long tnow = millis(); (!agtPGOOD) && (millis() - tnow < CHG_timeout * 60UL * 1000UL);) {

    agtPGOOD = digitalRead(superCapPGOOD);  // Read the PGOOD pin

    // Check battery voltage now we are drawing current for the LTC3225
    // If voltage is low, stop charging and go to sleep
    getVbat();
    if (agtVbat < VBAT_LOW) {
      break;  // break out of for loop
    }

    delay(100);  // Let's not pound the bus voltage monitor too hard!
  }

  // If voltage is low then go straight to sleep
  if (agtVbat < VBAT_LOW) {
    Serial.print(F("*** LOW VOLTAGE (start_LTC3225) "));
    Serial.print(agtVbat, 2);
    Serial.println(F("V ***"));
    blinkLED(-1);
    delay(500);
    blinkLED(VOLT_STATE);
    lowVBatCount++;
    if (lowVBatCount == lowVBatCountLimit) {
      loop_step = survival_mode;
    } else {
      loop_step = zzz;  // Go to sleep
    }
    return -1;
  }

  else if (agtPGOOD) {
    // If the capacitors charged OK
    Serial.println(F("Supercapacitors charged!"));
    return 0;  // causes moving on to waiting for the supercapacitors to charge a bit more
  }

  else {
    // The super capacitors did not charge so power down and go to sleep
    Serial.println(F("*** Supercapacitors failed to charge ***"));
    blinkLED(-1);
    delay(500);
    blinkLED(CAPACITOR_STATE);

    lowVBatCount = 0;  // reset low VBat since the error on this loop was not caused by a low battery
    loop_step = zzz;
    return -1;
  }
}

// wait for supercapacitors to charge
int waitForCapacitors() {
  Serial.println(F("Giving the supercapacitors extra time to charge..."));

  // Wait for TOPUP_timeout seconds, keep checking PGOOD and the battery voltage
  for (unsigned long tnow = millis(); (millis() - tnow) < (TOPUP_timeout * 1000UL);) {

    // Check battery voltage now we are drawing current for the LTC3225
    // If voltage is low, stop charging and go to sleep
    getVbat();
    if (agtVbat < VBAT_LOW) {
      break;
    }

    delay(100);  // Let's not pound the bus voltage monitor too hard!
  }

  // If voltage is low then go straight to sleep
  if (agtVbat < VBAT_LOW) {
    Serial.print(F("*** LOW VOLTAGE (wait_LTC3225) "));
    Serial.print(agtVbat, 2);
    Serial.println(F("V ***"));
    blinkLED(-1);
    delay(500);
    blinkLED(VOLT_STATE);
    lowVBatCount++;
    if (lowVBatCount == lowVBatCountLimit) {
      loop_step = survival_mode;
    } else {
      loop_step = zzz;  // Go to sleep
    }
    return -1;
  }

  else if (agtPGOOD) {
    // If the capacitors are still charged OK
    Serial.println(F("Supercapacitors charged!"));
    return 0;
  }

  else {
    // The super capacitors did not charge so power down and go to sleep
    Serial.println(F("*** Supercapacitors failed to hold charge in wait_LTC3225 ***"));
    blinkLED(-1);
    delay(500);
    blinkLED(CAPACITOR_STATE);

    lowVBatCount = 0;  // reset low VBat since the error on this loop was not caused by a low battery
    loop_step = zzz;
    return -1;
  }
}

// Power up Iridium communications
int powerUp9603() {
  blinkLED(100);
  // Enable power for the 9603N
  Serial.println(F("Enabling 9603N power..."));
  digitalWrite(iridiumPwrEN, HIGH);  // Enable Iridium Power
  delay(1000);

  // Relax timing constraints waiting for the supercap to recharge.
  modem.setPowerProfile(IridiumSBD::USB_POWER_PROFILE);
  // modem.setPowerProfile(IridiumSBD::DEFAULT_POWER_PROFILE);

  // Begin satellite modem operation
  // Also begin the serial port connected to the satellite modem via IridiumSBD::beginSerialPort
  Serial.println(F("Starting modem..."));
  agtErr = modem.begin();

  // Check if the modem started correctly
  if (agtErr != ISBD_SUCCESS) {
    // If the modem failed to start, disable it and go to sleep
    Serial.print(F("*** modem.begin failed with error "));
    Serial.print(agtErr);
    Serial.println(F(" ***"));
    blinkLED(-1);
    delay(500);
    blinkLED(SEND_STATE);
    
    // if (reading_num == 1){ // if we are on the second reading, it means the data has already been written to the SD, and an error in the 9603 power up is fine for us to go to sleep
      lowVBatCount = 0;  // reset low VBat since the error on this loop was not caused by a low battery
      loop_step = zzz;
    // }
    return -1;
  }
  return 0;
}

// Clear MO buffer and power down Iridium communications
void powerDown9603() {
  // Clear the Mobile Originated message buffer
  Serial.println(F("Clearing the MO buffer."));
  agtErr = modem.clearBuffers(ISBD_CLEAR_MO);  // Clear MO buffer
  if (agtErr != ISBD_SUCCESS) {
    Serial.print(F("*** modem.clearBuffers failed with error "));
    Serial.print(agtErr);
    Serial.println(F(" ***"));
    blinkLED(-1);
    delay(500);
    blinkLED(SEND_STATE);
  }

  // Power down the modem
  // Also disable the Serial1 RX pin via IridiumSBD::endSerialPort
  Serial.println(F("Putting the 9603N to sleep."));
  agtErr = modem.sleep();
  if (agtErr != ISBD_SUCCESS) {
    Serial.print(F("*** modem.sleep failed with error "));
    Serial.print(agtErr);
    Serial.println(F(" ***"));
    blinkLED(-1);
    delay(500);
    blinkLED(SEND_STATE);
  }

  // Make sure the Serial1 RX pin is disabled
  modem.endSerialPort();

  // Disable 9603N power
  Serial.println(F("Disabling 9603N power..."));
  digitalWrite(iridiumSleep, LOW);  // Disable 9603N via its ON/OFF pin (modem.sleep should have done this already)
  delay(1000);
  digitalWrite(iridiumPwrEN, LOW);  // Disable Iridium Power
  delay(1000);
}

//*****************************************************************************
//
//  Functions modified from Liam for grabbing data from the sensors that
//  are connected to the multiplexer.
//
//*****************************************************************************
void blinkLED(int x)
{
  int i;
  if (x == 100) {
    for (i = 0; i < 10; i++) {
      digitalWrite(LED_PIN, HIGH);
      delay(100);
      digitalWrite(LED_PIN, LOW);
      delay(100);
    }
  } else if (x >= 0) {
    for (i = 0; i < x + 1; i++) {
      digitalWrite(LED_PIN, HIGH);
      delay(500);
      digitalWrite(LED_PIN, LOW);
      delay(250);
    }
  } else {
    digitalWrite(LED_PIN, HIGH);
    delay(4000);
    digitalWrite(LED_PIN, LOW);
  }
}

int collectOtherData() {
  getTemp(TSYS01_AIR_PORT, airTempSensor, airTemp);

  if (getWaterQualitySensorReadings(RTD_PORT, RTD_ADDR, waterTemp, Ezo_samples) == -1)
    return -1;
  if (getWaterQualitySensorReadings(EC_PORT, EC_ADDR, conductivity, Ezo_samples) == -1)
    return -1;
  if (getWaterQualitySensorReadings(DO_PORT, DO_ADDR, dissolvedOxygen, Ezo_samples) == -1)
    return -1;
  if (getWaterQualitySensorReadings(PH_PORT, PH_ADDR, pH, Ezo_samples) == -1)
    return -1;
  if (getWaterQualitySensorReadings(ORP_PORT, ORP_ADDR, ORP, Ezo_samples) == -1)
    return -1;

  return 0;
}

/*
 * time: 2 sec
 * samples: 20
 */
void getTemp(int port, TSYS01 tempSensor, float *arr) {

  enableMuxPort(port);

  float sumOfTemps = 0;
  int count = 0;
  int NUM_READINGS = 20;
  size_t reading_ms;

  while (count < NUM_READINGS) {
    reading_ms = millis();
    tempSensor.read();
    sumOfTemps += tempSensor.temperature();
    count++;
    while (millis() < reading_ms + 100) {}  //  wait until 100 ms has passed to take another reading
  }

  arr[reading_num] = sumOfTemps / NUM_READINGS;

  disableMuxPort(port);
}

/*
 * time: 20 sec
 * samples: 10
 */
int getWaterQualitySensorReadings(int port, int sensor_ADDR, float *arr, int samples) {
  // For some reason, using the Ezo_i2c library with this board can cause hard faults.
  // It has something to do with issues with the AGT's i2c and shutting it down/initiliazing it.
  // A work around is easy enough - just manually do what the Ezo functions do.
  // ChatGPT helped to write bits of this, especially reading the characters from Wire.
  enableMuxPort(port);
  Serial.println(F("WQ mux port enabled"));

  float sumOfReadings = 0;
  int count = 0;
  int NUM_READINGS = samples;
  size_t reading_ms;

  for (int i = 0; i < NUM_READINGS; i++) {
    reading_ms = millis();
    Wire.beginTransmission(sensor_ADDR);  // Start transmission to the sensor
    Wire.write("r");                      // Send the command to request the reading.
    Wire.endTransmission();               // End the transmission

    delay(1000);  // the largest required processing delay for any of the ezo circuits we are using is 900 ms

    // Receive the response from the sensor
    char nameBuffer[20];                            // Buffer to store the received name
    int bytesRead = Wire.requestFrom(sensor_ADDR, 20);  // Request data from the sensor
    if (bytesRead > 0) {
      Ezo_code = Wire.read();  // the first byte is the response code, we read this separately.

      if (Ezo_code != 1){  // any code from the reading except 1 is effectively an error
        // put ezo board to sleep. Any command will wake them up.
        Wire.beginTransmission(sensor_ADDR);  // Start transmission to the sensor
        Wire.write("Sleep");                  // Send the command to request the reading.
        Wire.endTransmission();               // End the transmission
        disableMuxPort(port);
        return -1;
      }

      int index = 0;
      while (Wire.available()) {
        char receivedChar = Wire.read();
        nameBuffer[index++] = receivedChar;  // Read character by character
      }
      nameBuffer[index] = '\0';  // Null-terminate the string

      sumOfReadings += atof(nameBuffer);  // convert the reading to a float
    }
    count++;
    while (millis() < reading_ms + 2000) {}  //  wait until at least 2000 ms has passed to take another reading
  }
  arr[reading_num] = sumOfReadings / count; // Ezo readings are average of 10 readings over 20 seconds

  // put ezo board to sleep. Any command will wake them up.
  Wire.beginTransmission(sensor_ADDR);  // Start transmission to the sensor
  Wire.write("Sleep");                  // Send the command to request the reading.
  Wire.endTransmission();               // End the transmission

  disableMuxPort(port);
  return 0;
}

// - - - functions for computing wave statistics - - -
int sample_IMU() {
  DateTime imuDateTime = { agtYear[reading_num],
                           agtMonth[reading_num],
                           agtDay[reading_num],
                           agtHour[reading_num],
                           agtMinute[reading_num],
                           agtSecond[reading_num] };  // starts as first datetime read by the GNSS in this loop

  if (imuDateTime.year != 1970)                          // 1970 is default year value and means we did not get a GNSS fix.
    updateDateTime(imuDateTime, millis() - gnssMillis);  // this function isn't super necessary, as the last GNSS-ping date and time will be pretty close to when the IMU starts recording
  int doyInt = dayOfYear(imuDateTime);

  // Convert the date and time into strings
  if (imuDateTime.day < 10)
    sprintf(dd, "0%d", imuDateTime.day);
  else
    sprintf(dd, "%d", imuDateTime.day);
  if (imuDateTime.month < 10)
    sprintf(mM, "0%d", imuDateTime.month);
  else
    sprintf(mM, "%d", imuDateTime.month);
  sprintf(yyyy, "%d", imuDateTime.year);

  if (imuDateTime.hour < 10)
    sprintf(hh, "0%d", imuDateTime.hour);
  else
    sprintf(hh, "%d", imuDateTime.hour);
  if (imuDateTime.minute < 10)
    sprintf(mm, "0%d", imuDateTime.minute);
  else
    sprintf(mm, "%d", imuDateTime.minute);
  if (imuDateTime.second < 10)
    sprintf(ss, "0%d", imuDateTime.second);
  else
    sprintf(ss, "%d", imuDateTime.second);

  if (doyInt < 10)
    sprintf(doy, "0%d", doyInt);
  else
    sprintf(doy, "%d", doyInt);

  char filename[13];
  strcpy(filename, doy);
  strcat(filename, hh);
  strcat(filename, mm);
  strcat(filename, "I.txt");

  enableMuxPort(IMU_PORT);

  // // If IMU is not connected to the MUX but is daisychained:
  // Wire.requestFrom(ICM_address, 1);
  // if (!Wire.available()) return -1;  //Error
  // byte settings = Wire.read();
  // // Set the wanted bit to enable the port
  // settings |= (1 << 9);  // The IMU is at 0x69, therefore port = 9;
  // // Write the updated settings
  // Wire.beginTransmission(ICM_address);
  // Wire.write(settings);
  // Wire.endTransmission();

  size_t start_ms = ::millis();
  size_t reading_ms;

  myFile = SD.open(filename, FILE_WRITE);

  if (myFile) {

    Serial.println("file was made");

    myFile.print(yyyy);
    myFile.print(",");
    myFile.print(mM);
    myFile.print(",");
    myFile.print(dd);
    myFile.print(",");
    myFile.print(hh);
    myFile.print(",");
    myFile.print(mm);
    myFile.print(",");
    myFile.print(ss);
    myFile.print(",");
    myFile.print(agtSatellites[reading_num]);  // just uses num sats, lat-lon from last GPS reading rather than pinging before each IMU reading
    myFile.print(",");
    myFile.print(agtLatitude[reading_num]);
    myFile.print(",");
    myFile.println(agtLongitude[reading_num]);

    myFile.println("Ax (m/s^2),Ay (m/s^2),Az (m/s^2),Gx (deg/sec),Gy (deg/sec),Gz (deg/sec),Mx (mTesla),My (mTesla),Mz (mTesla)");

    int count = 0;

    float **array;
    array = new float *[9];
    for (int i = 0; i < 9; i++)
      array[i] = new float[IMU_SAMPLE_SIZE];

    // Wake the IMU up
    myICM.sleep(false);
    myICM.lowPower(false);

    while (count < IMU_SAMPLE_SIZE) {

      if (myICM.dataReady()) {
        reading_ms = millis();
        myICM.getAGMT();

        array[0][count] = myICM.accX() / 1000 * GRAVITY;
        array[1][count] = myICM.accY() / 1000 * GRAVITY;
        array[2][count] = myICM.accZ() / 1000 * GRAVITY;
        array[3][count] = myICM.gyrX();
        array[4][count] = myICM.gyrY();
        array[5][count] = myICM.gyrZ();
        array[6][count] = myICM.magX();
        array[7][count] = myICM.magY();
        array[8][count] = myICM.magZ();

        myFile.print(array[0][count]);  // m/s
        myFile.print(",");
        myFile.print(array[1][count]);  // m/s
        myFile.print(",");
        myFile.print(array[2][count]);  // m/s
        myFile.print(",");
        myFile.print(array[3][count]);  // degrees per second
        myFile.print(",");
        myFile.print(array[4][count]);  // degrees per second
        myFile.print(",");
        myFile.print(array[5][count]);  // degrees per second
        myFile.print(",");
        myFile.print(array[6][count]);  // micro teslas
        myFile.print(",");
        myFile.print(array[7][count]);  // micro teslas
        myFile.print(",");
        myFile.println(array[8][count]);  // micro teslas

        count++;
        while (millis() < reading_ms + 250) {}  //  wait until 250 ms has passed to take another reading
      }
    }

    // WaveProcessor waveProcessor = WaveProcessor(array);
    // float x = waveProcessor.getHeight();
    // Serial.println(x);

    for (int i = 0; i < 9; i++)
      delete[] array[i];
    delete[] array;

    myFile.close();
    Serial.println("IMU data collection complete");

  } else {
    Serial.println("ERROR: file was not made");
    // shutdown the IMU
    myICM.lowPower(true);
    myICM.sleep(true);

    disableMuxPort(IMU_PORT);

    // // If IMU is not connected to the MUX but is daisychained:
    // Wire.requestFrom(ICM_address, 1);
    // if (!Wire.available()) return -1;  //Error
    // byte settings_close = Wire.read();
    // // Clear the wanted bit to disable the port
    // settings_close &= ~(1 << 9);
    // // Write the updated settings
    // Wire.beginTransmission(ICM_address);
    // Wire.write(settings_close);
    // Wire.endTransmission();

    return -1;
  }
  // shutdown the IMU
  myICM.lowPower(true);
  myICM.sleep(true);

  disableMuxPort(IMU_PORT);

  // // If IMU is not connected to the MUX but is daisychained:
  // Wire.requestFrom(ICM_address, 1);
  // if (!Wire.available()) return -1;  //Error
  // byte settings_close = Wire.read();
  // // Clear the wanted bit to disable the port
  // settings_close &= ~(1 << 9);
  // // Write the updated settings
  // Wire.beginTransmission(ICM_address);
  // Wire.write(settings_close);
  // Wire.endTransmission();

  return 0;
}

//  - - - - - - - - - - - - - - - - - - - - -
//     function to configure sensors
//  - - - - - - - - - - - - - - - - - - - - -
int configureSensors() {
  enableMuxPort(IMU_PORT);
  while (true) {
    myICM.begin(Wire, 1);
    if (myICM.status == ICM_20948_Stat_Ok)
      break;
    delay(500);
  }
  disableMuxPort(IMU_PORT);

  enableMuxPort(TSYS01_AIR_PORT);
  airTempSensor.init();
  disableMuxPort(TSYS01_AIR_PORT);

  return 0;
}

//  - - - - - - - - - - - - - - - - - - - - -
//     functions to access mux devices
//  - - - - - - - - - - - - - - - - - - - - -

//Enables a specific port number
int enableMuxPort(byte portNumber) {
  if (portNumber > 7) portNumber = 7;

  //Read the current mux settings
  Wire.requestFrom(MUX_ADDR, 1);
  if (!Wire.available())
    return -1;  //Error
  byte settings = Wire.read();

  //Set the wanted bit to enable the port
  settings |= (1 << portNumber);

  Wire.beginTransmission(MUX_ADDR);
  Wire.write(settings);
  Wire.endTransmission();

  return 0;
}

//Disables a specific port number
int disableMuxPort(byte portNumber) {
  if (portNumber > 7) portNumber = 7;

  //Read the current mux settings
  Wire.requestFrom(MUX_ADDR, 1);
  if (!Wire.available())
    return -1;  //Error
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
int setUpSD() {
  Serial.print("Initializing SD card...");
  if (SD.begin(SD_PIN)) {

    Serial.println("initialization done.");
    myFile = SD.open("test.txt", FILE_WRITE);

    if (myFile) {
      myFile.println("Meow");
      myFile.close();
      return 0;
    }
  }

  Serial.println("initialization failed!");
  return -1;
}

int writeToMainFile() {
  sprintf(yyyy, "%d", agtYear[reading_num]);
  char filename[13];
  strcpy(filename, "BUOY");
  strcat(filename, yyyy);
  strcat(filename, ".txt");

  myFile = SD.open(filename, FILE_WRITE);
  if (myFile) {

    for (int i = 0; i < 2; i++) {
      myFile.print(agtYear[i]);
      myFile.print(",");
      myFile.print(agtMonth[i]);
      myFile.print(",");
      myFile.print(agtDay[i]);
      myFile.print(",");
      myFile.print(agtHour[i]);
      myFile.print(",");
      myFile.print(agtMinute[i]);
      myFile.print(",");
      myFile.print(agtSecond[i]);
      myFile.print(",");
      myFile.print(agtSatellites[i]);
      myFile.print(",");
      myFile.print(agtLatitude[i]);
      myFile.print(",");
      myFile.print(agtLongitude[i]);
      myFile.print(",");
      myFile.print(agtAltitude[i]);
      myFile.print(",");
      myFile.print(agtSpeed[i]);
      myFile.print(",");
      myFile.print(agtCourse[i]);
      myFile.print(",");
      myFile.print(agtPDOP[i]);
      myFile.print(",");
      myFile.print(agtPascals[i]);
      myFile.print(",");
      myFile.print(agtHumidity[i]);
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
      myFile.print(ORP[i]);
      myFile.print(",");
      myFile.println(reportVbat[i]);
    }
    myFile.close();
    return 0;
  }

  return -1;
}


//  - - - - - - - - - - - - - - - - - - -
//  Time keeping stuff for IMU readings
//  Functions generated by ChatGPT
//  - - - - - - - - - - - - - - - - - - -
bool isLeapYear(int year) {
  return (year % 4 == 0 && year % 100 != 0) || (year % 400 == 0);
}

int daysInMonthForYearMonth(int year, int month) {
  const int daysInMonth[] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };
  int days = daysInMonth[month - 1];
  if (month == 2 && isLeapYear(year)) {
    days++;
  }
  return days;
}

int dayOfYear(const DateTime &dateTime) {
  int days = dateTime.day;
  for (int m = 1; m < dateTime.month; ++m) {
    days += daysInMonthForYearMonth(dateTime.year, m);
  }
  return days;
}

void updateDateTime(DateTime &dateTime, unsigned long elapsedMillis) {
  const unsigned long millisPerSecond = 1000;
  const unsigned long millisPerMinute = 60 * millisPerSecond;
  const unsigned long millisPerHour = 60 * millisPerMinute;
  const unsigned long millisPerDay = 24 * millisPerHour;

  dateTime.second += elapsedMillis / millisPerSecond;
  elapsedMillis %= millisPerSecond;
  if (dateTime.second >= 60) {
    dateTime.minute += dateTime.second / 60;
    dateTime.second %= 60;
  }

  dateTime.minute += elapsedMillis / millisPerMinute;
  elapsedMillis %= millisPerMinute;
  if (dateTime.minute >= 60) {
    dateTime.hour += dateTime.minute / 60;
    dateTime.minute %= 60;
  }

  dateTime.hour += elapsedMillis / millisPerHour;
  elapsedMillis %= millisPerHour;
  if (dateTime.hour >= 24) {
    dateTime.day += dateTime.hour / 24;
    dateTime.hour %= 24;
  }

  // Now update the date if necessary
  while (true) {
    int daysInMonth = daysInMonthForYearMonth(dateTime.year, dateTime.month);
    if (dateTime.day > daysInMonth) {
      dateTime.day -= daysInMonth;
      dateTime.month++;
      if (dateTime.month > 12) {
        dateTime.year++;
        dateTime.month = 1;
      }
    } else {
      break;
    }
  }
}
