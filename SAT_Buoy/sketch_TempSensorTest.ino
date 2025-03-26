#include <Wire.h>

const int QWIIC_MUX_ADDRESS = 0x74; // I2C address of the Qwiic Mux
const int TSYS01_ADDRESS = 0x76; // I2C address of the TSYS01

void setup() {
  Serial.begin(9600); // initialize serial communication
  Wire.begin(); // initialize I2C communication
}

void loop() {
  // read the temperature from the first TSYS01 sensor
  int temperature1 = readTemperature(0);
  Serial.print("Temperature 1 ");
  Serial.println(temperature1); // print the temperature to the serial monitor
  delay(1000); // wait 1 second before reading the temperature again

  // read the temperature from the second TSYS01 sensor
  int temperature2 = readTemperature(1);
  Serial.print("Tempearture 2 ");
  Serial.println(temperature2); // print the temperature to the serial monitor
  delay(1000); // wait 1 second before reading the temperature again
}

int readTemperature(int channel) {
  Wire.beginTransmission(QWIIC_MUX_ADDRESS); // start a transmission to the Qwiic Mux
  Wire.write(0x01 << channel); // send a command to select the desired I2C channel
  Wire.endTransmission(); // end the transmission

  Wire.beginTransmission(TSYS01_ADDRESS); // start a transmission to the TSYS01
  Wire.write(0x48); // send a command to initiate a temperature conversion
  Wire.endTransmission(); // end the transmission

  delay(10); // wait for the conversion to complete

  Wire.requestFrom(TSYS01_ADDRESS, 3); // request 3 bytes from the TSYS01
  int temperature = Wire.read() << 8; // read the first byte and shift it left by 8 bits
  temperature |= Wire.read(); // OR the second byte with the first
  Wire.read(); // discard the third byte
  temperature = (temperature / 32768) * 165 - 40; // convert the raw temperature value to degrees Celsius
  return temperature;
}
