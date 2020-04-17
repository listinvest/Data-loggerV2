#include "UserTypes.h"
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>
// User data functions.  Modify these functions for your data items.

// Start time for data
static uint32_t startMicros;

// Used for hardware & software SPI
#define LIS3DH_CS 14
#define LIS3DH_CS2 15

// hardware SPI 1
Adafruit_LIS3DH lis = Adafruit_LIS3DH(LIS3DH_CS);
// hardware SPI 2
Adafruit_LIS3DH lis2 = Adafruit_LIS3DH(LIS3DH_CS2);
//#define Accel_1 = Adafruit_LIS3DH(LIS3DH_CS);
//#define Accel_2 = Adafruit_LIS3DH(LIS3DH_CS2);

/*const uint8_t ADXL345_CS = 9;

const uint8_t POWER_CTL = 0x2D;  //Power Control Register
const uint8_t DATA_FORMAT = 0x31;
const uint8_t DATAX0 = 0x32; //X-Axis Data 0
const uint8_t DATAX1 = 0x33; //X-Axis Data 1
const uint8_t DATAY0 = 0x34; //Y-Axis Data 0
const uint8_t DATAY1 = 0x35; //Y-Axis Data 1
const uint8_t DATAZ0 = 0x36; //Z-Axis Data 0
const uint8_t DATAZ1 = 0x37; //Z-Axis Data 1

void writeADXL345Register(const uint8_t registerAddress, const uint8_t value) {
  // Max SPI clock frequency is 5 MHz with CPOL = 1 and CPHA = 1.
  SPI.beginTransaction(SPISettings(5000000, MSBFIRST, SPI_MODE3));  
  digitalWrite(ADXL345_CS, LOW);
  SPI.transfer(registerAddress);
  SPI.transfer(value);
  digitalWrite(ADXL345_CS, HIGH);
  SPI.endTransaction();  
}*/

void userSetup() {
  pinMode(LIS3DH_CS, OUTPUT);
  digitalWrite(LIS3DH_CS, HIGH);
  pinMode(LIS3DH_CS2, OUTPUT);
  digitalWrite(LIS3DH_CS2, HIGH);
  SPI.begin();
  
  //Put the ADXL345 into +/- 4G range by writing the value 0x01 to the DATA_FORMAT register.
  //writeADXL345Register(DATA_FORMAT, 0x01);
  //Put the ADXL345 into Measurement Mode by writing 0x08 to the POWER_CTL register.
  //writeADXL345Register(POWER_CTL, 0x08);  //Measurement mode  

  /*    if (! lis.begin(0x18)) {   // Sensor 1, change this to 0x19 for alternative i2c address
    Serial.println("Couldnt start Sensor 1");
    while (1);
  }
  Serial.println("LIS3DH Sensor 1 found!");

  if (! lis2.begin(0x19)) {   // Sensor 2, SDO is 3V
    Serial.println("Couldnt start Sensor 2");
    while (1);
  }
  Serial.println("LIS3DH Sensor 2 found!");*/

    lis.setRange(LIS3DH_RANGE_16_G);   // 2, 4, 8 or 16 G!
    lis2.setRange(LIS3DH_RANGE_16_G);

    lis.setDataRate(LIS3DH_DATARATE_LOWPOWER_5KHZ); 
    lis2.setDataRate(LIS3DH_DATARATE_LOWPOWER_5KHZ); 
  
  Serial.print("Range 1/2 = "); 
  Serial.print(2 << lis.getRange());
  Serial.print("/");
  Serial.print(2 << lis2.getRange());  
  Serial.println("G");

  
}

// Acquire a data record.
void acquireData(data_t* data) {
  // Max SPI clock frequency is 5 MHz with CPOL = 1 and CPHA = 1.
  //SPI.beginTransaction(SPISettings(5000000, MSBFIRST, SPI_MODE3));
  data->time = micros();
  lis.read();     // get X Y and Z data at once (raw data) sensor 1
  lis2.read();    // get X Y and Z data at once (raw data) sensor 2
  // log raw data from Sensor 1 and 2
  // Note:  must normalize raw data which means measured values are between -32768 and 32767.  -Range G/+Range G = -32768/32767 
  
  //digitalWrite(ADXL345_CS, LOW);
  // Read multiple bytes so or 0XC0 with address.
  //SPI.transfer(DATAX0 | 0XC0);
  data->accel[0] = lis.x;
  data->accel[1] = lis.y;
  data->accel[2] = lis.z; 
  data->accel[3] = lis2.x;
  data->accel[4] = lis2.y;
  data->accel[5] = lis2.z; 
  //data->accel[0] = SPI.transfer(0) | (SPI.transfer(0) << 8);
  //data->accel[1] = SPI.transfer(0) | (SPI.transfer(0) << 8);
  //data->accel[2] = SPI.transfer(0) | (SPI.transfer(0) << 8); 
  //digitalWrite(ADXL345_CS, HIGH);
  //SPI.endTransaction();
}

// Print a data record.
void printData(Print* pr, data_t* data) {
  if (startMicros == 0) {
    startMicros = data->time;
  }
  pr->print(data->time - startMicros);
  for (int i = 0; i < ACCEL_DIM; i++) {
    pr->write(',');
    pr->print(data->accel[i]);
  }
  pr->println();
}

// Print data header.
void printHeader(Print* pr) {
  startMicros = 0;
  pr->println(F("micros,ax1,ay1,az1,ax2,ay2,az2"));
}
