#ifndef UserTypes_h
#define UserTypes_h
#include "Arduino.h"
#include "SPI.h"
#define USE_SHARED_SPI 3
#define FILE_BASE_NAME "Waxwing"
// User data types.  Modify for your data items.
const uint8_t ACCEL_DIM = 5;
struct data_t {
  uint32_t time;
  int16_t accel[ACCEL_DIM];
};
void acquireData(data_t* data);
void printData(Print* pr, data_t* data);
void printHeader(Print* pr);
void userSetup();
#endif  // UserTypes_h
