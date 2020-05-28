#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif

//Libraries
#include <SPI.h>
#include <SdFat.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>
#include <stdio.h>
//#include "freertos/FreeRTOS.h"
//#include "freertos/task.h"
//#include "esp_system.h"

#ifndef LED_BUILTIN
#define LED_BUILTIN 13
#endif

//------------------------------------------------------------------------------
// SD file definitions
const uint8_t sdChipSelect = 33;
SdFat sd;
SdFile file;
//------------------------------------------------------------------------------
// Fifo definitions

// size of fifo in records
const size_t FIFO_SIZE = 20;

// count of data records in fifo
SemaphoreHandle_t fifoData;

// count of free buffers in fifo
SemaphoreHandle_t fifoSpace;

// data type for fifo item
struct FifoItem_t {
  uint32_t usec;  
  int value;
  int error;
};
// array of data items
FifoItem_t fifoArray[FIFO_SIZE];
//------------------------------------------------------------------------------
// Accel Lis3dh definitions, I2C
#define LIS3DH_CS 16 //should be 32  //ESP32: 14/A6 , Cortex m0: 5, Use for upper accel, hbar, seatpost, etc.
// Sensor I2C 
Adafruit_LIS3DH lis = Adafruit_LIS3DH();
//------------------------------------------------------------------------------
// define two tasks for Sensor Data and SD Write
void TaskGetData( void *pvParameters );
//void TaskSDWrite( void *pvParameters );

//------------------------------------------------------------------------------
// the setup function runs once when you press reset or power the board
void setup() {

  // initialize serial communication at 115200 bits per second:
  Serial.begin(115200);

  if (! lis.begin(0x18)) {   // change this to 0x19 for alternative i2c address
  Serial.println("Couldnt start");
  while (1) yield();
  }
  Serial.println("LIS3DH found!");

  // Set accel range  
  lis.setRange(LIS3DH_RANGE_16_G);   // 2, 4, 8 or 16 G!
  //lis2.setRange(LIS3DH_RANGE_16_G);
  // Set DataRate
  lis.setDataRate(LIS3DH_DATARATE_LOWPOWER_5KHZ); //OPTIONS:  LIS3DH_DATARATE_400_HZ, LIS3DH_DATARATE_LOWPOWER_1K6HZ, LIS3DH_DATARATE_LOWPOWER_5KHZ
  //lis2.setDataRate(LIS3DH_DATARATE_LOWPOWER_5KHZ); 

   // open file
  if (!sd.begin(sdChipSelect)
    || !file.open("DATA.CSV", O_CREAT | O_WRITE | O_TRUNC)) {
    Serial.println(F("SD problem"));
    sd.errorHalt();
  }
  // initialize fifoData semaphore to no data available
  fifoData = xSemaphoreCreateCounting(FIFO_SIZE, 0);
  
  // initialize fifoSpace semaphore to FIFO_SIZE free records
  fifoSpace = xSemaphoreCreateCounting(FIFO_SIZE, FIFO_SIZE);

  
  // Setup up Tasks and where to run ============================================================  
  // Now set up two tasks to run independently.
  xTaskCreatePinnedToCore(
    TaskGetData
    ,  "GetData"   // A name just for humans
    ,  1800  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  3  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL 
    ,  ARDUINO_RUNNING_CORE);

  xTaskCreatePinnedToCore(
    TaskSDWrite
    ,  "SDWrite"
    ,  1024  // Stack size
    ,  NULL
    ,  2  // Priority
    ,  NULL 
    ,  ARDUINO_RUNNING_CORE);

  // Now the task scheduler, which takes over control of scheduling individual tasks, is automatically started.
}

void loop()
{
  // Empty. Things are done in Tasks.
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

void TaskGetData(void *pvParameters)  // This is a task.
{
  (void) pvParameters;

  for (;;) // A Task shall never return or exit.
  {
    //Get Event
    //lis.read();
    sensors_event_t event; 
    lis.getEvent(&event);
    //Serial.print("X:  "); Serial.print(lis.x);
    Serial.print("\t\tX: "); Serial.print(event.acceleration.x);
    Serial.print("\tY: "); Serial.print(event.acceleration.y);
    Serial.print("\tZ: "); Serial.print(event.acceleration.z);
    Serial.println(); 
    vTaskDelay(1);  // one tick delay (1000 uSec/1 mSec) in between reads for 1000 Hz reading 
    
    //digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    //vTaskDelay(100);  // one tick delay (15ms) in between reads for stability
    //digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    //vTaskDelay(100);  // one tick delay (15ms) in between reads for stability
  }
}

void TaskSDWrite(void *pvParameters)  // This is a task.
{
  (void) pvParameters;

  for (;;)
  {
    // SD write task
    // FIFO index for record to be written
  size_t fifoTail = 0;
  
  // time in micros of last point
  uint32_t last = 0;  
  
  while(1) {
    // wait for next data record
    xSemaphoreTake(fifoData, portMAX_DELAY);
    
    FifoItem_t* p = &fifoArray[fifoTail];

    // print interval between points
    if (last) {
      file.print(p->usec - last);
    } else {
      file.write("NA");
    }
    last = p->usec;
    file.write(',');
    file.print(p->value);
    file.write(',');
    file.println(p->error);

    // release record
    xSemaphoreGive(fifoSpace);
    
    // advance FIFO index
    fifoTail = fifoTail < (FIFO_SIZE - 1) ? fifoTail + 1 : 0;
    
    // check for end run
    if (Serial.available()) {
      // close file to insure data is saved correctly
      file.close();
      
      while(1);
  }
 }
} 
}
