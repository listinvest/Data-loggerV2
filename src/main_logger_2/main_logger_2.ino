#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif

//Libraries
#include <Wire.h>
#include <SPI.h>
#include <SdFat.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>
#include <stdio.h>
//#include "freertos/FreeRTOS.h"
//#include "freertos/task.h"
//#include "esp_system.h"

//------------------------------------------------------------------------------
// SD file definitions
const uint8_t sdChipSelect = 33;
SdFat sd;
SdFile file;
File logfile;
//------------------------------------------------------------------------------
// Fifo definitions

// size of fifo in records
const size_t FIFO_SIZE = 200;

// count of data records in fifo
SemaphoreHandle_t fifoData;

// count of free buffers in fifo
SemaphoreHandle_t fifoSpace;

// data type for fifo item
struct FifoItem_t {
  uint32_t usec;  
  //unsigned long value;
  float value; 
  int error;
};

//Counter
//unsigned long Micro = 0; 

//Accel Values
//unsigned long Ax = 0;
//unsigned long Ay = 0;
//unsigned long Az = 0;

// interval between points in units of 1000 usec
const uint16_t intervalTicks = 1000;

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
void TaskSDWrite( void *pvParameters );

//------------------------------------------------------------------------------
// the setup function runs once when you press reset or power the board
void setup() {

  // initialize serial communication at 115200 bits per second:
  Serial.begin(115200);

  //Outputs, Pins, Buttons, Etc. 
  pinMode(13, OUTPUT);  //set Built in LED to show writing on SD Card
  //pinMode(ledPin, OUTPUT);  //set LED to show when switched off
  //pinMode(button, INPUT); //button to turn recording on/off

  //ACCEL Setup and RUN
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

// SD CARD SETUP ====================================================================
// see if the card is present and can be initialized:  (Use highest SD clock possible, but lower if has error, 15 Mhz works, possible to go to to 50 Mhz if sample rate is low enough
if (!sd.begin(sdChipSelect, SD_SCK_MHZ(15))) {
  Serial.println("Card init. failed!");
  //error(2);
}

// Create filename scheme ====================================================================
  char filename[15];
  //  Setup filename to be appropriate for what you are testing
  strcpy(filename, "/WARBIRD00.TXT");
  for (uint8_t i = 0; i < 100; i++) {
    filename[8] = '0' + i/10;
    filename[9] = '0' + i%10;
    // create if does not exist, do not open existing, write, sync after write
    if (! sd.exists(filename)) {
      break;
    }
  }

// Create file and prepare it ============================================================
  logfile = sd.open(filename, FILE_WRITE);
  //file.open(filename, O_CREAT | O_WRITE | O_TRUNC);
  if( ! logfile ) {
    Serial.print("Couldnt create "); 
    Serial.println(filename);
    //error(3);
  }
  Serial.print("Writing to "); 
  Serial.println(filename);

  pinMode(13, OUTPUT);
  Serial.println("Ready!");

  // initialize fifoData semaphore to no data available
  fifoData = xSemaphoreCreateCounting(FIFO_SIZE, 0);
  
  // initialize fifoSpace semaphore to FIFO_SIZE free records
  fifoSpace = xSemaphoreCreateCounting(FIFO_SIZE, FIFO_SIZE);    
  
  
  // open file
  //if (!sd.begin(sdChipSelect)) 
  //  || !file.open("DATA.CSV", O_CREAT | O_WRITE | O_TRUNC)) {
  //  Serial.println(F("SD problem"));
  //  sd.errorHalt();
  //  }

 
// Setup up Tasks and where to run ============================================================  
// Now set up two tasks to run independently.
  xTaskCreatePinnedToCore(
    TaskGetData
    ,  "GetData"   // A name just for humans
    ,  1800  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL 
    ,  ARDUINO_RUNNING_CORE);

  xTaskCreatePinnedToCore(
    TaskSDWrite
    ,  "SDWrite"
    ,  6000  // Stack size
    ,  NULL
    ,  3  // Priority
    ,  NULL 
    ,  ARDUINO_RUNNING_CORE);

// Now the task scheduler, which takes over control of scheduling individual tasks, is automatically started.
// start scheduler (I might not need this, old version???)
//vTaskStartScheduler();
//Serial.println(F("Insufficient RAM"));
// while(1);

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
    // index of record to be filled
    size_t fifoHead = 0;

    // count of overrun errors
    int error = 0;

    // dummy data
    int count = 0;

    // initialise the ticks variable with the current time.
    TickType_t ticks = xTaskGetTickCount();

    while (1) {
    // wait until time for next data point
    vTaskDelayUntil(&ticks, intervalTicks);

    // get a buffer
    if (xSemaphoreTake(fifoSpace, 0) != pdTRUE) {
      // fifo full - indicate missed point
      error++;
      continue;
    }
    FifoItem_t* p = &fifoArray[fifoHead];
    p->usec = micros();

    // replace next line with data read from sensor
    //Micro = micros(); 
    sensors_event_t event;
    lis.getEvent(&event);
    p->value = event.acceleration.y;
    //lis.read();
    //Ax = event.acceleration.x;
    //Ay = event.acceleration.y;
    //Az = event.acceleration.z;
    
    //p->value = lis.x; 

    p->error = error;
    error = 0;

    // signal new data
    xSemaphoreGive(fifoData);

    // advance FIFO index
    fifoHead = fifoHead < (FIFO_SIZE - 1) ? fifoHead + 1 : 0;
    
    //Get Event
    //lis.read();
    //sensors_event_t event; 
    //lis.getEvent(&event);
    //Serial.print("X:  "); Serial.print(lis.x);
    //Serial.print("\tX: "); Serial.print(event.acceleration.x);
    //Serial.print("\tY: "); Serial.print(event.acceleration.y);
    //Serial.print("\tZ: "); Serial.print(event.acceleration.z);
    //Serial.println(); 
    //vTaskDelay(1000);  // one tick delay (1000 uSec/1 mSec) in between reads for 1000 Hz reading 
    
    //digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    //vTaskDelay(100);  // one tick delay (15ms) in between reads for stability
    //digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    //vTaskDelay(100);  // one tick delay (15ms) in between reads for stability
  }
}
}
//------------------------------------------------------------------------------
void TaskSDWrite(void *pvParameters)  // This is a task.
{
  (void) pvParameters;
    // SD write task
    // FIFO index for record to be written
  size_t fifoTail = 0;
  
  // time in micros of last point
  uint32_t last = 0;  
  
  for (;;)
  {
 
  while(1) {
    // wait for next data record
    xSemaphoreTake(fifoData, portMAX_DELAY);
    
    FifoItem_t* p = &fifoArray[fifoTail];

    // print interval between points
    if (last) {
      logfile.print(p->usec - last);
    } else {
      logfile.write("NA");
    }
    last = p->usec;
    Serial.println(p->usec);
    logfile.write(',');
    logfile.print(p->value);
    Serial.println(p->value);
    //logfile.print(event.acceleration.x);
    logfile.write(',');
    logfile.println(p->error);
    Serial.println(p->error);

    /*logfile.print(Micro);
    logfile.print("\t");
    logfile.print(Ax);
    logfile.print("\t");
    logfile.print(Ay);
    logfile.print("\t");
    logfile.print(Az); 
    logfile.println();*/

    // release record
    xSemaphoreGive(fifoSpace);
    
    // advance FIFO index
    fifoTail = fifoTail < (FIFO_SIZE - 1) ? fifoTail + 1 : 0;
    
    // check for end run
    //if (Serial.available()) {
      // close file to insure data is saved correctly
    //logfile.close();
    logfile.flush(); 

    //digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    //vTaskDelay(100);  // one tick delay (15ms) in between reads for stability
    //digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    //vTaskDelay(100);  // one tick delay (15ms) in between reads for stability
      
     // while(1);
  //}
 }
} 
}
