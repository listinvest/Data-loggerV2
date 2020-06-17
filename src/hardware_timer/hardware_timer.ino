//Use ESP32 duo core
const int TaskCore1  = 1;
const int TaskCore0 = 0;
#include <Arduino.h>
#include <SPI.h>
#include <SdFat.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>
#include "stdio.h"
#include "esp_system.h" //This inclusion configures the peripherals in the ESP system.
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
//#include "freertos/queue.h"

// SD Card uses SPI, Set the selct pin
#define cardSelect 33 // 33 for ESP32, 10 for Cortex M0

// Used for hardware & software SPI
#define LIS3DH_CS 14  //ESP32: 14/A6 , Cortex m0: 5, Use for upper accel, hbar, seatpost, etc.
#define LIS3DH_CS2 15  //ESP32: 15/A8, Cortex m0: 9, Use for lower accel, axles, etc. 

// hardware SPI 1 LIS3DH->Feather:  Power to Vin, Gnd to Gnd, SCL->SCK, SDA->MOSI, SDO->MOSO, CS->CS 14 or 15
// Sensor 1 
Adafruit_LIS3DH lis = Adafruit_LIS3DH(LIS3DH_CS);
// Sensor 2 
// hardware SPI 2
Adafruit_LIS3DH lis2 = Adafruit_LIS3DH(LIS3DH_CS2);

//SD writing
File logfile;
SdFat SD;

int count = 0; 
unsigned long Micro = 0; 

volatile int interruptCounter;
int totalInterruptCounter;
 
hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

//Create Interrupt Semaphore
SemaphoreHandle_t timerSemaphore;

//TaskHandle_t TaskGetData = NULL; 
//TaskHandle_t vTimerISR = NULL; 

//static signed BaseType_t xHigherPriorityTaskWoken;

void TaskGetData(void *pvParameters)  // This is a task.
{
  (void) pvParameters;
  
  for (;;) // A Task shall never return or exit.
  {
    if (xSemaphoreTake(timerSemaphore, 10) == pdTRUE)
    {
    sensors_event_t event; 
    lis.getEvent(&event);
    sensors_event_t event2;
    lis2.getEvent(&event2);
    Micro = micros();
    logfile.print(Micro); 
    logfile.print(",");
    logfile.print(event.acceleration.x,4);
    logfile.print(",");
    logfile.print(event.acceleration.y,4);
    logfile.print(",");
    logfile.print(event.acceleration.z,4);
    logfile.print(",");
    logfile.print(event2.acceleration.x,4);
    logfile.print(",");
    logfile.print(event2.acceleration.y,4);
    logfile.print(",");
    logfile.print(event2.acceleration.z,4);
    logfile.println();
    count++; 
    if (count == 5000){
      logfile.close();
      //Serial.print("shut the door");
      count = 0; 
    }
   }
  }
  vTaskDelete( NULL );
} 

void IRAM_ATTR vTimerISR()
  {
  portENTER_CRITICAL_ISR(&timerMux);
  interruptCounter++;
  xSemaphoreGiveFromISR( timerSemaphore, NULL );  //Unblock the task by releasing the semaphore. */
  portEXIT_CRITICAL_ISR(&timerMux); 
  //xHigherPriorityTaskWoken = pdFALSE;  
  }

//------------------------------------------------------------------------------
// define tasks for Sensor Data and SD Write
void TaskGetData( void *pvParameters ); //Always runs, executes on semaphore recieve 
 
void setup() {
 
  Serial.begin(115200);

  if (! lis.begin(0x18)) {   // Sensor 1, change this to 0x19 for alternative i2c address
    Serial.println("Couldnt start Sensor 1");
    while (1) yield();
  }
  Serial.println("LIS3DH Sensor 1 found!");

  if (! lis2.begin(0x19)) {   // Sensor 2, SDO is 3V
    Serial.println("Couldnt start Sensor 2");
    while (1) yield();
  }
  Serial.println("LIS3DH Sensor 2 found!");

  // Set accel range  
  lis.setRange(LIS3DH_RANGE_16_G);   // 2, 4, 8 or 16 G!
  lis2.setRange(LIS3DH_RANGE_16_G);

  // Set DataRate
  lis.setDataRate(LIS3DH_DATARATE_LOWPOWER_5KHZ); //OPTIONS:  LIS3DH_DATARATE_400_HZ, LIS3DH_DATARATE_LOWPOWER_1K6HZ, LIS3DH_DATARATE_LOWPOWER_5KHZ
  lis2.setDataRate(LIS3DH_DATARATE_LOWPOWER_5KHZ); 
  Serial.print("Range 1/2 = "); 
  Serial.print(2 << lis.getRange());
  Serial.print("/");
  Serial.print(2 << lis2.getRange());  
  Serial.println("G");

// see if the card is present and can be initialized:  (Use highest SD clock possible, but lower if has error, 15 Mhz works, possible to go to to 50 Mhz if sample rate is low enough
  if (!SD.begin(cardSelect, SD_SCK_MHZ(15))) {
    Serial.println("Card init. failed!");
    while (1) yield(); 
 }

 // Create filename scheme ====================================================================
  char filename[15];
  //  Setup filename to be appropriate for what you are testing
  strcpy(filename, "/DATA00.TXT");
  for (uint8_t i = 0; i < 100; i++) {
    filename[5] = '0' + i/10;
    filename[6] = '0' + i%10;
    // create if does not exist, do not open existing, write, sync after write
    if (! SD.exists(filename)) {
      break;
    }
  }

// Create file and prepare it ============================================================
  logfile = SD.open(filename, O_CREAT | O_WRITE); //FILE_WRITE);
  if( ! logfile ) {
    Serial.print("Couldnt create "); 
    Serial.println(filename);
    //error(3);
  }
  Serial.print("Writing to "); 
  Serial.println(filename);

  //pinMode(13, OUTPUT);
  Serial.println("Ready!");
  
    //Column labels
  logfile.print("Time"); 
  logfile.print("\t");
  logfile.print("Sensor 1 X");
  logfile.print("\t");
  logfile.print("Sensor 1 Y");
  logfile.print("\t");
  logfile.print("Sensor 1 Z");
  logfile.print("\t");
  logfile.print("Sensor 2 X");
  logfile.print("\t");
  logfile.print("Sensor 2 Y");
  logfile.print("\t");
  logfile.print("Sensor 2 Z");
  logfile.println();

  // Create semaphore to inform us when the timer has fired
  timerSemaphore = xSemaphoreCreateBinary();

 // Setup up Tasks and where to run ============================================================  
 // Now set up tasks to run independently.
  xTaskCreatePinnedToCore(
    TaskGetData
    ,  "Get Data from Accel to Queue"   // A name just for humans
    ,  10000 // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  3 // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL 
    ,  TaskCore1);

    // Create Timer ===============================================================================
  timer = timerBegin(1, 80, true);
  timerAttachInterrupt(timer, &vTimerISR, true);
  timerAlarmWrite(timer, 1000, true);
  timerAlarmEnable(timer); 
}
 
void loop() {
 //Nothing in here 
}
