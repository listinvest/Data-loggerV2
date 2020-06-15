// Data logger:  For measuring acceleration from LIS3DH
// Hardware:  Adafruit ESP32, Adalogger feather+RTC, 1x LIS3DH accels (Currently, 2x in future) 
// Created:  May 12 2020
// Updated:
// Uses SPI for SD, I2C for Accels, hoping for 1000 Hz. sampling 
// files are saves text files, csv = NNNNNNNN.TXT
// See ReadME and photos for additional hook up info

//Use ESP32 duo core
const int TaskCore1  = 1;
const int TaskCore0 = 0;

//Libraries
#include <Wire.h>
#include <SPI.h>
#include "SdFat.h"
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>
#include "stdio.h"
#include "esp_system.h" //This inclusion configures the peripherals in the ESP system.
//#include "freertos/Arduino_FreeRTOS.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
//#include "freertos/stream_buffer.h"

#define LED_BUILTIN LED_BUILTIN //LED light for notification
//------------------------------------------------------------------------------
// SD file definitions
const uint8_t sdChipSelect = 33;
SdFat sd;
SdFile file;
File logfile;
//------------------------------------------------------------------------------
// Queue definitions

// data type for Queue item
struct Data_t {
  uint32_t usec; 
  float value1X;
  float value1Y;
  float value1Z;
  float value2X;
  float value2Y;
  float value2Z;
} MY_Data_t;

//Declare Queue data type for FreeRTOS
QueueHandle_t DataQueue = NULL;

void *ptrtostruct;
void *ptr;

//------------------------------------------------------------------------------
// Accel Lis3dh definitions, SPI or I2C
// Used for software SPI
//#define LIS3DH_CLK 32  //SCL
//#define LIS3DH_MISO 15  //SDO
//#define LIS3DH_MOSI 27  //SDA
// Used for hardware & software SPI
#define LIS3DH_CS 14  //ESP32: 14/A6 , Cortex m0: 5, Use for upper accel (Sensor 1!!!) = hbar, seatpost, etc.
#define LIS3DH_CS2 15  //ESP32: 15/A8, Cortex m0: 9, Use for lower accel (Sensor 2!!!) = axles, etc. 
// software SPI
//Adafruit_LIS3DH lis = Adafruit_LIS3DH(LIS3DH_CS, LIS3DH_MOSI, LIS3DH_MISO, LIS3DH_CLK);

// hardware SPI 1 LIS3DH->Feather:  Power to Vin, Gnd to Gnd, SCL->SCK, SDA->MOSO, SDO->MOSI, CS->CS 14/15
// Sensor 1 Hardware SPI
Adafruit_LIS3DH lis = Adafruit_LIS3DH(LIS3DH_CS);
// Sensor 2 Hardware SPI
Adafruit_LIS3DH lis2 = Adafruit_LIS3DH(LIS3DH_CS2);

// Sensor I2C 
//Adafruit_LIS3DH lis = Adafruit_LIS3DH();

//------------------------------------------------------------------------------
// define tasks for Sensor Data and SD Write
void TaskGetData( void *pvParameters );
void TaskSDWrite( void *pvParameters );
//void TaskSDFlush( void *pvParameters );
//void TaskSDClose( void *pvParameters );
//------------------------------------------------------------------------------

//Hardware Timer
hw_timer_t * timer = NULL;  //create timer handler
volatile SemaphoreHandle_t timerSemaphore;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
volatile uint32_t isrCounter = 0;
volatile uint32_t lastIsrAt = 0;
void IRAM_ATTR onTimer();  //Initialize onTimer Function
 
// the setup function runs once when you press reset or power the board=================================================================================================================================
void setup() {

  // initialize serial communication at 115200 bits per second:
  Serial.begin(115200);

  //Queue Setup
  DataQueue = xQueueCreate(100, sizeof( MY_Data_t ));
  if(DataQueue == NULL){
     Serial.println("Error Creating the Queue");
   }

  // Create timer
  //TimerHandle_t timer1 = xTimerCreate("HZ sample timer", pdMS_TO_TICKS(100), pdTRUE, 0, TaskGetData);
  TimerHandle_t timer2 = xTimerCreate("flush timer", pdMS_TO_TICKS(5000), pdTRUE, 0, TaskSDFlush);
  TimerHandle_t timer3 = xTimerCreate("close file timer", pdMS_TO_TICKS(8000), pdTRUE, 0, TaskSDClose);
  /*if (timer1 == NULL) {
    Serial.println("Timer can not be created");
  } else {
    // Start timer
    if (xTimerStart(timer1, 0) == pdPASS) { // Start the scheduler
      Serial.println("Timer working");
    }
  }*/
  if (timer2 == NULL) {
    Serial.println("Timer 2 can not be created");
  } else {
    // Start timer
    if (xTimerStart(timer2, 0) == pdPASS) { // Start the scheduler
      Serial.println("Timer 2 working");
    }
  }
    if (timer3 == NULL) {
    Serial.println("Timer 3 can not be created");
  } else {
    // Start timer
    if (xTimerStart(timer3, 0) == pdPASS) { // Start the scheduler
      Serial.println("Timer 3 working");
    }
  }

  // Create semaphore to inform us when the timer has fired
  timerSemaphore = xSemaphoreCreateBinary();
  // Use 1st timer of 4 (counted from zero).
  // Set 80 divider for prescaler (see ESP32 Technical Reference Manual for more
  // info).
  timer = timerBegin(0, 80, true);
  // Attach onTimer function to our timer.
  timerAttachInterrupt(timer, &onTimer, true);
  // Set alarm to call onTimer function every second (value in microseconds).
  // Repeat the alarm (third parameter)
  timerAlarmWrite(timer, 1000000, true);
  // Start an alarm
  timerAlarmEnable(timer);
  
  //SPI.beginTransaction(SPISettings(80000000, MSBFIRST, SPI_MODE0));

  //Outputs, Pins, Buttons, Etc. 
  pinMode(LED_BUILTIN, OUTPUT);  //set Built in LED to show writing on SD Card
  pinMode(27, INPUT); //button to turn recording on/off, In [HIGH]
  
  //ACCEL Setup and RUN
  if (! lis.begin(0x18)) {   // change this to 0x19 for alternative i2c address
  Serial.println("Couldnt start");
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


// SD CARD SETUP ====================================================================
// see if the card is present and can be initialized:  (Use highest SD clock possible, but lower if has error, 15 Mhz works, possible to go to to 25 Mhz if sample rate is low enough
if (!sd.begin(sdChipSelect, SD_SCK_MHZ(15))) {
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
    if (! sd.exists(filename)) {
      break;
    }
  }

// Create file and prepare it ============================================================
  logfile = sd.open(filename, O_CREAT | O_WRITE);  //O_WRONLY | O_CREAT | O_EXCL ); // | O_TRUNC )); 
  if( ! logfile ) {
    Serial.print("Couldnt create "); 
    Serial.println(filename);
    while (1) yield(); 
  }
  Serial.print("Writing to "); 
  Serial.println(filename);

  //Column labels
  logfile.print("Time"); 
  logfile.print(",");
  logfile.print("Sensor 1 X");
  logfile.print(",");
  logfile.print("Sensor 1 Y");
  logfile.print(",");
  logfile.print("Sensor 1 Z");
  logfile.print(",");
  logfile.print("Sensor 2 X");
  logfile.print(",");
  logfile.print("Sensor 2 Y");
  logfile.print(",");
  logfile.print("Sensor 2 Z");
  logfile.println();
  

  
// Setup up Tasks and where to run ============================================================  
// Now set up tasks to run independently.
  xTaskCreatePinnedToCore(
    TaskGetData
    ,  "Get Data from Accel to Queue"   // A name just for humans
    ,  10000 // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  3  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL 
    ,  TaskCore1);

  xTaskCreatePinnedToCore(
    TaskSDWrite
    ,  "Get Data from Queue"
    ,  10000 // Stack size
    ,  NULL
    ,  3 // Priority
    ,  NULL 
    ,  TaskCore0);

  /*xTaskCreatePinnedToCore(
    TaskSDFlush
    ,  "Write Data to Card"
    ,  10000 // Stack size
    ,  NULL
    ,  3  // Priority
    ,  NULL 
    ,  TaskCore1);*/

  /*xTaskCreatePinnedToCore(
    TaskSDClose
    ,  "Close the File"
    ,  10000 // Stack size
    ,  NULL
    ,  3  // Priority
    ,  NULL 
    ,  TaskCore1);*/    
}
//================================================================================================================================
void loop()
{
  // Empty. Things are done in Tasks.
/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/
}

//================================================================================================================================
//================================================================================================================================
void IRAM_ATTR onTimer(){
// Increment the counter and set the time of ISR
portENTER_CRITICAL_ISR(&timerMux);
isrCounter++;
lastIsrAt = millis();
portEXIT_CRITICAL_ISR(&timerMux);
// Give a semaphore that we can check in the loop
xSemaphoreGiveFromISR(timerSemaphore, NULL);
Serial.println("Timer went off");
Serial.println(lastIsrAt);
// It is safe to use digitalRead/Write here if you want to toggle an output
}
  
void TaskSDFlush( TimerHandle_t timer2 )  // This is a task.
{
  logfile.flush(); 
  Serial.print("The Flush is IN");
}

void TaskSDClose( TimerHandle_t timer3 )  // This is a task.
{
  logfile.close(); 
  Serial.print("File = Done");
}
/////////////////////////////////////////////////////////////////////////////////////
/*void TaskGetData()  // This is a task.
  {
    QueueHandle_t DataQueue;
    
    struct Data_t *pSendData;  //Create pointer to the struct

    if (xSemaphoreTake(timerSemaphore, 0) == pdTRUE){    

    ptrtostruct = pvPortMalloc(sizeof (MY_Data_t));
      
    sensors_event_t event;
    lis.getEvent(&event);
    sensors_event_t event2;
    lis2.getEvent(&event2);
    pSendData->usec = micros();
    pSendData->value1X = event.acceleration.x;
    pSendData->value1Y = event.acceleration.y;
    pSendData->value1Z = event.acceleration.z;
    pSendData->value2X = event2.acceleration.x;
    pSendData->value2Y = event2.acceleration.y;
    pSendData->value2Z = event2.acceleration.z;
    Serial.print(event.acceleration.y);
    Serial.print(pSendData->usec); 
    Serial.print(',');
    Serial.print(pSendData->value1X,5);
    Serial.print(',');
    Serial.print(pSendData->value1Y,5);
    Serial.print(',');
    Serial.print(pSendData->value1Z,5);
    Serial.print(',');
    Serial.print(pSendData->value2X,5);
    Serial.print(',');
    Serial.print(pSendData->value2Y,5);
    Serial.print(',');
    Serial.print(pSendData->value2Z,5);
    Serial.println();

    if(xQueueSend( DataQueue, &pSendData, 2000 ) != pdPASS )  //portMAX_DELAY
      {
        Serial.println("xQueueSend is not working"); 
      }
    }
    else {
      Serial.print("nothing happening");
    }
    //digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    //vTaskDelay(100);  // one tick delay (15ms) in between reads for stability
    //digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    //vTaskDelay(100);  // one tick delay (15ms) in between reads for stability
 
    //vTaskDelay(intervalTicks);  // one tick delay (1000 uSec/1 mSec) in between reads for 1000 Hz reading 
    }*/

////////////////////////////////////////////////////////////////////////////////////////////
void TaskGetData(void *pvParameters)  // This is a task.
{
  (void) pvParameters;
  int D = 0;
  struct Data_t *pSendData;  //Create pointer to the struct

  for (;;) // A Task shall never return or exit.
  {
    if (xSemaphoreTake(timerSemaphore, 0) == pdTRUE){
      /*if(xQueueSend( DataQueue, (void *) &pSendData, 2000 ) != pdPASS )  //portMAX_DELAY
      {
        Serial.println("xQueueSend is not working"); 
      }*/
    sensors_event_t event;
    lis.getEvent(&event);
    sensors_event_t event2;
    lis2.getEvent(&event2);
    Serial.print("Start logging data cheese head"); 
    pSendData->usec = micros();
    pSendData->value1X = event.acceleration.x;
    pSendData->value1Y = event.acceleration.y;
    pSendData->value1Z = event.acceleration.z;
    pSendData->value2X = event2.acceleration.x;
    pSendData->value2Y = event2.acceleration.y;
    pSendData->value2Z = event2.acceleration.z;
    Serial.print(pSendData->usec); 
    Serial.print(',');
    Serial.print(pSendData->value1X,5);
    Serial.print(',');
    Serial.print(pSendData->value1Y,5);
    Serial.print(',');
    Serial.print(pSendData->value1Z,5);
    Serial.print(',');
    Serial.print(pSendData->value2X,5);
    Serial.print(',');
    Serial.print(pSendData->value2Y,5);
    Serial.print(',');
    Serial.print(pSendData->value2Z,5);
    Serial.println();
    }
    
    D++;
    Serial.println(D);
    //else {
      //Serial.print("nothing happening");
    //}
    //digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    //vTaskDelay(100);  // one tick delay (15ms) in between reads for stability
    //digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    //vTaskDelay(100);  // one tick delay (15ms) in between reads for stability
 
    //vTaskDelay(intervalTicks);  // one tick delay (1000 uSec/1 mSec) in between reads for 1000 Hz reading 
    }
    vTaskDelete( NULL );
}
//------------------------------------------------------------------------------
void TaskSDWrite(void *pvParameters)  // This is a task.
{
  (void) pvParameters;
  
  struct Data_t *RCV_Data; 
  
  for (;;)
  {

    if(DataQueue != NULL ) 
    {
      if( xQueueReceive( DataQueue, &RCV_Data, 10000 ) != pdPASS )   //portMAX_DELAY
      {
        Serial.println("xQueueRecieve is not working");
      }
        ptr = pvPortMalloc(sizeof (MY_Data_t));
        
        logfile.print(RCV_Data->usec);
        logfile.print(',');
        logfile.print(RCV_Data->value1X,4);
        logfile.print(',');
        logfile.print(RCV_Data->value1Y,4);
        logfile.print(',');
        logfile.print(RCV_Data->value1Z,4);
        logfile.print(',');
        logfile.print(RCV_Data->value2X,4);
        logfile.print(',');
        logfile.print(RCV_Data->value2Y,4);
        logfile.print(',');
        logfile.print(RCV_Data->value2Z,4);
        logfile.println(); 
        /*Serial.print(RCV_Data->usec); 
        Serial.print(',');
        Serial.print(RCV_Data->value1X,5);
        Serial.print(',');
        Serial.print(RCV_Data->value1Y,5);
        Serial.print(',');
        Serial.print(RCV_Data->value1Z,5);
        Serial.print(',');
        Serial.print(RCV_Data->value2X,5);
        Serial.print(',');
        Serial.print(RCV_Data->value2Y,5);
        Serial.print(',');
        Serial.print(RCV_Data->value2Z,5);
        Serial.println(); */
 
        uint16_t FreeSpace = uxQueueSpacesAvailable( DataQueue ); 
        Serial.println(FreeSpace);
      }
   }
   vTaskDelete( NULL ); 
}


//------------------------------------------------------------------------------
/*void TaskSDFlush(void *pvParameters)  // This is a task.
{
  (void) pvParameters;

  for (;;)
  {
    logfile.flush();
    //Serial.println("Flushed file"); 
    
  }
  vTaskDelete ( NULL ); 
}

//------------------------------------------------------------------------------
void TaskSDClose(void *pvParameters)  // This is a task.
{
  (void) pvParameters;

  for (;;)
  {
    //vTaskDelay( 5000 );
    logfile.close();
    //Serial.println("Close file"); 
    
  }
  vTaskDelete ( NULL ); 
}*/
