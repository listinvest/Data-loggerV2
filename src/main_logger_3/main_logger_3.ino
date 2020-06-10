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
  float valueX;
  float valueY;
  float valueZ;
} xData_t;

//Declare Queue data type for FreeRTOS
QueueHandle_t DataQueue = NULL;

// interval between points in units of 1000 usec
const uint16_t intervalTicks = 2;

//------------------------------------------------------------------------------
// Accel Lis3dh definitions, SPI or I2C
// Used for software SPI
//#define LIS3DH_CLK 32  //SCL
//#define LIS3DH_MISO 15  //SDO
//#define LIS3DH_MOSI 27  //SDA
// Used for hardware & software SPI
#define LIS3DH_CS 14  //ESP32: 14/A6 , Cortex m0: 5, Use for upper accel (Sensor 1!!!) = hbar, seatpost, etc.
//#define LIS3DH_CS2 15  //ESP32: 15/A8, Cortex m0: 9, Use for lower accel (Sensor 2!!!) = axles, etc. 
// software SPI
//Adafruit_LIS3DH lis = Adafruit_LIS3DH(LIS3DH_CS, LIS3DH_MOSI, LIS3DH_MISO, LIS3DH_CLK);

// hardware SPI 1 LIS3DH->Feather:  Power to Vin, Gnd to Gnd, SCL->SCK, SDA->MOSO, SDO->MOSI, CS->CS 14/15
// Sensor 1 Hardware SPI
Adafruit_LIS3DH lis = Adafruit_LIS3DH(LIS3DH_CS);
// Sensor 2 Hardware SPI
//Adafruit_LIS3DH lis2 = Adafruit_LIS3DH(LIS3DH_CS2);

// Sensor I2C 
//Adafruit_LIS3DH lis = Adafruit_LIS3DH();

//------------------------------------------------------------------------------
// define two tasks for Sensor Data and SD Write
void TaskGetData( void *pvParameters );
void TaskSDWrite( void *pvParameters );
void TaskSDFlush( void *pvParameters );
//------------------------------------------------------------------------------

// Start the scheduler so the created tasks start executing. Need this for ESP32???
//void vTaskStartScheduler();
   
// the setup function runs once when you press reset or power the board
void setup() {

  // initialize serial communication at 115200 bits per second:
  //Serial.begin(250000);

  //Outputs, Pins, Buttons, Etc. 
  pinMode(13, OUTPUT);  //set Built in LED to show writing on SD Card

  //ACCEL Setup and RUN
  if (! lis.begin(/*0x18*/)) {   // change this to 0x19 for alternative i2c address
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

  //Setup I2C speed
  //Wire.setClock(400000);

// SD CARD SETUP ====================================================================
// see if the card is present and can be initialized:  (Use highest SD clock possible, but lower if has error, 15 Mhz works, possible to go to to 25 Mhz if sample rate is low enough
if (!sd.begin(sdChipSelect, SD_SCK_MHZ(15))) {
  Serial.println("Card init. failed!");
  //error(2);
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
  logfile = sd.open(filename, O_WRONLY | O_CREAT | O_EXCL ); // | O_TRUNC )); O_CREAT | O_WRITE);
  if( ! logfile ) {
    Serial.print("Couldnt create "); 
    Serial.println(filename);
    //error(3);
  }
  Serial.print("Writing to "); 
  Serial.println(filename);

  pinMode(13, OUTPUT);
  Serial.println("Ready!");


  //Queue Setup
  DataQueue = xQueueCreate(10, sizeof( &xData_t ));
  if(DataQueue == NULL){
     Serial.println("Error Creating the Queue");
   }
  
// Setup up Tasks and where to run ============================================================  
// Now set up tasks to run independently.
  xTaskCreatePinnedToCore(
    TaskGetData
    ,  "Get Data from Accel to Queue"   // A name just for humans
    ,  10000 // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  4  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL 
    ,  TaskCore0);

  xTaskCreatePinnedToCore(
    TaskSDWrite
    ,  "Get Data from Queue"
    ,  10000 // Stack size
    ,  NULL
    ,  3 // Priority
    ,  NULL 
    ,  TaskCore1);

  xTaskCreatePinnedToCore(
    TaskSDFlush
    ,  "Write Data to Card"
    ,  10000 // Stack size
    ,  NULL
    ,  3  // Priority
    ,  NULL 
    ,  TaskCore1);
}

void loop()
{
  // Empty. Things are done in Tasks.
/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/
}

void TaskGetData(void *pvParameters)  // This is a task.
{
  (void) pvParameters;
  
  struct Data_t *pxPointerToxData_t;

  for (;;) // A Task shall never return or exit.
  {
    pxPointerToxData_t = &xData_t; 

    if(xQueueSend( DataQueue, (void *) &pxPointerToxData_t, 12 ) != pdPASS )  //portMAX_DELAY
      {
        //Serial.println("xQueueSend is not working"); 
      }
      
    sensors_event_t event;
    lis.getEvent(&event);
    pxPointerToxData_t->usec = micros();
    pxPointerToxData_t->valueX = event.acceleration.x;
    pxPointerToxData_t->valueY = event.acceleration.y;
    pxPointerToxData_t->valueZ = event.acceleration.z;
    /*Serial.print(pxPointerToxData_t->usec); 
    Serial.print(',');
    Serial.print(pxPointerToxData_t->valueX,5);
    Serial.print(',');
    Serial.print(pxPointerToxData_t->valueY,5);
    Serial.print(',');
    Serial.print(pxPointerToxData_t->valueZ,5);
    Serial.println();*/
    
    //digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    //vTaskDelay(100);  // one tick delay (15ms) in between reads for stability
    //digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    //vTaskDelay(100);  // one tick delay (15ms) in between reads for stability
 
    vTaskDelay(intervalTicks);  // one tick delay (1000 uSec/1 mSec) in between reads for 1000 Hz reading 
    }
    vTaskDelete( NULL );
}
//------------------------------------------------------------------------------
void TaskSDWrite(void *pvParameters)  // This is a task.
{
  (void) pvParameters;
  struct Data_t xData_RCV, *pxData_RCV;
  
  for (;;)
  {

    if(DataQueue != NULL ) 
    {
      if( xQueueReceive( DataQueue, &( pxData_RCV ), 12 ) != pdPASS )   //portMAX_DELAY
      {
        //Serial.println("xQueueRecieve is not working");
      }
        
      // print interval between points
      /*if (last) {
        logfile.print(p->usec - last);
      } else {
        logfile.write("NA");
      }
      last = p->usec;*/
      //for (uint8_t i = 0; i < 10; i++) { 
        logfile.print(pxData_RCV->usec);
        logfile.print(',');
        logfile.print(pxData_RCV->valueX,5);
        logfile.print(',');
        logfile.print(pxData_RCV->valueY,5);
        logfile.print(',');
        logfile.print(pxData_RCV->valueZ,5);
        logfile.println(); 
        /*Serial.print(pxData_RCV->usec); 
        Serial.print(',');
        Serial.print(pxData_RCV->valueX,5);
        Serial.print(',');
        Serial.print(pxData_RCV->valueY,5);
        Serial.print(',');
        Serial.print(pxData_RCV->valueZ,5);
        Serial.println(); */
      //}

        //uint16_t FreeSpace = uxQueueSpacesAvailable( DataQueue ); 
        //Serial.println(FreeSpace); 
        //logfile.flush();
      //uint8_t i = 0;
      }
   }
   vTaskDelete( NULL ); 
}


//------------------------------------------------------------------------------
void TaskSDFlush(void *pvParameters)  // This is a task.
{
  (void) pvParameters;

  for (;;)
  {
    vTaskDelay( 2000 );
    logfile.flush();
    //Serial.println("Flushed file"); 
    
  }
  //vTaskDelete ( NULL ); 
}
