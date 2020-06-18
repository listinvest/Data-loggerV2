// Data logger:  For measuring acceleration from LIS3DH/s
// Hardware:  Adafruit ESP32, Adalogger feather+RTC, 2x LIS3DH accels
// Created:  May 12 2020
// Updated:
// Uses SPI for SD & Accels, hoping for 1000 Hz. sampling 
// files are saves text files, csv = NNNNNNNN.TXT
// See ReadME and photos for additional hook up info

//Use ESP32 duo core
const int TaskCore1  = 1;
const int TaskCore0 = 0;
const int SampleRate = 10000; //Hz, Set sample rate here

//Libraries
//#include <Wire.h>
#include <SPI.h>
#include "SdFat.h"
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>
#include "stdio.h"
#include "esp_system.h" //This inclusion configures the peripherals in the ESP system.
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"

#define LED_BUILTIN LED_BUILTIN //LED light for notification
//------------------------------------------------------------------------------
// SD file definitions
const uint8_t sdChipSelect = 33;
SdFat sd;
SdFile file;
File logfile;
//------------------------------------------------------------------------------
// data type for Queue item
struct Data_t {
  uint32_t usec; 
  float value1X;
  float value1Y;
  float value1Z;
  float value2X;
  float value2Y;
  float value2Z;
} TX_Data_t, RX_Data_t;

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

//------------------------------------------------------------------------------
// define tasks for Sensor Data and SD Write
void TaskLed( void *pvParamaters );
void TaskGetData( void *pvParameters );
void TaskSDWrite( void *pvParameters );
void TaskSDFlush( void *pvParameters );
void TaskSDClose( void *pvParameters );
//------------------------------------------------------------------------------

//Hardware Timer
hw_timer_t * timer = NULL;  //create timer handler

//Declare Queue data type for FreeRTOS
QueueHandle_t DataQueue; // = NULL;

//ISR tools
//Create Interrupt Semaphore
SemaphoreHandle_t timerSemaphore; 
SemaphoreHandle_t ButtonSemaphore;

/////////////////////////////////////////////////////////////////////////////////////
void IRAM_ATTR vTimerISR()  //Timer ISR 
  {
  xSemaphoreGiveFromISR(timerSemaphore, NULL);
  }
//------------------------------------------------------------------------------
void IRAM_ATTR ButtonISR() 
  {
  xSemaphoreGiveFromISR(ButtonSemaphore, NULL); //Gives permission from button interrupt
  }
//------------------------------------------------------------------------------
void TaskGetData(void *pvParameters)  // This is a task.
{
  (void) pvParameters;

  for (;;) // A Task shall never return or exit.
  {
    if (xSemaphoreTake(timerSemaphore, 10) == pdTRUE){
    sensors_event_t event;
    lis.getEvent(&event);
    sensors_event_t event2;
    lis2.getEvent(&event2);
    TX_Data_t.usec = micros();
    TX_Data_t.value1X = event.acceleration.x;
    TX_Data_t.value1Y = event.acceleration.y;
    TX_Data_t.value1Z = event.acceleration.z;
    TX_Data_t.value2X = event2.acceleration.x;
    TX_Data_t.value2Y = event2.acceleration.y;
    TX_Data_t.value2Z = event2.acceleration.z;
    /*Serial.print(TX_Data_t.usec); 
    Serial.print(',');
    Serial.print(TX_Data_t.value1X,5);
    Serial.print(',');
    Serial.print(TX_Data_t.value1Y,5);
    Serial.print(',');
    Serial.print(TX_Data_t.value1Z,5);
    Serial.print(',');
    Serial.print(TX_Data_t.value2X,5);
    Serial.print(',');
    Serial.print(TX_Data_t.value2Y,5);
    Serial.print(',');
    Serial.print(TX_Data_t.value2Z,5);
    Serial.println();*/
    if(xQueueSend( DataQueue, ( void * ) &TX_Data_t, 200 ) != pdPASS )  //portMAX_DELAY
      {
        Serial.println("xQueueSend is not working"); 
      }
    }
  }
  vTaskDelete( NULL );
}
//------------------------------------------------------------------------------
void TaskSDWrite(void *pvParameters)  // This is a task.
{
  (void) pvParameters;
  
  //struct Data_t *RCV_Data; 
  
  for (;;)
  {

      if( xQueueReceive( DataQueue, &( RX_Data_t ), portMAX_DELAY ) != pdPASS )   //portMAX_DELAY
      {
        Serial.println("xQueueRecieve is not working");
      }
      logfile.print(RX_Data_t.usec);
      logfile.print(',');
      logfile.print(RX_Data_t.value1X,4);
      logfile.print(',');
      logfile.print(RX_Data_t.value1Y,4);
      logfile.print(',');
      logfile.print(RX_Data_t.value1Z,4);
      logfile.print(',');
      logfile.print(RX_Data_t.value2X,4);
      logfile.print(',');
      logfile.print(RX_Data_t.value2Y,4);
      logfile.print(',');
      logfile.print(RX_Data_t.value2Z,4);
      logfile.println(); 
      /*Serial.print(RX_Data_t.usec); 
      Serial.print(',');
      Serial.print(RX_Data_t.value1X,5);
      Serial.print(',');
      Serial.print(RX_Data_t.value1Y,5);
      Serial.print(',');
      Serial.print(RX_Data_t.value1Z,5);
      Serial.print(',');
      Serial.print(RX_Data_t.value2X,5);
      Serial.print(',');
      Serial.print(RX_Data_t.value2Y,5);
      Serial.print(',');
      Serial.print(RX_Data_t.value2Z,5);
      Serial.println();*/ 

      uint16_t FreeSpace = uxQueueSpacesAvailable( DataQueue ); 
      Serial.println(FreeSpace);
      }
   vTaskDelete( NULL ); 
}

//------------------------------------------------------------------------------
void TaskSDFlush(void *pvParameters)  // This is a task.
{
  (void) pvParameters;

  for (;;)
  {
    vTaskDelay( pdMS_TO_TICKS(5000) );
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
    vTaskDelay( pdMS_TO_TICKS(6000) );
    logfile.close();
    Serial.println("Close file"); 
  }
  vTaskDelete ( NULL ); 
}
//------------------------------------------------------------------------------

void TaskLed(void *pvParameters)
{
  (void) pvParameters;

  for (;;) 
    {
    // Take the semaphore.
    if (xSemaphoreTake(ButtonSemaphore, portMAX_DELAY) == pdPASS) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    //digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    //vTaskDelay(100);  // one tick delay (15ms) in between reads for stability
    //digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    //vTaskDelay(100);  // one tick delay (15ms) in between reads for stability
    //vTaskDelay(intervalTicks);  // one tick delay (1000 uSec/1 mSec) in between reads for 1000 Hz reading 
    }
   }
}
//===================================================================================================================
//===================================================================================================================
// the setup function runs once when you press reset or power the board
void setup() {

  // initialize serial communication at 115200 bits per second:
  Serial.begin(115200);

  //Queue Setup
  DataQueue = xQueueCreate(10, sizeof( Data_t ));
  if(DataQueue == NULL){
     Serial.println("Error Creating the Queue");
   }

  //============================================================================================================
  //SPI.beginTransaction(SPISettings(80000000, MSBFIRST, SPI_MODE0));

  //Outputs, Pins, Buttons, Etc. 
  pinMode(LED_BUILTIN, OUTPUT);  //set Built in LED to show writing on SD Card
  pinMode(27, INPUT); //button to turn recording on/off, In [HIGH]

  //Create button Interrupt Semaphore
  ButtonSemaphore = xSemaphoreCreateBinary();
  if (ButtonSemaphore != NULL) {
    // Attach interrupt for Arduino digital pin
    attachInterrupt(digitalPinToInterrupt(27), ButtonISR, FALLING);
  }

  // Create semaphore to inform us when the timer has fired
  timerSemaphore = xSemaphoreCreateBinary();
  
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
  logfile = sd.open(filename, O_CREAT | O_WRITE);  
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
    ,  4  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL 
    ,  TaskCore1);

  xTaskCreatePinnedToCore(
    TaskSDWrite
    ,  "Get Data from Queue"
    ,  10000 // Stack size
    ,  NULL
    ,  5 // Priority
    ,  NULL 
    ,  TaskCore0);

  xTaskCreatePinnedToCore(
    TaskSDFlush
    ,  "Write Data to Card"
    ,  2000 // Stack size
    ,  NULL
    ,  3  // Priority
    ,  NULL 
    ,  TaskCore1);

  xTaskCreatePinnedToCore(
    TaskSDClose
    ,  "Close the File"
    ,  2000 // Stack size
    ,  NULL
    ,  3  // Priority
    ,  NULL 
    ,  TaskCore0);  

    xTaskCreatePinnedToCore(
    TaskLed
    ,  "LED"
    ,  2000 // Stack size
    ,  NULL
    ,  3  // Priority
    ,  NULL 
    ,  TaskCore0);  
  
// Create Timer ===============================================================================
  // Use 1st timer of 4 (counted from zero).
  // Set 80 divider for prescaler (see ESP32 Technical Reference Manual for more
  // info).
  timer = timerBegin(0, 80, true);
  // Attach onTimer function to our timer.
  timerAttachInterrupt(timer, &vTimerISR, true);
  // Set alarm to call onTimer function every second (value in microseconds).
  // Repeat the alarm (third parameter)
  timerAlarmWrite(timer, SampleRate, true);
  // Start an alarm
  timerAlarmEnable(timer);
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
