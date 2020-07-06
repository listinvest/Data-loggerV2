// Data logger:  For measuring acceleration from LIS3DH/s
// Hardware:  ESP32 dev Board Kit C, Micro SD reader, 2x LIS3DH accels
// Created:  May 12 2020
// Updated:
// Uses SPI for Accels, SDMMC for SD
// Record reliably up to 1000 Hz.  
// files are saved in .CSV or .TXT format, file name scheme is = DATANN.CSV or .TXT
// See ReadME and photos for additional hook up info
// You cannot compile and upload with SD card in slot!

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
const int SampleRate = 1000; //Hz, Set sample rate here                    +
const int SampleLength = 10; //Seconds, How long to record                +
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//Use ESP32 duo core
const int TaskCore1  = 1;
const int TaskCore0 = 0;
int SampleInt = 1000000 / SampleRate; 
int TotalCount = SampleLength * SampleRate;

//Libraries
#include "FS.h"
#include "SD_MMC.h"
#include <SPI.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>
#include "stdio.h"
#include "esp_system.h" //This inclusion configures the peripherals in the ESP system.
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#define ONE_BIT_MODE true //true = 1 bit mode, 4 bit mode is faster (false) but never got to work

//------------------------------------------------------------------------------
//File callout for SDMMC
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

UBaseType_t uxHighWaterMark;  // Use for debug and reviewing stack height limits

//------------------------------------------------------------------------------
// Accel Lis3dh definitions, SPI 
// Connect the SD card to the following pins:
// SD Card | ESP32, DS->12 D3->13 CMD->15 VSS->GND VDD->3.3V CLK->14 VSS->GND D0->2 D1->4
// Power to Vin, Gnd to Gnd, SCL->SCK, SDA->MISO, SDO->MOSI, CS->CS 
//SPI3 / VSPI / CS = 5 / SCK = 18 / MISO/SDA = 19 / MOSI/SDO = 23
// Used for SPI3/VSPI
#define LIS3DH_CLK 18
#define LIS3DH_MISO 19
#define LIS3DH_MOSI 23
#define LIS3DH2_CLK 18
#define LIS3DH2_MISO 19
#define LIS3DH2_MOSI 23
// Used for hardware & software SPI
#define LIS3DH_CS 5    //ESP32: Use for upper accel (Sensor 1!!!) = hbar, seatpost, etc.
#define LIS3DH2_CS 17  //ESP32: Use for lower accel (Sensor 2!!!) = axles, etc. 

// Selct Pins
Adafruit_LIS3DH lis = Adafruit_LIS3DH(LIS3DH_CS, LIS3DH_MOSI, LIS3DH_MISO, LIS3DH_CLK);
Adafruit_LIS3DH lis2 = Adafruit_LIS3DH(LIS3DH2_CS, LIS3DH2_MOSI, LIS3DH2_MISO, LIS3DH2_CLK);

//------------------------------------------------------------------------------
// define tasks for Sensor Data and SD Write
void TaskLed( void *pvParamaters );
void TaskGetData( void *pvParameters );
void TaskSDWrite( void *pvParameters );
TaskHandle_t xGetData;
TaskHandle_t xSDWrite;
TaskHandle_t xLed; 
//------------------------------------------------------------------------------

//Hardware Timer
hw_timer_t * timer = NULL;  //create timer handler

//Declare Queue data type for FreeRTOS
QueueHandle_t DataQueue; // 
//RingbufHandle_t buf_handle;

//ISR tools
//Create Interrupt Semaphores
SemaphoreHandle_t timerSemaphore; 
SemaphoreHandle_t ButtonSemaphore;
int Count = 0; 
int LED = 32; 
int BUTTON = 34; 

//------------------------------------------------------------------------------
void IRAM_ATTR vTimerISR()  //Timer ISR 
  {
  xSemaphoreGiveFromISR(timerSemaphore, NULL);
  }
//------------------------------------------------------------------------------
void IRAM_ATTR ButtonISR() //Gives permission from button interrupt
  {
  xSemaphoreGiveFromISR(ButtonSemaphore, NULL); 
  }
//------------------------------------------------------------------------------
void TaskGetData(void *pvParameters)  // Get Data from Sensors Task
{
    (void) pvParameters;

  for (;;) // A Task shall never return or exit.
  {
    if (xSemaphoreTake(timerSemaphore, portMAX_DELAY ) == pdTRUE )
    {
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
    //uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
    //Serial.println(uxHighWaterMark);
    if(xQueueSend( DataQueue, ( void * ) &TX_Data_t, portMAX_DELAY ) != pdPASS )  //portMAX_DELAY
      {
        Serial.println("xQueueSend is not working"); 
      }
    }
  }
  vTaskDelete( NULL );
}
//------------------------------------------------------------------------------
void TaskSDWrite(void *pvParameters)  // Write Data to SD card
{
  (void) pvParameters;
  
  for (;;)
  {

      if( xQueueReceive( DataQueue, &( RX_Data_t ), portMAX_DELAY ) != pdPASS )   //portMAX_DELAY
      {
        Serial.println("xQueueRecieve is not working");
      }
      logfile.print(RX_Data_t.usec);
      logfile.print(',');
      logfile.print(RX_Data_t.value1X,5);
      logfile.print(',');
      logfile.print(RX_Data_t.value1Y,5);
      logfile.print(',');
      logfile.print(RX_Data_t.value1Z,5);
      logfile.print(',');
      logfile.print(RX_Data_t.value2X,5);
      logfile.print(',');
      logfile.print(RX_Data_t.value2Y,5);
      logfile.print(',');
      logfile.print(RX_Data_t.value2Z,5);
      logfile.println(); 
      //uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
      //Serial.println(uxHighWaterMark);
      Count++;
      //Serial.println(Count); 
      if ( Count == TotalCount )
        {
          logfile.close(); 
          
          vTaskDelay ( 8000 / portTICK_PERIOD_MS ); // Adding delay just to give SD card time close but may not be needed
          Serial.println("Data Saved to SD Card and Closed"); 
          digitalWrite(LED, LOW); 
          vTaskSuspend( xGetData );
          vTaskSuspend( NULL );
        }

      //uint16_t FreeSpace = uxQueueSpacesAvailable( DataQueue ); //Use if need to evaluate SD write and queue availability
      //Serial.println(FreeSpace);
      }
   vTaskDelete( NULL ); 
}

//------------------------------------------------------------------------------
void TaskLed(void *pvParameters)  //Always on Task, start task and Led notifications
{
  (void) pvParameters;

  // Start off Suspended, no recording, only runs once here
  vTaskSuspend( xGetData );
  vTaskSuspend( xSDWrite );

  for (;;) 
    {
     if (xSemaphoreTake(ButtonSemaphore, portMAX_DELAY) == pdPASS) 
      {
      digitalWrite(LED, !digitalRead(LED));
      Serial.println("Button Pressed, Writing"); 
      vTaskResume( xGetData );
      vTaskResume( xSDWrite ); 
      static uint32_t lastMillis = 0;
      vTaskDelay( 10000 / portTICK_PERIOD_MS );
      } 
    }
}
//===================================================================================================================
//===================================================================================================================
// the setup function runs once when you press reset or power the board
void setup() {

  // initialize serial communication at 115200 bits per second:
  Serial.begin(115200);

  // For SDMMC hookup need to pull certain channels high during startup
  pinMode(2, INPUT_PULLUP);
  pinMode(4, INPUT_PULLUP);
  pinMode(12, INPUT_PULLUP);
  pinMode(13, INPUT_PULLUP);
  pinMode(15, INPUT_PULLUP);

  //Queue Setup
  DataQueue = xQueueCreate(10, sizeof( Data_t ));
  if(DataQueue == NULL){
     Serial.println("Error Creating the Queue");
   }

  /*buf_handle = xRingbufferCreate(1028, RINGBUF_TYPE_NOSPLIT);
  if (buf_handle == NULL){
        printf("Failed to create ring buffer\n");
    }*/

  //============================================================================================================
  //Outputs, Pins, Buttons, Etc. 
  pinMode(LED, OUTPUT);  // Turn on LED for notification
  pinMode(BUTTON, INPUT); //button to turn recording on/off, Pulled HIGH to begin
  digitalWrite(LED, LOW); // Turn LED on

  ButtonSemaphore = xSemaphoreCreateBinary();  //Create button Interrupt Semaphore
  if (ButtonSemaphore != NULL) {
    // Attach interrupt for Arduino digital pin
    attachInterrupt(digitalPinToInterrupt(BUTTON), ButtonISR, FALLING);
  }

  timerSemaphore = xSemaphoreCreateBinary();  // Create semaphore to inform us when the timer has fired

  //ACCEL Setup and RUN
  if (! lis.begin(0x18)) {   // Sensor 1, change this to 0x19 for alternative i2c address
  Serial.println("Couldnt start");
  while (1) yield();
  }
  Serial.println("LIS3DH Sensor 1 found!");

  if (! lis2.begin(0x19)) {   // Sensor 2
  Serial.println("Couldnt start Sensor 2");
  while (1) yield();
  }
  Serial.println("LIS3DH Sensor 2 found!");
  Serial.print("Sample Interval time (uS):"); Serial.println(SampleInt); 
  Serial.print("Total # of samples:"); Serial.println(TotalCount); 
  
  // Set accel range  2, 4, 8 or 16 G!
  lis.setRange(LIS3DH_RANGE_16_G);   
  lis2.setRange(LIS3DH_RANGE_16_G);
  // Set DataRate  OPTIONS:  LIS3DH_DATARATE_400_HZ, LIS3DH_DATARATE_LOWPOWER_1K6HZ, LIS3DH_DATARATE_LOWPOWER_5KHZ
  lis.setDataRate(LIS3DH_DATARATE_LOWPOWER_5KHZ); 
  lis2.setDataRate(LIS3DH_DATARATE_LOWPOWER_5KHZ); 

  // SD CARD SETUP ====================================================================
  // see if the card is present and can be initialized:  
  if (!SD_MMC.begin("/sdcard", ONE_BIT_MODE)) {
    Serial.println("Card init. failed!");
    while (1) yield(); 
  }

  // Create filename scheme ====================================================================
  char filename[15];  //  Setup filename 
  strcpy(filename, "/DATA00.TXT");
  for (uint8_t i = 0; i < 100; i++) {
    filename[5] = '0' + i/10;
    filename[6] = '0' + i%10;
    // create if does not exist, do not open existing, write, sync after write
    if (! SD_MMC.exists(filename)) {
      break;
    }
  }

  // Create file and prepare it ============================================================
  logfile = SD_MMC.open(filename, FILE_WRITE); 
  if( ! logfile ) {
    Serial.print("Couldnt create "); 
    Serial.println(filename);
    while (1) yield(); 
  }
  Serial.print("Ready to write to:"); Serial.println(filename);

  //Column labels, change if not doing full 6 channel output
  logfile.print("Time uS"); 
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
    ,  "Grab Accel Data"   // A name just for humans
    ,  15000 // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  3  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  &xGetData
    ,  TaskCore1);

  xTaskCreatePinnedToCore(
    TaskSDWrite
    ,  "SD Write"
    ,  15000 // Stack size
    ,  NULL
    ,  3 // Priority
    ,  &xSDWrite
    ,  TaskCore0);

    xTaskCreatePinnedToCore(
    TaskLed
    ,  "LED" // Use for LED and button interrupts
    ,  10000 // Stack size
    ,  NULL
    ,  1  // Priority
    ,  &xLed
    ,  TaskCore1);  
  
  // Create Timer ===============================================================================
  timer = timerBegin(1, 80, true);    // Set 80 divider for prescaler (see ESP32 Technical Reference Manual for more info).
  timerAttachInterrupt(timer, &vTimerISR, true);  // Attach &vTimerISR to our timer.
  timerAlarmWrite(timer, SampleInt, true);  // Repeat the alarm (third parameter)
  timerAlarmEnable(timer);  // Start an alarm

  // LED notification system is ready
  int i = 0; 
  for (i=0; i <=7; i++){
    digitalWrite(LED, HIGH);
    delay(100);
    digitalWrite(LED, LOW);
    delay(100); 
    }

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
