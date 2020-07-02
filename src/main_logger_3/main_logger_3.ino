// Data logger:  For measuring acceleration from LIS3DH/s
// Hardware:  Adafruit ESP32, Adalogger feather+RTC, 2x LIS3DH accels
// Created:  May 12 2020
// Updated:
// Uses SPI for SD & Accels, hoping for 1000 Hz. sampling 
// files are saves text files = DATANN.TXT
// See ReadME and photos for additional hook up info
 /* Connect the SD card to the following pins:
 *
 * SD Card | ESP32
 *    D2       12
 *    D3       13
 *    CMD      15
 *    VSS      GND
 *    VDD      3.3V
 *    CLK      14
 *    VSS      GND
 *    D0       2  (add 1K pull up after flashing)
 *    D1       4
*/
 
//SPI3 / 
//    CS      5
//    SCK     18
//    MISO/SDA   19
//    MOSI/SDO    23

const int SampleRate = 500; //Hz, Set sample rate here
const int SampleLength = 80; //Seconds, Sample Length in Seconds

//Use ESP32 duo core
const int TaskCore1  = 1;
const int TaskCore0 = 0;
int SampleInt = 1000000 / SampleRate; 
int TotalCount = SampleLength * SampleRate;

//Libraries
#include "FS.h"
#include "SD_MMC.h"
#include <SPI.h>
//#include "SdFat.h"
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

#define ONE_BIT_MODE true //false is 4 bit mode, fastest

//#define LED_BUILTIN LED_BUILTIN //LED light for notification
//------------------------------------------------------------------------------

//SdFat sd;
//SdFile file;
File logfile;
//------------------------------------------------------------------------------
// data type for Queue item
struct Data_t {
  uint32_t usec; 
  float value1X;
  float value1Y;
  //float value1Z;
  float value2X;
  float value2Y;
  //float value2Z;
} TX_Data_t, RX_Data_t;

//char buffer[50];

UBaseType_t uxHighWaterMark;

//------------------------------------------------------------------------------
// Accel Lis3dh definitions, SPI or I2C
// hardware SPI 1 LIS3DH->Feather:  Power to Vin, Gnd to Gnd, SCL->SCK, SDA->MISO, SDO->MOSI, CS->CS 14/15
// Sensor 1 Hardware SPI
//Adafruit_LIS3DH lis = Adafruit_LIS3DH(LIS3DH_CS);
// Sensor 2 Hardware SPI
//Adafruit_LIS3DH lis2 = Adafruit_LIS3DH(LIS3DH_CS2);

// Used for software SPI
#define LIS3DH_CLK 18
#define LIS3DH_MISO 19
#define LIS3DH_MOSI 23
#define LIS3DH2_CLK 18
#define LIS3DH2_MISO 19
#define LIS3DH2_MOSI 23
// Used for hardware & software SPI
#define LIS3DH_CS 5    //ESP32: 14/A6 , Cortex m0: 5, Use for upper accel (Sensor 1!!!) = hbar, seatpost, etc.
#define LIS3DH2_CS 17  //ESP32: 15/A8, Cortex m0: 9, Use for lower accel (Sensor 2!!!) = axles, etc. 

// software SPI
Adafruit_LIS3DH lis = Adafruit_LIS3DH(LIS3DH_CS, LIS3DH_MOSI, LIS3DH_MISO, LIS3DH_CLK);
// software SPI 2 
Adafruit_LIS3DH lis2 = Adafruit_LIS3DH(LIS3DH2_CS, LIS3DH2_MOSI, LIS3DH2_MISO, LIS3DH2_CLK);

//------------------------------------------------------------------------------
// define tasks for Sensor Data and SD Write
void TaskLed( void *pvParamaters );
void TaskGetData( void *pvParameters );
void TaskSDWrite( void *pvParameters );
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
SemaphoreHandle_t CountSemaphore; 
int Count = 0; 
int G = 0;
int H = 0; 
int F = 0;

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
    if (xSemaphoreTake(timerSemaphore, portMAX_DELAY ) == pdTRUE )
    {
    sensors_event_t event;
    lis.getEvent(&event);
    sensors_event_t event2;
    lis2.getEvent(&event2);
    TX_Data_t.usec = micros();
    TX_Data_t.value1X = event.acceleration.x;
    TX_Data_t.value1Y = event.acceleration.y;
    //TX_Data_t.value1Z = event.acceleration.z;
    TX_Data_t.value2X = event2.acceleration.x;
    TX_Data_t.value2Y = event2.acceleration.y;
    //TX_Data_t.value2Z = event2.acceleration.z;
    //uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
    //Serial.println(uxHighWaterMark);
    if(xQueueSend( DataQueue, ( void * ) &TX_Data_t, portMAX_DELAY ) != pdPASS )  //portMAX_DELAY
      {
        Serial.println("xQueueSend is not working"); 
      }
    /*if(UBaseType_t res =  xRingbufferSend(buf_handle, ( void * ) &TX_Data_t, sizeof(Data_t), portMAX_DELAY) != pdPASS)
      {
        printf("Failed to send in Ring Buffer");
      }*/
    }
  }
  vTaskDelete( NULL );
}
//------------------------------------------------------------------------------
void TaskSDWrite(void *pvParameters)  // This is a task.
{
  (void) pvParameters;
  
  //struct Data_t *RCV_Data; 
  //size_t RX_Data_t; 
  
  for (;;)
  {

      if( xQueueReceive( DataQueue, &( RX_Data_t ), portMAX_DELAY ) != pdPASS )   //portMAX_DELAY
      {
        Serial.println("xQueueRecieve is not working");
      }
      /*//Receive an item from no-split ring buffer
      if( xRingbufferReceive(buf_handle, &( RX_Data_t ), portMAX_DELAY) != pdPASS );

    //Check received item
    if (RX_Data_t != NULL) {
        //Print item
        for (int i = 0; i < item_size; i++) {
            printf("%c", item[i]);
        }
        printf("\n");
        //Return Item
        //vRingbufferReturnItem(buf_handle, (void *)item);
    } else {
        //Failed to receive item
        printf("Failed to receive item\n");
    }*/
      logfile.print(RX_Data_t.usec);
      logfile.print(',');
      logfile.print(RX_Data_t.value1X,4);
      logfile.print(',');
      logfile.print(RX_Data_t.value1Y,4);
      //logfile.print(',');
      //logfile.print(RX_Data_t.value1Z,4);
      logfile.print(',');
      logfile.print(RX_Data_t.value2X,4);
      logfile.print(',');
      logfile.print(RX_Data_t.value2Y,4);
      //logfile.print(',');
      //logfile.print(RX_Data_t.value2Z,4);
      //sprintf(buffer, "%u, %f, %f, %f, %f", RX_Data_t.usec, RX_Data_t.value1X, RX_Data_t.value1Y, RX_Data_t.value2X, RX_Data_t.value2Y);
      //logfile.println(buffer); 
      logfile.println(); 
      //uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
      //Serial.println(uxHighWaterMark);
      Count++;
      //Serial.println(Count); 
      if ( Count == TotalCount )
        {
          logfile.close(); 
          
          vTaskDelay ( 8000 / portTICK_PERIOD_MS ); 
          Serial.println("All done here"); 
          vTaskDelay( 20000000 / portTICK_PERIOD_MS );
          //vTaskSuspendAll(); 
        }

      uint16_t FreeSpace = uxQueueSpacesAvailable( DataQueue ); 
      Serial.println(FreeSpace);
      }
   vTaskDelete( NULL ); 
}

//------------------------------------------------------------------------------
void TaskLed(void *pvParameters)
{
  (void) pvParameters;

  for (;;) 
    {
    // Take the semaphore.
    if( xSemaphoreTake(CountSemaphore, portMAX_DELAY) == pdPASS )
        {
        //vTaskSuspend( (void *) &TaskGetData );
        //vTaskSuspend( (void *) &TaskSDWrite );
        //Serial.println("Recieved count semaphore"); 
        //Serial.println("All done here");
        //vTaskDelay( 20000000 / portTICK_PERIOD_MS );
        //vTaskSuspendAll(); 
        }  
       
    if (xSemaphoreTake(ButtonSemaphore, portMAX_DELAY) == pdPASS) {
    //digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    vTaskDelay( 10 ); 
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
  //pinMode(LED_BUILTIN, OUTPUT);  //set Built in LED to show writing on SD Card
  pinMode(27, INPUT); //button to turn recording on/off, In [HIGH]

  //Create button Interrupt Semaphore
  ButtonSemaphore = xSemaphoreCreateBinary();
  if (ButtonSemaphore != NULL) {
    // Attach interrupt for Arduino digital pin
    attachInterrupt(digitalPinToInterrupt(27), ButtonISR, FALLING);
  }

  // Create semaphore to inform us when the timer has fired
  timerSemaphore = xSemaphoreCreateBinary();

  // Create semaphore for counting samples
  CountSemaphore = xSemaphoreCreateBinary(); 

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
  Serial.println(SampleInt); //,"Sample time interval");
  Serial.println(TotalCount); //,"Total # of samples");
  
  // Set accel range  
  lis.setRange(LIS3DH_RANGE_16_G);   // 2, 4, 8 or 16 G!
  lis2.setRange(LIS3DH_RANGE_16_G);
  // Set DataRate
  lis.setDataRate(LIS3DH_DATARATE_LOWPOWER_5KHZ); //OPTIONS:  LIS3DH_DATARATE_400_HZ, LIS3DH_DATARATE_LOWPOWER_1K6HZ, LIS3DH_DATARATE_LOWPOWER_5KHZ
  lis2.setDataRate(LIS3DH_DATARATE_LOWPOWER_5KHZ); 

  // SD CARD SETUP ====================================================================
  // see if the card is present and can be initialized:  (Use highest SD clock possible, but lower if has error, 35 Mhz works)
  if (!SD_MMC.begin("/sdcard", ONE_BIT_MODE)) {
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
    if (! SD_MMC.exists(filename)) {
      break;
    }
  }

  // Create file and prepare it ============================================================
  logfile = SD_MMC.open(filename, FILE_WRITE); //O_RDWR | O_CREAT | O_TRUNC); //O_CREAT | O_WRITE);  
  if( ! logfile ) {
    Serial.print("Couldnt create "); 
    Serial.println(filename);
    while (1) yield(); 
  }
  Serial.print("Writing to "); 
  Serial.println(filename);

  //Column labels
  logfile.print("Time uS"); 
  logfile.print(",");
  logfile.print("Sensor 1 X");
  logfile.print(",");
  logfile.print("Sensor 1 Y");
  //logfile.print(",");
  //logfile.print("Sensor 1 Z");
  logfile.print(",");
  logfile.print("Sensor 2 X");
  logfile.print(",");
  logfile.print("Sensor 2 Y");
  //logfile.print(",");
  //logfile.print("Sensor 2 Z");
  logfile.println();

  // Setup up Tasks and where to run ============================================================  
  // Now set up tasks to run independently.
  xTaskCreatePinnedToCore(
    TaskGetData
    ,  "Grab Accel Data"   // A name just for humans
    ,  15000 // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  3  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL 
    ,  TaskCore1);

  xTaskCreatePinnedToCore(
    TaskSDWrite
    ,  "SD Write"
    ,  15000 // Stack size
    ,  NULL
    ,  3 // Priority
    ,  NULL 
    ,  TaskCore0);

    xTaskCreatePinnedToCore(
    TaskLed
    ,  "LED"
    ,  10000 // Stack size
    ,  NULL
    ,  1  // Priority
    ,  NULL 
    ,  TaskCore1);  
  
// Create Timer ===============================================================================
  // Use 1st timer of 4 (counted from zero).
  // Set 80 divider for prescaler (see ESP32 Technical Reference Manual for more
  // info).
  timer = timerBegin(1, 80, true);
  // Attach onTimer function to our timer.
  timerAttachInterrupt(timer, &vTimerISR, true);
  // Set alarm to call onTimer function every second (value in microseconds).
  // Repeat the alarm (third parameter)
  timerAlarmWrite(timer, SampleInt, true);
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
