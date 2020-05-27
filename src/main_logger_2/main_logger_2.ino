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

// Accel Lis3dh definitions, I2C
#define LIS3DH_CS 16 //should be 32  //ESP32: 14/A6 , Cortex m0: 5, Use for upper accel, hbar, seatpost, etc.
// Sensor I2C 
Adafruit_LIS3DH lis = Adafruit_LIS3DH();

// define two tasks for Blink & AnalogRead
void TaskGetData( void *pvParameters );
//void TaskSDWrite( void *pvParameters );

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

  

  
// Setup up Taks and where to run ============================================================  
  // Now set up two tasks to run independently.
  xTaskCreatePinnedToCore(
    TaskGetData
    ,  "GetData"   // A name just for humans
    ,  2048  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL 
    ,  ARDUINO_RUNNING_CORE);

  xTaskCreatePinnedToCore(
    TaskSDWrite
    ,  "SDWrite"
    ,  1024  // Stack size
    ,  NULL
    ,  1  // Priority
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
    
  // initialize digital LED_BUILTIN on pin 13 as an output.
  //pinMode(LED_BUILTIN, OUTPUT);

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
    //vTaskDelay(1000);  // one tick delay (15ms) in between reads for stability
    
    //digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    //vTaskDelay(100);  // one tick delay (15ms) in between reads for stability
    //digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    //vTaskDelay(100);  // one tick delay (15ms) in between reads for stability
  }
}

void TaskSDWrite(void *pvParameters)  // This is a task.
{
  (void) pvParameters;
  
/*
  AnalogReadSerial
  Reads an analog input on pin A3, prints the result to the serial monitor.
  Graphical representation is available using serial plotter (Tools > Serial Plotter menu)
  Attach the center pin of a potentiometer to pin A3, and the outside pins to +5V and ground.

  This example code is in the public domain.
*/

  for (;;)
  {
    // read the input on analog pin A3:
    int sensorValueA3 = analogRead(A3);
    // print out the value you read:
    //Serial.println(sensorValueA3);
    vTaskDelay(10);  // one tick delay (15ms) in between reads for stability
  }
  
}
