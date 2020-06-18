/*
 * Example of a Arduino interruption and RTOS Binary Semaphore
 * https://www.freertos.org/Embedded-RTOS-Binary-Semaphores.html
 */


// Include Arduino FreeRTOS library
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
// Include semaphore supoport
#include "freertos/semphr.h"

#define LED_BUILTIN LED_BUILTIN
/* 
 * Declaring a global variable of type SemaphoreHandle_t 
 * 
 */
SemaphoreHandle_t interruptSemaphore;

void setup() {

  Serial.begin(9600);
  
  // Configure pin 2 as an input and enable the internal pull-up resistor
  pinMode(27, INPUT);

 // Create task for Arduino led 
  xTaskCreate(TaskLed, // Task function
              "Led", // Task name
              2048, // Stack size 
              NULL, 
              5, // Priority
              NULL );

  /**
   * Create a binary semaphore.
   * https://www.freertos.org/xSemaphoreCreateBinary.html
   */
  interruptSemaphore = xSemaphoreCreateBinary();
  if (interruptSemaphore != NULL) {
    // Attach interrupt for Arduino digital pin
    attachInterrupt(digitalPinToInterrupt(27), interruptHandler, FALLING);
  }

  
}

void loop() {}


void interruptHandler() {
  /**
   * Give semaphore in the interrupt handler
   * https://www.freertos.org/a00124.html
   */
  
  xSemaphoreGiveFromISR(interruptSemaphore, NULL);
}


/* 
 * Led task. 
 */
void TaskLed(void *pvParameters)
{
  (void) pvParameters;

  pinMode(LED_BUILTIN, OUTPUT);

  for (;;) {
    
    /**
     * Take the semaphore.
     * https://www.freertos.org/a00122.html
     */
     Serial.println("what is happening?"); 
    if (xSemaphoreTake(interruptSemaphore, portMAX_DELAY) == pdPASS) {
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      Serial.println("light it up");
    }
    else {
      Serial.println("nothing");
    }
  }
}
