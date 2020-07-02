// Data logger for acceleration at hubs and touch points 
// Hardware:  Adafruit ESP32, Adalogger feather+RTC, 2x LIS3DH accels, LED, push button
// Created:  Sep 7 2019
// Updated:  May 10 2020
// Uses SPI for everything, this has now helped me achieve >2000 Hz . . but with regular lag intervals.  This is about 500 uS records with gaps inbetween when data overruns card write.
// Setup to record 3 minutes, close file, and then ready again.  
// files are saves text files, tab spacing = NNNNNNNN.TXT
// This works, must have all sensors hooked up and SD card
// See ReadME and photos for additional hook up info
// 

//#include <Arduino.h>
//#include <SPI.h>
//#include <SdFat.h>
#include "FS.h"
#include "SD_MMC.h"
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>
#include "stdio.h"
#include "esp_system.h" //This inclusion configures the peripherals in the ESP system.
#include <stdio.h>
#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_err.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "driver/sdmmc_host.h"
#include "driver/sdspi_host.h"
#include "sdmmc_cmd.h"

static const char *TAG = "example";

#define ONE_BIT_MODE true //false is 4 bit mode, fastest

const int SampleRate = 700; //Hz, Set sample rate here
const int SampleLength = 40; //Seconds, Sample Length in Seconds

//Use ESP32 duo core
const int TaskCore1  = 1;
const int TaskCore0 = 0;
int SampleInt = 1000000 / SampleRate; 
int TotalCount = SampleLength * SampleRate;

//SdFat sd;
//SdFile file;
File logfile;
//File fs; 
//FS SD_MMC; 
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

// Used for hardware & software SPI
#define LIS3DH_CS 5  //ESP32: 14/A6 , Cortex m0: 5, Use for upper accel, hbar, seatpost, etc.
#define LIS3DH2_CS 17  //ESP32: 15/A8, Cortex m0: 9, Use for lower accel, axles, etc. 

// hardware SPI 1 LIS3DH->Feather:  Power to Vin, Gnd to Gnd, SCL->SCK, SDA->MOSI, SDO->MISO, CS->CS 14 or 15
// Sensor 1 
//Adafruit_LIS3DH lis = Adafruit_LIS3DH(LIS3DH_CS);
// Sensor 2 
// hardware SPI 2
//Adafruit_LIS3DH lis2 = Adafruit_LIS3DH(LIS3DH_CS2);
//  
// Used for software SPI
#define LIS3DH_CLK 18
#define LIS3DH_MISO 19
#define LIS3DH_MOSI 23
#define LIS3DH2_CLK 18
#define LIS3DH2_MISO 19
#define LIS3DH2_MOSI 23

// software SPI
Adafruit_LIS3DH lis = Adafruit_LIS3DH(LIS3DH_CS, LIS3DH_MOSI, LIS3DH_MISO, LIS3DH_CLK);
// software SPI 2 
Adafruit_LIS3DH lis2 = Adafruit_LIS3DH(LIS3DH2_CS, LIS3DH2_MOSI, LIS3DH2_MISO, LIS3DH2_CLK);

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
int count = 0; 
int Micro = 0;

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

//=============================================================================================
//=============================================================================================
void setup() {

  Serial.begin(115200);  //230400 works 

  pinMode(2, INPUT_PULLUP);
  pinMode(4, INPUT_PULLUP);
  pinMode(12, INPUT_PULLUP);
  pinMode(13, INPUT_PULLUP);
  pinMode(15, INPUT_PULLUP);

  //pinMode(button, INPUT); //button to turn recording on/off

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

  /*ESP_LOGI(TAG, "Initializing SD card");
  Serial.print("Initializing SD Card"); 

  //#ifndef USE_SPI_MODE
    ESP_LOGI(TAG, "Using SDMMC peripheral");
    Serial.print("Using SDMMC peripheral");
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();

    // This initializes the slot without card detect (CD) and write protect (WP) signals.
    // Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.
    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();

    // To use 1-line SD mode, uncomment the following line:
    // slot_config.width = 1;

    // GPIOs 15, 2, 4, 12, 13 should have external 10k pull-ups.
    // Internal pull-ups are not sufficient. However, enabling internal pull-ups
    // does make a difference some boards, so we do that here.
    gpio_set_pull_mode(GPIO_NUM_15, GPIO_PULLUP_ONLY);   // CMD, needed in 4- and 1- line modes
    gpio_set_pull_mode(GPIO_NUM_2, GPIO_PULLUP_ONLY);    // D0, needed in 4- and 1-line modes
    gpio_set_pull_mode(GPIO_NUM_4, GPIO_PULLUP_ONLY);    // D1, needed in 4-line mode only
    gpio_set_pull_mode(GPIO_NUM_12, GPIO_PULLUP_ONLY);   // D2, needed in 4-line mode only
    gpio_set_pull_mode(GPIO_NUM_13, GPIO_PULLUP_ONLY);   // D3, needed in 4- and 1-line modes

        // Options for mounting the filesystem.
    // If format_if_mount_failed is set to true, SD card will be partitioned and
    // formatted in case when mounting fails.
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };

    // Use settings defined above to initialize SD card and mount FAT filesystem.
    // Note: esp_vfs_fat_sdmmc_mount is an all-in-one convenience function.
    // Please check its source code and implement error recovery when developing
    // production applications.
    sdmmc_card_t* card;
    esp_err_t ret = esp_vfs_fat_sdmmc_mount("/sdcard", &host, &slot_config, &mount_config, &card);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount filesystem. "
                "If you want the card to be formatted, set format_if_mount_failed = true.");
        } else {
            ESP_LOGE(TAG, "Failed to initialize the card (%s). "
                "Make sure SD card lines have pull-up resistors in place.", esp_err_to_name(ret));
        }
        return;
    }

    // Card has been initialized, print its properties
    sdmmc_card_print_info(stdout, card);  //SD_CHIP_SELECT, SD_SCK_MHZ(15)

    // Use POSIX and C standard library functions to work with files.
    // First create a file.
    ESP_LOGI(TAG, "Opening file");
    FILE* f = fopen("/sdcard/hello.txt", "w");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for writing");
        return;
        }*/

    char filename[15];
    //  Setup filename to be appropriate for what you are testing
    strcpy(filename, "/DATA00.TXT");

    if(!SD_MMC.begin("/sdcard", ONE_BIT_MODE)){
        Serial.println("Card Mount Failed");
        return;
    }

    
    logfile = SD_MMC.open(filename, FILE_WRITE);
    if(!filename){
        Serial.println("Failed to open file for writing");
        return;
    }    

    //Column labels ===============================================================================
    /*fprintf(f,"Time"); 
    fprintf(f, "\t");
    fprintf(f, "Sensor 1 X");
    fprintf(f, "\t");
    fprintf(f, "Sensor 1 Y");
    fprintf(f, "\t");
    fprintf(f, "Sensor 1 Z");
    fprintf(f, "\t");
    fprintf(f, "Sensor 2 X");
    fprintf(f, "\t");
    fprintf(f, "Sensor 2 Y");
    fprintf(f, "\t");
    fprintf(f, "Sensor 2 Z");
    fprintf(f, "\n");*/

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
 



  // Create semaphore to inform us when the timer has fired
  timerSemaphore = xSemaphoreCreateBinary();

  // Create semaphore for counting samples
  CountSemaphore = xSemaphoreCreateBinary(); 
  
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
  
//=============================================================================================
//=============================================================================================
void loop() {

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
    
      //Serial for debug
      Serial.print(" \tX: "); Serial.print(event.acceleration.x,4);
      Serial.print(" \tY: "); Serial.print(event.acceleration.y,4); 
      Serial.print(" \tZ: "); Serial.print(event.acceleration.z,4); 
      Serial.print(" \tX: "); Serial.print(event2.acceleration.x,4);
      Serial.print(" \tY: "); Serial.print(event2.acceleration.y,4); 
      Serial.print(" \tZ: "); Serial.print(event2.acceleration.z,4); 
      Serial.println(" m/s^2 "); 

      
      }
}

//==================datalogging loop=====================================
void DataLogger() {

  //MicroInt = micros(); 
  // Do appropriate calculations to decide how much data you want.  
  // 360000 samples at 500hz = 3 minutes of data AQ
  //for (int x = 0; x < 10000; x++){

  //Counter
  Micro = micros();
  logfile.print(Micro); 
  logfile.print("\t");

  // log raw data from Sensor 1 and 2
  // Raw Data
  // Note:  must normalize raw data which means measured values are between -32768 and 32767.  -Range G/+Range G = -32768/32767 
  lis.read();     // get X Y and Z data at once (raw data) sensor 1
  lis2.read();    // get X Y and Z data at once (raw data) sensor 2
  logfile.print(lis.x);
  logfile.print("\t");
  logfile.print(lis.y);
  logfile.print("\t");
  logfile.print(lis.z);
  logfile.print("\t");
  logfile.print(lis2.x);
  logfile.print("\t");
  logfile.print(lis2.y);
  logfile.print("\t");
  logfile.print(lis2.z);
  logfile.println(); 

  //Serial Print for debug
  Serial.print(lis.x);
  Serial.print("\t");
  Serial.print(lis.y);
  Serial.print("\t");
  Serial.print(lis.z);
  Serial.print("\t");
  Serial.print(lis2.x);
  Serial.print("\t");
  Serial.print(lis2.y);
  Serial.print("\t");
  Serial.print(lis2.z);
  Serial.println();

  //Adafruit sensor library to get normalized data
  //Display the results (acceleration is measured in m/s^2) 
  // log from Sensor 1 and 2 
  /*sensors_event_t event; 
  lis.getEvent(&event);
  sensors_event_t event2;
  lis2.getEvent(&event2); 
  logfile.print(event.acceleration.x,3);
  logfile.print("\t");
  logfile.print(event.acceleration.y,3);
  logfile.print("\t");
  logfile.print(event.acceleration.z,3);
  logfile.print("\t");
  logfile.print(event2.acceleration.x,3);
  logfile.print("\t");
  logfile.print(event2.acceleration.y,3);
  logfile.print("\t");
  logfile.print(event2.acceleration.z,3);
  logfile.println();*/
  count++; 
  Serial.println(count); 

  if (count == 5){
    logfile.close();
    count = 0; 
  }
}



    
