# Data-logger Version 2
This is an ESP32 based data logger utilizing up to two LIS3DH accelerometers.  This program uses FreeRTOS to run multiply tasks at the same time to read and write data to SD Card.
Card write uses One bit Mode (half duplexing) SDMMC for fast write speeds up to 371 kB/s measured.  This logger can record and write up to 1000 Hz from the 2x LIS3DHs using 
6 channels (x,y,z for both) + time in microSeconds.  Hardware timer is used to provide interrupt service to run TaskGetData for precise timing. 
A button is used for external interrupt to start datalogging, currently "set time" is used to run loop until called to stop. 
A 3.7 to 5 V battery can be used but most LiPO are 3.7v for microcontrollers.  

User inputs:  
Sample time = time you want to collect data in Seconds
Sampe Rate = in Hertz (samples/second) up to 1000 Hz (1000 hz is MAX. CAP if using all 6 channels (28 kB per data structure), if you use 1x accel or minimize data structure likely 
can higher but needs more testing and validation)
These parameters must be written inside the .ino Arduino file.  Once these items are set the board (ESP32 Microcontroller) will need to be flashed with new program.  The program will always use these 
updated parameters unless they are changed and the board is flashed again.    

Start up:
- Plug battery in
- An LED is used to notify user if logger is ready for use.  During start up (plugged in/powered on) the logger runs a setup code to check that it is connected to "Sensor 1" then "Sensor 2", then 
that an SD card is available and that it can write to it.  If these operations are successful then the LED blinks 7 times and turns off.  When data is being recorded the LED will be on (high)
then turn off once SD write is off and file closed.  An SD card can be removed once this operation is over with logger still powered.  
- If you did not get LED flashes, check accelerometer connections and make sure SD card is inserted.  Make sure SD card is properly formated (SDFat, 32kB partitions, use official SD formatter for stability)
    If you still don't get LED flash, you'll need to plug into computer and run Arduino Monitor/Serial for more detailed troubleshooting.
- Once properly setup and armed, start recording with one push of the button.  The program will only notice when it has been pressed once, and ignore after that.  It will run for its set time (sample time) then the LED
will turn off and SD card is ready for removal.  
- Current file format is .TXT file with comma delimiter (use this in excel when importing if using excel).  File format .CSV can be used but its not as stable at high speed recording, so caution against this above 500 Hz. 

- Wiring - Connectors are simple JST type.  Recommend taping connections for reliability.  Wires are color coded except at accelerometers which hoping to upgrade that eventually to make it easier for plug in.
- Sensor 1 = Upper sensor, typically at handlebar, seatpost, etc.
- Sensor 2 = Lower sensor, typically at axles, droput mounts, etc.
- If only one sensor is used, typically use Sensor 1

LIS3DH wiring
- Wire colors 
RED - 3v, 
BLACK - GND, 
White - CS line, 
GREEN - SCK/CLK, 
BLUE - MISO/SDA, 
YELLOW - MOSI/SDO 
(currently on Accels Vout and Interrupt are not used)

SDMMC wiring
1 -> 4
0 -> 2
GND
CLK -> 14
VDD 
GND
CMD -> 15
3 -> 13
4 ->12

LED = PIN 32

Button = PIN 34 (PUlled HIGH)

Information about program itself:
- Program is written in Arduino/C++ and uses Arduino, C++, and ESP32 API libraries
- SPI:  this program leverages the use of ESP32 SPIs, HSPI -> SPI2, VSPI -> SPI3, 
- program uses FreeRTOS, a light weight operating system that allows multi-tasking/threading, this allows getting data and writing essentially to happen simultaneously 
- FreeRTOS tasks:  vTask "TaskGetData", "TaskSDWrite", "TaskLED"
- Uses both ESP32 cores for computing
- Hardware timer used for accurate timing, avoids locking program with "delays".  Uses ISR gives semaphore for data recording
- Data is record via Data_t, then each task uses two Data_t objects for Transfer and Recieve
- A data queue is created inbetween Get and SDwrite of 10 items to buffer data and avoid any loss
- Button uses external interrupt task.  Gives semaphore to GetData to start
- File finishes with close and delay to make sure SD has time to write and close
- Currently no flush is used, so if logger losses power or connections during testing, likely all data is lost






