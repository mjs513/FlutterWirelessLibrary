/*
This example code for Flutter is
Copyright 2015, Taylor Alexander and Flutter Wireless, Inc.

Code for using BME280 adafruit libray, MPU9250 library and SDCard
using SDfat libraries implemented for Flutter developed by
MJS.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/
#include <Wire.h>
#include <SPI.h>
#include "SdFat.h"

#include <elapsedMillis.h>
elapsedMillis timeElapsed;
#define sensorUpdate 100

//Set to 0 to not print to USB
#define DEBUG_PRINT 0

// Set to 0 to not log data
//#define LOGGER



/**********************************************************
 *  Setup Flutter instance
 **********************************************************/
#include "Flutter.h"
boolean _running = false;

Flutter flutter;
unsigned long delayTime;

byte mydata = 0;

/**********************************************************
 * TinyPacks is data serialization format for constrained environments 
 * like 8-bit and 16-bit microcontrollers. It can be downloaded to your
 * Arduino Libraries folder from https://github.com/francc/tinypacks
 **********************************************************/
#include "TinyPacks.h"

/**********************************************************
 * variables for tinypacks and output from BME280
 **********************************************************/
float Temp = 0;
float Pressure = 0;
float Altitude = 0;
float Humidity = 0;
float accel[3] = {0, 0, 0};
float gyro[3] = {0, 0, 0};
float mag[3] = {0, 0, 0};
int32_t timeStamp;

/**********************************************************
 * To test this program, flash one board as a transmitter and then 
 * comment out this line (with //) to flash another board as a receiver
 **********************************************************/
#define TRANSMITTER

#ifdef TRANSMITTER
  /**********************************************************
   * Setup Adafruit BME280 Pressure Sensor on I2C
   * Only needed for the transmitter side
   **********************************************************/
  #include <Adafruit_Sensor.h>
  #include <Adafruit_BME280.h>
  
  #define SEALEVELPRESSURE_HPA (1013.25)

  #define BME_SCK  10  //d10
  #define BME_MISO  8  //d9
  #define BME_MOSI  9  //d8
  #define BME_CS    7  //d7, 
  Adafruit_BME280 bme; // I2C
  //Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK);
  //Adafruit_BME280 bme(BME_CS);

  /**********************************************************
   * DigitalIO provides Fast Digital I/O, Software I2C, and Software SPI 
   * developed by Bill Greiman and can be downloaded from
   * https://github.com/greiman/DigitalIO.  Version used here is modified
   * to include the flutter Wireless board.  It provided in the Flutter
   * Wireless core-libs Library folder.
   *
   * Pin numbers do not correspond to whats on the pin map pdf
   * what is used are the actual pin numbers
   ***********************************************************/
  #include "DigitalIO.h"
  const uint8_t SOFT_SPI_MISO_PIN = 8;
  const uint8_t SOFT_SPI_MOSI_PIN = 9;
  const uint8_t SOFT_SPI_SCK_PIN  = 10;
  const uint8_t SPI_MODE = 0;
  SoftSPI<SOFT_SPI_MISO_PIN, SOFT_SPI_MOSI_PIN, SOFT_SPI_SCK_PIN, SPI_MODE> spi;
  
  /**********************************************************
   * MPU9250 driver library based on MBED implementation
   * uses Software SPI
   **********************************************************/
  #include <mpu9250_spi.h>
  mpu9250_spi imu(BME_CS);
  
  //#define PACKET_SIZE  18
  //uint8_t data[PACKET_SIZE]; // data array to hold accelerometer and gyro x, y, z, data
  //uint16_t ii, packet_count, fifo_count;
    
  /**********************************************************
   * initiate TinyPacks as a writer inorder to transmit data
   **********************************************************/
  PackWriter writer;

  #ifdef LOGGER
    /**********************************************************
     * Chip select may be constant or RAM variable for software
     * SPI for the SD Card - Datalogging
     **********************************************************/
    const uint8_t SD_SOFT_SPI_MISO_PIN = 24;
    const uint8_t SD_SOFT_SPI_MOSI_PIN = 25;
    const uint8_t SD_SOFT_SPI_SCK_PIN  = 26;
    const uint8_t SD_CHIP_SELECT_PIN = 6;
    // SdFat software SPI template
    SdFatSoftSpi<SD_SOFT_SPI_MISO_PIN, SD_SOFT_SPI_MOSI_PIN, SD_SOFT_SPI_SCK_PIN> sd;
    
    // Setup instance of SdFile
    SdFile file;
    
    // Log file base name.  Must be six characters or less.
    #define FILE_BASE_NAME "Data"
  
    // Interval between data records in milliseconds.
    // The interval must be greater than the maximum SD write latency plus the
    // time to acquire and write data to the SD to avoid overrun errors.
    // Run the bench example to check the quality of your SD card.
    const uint32_t SAMPLE_INTERVAL_MS = sensorUpdate;
  #endif
  
  /**********************************************************
   * simple StopWatch class to measure elapsed time for
   * logger
   **********************************************************/
  #include <StopWatch.h>
  StopWatch logTime;

  
  /**********************************************************
   *  Finish Transmitter
   **********************************************************/
   
#else
  /**********************************************************/
  //initiate TinyPacks as a writer to unpack data
  PackReader reader;
  
  /**********************************************************/
  /***********************  FINISH RECEIVER *****************/
  /**********************************************************/
#endif

//TinyPacks Constants, need for both pack and unpack
#define MAX_PACKED_DATA 256
unsigned char packed_data[MAX_PACKED_DATA];
int packed_data_length;

/**********************************************************/

void setup()
{
  //Starts Serial on pins D2 and D3
  Serial.begin(115200);
  spi.begin();
  
  /********************************************************************
    * Using SerialUSB has it quirks so becareful in using it
    * 1. It appears that the radio is somehow linked to the operation
    *    of the USB so you can not use it unless you actually have it 
    *    operational either as a transmitter or as a receiver.
    * 2. When you first start the radio as in the initRadio function
    *    you can not use SerialUSB, if you try it will hang the sketch
    *    at the initializing the radio.
    * 3. After loading a sketch and you restart the Flutter (may restart
    *    automatically but more often than not you have to unplug it 
    *    and plug it back in) it will appear as a Arduino Due programmers
    *    port.  Not to worry it still works.
    * 4. If you open the serialUSB port while the sketch is running and 
    *    then close it, the sketch will stop, period and you have to
    *    disconnect from the computer and then reconnect it.  This goes
    *    for both the transmit and receive sketches
    * 5. Bi-directional communction is not implemented.
    *
   *********************************************************************/
  SerialUSB.begin(115200);   //Starts Serial output on USB Connector
  
  // I put a delay of 5 seconds here only to give me time to open SerialUSB
  // in case i want to see the data while debugging
  delay(5000);
  
  // Initialized the radio and network
  initRadio();

  // Since the sketch works for both transmit and recieve this starts the 
  // transmitter portion of the sketch
  #ifdef TRANSMITTER
  
    // Initializing the BME280
    SerialUSB.println(F("BME280 Initializing..."));
    bool status1;

    pinMode(BME_CS, OUTPUT);  //for spi only
    // default settings for the BME280
    status1 = bme.begin();
    if (!status1) {
        SerialUSB.println("Could not find a valid BME280 sensor, check wiring!");
        while (1);
    }
  
    delayTime = 1000;
    SerialUSB.println();
  
    if(imu.init(1,BITS_DLPF_CFG_188HZ))
    {  
      SerialUSB.println(F("Couldn't initialize MPU9250 via SPI!"));
    }

    SerialUSB.print(F("WHOAMI = ")); SerialUSB.println(imu.whoami());  //output the I2C address to know if SPI is working, it should be 104
    delay(1);    
    SerialUSB.print("Gyro_scale = "); SerialUSB.println(imu.set_gyro_scale(BITS_FS_2000DPS));  //Set full scale range for gyros
    delay(1);  
    SerialUSB.print(F("Acc_scale = ")); SerialUSB.println(imu.set_acc_scale(BITS_FS_16G));      //Set full scale range for accs
    delay(100);
    SerialUSB.print(F("AK8963 WHIAM = ")); SerialUSB.println(imu.AK8963_whoami());
    delay(1);  
    imu.AK8963_calib_Magnetometer();
  
    delay(100);

    #ifdef LOGGER
      /********************************************************************
       * Initialize SD Card and set up file name
  	   ********************************************************************/
      const uint8_t BASE_NAME_SIZE = sizeof(FILE_BASE_NAME) - 1;
      char fileName[13] = FILE_BASE_NAME "00.csv";
  
      // Initialize sd.
      if (!sd.begin(SD_CHIP_SELECT_PIN)) {
        sd.initErrorHalt();
      }
  
      // Find an unused file name.
      if (BASE_NAME_SIZE > 6) {
        SerialUSB.println("FILE_BASE_NAME too long");
      }
      
      while (sd.exists(fileName)) {
        if (fileName[BASE_NAME_SIZE + 1] != '9') {
          fileName[BASE_NAME_SIZE + 1]++;
        } else if (fileName[BASE_NAME_SIZE] != '9') {
          fileName[BASE_NAME_SIZE + 1] = '0';
          fileName[BASE_NAME_SIZE]++;
        } else {
          SerialUSB.println("Can't create file name");
        }
      }
      if (!file.open(fileName, O_CREAT | O_WRITE | O_EXCL)) {
        SerialUSB.println("file.open");
      }
      
      SerialUSB.print(F("Logging to: "));
      SerialUSB.println(fileName);
    
      SerialUSB.println(F("Done."));
  
      /***********************************************************
       * Write Header to file
       */
       writeHeader();
    #endif
    
  #endif
}


void loop()
{
  // Checks status of the flutter radio
  status();

  // Time for next record.
  logTime.start();
  
  #ifdef TRANSMITTER    
    if (timeElapsed > sensorUpdate) {
  	  mydata++;
	
    //Get BME280 Sensor Data
    Temp     = bme.readTemperature();
    Pressure = bme.readPressure() / 100.0F;
    Altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
    Humidity = bme.readHumidity();
	
	accel[0] = imu.accelerometer_data[0];
	accel[1] = imu.accelerometer_data[1];
	accel[2] = imu.accelerometer_data[2];
	gyro[0] = imu.gyroscope_data[0];
	gyro[1] = imu.gyroscope_data[1];
	gyro[2] = imu.gyroscope_data[2];
	mag[0] = imu.Magnetometer[0];
	mag[1] = imu.Magnetometer[1];
	mag[2] = imu.Magnetometer[2];

    
    /**********************************************************/
    imu.read_all();
	  if (DEBUG_PRINT == 1){
        SerialUSB.print(F("Gyro : "));
        SerialUSB.print(gyro[0]); SerialUSB.print(F(","));
        SerialUSB.print(gyro[1]); SerialUSB.print(F(","));
        SerialUSB.print(gyro[2]); SerialUSB.print(F(","));
        SerialUSB.print(F(" Acc : "));
        SerialUSB.print(accel[0]); SerialUSB.print(F(","));
        SerialUSB.print(accel[1]); SerialUSB.print(F(","));
        SerialUSB.print(accel[2]); SerialUSB.print(F(","));
        SerialUSB.print(F(" Mag : "));
        SerialUSB.print(mag[0]); SerialUSB.print(F(","));
        SerialUSB.print(mag[1]); SerialUSB.print(F(","));
        SerialUSB.println(mag[2]);
	  }

  #ifdef LOGGER
  	/**************************************************************
  	 * Write data to the SD Card
  	 **************************************************************/
      logData();
      
      // Force data to SD and update the directory entry to avoid data loss.
      if (!file.sync() || file.getWriteError()) {
        SerialUSB.println("write error");
      }
   #endif
   
	/***************************************************************
	 * packet data to transmit
	 ***************************************************************/
    packData();
 
	// send a byte array over the radio
	flutter.sendData(packed_data, packed_data_length, 2); // 2 is car's address

  	switch ((mydata - 1) % 3)
  	{
  		case 0:
  			flutter.setLED(RED);
  			break;
  
  		case 1:
  			flutter.setLED(BLUE);
  			break;
  
  		case 2:
  			flutter.setLED(GREEN);
  			break;
  
  		default:
  			break;
  	}
  
  	delay(30); //spend some time smelling the roses
    }
    //*************************************************
    // END TRANSMITTER SECTION
    //*************************************************

  #else
  	if (flutter.dataAvailable() > 0)
  	{
  		int packetSize = flutter.nextPacketLength();
  		unsigned char array[packetSize];
  		flutter.readBytes(array, packetSize);
  
  		SerialUSB.print("Packet Size: ");
  		SerialUSB.println(packetSize);
  
  		for (int i = 0; i < packetSize; i++)
  		{
  			SerialUSB.print("[");
  			SerialUSB.print(array[i]);
  			SerialUSB.print("]");
  		}
  
  		// Have to start at index 5 since 0-4 are used as a header
  		// by the radio
  		for (int i = 5; i < packetSize; i++){
  		    packed_data[i-5] = array[i];
  		}
  
          unpackData(packetSize);
  		
  		SerialUSB.println();
  		
  		mydata = array[6];
  		int rssi = flutter.packetRSSI(array, packetSize);
  		SerialUSB.print("RSSI: ");
  		SerialUSB.print(rssi);
  		SerialUSB.println(" dBm");
  
  		for (int j = 0; j > rssi; j--)
  		{
  			SerialUSB.print("=");
  		}
  
  		SerialUSB.println("");
  
  		switch (mydata % 3)
  		{
  			case 0:
  				flutter.setLED(RED);
  				break;
  
  			case 1:
  				flutter.setLED(BLUE);
  				break;
  
  			case 2:
  				flutter.setLED(GREEN);
  				break;
  
  			default:
  				break;
  		}
  		flutter.nextPacket();
  	}
  #endif
}

#ifdef LOGGER
  //------------------------------------------------------------------------------
  // Write data header.
  void writeHeader() {
    file.print(F("micros"));
    file.print(F("gx")); file.write(',');
    file.print(F("gy")); file.write(',');
    file.print(F("gz")); file.write(',');
    //SerialUSB.print(F(" Acc : "));
    file.print(F("ax")); file.write(',');
    file.print(F("ay")); file.write(',');
    file.print(F("az")); file.write(',');
    //SerialUSB.print(F(" Mag : "));
    file.print(F("mx")); file.write(',');
    file.print(F("my")); file.write(',');
    file.print(F("mz")); file.write(',');
    
    file.print(F("Temp")); file.write(',');
    file.print(F("Pres")); file.write(',');
    file.print(F("Alt"));
    file.println();
    file.println();
  }
  
  //------------------------------------------------------------------------------
  // Log a data record.
  void logData() {
  
    // Write data to file.  Start with log time in micros.
    file.print(logTime.value()); file.write(',');
  
    //SerialUSB.print(F("Gyro : "));
    file.print(imu.gyroscope_data[0],6); file.write(',');
    file.print(imu.gyroscope_data[1],6); file.write(',');
    file.print(imu.gyroscope_data[2],6); file.write(',');
    //SerialUSB.print(F(" Acc : "));
    file.print(imu.accelerometer_data[0],6); file.write(',');
    file.print(imu.accelerometer_data[1],6); file.write(',');
    file.print(imu.accelerometer_data[2],6); file.write(',');
    //SerialUSB.print(F(" Mag : "));
    file.print(imu.Magnetometer[0],6); file.write(',');
    file.print(imu.Magnetometer[1],6); file.write(',');
    file.print(imu.Magnetometer[2],6); file.write(',');
    
    file.print(Temp); file.write(',');
    file.print(Pressure);  file.write(',');
    file.print(Altitude);
    file.println();
  }
#endif

/**********************************************************
 *  TinyPacks - Pack and Unpack routines
 **********************************************************/
void packData()
{
    //Pack Sensor Data
    writer.setBuffer(packed_data, MAX_PACKED_DATA);
    writer.openMap();
    writer.putString("timeStamp");    writer.putInteger(logTime.value());
    writer.putString("Temp");       writer.putReal( Temp);
    writer.putString("Pressure");   writer.putReal( Pressure);
    writer.putString("Altitude");   writer.putReal( Altitude);
    writer.putString("Humidity");   writer.putReal( Humidity);
	
    writer.putString("ax");   writer.putReal( accel[0]);
    writer.putString("ay");   writer.putReal( accel[1]);
    writer.putString("az");   writer.putReal( accel[2]);
    writer.putString("gx");   writer.putReal( gyro[0]);
    writer.putString("gy");   writer.putReal( gyro[1]);
    writer.putString("gz");   writer.putReal( gyro[2]);
    writer.putString("mx");   writer.putReal( mag[0]);
    writer.putString("my");   writer.putReal( mag[1]);
    writer.putString("mz");   writer.putReal( mag[2]);	
	
    writer.close();
    packed_data_length = writer.getOffset();
    /**********************************************************/

    // Prints the packed data from TinyPacks as a debugging aid
    if(DEBUG_PRINT == 1){
      SerialUSB.println(packed_data_length);
      SerialUSB.println("PACKED DATA");
      for(int i = 0; i < packed_data_length; i++)
        SerialUSB.print(packed_data[i]);
      SerialUSB.println(); 
    }   

}

#ifndef TRANSMITTER
void unpackData(int packetSize)
{
    // Unpack
    reader.setBuffer(packed_data, packetSize-5);
    reader.next();
    if(reader.openMap()) {
		while(reader.next()) {
			if(reader.match("timeStamp"))        timeStamp = reader.getInteger();     
			else if(reader.match("Temp"))      Temp     = reader.getReal();
			else if(reader.match("Pressure"))  Pressure = reader.getReal();
			else if(reader.match("Altitude"))  Altitude = reader.getReal();
			else if(reader.match("Humidity"))  Humidity = reader.getReal();
			else if(reader.match("ax"))  accel[0] = reader.getReal();
			else if(reader.match("ay"))  accel[1] = reader.getReal();
			else if(reader.match("az"))  accel[2] = reader.getReal();
			else if(reader.match("gx"))  gyro[0] = reader.getReal();
			else if(reader.match("gy"))  gyro[1] = reader.getReal();
			else if(reader.match("gz"))  gyro[2] = reader.getReal();
			else if(reader.match("mx"))  mag[0] = reader.getReal();
			else if(reader.match("my"))  mag[1] = reader.getReal();
			else if(reader.match("mz"))  mag[2] = reader.getReal();
			else reader.next();
		}
		reader.close();

		// Print unpacked data
		SerialUSB.println();
		SerialUSB.println("Map contents:");
		SerialUSB.print("  Temp: ");
		SerialUSB.print(Temp);
		SerialUSB.println("  Press: ");
		SerialUSB.print(Pressure);
		SerialUSB.println("  Altitude: ");
		SerialUSB.print(Altitude);
		SerialUSB.println("  Humidity: ");
		SerialUSB.print(Humidity);
		SerialUSB.println();
		
		SerialUSB.print(F("Gyro : "));
        SerialUSB.print(gyro[0]); SerialUSB.print(F(","));
        SerialUSB.print(gyro[1]); SerialUSB.print(F(","));
        SerialUSB.print(gyro[2]); SerialUSB.print(F(","));
        SerialUSB.print(F(" Acc : "));
        SerialUSB.print(accel[0]); SerialUSB.print(F(","));
        SerialUSB.print(accel[1]); SerialUSB.print(F(","));
        SerialUSB.print(accel[2]); SerialUSB.print(F(","));
        SerialUSB.print(F(" Mag : "));
        SerialUSB.print(mag[0]); SerialUSB.print(F(","));
        SerialUSB.print(mag[1]); SerialUSB.print(F(","));
        SerialUSB.println(mag[2]);
		SerialUSB.println();
        /**********************************************************/
	}
}
#endif

/**********************************************************
 *  Radio routines
 **********************************************************/
void initRadio()
{
  flutter.band = BAND;
  flutter.setNetworkName("Range Test");
  Serial.println("Initializing...");

  if (flutter.init() == true)
  {
    Serial.println("Init success.");
    flutter.ledLightShow();
    delay(500);
    //analogWrite(LED_R, 128);
  }
  else
  {
    flutter.setLED(RED);
    Serial.println("Init failed.");

    while (true);
  }

  //flutter.setAddress(1);
  #ifdef TRANSMITTER
    flutter.setAddress(1);
  #else
    flutter.setAddress(2);
  #endif
    flutter.connect(1); //form a network with this and one other device
}
 
 void status()
{
    if(flutter.getState()!=NORMAL_OPERATION) //if we aren't synchronized with another radio, just loop and blink lights.
    {
      if(millis()%400<200)
      {
        flutter.setLED(RED);
      }
      
      else
      {
        flutter.setLED(BLUE);
      }
      
    }
}



void button1()
{
	interrupts();
	int val = digitalRead(BUTTON1); //top button

	if (val == HIGH)
	{
		// _button1=255;
	}
	else
	{
		//  _button1=0;
	}

// buttonsChanged=true;
}

void button2()
{
	interrupts();
	int val = digitalRead(BUTTON2);
#ifdef FLUTTER_R2

	if (val == HIGH)
#else
	if (val == LOW)
#endif
	{
		//_button2=255;
	}
	else
	{
		//_button2=0;
	}

// buttonsChanged=true;
}

void systemReset()
{
	flutter.setLED(0, 0, 255);
	delayMicroseconds(16000);
	delayMicroseconds(16000);
	flutter.setLED(0, 0, 0);
	delayMicroseconds(16000);
	delayMicroseconds(16000);
	flutter.setLED(0, 255, 0);
	delayMicroseconds(16000);
	delayMicroseconds(16000);
	flutter.setLED(0, 0, 0);
	delayMicroseconds(16000);
	delayMicroseconds(16000);
	flutter.setLED(255, 0, 0);
	delayMicroseconds(16000);
	delayMicroseconds(16000);
	flutter.setLED(0, 0, 0);
	initiateReset(1);
	tickReset();
}



void radioInterrupt()
{
	flutter.interrupt();
}

void softInt()
{
	flutter.processSoftInt();
}

extern boolean tickInterrupt()
{
	if (!flutter.initialized)
	{
		return true;
	}

	return flutter.tickInt();
}



