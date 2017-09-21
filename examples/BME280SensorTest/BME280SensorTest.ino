/*
This example code for Flutter is
Copyright 2015, Taylor Alexander and Flutter Wireless, Inc.

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
#include "Flutter.h"

//
// TinyPacks is data serialization format for constrained environments 
// like 8-bit and 16-bit microcontrollers. It can be downloaded to your
// Arduino Libraries folder from https://github.com/francc/tinypacks
//
#include "TinyPacks.h"

boolean _running = false;
Flutter flutter;
unsigned long delayTime;

byte mydata = 0;

float Temp = 0;
float Pressure = 0;
float Altitude = 0;
float Humidity = 0;

// To test this program, flash one board as a transmitter and then 
// comment out this line (with //) to flash another board as a receiver
#define TRANSMITTER


#ifdef TRANSMITTER

  //Setup Adafruit BME280 Pressure Sensor on I2C
  #include <Adafruit_Sensor.h>
  #include <Adafruit_BME280.h>
  #include <Wire.h>
  #include <SPI.h>
  
  #define SEALEVELPRESSURE_HPA (1013.25)
  Adafruit_BME280 bme; // I2C

  //initiate TinyPacks as a writer inorder to transmit data
  PackWriter writer;
#else
  //initiate TinyPacks as a writer to unpack data
  PackReader reader;
#endif

//TinyPacks Constants
#define MAX_PACKED_DATA 256
unsigned char packed_data[MAX_PACKED_DATA];
int packed_data_length;

void setup()
{
  //Starts Serial on pins D2 and D3
  Serial.begin(115200);
  
  /********************************************************************/
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
    
    // default settings for the BME280
    status1 = bme.begin();
    if (!status1) {
        SerialUSB.println("Could not find a valid BME280 sensor, check wiring!");
        while (1);
    }
  #endif
  
  delayTime = 1000;
  SerialUSB.println();
  delay(100); // let sensor boot up
}


void loop()
{
  // Checks status of the flutter radio
  status();

  #ifdef TRANSMITTER
	mydata++;
    //Get BME280 Sensor Data
    Temp     = bme.readTemperature();
    Pressure = bme.readPressure() / 100.0F;
    Altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
    Humidity = bme.readHumidity();
    
    //Pack Sensor Data
    writer.setBuffer(packed_data, MAX_PACKED_DATA);
    writer.openMap();
    writer.putString("Temp");
    writer.putReal( Temp);
    writer.putString("Pressure");
    writer.putReal( Pressure);
    writer.putString("Altitude");
    writer.putReal( Altitude);
    writer.putString("Humidity");
    writer.putReal( Humidity);
    writer.close();
    packed_data_length = writer.getOffset();

    // Prints the packed data from TinyPacks as a debugging aid
    SerialUSB.println(packed_data_length);
    SerialUSB.println("PACKED DATA");
    for(int i = 0; i < packed_data_length; i++)
        SerialUSB.print(packed_data[i]);
    SerialUSB.println();    

 
	// send a byte array over the radio
	flutter.sendData(packed_data, packed_data_length, 2); //legth is 3, 2 is car's address

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

	delay(40); //spend some time smelling the roses
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

		// Unpack
		reader.setBuffer(packed_data, packetSize-5);
		reader.next();
		if(reader.openMap()) {
		  while(reader.next()) {
			if     (reader.match("Temp"))      Temp     = reader.getReal();
			else if(reader.match("Pressure"))  Pressure = reader.getReal();
			else if(reader.match("Altitude"))  Altitude = reader.getReal();
			else if(reader.match("Humidity"))  Humidity = reader.getReal();
			else reader.next();
		  }
		  reader.close();
		}
   
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



