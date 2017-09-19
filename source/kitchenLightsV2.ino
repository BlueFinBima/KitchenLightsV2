//******************************************************************************************
//  File: ST_Anything_Multiples_ESP8266WiFi.ino
//  Authors: Dan G Ogorchock & Daniel J Ogorchock (Father and Son)
//
//  Summary:  This Arduino Sketch, along with the ST_Anything library and the revised SmartThings 
//            library, demonstrates the ability of one NodeMCU ESP8266 to 
//            implement a multi input/output custom device for integration into SmartThings.
//            The ST_Anything library takes care of all of the work to schedule device updates
//            as well as all communications with the NodeMCU ESP8266's WiFi.
//
//            ST_Anything_Multiples implements the following ST Capabilities as a demo of what is possible with a single NodeMCU ESP8266
//              - 1 x Alarm device (using a simple digital output)
//              - 1 x Contact Sensor devices (used to monitor magnetic door sensors)
//              - 1 x Switch devices (used to turn on a digital output (e.g. LED, relay, etc...)
//              - 1 x Motion devices (used to detect motion)
//              - 1 x Smoke Detector devices (using simple digital input)
//              - 1 x Temperature Measurement devices (Temperature from Dallas Semi 1-Wire DS18B20 device)
//              - 1 x Relay Switch devices (used to turn on a digital output for a set number of cycles And On/Off times (e.g.relay, etc...))
//              - 2 x Button devices (sends "pushed" if held for less than 1 second, else sends "held"
//              - 1 x Water Sensor devices (using the 1 analog input pin to measure voltage from a water detector board)
//    
//  Change History:
//
//    Date        Who            What
//    ----        ---            ----
//    2015-01-03  Dan & Daniel   Original Creation
//    2017-02-12  Dan Ogorchock  Revised to use the new SMartThings v2.0 library
//    2017-04-17  Dan Ogorchock  New example showing use of Multiple device of same ST Capability
//                               used with new Parent/Child Device Handlers (i.e. Composite DH)
//    2017-05-25  Dan Ogorchock  Revised example sketch, taking into account limitations of NodeMCU GPIO pins
//
//******************************************************************************************
//******************************************************************************************
// SmartThings Library for ESP8266WiFi
//******************************************************************************************
#include <ESPDMX.h>
DMXESPSerial dmx;  // instantiate the DMX sender on TXD1 / GPIO2 / D4 

#define MAXDMXCHANNELS 21        // this is to help performance
#define MAXDMXCIRCUITS 19        // this is the number of active circuits in the lighting plan plus 4 for the leds in the switches which are not DMX
#define MAXDMXTHEMES 14          // this is the dimension of the lighting array - This will probably be retired to allow the SmartThings to control this directly
#define c_UNC 256                // this is the "unchanged" value for a target lighting theme ie the channel is excluded from the lighting operation
#define c__ON 255                // this is the fully ON setting for the lighting operation
#define c_OFF 0                  // this is the OFF setting for the lighting opperation
								   // these are channel numbers for the 240v circuits
								   // 
								   // DownLightAChannel are the dimmable GU10s around the worktops
								   // DownLightBChannel are the dimmable GU10s in the centre of the room
								   // DMX 12/4 must be set with a minimum of F11 downlight's are now dimmable LEDs

								   //From the NJD DPX12/4 Manual
								   //
								   //F0 All channels switching
								   //F1 Channel 1 dimming, Channels 2-12 switching
								   //F2 Channels 1-2 dimming, Channels 3-12 switching
								   //F3 Channels 1-3 dimming, Channels 4-12 switching
								   //F4 Channels 1-4 dimming, Channels 5-12 switching
								   //F5 Channels 1-5 dimming, Channels 6-12 switching
								   //F6 Channels 1-6 dimming, Channels 7-12 switching
								   //F7 Channels 1-7 dimming, Channels 8-12 switching
								   //F8 Channels 1-8 dimming, Channels 9-12 switching
								   //F9 Channels 1-9 dimming, Channels 10-12 switching
								   //F10 Channels 1-10 dimming, Channels 11-12 switching
								   //F11 Channels 1-11 dimming, Channel 12 switching
								   //F12 All channels dimming.
								   //When set to switching, the channel is either fully on or fully
								   //off, with no intermediate settings and no preheat.
								   //
								   //
#define dpx124channel_1   0   // this seems to be broken
#define dpx124channel_2   1   // unused   
#define WallLightChannel  2 
#define PendLightAChannel 3
#define PendLightBChannel 4
#define SpareLightChannel 5
#define dpx124channel_7   6   // unused
#define dpx124channel_8   7   // unused
#define dpx124channel_9   8   // unused
#define DownLightBChannel 9   // Round Hob - Dimmable LED GU10s These can be changed from dimmable (F10) to switched (F9)
#define DownLightAChannel 10  // Need to be switched so F10 (maximum) must be set to achieve this 
#define dpx124channel_12  11  // unused


				   // these are the channel numbers for the 12v LED dimmers
#define LEDChannelR 13  //this is used for the under unit  white lighting
#define LEDChannelG 14
#define LEDChannelB 15 

				   // 17 blue 18 Green 16 Red
#define AccentLEDChannelR 16   
#define AccentLEDChannelG 17
#define AccentLEDChannelB 18

				   // 19 - 21 are the three relays used for 240V LED GU10 floods for the ceiling
#define FloodRelay0 19   
#define FloodRelay1 20   // this is the main area lighting
#define FloodRelay2 21   // this is the one which can be used for around the kitchen units

// these are the LEDs in the switch units - these need to be analogue lines to get PWM
// ?? need to work out if it is possible to do PWM (and if so) if there are enough to control
// ?? the LEDs in the switch units.
// one possibility to save pins is to stick these on an i2c expander but obviously this will not have PWM

#define  SwithchLEDPinR1 5   // This is on a PWM pin
#define  SwithchLEDPinR2 9   // This is on a PWM pin
#define  SwithchLEDPinG1 3   // This is on a PWM pin
#define  SwithchLEDPinG2 6   // This is on a PWM pin
#define  SwithchLEDPinR3 16  // This is NOT on a PWM pin
#define  SwithchLEDPinR4 15  // This is NOT on a PWM pin
#define  SwithchLEDPinG3 10  // This is on a PWM pin
#define  SwithchLEDPinG4 14  // This is NOT on a PWM pin

// these are the switches by the kitchen door
// these are on a resistor ladder so need to be on an analogue input pin which the ESP8266 does not do very well
//
#define  SwALeftTop  0
#define  SwALeftMid  3
#define  SwALeftBot  1
#define  SwARightTop 4
#define  SwARightMid 6
#define  SwARightBot 7

// these are the switches by the back door
// these are on a resistor ladder so need to be on an analogue input pin which the ESP8266 does not do very well
//
#define SwBLeftTop   3
#define SwBLeftMid   1
#define SwBLeftBot   0
#define SwBRightTop  4
#define SwBRightMid  7
#define SwBRightBot  6

				   // these are the generic switch value
#define  SwLeftTop   1
#define  SwLeftMid   2
#define  SwLeftBot   3
#define  SwRightTop  4
#define  SwRightMid  5
#define  SwRightBot  6
#define  SwNone      0

// this is the lighting object
// 
// The columns are as follows
// 
// | Channel # | Current Value | Target Value | Step Value | Min Value |  Theme 0 | Theme 1 | Theme 2 | Theme 3 | Theme 4 |Theme 5 | Theme 6 | Theme 7 | Theme 8 |
// 
// The Min value is set so that things like compact tubes only go down so far before turning off.
// 
// Channel numbers > 512 are pins on the Arduino
// 

// This array contains the theme target values and current state for each of the DMX devices
int lighting[MAXDMXCIRCUITS][MAXDMXTHEMES] = {
	{ DownLightAChannel,          0,    0,    1,    0,c_OFF,   255  ,  255,c_OFF,   c_OFF,c_OFF,   c_OFF,c_OFF,c_OFF },
	{ DownLightBChannel,          0,    0,    1,    0,c_OFF,   255  ,  255,c_OFF,   c_OFF,c_OFF,   c_OFF,c_OFF,c_OFF },
	{ WallLightChannel,           0,    0,    1,    0,c_OFF,   c_OFF,c_UNC,c_OFF,     128,c_UNC,   c_OFF,c_OFF,c_OFF },
	{ PendLightBChannel,          0,    0,    1,    0,c_OFF,   c_OFF,c_UNC,c_OFF,     192,c_UNC,   c_OFF,c_OFF,c_OFF },
	{ LEDChannelR,                0,    0,    1,   24,c_OFF,   c__ON,c__ON,  255,   c_OFF,c_OFF,   c_OFF,c_OFF,c_OFF },
	{ AccentLEDChannelR,          0,    0,    1,   16,c_OFF,   c_OFF,c_UNC,  128,     128,c_UNC,   c_OFF,c_OFF,c_OFF },
	{ AccentLEDChannelG,          0,    0,    1,   16,c_OFF,   c_OFF,c_UNC,   16,      16,c_UNC,   c_OFF,c_OFF,c_OFF },
	{ AccentLEDChannelB,          0,    0,    1,   16,c_OFF,   c_OFF,c_UNC,   16,      16,c_UNC,   c_OFF,c_OFF,c_OFF },
	{ SwithchLEDPinR1 + 512,      0,    0,    1,    0,c_OFF,   c_OFF,c__ON,c_OFF,   c__ON,c_OFF,   c_OFF,c_OFF,c_OFF },
	{ SwithchLEDPinR2 + 512,      0,    0,    1,    0,c_OFF,   c_OFF,c_OFF,c_OFF,   c_OFF,c__ON,   c_OFF,c_OFF,c_OFF },
	{ SwithchLEDPinG1 + 512,      0,    0,    1,    0,   32,   c_OFF,c_OFF,c__ON,   c__ON,c_OFF,   c_OFF,c_OFF,c_OFF },
	{ SwithchLEDPinG2 + 512,      0,    0,    1,    0,c_OFF,   c_OFF,c_OFF,c_OFF,   c_OFF,c_OFF,   c_OFF,c_OFF,c_OFF },
	{ SwithchLEDPinR3 + 512,      0,    0,    1,    0,c_OFF,   c_OFF,c__ON,c_OFF,   c__ON,c_OFF,   c_OFF,c_OFF,c_OFF },
	{ SwithchLEDPinR4 + 512,      0,    0,    1,    0,c_OFF,   c_OFF,c_OFF,c_OFF,   c_OFF,c__ON,   c_OFF,c_OFF,c_OFF },
	{ SwithchLEDPinG3 + 512,      0,    0,    1,    0,   32,   c_OFF,c_OFF,c__ON,   c__ON,c_OFF,   c_OFF,c_OFF,c_OFF },
	{ SwithchLEDPinG4 + 512,      0,    0,    1,    0,c_OFF,   c_OFF,c_OFF,c_OFF,   c_OFF,c_OFF,   c_OFF,c_OFF,c_OFF },
	{ FloodRelay0,                0,    0,    0,    0,c_OFF,   c_OFF,c_OFF,c_OFF,   c_OFF,c_OFF,   c_OFF,c_OFF,c_OFF },
	{ FloodRelay1,                0,    0,    0,    0,c_OFF,   c_OFF,c_OFF,c_OFF,   c_OFF,c_OFF,   c_OFF,c_OFF,c_OFF },
	{ FloodRelay2,                0,    0,    0,    0,c_OFF,   c_OFF,c_OFF,c_OFF,   c_OFF,c_OFF,   c_OFF,c_OFF,c_OFF }
};

#include <SPI.h>

#include <SmartThingsESP8266WiFi.h>

				   //******************************************************************************************
				   // ST_Anything Library 
				   //******************************************************************************************
#include <Constants.h>       //Constants.h is designed to be modified by the end user to adjust behavior of the ST_Anything library
#include <Device.h>          //Generic Device Class, inherited by Sensor and Executor classes
#include <Sensor.h>          //Generic Sensor Class, typically provides data to ST Cloud (e.g. Temperature, Motion, etc...)
#include <Executor.h>        //Generic Executor Class, typically receives data from ST Cloud (e.g. Switch)
#include <InterruptSensor.h> //Generic Interrupt "Sensor" Class, waits for change of state on digital input 
#include <PollingSensor.h>   //Generic Polling "Sensor" Class, polls Arduino pins periodically
#include <Everything.h>      //Master Brain of ST_Anything library that ties everything together and performs ST Shield communications

#include <PS_Illuminance.h>  //Implements a Polling Sensor (PS) to measure light levels via a photo resistor

#include <PS_TemperatureHumidity.h>  //Implements a Polling Sensor (PS) to measure Temperature and Humidity via DHT library
#include <PS_DS18B20_Temperature.h>  //Implements a Polling Sesnor (PS) to measure Temperature via DS18B20 libraries 
#include <PS_Water.h>        //Implements a Polling Sensor (PS) to measure presence of water (i.e. leak detector)
#include <IS_Motion.h>       //Implements an Interrupt Sensor (IS) to detect motion via a PIR sensor
#include <IS_Contact.h>      //Implements an Interrupt Sensor (IS) to monitor the status of a digital input pin
#include <IS_Smoke.h>        //Implements an Interrupt Sensor (IS) to monitor the status of a digital input pin
#include <IS_DoorControl.h>  //Implements an Interrupt Sensor (IS) and Executor to monitor the status of a digital input pin and control a digital output pin
#include <IS_Button.h>       //Implements an Interrupt Sensor (IS) to monitor the status of a digital input pin for button presses
#include <EX_Switch.h>       //Implements an Executor (EX) via a digital output to a relay
#include <EX_Alarm.h>        //Implements Executor (EX)as an Alarm Siren capability via a digital output to a relay
#include <S_TimedRelay.h>    //Implements a Sensor to control a digital output pin with timing capabilities

				   //*************************************************************************************************
				   //NodeMCU v1.0 ESP8266-12e Pin Definitions (makes it much easier as these match the board markings)
				   //*************************************************************************************************
				   //#define LED_BUILTIN 16
				   //#define BUILTIN_LED 16
				   //
				   //#define D0 16  //no internal pullup resistor
				   //#define D1  5
				   //#define D2  4
				   //#define D3  0  //must not be pulled low during power on/reset, toggles value during boot
				   //#define D4  2  //must not be pulled low during power on/reset, toggles value during boot
				   //#define D5 14
				   //#define D6 12
				   //#define D7 13
				   //#define D8 15  //must not be pulled high during power on/reset

				   //******************************************************************************************
				   //Define which Arduino Pins will be used for each device
				   //******************************************************************************************
#define PIN_WATER_1               A0  //NodeMCU ESP8266 only has one Analog Input Pin 'A0'

#define PIN_ALARM_1               D0  //SmartThings Capabilty "Alarm"
#define PIN_SWITCH_1              D1  //SmartThings Capability "Switch"
#define PIN_CONTACT_1             D2  //SmartThings Capabilty "Contact Sensor"
#define PIN_BUTTON_1              D3  //SmartThings Capabilty Button / Holdable Button (Normally Open!)
#define PIN_BUTTON_2              D4  //SmartThings Capabilty Button / Holdable Button (Normally Open!)
#define PIN_MOTION_1              D5  //SmartThings Capabilty "Motion Sensor" (HC-SR501 PIR Sensor)
#define PIN_SMOKE_1               D6  //SmartThings Capabilty "Smoke Detector"
#define PIN_TEMPERATURE_1         D7  //SmartThings Capabilty "Temperature Measurement" (Dallas Semiconductor DS18B20)
#define PIN_TIMEDRELAY_1          D8  //SmartThings Capability "Relay Switch"

				   //******************************************************************************************
				   //ESP8266 WiFi Information
				   //******************************************************************************************
#include "KitchenLightsV2private.h"    // this holds my personal wifi info so if this appears in your source then just delete it
#ifndef _KitchenLightsV2private_h_
#define _KitchenLightsV2private_h_
String str_ssid = "Your SSID";								//  <---You must edit this line!
String str_password = "Your WiFi Password";					//  <---You must edit this line!
IPAddress ip(192, 168, 1, 32);          //Device IP Address //  <---You must edit this line!
IPAddress gateway(192, 168, 1, 254);    //Router gateway	//  <---You must edit this line!
IPAddress subnet(255, 255, 255, 0);     //LAN subnet mask   //  <---You must edit this line!
IPAddress dnsserver(192, 168, 1, 254);  //DNS server        //  <---You must edit this line!
const unsigned int serverPort = 8090; // port to run the http server on
									  // Smartthings Hub Information
IPAddress hubIp(192, 168, 1, 186);    // smartthings hub ip     //  <---You must edit this line!
const unsigned int hubPort = 39500;   // smartthings hub port
#endif
									  //******************************************************************************************
									  //st::Everything::callOnMsgSend() optional callback routine.  This is a sniffer to monitor 
									  //    data being sent to ST.  This allows a user to act on data changes locally within the 
									  //    Arduino sktech.
									  //******************************************************************************************
void callback(const String &msg)
{
	//  Serial.print(F("ST_Anything Callback: Sniffed data = "));
	//  Serial.println(msg);

	//TODO:  Add local logic here to take action when a device's value/state is changed

	//Masquerade as the ThingShield to send data to the Arduino, as if from the ST Cloud (uncomment and edit following line)
	//st::receiveSmartString("Put your command here!");  //use same strings that the Device Handler would send
}

//******************************************************************************************
//Arduino Setup() routine
//******************************************************************************************
void setup()
{
	// Documentation and samples are available at https://github.com/Rickgg/ESP-Dmx
	// Connect GPIO02 - TDX1 to MAX3485 or other driver chip to interface devices
	// Pin is defined in library
	dmx.init(MAXDMXCHANNELS);    // initialization for the maximum address to help with perfomance
							  //******************************************************************************************
							  //Declare each Device that is attached to the Arduino
							  //  Notes: - For each device, there is typically a corresponding "tile" defined in your 
							  //           SmartThings Device Hanlder Groovy code, except when using new COMPOSITE Device Handler
							  //         - For details on each device's constructor arguments below, please refer to the 
							  //           corresponding header (.h) and program (.cpp) files.
							  //         - The name assigned to each device (1st argument below) must match the Groovy
							  //           Device Handler names.  (Note: "temphumid" below is the exception to this rule
							  //           as the DHT sensors produce both "temperature" and "humidity".  Data from that
							  //           particular sensor is sent to the ST Hub in two separate updates, one for 
							  //           "temperature" and one for "humidity")
							  //         - The new Composite Device Handler is comprised of a Parent DH and various Child
							  //           DH's.  The names used below MUST not be changed for the Automatic Creation of
							  //           child devices to work properly.  Simply increment the number by +1 for each duplicate
							  //           device (e.g. contact1, contact2, contact3, etc...)  You can rename the Child Devices
							  //           to match your specific use case in the ST Phone Application.
							  //******************************************************************************************
							  //Polling Sensors
	static st::PS_Water               sensor1(F("water1"), 60, 20, PIN_WATER_1, 200);
	static st::PS_DS18B20_Temperature sensor2(F("temperature1"), 15, 0, PIN_TEMPERATURE_1, false, 10, 1);

	//Interrupt Sensors 
	static st::IS_Contact             sensor3(F("contact1"), PIN_CONTACT_1, LOW, true);
	static st::IS_Button              sensor4(F("button1"), PIN_BUTTON_1, 1000, LOW, true, 500);
	static st::IS_Button              sensor5(F("button2"), PIN_BUTTON_2, 1000, LOW, true, 500);
	static st::IS_Motion              sensor6(F("motion1"), PIN_MOTION_1, HIGH, false);
	static st::IS_Smoke               sensor7(F("smoke1"), PIN_SMOKE_1, HIGH, true, 500);

	//Special sensors/executors (uses portions of both polling and executor classes)
	static st::S_TimedRelay           sensor8(F("relaySwitch1"), PIN_TIMEDRELAY_1, LOW, false, 3000, 0, 1);

	//Executors
	static st::EX_Alarm executor1(F("alarm1"), PIN_ALARM_1, LOW, true);
	static st::EX_Switch executor2(F("switch1"), PIN_SWITCH_1, LOW, true);  //Inverted logic for "Active Low" Relay Board

																			//*****************************************************************************
																			//  Configure debug print output from each main class 
																			//  -Note: Set these to "false" if using Hardware Serial on pins 0 & 1
																			//         to prevent communication conflicts with the ST Shield communications
																			//*****************************************************************************
	st::Everything::debug = true;
	st::Executor::debug = true;
	st::Device::debug = true;
	st::PollingSensor::debug = true;
	st::InterruptSensor::debug = true;

	//*****************************************************************************
	//Initialize the "Everything" Class
	//*****************************************************************************

	//Initialize the optional local callback routine (safe to comment out if not desired)
	st::Everything::callOnMsgSend = callback;

	//Create the SmartThings ESP8266WiFi Communications Object
	//STATIC IP Assignment - Recommended
	st::Everything::SmartThing = new st::SmartThingsESP8266WiFi(str_ssid, str_password, ip, gateway, subnet, dnsserver, serverPort, hubIp, hubPort, st::receiveSmartString);

	//DHCP IP Assigment - Must set your router's DHCP server to provice a static IP address for this device's MAC address
	//st::Everything::SmartThing = new st::SmartThingsESP8266WiFi(str_ssid, str_password, serverPort, hubIp, hubPort, st::receiveSmartString);

	//Run the Everything class' init() routine which establishes WiFi communications with SmartThings Hub
	st::Everything::init();

	//*****************************************************************************
	//Add each sensor to the "Everything" Class
	//*****************************************************************************
	st::Everything::addSensor(&sensor1);
	st::Everything::addSensor(&sensor2);
	st::Everything::addSensor(&sensor3);
	st::Everything::addSensor(&sensor4);
	st::Everything::addSensor(&sensor5);
	st::Everything::addSensor(&sensor6);
	st::Everything::addSensor(&sensor7);
	st::Everything::addSensor(&sensor8);

	//*****************************************************************************
	//Add each executor to the "Everything" Class
	//*****************************************************************************
	st::Everything::addExecutor(&executor1);
	st::Everything::addExecutor(&executor2);

	//*****************************************************************************
	//Initialize each of the devices which were added to the Everything Class
	//*****************************************************************************
	st::Everything::initDevices();

}

//******************************************************************************************
//Arduino Loop() routine
//******************************************************************************************
void loop()
{
	dmx.write(2, 0);        // channal 3 off
	dmx.write(1, 255);      // channal 1 on
	dmx.update();           // update the DMX bus
	delay(1000);            // wait for 1s
	dmx.write(3, 0);        // channal 3 off
	dmx.write(2, 255);      // channal 1 on
	dmx.update();           // update the DMX bus
	delay(1000);            // wait for 1s
	dmx.write(1, 0);        // channal 3 off
	dmx.write(3, 255);      // channal 1 on
	dmx.update();           // update the DMX bus
	delay(1000);            // wait for 1s

							//*****************************************************************************
							//Execute the Everything run method which takes care of "Everything"
							//*****************************************************************************
	st::Everything::run();
}
