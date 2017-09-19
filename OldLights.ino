// This is a migration file to manipulate the source for the move from DMXSerial and AtMega32U4 to ESP8266 and ESP-DMX
// ESP-DMX uses TX1 on D4/GPIO2.  The NodeMCU board is using a simple 75168 (5V) to talk DMX.  The intent is to 
// have this controller work with Samsung's SmartThings controller which is the main reason for the move to this
// WiFi enabled platform.
// 
// These are the DMX devices controlled by this program:
// 1)  NJD DPX12/4 Channel Mains lighting Dimmer (This now needs to be set
//     to F9 as no GU10's are dimmable due to bulb failure rate).
// 2)  CT305R LED Dimmer
// 3)  3Ch Relay box  - This is a late addition to fix a problem with the CF GU10's which failed very
//     easily and even when the DPX12 simply acted as a switch, this caused the replacement LED GU10s to 
//     flash.  While the code for this device is in this program, it is not currently used.
// 
// notes:
// defines are used instead of constants because the Arduino does not seem to support the latter
// The  maximum number of DMX channels is defined because I think the library loops round all 
// of the active channels and therefore the max channels helps with performance
// 
// Each lamp has an array of values which describes what happens to it they are
// 1)  Current value
// 2)  Target value
// 3)  step size ie the size of steps to take from the current to the target


#include <DMXSerial.h>

#define rel 0.0.10               //The is the release number for this program
boolean debug = false;           // this controls debugging information and in particular whether the serial port messages are produced
boolean emer_sw = false;
int emer_mode = 0;
#define EMERGENCYPIN 2
int ButtonValue = 0;        // variable to hold the analog value of the combined switches
int LastButtonValue = 0;        // this is used to determine if the same button has been pressed

#define MAXDMXCHANNELS 21        // this is to help performance
#define MAXDMXCIRCUITS 19           // this is the number of active circuits in the lighting plan plus 4 for the leds in the switches which are not DMX
#define MAXDMXTHEMES 14            // this is the dimension of the lighting array
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

#define  SwithchLEDPinR1 5   // This is on a PWM pin
#define  SwithchLEDPinR2 9   // This is on a PWM pin
#define  SwithchLEDPinG1 3   // This is on a PWM pin
#define  SwithchLEDPinG2 6   // This is on a PWM pin
#define  SwithchLEDPinR3 16  // This is NOT on a PWM pin
#define  SwithchLEDPinR4 15  // This is NOT on a PWM pin
#define  SwithchLEDPinG3 10  // This is on a PWM pin
#define  SwithchLEDPinG4 14  // This is NOT on a PWM pin

								// these are the switches by the kitchen door
#define  SwALeftTop  0
#define  SwALeftMid  3
#define  SwALeftBot  1
#define  SwARightTop 4
#define  SwARightMid 6
#define  SwARightBot 7

								// these are the switches by the back door
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
int lighting[MAXCIRCUITS][MAXTHEMES] = {
	{DownLightAChannel,          0,    0,    1,    0,c_OFF,   255  ,  255,c_OFF,   c_OFF,c_OFF,   c_OFF,c_OFF,c_OFF },
	{DownLightBChannel,          0,    0,    1,    0,c_OFF,   255  ,  255,c_OFF,   c_OFF,c_OFF,   c_OFF,c_OFF,c_OFF },
	{WallLightChannel,           0,    0,    1,    0,c_OFF,   c_OFF,c_UNC,c_OFF,     128,c_UNC,   c_OFF,c_OFF,c_OFF },
	{PendLightBChannel,          0,    0,    1,    0,c_OFF,   c_OFF,c_UNC,c_OFF,     192,c_UNC,   c_OFF,c_OFF,c_OFF },
	{LEDChannelR,                0,    0,    1,   24,c_OFF,   c__ON,c__ON,  255,   c_OFF,c_OFF,   c_OFF,c_OFF,c_OFF },
	{AccentLEDChannelR,          0,    0,    1,   16,c_OFF,   c_OFF,c_UNC,  128,     128,c_UNC,   c_OFF,c_OFF,c_OFF },
	{AccentLEDChannelG,          0,    0,    1,   16,c_OFF,   c_OFF,c_UNC,   16,      16,c_UNC,   c_OFF,c_OFF,c_OFF },
	{AccentLEDChannelB,          0,    0,    1,   16,c_OFF,   c_OFF,c_UNC,   16,      16,c_UNC,   c_OFF,c_OFF,c_OFF },
	{SwithchLEDPinR1 + 512,        0,    0,    1,    0,c_OFF,   c_OFF,c__ON,c_OFF,   c__ON,c_OFF,   c_OFF,c_OFF,c_OFF },
	{SwithchLEDPinR2 + 512,        0,    0,    1,    0,c_OFF,   c_OFF,c_OFF,c_OFF,   c_OFF,c__ON,   c_OFF,c_OFF,c_OFF },
	{SwithchLEDPinG1 + 512,        0,    0,    1,    0,   32,   c_OFF,c_OFF,c__ON,   c__ON,c_OFF,   c_OFF,c_OFF,c_OFF },
	{SwithchLEDPinG2 + 512,        0,    0,    1,    0,c_OFF,   c_OFF,c_OFF,c_OFF,   c_OFF,c_OFF,   c_OFF,c_OFF,c_OFF },
	{SwithchLEDPinR3 + 512,        0,    0,    1,    0,c_OFF,   c_OFF,c__ON,c_OFF,   c__ON,c_OFF,   c_OFF,c_OFF,c_OFF },
	{SwithchLEDPinR4 + 512,        0,    0,    1,    0,c_OFF,   c_OFF,c_OFF,c_OFF,   c_OFF,c__ON,   c_OFF,c_OFF,c_OFF },
	{SwithchLEDPinG3 + 512,        0,    0,    1,    0,   32,   c_OFF,c_OFF,c__ON,   c__ON,c_OFF,   c_OFF,c_OFF,c_OFF },
	{SwithchLEDPinG4 + 512,        0,    0,    1,    0,c_OFF,   c_OFF,c_OFF,c_OFF,   c_OFF,c_OFF,   c_OFF,c_OFF,c_OFF },
	{FloodRelay0,                0,    0,    0,    0,c_OFF,   c_OFF,c_OFF,c_OFF,   c_OFF,c_OFF,   c_OFF,c_OFF,c_OFF },
	{FloodRelay1,                0,    0,    0,    0,c_OFF,   c_OFF,c_OFF,c_OFF,   c_OFF,c_OFF,   c_OFF,c_OFF,c_OFF },
	{FloodRelay2,                0,    0,    0,    0,c_OFF,   c_OFF,c_OFF,c_OFF,   c_OFF,c_OFF,   c_OFF,c_OFF,c_OFF }
};

// these constants are to name the fields in the arrays
// any changes to these definitions will require a change to the MAXTHEMES defintion
#define l_chan   0
#define l_curval 1
#define l_tarval 2
#define l_step   3
#define l_min    4
#define l_theme  5

boolean lightAction = false;
int colChange = 0;
int xmasTheme = 6;
#define XMASCOUNT 4096
boolean lit = false;
boolean xmasMode = false;
void setup() {

	if (debug) {
		Serial.begin(9600);
		Serial.println("Lighting Computer Active");
	}

	LastButtonValue = 0;
	// Initialize the DMX controller
	DMXSerial.init(DMXController);
	// DMX devices typically need to receive a complete set of channels

	// We start the program by setting all the channels to zero. 
	// This might not be a great idea given it will plunge into darkness, 
	// but it is a reasonable starting point at this time.
	pinMode(EMERGENCYPIN, INPUT_PULLUP); // emergency toggle switch
	EverythingOff();  // this should be the only time we use this routine which sets all of the DMX channels to zero and manually sets the LED values. 

}

void loop() {
	if (lightAction)
	{
		lightAction = action();
		delay(10);  // reduced from 100 to 10 to see if this cures a flicker in the LEDs
	}
	else
	{
		//  This section is simply to count a delay between the Christmas transitions
		//  we know that we have reached the requested state
		if (xmasMode)
		{
			if (colChange-- == 0)
			{
				if (lit) {
					SetTheme(8, 8);
					lit = false;
					colChange = 0;
				}
				else
				{
					colChange = XMASCOUNT;
					SetTheme(xmasTheme++, 1);
					if (xmasTheme == 8)
					{
						xmasTheme = 6;
					}
					lit = true;
				}
			}
		}
	}
	ButtonValue = ReadButtons();
	if (ButtonValue > 0 && LastButtonValue != ButtonValue)
	{
		switch (ButtonValue)
		{
		case SwLeftTop:
			xmasMode = false;
			SetTheme(1, 8);
			dbgprint("Theme 1 Requested");
			break;
		case SwLeftMid:
			xmasMode = false;
			SetTheme(2, 8);
			dbgprint("Theme 2 Requested");
			break;
		case SwLeftBot:
			xmasMode = false;
			SetTheme(3, 8);
			dbgprint("Theme 3 Requested");
			break;
		case SwRightTop:
			xmasMode = false;
			SetTheme(4, 8);
			dbgprint("Theme 4 Requested");
			break;
		case SwRightMid:
			xmasMode = false;
			SetTheme(5, 8);
			dbgprint("Theme 5 Requested");
			break;
		case SwRightBot:
			SetTheme(0, 255);  //  Theme 0 represets the off state
			dbgprint("Theme Off Requested");
			break;
		default:
			break;
		}
		LastButtonValue = ButtonValue;
	}
}
void SetTheme(int theme, int fade) {
	// this routine loops round all the circuits changing the target value to the respective theme value
	lightAction = true; // Say there is something to action
	for (int i = 0;i < MAXCIRCUITS; i++) {
		if (lighting[i][l_theme + theme]< c_UNC) {
			// lighting levels > 255 means that the lighting level remains unchanged.
			lighting[i][l_tarval] = lighting[i][l_theme + theme];
			lighting[i][l_step] = fade;
		}
	}
}

int ReadButtons(void) {
	// this routine reads the buttons from both of the switch pads and turns.
	// the values for each button are different in the two keypads :-(
	// potentially both pads have buttons pressed so the main keypad takes priority.
	//
	// Check the digital pins ahead of the analogue ladders because these are used for debugging
	//

	if (digitalRead(EMERGENCYPIN) == LOW) { // if the digital pin for the emergency mode has been pulled LOW
		emer_sw = !emer_sw;  // toggle the switch state
		delay(500);          // debounce 0.5 secs
		if (emer_sw) {
			//digitalWrite(13,HIGH); 
			emer_mode++;
			if (emer_mode == MAXTHEMES) { emer_mode = 0; }
			emer_sw = !emer_sw;  // toggle the switch state
			return(emer_mode); // turn on the lights
		}
	}

	// this reads the main keypad
	int sw1 = analogRead(0) / 100;
	int sw2 = analogRead(1) / 100;
	if (sw1 < 9) {
		Serial.print("sw1 : ");
		Serial.println(sw1);
	}
	if (sw2 < 9)
	{
		Serial.print("sw2 : ");
		Serial.println(sw2);
	}
	switch (analogRead(0) / 100) {
	case SwALeftTop:
		return(1);
		break;
	case SwALeftMid:
		return(2);
		break;
	case SwALeftBot:
		return(3);
		break;
	case SwARightTop:
		return(4);
		break;
	case SwARightMid:
		return(5);
		break;
	case SwARightBot:
		return(6);
		break;
	default:
		// if there was no value set for this switch we defer to what was on the other one.
		break;
	}
	// this reads the auxilliary keypad      
	switch (analogRead(1) / 100) {
	case SwBLeftTop:
		return(1);
		break;
	case SwBLeftMid:
		return(2);
		break;
	case SwBLeftBot:
		return(3);
		break;
	case SwBRightTop:
		return(4);
		break;
	case SwBRightMid:
		return(5);
		break;
	case SwBRightBot:
		return(6);
		break;
	default:
		return(0);
		break;
	}
}
void dbgprint(const char debugString[]) {
	if (debug)
	{
		Serial.println(debugString);
	}
}
bool action(void) {
	// this routine loops round all of the lighting channels and nudges the current values
	// towards their target values
	// it returns true if any of the lighting values were changed and false if nothing was done
	bool LightingChanged = false;
	int FadeAmount = 0;  //fade amount
	for (int i = 0;i < MAXCIRCUITS; i++) {
		if (lighting[i][l_step] > 0) {
			FadeAmount = lighting[i][l_tarval] - lighting[i][l_curval];  //result is signed for direction of dimming 
			if (FadeAmount > 0) {
				// fade up, so we check to see if our target is below our minimum value and if it is we adjust it up to the minimum
				if (lighting[i][l_tarval] < lighting[i][l_min]) {
					lighting[i][l_tarval] = lighting[i][l_min];
				}
				if (FadeAmount > lighting[i][l_step]) {
					//  step value is less than the amount still to fade so we use it to increment the light level
					//  if the new value isn't below our minimum value for the channel
					if (lighting[i][l_curval] >= lighting[i][l_min]) {
						lighting[i][l_curval] += lighting[i][l_step];
					}
					else {
						lighting[i][l_curval] = lighting[i][l_min];
					}

				}
				else {
					//  we don't want to overshoot the target value so we'll say we're done 
					lighting[i][l_curval] = lighting[i][l_tarval];
					lighting[i][l_step] = 0;  //indicate that no more changes are necessary
				}
			}
			else if (FadeAmount < 0) {
				// fade down and if the target is lower than the minimum allowed for the channel then we lower our target to off
				if (lighting[i][l_tarval] < lighting[i][l_min]) {
					lighting[i][l_tarval] = 0;
				}

				if (abs(FadeAmount) > lighting[i][l_step]) {
					//  step value is less than the amount still to fade so we use it to decrement the light level
					if (lighting[i][l_curval] < lighting[i][l_min]) {
						lighting[i][l_curval] = 0;  //  if we're below the minimum value, then we turn off the channel 
						lighting[i][l_step] = 0;    //  indicate that no more changes are necessary
						lighting[i][l_tarval] = 0;  //  The target is now adjusted to off even if it is below the minimum value

					}
					else {
						lighting[i][l_curval] -= lighting[i][l_step];
					}
				}
				else {
					//  we don't want to overshoot the target value so we'll say we're done 
					lighting[i][l_curval] = lighting[i][l_tarval];
					lighting[i][l_step] = 0;  //indicate that no more changes are necessary
				}
			}
			if (FadeAmount != 0) {
				LightingChanged = true;
				lightingWrite(lighting[i][l_chan], lighting[i][l_curval]);
			}
		}
	}
	return(LightingChanged);
}
void lightingWrite(int chan, int level) {
#define typeDMX 1
#define typeLED 2
	int chan_type = 0;  // DMX channel

						// the channels below 512 are DMX, after that the channels have special characteristics
						// between 512 and 1023, these are pins on the arduino
	if (chan > 512) {
		chan_type = typeLED;
		chan = chan - 512;
	}
	else {
		chan_type = typeDMX;
	}
	switch (chan_type) {
	case typeDMX:
		DMXSerial.write(chan, level);
		break;
	case typeLED:
		if (level > 0)
			digitalWrite(chan, HIGH);
		else
			digitalWrite(chan, LOW);

		break;
	default:
		break;
	}
}
void lightingDump() {
	int i = 0;
	int j = 0;
	if (debug) {
		Serial.println("* * * * * * * * * * * * * * * * * *");
		Serial.println("* * * Dumping Lighting States * * *");
		Serial.println("* * * * * * * * * * * * * * * * * *");
		for (i = 0, j = 0;i<MAXCIRCUITS && j < MAXTHEMES;i++, j++) {
			if (j == 0) {
				Serial.print("\r\n: ");
			}
			else {
				Serial.print(", ");
			}
			Serial.print(lighting[i][j], HEX);
		}
	}
}
void EverythingOff(void) {
	int channel_number = 0;
	/* go through the channels setting them to zero in case they are on */
	// this is the simple way to set everything off, but we only use this in the setup routine.

	dbgprint("All Off");
	for (channel_number = 0; channel_number <= MAXDMXCHANNELS; channel_number++) {
		DMXSerial.write(channel_number, 0);
	}
	for (int i = 0;i < MAXCIRCUITS; i++) {
		lighting[i][l_tarval] = 0;
		lighting[i][l_curval] = 0;
	}

	//  analogWrite(SwithchLEDPinR1,0);
	//  analogWrite(SwithchLEDPinR2,0);
	//  analogWrite(SwithchLEDPinG1,64);
	//  analogWrite(SwithchLEDPinG2,0); 

	pinMode(SwithchLEDPinR1, OUTPUT);
	pinMode(SwithchLEDPinR2, OUTPUT);
	pinMode(SwithchLEDPinG1, OUTPUT);
	pinMode(SwithchLEDPinG2, OUTPUT);
	pinMode(SwithchLEDPinR3, OUTPUT);
	pinMode(SwithchLEDPinR4, OUTPUT);
	pinMode(SwithchLEDPinG3, OUTPUT);
	pinMode(SwithchLEDPinG4, OUTPUT);

	digitalWrite(SwithchLEDPinR1, LOW);
	digitalWrite(SwithchLEDPinR2, LOW);
	digitalWrite(SwithchLEDPinG1, HIGH);
	digitalWrite(SwithchLEDPinG2, LOW);
	digitalWrite(SwithchLEDPinR3, LOW);
	digitalWrite(SwithchLEDPinR4, LOW);
	digitalWrite(SwithchLEDPinG3, HIGH);
	digitalWrite(SwithchLEDPinG4, LOW);
}




