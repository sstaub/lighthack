/*
Copyright (c) 2017 Electronic Theatre Controls, Inc., http://www.etcconnect.com
Copyright (c) 2020 Stefan Staub

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/

// put 100nF ceramic capitors between ground and the input of the buttons and encoders

// libraries included

#include "Arduino.h"
#include <LiquidCrystal.h>
#include <OSCMessage.h>

#ifdef BOARD_HAS_USB_SERIAL
	#include <SLIPEncodedUSBSerial.h>
	SLIPEncodedUSBSerial SLIPSerial(thisBoardsSerialUSB);
#else
	#include <SLIPEncodedSerial.h>
	SLIPEncodedSerial SLIPSerial(Serial);
#endif

// constants and macros
#define LCD_CHARS				16
#define LCD_LINES				2

#define LCD_RS					2
#define LCD_ENABLE			3
#define LCD_D4					4
#define LCD_D5					5
#define LCD_D6					6
#define LCD_D7					7

#define ENC_1_A					A0
#define ENC_1_B					A1
#define ENC_2_A					A2
#define ENC_2_B					A3

#define NEXT_BTN				8
#define LAST_BTN				9
#define SHIFT_BTN				10

#define SUBSCRIBE				1
#define UNSUBSCRIBE			0

#define UP							0
#define DOWN						1

#define EDGE_UP					0
#define EDGE_DOWN				1

#define FORWARD					0
#define REVERSE					1

// Change these values to switch which direction increase/decrease pan/tilt
#define PAN_DIR					FORWARD
#define TILT_DIR				FORWARD

// Use these values to make the encoder more coarse or fine. This controls
// the number of wheel "ticks" the device sends to Eos for each tick of the
// encoder. 1 is the default and the most fine setting. Must be an integer.
#define PAN_SCALE				1
#define TILT_SCALE			1
#define WHEEL_ACC				4 // only used for intens

#define SIG_DIGITS			3 // Number of significant digits displayed

const String HANDSHAKE_QUERY = "ETCOSC?";
const String HANDSHAKE_REPLY = "OK";
const String PING_QUERY = "box1_hello";

//See displayScreen() below - limited to 10 chars (after 6 prefix chars)
const String VERSION_STRING = "1.1.0.0";

// Change these values to alter how long we wait before sending an OSC ping
// to see if Eos is still there, and then finally how long before we
// disconnect and show the splash screen
// Values are in milliseconds
#define PING_AFTER_IDLE_INTERVAL		2500
#define TIMEOUT_AFTER_IDLE_INTERVAL	5000

// globalvariables
bool updateDisplay = false;
bool connectedToEos = false;
unsigned long lastMessageRxTime = 0;
bool timeoutPingSent = false;

// datatypes
enum WHEEL_TYPE {TILT, PAN};
enum WHEEL_MODE {COARSE, FINE};

struct Encoder {
		uint8_t pinA;
		uint8_t pinB;
		int pinAPrevious;
		int pinBPrevious;
		float pos;
		uint8_t direction;
	}panWheel, tiltWheel;

// constructors
LiquidCrystal lcd(LCD_RS, LCD_ENABLE, LCD_D4, LCD_D5, LCD_D6, LCD_D7); // rs, enable, d0, d1, d2, d3


// local functions

/**
 * @brief 
 * Send a ping with a message to the console
 *
 */
void sendPing() {
	OSCMessage ping("/eos/ping");
	ping.add(PING_QUERY);
	SLIPSerial.beginPacket();
	ping.send(SLIPSerial);
	SLIPSerial.endPacket();
	timeoutPingSent = true;
	}

/**
 * @brief 
 * Add a filter so we don't get spammed with unwanted OSC messages from Eos
 * 
 */
void issueFilters() {
	// Add a filter so we don't get spammed with unwanted OSC messages from Eos
	OSCMessage filter("/eos/filter/add");
	filter.add("/eos/out/param/*");
	filter.add("/eos/out/ping");
	SLIPSerial.beginPacket();
	filter.send(SLIPSerial);
	SLIPSerial.endPacket();
	}

/**
 * @brief 
 * Issues all our subscribes to Eos. When subscribed, Eos will keep us updated
 * with the latest values for a given parameter.
 * 
 */
void issueSubscribes() {
	// subscribe to Eos pan & tilt updates
	OSCMessage subPan("/eos/subscribe/param/pan");
	subPan.add(SUBSCRIBE);
	SLIPSerial.beginPacket();
	subPan.send(SLIPSerial);
	SLIPSerial.endPacket();

	OSCMessage subTilt("/eos/subscribe/param/tilt");
	subTilt.add(SUBSCRIBE);
	SLIPSerial.beginPacket();
	subTilt.send(SLIPSerial);
	SLIPSerial.endPacket();
	}

/**
 * @brief
 * Init the console, gives back a handshake reply
 * and send the filters and subscribtions.
 *
 */
void initEOS() {
	SLIPSerial.print(HANDSHAKE_REPLY);
	issueFilters();
	issueSubscribes();
	}

/**
 * @brief 
 * Given a valid OSCMessage (relevant to Pan/Tilt), we update our Encoder struct
 * with the new valueition information.
 * 
 * @param msg - the OSC message we will use to update our internal data
 * @param addressOffset - unused (allows for multiple nested roots)
 */
void parsePanUpdate(OSCMessage& msg, int addressOffset) {
	panWheel.pos = msg.getOSCData(0)->getFloat();
	connectedToEos = true; // Update this here just in case we missed the handshake
	updateDisplay = true; 
	}

void parseTiltUpdate(OSCMessage& msg, int addressOffset) {
	tiltWheel.pos = msg.getOSCData(0)->getFloat();
	connectedToEos = true; // Update this here just in case we missed the handshake
	updateDisplay = true;
	}

/**
 * @brief 
 * Given an unknown OSC message we check to see if it's a handshake message.
 * If it's a handshake we issue a subscribe, otherwise we begin route the OSC
 * message to the appropriate function.
 * 
 * @param msg - the OSC message of unknown importance
 *
 */
void parseOSCMessage(String& msg) {
	// check to see if this is the handshake string
	if (msg.indexOf(HANDSHAKE_QUERY) != -1) {
		initEOS();
		// Make our splash screen go away
		connectedToEos = true;
		updateDisplay = true;
		}
	else {
		// prepare the message for routing by filling an OSCMessage object with our message string
		OSCMessage oscmsg;
		oscmsg.fill((uint8_t*)msg.c_str(), (int)msg.length());
		// route pan/tilt messages to the relevant update function
		oscmsg.route("/eos/out/param/pan", parsePanUpdate);
		oscmsg.route("/eos/out/param/tilt", parseTiltUpdate);
		}
	}

/**
 * @brief 
 * Updates the display with the latest pan and tilt valueitions.
 * 
 */
void displayStatus() {
	lcd.clear();

	if (!connectedToEos) {
		// display a splash message before the Eos connection is open
		lcd.setCursor(0, 0);
		lcd.print(String("Box1 v" + VERSION_STRING).c_str());
		lcd.setCursor(0, 1);
		lcd.print("waiting for eos");
		}
	else {
		// put the cursor at the begining of the first line
		lcd.setCursor(0, 0);
		lcd.print("Pan:  ");
		lcd.print(panWheel.pos, SIG_DIGITS);

		// put the cursor at the begining of the second line
		lcd.setCursor(0, 1);
		lcd.print("Tilt: ");
		lcd.print(tiltWheel.pos, SIG_DIGITS);
		}
	updateDisplay = false;
	}

/**
 * @brief 
 * Initializes a given encoder struct to the requested parameters.
 * 
 * @param encoder - pointer to the encoder we will be initializing
 * @param pinA - where the A pin is connected to the Arduino
 * @param pinB - where the B pin is connected to the Arduino
 * @param direction - determines if clockwise or counterclockwise is "forward"
 */
void initEncoder(struct Encoder* encoder, uint8_t pinA, uint8_t pinB, uint8_t direction) {
	encoder->pinA = pinA;
	encoder->pinB = pinB;
	encoder->direction = direction;

	pinMode(pinA, INPUT_PULLUP);
	pinMode(pinB, INPUT_PULLUP);

	encoder->pinAPrevious = digitalRead(pinA);
	encoder->pinBPrevious = digitalRead(pinB);
	}

/**
 * @brief 
 * Checks if the encoder has moved by comparing the previous state of the pins
 * with the current state. If they are different, we know there is movement.
 * In the event of movement we update the current state of our pins.
 * 
 * @param encoder - pointer to the encoder we will be checking for motion
 * @return int8_t 
 * encoderMotion - returns the 0 if the encoder has not moved
 *                              1 for forward motion
 *                             -1 for reverse motion
 *
 */
int8_t updateEncoder(struct Encoder* encoder) {
	int8_t encoderMotion = 0;
	int pinACurrent = digitalRead(encoder->pinA);
	int pinBCurrent = digitalRead(encoder->pinB);

	// has the encoder moved at all?
	if (encoder->pinAPrevious != pinACurrent) {
		// Since it has moved, we must determine if the encoder has moved forwards or backwards
		encoderMotion = (encoder->pinAPrevious == encoder->pinBPrevious) ? -1 : 1;

		// If we are in reverse mode, flip the direction of the encoder motion
		if (encoder->direction == REVERSE) encoderMotion = -encoderMotion;
		}

	encoder->pinAPrevious = pinACurrent;
	encoder->pinBPrevious = pinBCurrent;

	return encoderMotion;
	}

/**
 * @brief Sends a message to Eos informing them of a wheel movement.
 * 
 * @param type - the type of wheel that's moving (i.e. pan or tilt)
 * @param ticks - the direction and intensity of the movement
 */
void sendWheelMove(WHEEL_TYPE type, float ticks) {
	String wheelMsg("/eos/wheel");

	if (digitalRead(SHIFT_BTN) == LOW) wheelMsg.concat("/fine");
	else wheelMsg.concat("/coarse");

	if (type == PAN) wheelMsg.concat("/pan");
	else if (type == TILT) wheelMsg.concat("/tilt");
	else // something has gone very wrong
	return;

	OSCMessage wheelUpdate(wheelMsg.c_str());
	wheelUpdate.add(ticks);
	SLIPSerial.beginPacket();
	wheelUpdate.send(SLIPSerial);
	SLIPSerial.endPacket();
	}

/**
 * @brief Sends a message to Eos informing them of a key press.
 * 
 * @param down - whether a key has been pushed down (true) or released (false)
 * @param key - the key that has moved
 */
void sendKeyPress(bool down, String key) {
	key = "/eos/key/" + key;
	OSCMessage keyMsg(key.c_str());

	if (down) keyMsg.add(EDGE_DOWN);
	else keyMsg.add(EDGE_UP);

	SLIPSerial.beginPacket();
	keyMsg.send(SLIPSerial);
	SLIPSerial.endPacket();
	}

/*******************************************************************************
 * Checks the status of all the buttons relevant to Eos (i.e. Next & Last)
 *
 * NOTE: This does not check the shift key. The shift key is used in tandem with
 * the encoder to determine coarse/fine mode and thus does not report to Eos
 * directly.
 *
 * Parameters: none
 *
 * Return Value: void
 *
 ******************************************************************************/

/**
 * @brief Checks the status of all the buttons relevant to Eos (i.e. Next & Last)
 *
 * NOTE: This does not check the shift key. The shift key is used in tandem with
 * the encoder to determine coarse/fine mode and thus does not report to Eos
 * directly.
 * 
 */
void checkButtons() {
	static int nextKeyState = HIGH;
	static int lastKeyState = HIGH;

	// Has the button state changed
	if (digitalRead(NEXT_BTN) != nextKeyState) {
		// Notify Eos of this key press
		if (nextKeyState == LOW) {
			sendKeyPress(false, "NEXT");
			nextKeyState = HIGH;
			}
		else {
			sendKeyPress(true, "NEXT");
			nextKeyState = LOW;
			}
		}

	if (digitalRead(LAST_BTN) != lastKeyState) {
		if (lastKeyState == LOW) {
			sendKeyPress(false, "LAST");
			lastKeyState = HIGH;
			}
		else {
			sendKeyPress(true, "LAST");
			lastKeyState = LOW;
			}
		}
	}

/**
 * @brief 
 * Here we setup our encoder, lcd, and various input devices. We also prepare
 * to communicate OSC with Eos by setting up SLIPSerial. Once we are done with
 * setup() we pass control over to loop() and never call setup() again.
 *
 * NOTE: This function is the entry function. This is where control over the
 * Arduino is passed to us (the end user).
 * 
 */
void setup() {
	SLIPSerial.begin(115200);
	// This is a hack around an Arduino bug. It was taken from the OSC library
	//examples
#ifdef BOARD_HAS_USB_SERIAL
	while (!SerialUSB);
#else
	while (!Serial);
#endif

	lcd.begin(LCD_CHARS, LCD_LINES);
	lcd.clear();

	initEOS(); // for hotplug with Arduinos without native USB like UNO

	pinMode(NEXT_BTN, INPUT_PULLUP);
	pinMode(LAST_BTN, INPUT_PULLUP);
	pinMode(SHIFT_BTN, INPUT_PULLUP);

	initEncoder(&panWheel, ENC_1_A, ENC_1_B, PAN_DIR);
	initEncoder(&tiltWheel, ENC_2_A, ENC_2_B, TILT_DIR);

	displayStatus();
	}

/**
 * @brief 
 * Here we service, monitor, and otherwise control all our peripheral devices.
 * First, we retrieve the status of our encoders and buttons and update Eos.
 * Next, we check if there are any OSC messages for us.
 * Finally, we update our display (if an update is necessary)
 *
 * NOTE: This function is our main loop and thus this function will be called
 * repeatedly forever
 * 
 */
void loop() {
	static String curMsg;
	int size;

	// get the updated state of each encoder
	int8_t panMotion = updateEncoder(&panWheel);
	int8_t tiltMotion = updateEncoder(&tiltWheel);

	// Scale the result by a scaling factor
	panMotion *= PAN_SCALE;
	tiltMotion *= TILT_SCALE;

	// check for next/last updates
	checkButtons();

	// now update our wheels
	if (tiltMotion != 0) sendWheelMove(TILT, tiltMotion);
	if (panMotion != 0) sendWheelMove(PAN, panMotion);

	// Then we check to see if any OSC commands have come from Eos
	// and update the display accordingly.
	size = SLIPSerial.available();
	if (size > 0) {
		// Fill the msg with all of the available bytes
		while (size--) curMsg += (char)(SLIPSerial.read());
		}
	if (SLIPSerial.endofPacket()) {
		parseOSCMessage(curMsg);
		lastMessageRxTime = millis();
		// We only care about the ping if we haven't heard recently
		// Clear flag when we get any traffic
		timeoutPingSent = false;
		curMsg = String();
		}

	if (lastMessageRxTime > 0) {
		unsigned long diff = millis() - lastMessageRxTime;
		//We first check if it's been too long and we need to time out
		if (diff > TIMEOUT_AFTER_IDLE_INTERVAL) {
			connectedToEos = false;
			lastMessageRxTime = 0;
			updateDisplay = true;
			timeoutPingSent = false;
			}
		//It could be the console is sitting idle. Send a ping once to
		// double check that it's still there, but only once after 2.5s have passed
		if (!timeoutPingSent && diff > PING_AFTER_IDLE_INTERVAL) sendPing();
		}
	if (updateDisplay) displayStatus();
	}

