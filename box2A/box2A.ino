// Copyright (c) 2017 Electronic Theatre Controls, Inc., http://www.etcconnect.com
// Copyright (c) 2019 Stefan Staub
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.


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

#define PARAM_UP_BTN		8
#define PARAM_DOWN_BTN	9
#define SHIFT_BTN				10
#define ENC_1_A					A0
#define ENC_1_B					A1
#define ENC_2_A					A2
#define ENC_2_B					A3

#define SUBSCRIBE				1
#define UNSUBSCRIBE			0

#define UP							0
#define DOWN						1

#define EDGE_UP					0
#define EDGE_DOWN				1

// number of encoders you use
#define ENCODERS				2

// These define which direction is "forward" for an encoder
#define FORWARD					0
#define REVERSE					1

// Change these values to switch which direction increase/decrease pan/tilt
#define ENC_1_DIR				FORWARD
#define ENC_2_DIR				FORWARD

// Use these values to make the encoder more coarse or fine. This controls
// the number of wheel "ticks" the device sends to Eos for each tick of the
// encoder. 1 is the default and the most fine setting. Must be an integer.
#define WHEEL_ACC				4 // only used for intens

#define SIG_DIGITS			2 // number of significant digits displayed

#define PING_AFTER_IDLE_INTERVAL		2500
#define TIMEOUT_AFTER_IDLE_INTERVAL	5000

const String VERSION = "#lighthack box2A";
const String HANDSHAKE_QUERY = "ETCOSC?";
const String HANDSHAKE_REPLY = "OK";
const String PING_QUERY = "box2a_hello";
const String SUBSCRIBE_QUERY = "/eos/subscribe/param/";
const String UNSUBSCRIBE_QUERY = "/eos/subscribe/param/*";
const String PARAMETER_QUERY = "/eos/out/param/";

const String EOS_KEY = "/eos/key";
const String EOS_NEXT_KEY = "NEXT";
const String EOS_LAST_KEY = "LAST";

const String NO_PARAMETER = "none"; // none is a keyword used when there is no parameter

// variables
bool updateDisplay = false;
bool connectedToEos = false;
uint32_t lastMessageRxTime = 0;
bool timeoutPingSent = false;
int8_t idx = 0; // start with parameter index 2 must even

// constructors
LiquidCrystal lcd(LCD_RS, LCD_ENABLE, LCD_D4, LCD_D5, LCD_D6, LCD_D7); // rs, enable, d0, d1, d2, d3

// datatypes
struct Parameter {
	String name;
	String display;
	float value;
	};

const uint8_t PARAMETER_MAX = 14; // number of parameters must even
struct Parameter parameter[PARAMETER_MAX] = {
	{"none", "------"},
	{"Intens"},
	{"Pan"},
	{"Tilt"},
	{"Zoom"},
	{"Edge"},
	{"Iris"},
	{"Diffusn"},
	//{"Hue"},
	//{"Saturatn"},
	{"Red"},
	{"Green"},
	{"Blue"},
	{"Cyan"},
	{"Magenta"},
	{"Yellow"}/*,
	{"cto", "CTO"},
	{"frame_assembly", "Assembly"},
	{"thrust_A", "ThrustA"},
	{"angle_A", "AngleA"},
	{"thrust_B", "ThrustB"},
	{"thrust_B", "AngleB"},
	{"thrust_C", "ThrustC"},
	{"angle_C", "AngleC"},
	{"thrust_D", "ThrustD"},
	{"angle_D", "AngleD"}*/
	};

struct Encoder {
	uint8_t parameterIdx;
	uint8_t pinA;
	uint8_t pinB;
	uint8_t pinAPrevious;
	uint8_t pinBPrevious;
	uint8_t direction;
	} encoder1, encoder2;

struct ControlButton {
	uint8_t function;
	uint8_t pin;
	uint8_t last;
	} parameterUp, parameterDown;

struct EncoderButton {
	uint8_t pin;
	uint8_t last;
	uint8_t encoderNumber;
	} encoderButton1, encoderButton2;

struct Key {
  String keyName;
  uint8_t pin;
  uint8_t last;
  } lastKey, nextKey;

/**
 * @brief send a ping with a message to the console
 *
 */
void sendPing() {
	OSCMessage ping("/eos/ping");
	ping.add(PING_QUERY.c_str());
	SLIPSerial.beginPacket();
	ping.send(SLIPSerial);
	SLIPSerial.endPacket();
	timeoutPingSent = true;
	}

/**
 * @brief 
 * add a filter so we don't get spammed with unwanted OSC messages from Eos
 * 
 */
void issueFilters() {
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
	// unsubscribes all parameters
	String unsubMsg = UNSUBSCRIBE_QUERY;
	OSCMessage unsubscribe(unsubMsg.c_str());
	unsubscribe.add(UNSUBSCRIBE);
	SLIPSerial.beginPacket();
	unsubscribe.send(SLIPSerial);
	SLIPSerial.endPacket();

	// subscribes the displayed parameters, exept the parameter with keyword none
	String subMsg;
	if (parameter[idx].name == NO_PARAMETER) {
		subMsg = SUBSCRIBE_QUERY + parameter[idx + 1].name;
		}
	else if (parameter[idx + 1].name == NO_PARAMETER) {
		subMsg = SUBSCRIBE_QUERY + parameter[idx].name;
		}
	else {
		subMsg = SUBSCRIBE_QUERY + parameter[idx].name + "/" + parameter[idx + 1].name;
		}
	OSCMessage subscribe(subMsg.c_str());
	subscribe.add(SUBSCRIBE);
	SLIPSerial.beginPacket();
	subscribe.send(SLIPSerial);
	SLIPSerial.endPacket();
	}

/**
 * @brief
 * Init the console, gives back a handshake reply
 * and send the subscribtions.
 *
 */
void initEOS() {
	SLIPSerial.beginPacket();
	SLIPSerial.write((const uint8_t*)HANDSHAKE_REPLY.c_str(), (size_t)HANDSHAKE_REPLY.length());
	SLIPSerial.endPacket();
	issueFilters();
	issueSubscribes();
	}

/**
 * @brief
 * Given an unknown OSC message we check to see if it's a handshake message.
 * If it's a handshake we issue a subscribe, otherwise we begin route the OSC
 * message to the appropriate function.
 *
 * @param msg OSC message
 */
void parseOSCMessage(String& msg) {
	// check to see if this is the handshake string
	if (msg.indexOf(HANDSHAKE_QUERY) != -1) {
		initEOS();
		connectedToEos = true;
		updateDisplay = true;
		}

	else {
		// checks if there is a message with data of your parameter list
		OSCMessage oscmsg;
		oscmsg.fill((uint8_t*)msg.c_str(), (int)msg.length());
		for (int i = 0; i < PARAMETER_MAX; i++) {
			String parseMsg = PARAMETER_QUERY + parameter[i].name;
			if(msg.indexOf(parseMsg) != -1) {
				parameter[i].value = oscmsg.getOSCData(0)->getFloat(); // get the value
				connectedToEos = true; // Update this here just in case we missed the handshake
				updateDisplay = true;
				}
			}
		}
	}

/**
 * @brief update the display
 *
 */
void displayStatus() {
	lcd.clear();
	if (!connectedToEos) {
		// display a splash message before the Eos connection is open
		lcd.setCursor(0, 0);
		lcd.print(VERSION);
		lcd.setCursor(0, 1);
		lcd.print("waiting for eos");
		}
	else {
		// put the cursor at the begining of the first line
		lcd.setCursor(0, 0);
		// check if display name is empty, if thrue take the parameter name
		if (parameter[encoder1.parameterIdx].display.length() == 0) {
			lcd.print(parameter[encoder1.parameterIdx].name);
			}
		else {
			lcd.print(parameter[encoder1.parameterIdx].display);
			}
		lcd.setCursor(8, 0);
		if (parameter[encoder2.parameterIdx].display.length() == 0) {
			lcd.print(parameter[encoder2.parameterIdx].name);
			}
		else {
		lcd.print(parameter[encoder2.parameterIdx].display);
			}

		// put the cursor at the begining of the second line
		lcd.setCursor(0, 1);
		lcd.print(parameter[encoder1.parameterIdx].value, SIG_DIGITS);
		lcd.setCursor(8, 1);
		lcd.print(parameter[encoder2.parameterIdx].value, SIG_DIGITS);
		}
		updateDisplay = false;
	}

/**
 * @brief initialise the encoder
 *
 * @param encoder
 * @param pinA
 * @param pinB
 * @param direction
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
 * @brief update the encoder
 *
 * @param encoder
 */
void updateEncoder(struct Encoder* encoder) {
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
	if ((encoderMotion != 0) && (encoder->parameterIdx >= 0)) {
		if (parameter[encoder->parameterIdx].name == NO_PARAMETER) return;
		String wheelMsg("/eos/wheel");
		if (parameter[encoder->parameterIdx].name == "Intens") {
			if (digitalRead(SHIFT_BTN) == LOW) encoderMotion *= WHEEL_ACC;
			}
		else if (encoder->parameterIdx > 1) {
			if (digitalRead(SHIFT_BTN) == LOW) wheelMsg.concat("/fine");
			else wheelMsg.concat("/coarse");
			}
		wheelMsg.concat('/' + parameter[encoder->parameterIdx].name);
		OSCMessage wheelUpdate(wheelMsg.c_str());
		wheelUpdate.add(encoderMotion);
		SLIPSerial.beginPacket();
		wheelUpdate.send(SLIPSerial);
		SLIPSerial.endPacket();
		}
	}


/**
 * @brief initalizes a given button struct to the requested parameters
 *
 * @param button name of the button structure
 * @param pin arduino pin of the button
 */
void initControlButton(struct ControlButton* button, uint8_t pin, uint8_t function) {
	button->function = function;
	button->pin = pin;
	button->last = HIGH;
	pinMode(pin, INPUT_PULLUP);
	}

/**
 * @brief sends a message to Eos informing them of a key press
 *
 * @param button
 */
void updateControlButton(struct ControlButton* button) {
	if ((digitalRead(button->pin)) != button->last) {
		if (button->last == LOW) {
			button->last = HIGH;
			//calc index
			if (button->function == UP) {
				idx += ENCODERS;
				if (idx > PARAMETER_MAX - ENCODERS) {
					idx = 0;
					} 
				encoder1.parameterIdx = idx;
				encoder2.parameterIdx = idx + 1;
				issueSubscribes();
				displayStatus();
				}
			
			if (button->function == DOWN) {
				idx -= ENCODERS;
				if (idx < 0) {
					idx = PARAMETER_MAX - ENCODERS;
					} 
				encoder1.parameterIdx = idx;
				encoder2.parameterIdx = idx + 1;
				issueSubscribes();
				displayStatus();
				}      
			}
		else {
			button->last = LOW;
			}
		}
	}

/**
 * @brief initalizes a given button struct to the requested parameters
 *
 * @param button name of the button structure
 * @param pin arduino pin of the button
 * @param key OSC name of the eos key
 */
void initEncoderButton(struct EncoderButton* button, uint8_t pin, uint8_t encoderNumber) {
	button->encoderNumber = encoderNumber;
	button->pin = pin;
	button->last = HIGH;
	pinMode(pin, INPUT_PULLUP);
	}

/**
 * @brief setup arduino
 *
 */
void setup() {
	SLIPSerial.begin(115200);

	#ifdef BOARD_HAS_USB_SERIAL
	 while (!SerialUSB);
	 #else
	 while (!Serial);
	#endif

	lcd.begin(LCD_CHARS, LCD_LINES);
	lcd.clear();

	initEOS(); // for hotplug with Arduinos without native USB like UNO

	// init of hardware elements
	initEncoder(&encoder1, A0, A1, ENC_1_DIR);
	initEncoder(&encoder2, A2, A3, ENC_2_DIR);
	encoder1.parameterIdx = idx;
	encoder2.parameterIdx = idx + 1;

	pinMode(SHIFT_BTN, INPUT_PULLUP);

	initControlButton(&parameterUp, PARAM_UP_BTN, UP);
	initControlButton(&parameterDown, PARAM_DOWN_BTN, DOWN);

	displayStatus();
	}

/**
 * @brief arduino loop
 *
 */
void loop() {
	static String curMsg;

	// Then we check to see if any OSC commands have come from Eos
	// and update the display accordingly.
	int size = SLIPSerial.available();
	if (size > 0) {
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

	// check for next/last updates
	updateEncoder(&encoder1);
	updateEncoder(&encoder2);
	updateControlButton(&parameterUp);
	updateControlButton(&parameterDown);

	if(lastMessageRxTime > 0) {
		unsigned long diff = millis() - lastMessageRxTime;
		//We first check if it's been too long and we need to time out
		if (diff > TIMEOUT_AFTER_IDLE_INTERVAL) {
			connectedToEos = false;
			lastMessageRxTime = 0;
			updateDisplay = true;
			timeoutPingSent = false;
			}

		// It could be the console is sitting idle. Send a ping once to
		// double check that it's still there, but only once after 2.5s have passed
		if (!timeoutPingSent && diff > PING_AFTER_IDLE_INTERVAL) {
			sendPing();
			}
		}
		if (updateDisplay) displayStatus();
	}
