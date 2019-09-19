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


// put 100nF ceramic capitors between ground and buttons output to prevent spikes

// libraries included
#include "Arduino.h"
#include <OSCMessage.h>

#ifdef BOARD_HAS_USB_SERIAL
  #include <SLIPEncodedUSBSerial.h>
  SLIPEncodedUSBSerial SLIPSerial(thisBoardsSerialUSB);
  #else
  #include <SLIPEncodedSerial.h>
  SLIPEncodedSerial SLIPSerial(Serial);
#endif

// constants and macros
#define MACRO_BTN_1			A0
#define MACRO_BTN_2			A1
#define MACRO_BTN_3			A2
#define MACRO_BTN_4			A3
#define MACRO_BTN_5			A4
#define MACRO_BTN_6			A5
#define MACRO_BTN_7			2
#define MACRO_BTN_8			3
#define MACRO_BTN_9			4
#define MACRO_BTN_10		5
#define MACRO_BTN_11		6
#define MACRO_BTN_12		7

#define SUBSCRIBE				1
#define UNSUBSCRIBE			0

#define EDGE_DOWN				1
#define EDGE_UP					0

#define DEFAULT_USER_ID 0

const String HANDSHAKE_QUERY = "ETCOSC?";
const String HANDSHAKE_REPLY = "OK";
const String PING_QUERY = "macrobox_hello";
const String PING_REPLAY = "/eos/out/ping";

// Change these values to alter how long we wait before sending an OSC ping
// to see if Eos is still there, and then finally how long before we
// disconnect and show the splash screen
// Values are in milliseconds
#define PING_AFTER_IDLE_INTERVAL    2500
#define TIMEOUT_AFTER_IDLE_INTERVAL 5000

// variables
bool connectedToEos = false;
unsigned long lastMessageRxTime = 0;
bool timeoutPingSent = false;

// datatypes
struct macroButton {
	uint8_t macro;
	uint16_t user;
	uint8_t pin;
	uint8_t last;
	String firePattern;
	} button1, button2, button3, button4, button5, button6, button7, button8, button9, button10, button11, button12; 

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
void filterSubscribes() {
	// Add a filter so we don't get spammed with unwanted OSC messages from Eos
	OSCMessage filter("/eos/filter/add");
	filter.add("/eos/out/ping");
	SLIPSerial.beginPacket();
	filter.send(SLIPSerial);
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
	filterSubscribes();
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
		}
	if (msg.indexOf(PING_REPLAY) != -1) {
		connectedToEos = true;
		}
	}

/**
 * @brief 
 * Initializes a given button struct to the requested parameters
 * 
 * @param button 
 * @param pin 
 * @param user 
 * @param macro 
 */
void initMacroButton(struct macroButton* button, uint8_t pin, uint16_t user, uint16_t macro) {
	button->macro = macro;
	button->user = user;
	button->pin = pin;
	button->last = HIGH;
	button->firePattern = "/eos/user/" + String(user) + "/macro/" + String(macro) + "/fire";
	pinMode(pin, INPUT_PULLUP);
	}

/**
 * @brief *
 * Checks if a mocro button has pressed or released by comparing the previous 
 * state of the pins with the current state. If they are different, 
 * we know the button is pressed or released.
 * In the event of a macro button is pressed or released we send an message.
 * 
 * @param button 
 */
void updateMacroButton(struct macroButton* button) {
	if ((digitalRead(button->pin)) != button->last) {
		OSCMessage fireUpdate(button->firePattern.c_str());
		if (button->last == LOW) {
			button->last = HIGH;
			fireUpdate.add(EDGE_UP);
			}
		else {
			button->last = LOW;
			fireUpdate.add(EDGE_DOWN);
			}
		SLIPSerial.beginPacket();
		fireUpdate.send(SLIPSerial);
		SLIPSerial.endPacket();
		} 
	}

/**
 * @brief setup arduino
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

	initEOS(); // for hotplug with Arduinos without native USB like UNO

	// initialize the macro buttons
	initMacroButton(&button1, MACRO_BTN_1, DEFAULT_USER_ID, 101);
	initMacroButton(&button2, MACRO_BTN_2, DEFAULT_USER_ID, 102);
	initMacroButton(&button3, MACRO_BTN_3, DEFAULT_USER_ID, 103);
	initMacroButton(&button4, MACRO_BTN_4, DEFAULT_USER_ID, 104);
	initMacroButton(&button5, MACRO_BTN_5, DEFAULT_USER_ID, 105);
	initMacroButton(&button6, MACRO_BTN_6, DEFAULT_USER_ID, 106);
	initMacroButton(&button7, MACRO_BTN_7, DEFAULT_USER_ID, 107);
	initMacroButton(&button8, MACRO_BTN_8, DEFAULT_USER_ID, 108);
	initMacroButton(&button9, MACRO_BTN_9, DEFAULT_USER_ID, 109);
	initMacroButton(&button10, MACRO_BTN_10, DEFAULT_USER_ID, 110);
	initMacroButton(&button11, MACRO_BTN_11, DEFAULT_USER_ID, 111);
	initMacroButton(&button12, MACRO_BTN_12, DEFAULT_USER_ID, 112);

}

/**
 * @brief arduino loop
 *
 */
void loop()
{
	static String curMsg;
	int size;

	// Then we check to see if any OSC commands have come from Eos
	// and update the display accordingly.
	size = SLIPSerial.available();
	if (size > 0)
	{
		// Fill the msg with all of the available bytes
		while (size--)
			curMsg += (char)(SLIPSerial.read());
	}
	if (SLIPSerial.endofPacket())
	{
		parseOSCMessage(curMsg);
		lastMessageRxTime = millis();
		// We only care about the ping if we haven't heard recently
		// Clear flag when we get any traffic
		timeoutPingSent = false;
		curMsg = String();
	}

	// update for the macro buttons
	updateMacroButton(&button1);
	updateMacroButton(&button2);
	updateMacroButton(&button3);
	updateMacroButton(&button4);
	updateMacroButton(&button5);
	updateMacroButton(&button6);
	updateMacroButton(&button7);
	updateMacroButton(&button8);
	updateMacroButton(&button9);
	updateMacroButton(&button10);
	updateMacroButton(&button11);
	updateMacroButton(&button12);

	if (lastMessageRxTime > 0) 
	{
		unsigned long diff = millis() - lastMessageRxTime;
		//We first check if it's been too long and we need to time out
		if (diff > TIMEOUT_AFTER_IDLE_INTERVAL) 
		{
			connectedToEos = false;
			lastMessageRxTime = 0;

			timeoutPingSent = false;
		}

		//It could be the console is sitting idle. Send a ping once to
		// double check that it's still there, but only once after 2.5s have passed
		if (!timeoutPingSent && diff > PING_AFTER_IDLE_INTERVAL) {
			sendPing();
		}
	}
}
