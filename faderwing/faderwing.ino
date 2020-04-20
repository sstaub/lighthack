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

/*
fader is a linear 10kOhm, from Bourns or ALPS and can be 45/60/100mm long
put 10nF ceramic capitors between ground and fader levelers to prevent analog noise
Arduino UNO, MEGA:
use IOREF +5V to the top (single pin) of the fader (100%)
GND to the center button pin (2 pins, the outer pin is normaly for the leveler) of the fader (0%)
TEENSY:
+3.3V to the top (single pin) of the fader (100%)
use ANALOG GND instead the normal GND to the center button pin (2 pins, the outer pin is normaly for the leveler) of the fader (0%)

put 100nF ceramic capitors between ground and the input of the buttons
*/

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

// PINs setup
#define FADER_1_LEVELER			A0
#define FADER_1_FIRE_BUTTON	2
#define FADER_1_STOP_BUTTON	8
#define FADER_2_LEVELER			A1
#define FADER_2_FIRE_BUTTON	3
#define FADER_2_STOP_BUTTON	9
#define FADER_3_LEVELER			A2
#define FADER_3_FIRE_BUTTON	4
#define FADER_3_STOP_BUTTON	10
#define FADER_4_LEVELER			A3
#define FADER_4_FIRE_BUTTON	5
#define FADER_4_STOP_BUTTON	11
#define FADER_5_LEVELER			A4
#define FADER_5_FIRE_BUTTON	6
#define FADER_5_STOP_BUTTON	12
#define FADER_6_LEVELER			A5
#define FADER_6_FIRE_BUTTON	7
#define FADER_6_STOP_BUTTON	13


// constants and macros
#define SUBSCRIBE		1
#define UNSUBSCRIBE	0

#define EDGE_DOWN		1
#define EDGE_UP			0

#define PING_AFTER_IDLE_INTERVAL		2500
#define TIMEOUT_AFTER_IDLE_INTERVAL	5000

#define FADER_BANK				1
#define FADER_PAGE				1 // fader page on EOS / Nomad
#define NUMBER_OF_FADERS	10 // size of the faders per page on EOS / NOmad

#define FADER_UPDATE_RATE_MS	40 // update each 40ms

const int16_t THRESHOLD = 4; // Jitter threshold of the faders

uint32_t updateTime; 

const String HANDSHAKE_QUERY = "ETCOSC?";
const String HANDSHAKE_REPLY = "OK";
const String PING_QUERY = "faderwing_hello";
const String EOS_FADER = "/eos/fader";

// variables
bool connectedToEos = false;
unsigned long lastMessageRxTime = 0;
bool timeoutPingSent = false;

// datatypes
struct Fader {
	uint8_t number;
	uint8_t bank;
	uint8_t analogPin;
	uint8_t firePin;
	uint8_t stopPin;
	int16_t analogLast;
	int16_t fireLast;
	int16_t stopLast;
	String analogPattern;
	String firePattern;
	String stopPattern;
	uint32_t updateTime;
	} fader1, fader2, fader3, fader4, fader5, fader6;

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
	filter.add("/eos/out/ping");
	SLIPSerial.beginPacket();
	filter.send(SLIPSerial);
	SLIPSerial.endPacket();
	}

/**
 * @brief initialize a fader bank
 * 
 * @param bank number of the fader bank
 * @param page number of the fader page
 * @param faders number of faders in this bank
 */
void initFaders(uint8_t bank, uint8_t page, uint8_t faders) {
	String faderInit = "/eos/fader/";
	faderInit += bank;
	faderInit += "/config/";
	faderInit += page;
	faderInit += '/';
	faderInit += faders;
	OSCMessage faderBank(faderInit.c_str());
	SLIPSerial.beginPacket();
	faderBank.send(SLIPSerial);
	SLIPSerial.endPacket();
	}

/**
 * @brief
 * Init the console, gives back a handshake reply
 * and send the subscribtions.
 *
 */
void initEOS() {
	// do the handshake reply for usb conection
	SLIPSerial.beginPacket();
	SLIPSerial.write((const uint8_t*)HANDSHAKE_REPLY.c_str(), (size_t)HANDSHAKE_REPLY.length());
	SLIPSerial.endPacket();

	// Let Eos know we want updates on some things
	issueFilters();

	// activate a fader bank
	initFaders(FADER_BANK, FADER_PAGE, NUMBER_OF_FADERS);
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
		// handshake string found!
		connectedToEos = true;
		initEOS();
		}

	if (msg.indexOf(PING_QUERY) != -1) {
		// handshake string found!
		connectedToEos = true;
		}
	}

/**
 * @brief initialise the fader
 *
 * @param fader
 * @param bank
 * @param number
 * @param analogPin
 * @param firePin
 * @param stopPin
 */
void initFader(struct Fader* fader, uint8_t bank, uint8_t number, uint8_t analogPin, uint8_t firePin, uint8_t stopPin) {
	fader->bank = bank;
	fader->number = number;
	fader->analogPin = analogPin;
	fader->firePin = firePin;
	fader->stopPin = stopPin;
	fader->analogLast = 0xFFFF; // forces an osc output of the fader
	fader->fireLast = HIGH;
	fader->stopLast = HIGH;
	pinMode(fader->firePin, INPUT_PULLUP);
	pinMode(fader->stopPin, INPUT_PULLUP);
	fader->analogPattern = EOS_FADER + '/' + String(fader->bank) + '/' + String(fader->number);
	fader->firePattern = EOS_FADER + '/' + String(fader->bank) + '/' + String(fader->number) + "/fire";
	fader->stopPattern = EOS_FADER + '/' + String(fader->bank) + '/' + String(fader->number) + "/stop";
	fader->updateTime = millis();
	}

/**
 * @brief update the fader
 *
 * @param fader
 */
void updateFader(struct Fader* fader) {
	if((fader->updateTime + FADER_UPDATE_RATE_MS) < millis()) {
		int16_t raw = analogRead(fader->analogPin) >> 2; // reduce to 8 bit
		int16_t current = THRESHOLD;
		int16_t delta = raw - current;
		if (delta <= -THRESHOLD) current = raw + THRESHOLD;
		if (delta >= THRESHOLD) current = raw - THRESHOLD;
		if (current != fader->analogLast) {
			float value = ((current - THRESHOLD) * 1.0 / (255 - 2 * THRESHOLD)) / 1.0; // normalize to values between 0.0 and 1.0
			fader->analogLast = current;

			OSCMessage faderUpdate(fader->analogPattern.c_str());
			faderUpdate.add(value);
			SLIPSerial.beginPacket();
			faderUpdate.send(SLIPSerial);
			SLIPSerial.endPacket();
			}

		if((digitalRead(fader->firePin)) != fader->fireLast) {
			OSCMessage fireUpdate(fader->firePattern.c_str());
			if(fader->fireLast == LOW) {
				fader->fireLast = HIGH;
				fireUpdate.add(EDGE_UP);
				}
			else {
				fader->fireLast = LOW;
				fireUpdate.add(EDGE_DOWN);
				}
			SLIPSerial.beginPacket();
			fireUpdate.send(SLIPSerial);
			SLIPSerial.endPacket();
			}

		if((digitalRead(fader->stopPin)) != fader->stopLast) {
			OSCMessage stopUpdate(fader->stopPattern.c_str());
			if(fader->stopLast == LOW) {
				fader->stopLast = HIGH;
				stopUpdate.add(EDGE_UP);
				}
			else {
				fader->stopLast = LOW;
				stopUpdate.add(EDGE_DOWN);
				}
			SLIPSerial.beginPacket();
			stopUpdate.send(SLIPSerial);
			SLIPSerial.endPacket();
			}
		fader->updateTime = millis();
		}
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
	// this is necessary for reconnecting a device because it need some timme for the serial port to get open, but meanwhile the handshake message was send from eos
	
	initEOS();

	// init of hardware elements
	initFader(&fader1, FADER_BANK, 1, FADER_1_LEVELER, FADER_1_FIRE_BUTTON, FADER_1_STOP_BUTTON);
	initFader(&fader2, FADER_BANK, 2, FADER_2_LEVELER, FADER_2_FIRE_BUTTON, FADER_2_STOP_BUTTON);
	initFader(&fader3, FADER_BANK, 3, FADER_3_LEVELER, FADER_3_FIRE_BUTTON, FADER_3_STOP_BUTTON);
	initFader(&fader4, FADER_BANK, 4, FADER_4_LEVELER, FADER_4_FIRE_BUTTON, FADER_4_STOP_BUTTON);
	initFader(&fader5, FADER_BANK, 5, FADER_5_LEVELER, FADER_5_FIRE_BUTTON, FADER_5_STOP_BUTTON);
	initFader(&fader6, FADER_BANK, 6, FADER_6_LEVELER, FADER_6_FIRE_BUTTON, FADER_6_STOP_BUTTON);
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

	if(lastMessageRxTime > 0) {
		unsigned long diff = millis() - lastMessageRxTime;
		//We first check if it's been too long and we need to time out
		if(diff > TIMEOUT_AFTER_IDLE_INTERVAL) {
			connectedToEos = false;
			lastMessageRxTime = 0;
			timeoutPingSent = false;
			}

		// It could be the console is sitting idle. Send a ping once to
		// double check that it's still there, but only once after 2.5s have passed
		if(!timeoutPingSent && diff > PING_AFTER_IDLE_INTERVAL) {
			sendPing();
			}
		}

	// do all necessary updates
	updateFader(&fader1);
	updateFader(&fader2);
	updateFader(&fader3);
	updateFader(&fader4);
	updateFader(&fader5);
	updateFader(&fader6);
	}
