/**
 * Servo Board - Code for a 16 servo controller board <https://github.com/tirithen/servoboard>
 * Copyright (C) 2012  Fredrik Söderström <tirithen@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <avr/interrupt.h>
#include <avr/io.h>
#include <SPI.h>
#include <digitalWriteFast.h>

// SPI MOSI at pin 11
// SPI SCLK at pin 13
#define SPILATCHPIN 8 // SPI SS at pin 8

#define SERVOMINTIMEOUT 6200
#define SERVOMAXTIMEOUT 59000
#define SERVOCOUNT 16

#define DEBUG

#ifdef DEBUG
bool debug = false;
#endif

typedef struct
{
	// TODO: add calibration min/max values for control, position and load
	bool isEnabled; // TODO: implement
	uint16_t controlMask;
	//uint8_t positionPin; // TODO: implement
	//uint8_t loadPin; // TODO: implement
	uint16_t goal;
	uint16_t timeout;
	uint16_t minTimeout;
	uint16_t maxTimeout;
	//uint16_t latestPosition; // TODO: implement10000
	//uint16_t latestLoad; // TODO: implement
} ServoData_t;

typedef struct
{
	uint16_t controlMask;
	uint16_t timeout; // TODO: change to uint8_t if no more than 256 steps of resolution is possible
} ServoGroup_t;

ServoData_t servos[SERVOCOUNT];
ServoGroup_t servoGroups[SERVOCOUNT];
uint8_t servoOrder[SERVOCOUNT];

uint8_t servoGroupIterator = 0;
uint8_t servoGroupCount = 0;

String serialInputString = "";
bool serialInputStringComplete = false;
typedef struct
{
	String code;
	uint16_t arguments[2];
} SerialInstruction_t;
SerialInstruction_t serialInstruction;

bool timer2firstIteration = true;

void spi(uint16_t value) {
	// Select the IC to tell it to expect data
	digitalWriteFast(SPILATCHPIN, HIGH);
	// Send 8 bits, MSB first, pulsing the clock after each bit
	SPI.transfer(highByte(value));
	SPI.transfer(lowByte(value));
	// Lower the latch to apply the changes
	digitalWriteFast(SPILATCHPIN, LOW);
}

ISR(TIMER1_COMPA_vect) // This interrupt runs when it's time to stop the current servo group or stops this timer if all groups are done
{
	if(servoGroupIterator < servoGroupCount) {
		spi(servoGroups[servoGroupIterator].controlMask);
		OCR1A = servoGroups[servoGroupIterator].timeout;
		servoGroupIterator++;
	}
	else {
		// Disable timer1
		TCCR1B &= ~_BV(CS10); // Unset prescaler at 1
		TIMSK1 &= ~_BV(OCIE1A); // Disable timer 1 compare A interrupt
	}
}

ISR(TIMER2_COMPA_vect) // This interrupt will trigger every 10ms, half of the times (20ms) the servo pulses will be started
{
	// Slow down the speed of timer2 to half
	if(timer2firstIteration == true) {
		timer2firstIteration = false;
	}
	else {
		timer2firstIteration = true;
		servoGroupIterator = 0;

		spi(0); // Set all servo pins to low

		OCR1A = servoGroups[0].timeout;

		TIMSK1 |= _BV(OCIE1A); // Enable timer 1 compare A interrupt
		TCCR1B |= _BV(CS10); // Set prescaler at 1
	}
}

void calculateServoGroups()
{
	uint8_t i;
	uint8_t j;
	uint8_t currentGroupIndex = 0;
	uint16_t prevTimeout = 0;
	uint16_t deltaTimeout = 0;
	uint16_t prevMask = 0xffff;

	servoGroupCount = 0;

	for(i = 0; i < SERVOCOUNT; i++) {
		j = servoOrder[i];
		if(servos[j].isEnabled) {
			deltaTimeout = servos[j].timeout - prevTimeout;
			if(deltaTimeout > 0) {
				servoGroupCount++;
				currentGroupIndex = servoGroupCount - 1;
				servoGroups[currentGroupIndex].timeout = deltaTimeout;
				prevTimeout = servos[j].timeout;
			}

			prevMask = prevMask & ~servos[j].controlMask;
			servoGroups[currentGroupIndex].controlMask = 0xffff & ~prevMask;
		}
	}
}

void updateServoOrder()
{
	uint8_t i;
	uint8_t j;
	uint8_t tmp = 0;

	// TODO: refactor into a faster algorithm should be possible
	for(i = 0; i < SERVOCOUNT; i++) {
		for(j = 0; j < SERVOCOUNT; j++) {
			if(i != j && servos[servoOrder[j]].timeout > servos[servoOrder[i]].timeout) {
				tmp = servoOrder[j];
				servoOrder[j] = servoOrder[i];
				servoOrder[i] = tmp;
			}
	    }
	}
}

bool setServoGoal(uint8_t index, uint16_t goal, bool updateOrder)
{
	if(index < SERVOCOUNT && goal < 1000) {
		servos[index].goal = goal;
		servos[index].timeout = map(goal, 0, 999, servos[index].minTimeout, servos[index].maxTimeout);

		if(updateOrder) {
			updateServoOrder();
			calculateServoGroups();
		}

		return true;
	}
	else {
		return false;
	}
}

void setupServosAndServoGroups()
{
	uint8_t i;
	for(i = 0; i < SERVOCOUNT; i++) {
		// Setup servo group
		servoGroups[i].timeout = 0;
		servoGroups[i].controlMask = 0;
		servoOrder[i] = i;

		// Setup servo
		servos[i].isEnabled = true;
		servos[i].controlMask = 1 << i;
		servos[i].minTimeout = SERVOMINTIMEOUT;
		servos[i].maxTimeout = SERVOMAXTIMEOUT;
		setServoGoal(i, 511, false);
	}
}

void sendHelp()
{
	Serial.println("Input");
	Serial.println("-----");
	Serial.println("HE - Show this message");
	Serial.println("SS,<id 0-15>,<pos 0-999> - Set servo goal");
#ifdef DEBUG
	Serial.println("DE - Debug enable (only in debug)");
	Serial.println("DD - Debug disable (only in debug)");
#endif
	Serial.println("");
	Serial.println("Output");
	Serial.println("------");
	Serial.println("IN,<message> - Info message");
	Serial.println("ER,<code>,<message> - Error message");
	//Serial.println("SP,<pos> * 16 - All 16 servo positions");
	//Serial.println("SG,<goal> * 16 - All 16 servo goals");
	//Serial.println("SL,<load> * 16 - All 16 servo loads");
	Serial.println("");
}

void sendServoGoals()
{
	uint8_t i;

	Serial.print("SG");
	for(i = 0; i < SERVOCOUNT; i++) {
		Serial.print(",");
		Serial.print(servos[i].goal);
	}
	Serial.println("");
}

int stringToInt(String str)
{
	uint8_t length = str.length() + 1;
	char strCharArray[length];

	str.toCharArray(strCharArray, length);

	return atoi(strCharArray);
}

void serialInputStringToInstruction()
{
	uint8_t i = 0;
	uint8_t j = 0;
	String buffer = "";
	serialInstruction.code = "";
	serialInstruction.arguments[0] = 0;
	serialInstruction.arguments[1] = 0;

	while(serialInputString[i] != '\n')
	{
		if(serialInputString[i] == ',' && j < 2) {
			if(j == 0) { // If instruction name
				serialInstruction.code = buffer;
			}
			else {
				serialInstruction.arguments[j - 1] = stringToInt(buffer);
			}
			buffer = "";
			j++;
		}
		else {
			buffer += serialInputString[i];
		}

		i++;
	}

	if(j == 0) {
		serialInstruction.code = buffer;
	}
	else if(j < 3) {
		serialInstruction.arguments[j - 1] = stringToInt(buffer);
	}
}

void serialEvent() // Serial data handler
{
	while(Serial.available()) {
		char inputCharacter = (char)Serial.read();
		if(inputCharacter == '\n') {
			serialInputString += '\n';
			serialInputStringToInstruction();
			serialInputStringComplete = true;
		}
		else {
			serialInputString += inputCharacter;
		}
	}
}

void serialInputHandler()
{
	if(serialInputStringComplete == true) {
#ifdef DEBUG
		if(debug == true) {
			Serial.print("IN,Input recieved: ");
			Serial.print(serialInputString);
			Serial.print("IN,Input interpreted as: ");
			Serial.print(serialInstruction.code);
			Serial.print(" ");
			Serial.print(serialInstruction.arguments[0]);
			Serial.print(" ");
			Serial.println(serialInstruction.arguments[1]);
		}
#endif
		if(serialInstruction.code == "SS") {
			if(setServoGoal(
				serialInstruction.arguments[0],
				serialInstruction.arguments[1],
				true
			)) {
#ifdef DEBUG
				if(debug == true) {
					Serial.print("IN,Servo ");
					Serial.print(serialInstruction.arguments[0]);
					Serial.print(" goal set to ");
					Serial.println(serialInstruction.arguments[1]);
				}
#endif
			}
			else {
				Serial.println("ER,410,SS invalid args");
			}
		}
		else if(serialInstruction.code == "HE") {
			sendHelp();
		}
#ifdef DEBUG
		else if(serialInstruction.code == "DE") {
			debug = true;
			Serial.println("IN,Debug enabled");
		}
		else if(serialInstruction.code == "DD") {
			debug = false;
			Serial.println("IN,Debug disabled");
		}
#endif
		else {
			Serial.println("ER,400,Invalid input");
		}

		serialInputStringComplete = false;
		serialInputString = "";
	}
}

void setup()
{
	// Setup serial
	Serial.begin(115200);

	noInterrupts();

	// Timer1 setup
	TIMSK1 &= ~(_BV(ICIE1) | _BV(TOIE1) | _BV(OCIE1A) | _BV(OCIE1B));
	TCCR1A = 0;
	TCCR1B = 0;
	TCNT1 = 0;
	TIMSK1 = 0;
	OCR1A = 0;
	TCCR1B |= _BV(WGM12);	// CTC mode
	TCCR1B |= _BV(CS10); // Set prescaler at 1
	TIMSK1 |= _BV(OCIE1A);


	// Timer2 setup
	TIMSK2 &= ~(_BV(TOIE2) | _BV(OCIE2A) | _BV(OCIE2B));
	TCCR2A = 0;
	TCCR2B = 0;
	TCNT2 = 0;
	TIMSK2 = 0;
	OCR2A = 156;			// 99.52hz
	TCCR2A |= _BV(WGM21);	// CTC mode
	TCCR2B |= _BV(CS20) | _BV(CS21) | _BV(CS22); // Set prescaler at 1024
	TIMSK2 |= _BV(OCIE2A);	// Enable timer 2 compare A interrupt

	// Setup SPI
	pinModeFast(SPILATCHPIN, OUTPUT);
	SPI.setBitOrder(MSBFIRST); // Transmit most significant bit first when sending data with SPI
	SPI.setClockDivider(SPI_CLOCK_DIV2);
	SPI.begin();

	// Setup the servo pin/mask configurations
	// TODO: setup up servos
	setupServosAndServoGroups();
	// TODO: read calibration data
	// TODO: read initial servo goals from eeprom or set to some standard value
	updateServoOrder();
	calculateServoGroups();

	Serial.println("IN,Servo Board started. Send \"HE\" and \\n for help");

	interrupts();
}

void loop() {
	serialInputHandler();
	//sendServoGoals();
}
