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

#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <SPI.h>
#include <digitalWriteFast.h>

#define DEBUG // Remove to disable debug code

#ifdef DEBUG
bool debugOutput = false;
#endif

// SPI MOSI at pin 11
// SPI SCLK at pin 13
#define SPI_PIN_LATCH 8 // SPI SS at pin 8

#define SERVO_MIN_TIMEOUT 6200
#define SERVO_MAX_TIMEOUT 59000
#define SERVO_COUNT 16
#define SERVO_MAX_GOAL 254

#define MAX_SERIAL_INSTRUCTION_LENGTH 18
#define INSTRUCTION_END_OF_INSTRUCTION 255

#ifdef DEBUG
#define INSTRUCTION_INPUT_DEBUG_ENABLE			0
#define INSTRUCTION_INPUT_DEBUG_DISABLE			1
#endif
#define INSTRUCTION_INPUT_SET_ID				2
#define INSTRUCTION_INPUT_SERVOS_GOAL			3
#define INSTRUCTION_INPUT_SERVOS_ENABLED		4
#define INSTRUCTION_INPUT_SERVOS_CALIBRATE		5

#define INSTRUCTION_OUTPUT_READY				0
#define INSTRUCTION_OUTPUT_INFO					1
#define INSTRUCTION_OUTPUT_DEBUG				2
#define INSTRUCTION_OUTPUT_ERROR				3
#define INSTRUCTION_OUTPUT_ALL_SERVOS_POSITION	4
#define INSTRUCTION_OUTPUT_ALL_SERVOS_LOAD		5

#define INSTRUCTION_ERROR_UNKNOWN				0
//#define INSTRUCTION_ERROR_NO_CONNECTION		1 // Client only error
#define INSTRUCTION_ERROR_INSTRUCTION_ILLEGAL	2
//#define INSTRUCTION_ERROR_RESPONSE_ILLEGAL	3 // Client only error
#define INSTRUCTION_ERROR_INVALID_BOARD_ID		4

typedef struct
{
	bool isEnabled;
	uint16_t controlMask;
	//uint8_t positionPin; // TODO: implement
	//uint8_t loadPin; // TODO: implement
	uint8_t goal;
	uint16_t timeout;
	uint16_t *minTimeout;
	uint16_t *maxTimeout;
	uint8_t position;
	uint8_t load;
} ServoData_t;

typedef struct
{
	uint16_t controlMask;
	uint16_t timeout; // TODO: change to uint8_t if no more than 256 steps of resolution is possible
} ServoGroup_t;

struct settings_t
{
	uint8_t boardId;
	uint16_t servosMinTimeout[SERVO_COUNT];
	uint16_t servosMaxTimeout[SERVO_COUNT];
} settings;

ServoData_t servos[SERVO_COUNT];
ServoGroup_t servoGroups[SERVO_COUNT];
uint8_t servoOrder[SERVO_COUNT];

uint8_t servoGroupIterator = 0;
uint8_t servoGroupCount = 0;

bool timer2firstIteration = true;

uint8_t serialInstruction[MAX_SERIAL_INSTRUCTION_LENGTH];

void spi(uint16_t value)
{
	// Select the IC to tell it to expect data
	digitalWriteFast(SPI_PIN_LATCH, HIGH);
	// Send 8 bits, MSB first, pulsing the clock after each bit
	SPI.transfer(highByte(value));
	SPI.transfer(lowByte(value));
	// Lower the latch to apply the changes
	digitalWriteFast(SPI_PIN_LATCH, LOW);
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

ISR(TIMER2_COMPA_vect)	// This interrupt will trigger every 10ms, half of the times (20ms)
{						// the servo pulses will be started
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

void saveSettings()
{
	eeprom_write_block((const void*)&settings, (void*)0, sizeof(settings));
}

void loadSettings()
{
	uint8_t i;

	eeprom_read_block((void*)&settings, (void*)0, sizeof(settings));

	for(i = 0; i < SERVO_COUNT; i++) { // Keep the servo max/min timeouts within reasonable limits
		servos[i].minTimeout = &settings.servosMinTimeout[i];
		servos[i].maxTimeout = &settings.servosMaxTimeout[i];

		if(
			settings.servosMinTimeout[i] < SERVO_MIN_TIMEOUT ||
			settings.servosMinTimeout[i] > SERVO_MAX_TIMEOUT
		) {
			settings.servosMinTimeout[i] = SERVO_MIN_TIMEOUT;
		}

		if(
			settings.servosMaxTimeout[i] > SERVO_MAX_TIMEOUT ||
			settings.servosMaxTimeout[i] < SERVO_MIN_TIMEOUT
		) {
			settings.servosMaxTimeout[i] = SERVO_MAX_TIMEOUT;
		}
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

	for(i = 0; i < SERVO_COUNT; i++) {
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
	for(i = 0; i < SERVO_COUNT; i++) {
		for(j = 0; j < SERVO_COUNT; j++) {
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
	if(index < SERVO_COUNT && goal <= SERVO_MAX_GOAL) {
		servos[index].goal = goal;
		servos[index].timeout = map(goal, 0, SERVO_MAX_GOAL, *servos[index].minTimeout, *servos[index].maxTimeout);

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
	for(i = 0; i < SERVO_COUNT; i++) {
		// Setup servo group
		servoGroups[i].timeout = 0;
		servoGroups[i].controlMask = 0;
		servoOrder[i] = i;

		// Setup servo
		servos[i].controlMask = 1 << i;
		setServoGoal(i, 0, false);
	}
}

int stringToInt(String str)
{
	uint8_t length = str.length() + 1;
	char strCharArray[length];

	str.toCharArray(strCharArray, length);

	return atoi(strCharArray);
}

void serialEvent() // Serial data handler
{
	uint8_t i = 0;
	while(Serial.available()) {
		serialInstruction[i] = (uint8_t) Serial.read();
		if(serialInstruction[i] == INSTRUCTION_END_OF_INSTRUCTION) {
			serialInputHandler();
			break;
		}
		i++;
	}
}

void serialInputHandler()
{
	uint8_t i;

	if(serialInstruction[0] >> 4 == settings.boardId) { // If the first 4 bits matches this board id, continiue
		serialInstruction[0] &= 0x00ff; // Remove the board id from the first byte, leave the instruction bits

		// Set servos goal
		if(serialInstruction[0] == INSTRUCTION_INPUT_SERVOS_GOAL) {
			i = 1;

			while(serialInstruction[i] != INSTRUCTION_END_OF_INSTRUCTION) {
				setServoGoal(i - 1, serialInstruction[i], false);
				i++;
			}

			updateServoOrder();
			calculateServoGroups();

#ifdef DEBUG
			if(debugOutput) {
				Serial.write((settings.boardId << 4) + INSTRUCTION_OUTPUT_DEBUG);
				Serial.write("Updated servos goal");
				Serial.write(INSTRUCTION_END_OF_INSTRUCTION);
			}
#endif
		}
		// Set enabled state for servos
		else if(serialInstruction[0] == INSTRUCTION_INPUT_SERVOS_ENABLED) {
			for(i = 0; i < SERVO_COUNT; i++) {
				if(((serialInstruction[1] >> i) & 1) == 1) {
					servos[i].isEnabled = true;
				}
				else {
					servos[i].isEnabled = false;
				}
			}

#ifdef DEBUG
			if(debugOutput) {
				Serial.write((settings.boardId << 4) + INSTRUCTION_OUTPUT_DEBUG);
				Serial.write("Enabled/disabled servos");
				Serial.write(INSTRUCTION_END_OF_INSTRUCTION);
			}
#endif
		}
		// Calibrate servos
		else if(serialInstruction[0] == INSTRUCTION_INPUT_SERVOS_CALIBRATE) {
			for(i = 0; i < SERVO_COUNT; i++) {
				if(((serialInstruction[1] >> i) & 1) == 1) {
					// TODO: Calibrate servo
				}
			}
#ifdef DEBUG
			if(debugOutput) {
				Serial.write((settings.boardId << 4) + INSTRUCTION_OUTPUT_DEBUG);
				Serial.write("Servo(s) calibrated");
				Serial.write(INSTRUCTION_END_OF_INSTRUCTION);
			}
#endif
		}
		// Set the board id
		else if(serialInstruction[0] == INSTRUCTION_INPUT_SET_ID) {
			if(serialInstruction[1] < 16) {
				if(settings.boardId != serialInstruction[1]) {
					settings.boardId = serialInstruction[1];
					saveSettings();
#ifdef DEBUG
					if(debugOutput) {
						Serial.write((settings.boardId << 4) + INSTRUCTION_OUTPUT_DEBUG);
						Serial.write("Servo board id updated");
						Serial.write(INSTRUCTION_END_OF_INSTRUCTION);
					}
#endif
				}
				else {
#ifdef DEBUG
					if(debugOutput) {
						Serial.write((settings.boardId << 4) + INSTRUCTION_OUTPUT_DEBUG);
						Serial.write("Servo board id given is already set");
						Serial.write(INSTRUCTION_END_OF_INSTRUCTION);
					}
#endif

				}
			}
			else { // Output invalid id
				Serial.write((settings.boardId << 4) + INSTRUCTION_OUTPUT_ERROR);
				Serial.write(INSTRUCTION_ERROR_INVALID_BOARD_ID);
				Serial.write(INSTRUCTION_END_OF_INSTRUCTION);
			}
		}
#ifdef DEBUG
		// Enable debug output
		else if(serialInstruction[0] == INSTRUCTION_INPUT_DEBUG_ENABLE) {
			debugOutput = true;
			Serial.write((settings.boardId << 4) + INSTRUCTION_OUTPUT_INFO);
			Serial.write("Debug output enabled");
			Serial.write(INSTRUCTION_END_OF_INSTRUCTION);
		}
		// Disable debug output
		else if(serialInstruction[0] == INSTRUCTION_INPUT_DEBUG_DISABLE) {
			debugOutput = false;
			Serial.write((settings.boardId << 4) + INSTRUCTION_OUTPUT_INFO);
			Serial.write("Debug output disabled");
			Serial.write(INSTRUCTION_END_OF_INSTRUCTION);
		}
#endif
		// Output illegal instruction error
		else {
			Serial.write((settings.boardId << 4) + INSTRUCTION_OUTPUT_ERROR);
			Serial.write(INSTRUCTION_ERROR_INSTRUCTION_ILLEGAL);
			Serial.write(INSTRUCTION_END_OF_INSTRUCTION);
		}
	}
}

void serialOutputServoPositions() // Output servo positions
{
	uint8_t i;
	Serial.write((settings.boardId << 4) + INSTRUCTION_OUTPUT_ALL_SERVOS_POSITION);
	for(i = 0; i < SERVO_COUNT; i++) {
		Serial.write(servos[i].position);
	}
	Serial.write(INSTRUCTION_END_OF_INSTRUCTION);
}

void serialOutputServoLoads() // Output servo loads
{
	uint8_t i;
	Serial.write((settings.boardId << 4) + INSTRUCTION_OUTPUT_ALL_SERVOS_LOAD);
	for(i = 0; i < SERVO_COUNT; i++) {
		Serial.write(servos[i].load);
	}
	Serial.write(INSTRUCTION_END_OF_INSTRUCTION);
}

void setup()
{
	// Setup serial
	Serial.begin(115200);

	noInterrupts();

	// Load settings
	loadSettings();

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
	pinModeFast(SPI_PIN_LATCH, OUTPUT);
	SPI.setBitOrder(MSBFIRST); // Transmit most significant bit first when sending data with SPI
	SPI.setClockDivider(SPI_CLOCK_DIV2);
	SPI.begin();

	// Setup the servo pin/mask configurations
	setupServosAndServoGroups();
	updateServoOrder();
	calculateServoGroups();

	// Wait before sending next time to give the client a chance to handle last message
	delay(100);

	// Output end of instruction byte to make sure listners are in sync
	// TODO: figure out if this is working
	Serial.write((settings.boardId << 4) + INSTRUCTION_OUTPUT_INFO);
	Serial.write("Servo Board starting...");
	Serial.write(INSTRUCTION_END_OF_INSTRUCTION);

	// Wait before sending next time to give the client a chance to handle last message
	delay(100);

	// Output servo board ready
	Serial.write((settings.boardId << 4) + INSTRUCTION_OUTPUT_READY);
	Serial.write(INSTRUCTION_END_OF_INSTRUCTION);

	interrupts();
}

void loop() {
	serialOutputServoPositions();
	serialOutputServoLoads();
}
