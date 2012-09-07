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

uint8_t spiDataPin = 11; // SPI MOSI ; use this instead of pin 2
uint8_t spiClockPin = 13; // SPI SCLK ; use this instead of pin 3
uint8_t spiLatchPin = 4; // SPI SS

bool timer2firstIteration = true;

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

#define SERVOMINTIMEOUT 6200;
#define SERVOMAXTIMEOUT 59000;


#define SERVOCOUNT 16
ServoData_t servos[SERVOCOUNT];
ServoGroup_t servoGroups[SERVOCOUNT];
uint8_t servoOrder[SERVOCOUNT];

uint8_t servoGroupIterator = 0;
uint8_t servoGroupCount = 0;

void spi(uint16_t value) {
	// Select the IC to tell it to expect data
	digitalWriteFast(spiLatchPin, HIGH);
	// Send 8 bits, MSB first, pulsing the clock after each bit
	SPI.transfer(highByte(value));
	SPI.transfer(lowByte(value));
	// Lower the latch to apply the changes
	digitalWriteFast(spiLatchPin, LOW);
}

ISR(TIMER1_COMPA_vect)
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

void setServoGoal(uint8_t index, uint16_t goal, bool updateOrder)
{
	if(index < SERVOCOUNT) {
		servos[index].goal = goal;
		servos[index].timeout = map(goal, 0, 1023, servos[index].minTimeout, servos[index].maxTimeout);

		if(updateOrder) {
			updateServoOrder();
			calculateServoGroups();
		}
	}
}

void setInitialServoValues()
{
	for(uint8_t i = 0; i < SERVOCOUNT; i++) {
		servos[i].isEnabled = true;
		servos[i].controlMask = 1 << i;
		servos[i].minTimeout = SERVOMINTIMEOUT;
		servos[i].maxTimeout = SERVOMAXTIMEOUT;
		setServoGoal(i, 511, false);
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
	pinModeFast(spiLatchPin, OUTPUT);
	digitalWriteFast(spiLatchPin, LOW);
	SPI.setBitOrder(MSBFIRST); // Transmit most significant bit first when sending data with SPI
	SPI.setClockDivider(SPI_CLOCK_DIV2);
	SPI.begin();

	// Setup the servo pin/mask configurations
	// TODO: setup up servos
	uint8_t i;
	for(i = 0; i < SERVOCOUNT; i++) {
		servoGroups[i].timeout = 0;
		servoGroups[i].controlMask = 0;
		servoOrder[i] = i;
	}
	setInitialServoValues();
	// TODO: read calibration data
	// TODO: read initial servo goals from eeprom or set to some standard value
	updateServoOrder();
	calculateServoGroups();

	interrupts();
}

void loop()
{
}
