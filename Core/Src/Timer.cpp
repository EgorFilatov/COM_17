#include "Timer.h"

uint32_t Timer::timerClassCounter = 0;

Timer::Timer() {
	timerStartTime = 0;
	timerState = 0;
	eventTime = 0;
	eventFlag = 0;
}

Timer::Timer(uint32_t eventTime) {
	timerStartTime = 0;
	timerState = 0;
	this->eventTime = eventTime;
	eventFlag = 0;
}

void Timer::start() {
	timerState = ON;
	timerStartTime = timerClassCounter;
}
void Timer::stop() {
	timerState = OFF;
}

void Timer::reset() {
	timerStartTime = timerClassCounter;
}
void Timer::setEventTime(uint32_t time) {
	eventTime = time;
}
uint8_t Timer::isEvent() {
	if (timerState == ON) {
		if (timerClassCounter > timerStartTime) {
			if ((timerClassCounter - timerStartTime) > eventTime) {
				return 1;
			}
		} else {
			if ((0xFFFFFFFF - timerStartTime + timerClassCounter) > eventTime) {
				return 1;
			}
		}
	}
	return 0;
}

uint8_t Timer::isOn() {
	return timerState;
}

