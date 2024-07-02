#ifndef TIMER_H_
#define TIMER_H_

#include "stm32f413xx.h"

#define ON 1
#define OFF 0

class Timer {
private:
	uint32_t timerStartTime;
	uint8_t timerState;
	uint32_t eventTime;
	uint8_t eventFlag;
	/*
	 0- Системный таймер(используетсяHAL_GetTick())
	 1- Другой таймер(необходимо инкрементировать переменную timerClassCounter в прерывании по переполнению этого таймера)
	static uint8_t timerSource;
	*/

public:
	static uint32_t timerClassCounter;
	Timer();
	Timer(uint32_t time);
	void start();
	void stop();
	void reset();
	void setEventTime(uint32_t time);
	uint8_t isEvent();
	uint8_t isOn();
};

#endif
