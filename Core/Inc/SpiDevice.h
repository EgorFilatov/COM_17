#include "stm32f413xx.h"

#ifndef SRC_SPIDEVICE_H_
#define SRC_SPIDEVICE_H_

#define READY 0
#define BUSY 1

enum DeviceState {
	NOT_AVAILIABLE,
	ACTIVE,
    DATA_CHANGED
};

enum DeviceType {
	NOT_DEFINED,
	I20_G4_SS,
	O16NO_G4T,
	O20NO_G2T
};

class SpiDevice {
private:
	/* ����� ��� �������:
	 0:		����������� �����
	 1:		��� �����
	 2-5:	������ */
	uint8_t rxBuff_0[6] { 0 };
	uint8_t rxBuff_1[6] { 0 };
	uint8_t *rxBuffPtr[2] { rxBuff_0, rxBuff_1 };
	uint8_t previousRxBuff[6] { 0 };


	/* ����� ��� ��������:
	 0:		����������� �����
	 1:		��� �����
	 2-5:	������ */
	uint8_t txBuff_0[6] { 0 };
	uint8_t txBuff_1[6] { 0 };
	uint8_t *txBuffPtr[2] { txBuff_0, txBuff_1 };

	uint8_t uartTxBuff_0[10] { 0x55, 0xAA, 0x05, 0, 0, 0, 0, 0, 0, 0 };
	uint8_t uartTxBuff_1[10] { 0x55, 0xAA, 0x05, 0, 0, 0, 0, 0, 0, 0 };
	uint8_t *uartTxBuffPtr[2] { uartTxBuff_0, uartTxBuff_1 };

	uint8_t previousUartRxBuff[10] { 0 };

	uint8_t index;
	uint8_t uartSendNeeded;
	DeviceType type;
	GPIO_TypeDef *csPort;
	uint8_t csPin;

	static uint8_t numOfDevices;
	static uint8_t currentDeviceIndex;
	static uint8_t rxBuffState[2];
	static uint8_t txBuffState[2];
	static uint8_t uartTxBuffState[2];


public:
	SpiDevice();
	void setCS(GPIO_TypeDef *port, uint8_t pin);

	uint8_t* getTxBuffPtr(uint8_t buffIndex);
	uint8_t* getRxBuffPtr(uint8_t buffIndex);
	void setTxBuff(uint8_t *data, uint8_t buffIndex);
	void setTxBuffData(uint8_t *data, uint8_t buffIndex);

	uint8_t* getUartTxBuffPtr(uint8_t buffIndex);
	void setUartTxBuff(uint8_t uartBuffIndex, uint8_t spiBuffIndex);

	uint8_t isUartRxBuffChanged(uint8_t *uartRxBuff);

	void select();
	void deselect();


	void calculateTxChecksum(uint8_t buffIndex);
	uint8_t verifyRxChecksum(uint8_t buffIndex);
	uint8_t isRxBuffChanged(uint8_t buffIndex);

	static uint8_t getCurrentDeviceIndex();
	static void setCurrentDeviceIndex(uint8_t index);
	static void increaseCurrentDeviceIndex();
	static void decreaseCurrentDeviceIndex();

	static void setRxBuffState(uint8_t buffIndex, uint8_t state);
	static uint8_t getRxBuffState(uint8_t buffIndex);

	static void setTxBuffState(uint8_t buffIndex, uint8_t state);
	static uint8_t getTxBuffState(uint8_t buffIndex);

	static uint8_t getNumOfDevices();

	uint8_t isUartSendNeeded();
	void setUartSendNeeded(uint8_t state);

	DeviceType getType();
	void setType(DeviceType type);
};

#endif
