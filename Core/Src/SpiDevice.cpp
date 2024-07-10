#include <SpiDevice.h>

uint8_t SpiDevice::numOfDevices = 0;
uint8_t SpiDevice::currentDeviceIndex = 0;
uint8_t SpiDevice::rxBuffState[2] { READY };
uint8_t SpiDevice::txBuffState[2] { READY };

SpiDevice::SpiDevice() {
	index = numOfDevices;
	++numOfDevices;
	uartSendNeeded = 0;
	type = NOT_DEFINED;
	csPort = GPIOA;
	csPin = 0;
}

void SpiDevice::setCS(GPIO_TypeDef *port, uint8_t pin) {
	csPort = port;
	csPin = pin;
}

uint8_t* SpiDevice::getTxBuffPtr(uint8_t buffIndex) {
	return txBuffPtr[buffIndex];
}

uint8_t* SpiDevice::getRxBuffPtr(uint8_t buffIndex) {
	return rxBuffPtr[buffIndex];
}


void SpiDevice::setTxBuff(uint8_t *data, uint8_t buffIndex) {
	for (int i = 0; i < 6; ++i) {
		txBuffPtr[buffIndex][i] = data[i];
	}
}

void SpiDevice::setTxBuffData(uint8_t *data, uint8_t buffIndex) {
	for (int i = 0; i < 4; ++i) {
		txBuffPtr[buffIndex][i + 2] = data[i];
	}
}


void SpiDevice::select() {
	csPort->BSRR |= (1 << (csPin + 16));
}

void SpiDevice::deselect() {
	csPort->BSRR |= (1 << csPin);
}

void SpiDevice::calculateTxChecksum(uint8_t buffIndex) {
	txBuffPtr[buffIndex][0] = txBuffPtr[buffIndex][1] + txBuffPtr[buffIndex][2]
			+ txBuffPtr[buffIndex][3] + txBuffPtr[buffIndex][4]
			+ txBuffPtr[buffIndex][5];
}

uint8_t SpiDevice::verifyRxChecksum(uint8_t buffIndex) {
	uint8_t sum { 0 };
	for (uint8_t i = 1; i < 6; ++i) {
		sum += rxBuffPtr[buffIndex][i];
	}
	return (sum == rxBuffPtr[buffIndex][0]) ? 1 : 0;
}

uint8_t SpiDevice::isRxBuffChanged(uint8_t buffIndex) {
	uint8_t rxDataChanged { 0 };
	for (uint8_t i = 0; i < 6; ++i) {
		if (rxBuffPtr[buffIndex][i] != previousRxBuff[i]) {
			uartSendNeeded = 1;
			rxDataChanged = 1;
		}
		previousRxBuff[i] = rxBuffPtr[buffIndex][i];
	}
	return rxDataChanged;
}

uint8_t SpiDevice::getCurrentDeviceIndex() {
	return currentDeviceIndex;
}
void SpiDevice::setCurrentDeviceIndex(uint8_t index) {
	currentDeviceIndex = index;
}
void SpiDevice::increaseCurrentDeviceIndex() {
	++currentDeviceIndex;
}
void SpiDevice::decreaseCurrentDeviceIndex() {
	--currentDeviceIndex;
}

void SpiDevice::setRxBuffState(uint8_t buffIndex, uint8_t state) {
	rxBuffState[buffIndex] = state;
}
uint8_t SpiDevice::getRxBuffState(uint8_t buffIndex) {
	 return rxBuffState[buffIndex];
}

void SpiDevice::setTxBuffState(uint8_t buffIndex, uint8_t state) {
	txBuffState[buffIndex] = state;
}
uint8_t SpiDevice::getTxBuffState(uint8_t buffIndex) {
	 return txBuffState[buffIndex];
}

uint8_t SpiDevice::getNumOfDevices() {
	return numOfDevices;
}

uint8_t SpiDevice::isUartSendNeeded() {
	return uartSendNeeded;
}
void SpiDevice::setUartSendNeeded(uint8_t state) {
	uartSendNeeded = state;
}

DeviceType SpiDevice::getType() {
	return type;
}
void SpiDevice::setType(DeviceType type) {
	this->type = type;
}

void SpiDevice::saveRxBuff(uint8_t buffIndex) {
	for (uint8_t i = 0; i < 6; ++i) {
		rxBuffSaved[i] = rxBuffPtr[buffIndex][i];
	}
}

uint8_t* SpiDevice::getSaved() {
	return rxBuffSaved;
}







