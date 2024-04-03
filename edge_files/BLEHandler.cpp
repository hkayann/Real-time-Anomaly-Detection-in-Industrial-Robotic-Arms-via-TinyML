#include "BLEHandler.h"

// DFU channels
BLEService dfuService("34c2e3b8-34aa-11eb-adc1-0242ac120002");
auto dfuInternalUuid = "34c2e3b9-34aa-11eb-adc1-0242ac120002";
auto dfuExternalUuid = "34c2e3ba-34aa-11eb-adc1-0242ac120002";
BLECharacteristic dfuInternalCharacteristic(dfuInternalUuid, BLEWrite, sizeof(DFUPacket), true);
BLECharacteristic dfuExternalCharacteristic(dfuExternalUuid, BLEWrite, sizeof(DFUPacket), true);

Stream* BLEHandler::_debug = NULL;
BLEHandler::BLEHandler() : _lastDfuPack(false)
{
}

BLEHandler::~BLEHandler()
{
}

// DFU channel
void BLEHandler::processDFUPacket(DFUType dfuType, BLECharacteristic characteristic) 
{
  uint8_t data[sizeof(DFUPacket)];
  characteristic.readValue(data, sizeof(data));
  if (_debug) {
    _debug->print("Size of data: ");
    _debug->println(sizeof(data));
  }
  dfuManager.processPacket(bleDFU, dfuType, data);

  if (data[0]) {
    //Last packet
    _lastDfuPack = true;
    dfuManager.closeDfu();
  }
}

void BLEHandler::receivedInternalDFU(BLEDevice central, BLECharacteristic characteristic)
{
  if (_debug) {
    _debug->println("receivedInternalDFU");
  }
  bleHandler.processDFUPacket(DFU_INTERNAL, characteristic);
}

void BLEHandler::receivedExternalDFU(BLEDevice central, BLECharacteristic characteristic)
{
  bleHandler.processDFUPacket(DFU_EXTERNAL, characteristic);
}

bool BLEHandler::begin()
{
  if (!BLE.begin()) {
    return false;
  }
  bleActive = true;
  // DFU channel
  BLE.setAdvertisedService(dfuService);
  dfuService.addCharacteristic(dfuInternalCharacteristic);
  dfuService.addCharacteristic(dfuExternalCharacteristic);
  BLE.addService(dfuService);
  dfuInternalCharacteristic.setEventHandler(BLEWritten, receivedInternalDFU);
  dfuExternalCharacteristic.setEventHandler(BLEWritten, receivedExternalDFU);
  return true;
}

void BLEHandler::update()
{
  BLE.poll();
}

void BLEHandler::poll(unsigned long timeout)
{
  BLE.poll(timeout);
}


void BLEHandler::end()
{
  bleActive = false;
  BLE.end();
}

void BLEHandler::debug(Stream &stream)
{
  _debug = &stream;
  BLE.debug(stream);
}

BLEHandler bleHandler;


