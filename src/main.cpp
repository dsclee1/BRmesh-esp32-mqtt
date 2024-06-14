#include "BLEDevice.h"
#include "BLEUtils.h"
#include "BLEServer.h"
#include "BLEBeacon.h"
#include <WiFi.h>
#include <ArduinoHA.h>
#include <String>
#include <vector>
#include <cstdio>

#include "FS.h"
#include "SPIFFS.h"
/* You only need to format SPIFFS the first time you run a
   test or else use the SPIFFS plugin to create a partition
   https://github.com/me-no-dev/arduino-esp32fs-plugin */
#define FORMAT_SPIFFS_IF_FAILED true

//////////////////////////////////////////////////////
//CONFIGURATION
//////////////////////////////////////////////////////
//IP Address of your MQTT Broker (probably your Home Assistant host)
#define MQTT_BROKER_ADDR IPAddress(192,168,0,1)
//Your WiFi SSID
#define WIFI_SSID "YOUR_SSID"
//Your Wifi Password
#define WIFI_PASS "YOUR_WIFI_PASS"
// UUID 1 128-Bit (may use linux tool uuidgen or random numbers via https://www.uuidgenerator.net/)
#define BEACON_UUID "a1885535-7e56-4c9c-ae19-796ce9864f3f"
//////////////////////////////////////////////////////
//END
//////////////////////////////////////////////////////

uint8_t my_key[4];
byte mac[6];
WiFiClient client;
HADevice device;
HAMqtt* mqtt;
BLEAdvertising* pAdvertising;
const uint8_t default_key[] = { 0x5e, 0x36, 0x7b, 0xc4 };
struct LightType {
  std::string name;
  uint8_t code[2];
};
std::vector<LightType> lightTypes = {
  {"Smart", {0x39, 0xae}}, // 44601 - AE39 - smart light
  {"RGBW" , {0xa1, 0xa8}}, // 43169 - A8A1 - RGBW light
  {"RGB"  , {0xa0, 0xa8}}, // 43168 - A8A0 - RGB light
};
#define BLESCAN_DURATION 5
struct LightDevice {
  BLEAdvertisedDevice device;
  uint8_t type[2];
  bool isRegistered = false;
  std::string id;
  HALight* light;
  std::string name;
  uint8_t number;
};
std::vector<LightDevice> myLights;
#define BUFFER_SIZE 200
char str_buffer[BUFFER_SIZE];
BLEScan* pBLEScan;
const int ledPin = 2;

bool doesStringMatchBytes(std::string str, const u_int8_t* bytes) {
  bool result = true;
  for (int i = 0; i < str.length(); i++) {
    if (str[i] != bytes[i]) {
      result = false;
      break;
    }
  }
  return result;
}

uint8_t SEND_SEQ = 0;
uint8_t SEND_COUNT = 1;
const uint8_t DEFAULT_BLE_FASTCON_ADDRESS[] = { 0xC1, 0xC2, 0xC3 };
const uint8_t addrLength = 3;

void dump(const uint8_t* data, int length)
{
  for (int i = 0; i < length; i++)
  {
    Serial.printf("%2.2X", data[i]);
  }
}

void dump(std::string str)
{
  for (int i = 0; i < str.size(); i++)
  {
    Serial.printf("%2.2X", str[i]);
  }
}

void readFile(fs::FS &fs, const char * path){
    Serial.printf("Reading file: %s\r\n", path);
    File file = fs.open(path);
    if(!file || file.isDirectory()){
        Serial.println("- failed to open file for reading");
        return;
    }
    Serial.println("- read from file:");
    while(file.available()){
        Serial.write(file.read());
    }
    file.close();
}
void writeFile(fs::FS &fs, const char * path, const char * message){
    Serial.printf("Writing file: %s\r\n", path);
    File file = fs.open(path, FILE_WRITE);
    if(!file){
        Serial.println("- failed to open file for writing");
        return;
    }
    if(file.print(message)){
        Serial.println("- file written");
    } else {
        Serial.println("- write failed");
    }
    file.close();
}

uint8_t package_ble_fastcon_body(int i, int i2, uint8_t sequence, uint8_t safe_key, int forward, const uint8_t* data, int length, const uint8_t* key, uint8_t*& payload)
{
  if (length > 12)
  {
    Serial.print("data too long");
    payload = 0;
    return 0;
  }
  uint8_t payloadLength = 4 + 12;
  payload = (uint8_t*)malloc(payloadLength);
  payload[0] = (i2 & 0b1111) << 0 | (i & 0b111) << 4 | (forward & 0xff) << 7;
  payload[1] = sequence & 0xff;
  payload[2] = safe_key;
  payload[3] = 0; // checksum
  // fill payload with zeros
  for (int j = 4; j < payloadLength; j++) payload[j]=0;
  memcpy(payload + 4, data, length);

  uint8_t checksum = 0;
  for (int j = 0; j < length + 4; j++)
  {
    if (j == 3) continue;
    checksum = (checksum + payload[j]) & 0xff;
  }
  payload[3] = checksum;
  for (int j = 0; j < 4; j++) {
    payload[j] = default_key[j & 3] ^ payload[j];
  }
  for (int j = 0; j < 12; j++) {
    payload[4 + j] = key[j & 3] ^ payload[4 + j];
  }
  return payloadLength;
}

uint8_t get_payload_with_inner_retry(int i, const uint8_t* data, int length, int i2, const uint8_t* key, int forward, uint8_t*& payload) {
  SEND_COUNT++;
  SEND_SEQ = SEND_COUNT;
  Serial.print("data: "); dump(data, length); Serial.print("\n");
  Serial.print("key: "); dump(key, 4); Serial.print("\n");
  Serial.printf("sequence: %d\n", SEND_SEQ);
  uint8_t safe_key = 0xff;
  bool hasKey = false;
  for (int i = 0; i < 4; i++) {
    if (key[i] != 0) {
      hasKey = true;
      break;
    }
  }
  if (hasKey) safe_key = key[3];
  uint8_t result = package_ble_fastcon_body(i, i2, SEND_SEQ, safe_key, forward, data, length, key, payload);
  if (!hasKey) {
    // set the data content to the default key
    for (int i = 4; i < 16; i++) {
      payload[i] = default_key[i & 3];
    }
  }
  return result;
}

void whiteningInit(uint8_t val, uint8_t* ctx)
{
  ctx[0] = 1;
  ctx[1] = (val >> 5) & 1;
  ctx[2] = (val >> 4) & 1;
  ctx[3] = (val >> 3) & 1;
  ctx[4] = (val >> 2) & 1;
  ctx[5] = (val >> 1) & 1;
  ctx[6] = val & 1;
}

void whiteningEncode(const uint8_t* data, int len, uint8_t* ctx, uint8_t* result)
{
  memcpy(result, data, len);
  for (int i = 0; i < len; i++) {
    int ctx3 = ctx[3];
    int ctx5 = ctx[5];
    int ctx6 = ctx[6];
    int ctx4 = ctx[4];
    int ctx52 = ctx5 ^ ctx[2];
    int ctx41 = ctx4 ^ ctx[1];
    int ctx63 = ctx6 ^ ctx3;
    int ctx630 = ctx63 ^ ctx[0];

    int c = result[i];
    result[i] = ((c & 0x80) ^ ((ctx52 ^ ctx6) << 7))
      + ((c & 0x40) ^ (ctx630 << 6))
      + ((c & 0x20) ^ (ctx41 << 5))
      + ((c & 0x10) ^ (ctx52 << 4))
      + ((c & 0x08) ^ (ctx63 << 3))
      + ((c & 0x04) ^ (ctx4 << 2))
      + ((c & 0x02) ^ (ctx5 << 1))
      + ((c & 0x01) ^ (ctx6 << 0));

    ctx[2] = ctx41;
    ctx[3] = ctx52;
    ctx[4] = ctx52 ^ ctx3;
    ctx[5] = ctx630 ^ ctx4;
    ctx[6] = ctx41 ^ ctx5;
    ctx[0] = ctx52 ^ ctx6;
    ctx[1] = ctx630;
  }
}

uint8_t reverse_8(uint8_t d)
{
  uint8_t result = 0;
  for (uint8_t k = 0; k < 8; k++) {
    result |= ((d >> k) & 1) << (7 - k);
  }
  return result;
}

uint16_t reverse_16(uint16_t d) {
  uint16_t result = 0;
  for (uint8_t k = 0; k < 16; k++) {
    result |= ((d >> k) & 1) << (15 - k);
  }
  return result;
}

uint16_t crc16(const uint8_t* addr, const uint8_t* data, uint8_t dataLength)
{
  uint16_t crc = 0xffff;
  for (int8_t i = addrLength - 1; i >= 0; i--)
  {
    crc ^= addr[i] << 8;
    for (uint8_t ii = 0; ii < 4; ii++) {
      uint16_t tmp = crc << 1;
      if ((crc & 0x8000) != 0) tmp ^= 0x1021;
      crc = tmp << 1;
      if ((tmp & 0x8000) != 0) crc ^= 0x1021;
    }
  }
  for (uint8_t i = 0; i < dataLength; i++) {
    crc ^= reverse_8(data[i]) << 8;
    for (uint8_t ii = 0; ii < 4; ii++) {
      uint16_t tmp = crc << 1;
      if ((crc & 0x8000) != 0) tmp ^= 0x1021;
      crc = tmp << 1;
      if ((tmp & 0x8000) != 0) crc ^= 0x1021;
    }
  }
  crc = ~reverse_16(crc) & 0xffff;
  return crc;
}

uint8_t get_rf_payload(const uint8_t* addr, const uint8_t* data, uint8_t dataLength, uint8_t*& rfPayload)
{
  uint8_t data_offset = 0x12;
  uint8_t inverse_offset = 0x0f;
  uint8_t result_data_size = data_offset + addrLength + dataLength+2;
  uint8_t* resultbuf = (uint8_t*)malloc(result_data_size);
  memset(resultbuf, 0, result_data_size);

  resultbuf[0x0f] = 0x71;
  resultbuf[0x10] = 0x0f;
  resultbuf[0x11] = 0x55;

  for (uint8_t j = 0; j < addrLength; j++) {
    resultbuf[data_offset + addrLength - j - 1] = addr[j];
  }

  for (int j = 0; j < dataLength; j++) {
    resultbuf[data_offset + addrLength + j] = data[j];
  }

  for (int i = inverse_offset; i < inverse_offset + addrLength + 3; i++) {
    resultbuf[i] = reverse_8(resultbuf[i]);
  }

  int crc = crc16(addr, data, dataLength);
  resultbuf[result_data_size-2] = crc & 0xff;
  resultbuf[result_data_size-1] = (crc >> 8) & 0xff;
  rfPayload = resultbuf;
  return result_data_size;
}

uint8_t do_generate_command(int i, const uint8_t* data, uint8_t length, const uint8_t* key, int forward, int use_default_adapter, int i2, uint8_t*& rfPayload)
{
  if (i2 < 0) i2 = 0;
  uint8_t* payload = 0;
  uint8_t payloadLength = get_payload_with_inner_retry(i, data, length, i2, key, forward, payload);
  uint8_t* rfPayloadTmp = 0;
  Serial.print("payload: "); dump(payload, payloadLength); Serial.print("\n");
  uint8_t rfPayloadLength = get_rf_payload(DEFAULT_BLE_FASTCON_ADDRESS, payload, payloadLength, rfPayloadTmp);
  free(payload);
  uint8_t ctx[7];
  whiteningInit(0x25, &ctx[0]);
  uint8_t* result = (uint8_t*)malloc(rfPayloadLength);
  whiteningEncode(rfPayloadTmp, rfPayloadLength, ctx, result);
  rfPayload = (uint8_t*)malloc(rfPayloadLength-15);
  memcpy(rfPayload, result + 15, rfPayloadLength - 15);
  Serial.print("rf payload: "); dump(rfPayload, rfPayloadLength-15); Serial.print("\n");
  free(result);
  free(rfPayloadTmp);
  return rfPayloadLength-15;
}

std::string getServiceData(uint8_t rfPayloadLength, uint8_t* rfPayload)
{
  uint8_t ble_adv_data[] = { 0x02, 0x01, 0x1A, 0x1B, 0xFF, 0xF0, 0xFF };
  uint8_t* advPacket = (uint8_t*)malloc(rfPayloadLength + sizeof(ble_adv_data));
  memcpy(advPacket, ble_adv_data, sizeof(ble_adv_data));
  memcpy(advPacket + sizeof(ble_adv_data), rfPayload, rfPayloadLength);
  Serial.print("send: "); dump(advPacket, rfPayloadLength + sizeof(ble_adv_data)); Serial.print("\n");
  uint8_t dataLength = rfPayloadLength + sizeof(ble_adv_data);
  std::string serviceData = "";
  serviceData += (char)(dataLength-4);
  for (int i = 4; i < dataLength; i++) serviceData += (char)advPacket[i];
  free(advPacket);
  return serviceData;
}

void single_control(const uint8_t* key, const uint8_t* data)
{
  uint8_t* rfPayload = 0;
  uint8_t rfPayloadLength = do_generate_command(5, data, 8, key, true /* forward */, true /* use_default_adapter */, 0, rfPayload);
  std::string serviceData = getServiceData(rfPayloadLength, rfPayload);
  BLEAdvertisementData oAdvertisementData = BLEAdvertisementData();
  oAdvertisementData.setFlags(0x04); // BR_EDR_NOT_SUPPORTED 0x04
  oAdvertisementData.addData(serviceData);
  pAdvertising->setAdvertisementData(oAdvertisementData);
  pAdvertising->setMinInterval(50);
  pAdvertising->setMaxInterval(50);
  pAdvertising->start();
  delay(250);
  pAdvertising->stop();
  Serial.println("");
}

LightDevice getLight(std::string id)
{
  for (int i = 0; i < myLights.size(); i++) {
    if (myLights[i].id == id) return myLights[i];
  }
  throw std::runtime_error("Light not found");
}

void onStateCommand(bool state, HALight* sender)
{
  Serial.print("Light: ");
  Serial.println(sender->getName());
  Serial.print("ID: ");
  Serial.println(sender->uniqueId());
  Serial.print("State: ");
  Serial.println(state);
  LightDevice light = getLight(sender->uniqueId());
  uint8_t data[] = { 0x22, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  data[1] = light.number;
  if (state) data[2] = 0x80;
  single_control(my_key, data);
  sender->setState(state); // report state back to the Home Assistant
}

void onBrightnessCommand(uint8_t brightness, HALight* sender)
{
  Serial.print("Light: ");
  Serial.println(sender->getName());
  Serial.print("ID: ");
  Serial.println(sender->uniqueId());
  Serial.print("Brightness: ");
  Serial.println(brightness);
  LightDevice light = getLight(sender->uniqueId());
  uint8_t data[] = { 0x22, 0x00, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00 };
  data[1] = light.number;
  data[2] = brightness & 127;
  single_control(my_key, data);
  sender->setBrightness(brightness); // report brightness back to the Home Assistant
}

void onColorTemperatureCommand(uint16_t temperature, HALight* sender)
{
  Serial.print("Light: ");
  Serial.println(sender->getName());
  Serial.print("ID: ");
  Serial.println(sender->uniqueId());
  Serial.print("Color temperature: ");
  Serial.println(temperature);
  LightDevice light = getLight(sender->uniqueId());
  uint8_t data[] = { 0x72, 0x00, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00 };
  data[1] = light.number;
  data[2] = sender->getCurrentBrightness() & 127;
  data[6] = temperature & 127;
  data[7] = (temperature >> 8) & 127;
  single_control(my_key, data);
  sender->setColorTemperature(temperature); // report color temperature back to the Home Assistant
}

void onRGBColorCommand(HALight::RGBColor color, HALight* sender)
{
  Serial.print("Light: ");
  Serial.println(sender->getName());
  Serial.print("ID: ");
  Serial.println(sender->uniqueId());
  Serial.print("Red: ");
  Serial.println(color.red);
  Serial.print("Green: ");
  Serial.println(color.green);
  Serial.print("Blue: ");
  Serial.println(color.blue);
  LightDevice light = getLight(sender->uniqueId());
  uint8_t data[] = { 0x72, 0x00, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00 };
  data[1] = light.number;
  data[2] = sender->getCurrentBrightness() & 127;
  data[3] = color.blue;
  data[4] = color.red;
  data[5] = color.green;
  single_control(my_key, data);
  sender->setRGBColor(color); // report color back to the Home Assistant
}

class AddDeviceCallback: public BLEAdvertisedDeviceCallbacks
{
  void onResult(BLEAdvertisedDevice foundDevice)
  {
    std::string address = foundDevice.getAddress().toString();
    std::string name = foundDevice.getName();
    std::string mData = foundDevice.getManufacturerData();
    int rssi = foundDevice.getRSSI();
    Serial.print("BLE Device found -> Address: ");
    Serial.print(address.c_str());
    Serial.print(", Name: ");
    Serial.print(name.c_str());
    Serial.print(", RSSI: ");
    Serial.print(rssi);
    Serial.print(", Manufacturer Data: ");
    dump(mData);
    // check the device is a light, and using the default key
    // check response
    if (mData.size() == 18) {
      std::string type = mData.substr(12,2);
      std::string key = mData.substr(14,4);
      bool knownType = false;
      for (int i = 0; i < lightTypes.size(); i++) {
        if (doesStringMatchBytes(type, lightTypes[i].code)) {
          Serial.printf(", It's a %s light!", lightTypes[i].name.c_str());
          knownType = true;
          break;
        }
      }
      if (knownType && doesStringMatchBytes(key, default_key)) {
        Serial.print(", Using the default key!");
        bool alreadyKnown = false;
        for (int i = 0; i < myLights.size(); i++) {
          if (myLights[i].device.getAddress().toString() == foundDevice.getAddress().toString()) {
            // we already know about this device, so ignore it
            alreadyKnown = true;
            break;
          }
        }
        if (alreadyKnown == false) {
          Serial.print(", Stored it!");
          LightDevice light;
          light.device = foundDevice;
          light.type[0] = type[0];
          light.type[1] = type[1];
          myLights.push_back(light);
        }
      }
    }
    Serial.println("");
  }
};

class AddLightCallback: public BLEAdvertisedDeviceCallbacks
{
  void onResult(BLEAdvertisedDevice foundDevice)
  {
    std::string address = foundDevice.getAddress().toString();
    std::string name = foundDevice.getName();
    std::string mData = foundDevice.getManufacturerData();
    int rssi = foundDevice.getRSSI();
    Serial.print("BLE Device found -> Address: ");
    Serial.print(address.c_str());
    Serial.print(", Name: ");
    Serial.print(name.c_str());
    Serial.print(", RSSI: ");
    Serial.print(rssi);
    Serial.print(", Manufacturer Data: ");
    dump(mData);
    // check response
    if (mData.size() == 18) {
      for (int i = 0; i < myLights.size(); i++) {
        if (myLights[i].device.getAddress().toString() == foundDevice.getAddress().toString()) {
          Serial.print(", It's one of our lights!");
          // check it's not using the default key
          std::string key = mData.substr(14,4);
          if (doesStringMatchBytes(key, default_key)) {
            Serial.print(", still using the default key (ignore)");
          } else {
            Serial.print(", with the new key");
            if (myLights[i].isRegistered) {
              Serial.print(", but it's already registered!");
            } else {
              myLights[i].isRegistered = true;
              // get the light number from the light itself
              uint8_t cleanManufacturerData[12];
              uint8_t* manufacturerData = (uint8_t*)foundDevice.getManufacturerData().substr(2).c_str();
              // use the key to clean it
              for (int j = 0; j < 12; j++) {
                cleanManufacturerData[j] = my_key[j & 3] ^ manufacturerData[4 + j];
              }
              Serial.print(", clean manufacturer data: "); dump(cleanManufacturerData, 12);
              myLights[i].number = cleanManufacturerData[1];
              myLights[i].id = myLights[i].device.getAddress().toString().substr(3,2) + myLights[i].device.getAddress().toString().substr(0,2);
              myLights[i].name = "Light_" + myLights[i].id;
              // enable features based on type
              std::string typeName = "";
              for (int j = 0; j < lightTypes.size(); j++) {
                if (myLights[i].type[0] == lightTypes[j].code[0] && myLights[i].type[1] == lightTypes[j].code[1]) {
                  typeName = lightTypes[j].name;
                  break;
                }
              }
              // create the HA object
              if (typeName == "RGBW") {
                HALight* light = new HALight(myLights[i].id.c_str(), HALight::BrightnessFeature | HALight::ColorTemperatureFeature | HALight::RGBFeature);
                light->setName(myLights[i].name.c_str());
                light->onStateCommand(onStateCommand);
                light->onBrightnessCommand(onBrightnessCommand);
                light->onColorTemperatureCommand(onColorTemperatureCommand);
                light->onRGBColorCommand(onRGBColorCommand);
                light->setBrightnessScale(127);
                light->setBrightness(127);
                myLights[i].light = light;
              } else if (typeName == "RGB") {
                HALight* light = new HALight(myLights[i].id.c_str(), HALight::BrightnessFeature | HALight::RGBFeature);
                light->setName(myLights[i].name.c_str());
                light->onStateCommand(onStateCommand);
                light->onBrightnessCommand(onBrightnessCommand);
                light->onRGBColorCommand(onRGBColorCommand);
                light->setBrightnessScale(127);
                light->setBrightness(127);
                myLights[i].light = light;
              } else {
                // "Smart" - no additional features
                HALight* light = new HALight(myLights[i].id.c_str());
                light->setName(myLights[i].name.c_str());
                light->onStateCommand(onStateCommand);
                myLights[i].light = light;
              }
              Serial.printf(", created light ");
              Serial.printf(myLights[i].light->uniqueId());
            }
            Serial.println("");
          }
          break;
        }
      }
    }
    Serial.println("");
  }
};

void scan()
{
  Serial.println("Send wake command");
  uint8_t data[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  const uint8_t key[] = { 0x00, 0x00, 0x00, 0x00 };
  uint8_t* rfPayload = 0;
  uint8_t rfPayloadLength = do_generate_command(0, data, 6, key, false, true, 0, rfPayload);
  std::string serviceData = getServiceData(rfPayloadLength, rfPayload);
  BLEAdvertisementData oAdvertisementData = BLEAdvertisementData();
  oAdvertisementData.setFlags(0x04); // BR_EDR_NOT_SUPPORTED 0x04
  oAdvertisementData.addData(serviceData);
  pAdvertising->setAdvertisementData(oAdvertisementData);
  pAdvertising->start();
  Serial.println("Scan for lights");
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new AddDeviceCallback());
  pBLEScan->setInterval(500);
  pBLEScan->setWindow(500);
  pBLEScan->setActiveScan(true);
  pBLEScan->start(BLESCAN_DURATION, false);
  pAdvertising->stop();
}

void addLight(uint8_t lightNumber, LightDevice light)
{
  uint8_t data[12];
  std::string lightMac = light.device.getManufacturerData().substr(6, 6);
  Serial.printf("Setting key on light %d, MAC: ", lightNumber); dump(lightMac); Serial.print("\n");
  Serial.print("new key: "); dump(my_key, 4); Serial.print("\n");
  for (int i = 0; i < 6; i++) data[i] = (uint8_t)lightMac[i]; // mac address
  data[6] = lightNumber; // light id - we're requesting that it's set to this
  data[7] = 0x01; // group id
  data[8] = my_key[0];
  data[9] = my_key[1];
  data[10] = my_key[2];
  data[11] = my_key[3];
  uint8_t* rfPayload = 0;
  uint8_t rfPayloadLength = do_generate_command(2, data, 12, default_key, false, true, 0, rfPayload);
  std::string serviceData = getServiceData(rfPayloadLength, rfPayload);
  BLEAdvertisementData oAdvertisementData = BLEAdvertisementData();
  oAdvertisementData.setFlags(0x04);
  oAdvertisementData.addData(serviceData);
  pAdvertising->setAdvertisementData(oAdvertisementData);
  pAdvertising->setMinInterval(50);
  pAdvertising->setMaxInterval(50);
  pAdvertising->start();
  // get light response
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new AddLightCallback());
  pBLEScan->setInterval(50);
  pBLEScan->setWindow(50);
  pBLEScan->setActiveScan(true);
  pBLEScan->start(1);
  pAdvertising->stop();
}

void addLights()
{
  scan();
  // wait before adding lights, as they seem to need a brief pause after the scan
  delay(1000);
  for (int i = 0; i < myLights.size(); i++) {
    addLight(i + 1, myLights[i]);
  }
  // file ops go here? I think...
  writeFile(SPIFFS, "/myLights.txt", myLights);
  //appendFile(SPIFFS, "/hello.txt", "World!\r\n");
  //readFile(SPIFFS, "/hello.txt");
}

void setup() {

  pinMode (ledPin, OUTPUT);
  // turn on to show we're still in setup (and are adding for lights)
  digitalWrite (ledPin, HIGH);
  Serial.begin(115200);
  // setup files
  if(!SPIFFS.begin(FORMAT_SPIFFS_IF_FAILED)){
    Serial.println("SPIFFS Mount Failed");
    return;
  }
  // create new key
  uint32_t new_key = esp_random();
  my_key[0] = new_key & 0xFF;
  my_key[1] = (new_key >> 8) & 0xFF;
  my_key[2] = (new_key >> 16) & 0xFF;
  my_key[3] = (new_key >> 24) & 0xFF;
  WiFi.macAddress(mac);
  device.setUniqueId(mac, sizeof(mac));
  device.setName("BRMesh");
  device.setManufacturer("BRMesh");
  device.setModel("BRMesh");
  mqtt = new HAMqtt(client, device);
  Serial.printf("ESP32 MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
  // Create the BLE Device
  BLEDevice::init("ESP32 as iBeacon");
  pAdvertising = BLEDevice::getAdvertising();
  BLEDevice::startAdvertising();
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500); // waiting for the connection
    Serial.println("Connecting to WiFi...");
  }
  // add the lights
  addLights();
  // print the added lights with their IDs
  for (int i = 0; i < myLights.size(); i++) {
    if (myLights[i].isRegistered) {
      Serial.printf("Light %d - ", myLights[i].number);
      Serial.println(myLights[i].light->uniqueId());
    }
  }
  // finished adding lights
  digitalWrite (ledPin, LOW);
  Serial.println("Starting MQTT");
  mqtt->begin(MQTT_BROKER_ADDR, 1883);
  Serial.println("Ready");
}

void loop()
{
  mqtt->loop();
}

