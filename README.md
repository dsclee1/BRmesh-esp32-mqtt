
# BRmesh esp32 mqtt

An ESP32 MQTT Home Assistant implementation of the BRmesh app to control lights.

Automatically adds lights and makes them available via an MQTT broker.


## Acknowledgements

The great existing projects that this work is based off of:

 - [BRMesh_homeassistant by @millskyle](https://github.com/millskyle/BRMesh_homeassistant)
 - [brMeshMQTT by @ArcadeMachinist](https://github.com/ArcadeMachinist/brMeshMQTT)

## Installation

It's PlatformIO based, built via VSCode. Download the source and it flash to an ESP32 device using PlatformIO.

By default the ESP32 partitions will be too small, so I've also included the partition table layout, which can also be flashed using PlatformIO.

Set your WiFi details and MQTT Broker in main.cpp before flashing.

```c
//IP Address of your MQTT Broker (probably your Home Assistant host)
#define MQTT_BROKER_ADDR IPAddress(192,168,0,1)
//Your WiFi SSID
#define WIFI_SSID "YOUR_SSID"
//Your Wifi Password
#define WIFI_PASS "YOUR_WIFI_PASS"
```


    
## Usage

Turn off your lights.

Turn on the ESP32, if using an ESP32 Dev Module (like I am) the blue light will come on to show it's in scanning mode.

Turn on your lights.

The ESP32 sends an "alive" message to the lights, receives a response back from them, sends a new key (which makes each light flash), and they respond back to say they're set. These are then made available as MQTT devices (should be viewable on your broker by using https://github.com/thomasnordquist/MQTT-Explorer).

You're good to go!




## Bugs

I'm unable to test the "ColorTemperature" code properly as it's not a function that my lights have.

Adding lights has occasionally been flakey. I've tested this code on a group of 7 lights, for which it worked fine, but you'll have to see how you get on. Some of the polling times for the BLE Advertising frames might need adjustment.

## Contributing

Contributions are always welcome!

