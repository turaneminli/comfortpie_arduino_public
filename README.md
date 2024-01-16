# Comfort Pie Arduino firmware

This repository represents the Adunio code for the Comfort Pie project to obtain data from the IoT devices in AIPL building of Universite de Lorraine.
The codebase is fully contributed by Daniel Gonzalez ([GitHub](https://github.com/DanielGonzalezArango)).

## Configuration

Add MQTT and WiFi credentials to the code:

- CollectingAndSending_MQTT.ino
```arduino
const char *mqttUser = "<username>";
const char *mqttPassword = "<password>";
```

- private_info.h
```arduino
#define SECRET_SSID "<ssid>"
#define SECRET_PASS "<password>"
```
