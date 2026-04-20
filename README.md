# TV lifting project with Mega2560 WiFi R3

This project controls a TV lift system using Mega2560 WiFi R3 with integrated ESP8266. Includes control of 6 relays for pistons and doors, reading of sensors for height and current of the TV, a web page for manual control and configuration times, and publication of data via MQTT for integration with home automation.

## Required Hardware Components

- Mega2560 WiFi R3 with integrated ESP8266
- 6 channel relay board
- 3 linear pistons (actuators) for TV lifting
- 2 Door pistons (front and rear)
- VL53L1X or VL53L0X height sensor on I2C
- ACS712 current sensor
- Appropriate nutrition

## Links

- Relay 1 (D2): Front door opening
- Relay 2 (D3): Front piston closing
- Relay 3 (D4): TV raised
- Relay 4 (D5): TV lowering
- Relay 5 (D6): Rear door opening
- Relay 6 (D7): Rear door closing
- VL53 height sensor: SDA on pin 20, SCL on pin 21, I2C address 0x29
- ACS712 current sensor: A0

## Operation Logic

### Opening (when TV on and minimum height)
1. Activate relay 1 for T1 ms (front opening)
2. Activate relay 5 for T5 ms (rear opening)
3. Activate relay 3 up to maximum height (TV rise)
4. Activate relay 2 for T2 ms (front closing)
5. Rear door remains open

### Closing (when TV is off)
1. Activate relay 1 for T1 ms (front opening)
2. Activate relay 4 until minimum height (TV lowering)
3. Activate relay 2 for T2 ms (front closing)
4. Activate relay 6 for T6 ms (rear closing)

## Software Configuration

1. Install the Arduino AVR and ESP8266 boards in the Arduino IDE
2. Install libraries: Adafruit_VL53L1X or Adafruit_VL53L0X, ESP8266WiFi, ESP8266WebServer, PubSubClient
3. Edit WiFi and MQTT configurations
4. Load mega2560_logic on the Mega and mega2560_esp on the integrated ESP8266

## Check

- **Web**: Access http://IP_ARDUINO for controls and timing settings
- **MQTT**: "open"/"close"/"stop" commands on topic command, status on topic status
- **Automatic**: Based on TV status and height
- **Sensor diagnostics**: the UI shows sensor status, model and detected I2C address

## Configurable times

All times are saved in EEPROM and configurable via web:
- T1: Front opening
- T2: Front closure
- T3: TV riser (not used directly, sensor based)
- T4: TV lowering (not used directly)
- T5: Rear opening
- T6: Rear closure

## Safety

- Automatic stop at height thresholds
- TV status control to avoid incorrect movements
- Possibility of manual stop
