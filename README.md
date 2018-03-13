# CfOnlinestatus
Remotely reading the status of an EV charging station and sending it to an endpoint. The project is released under the MIT license (see LICENSE.txt or [online info about MIT license](https://choosealicense.com/licenses/mit/))

This it the [main development thread (in German)](https://www.goingelectric.de/forum/goingelectric-crowdfunding/neues-projekt-onlinestatus-fuer-crowdfunding-ladepunkte-t29325.html), but we understand English as well. 

[Slack Chat](https://cfonlinestatus.slack.com)

## Requirements
To use this software, you will need
- an EV charging station with a few readable sensors. Currently supported sensors are
  - S0 impulse counter (for an electricity meter that supports DIN EN 62053-31)
  - digital input pins (high- and low-active, with or without pull-up/pull-down)
  - analog input pins (for CP-PE voltage, only 8 bit precision is used -- you can define 3 voltage thresholds depending on used voltage divider for standby, vehicle detected and ready charging)
  - HC-SR04 ultrasound distance sensors
- a compatible microcontroller platform (currently ESP8266 or Arduino)
- an endpoint to receive the data output
  - serial interface
  - MQTT server
  - (other endpoints may be implemented in the future)
- a way to connect to a network (if you don't exclusively use serial output)
  - WiFi (currently ESP8266 only)
  - Ethernet (currently not implemented)
  - LoRaWAN (currently not implemented)
  - GSM (currently not implemented)
  
## Quickstart
- Download or clone the repository
- Edit sketch/cfos_config.h to suit your needs
- In your Arduino IDE, add the correct board in the board manager (e.g. ESP8266) and any needed libraries (e.g. PubSubClient for MQTT)
- Compile and upload the sketch to your hardware
- Profit! (hopefully)

## Serial output
You can configure the interval between two serial outputs. Serial output begins with  
`id_<chargepoint_id>:<ms_since_controller_start>:<newline \n>`  
Example: `id_MusterstadtGoethestr12:480068:`

Serial format for S0 is `s0_<pin_name>:<ms_between_last_2_impulses>:<impulses_in_previous_update_timeframe>:<seconds_since_last_impulse>:<watt_in_previous>:<newline \n>`

S0 example:
```
s0_CounterA:300:6:0:1440:  // s0 pin CounterA had 300 ms between the last two impulses, 6 impulses in the previous update timeframe and 0 seconds since the last impulse for a power of 1440W in the previous update timeframe
s0_CounterB:465:0:20:0: // s0 pin CounterB had 465 ms between the last two impulses, 0 impulses in the previous update timeframe and 20 seconds since the last impulse for a power of 0W in the previous update timeframe
```

Serial format for digital input: `di_<pin_name>:<status on or off>:<newline \n>`

Digital input example:
```
di_SpaceOccupied:on:
di_ContactorOn:off:
```

Serial format for analog input: `ai_<pin_name>:<analog_value_as_uint8>:<low, mid or high>:<newline \n>`

Analog input example (ready charging means <= off_value, standby means > on_value, vehicle detected means in between the two):
```
ai_PPVoltage:100:vehicle detected:
ai_PPVoltage:80:ready charging:
ai_PPVoltage:220:standby:
```

Serial format for HC-SR04 ultrasound input: `us_<sensor_name>:<delay_in_us_as_uint32>:<distance_in_cm_as_uint32>:<newline \n>`

Ultrasound example (If a timeout occurs, the printed delay and distance value is 0)
```
us_CarDistance:5800:100: // 1m distance
us_CarDistance:0:0:      // nothing in range (timeout occured)
```

## MQTT Output
With an active network/internet connection, CfOnlinestatus can send MQTT messages in a configurable interval. CfOnlinestatus uses its configured chargepoint_id as MQTT client id. Every published message begins with the topic `CFOS/<id>/`.

The following MQTT topics are being published. All messages (even with numeric values) consist of a string. In this example, the chargepoint_id is `MusterCF`:
- `CFOS/MusterCF/status` is the only retained message with values `online` (published after (re-)connecting) and `offline` (published by the LWT feature of MQTT after losing connection)
- `CFOS/MusterCF/s0_CounterA/lastspan`: the number of milliseconds between the last 2 impulses
- `CFOS/MusterCF/s0_CounterA/impulses_timeframe`: the number of impulses in the last full update timespan (default: 15 seconds)
- `CFOS/MusterCF/s0_CounterA/secs_since_last_impulse`: the number of seconds since the last impulse was received
- `CFOS/MusterCF/s0_CounterA/power`: the power (in W) in the last full update timespan (default: 15 seconds)
- `CFOS/MusterCF/di_SpaceOccupied/status`: `on` or  `off`
- `CFOS/MusterCF/ai_PPVoltage/value`: The 8-bit unsigned value (0-255) of the analog conversion
- `CFOS/MusterCF/ai_PPVoltage/meaning`: `low`, `mid` or `high` (see serial output)
- `CFOS/MusterCF/us_CarDistance/duration_microsecs`: Ultrasound pulse duration in microseconds (0 when timed out)
- `CFOS/MusterCF/us_CarDistance/distance_cm`: Calculated object distance in cm (0 when timed out)
- `CFOS/MusterCF/us_CarDistance/object_detected`: `no` (when timed out) or `yes`

## MQTT Broker for GoingElectric Cf charging stations
Use the central MQTT Broker IP: 46.38.232.97 TCP Port: 1883 for GoingElectric Cf charging stations only. 
Please send your "chargepoint_id" by mail to request username and password from tho.walther@gmail.com.
