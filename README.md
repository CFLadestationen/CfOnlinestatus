# CfOnlinestatus
Remotely reading the status of an EV charging station and sending it to an endpoint. The project is released under the MIT license (see LICENSE.txt or [online info about MIT license](https://choosealicense.com/licenses/mit/))

This it the [main development thread (in German)](https://www.goingelectric.de/forum/goingelectric-crowdfunding/neues-projekt-onlinestatus-fuer-crowdfunding-ladepunkte-t29325.html), but we understand English as well. We also have a [Slack Chat](https://cfonlinestatus.slack.com).

## Requirements
To use this software, you will need
- a compatible microcontroller platform (currently ESP8266 or Arduino). If you don't know what to use, buy the following:
  - Microcontroller board: [WEMOS D1 mini Pro](https://wiki.wemos.cc/products:d1:d1_mini_pro) (under 10$/10€)
  - Power supply: Any USB charger that provides a **stable** 500 mA or more, e.g. [MeanWell HDR-15-5](http://www.meanwell.com/productPdf.aspx?i=751)
  - Level shifter(s): The ESP8266 only supports 3.3V input, so you will need a [logic level converter](http://www.ebay.com/sch/?_nkw=logic%20level%20converter%203.3v) to connect 5V inputs
  - Distance sensor: If you want to detect a parking vehicle, you can use a [HC-SR04P sensor](http://www.ebay.com/sch/?_nkw=hc-sr04p)
  - External WiFi antenna: for better WiFi reception, get an [antenna with an u.fl connector](http://www.ebay.com/sch/?_nkw=u.fl%20antenna)
- an EV charging station with a few readable sensors. Currently supported sensors are
  - S0 impulse counter (for an electricity meter that supports DIN EN 62053-31)
  - digital input pins (high- and low-active, with or without pull-up/pull-down)
  - analog input pins (for CP-PE voltage, only 8 bit precision is used -- you can define voltage thresholds for the three possible states `standby`, (vehicle) `detected` and (vehicle) `charging`)
  - HC-SR04 ultrasound distance sensors
  - the serial output of a [SmartEVSE](https://github.com/SmartEVSE/smartevse) device (due to the needed fast SoftwareSerial, this feature is currently ESP8266 only)
- an endpoint to receive the data output
  - serial interface on a connected PC or other device
  - MQTT server
  - [TheThingsNetwork](https://www.thethingsnetwork.org) Application
  - (other endpoints may be implemented in the future)
  - in case of GoingElectric.de crowdfunding charging station you can use the central services (provided by [nextmove](https://nextmove.de/))
    - MQTT Broker IP: 46.38.232.97 TCP Port: 1883, please send your "chargepoint_id" by mail to request username and password from tho.walther@gmail.com
    - TheThingsNetwork application eui: 70B3D57ED000ABB5, please send your device eui by mail to request app key from tho.walther@gmail.com
- a way to connect to a network (if you don't exclusively use serial output)
  - WiFi (currently ESP8266 only)
  - Ethernet (currently Arduino only)
  - LoRaWAN TheThingsNetwork (currently Arduino only), location must be covered by [TheThingsNetwork Gateway](https://www.thethingsnetwork.org/map) (see [TTN Mapper)](https://ttnmapper.org/))
  - GSM (currently not implemented)
  
## Quickstart
- Download or clone the repository
- Edit sketch/cfos_config.h to suit your needs
- In your Arduino IDE, add the correct board in the board manager
  -  ESP8266
- In your Arduino IDE, add any needed additional libraries
  - PubSubClient for MQTT
  - TheThingsNetwork for LoRaWAN
- Compile and upload the sketch to your hardware
- Connect all input pins
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

Serial format for analog input: `ai_<pin_name>:<analog_value_as_uint8>:<charging, detected or standby>:<newline \n>`
**Caution**: Some controllers only have a limited input voltage range. Be sure to transform the signal so that it fits the analog input range.
Analog input example: `charging` means <= off_value (corresponds to EVSE state C or 6V), `standby` means > on_value (EVSE state A or 12V), `detected` means in between the two (EVSE state B or 9V)
```
ai_PPVoltage:100:detected:
ai_PPVoltage:80:charging:
ai_PPVoltage:220:standby:
```

Serial format for HC-SR04 ultrasound input: `us_<sensor_name>:<delay_in_us_as_uint32>:<distance_in_cm_as_uint32>:<newline \n>`

Ultrasound example (If a timeout occurs, the printed delay and distance value is 0)
```
us_CarDistance:5800:100: // 1m distance
us_CarDistance:0:0:      // nothing in range (timeout occured)
```

Serial format for SmartEVSE serial input: `ev_<pin_name>:<status>:<seconds_since_last_status_change>:<newline \n>`

SmartEVSE example (status is one of these 3: `standby` (state A), `detected` (but not charging, state B), or `charging` (state C)
```
ev_Type2Left:standby:3600:  // no connection, last change an hour ago
ev_Type2Right:detected:3:   // 3 seconds ago, a cable was plugged in (or charging stopped)
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
- `CFOS/MusterCF/ai_PPVoltage/meaning`: `charging`, `detected` or `standby` (see serial output)
- `CFOS/MusterCF/us_CarDistance/duration_microsecs`: Ultrasound pulse duration in microseconds (0 when timed out)
- `CFOS/MusterCF/us_CarDistance/distance_cm`: Calculated object distance in cm (0 when timed out)
- `CFOS/MusterCF/us_CarDistance/object_detected`: `no` (when timed out) or `yes`
- `CFOS/MusterCF/ev_Type2Right/status`: `standby`, `detected`, or `charging` (see serial output)
- `CFOS/MusterCF/ev_Type2Right/secs_since_last_change`: the number of seconds since the status has changed
