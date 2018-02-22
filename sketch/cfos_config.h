/**
 * CfOnlinestatus
 * Remotely reading the status of an EV charging station and sending it to an endpoint.
 * Repository: https://github.com/CFLadestationen/CfOnlinestatus
 */
#include "cfos_types.h"
// Enable software features by uncommenting the #define directive (remove the //)
// Which inputs are activated?
#define CFOS_IN_S0
#define CFOS_IN_DIGITAL
#define CFOS_IN_ANALOG
#define CFOS_IN_ULTRASOUND

// Which networking feature is activated? (only one allowed)
#define CFOS_NET_WIFI
//#define CFOS_NET_ETHERNET
//#define CFOS_NET_LORA
//#define CFOS_NET_GSM

// Which output methods are activated?
#define CFOS_OUT_SERIAL
#define CFOS_OUT_MQTT

// Unique name for the charging station
const char* chargepoint_id = "MusterstadtGoethestr12";
// Length of update timeframe: Update sensors every ... ms
const uint32_t sensor_update_interval = 15000;

#if defined(CFOS_NET_WIFI)
const char* wifi_ssid     = "WiFiSSID";
const char* wifi_key      = "WiFiPresharedKey";
#endif //CFOS_NET_WIFI

#if defined(CFOS_OUT_MQTT)
// send MQTT updates every ... ms - don't use less than 30000
const uint32_t mqtt_update_interval = 60000;
const char*   mqtt_server = "192.168.178.21";
const uint16_t  mqtt_port = 1883;
const char* mqtt_username = NULL;
const char* mqtt_password = NULL;
#endif //CFOS_OUT_MQTT

#if defined(CFOS_OUT_SERIAL)
const uint32_t serial_baudrate = 115200;
// Send serial updates every ... ms
const uint32_t serial_output_interval = 60000;
#endif //CFOS_OUT_SERIAL

#if defined(CFOS_IN_S0)
// Definition of S0 inputs
const named_pin s0[] = {
  {
    "CounterA",  // pin name
    D1,          // pin number
    HIGH,        // "on" value
    LOW,         // "off" value
    INPUT        // pin mode
  },
  {
    "CounterB",  // pin name
    D2,          // pin number
    LOW,         // "on" value
    HIGH,        // "off" value
    INPUT_PULLUP // pin mode    
  }
};
#endif //CFOS_IN_S0

#if defined(CFOS_IN_DIGITAL)
// Definition of digital inputs
const named_pin digital_input[] = {
  {
    "SpaceOccupied", // pin name
    D5,              // pin number
    HIGH,            // "on" value
    LOW,             // "off" value
    INPUT            // pin mode
  },
  {
    "ContactorOn",   // pin name
    D4,              // pin number
    LOW,             // "on" value
    HIGH,            // "off" value
    INPUT_PULLUP     // pin mode    
  }
};
#endif //CFOS_IN_DIGITAL

#if defined(CFOS_IN_ANALOG)
// Definition of analog inputs
// CAUTION: ESP8266 only has one analog input (A0) with a voltage range of 0V - 1V, 
//          so you will need a 1:4 voltage divider (5V becomes 1V) to detect PP voltage.
const named_pin analog_input[] = {
  {
    "PPVoltage",   // pin name
    A0,            // pin number
    217, // 0.85 V // "high" voltage (for SAE J1772: PP = not connected)
    87,  // 0.34 V // "low" voltage (for SAE J1772: PP = connected)
    // if value is between the two limits, it is considered "mid" (for SAE J1772: PP = button pressed)
    INPUT        // pin mode
  }
};
#endif //CFOS_IN_ANALOG

#if defined(CFOS_IN_ULTRASOUND)
// Definition of HC-SR04 ultrasound sensors
const ultrasound_sensor us_sensor[] {
  {
    "CarDistance", // sensor name
    D8,            // Trigger pin
    D6,            // Echo pin
    20000,         // Timeout in us - multiply max distance by 58, add 10% safety margin
    HIGH,          // Trigger ON value
    LOW,           // Trigger OFF value
    HIGH           // Echo ON value
  }
};
#endif //CFOS_IN_ULTRASOUND
