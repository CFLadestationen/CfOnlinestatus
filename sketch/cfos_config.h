/**
 * CfOnlinestatus
 * Remotely reading the status of an EV charging station and sending it to an endpoint.
 * Repository: https://github.com/CFLadestationen/CfOnlinestatus
 */
#include "cfos_types.h"
// Enable software features by uncommenting the #define directive (remove the //)

// INPUT METHODS: Which inputs are activated?
// digital S0 power meter input
#define CFOS_IN_S0
// generic digital input
#define CFOS_IN_DIGITAL
// analog input to determine the EV state (standby/connected/charging)
#define CFOS_IN_ANALOG
// hc-sr04p ultrasound input to determine if a vehicle is parked
#define CFOS_IN_ULTRASOUND
// serial input from a SmartEVSE to determine the EV state (standby/connected/charging)
#define CFOS_IN_SMARTEVSE

// NETWORK ACCESS: Which networking feature is activated? (only one allowed)
// ESP8266 WiFi
#define CFOS_NET_WIFI
// Arduino Ethernet shield
//#define CFOS_NET_ETHERNET
// Arduino GSM shield
//#define CFOS_NET_GSM

// OUTPUT METHODS: Which output methods are activated?
// Serial output of the controller
#define CFOS_OUT_SERIAL
// MQTT output (network access needed)
#define CFOS_OUT_MQTT
// LoRaWAN output
//#define CFOS_OUT_LORA

// Unique identifier for the charging station (use the GoingElectric charge point number)
const char* chargepoint_id = "100000";
// Length of update timeframe: Update sensors every ... s
const uint32_t sensor_update_interval_s = 15;

#if defined(CFOS_NET_WIFI)
const char* wifi_ssid     = "WiFiSSID";
const char* wifi_key      = "WiFiPresharedKey";
#endif //CFOS_NET_WIFI

#if defined(CFOS_NET_ETHERNET)
const byte mac[] = {0x00, 0xAA, 0xBB, 0xCC, 0xDE, 0x02};
const uint32_t ethernet_dhcp_renew_s = 300;
#endif //CFOS_NET_WIFI

#if defined(CFOS_OUT_LORA)
// send TTN LoRa updates every ... s - don't use less than 60
const uint32_t lora_update_interval_s = 60;
const int8_t retries = -1;
const uint32_t retryDelay = 300000;
const ttn_fp_t freqPlan = TTN_FP_EU868;
const char* appEui = "appEui";
const char* appKey = "appKey";
#endif //CFOS_OUT_LORA

#if defined(CFOS_OUT_MQTT)
// send MQTT updates every ... s - don't use less than 30
const uint32_t mqtt_update_interval_s = 60;
const char*   mqtt_server = "46.38.232.97";
const uint16_t  mqtt_port = 1883;
const char* mqtt_username = chargepoint_id;
const char* mqtt_password = "";
#endif //CFOS_OUT_MQTT

#if defined(CFOS_OUT_SERIAL)
const uint32_t serial_baudrate = 115200;
// Send serial updates every ... s
const uint32_t serial_output_interval_s = 60;
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
const uint32_t s0_impulses_per_kWh[]
{
  1000, // 1000 impulses per kWh for first s0 pin
  160   // 160 impulses per kWh for second s0 pin
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
    20000,         // Timeout in us - multiply max distance in cm by 58, add 10% safety margin
    HIGH,          // Trigger ON value
    LOW,           // Trigger OFF value
    HIGH,          // Echo ON value
    250            // How many cm must be free so that this space is considered free?
  }
};
#endif //CFOS_IN_ULTRASOUND

#if defined(CFOS_IN_SMARTEVSE)
// Used for reading the connection status of a SmartEVSE using software serial
const smartevse_pin evse_input[] {
  {
    "Type2Left",  // SmartEVSE name
    D0,           // SmartEVSE input pin
    115200        // SmartEVSE baudrate
  },
  {
    "Type2Right", // SmartEVSE name
    D7,           // SmartEVSE input pin
    115200        // SmartEVSE baudrate
  }
};
#endif //CFOS_IN_SMARTEVSE
