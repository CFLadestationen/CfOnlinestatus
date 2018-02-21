
/**
 * Arduino-Sketch for the online status of crowdfunded charging stations
 * Repository: https://github.com/CFLadestationen/CfOnlinestatus
 * Development thread (in German): https://www.goingelectric.de/forum/goingelectric-crowdfunding/neues-projekt-onlinestatus-fuer-crowdfunding-ladepunkte-t29325.html
 * 
 * Current features
 * - Read an arbitrary number of S0 inputs, digital inputs, analog inputs and HC-SR04 ultrasound sensors
 * - Print the status of those inputs on the serial console
 * - Serial output begins with id_<chargepoint_id>:<ms_since_controller_start>:<newline \n>
 *   Example:
 *     id_CFMusterstadtGoethestr12:480068:
 * - Serial format for S0: s0_<pin_name>:<ms_between_last_2_impulses>:<impulses_in_previous_update_timeframe>:<seconds since last impulse>:<newline \n>
 *   S0 example:
 *     s0_CounterA:300:6:0:  // s0 pin CounterA had 300 ms between the last two impulses, 6 impulses in the previous update timeframe and 0 seconds since the last impulse
 *     s0_CounterB:465:0:20: // s0 pin CounterB had 465 ms between the last two impulses, 0 impulses in the previous update timeframe and 20 seconds since the last impulse
 * - Serial format for digital input: di_<pin_name>:<status on or off>:<newline \n>
 *   Digital input example:
 *     di_SpaceOccupied:on:
 *     di_ContactorOn:off:
 * - Serial format for analog input: ai_<pin_name>:<analog_value_as_uint8>:<low, mid or high>:<newline \n>
 *   Analog input example (low means <= off_value, high means > on_value, mid means in between the two)
 *     ai_PPVoltage:100:mid:
 *     ai_PPVoltage:80:low:
 *     ai_PPVoltage:220:high:
 * - Serial format for HC-SR04 ultrasound input: us_<sensor_name>:<delay_in_us_as_uint32>:<distance_in_cm_as_uint32>:<newline \n>
 *   Ultrasound example (If a timeout occurs, the printed delay and distance value is 0)
 *     us_CarDistance:5800:100: // 1m distance
 *     us_CarDistance:0:0:      // nothing in range (timeout occured)
 */
#include "cfos_types.h"
// Enable software features by uncommenting the #define directive (remove the //)
#define INPUT_S0         1
#define INPUT_DIGITAL    1
#define INPUT_ANALOG     1
#define INPUT_ULTRASOUND 1
#define OUTPUT_SERIAL    1
//#define OUTPUT_WIFI      1
//#define OUTPUT_LAN       1
//#define OUTPUT_LORA      1
#define OUTPUT_GSM       1

// Unique name for the charging station
const char* chargepoint_id = "CFMusterstadtGoethestr12";
// Length of update timeframe: Update sensors every ... ms
const uint32_t sensor_update_interval = 15000;

#if defined(OUTPUT_SERIAL)
const uint32_t serial_baudrate = 115200;
// Send serial updates every ... ms
const uint32_t serial_output_interval = 60000;
#endif // OUTPUT_SERIAL

#if defined(INPUT_S0)
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
#endif //INPUT_S0

#if defined(INPUT_DIGITAL)
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
#endif //INPUT_DIGITAL

#if defined(INPUT_ANALOG)
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
#endif //INPUT_ANALOG

#if defined(INPUT_ULTRASOUND)
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
#endif //INPUT_ULTRASOUND
