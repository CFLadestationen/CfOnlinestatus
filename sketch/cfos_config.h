/**
 * CfOnlinestatus
 * Remotely reading the status of an EV charging station and sending it to an endpoint.
 * Repository: https://github.com/CFLadestationen/CfOnlinestatus
 */
#include "cfos_types.h"


/* TODO LoRa config
#if defined(CFOS_OUT_LORA)
#if defined(ARDUINO_AVR_UNO)
#include <TheThingsNetwork.h>
#define loraSerial Serial
// send TTN LoRa updates every ... s - don't use less than 60
const uint32_t lora_update_interval_s = 60;
const int8_t retries = -1;
const uint32_t retryDelay = 300000;
const ttn_fp_t freqPlan = TTN_FP_EU868;
const char* appEui = "appEui";
const char* appKey = "appKey";
#endif //ARDUINO_AVR_UNO
#endif //CFOS_OUT_LORA
*/

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
#endif //CFOS_IN_SMARTEVSE */
