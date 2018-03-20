/**
 * CfOnlinestatus
 * Remotely reading the status of an EV charging station and sending it to an endpoint.
 * Repository: https://github.com/CFLadestationen/CfOnlinestatus
 *
 * Configuration settings go into cfos_config.h
 * Only change here if you know what you're doing.
 * If you add a bugfix, please submit it via GitHub!
 */
#ifndef CFOS_TYPES_H
#define CFOS_TYPES_H

struct named_pin {
  const char* pin_name;
  const uint8_t pin_number;
  const uint8_t on_value;
  const uint8_t off_value;
  const uint8_t pin_mode;
};

struct ultrasound_sensor {
  const char* sensor_name;
  const uint8_t trigger_pin;
  const uint8_t echo_pin;
  const uint32_t timeout_us;
  const uint8_t trigger_on;
  const uint8_t trigger_off;
  const uint8_t echo_on;
};

struct smartevse_pin {
  const char* pin_name;
  const uint8_t pin_number;
  const uint32_t baudrate;
};

enum evse_state : uint8_t {
  EVSE_STATE_A = 1, // not connected
  EVSE_STATE_B = 2, // vehicle connected, not charging
  EVSE_STATE_C = 3  // vehicle connected, charging
};

#endif //CFOS_TYPES_H
