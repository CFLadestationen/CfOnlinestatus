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

#endif //CFOS_TYPES_H
