/**
 * Arduino-Sketch for the online status of crowdfunded charging stations
 * Repository: https://github.com/CFLadestationen/CfOnlinestatus
 * Development thread (in German): https://www.goingelectric.de/forum/goingelectric-crowdfunding/neues-projekt-onlinestatus-fuer-crowdfunding-ladepunkte-t29325.html
 * 
 * Configuration settings go into config.h
 * Only change here if you know what you're doing!
 */
#include "cfos_config.h"

#if defined(ARDUINO_ESP8266_WEMOS_D1MINI)
#define CFOS_HARDWARE_PLATFORM ("WeMos D1 R2 & mini")
#elif defined(ESP8266)
#define CFOS_HARDWARE_PLATFORM ("Generic ESP8266")
#elif defined(ARDUINO_AVR_UNO)
#define CFOS_HARDWARE_PLATFORM ("Arduino/Genuino Uno")
#else
#define CFOS_HARDWARE_PLATFORM ("unknown")
#error CfOnlinestatus does not know this hardware platform - please fix!
#endif //Hardware

#if defined(CFOS_NET_WIFI)
#if defined(ESP8266)
#include <ESP8266WiFi.h>
#else // not ESP8266
#error The selected hardware platform is not capable of WiFi!
#endif //ESP8266
#endif //CFOS_NET_WIFI

// Internal variables
uint32_t last_sensor_update;
#if defined(CFOS_OUT_SERIAL)
uint32_t last_serial_output;
#endif //CFOS_OUT_SERIAL
#if defined(CFOS_IN_S0)
const uint8_t s0_pincount = sizeof(s0)/sizeof(s0[0]);
static_assert(s0_pincount>0, "S0 input selected, but no S0 input pins defined");
uint32_t last_s0_millis[s0_pincount];
uint32_t last_s0_span[s0_pincount];
uint8_t  last_s0_state[s0_pincount];
uint32_t impulses_since_update[s0_pincount];
uint32_t impulses_in_previous_timeframe[s0_pincount];
#endif //CFOS_IN_S0
#if defined(CFOS_IN_DIGITAL)
const uint8_t di_pincount = sizeof(digital_input)/sizeof(digital_input[0]);
static_assert(di_pincount>0, "Digital input selected, but no digital input pins defined");
uint8_t di_value[di_pincount];
#endif //CFOS_IN_DIGITAL
#if defined(CFOS_IN_ANALOG)
const uint8_t ai_pincount = sizeof(analog_input)/sizeof(analog_input[0]);
static_assert(ai_pincount>0, "Analog input selected, but no analog input pins defined");
uint8_t ai_value[ai_pincount];
#endif //CFOS_IN_ANALOG
#if defined(CFOS_IN_ULTRASOUND)
const uint8_t us_pincount = sizeof(us_sensor)/sizeof(us_sensor[0]);
static_assert(us_pincount>0, "Ultrasound input selected, but no ultrasound input pins defined");
uint32_t us_duration[us_pincount];
#endif //CFOS_IN_ULTRASOUND

void setup() {
  last_sensor_update = 0;
  init_serial();
  init_inputs();
  init_network();

#if defined(CFOS_OUT_SERIAL)
  Serial.println("CfOnlinestatus initialisation complete.");
#endif //CFOS_OUT_SERIAL
}

void loop() {
  uint32_t current_time = millis();
  read_s0_inputs();
  if(((uint32_t)(current_time-last_sensor_update)) >= sensor_update_interval) {
    last_sensor_update = current_time;
    update_s0();
    update_digital_input();
    update_analog_input();
    update_ultrasound();
  }
  print_serial_interval();
}

inline void init_serial() {
  #if defined(CFOS_OUT_SERIAL)
  Serial.begin(serial_baudrate);
  // wait for serial connection
  while(!Serial) {}
  last_serial_output = 0;
  Serial.print("\n\nCfOnlinestatus id ");
  Serial.print(chargepoint_id);
  Serial.println(" starting with following features:");
  Serial.print("Hardware platform: ");
  Serial.println(CFOS_HARDWARE_PLATFORM);
  Serial.print("Update interval: ");
  Serial.print(sensor_update_interval);
  Serial.println(" milliseconds");
  Serial.print("Serial output interval: ");
  Serial.print(serial_output_interval);
  Serial.println(" milliseconds");
#if defined(CFOS_IN_S0)
  Serial.print(s0_pincount);
  Serial.println(" S0 pin(s)");
#endif //CFOS_IN_S0
#if defined(CFOS_IN_DIGITAL)
  Serial.print(di_pincount);
  Serial.println(" digital input pin(s)");
#endif //CFOS_IN_DIGITAL
#if defined(CFOS_IN_ANALOG)
  Serial.print(ai_pincount);
  Serial.println(" analog input pin(s)");
#endif //CFOS_IN_ANALOG
#if defined(CFOS_IN_ULTRASOUND)
  Serial.print(us_pincount);
  Serial.println(" HC-SR04 ultrasound sensor(s)");
#endif //CFOS_IN_ULTRASOUND
#if defined(CFOS_NET_WIFI)
  Serial.println("WiFi connection");
#endif //CFOS_NET_WIFI
#if defined(CFOS_NET_LAN)
  Serial.println("LAN connection (not implemented yet!)");
#endif //CFOS_NET_LAN
#if defined(CFOS_NET_LORA)
  Serial.println("LoRaWAN connection (not implemented yet!)");
#endif //CFOS_NET_LORA
#if defined(CFOS_NET_GSM)
  Serial.println("GSM connection (not implemented yet!)");
#endif //CFOS_NET_GSM
#endif //CFOS_OUT_SERIAL
}

inline void init_inputs() {
#if defined(CFOS_IN_S0)
  for(uint8_t i = 0; i < s0_pincount; i++) {
    pinMode(s0[i].pin_number, s0[i].pin_mode);
    last_s0_millis[i] = 0;
    last_s0_span[i] = 0;
    last_s0_state[i] = s0[i].off_value;
    impulses_since_update[i] = 0;
#if defined(CFOS_OUT_SERIAL)
    Serial.print("Configured pin number ");
    Serial.print(s0[i].pin_number);
    Serial.print(" as S0 pin named ");
    Serial.println(s0[i].pin_name);
#endif //CFOS_OUT_SERIAL
  }
#endif //CFOS_IN_S0
#if defined(CFOS_IN_DIGITAL)
  for(uint8_t i = 0; i < di_pincount; i++) {
    pinMode(digital_input[i].pin_number, digital_input[i].pin_mode);
#if defined(CFOS_OUT_SERIAL)
    Serial.print("Configured pin number ");
    Serial.print(digital_input[i].pin_number);
    Serial.print(" as digital input pin named ");
    Serial.println(digital_input[i].pin_name);
#endif //CFOS_OUT_SERIAL
  }
#endif //CFOS_IN_DIGITAL
#if defined(CFOS_IN_ANALOG)
  for(uint8_t i = 0; i < ai_pincount; i++) {
    pinMode(analog_input[i].pin_number, analog_input[i].pin_mode);
#if defined(CFOS_OUT_SERIAL)
    Serial.print("Configured pin number ");
    Serial.print(analog_input[i].pin_number);
    Serial.print(" as analog input pin named ");
    Serial.print(analog_input[i].pin_name);
    Serial.print(" with low threshold ");
    Serial.print(analog_input[i].off_value);
    Serial.print(" and high threshold ");
    Serial.println(analog_input[i].on_value);
#endif //CFOS_OUT_SERIAL
  }
#endif //CFOS_IN_ANALOG
#if defined(CFOS_IN_ULTRASOUND)
  for(uint8_t i = 0; i < us_pincount; i++) {
    pinMode(us_sensor[i].echo_pin, INPUT);
    pinMode(us_sensor[i].trigger_pin, OUTPUT);
    digitalWrite(us_sensor[i].trigger_pin, us_sensor[i].trigger_off);
#if defined(CFOS_OUT_SERIAL)
    Serial.print("Configured ultrasound sensor named ");
    Serial.print(us_sensor[i].sensor_name);
    Serial.print(" with echo pin ");
    Serial.print(us_sensor[i].echo_pin);
    Serial.print(", trigger pin ");
    Serial.print(us_sensor[i].trigger_pin);
    Serial.print(" and timeout ");
    Serial.print(us_sensor[i].timeout_us);
    Serial.println(" us");
#endif //CFOS_OUT_SERIAL
  }
#endif //CFOS_IN_ULTRASOUND
}

inline void init_network() {
#if defined(CFOS_NET_WIFI)
  WiFi.mode(WIFI_STA);
  WiFi.begin(wifi_ssid, wifi_key);
  int wifi_tries = 20;
  while(wifi_tries-->0 && WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
#if defined(CFOS_OUT_SERIAL)
  Serial.print("Configured Wifi with SSID ");
  Serial.print(wifi_ssid);
  Serial.print(" and key ");
  Serial.print(wifi_key);
  Serial.print(" -- status is currently ");
  if(WiFi.status() == WL_CONNECTED) {
    Serial.print("connected with local IP ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println(WiFi.status());
  }
#endif //CFOS_OUT_SERIAL
#endif //CFOS_NET_WIFI
}

inline void print_serial_interval() {
#if defined(CFOS_OUT_SERIAL)
  uint32_t current_time = millis();
  if(((uint32_t)(current_time-last_serial_output)) < serial_output_interval) {
    return;
  }
  last_serial_output = current_time;
  Serial.print("id_");
  Serial.print(chargepoint_id);
  Serial.print(':');
  Serial.print(current_time);
  Serial.println(':');
#if defined(CFOS_IN_S0)
  print_s0_status();
#endif //CFOS_IN_S0
#if defined(CFOS_IN_DIGITAL)
  print_digital_input_status();
#endif //CFOS_IN_DIGITAL
#if defined(CFOS_IN_ANALOG)
  print_analog_input_status();
#endif //CFOS_IN_ANALOG
#if defined(CFOS_IN_ULTRASOUND)
  print_ultrasound_status();
#endif //CFOS_IN_ULTRASOUND
#endif //CFOS_OUT_SERIAL
}


inline void read_s0_inputs() {
#if defined(CFOS_IN_S0)
  uint32_t current_time = millis();
  for(uint8_t i = 0; i < s0_pincount; i++) {
    uint8_t value = digitalRead(s0[i].pin_number);
    if(value == s0[i].on_value && last_s0_state[i] == s0[i].off_value) {
      // change detected
      last_s0_span[i] = (uint32_t)(current_time - last_s0_millis[i]); // cast to avoid rollover problems
      last_s0_millis[i] = current_time;
      impulses_since_update[i]++;
    }
    last_s0_state[i] = value;
  }
#endif //CFOS_IN_S0
}

inline void update_s0() {
#if defined(CFOS_IN_S0)
  for(uint8_t i = 0; i < s0_pincount; i++) {
    impulses_in_previous_timeframe[i] = impulses_since_update[i];
    impulses_since_update[i] = 0;
  }
#endif //CFOS_IN_S0
}

#if defined(CFOS_IN_S0) && defined(CFOS_OUT_SERIAL)
void print_s0_status() {
  uint32_t current_time = millis();
  for(uint8_t i = 0; i < s0_pincount; i++) {
    Serial.print("s0_");
    Serial.print(s0[i].pin_name);
    Serial.print(':');
    Serial.print(last_s0_span[i]);
    Serial.print(':');
    Serial.print(impulses_in_previous_timeframe[i]);
    Serial.print(':');
    Serial.print(((uint32_t)(current_time - last_s0_millis[i]))/1000);
    Serial.println(":");
  }
}
#endif //CFOS_IN_S0 && CFOS_OUT_SERIAL


inline void update_digital_input() {
#if defined(CFOS_IN_DIGITAL)
  for(uint8_t i = 0; i < di_pincount; i++) {
    di_value[i] = digitalRead(digital_input[i].pin_number);
  }
#endif //CFOS_IN_DIGITAL
}


#if defined(CFOS_IN_DIGITAL) && defined(CFOS_OUT_SERIAL)
void print_digital_input_status() {
  for(uint8_t i = 0; i < di_pincount; i++) {
    Serial.print("di_");
    Serial.print(digital_input[i].pin_name);
    Serial.print(':');
    if(di_value[i] == digital_input[i].on_value) {
      Serial.print("on");
    } else {
      Serial.print("off");
    }
    Serial.println(":");
  }
}
#endif //CFOS_IN_DIGITAL && CFOS_OUT_SERIAL

inline void update_analog_input() {
#if defined(CFOS_IN_ANALOG)
  for(uint8_t i = 0; i < ai_pincount; i++) {
    ai_value[i] = (uint8_t)(analogRead(analog_input[i].pin_number) >> 2);
  }
#endif //CFOS_IN_ANALOG
}

#if defined(CFOS_IN_ANALOG) && defined(CFOS_OUT_SERIAL)
void print_analog_input_status() {
  for(uint8_t i = 0; i < ai_pincount; i++) {
    Serial.print("ai_");
    Serial.print(analog_input[i].pin_name);
    Serial.print(':');
    Serial.print(ai_value[i]);
    Serial.print(':');
    if(ai_value[i] > analog_input[i].on_value) {
      Serial.print("high");
    } else if(ai_value[i] <= analog_input[i].off_value) {
      Serial.print("low");
    } else {
      Serial.print("mid");
    }
    Serial.println(":");
  }
}
#endif //CFOS_IN_ANALOG && CFOS_OUT_SERIAL


inline void update_ultrasound() {
#if defined(CFOS_IN_ULTRASOUND)
  for(uint8_t i = 0; i < us_pincount; i++) {
    digitalWrite(us_sensor[i].trigger_pin, us_sensor[i].trigger_off);
    delayMicroseconds(10);
    digitalWrite(us_sensor[i].trigger_pin, us_sensor[i].trigger_on);
    delayMicroseconds(10);
    digitalWrite(us_sensor[i].trigger_pin, us_sensor[i].trigger_off);
    us_duration[i] = pulseIn(us_sensor[i].echo_pin, us_sensor[i].echo_on, us_sensor[i].timeout_us);
  }
#endif //CFOS_IN_ULTRASOUND
}

#if defined(CFOS_IN_ULTRASOUND) && defined(CFOS_OUT_SERIAL)
void print_ultrasound_status() {
  for(uint8_t i = 0; i < us_pincount; i++) {
    uint32_t distance = us_duration[i] / 58;
    Serial.print("us_");
    Serial.print(us_sensor[i].sensor_name);
    Serial.print(':');
    Serial.print(us_duration[i]);
    Serial.print(':');
    Serial.print(distance);
    Serial.println(':');
  }
}
#endif //CFOS_IN_ULTRASOUND && CFOS_OUT_SERIAL

