/**
 * CfOnlinestatus
 * Remotely reading the status of an EV charging station and sending it to an endpoint.
 * Repository: https://github.com/CFLadestationen/CfOnlinestatus
 *
 * Configuration settings go into cfos_config.h
 * Only change here if you know what you're doing.
 * If you add a bugfix, please submit it via GitHub!
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

#if (defined(CFOS_OUT_MQTT)) && (!defined(CFOS_NET_WIFI) && !defined(CFOS_NET_ETHERNET) && !defined(CFOS_NET_LORA) && !defined(CFOS_NET_GSM))
#error Network output selected, but no network access method defined
#endif //Network check

#if (defined(CFOS_NET_WIFI) && defined(CFOS_NET_ETHERNET)) || (defined(CFOS_NET_WIFI) && defined(CFOS_NET_LORA)) || (defined(CFOS_NET_WIFI) && defined(CFOS_NET_GSM)) || (defined(CFOS_NET_ETHERNET) && defined(CFOS_NET_LORA)) || (defined(CFOS_NET_ETHERNET) && defined(CFOS_NET_GSM))  || (defined(CFOS_NET_LORA) && defined(CFOS_NET_GSM))
#error Only one network access method is allowed
#endif //Network access

#if defined(CFOS_NET_WIFI)
#if defined(ESP8266)
#include <ESP8266WiFi.h>
WiFiClient wifi_client;
#else //not ESP8266
#error CfOnlinestatus does not know how to use WiFi with this device!
#endif //ESP8266
#endif //CFOS_NET_WIFI

#if defined(CFOS_NET_LORA)
#if defined(ARDUINO_AVR_UNO)
#include <TheThingsNetwork.h>
const uint32_t lora_update_interval = 1000 * lora_update_interval_s;
static_assert(lora_update_interval>59999, "MQTT update interval must be 60 seconds or more");
#define DEBUGRATE 115200
#define LORA_RATE 57600
#define loraSerial Serial1
#define debugSerial Serial
TheThingsNetwork ttn(loraSerial, debugSerial, freqPlan);
byte payload[12];
payload[0] = (int)0; //S0 Meter A power (watts)
payload[1] = (int)0; //S0 Meter A power (watts)
payload[2] = (int)0; //S0 Meter A power (watts)
payload[3] = (int)0; //S0 Meter A power (watts)
payload[4] = (int)0; //S0 Meter A power (watts)
payload[5] = (int)0; //S0 Meter B power (watts)
payload[6] = (int)0; //S0 Meter B power (watts)
payload[7] = (int)0; //S0 Meter B power (watts)
payload[8] = (int)9; //Digital In SpaceOccupied (0/1)
payload[9] = (int)9; //Digital In ContactorOn (0/1)
payload[10] = (int)9; //Analog In (0/1/2)
payload[11] = (int)9; //Ultrasound object_detected (0/1)
payload[12] = (int)9; //SmartEVSE A serial (0/1/2)
payload[13] = (int)9; //SmartEVSE B serial (0/1/2)
uint32_t last_lora_output;
#else //not ARDUINO_AVR_UNO
#error CfOnlinestatus does not know how to use LoRa with this device!
#endif //ARDUINO_AVR_UNO
#endif //CFOS_NET_LORA

#if defined(CFOS_IN_SMARTEVSE)
#if !defined(ESP8266)
#error CfOnlinestatus only supports ESP8266 for SmartEVSE input!
#endif //!ESP8266
const uint8_t evse_pincount = sizeof(evse_input)/sizeof(evse_input[0]);
static_assert(evse_pincount>0, "SmartEVSE input selected, but no SmartEVSE input pins defined");
#include "SoftwareSerial.h"
SoftwareSerial* evse_serial[evse_pincount];
evse_state evse_status[evse_pincount];
uint32_t last_evse_change[evse_pincount];
char evse_buffer[evse_pincount][12];
uint8_t evse_buffer_pos[evse_pincount];
#endif //CFOS_IN_SMARTEVSE

#if defined(CFOS_OUT_MQTT)
#include <PubSubClient.h>
PubSubClient mqtt_client;
uint32_t last_mqtt_output;
char mqtt_topic_buf[80];
char mqtt_msg_buf[80];
bool last_mqtt_connected;
bool current_mqtt_connected;
const uint32_t mqtt_update_interval = 1000 * mqtt_update_interval_s;
static_assert(mqtt_update_interval>29999, "MQTT update interval must be 30 seconds or more");
#endif //CFOS_OUT_MQTT

// Internal variables
uint32_t last_sensor_update;
const uint32_t sensor_update_interval = 1000 * sensor_update_interval_s;
#if defined(CFOS_OUT_SERIAL)
uint32_t last_serial_output;
const uint32_t serial_output_interval = 1000 * serial_output_interval_s;
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
  init_lora();
  init_mqtt();
  init_smartevse();

#if defined(CFOS_OUT_SERIAL)
  Serial.println("CfOnlinestatus initialisation complete.");
#endif //CFOS_OUT_SERIAL
}

void loop() {
  uint32_t current_time = millis();
  read_s0_inputs();
  read_evse_buffers();
  if(((uint32_t)(current_time-last_sensor_update)) >= sensor_update_interval) {
    last_sensor_update = current_time;
    update_s0();
    update_digital_input();
    update_analog_input();
    update_ultrasound();
  }
  output_serial_interval();
  output_mqtt_interval();
  output_lora_interval();
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
#if defined(CFOS_IN_SMARTEVSE)
  Serial.print(evse_pincount);
  Serial.println(" SmartEVSE serial input(s)");
#endif //CFOS_IN_SMARTEVSE
#if defined(CFOS_NET_WIFI)
  Serial.println("WiFi connection");
#endif //CFOS_NET_WIFI
#if defined(CFOS_NET_ETHERNET)
  Serial.println("LAN connection (not implemented yet!)");
#endif //CFOS_NET_ETHERNET
#if defined(CFOS_NET_LORA)
  Serial.println("LoRaWAN connection");
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

inline void init_mqtt() {
#if defined(CFOS_OUT_MQTT)
#if defined(CFOS_NET_WIFI)
  mqtt_client.setClient(wifi_client);
#endif //CFOS_NET_WIFI
#if defined(CFOS_NET_ETHERNET)
#error Ethernet is not implemented yet!
  mqtt_client.setClient(ethernet_client);
#endif //CFOS_NET_ETHERNET
  mqtt_client.setServer(mqtt_server, mqtt_port);
#endif //CFOS_OUT_MQTT
}

inline void output_serial_interval() {
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
#if defined(CFOS_IN_SMARTEVSE)
  print_evse_status();
#endif //CFOS_IN_SMARTEVSE
#endif //CFOS_OUT_SERIAL
}

inline void output_mqtt_interval() {
#if defined(CFOS_OUT_MQTT)
  last_mqtt_connected = current_mqtt_connected;
  current_mqtt_connected = mqtt_client.loop();
  if(!current_mqtt_connected) {
    // start reconnecting and wait 15s
    last_mqtt_output = millis()-15000;
    snprintf(mqtt_topic_buf, sizeof(mqtt_topic_buf), "CFOS/%s/status", chargepoint_id);
    mqtt_client.connect(chargepoint_id, mqtt_username, mqtt_password, mqtt_topic_buf, 0, 1, "offline");
    delay(10);
    return;
  }
  if(current_mqtt_connected && !last_mqtt_connected) {
    // client has just successfully reconnected
    snprintf(mqtt_topic_buf, sizeof(mqtt_topic_buf), "CFOS/%s/status", chargepoint_id);
    mqtt_client.publish(mqtt_topic_buf, "online");
  }
  uint32_t current_time = millis();
  if(((uint32_t)(current_time-last_mqtt_output)) < mqtt_update_interval) {
    return;
  }
  last_mqtt_output = current_time;
#if defined(CFOS_IN_S0)
  send_mqtt_s0_status();
#endif //CFOS_IN_S0
#if defined(CFOS_IN_DIGITAL)
  send_mqtt_digital_input_status();
#endif //CFOS_IN_DIGITAL
#if defined(CFOS_IN_ANALOG)
  send_mqtt_analog_input_status();
#endif //CFOS_IN_ANALOG
#if defined(CFOS_IN_ULTRASOUND)
  send_mqtt_ultrasound_status();
#endif //CFOS_IN_ULTRASOUND
#if defined(CFOS_IN_SMARTEVSE)
  send_mqtt_evse_status();
#endif //CFOS_IN_SMARTEVSE
#endif //CFOS_OUT_MQTT
}

inline void output_lora_interval() {
#if defined(CFOS_NET_LORA)
  uint32_t current_time = millis();
  if(((uint32_t)(current_time-last_lora_output)) < lora_update_interval) {
    return;
  }
  last_lora_output = current_time;
#if defined(CFOS_IN_S0)
  send_lora_s0_status();
#endif //CFOS_IN_S0
#if defined(CFOS_IN_DIGITAL)
  send_lora_digital_input_status();
#endif //CFOS_IN_DIGITAL
#if defined(CFOS_IN_ANALOG)
  send_lora_analog_input_status();
#endif //CFOS_IN_ANALOG
#if defined(CFOS_IN_ULTRASOUND)
  send_lora_ultrasound_status();
#endif //CFOS_IN_ULTRASOUND
#if defined(CFOS_IN_SMARTEVSE)
  send_lora_evse_status();
#endif //CFOS_IN_SMARTEVSE
#if defined(CFOS_OUT_SERIAL)
  Serial.print("Send data to TTN over LoRa: ");
#endif //CFOS_OUT_SERIAL
ttn.sendBytes(payload, sizeof(payload));
#endif //CFOS_NET_LORA
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

#if defined(CFOS_IN_S0)
inline uint32_t last_s0_watts(uint8_t pin) {
  // Calculation:
  //  impulses_per_h = impulses_in_previous_timeframe[pin] * 3600 / sensor_update_interval_s
  //  wh_per_impulse = 1000/s0_impulses_per_kWh[pin]
  //  w = wh_per_impulse * impulses_per_h
  // best would be: first multiplication, then division
  //  w = impulses_in_previous_timeframe[pin] * 3600 * 1000 / sensor_update_interval_s / s0_impulses_per_kWh[pin]
  // problem: at 10000 imp/kWh, there can be 1875 impulses in 15s at 45kW power, so we get an overflow
  // for uint32_t (1875 * 3600 * 1000 = 6.75e9, max for uint32_t is 4.2e9)
  // Compromise: use the smallest divisor before the first potential overflow:
  //  w = (((impulses_in_previous_timeframe[i] * 3600) / sensor_update_interval_s) * 1000) / s0_impulses_per_kWh
  return (((impulses_in_previous_timeframe[pin] * 3600) / sensor_update_interval_s) * 1000) / s0_impulses_per_kWh[pin];
}
#endif //CFOS_IN_S0

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
    Serial.print(':');
    Serial.print(last_s0_watts(i));
    Serial.println(':');
  }
}
#endif //CFOS_IN_S0 && CFOS_OUT_SERIAL

#if defined(CFOS_IN_S0) && defined(CFOS_OUT_MQTT)
inline void send_mqtt_s0_status() {
  uint32_t current_time = millis();
  for(uint8_t i = 0; i < s0_pincount; i++) {
    snprintf(mqtt_topic_buf, sizeof(mqtt_topic_buf), "CFOS/%s/s0_%s/lastspan", chargepoint_id, s0[i].pin_name);
    snprintf(mqtt_msg_buf, sizeof(mqtt_msg_buf), "%d", last_s0_span[i]);
    mqtt_client.publish(mqtt_topic_buf, mqtt_msg_buf);
    delay(10);
    snprintf(mqtt_topic_buf, sizeof(mqtt_topic_buf), "CFOS/%s/s0_%s/impulses_timeframe", chargepoint_id, s0[i].pin_name);
    snprintf(mqtt_msg_buf, sizeof(mqtt_msg_buf), "%d", impulses_in_previous_timeframe[i]);
    mqtt_client.publish(mqtt_topic_buf, mqtt_msg_buf);
    delay(10);
    snprintf(mqtt_topic_buf, sizeof(mqtt_topic_buf), "CFOS/%s/s0_%s/secs_since_last_impulse", chargepoint_id, s0[i].pin_name);
    snprintf(mqtt_msg_buf, sizeof(mqtt_msg_buf), "%d", ((uint32_t)(current_time - last_s0_millis[i]))/1000);
    mqtt_client.publish(mqtt_topic_buf, mqtt_msg_buf);
    delay(10);
    snprintf(mqtt_topic_buf, sizeof(mqtt_topic_buf), "CFOS/%s/s0_%s/power", chargepoint_id, s0[i].pin_name);
    snprintf(mqtt_msg_buf, sizeof(mqtt_msg_buf), "%d", last_s0_watts(i));
    mqtt_client.publish(mqtt_topic_buf, mqtt_msg_buf);
    delay(10);
  }
}
#endif //CFOS_IN_S0 && CFOS_OUT_MQTT


#if defined(CFOS_IN_S0) && defined(CFOS_NET_LORA)
inline void send_lora_s0_status() {   
  payload[0] = (int) ((last_s0_watts(0) & 0xFF000000) >> 24 );
  payload[1] = (int) ((last_s0_watts(0) & 0x00FF0000) >> 16 );
  payload[2] = (int) ((last_s0_watts(0) & 0x0000FF00) >> 8  );
  payload[3] = (int) ((last_s0_watts(0) & 0X000000FF)       );
  if(s0_pincount > 1) {
      payload[4] = (int) ((last_s0_watts(1) & 0xFF000000) >> 24 );
      payload[5] = (int) ((last_s0_watts(1) & 0x00FF0000) >> 16 );
      payload[6] = (int) ((last_s0_watts(1) & 0x0000FF00) >> 8  );
      payload[7] = (int) ((last_s0_watts(1) & 0X000000FF)       );
  } 
}
#endif //CFOS_IN_S0 && CFOS_NET_LORA


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

#if defined(CFOS_IN_DIGITAL) && defined(CFOS_OUT_MQTT)
void send_mqtt_digital_input_status() {
  for(uint8_t i = 0; i < di_pincount; i++) {
    snprintf(mqtt_topic_buf, sizeof(mqtt_topic_buf), "CFOS/%s/di_%s/status", chargepoint_id, digital_input[i].pin_name);
    if(di_value[i] == digital_input[i].on_value) {
      mqtt_client.publish(mqtt_topic_buf, "on");
    } else {
      mqtt_client.publish(mqtt_topic_buf, "off");
    }
    delay(10);
  }
}
#endif //CFOS_IN_DIGITAL && CFOS_OUT_MQTT

#if defined(CFOS_IN_DIGITAL) && defined(CFOS_NET_LORA)
void send_lora_digital_input_status() {
  if(di_value[0] == digital_input[0].on_value) {
    payload[8] = (int)1;  
    } else {
    payload[8] = (int)0; 
    }
    if(di_value[1] == digital_input[1].on_value) {
    payload[9] = (int)1;  
    } else {
    payload[9] = (int)0; 
    } 
  }
}
#endif //CFOS_IN_DIGITAL && CFOS_NET_LORA

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
      Serial.print("standby");
    } else if(ai_value[i] <= analog_input[i].off_value) {
      Serial.print("ready charging");
    } else {
      Serial.print("vehicle detected");
    }
    Serial.println(":");
  }
}
#endif //CFOS_IN_ANALOG && CFOS_OUT_SERIAL

#if defined(CFOS_IN_ANALOG) && defined(CFOS_OUT_MQTT)
void send_mqtt_analog_input_status() {
  for(uint8_t i = 0; i < ai_pincount; i++) {
    snprintf(mqtt_topic_buf, sizeof(mqtt_topic_buf), "CFOS/%s/ai_%s/value", chargepoint_id, analog_input[i].pin_name);
    snprintf(mqtt_msg_buf, sizeof(mqtt_msg_buf), "%d", ai_value[i]);
    mqtt_client.publish(mqtt_topic_buf, mqtt_msg_buf);
    delay(10);
    snprintf(mqtt_topic_buf, sizeof(mqtt_topic_buf), "CFOS/%s/ai_%s/meaning", chargepoint_id, analog_input[i].pin_name);
    if(ai_value[i] > analog_input[i].on_value) {
      mqtt_client.publish(mqtt_topic_buf, "standby");
    } else if(ai_value[i] <= analog_input[i].off_value) {
      mqtt_client.publish(mqtt_topic_buf, "ready charging");
    } else {
      mqtt_client.publish(mqtt_topic_buf, "vehicle detected");
    }
    delay(10);
  }
}
#endif //CFOS_IN_ANALOG && CFOS_OUT_MQTT

#if defined(CFOS_IN_ANALOG) && defined(CFOS_NET_LORA)
void send_lora_analog_input_status() {
    if(ai_value[0] > analog_input[0].on_value) {      
      payload[10] = 0; //standby
    } else if(ai_value[0] <= analog_input[0].off_value) {     
      payload[10] = (int)1; //vehicle charging
    } else {      
      payload[10] = (int)2; //vehicle detected
    }
  }
}
#endif //CFOS_IN_ANALOG && CFOS_NET_LORA

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

#if defined(CFOS_IN_ULTRASOUND) && defined(CFOS_OUT_MQTT)
void send_mqtt_ultrasound_status() {
  for(uint8_t i = 0; i < us_pincount; i++) {
    uint32_t distance = us_duration[i] / 58;
    snprintf(mqtt_topic_buf, sizeof(mqtt_topic_buf), "CFOS/%s/us_%s/duration_microsecs", chargepoint_id, us_sensor[i].sensor_name);
    snprintf(mqtt_msg_buf, sizeof(mqtt_msg_buf), "%d", us_duration[i]);
    mqtt_client.publish(mqtt_topic_buf, mqtt_msg_buf);
    delay(10);
    snprintf(mqtt_topic_buf, sizeof(mqtt_topic_buf), "CFOS/%s/us_%s/distance_cm", chargepoint_id, us_sensor[i].sensor_name);
    snprintf(mqtt_msg_buf, sizeof(mqtt_msg_buf), "%d", distance);
    mqtt_client.publish(mqtt_topic_buf, mqtt_msg_buf);
    delay(10);
    snprintf(mqtt_topic_buf, sizeof(mqtt_topic_buf), "CFOS/%s/us_%s/object_detected", chargepoint_id, us_sensor[i].sensor_name);
    snprintf(mqtt_msg_buf, sizeof(mqtt_msg_buf), "%s", distance==0?"no":"yes");
    mqtt_client.publish(mqtt_topic_buf, mqtt_msg_buf);
    delay(10);
  }
}
#endif //CFOS_IN_ULTRASOUND && CFOS_OUT_MQTT

#if defined(CFOS_IN_ULTRASOUND) && defined(CFOS_NET_LORA)
void send_lora_ultrasound_status() {
    uint32_t distance = us_duration[i] / 58; 
    if(distance<=0) {
      payload[11] = (int)1; 
    } else {
      payload[11] = (int)0; 
    }
}
#endif //CFOS_IN_ULTRASOUND && CFOS_NET_LORA

#if defined(CFOS_IN_SMARTEVSE) && defined(CFOS_OUT_SERIAL)
void print_evse_status() {
  uint32_t current_time = millis();
  for(uint8_t i = 0; i < evse_pincount; i++) {
    Serial.print("ev_");
    Serial.print(evse_input[i].pin_name);
    Serial.print(':');
    if(evse_status[i] == EVSE_STATE_C) {
      Serial.print("vehicle charging");
    } else if(evse_status[i] == EVSE_STATE_B) {
      Serial.print("vehicle detected");
    } else {
      Serial.print("standby");
    }
    Serial.print(':');
    Serial.print(((uint32_t)(current_time - last_evse_change[i]))/1000);
    Serial.println(':');
  }
}
#endif //CFOS_IN_SMARTEVSE && CFOS_OUT_SERIAL

#if defined(CFOS_IN_SMARTEVSE) && defined(CFOS_OUT_MQTT)
void send_mqtt_evse_status() {
  uint32_t current_time = millis();
  for(uint8_t i = 0; i < evse_pincount; i++) {
    snprintf(mqtt_topic_buf, sizeof(mqtt_topic_buf), "CFOS/%s/ev_%s/status", chargepoint_id, evse_input[i].pin_name);
    if(evse_status[i] == EVSE_STATE_C) {
      mqtt_client.publish(mqtt_topic_buf, "vehicle charging");
    } else if(evse_status[i] == EVSE_STATE_B) {
      mqtt_client.publish(mqtt_topic_buf, "vehicle detected");
    } else {
      mqtt_client.publish(mqtt_topic_buf, "standby");
    }
    delay(10);
    snprintf(mqtt_topic_buf, sizeof(mqtt_topic_buf), "CFOS/%s/ev_%s/secs_since_last_change", chargepoint_id, evse_input[i].pin_name);
    snprintf(mqtt_msg_buf, sizeof(mqtt_msg_buf), "%d", ((uint32_t)(current_time - last_evse_change[i]))/1000);
    mqtt_client.publish(mqtt_topic_buf, mqtt_msg_buf);
    delay(10);
  }
}
#endif //CFOS_IN_SMARTEVSE && CFOS_OUT_MQTT

#if defined(CFOS_IN_SMARTEVSE) && defined(CFOS_NET_LORA)
void send_lora_evse_status() {
  uint32_t current_time = millis();
  uint32_t secs_since_last_change;
    if(evse_status[0] == EVSE_STATE_C) {
      payload[12] = 2; //vehicle charging
    } else if(evse_status[0] == EVSE_STATE_B) {
      payload[12] = 1; //vehicle detected
    } else {
      payload[12] = 0; //standby
    }
    if(evse_status[1] == EVSE_STATE_C) {
      payload[13] = 2; //vehicle charging
    } else if(evse_status[1] == EVSE_STATE_B) {
      payload[13] = 1; //vehicle detected
    } else {
      payload[13] = 0; //standby
    }  
  }
}
#endif //CFOS_IN_SMARTEVSE && CFOS_NET_LORA


inline void init_smartevse() {
  #if defined(CFOS_IN_SMARTEVSE)
  for(uint8_t i = 0; i < evse_pincount; i++) {
    // initialize read-only SoftwareSerial 
    evse_serial[i] = new SoftwareSerial(evse_input[i].pin_number, SW_SERIAL_UNUSED_PIN, false, 256);
    evse_status[i] = EVSE_STATE_A;
    last_evse_change[i] = millis();
    evse_buffer_pos[i] = 0;
    evse_serial[i]->begin(evse_input[i].baudrate);
    #if defined(CFOS_OUT_SERIAL)
      Serial.print("Configured pin number ");
      Serial.print(evse_input[i].pin_number);
      Serial.print(" as SmartEVSE input pin named ");
      Serial.print(evse_input[i].pin_name);
      Serial.print(" with baudrate ");
      Serial.println(evse_input[i].baudrate);
    #endif //CFOS_OUT_SERIAL
  }
  #endif //CFOS_IN_SMARTEVSE
}

inline void init_lora() {
  #if defined(CFOS_NET_LORA)
  #if defined(CFOS_OUT_SERIAL)
      Serial.print("Join TheThingsNetwork: ");
  #endif //CFOS_OUT_SERIAL
  debugSerial.begin(DEBUGRATE);
  loraSerial.begin(LORA_RATE); 
  while (!debugSerial && millis() < 10000);  
  ttn.showStatus();
  ttn.join(appEui, appKey); // OTAA
  #endif //CFOS_NET_LORA
}


#if defined(CFOS_IN_SMARTEVSE)
void check_evse_message(uint8_t pin) {
  if(evse_buffer_pos[pin] < 10) return;
  if(strncmp(evse_buffer[pin], "STATE ", 6) != 0 || evse_buffer[pin][7] != '-' || evse_buffer[pin][8] != '>') {
    // no match - delete everything
    evse_buffer_pos[pin] = 0;
    return;
  }
  uint32_t current_time = millis();
  // OK, we have a match for "STATE x->y" (we don't need x, only y)
  switch(evse_buffer[pin][9]) {
    case 'A':
      evse_status[pin] = EVSE_STATE_A;
      last_evse_change[pin] = current_time;
      break;
    case 'B':
      evse_status[pin] = EVSE_STATE_B;
      last_evse_change[pin] = current_time;
      break;
    case 'C':
    case 'D':
      evse_status[pin] = EVSE_STATE_C;
      last_evse_change[pin] = current_time;
      break;
    default:
      // invalid value
      evse_buffer_pos[pin] = 0;
      break;
  }
}
#endif //CFOS_IN_SMARTEVSE

// read from software serial buffers and wait for the message "STATE x->y", where x and y are A, B or C
inline void read_evse_buffers() {
  #if defined(CFOS_IN_SMARTEVSE)
  for(uint8_t i = 0; i < evse_pincount; i++) {
    while(evse_serial[i]->available()) {
      yield();
      char received = (char)evse_serial[i]->read();
      if(received == 'S') {
        // possible new STATE message starts
        evse_buffer[i][0] = 'S';
        evse_buffer_pos[i]=1;
        continue;
      }
      evse_buffer[i][evse_buffer_pos[i]++] = received;
      if(evse_buffer_pos[i] >= 10) {
        check_evse_message(i);
      }
    }
  }
  #endif //CFOS_IN_SMARTEVSE
}

