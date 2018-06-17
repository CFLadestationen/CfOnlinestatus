/**
 * CfOnlinestatus v2 - ESP-ONLY
 * Remotely reading the status of an EV charging station and sending it to an endpoint.
 * Repository: https://github.com/CFLadestationen/CfOnlinestatus
 *
 * Configuration via web interface
 * Only change here if you know what you're doing.
 * If you add a bugfix, please submit it via GitHub!
 */

#include "cfos_types.h"
#include <EEPROM.h>
#include <ESP8266WiFi.h>
#include <CRC32.h>
cfos_config cfg;

#include "SoftwareSerial.h"
SoftwareSerial* evse_serial[2];
evse_state evse_status[2];
uint32_t last_evse_change[2];
char evse_buffer[2][12];
uint8_t evse_buffer_pos[2];

#include <PubSubClient.h>
PubSubClient mqtt_client;
uint32_t last_mqtt_output;
char mqtt_topic_buf[80];
char mqtt_msg_buf[80];
bool last_mqtt_connected;
bool current_mqtt_connected;
uint32_t mqtt_update_interval; // = 1000 * mqtt_update_interval_s;

uint32_t last_sensor_update;
uint32_t sensor_update_interval; // = 1000 * sensor_update_interval_s;
uint32_t last_serial_output;
uint32_t serial_output_interval; // = 1000 * serial_output_interval_s;
uint32_t last_s0_millis[2];
uint32_t last_s0_span[2];
uint8_t  last_s0_state[2];
uint32_t impulses_since_update[2];
uint32_t impulses_in_previous_timeframe[2];

uint8_t di_value[2];

uint8_t analogvals[100];
uint8_t analogpos=0;
uint16_t analogsum = 0;
uint32_t last_nonzero_analog = 0;

uint32_t us_duration[2];

void setup() {
  Serial.begin(115200);
  while(!Serial) {}
  Serial.println();
  Serial.println();

  init_config();
  
/*  
  last_sensor_update = 0;
  init_serial();
  init_inputs();
  init_network();
  init_lora();
  init_mqtt();
  init_smartevse();
  pinMode(D1, OUTPUT);
  digitalWrite(D1, LOW);
    pinMode(D4, OUTPUT);
  digitalWrite(D4, HIGH);

#if defined(CFOS_OUT_SERIAL)
  Serial.println("CfOnlinestatus initialisation complete.");
#endif //CFOS_OUT_SERIAL */
}

void loop() {
/*  uint32_t current_time = millis();
  read_s0_inputs();
  read_evse_buffers();
  read_cp_input();
  if(((uint32_t)(current_time-last_sensor_update)) >= sensor_update_interval) {
    last_sensor_update = current_time;
    update_s0();
    update_digital_input();
    //update_analog_input();
    update_ultrasound();
  }
  ethernet_renew_dhcp();
  output_serial_interval();
  output_mqtt_interval();
  output_lora_interval(); */
}


/* TODO LoRa
#if defined(CFOS_OUT_LORA)
#if defined(ARDUINO_AVR_UNO)
const uint32_t lora_update_interval = 1000 * lora_update_interval_s;
static_assert(lora_update_interval>59999, "LoRa update interval must be 60 seconds or more");
#define DEBUGRATE 115200
#define LORA_RATE 57600
Stream *nullStream = NULL;
TheThingsNetwork ttn(loraSerial, *nullStream, freqPlan);
lora_payload payload;
uint32_t last_lora_output;
#else //not ARDUINO_AVR_UNO
#error CfOnlinestatus does not know how to use LoRa with this device!
#endif //ARDUINO_AVR_UNO
#endif //CFOS_OUT_LORA
*/



void init_config() {
  EEPROM.begin(1024);
  EEPROM.get(0, cfg);
  EEPROM.end();
  uint32_t checksum_eeprom = cfg.checksum;
  cfg.checksum = 0;
  uint32_t checksum_calculated = CRC32::calculate((uint8_t*)(&cfg), 1024);
  Serial.print(F("Checksum EEPROM: "));
  Serial.println(checksum_eeprom, 16);
  Serial.print(F("Checksum calc  : "));
  Serial.println(checksum_calculated, 16);

  if(cfg.magic_marker != 0x51EF || checksum_eeprom != checksum_calculated) {
    cfg = default_config;
    Serial.println(F("invalid config - saving new default config"));
    save_config();
  }
}

void save_config() {
  cfg.checksum = 0;
  uint32_t checksum_calculated = CRC32::calculate((uint8_t*)(&cfg), 1024);
  cfg.checksum = checksum_calculated;
  Serial.println(F("saving config"));
  EEPROM.begin(1024);
  EEPROM.put(0, cfg);
  delay(200);
  EEPROM.commit();
  EEPROM.end();
  Serial.println(F("saving config done"));
}

/*



void read_cp_input() {
  uint8_t val = (uint8_t)(analogRead(A0)/4);
  if(val > 20) {
    last_nonzero_analog = millis();
    analogsum -= analogvals[analogpos];
    analogvals[analogpos++] = val;
    analogsum += val;
    if(analogpos >= 100) {
      analogpos = 0;
    }
  } else if((uint32_t)(millis()-last_nonzero_analog) > 1000) {
    analogsum = 0;
    for(uint8_t i = 0; i < 100; i++) {
      analogvals[i]=0;
    }
  }
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
  Serial.println("LAN connection");
#endif //CFOS_NET_ETHERNET
#if defined(CFOS_OUT_LORA)
  Serial.println("LoRaWAN connection");
#endif //CFOS_OUT_LORA
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
#if defined(CFOS_NET_ETHERNET)
  #if defined(CFOS_OUT_SERIAL)
  Serial.print("Ethernet");
  Serial.print("Requesting IP from DHCP");
  #endif //CFOS_OUT_SERIAL
  Ethernet.begin(mac);
  if (Ethernet.begin(mac) == 0) {
    #if defined(CFOS_OUT_SERIAL)
    Serial.println("Failed to configure Ethernet using DHCP");
    #endif //CFOS_OUT_SERIAL
  }
#endif //CFOS_NET_ETHERNET
}


inline void init_mqtt() {
#if defined(CFOS_OUT_MQTT)
#if defined(CFOS_NET_WIFI)
  mqtt_client.setClient(wifi_client);
#endif //CFOS_NET_WIFI
#if defined(CFOS_NET_ETHERNET)
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
Serial.print("Analog sum: ");
Serial.println(analogsum);
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
#if defined(CFOS_OUT_LORA)
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

#if defined(CFOS_IN_ULTRASOUND)
  send_lora_ultrasound_status();
#endif //CFOS_IN_ULTRASOUND
#if defined(CFOS_IN_ANALOG) || defined(CFOS_IN_SMARTEVSE)
  send_lora_connection_status();
#endif //CFOS_IN_ANALOG || CFOS_IN_SMARTEVSE
#if defined(CFOS_OUT_SERIAL)
  Serial.print("Send data to TTN over LoRa: ");
#endif //CFOS_OUT_SERIAL
ttn.sendBytes((const byte*)&payload, sizeof(payload));
#endif //CFOS_OUT_LORA
}

inline void ethernet_renew_dhcp() {
#if defined(CFOS_NET_ETHERNET)
  uint32_t current_time = millis();
  if(((uint32_t)(current_time-last_ethernet_dhcp_renew)) < ethernet_dhcp_renew_interval) {
    return;
  }
  last_ethernet_dhcp_renew = current_time;
#if defined(CFOS_OUT_SERIAL)
  Serial.print("Renew Ethernet DHCP lease");
#endif //CFOS_OUT_SERIAL
  Ethernet.maintain();
#endif //CFOS_NET_ETHERNET
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
    snprintf(mqtt_msg_buf, sizeof(mqtt_msg_buf), "%" PRIu32, last_s0_span[i]);
    mqtt_client.publish(mqtt_topic_buf, mqtt_msg_buf);
    delay(10);
    snprintf(mqtt_topic_buf, sizeof(mqtt_topic_buf), "CFOS/%s/s0_%s/impulses_timeframe", chargepoint_id, s0[i].pin_name);
    snprintf(mqtt_msg_buf, sizeof(mqtt_msg_buf), "%" PRIu32, impulses_in_previous_timeframe[i]);
    mqtt_client.publish(mqtt_topic_buf, mqtt_msg_buf);
    delay(10);
    snprintf(mqtt_topic_buf, sizeof(mqtt_topic_buf), "CFOS/%s/s0_%s/secs_since_last_impulse", chargepoint_id, s0[i].pin_name);
    snprintf(mqtt_msg_buf, sizeof(mqtt_msg_buf), "%" PRIu32, ((uint32_t)(current_time - last_s0_millis[i]))/1000);
    mqtt_client.publish(mqtt_topic_buf, mqtt_msg_buf);
    delay(10);
    snprintf(mqtt_topic_buf, sizeof(mqtt_topic_buf), "CFOS/%s/s0_%s/power", chargepoint_id, s0[i].pin_name);
    snprintf(mqtt_msg_buf, sizeof(mqtt_msg_buf), "%" PRIu32, last_s0_watts(i));
    mqtt_client.publish(mqtt_topic_buf, mqtt_msg_buf);
    delay(10);
  }
}
#endif //CFOS_IN_S0 && CFOS_OUT_MQTT


#if defined(CFOS_IN_S0) && defined(CFOS_OUT_LORA)
inline void send_lora_s0_status() {
  for(uint8_t i = 0; i < s0_pincount; i++) {
    payload.s0_watts[i] = (uint8_t)(last_s0_watts(i)/200);
  }
}
#endif //CFOS_IN_S0 && CFOS_OUT_LORA


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

#if defined(CFOS_IN_DIGITAL) && defined(CFOS_OUT_LORA)
void send_lora_digital_input_status() {
  payload.digital_status = 0;
  for(uint8_t i = 0; i < di_pincount; i++) {
    if(di_value[i] == digital_input[i].on_value) {
      payload.digital_status |= 1<<i;
    }
  }
}
#endif //CFOS_IN_DIGITAL && CFOS_OUT_LORA

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
      Serial.print("charging");
    } else {
      Serial.print("detected");
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
      mqtt_client.publish(mqtt_topic_buf, "charging");
    } else {
      mqtt_client.publish(mqtt_topic_buf, "detected");
    }
    delay(10);
  }
}
#endif //CFOS_IN_ANALOG && CFOS_OUT_MQTT

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
    snprintf(mqtt_msg_buf, sizeof(mqtt_msg_buf), "%" PRIu32, us_duration[i]);
    mqtt_client.publish(mqtt_topic_buf, mqtt_msg_buf);
    delay(10);
    snprintf(mqtt_topic_buf, sizeof(mqtt_topic_buf), "CFOS/%s/us_%s/distance_cm", chargepoint_id, us_sensor[i].sensor_name);
    snprintf(mqtt_msg_buf, sizeof(mqtt_msg_buf), "%" PRIu32, distance);
    mqtt_client.publish(mqtt_topic_buf, mqtt_msg_buf);
    delay(10);
    snprintf(mqtt_topic_buf, sizeof(mqtt_topic_buf), "CFOS/%s/us_%s/object_detected", chargepoint_id, us_sensor[i].sensor_name);
    snprintf(mqtt_msg_buf, sizeof(mqtt_msg_buf), "%s", (distance>us_sensor[i].free_distance || distance==0)?"no":"yes");
    mqtt_client.publish(mqtt_topic_buf, mqtt_msg_buf);
    delay(10);
  }
}
#endif //CFOS_IN_ULTRASOUND && CFOS_OUT_MQTT

#if defined(CFOS_IN_ULTRASOUND) && defined(CFOS_OUT_LORA)
void send_lora_ultrasound_status() {
  payload.us_status = 0;
  for(uint8_t i = 0; i < us_pincount; i++) {
    uint32_t distance = us_duration[i] / 58;
    if(distance<=us_sensor[i].free_distance && distance>0) {
      payload.us_status |= 1<<i;
    }
  }
}
#endif //CFOS_IN_ULTRASOUND && CFOS_OUT_LORA

#if defined(CFOS_IN_SMARTEVSE) && defined(CFOS_OUT_SERIAL)
void print_evse_status() {
  uint32_t current_time = millis();
  for(uint8_t i = 0; i < evse_pincount; i++) {
    Serial.print("ev_");
    Serial.print(evse_input[i].pin_name);
    Serial.print(':');
    if(evse_status[i] == EVSE_STATE_C) {
      Serial.print("charging");
    } else if(evse_status[i] == EVSE_STATE_B) {
      Serial.print("detected");
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
      mqtt_client.publish(mqtt_topic_buf, "charging");
    } else if(evse_status[i] == EVSE_STATE_B) {
      mqtt_client.publish(mqtt_topic_buf, "detected");
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

#if (defined(CFOS_IN_ANALOG) || defined(CFOS_IN_SMARTEVSE)) && defined(CFOS_OUT_LORA)
void send_lora_connection_status() {
  uint8_t n = 0;
  payload.evse_status = 0;
#if defined(CFOS_IN_ANALOG)
  for(uint8_t i = 0; i < ai_pincount; i++) {
    if(ai_value[i] > analog_input[i].on_value) {      
      payload.evse_status |= EVSE_STATE_A<<(2*n);
    } else if(ai_value[i] <= analog_input[i].off_value) {     
      payload.evse_status |= EVSE_STATE_C<<(2*n);
    } else {      
      payload.evse_status |= EVSE_STATE_B<<(2*n);
    }
    n++;
  }
#endif//CFOS_IN_ANALOG
#if defined(CFOS_IN_SMARTEVSE)
  for(uint8_t i = 0; i < evse_pincount; i++) {
    payload.evse_status |= evse_status[i]<<(2*n);
    n++;
  }
#endif//CFOS_IN_SMARTEVSE
}
#endif //(CFOS_IN_ANALOG||CFOS_IN_SMARTEVSE) && CFOS_OUT_LORA


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
  #if defined(CFOS_OUT_LORA)
  #if defined(CFOS_OUT_SERIAL)
      Serial.print("Join TheThingsNetwork: ");
  #endif //CFOS_OUT_SERIAL
  //debugSerial.begin(DEBUGRATE);
  loraSerial.begin(LORA_RATE); 
  //while (!debugSerial && millis() < 10000);  
  ttn.showStatus();
  ttn.join(appEui, appKey); // OTAA
  #endif //CFOS_OUT_LORA
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
*/

