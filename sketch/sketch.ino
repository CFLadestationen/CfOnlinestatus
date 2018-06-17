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
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
WiFiClient wifi_client;
ESP8266WebServer webserver(80);

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
uint32_t last_mqtt_output = 0;
char mqtt_topic_buf[80];
char mqtt_msg_buf[80];
bool last_mqtt_connected = 0;
bool current_mqtt_connected = 0;
uint32_t mqtt_update_interval;

/* TODO LoRa for ESP8266
#include <TheThingsNetwork.h>
uint32_t lora_update_interval;
Stream *nullStream = NULL;
TheThingsNetwork ttn(loraSerial, *nullStream, freqPlan);
lora_payload payload;
uint32_t last_lora_output;
*/

uint32_t last_sensor_update;
uint32_t sensor_update_interval;
uint32_t last_serial_output;
uint32_t serial_output_interval;
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
uint8_t cp_value;

uint32_t us_duration[2];

bool forgetWifi = false;
bool configured = false;

void setup() {
  pinMode(D5, INPUT_PULLUP);
  pinMode(D8, INPUT);
  forgetWifi = (digitalRead(D5) == LOW && digitalRead(D8) == HIGH);
  init_config();

  if(configured) {
    last_sensor_update = 0;
    init_serial();
    init_inputs();
    init_network();
    /* TODO LoRa for ESP8266
    init_lora();
    */
    init_mqtt();
    init_smartevse();
    Serial.println("CfOnlinestatus initialisation complete");
  } else {
    Serial.begin(115200);
    while(!Serial) {}
    Serial.println("CfOnlinestatus initialisation complete (unconfigured)");
    init_network();
  }
}

void loop() {
  uint32_t current_time = millis();
  if(configured) {
    read_s0_inputs();
    read_evse_buffers();
    read_cp_input();
    if(((uint32_t)(current_time-last_sensor_update)) >= sensor_update_interval) {
      last_sensor_update = current_time;
      update_s0();
      update_digital_input();
      update_cp();
      update_ultrasound();
    }
    output_serial_interval();
    output_mqtt_interval();
    /* TODO LoRa for ESP8266
    output_lora_interval();
    */
  } else {
    delay(1);
  }
  webserver.handleClient();
}

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
  if(forgetWifi) {
    cfg.wifi_ssid[0] = 0;
    cfg.wifi_key[0] = 0;
  }

  if(cfg.magic_marker != 0x51EF || checksum_eeprom != checksum_calculated) {
    cfg = default_config;
    if(forgetWifi) {
      cfg.wifi_ssid[0] = 0;
      cfg.wifi_key[0] = 0;
    }
    Serial.println(F("invalid config - saving new default config"));
    save_config();
  } else {
    configured = true;
    update_config_values();
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
  update_config_values();
}

void update_config_values() {
  mqtt_update_interval = 1000 * cfg.mqtt_update_interval_s;
  sensor_update_interval = 1000 * cfg.sensor_update_interval_s;
  serial_output_interval = 1000 * cfg.serial_output_interval_s;
  /* TODO LoRa for ESP8266 
   lora_update_interval = 1000 * lora_update_interval_s;
   */
}

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
  Serial.begin(cfg.serial_baudrate);
  // wait for serial connection
  while(!Serial) {}
  last_serial_output = 0;
  Serial.print("\n\nCfOnlinestatus id ");
  Serial.print(cfg.chargepoint_id);
  Serial.println(" starting with following features:");
  Serial.print("Update interval: ");
  Serial.print(sensor_update_interval);
  Serial.println(" milliseconds");
  Serial.print("Serial output interval: ");
  Serial.print(serial_output_interval);
  Serial.println(" milliseconds");
  Serial.print(cfg.input_count_s0);
  Serial.println(" S0 pin(s)");
  Serial.print(cfg.input_count_di);
  Serial.println(" digital input pin(s)");
  Serial.print(cfg.input_count_cp);
  Serial.println(" CP input pin(s)");
  Serial.print(cfg.input_count_us);
  Serial.println(" HC-SR04 ultrasound sensor(s)");
  Serial.print(cfg.input_count_ev);
  Serial.println(" SmartEVSE serial input(s)");
  Serial.println("WiFi connection");
}

inline void init_inputs() {
  for(uint8_t i = 0; i < cfg.input_count_s0; i++) {
    pinMode(cfg.s0[i].pin_number, cfg.s0[i].pin_mode);
    last_s0_millis[i] = 0;
    last_s0_span[i] = 0;
    last_s0_state[i] = cfg.s0[i].off_value;
    impulses_since_update[i] = 0;
    Serial.print("Configured pin number ");
    Serial.print(cfg.s0[i].pin_number);
    Serial.print(" as S0 pin named ");
    Serial.println(cfg.s0[i].pin_name);
  }
  for(uint8_t i = 0; i < cfg.input_count_di; i++) {
    pinMode(cfg.di[i].pin_number, cfg.di[i].pin_mode);
    Serial.print("Configured pin number ");
    Serial.print(cfg.di[i].pin_number);
    Serial.print(" as digital input pin named ");
    Serial.println(cfg.di[i].pin_name);
  }
  if(cfg.input_count_cp > 0) {
    pinMode(cfg.cp.pin_number, cfg.cp.pin_mode);
    Serial.print("Configured pin number ");
    Serial.print(cfg.cp.pin_number);
    Serial.print(" as CP input pin named ");
    Serial.print(cfg.cp.pin_name);
    Serial.print(" with low threshold ");
    Serial.print(cfg.cp.off_value);
    Serial.print(" and high threshold ");
    Serial.println(cfg.cp.on_value);
  }
  for(uint8_t i = 0; i < cfg.input_count_us; i++) {
    pinMode(cfg.us[i].echo_pin, INPUT);
    pinMode(cfg.us[i].trigger_pin, OUTPUT);
    digitalWrite(cfg.us[i].trigger_pin, cfg.us[i].trigger_off);
    Serial.print("Configured ultrasound sensor named ");
    Serial.print(cfg.us[i].sensor_name);
    Serial.print(" with echo pin ");
    Serial.print(cfg.us[i].echo_pin);
    Serial.print(", trigger pin ");
    Serial.print(cfg.us[i].trigger_pin);
    Serial.print(" and timeout ");
    Serial.print(cfg.us[i].timeout_us);
    Serial.println(" us");
  }
}

String p2n(uint8_t pin) {
  switch(pin) {
    case D0:
      return "D0";
      break;
    case D1:
      return "D1";
      break;
    case D2:
      return "D2";
      break;
    case D3:
      return "D3";
      break;
    case D4:
      return "D4";
      break;
    case D5:
      return "D5";
      break;
    case D6:
      return "D6";
      break;
    case D7:
      return "D7";
      break;
    case D8:
      return "D8";
      break;
    case A0:
      return "A0";
      break;
  }
  return "INVALID";
}
uint8_t n2p(String name) {
  return 0;
}

inline void init_network() {
  if(configured && strlen(cfg.wifi_ssid) > 0) {
    WiFi.mode(WIFI_STA);
    WiFi.begin(cfg.wifi_ssid, cfg.wifi_key);
      int wifi_tries = 20;
    while(wifi_tries-->0 && WiFi.status() != WL_CONNECTED) {
      delay(500);
    }
    Serial.print("Configured Wifi with SSID ");
    Serial.print(cfg.wifi_ssid);
    Serial.print(" and key ");
    Serial.print(cfg.wifi_key);
    Serial.print(" -- status is currently ");
    if(WiFi.status() == WL_CONNECTED) {
      Serial.print("connected with local IP ");
      Serial.println(WiFi.localIP());
    } else {
      Serial.println(WiFi.status());
    }
  } else {
    WiFi.mode(WIFI_AP);
    WiFi.softAP("CFOSConfig");
  }
  webserver.on("/", ConfigPage);
  webserver.begin();
  MDNS.begin("cfos");
  MDNS.addService("http", "tcp", 80);
}

void ConfigPage() {
  String page;
  page += "";
  webserver.send(200, "text/plain", "Hallo");
}


inline void init_mqtt() {
  mqtt_client.setClient(wifi_client);
  mqtt_client.setServer(cfg.mqtt_server, cfg.mqtt_port);
}

inline void output_serial_interval() {
  uint32_t current_time = millis();
  if(((uint32_t)(current_time-last_serial_output)) < serial_output_interval) {
    return;
  }
  last_serial_output = current_time;
  Serial.print("id_");
  Serial.print(cfg.chargepoint_id);
  Serial.print(':');
  Serial.print(current_time);
  Serial.println(':');
  print_s0_status();
  print_digital_input_status();
  print_cp_status();
  print_ultrasound_status();
  print_evse_status();
}

inline void output_mqtt_interval() {
  last_mqtt_connected = current_mqtt_connected;
  current_mqtt_connected = mqtt_client.loop();
  if(!current_mqtt_connected) {
    // start reconnecting and wait 15s
    last_mqtt_output = millis()-15000;
    snprintf(mqtt_topic_buf, sizeof(mqtt_topic_buf), "CFOS/%s/status", cfg.chargepoint_id);
    mqtt_client.connect(cfg.chargepoint_id, cfg.mqtt_username, cfg.mqtt_password, mqtt_topic_buf, 0, 1, "offline");
    delay(10);
    return;
  }
  if(current_mqtt_connected && !last_mqtt_connected) {
    // client has just successfully reconnected
    snprintf(mqtt_topic_buf, sizeof(mqtt_topic_buf), "CFOS/%s/status", cfg.chargepoint_id);
    mqtt_client.publish(mqtt_topic_buf, "online");
  }
  uint32_t current_time = millis();
  if(((uint32_t)(current_time-last_mqtt_output)) < mqtt_update_interval) {
    return;
  }
  last_mqtt_output = current_time;
  send_mqtt_s0_status();
  send_mqtt_digital_input_status();
  send_mqtt_cp_status();
  send_mqtt_ultrasound_status();
  send_mqtt_evse_status();
}

/* TODO LoRa for ESP8266
inline void output_lora_interval() {
  uint32_t current_time = millis();
  if(((uint32_t)(current_time-last_lora_output)) < lora_update_interval) {
    return;
  }
  last_lora_output = current_time;
  send_lora_s0_status();
  send_lora_digital_input_status();
  send_lora_ultrasound_status();
  send_lora_connection_status();
  Serial.print("Send data to TTN over LoRa: ");
  ttn.sendBytes((const byte*)&payload, sizeof(payload));
} */

inline void read_s0_inputs() {
  uint32_t current_time = millis();
  for(uint8_t i = 0; i < cfg.input_count_s0; i++) {
    uint8_t value = digitalRead(cfg.s0[i].pin_number);
    if(value == cfg.s0[i].on_value && last_s0_state[i] == cfg.s0[i].off_value) {
      // change detected
      last_s0_span[i] = (uint32_t)(current_time - last_s0_millis[i]); // cast to avoid rollover problems
      last_s0_millis[i] = current_time;
      impulses_since_update[i]++;
    }
    last_s0_state[i] = value;
  }
}

inline void update_s0() {
  for(uint8_t i = 0; i < cfg.input_count_s0; i++) {
    impulses_in_previous_timeframe[i] = impulses_since_update[i];
    impulses_since_update[i] = 0;
  }
}

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
  return (((impulses_in_previous_timeframe[pin] * 3600) / cfg.sensor_update_interval_s) * 1000) / cfg.s0_impulses_per_kWh[pin];
}

void print_s0_status() {
  uint32_t current_time = millis();
  for(uint8_t i = 0; i < cfg.input_count_s0; i++) {
    Serial.print("s0_");
    Serial.print(cfg.s0[i].pin_name);
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

inline void send_mqtt_s0_status() {
  uint32_t current_time = millis();
  for(uint8_t i = 0; i < cfg.input_count_s0; i++) {
    snprintf(mqtt_topic_buf, sizeof(mqtt_topic_buf), "CFOS/%s/s0_%s/lastspan", cfg.chargepoint_id, cfg.s0[i].pin_name);
    snprintf(mqtt_msg_buf, sizeof(mqtt_msg_buf), "%" PRIu32, last_s0_span[i]);
    mqtt_client.publish(mqtt_topic_buf, mqtt_msg_buf);
    delay(10);
    snprintf(mqtt_topic_buf, sizeof(mqtt_topic_buf), "CFOS/%s/s0_%s/impulses_timeframe", cfg.chargepoint_id, cfg.s0[i].pin_name);
    snprintf(mqtt_msg_buf, sizeof(mqtt_msg_buf), "%" PRIu32, impulses_in_previous_timeframe[i]);
    mqtt_client.publish(mqtt_topic_buf, mqtt_msg_buf);
    delay(10);
    snprintf(mqtt_topic_buf, sizeof(mqtt_topic_buf), "CFOS/%s/s0_%s/secs_since_last_impulse", cfg.chargepoint_id, cfg.s0[i].pin_name);
    snprintf(mqtt_msg_buf, sizeof(mqtt_msg_buf), "%" PRIu32, ((uint32_t)(current_time - last_s0_millis[i]))/1000);
    mqtt_client.publish(mqtt_topic_buf, mqtt_msg_buf);
    delay(10);
    snprintf(mqtt_topic_buf, sizeof(mqtt_topic_buf), "CFOS/%s/s0_%s/power", cfg.chargepoint_id, cfg.s0[i].pin_name);
    snprintf(mqtt_msg_buf, sizeof(mqtt_msg_buf), "%" PRIu32, last_s0_watts(i));
    mqtt_client.publish(mqtt_topic_buf, mqtt_msg_buf);
    delay(10);
  }
}

/* TODO LoRa for ESP8266
inline void send_lora_s0_status() {
  for(uint8_t i = 0; i < cfg.input_count_s0; i++) {
    payload.s0_watts[i] = (uint8_t)(last_s0_watts(i)/200);
  }
}
*/

inline void update_digital_input() {
  for(uint8_t i = 0; i < cfg.input_count_di; i++) {
    di_value[i] = digitalRead(cfg.di[i].pin_number);
  }
}

void print_digital_input_status() {
  for(uint8_t i = 0; i < cfg.input_count_di; i++) {
    Serial.print("di_");
    Serial.print(cfg.di[i].pin_name);
    Serial.print(':');
    if(di_value[i] == cfg.di[i].on_value) {
      Serial.print("on");
    } else {
      Serial.print("off");
    }
    Serial.println(":");
  }
}

void send_mqtt_digital_input_status() {
  for(uint8_t i = 0; i < cfg.input_count_di; i++) {
    snprintf(mqtt_topic_buf, sizeof(mqtt_topic_buf), "CFOS/%s/di_%s/status", cfg.chargepoint_id, cfg.di[i].pin_name);
    if(di_value[i] == cfg.di[i].on_value) {
      mqtt_client.publish(mqtt_topic_buf, "on");
    } else {
      mqtt_client.publish(mqtt_topic_buf, "off");
    }
    delay(10);
  }
}

/* TODO LoRa for ESP8266
void send_lora_digital_input_status() {
  payload.digital_status = 0;
  for(uint8_t i = 0; i < cfg.input_count_di; i++) {
    if(di_value[i] == cfg.di[i].on_value) {
      payload.digital_status |= 1<<i;
    }
  }
}
*/

inline void update_cp() {
  cp_value = analogsum / 100;
}

void print_cp_status() {
  if(cfg.input_count_cp == 0) return;
  Serial.print("cp_");
  Serial.print(cfg.cp.pin_name);
  Serial.print(':');
  Serial.print(cp_value);
  Serial.print(':');
  if(cp_value <= 40) {
    Serial.print("error");
  } else if(cp_value > cfg.cp.on_value) {
    Serial.print("standby");
  } else if(cp_value <= cfg.cp.off_value) {
    Serial.print("charging");
  } else {
    Serial.print("detected");
  }
  Serial.println(":");
}

void send_mqtt_cp_status() {
  if(cfg.input_count_cp == 0) return;
  snprintf(mqtt_topic_buf, sizeof(mqtt_topic_buf), "CFOS/%s/cp_%s/value", cfg.chargepoint_id, cfg.cp.pin_name);
  snprintf(mqtt_msg_buf, sizeof(mqtt_msg_buf), "%d", cp_value);
  mqtt_client.publish(mqtt_topic_buf, mqtt_msg_buf);
  delay(10);
  snprintf(mqtt_topic_buf, sizeof(mqtt_topic_buf), "CFOS/%s/cp_%s/meaning", cfg.chargepoint_id, cfg.cp.pin_name);
  if(cp_value < 40) {
    mqtt_client.publish(mqtt_topic_buf, "error");
  } else if(cp_value > cfg.cp.on_value) {
    mqtt_client.publish(mqtt_topic_buf, "standby");
  } else if(cp_value <= cfg.cp.off_value) {
    mqtt_client.publish(mqtt_topic_buf, "charging");
  } else {
    mqtt_client.publish(mqtt_topic_buf, "detected");
  }
  delay(10);
}

inline void update_ultrasound() {
  for(uint8_t i = 0; i < cfg.input_count_us; i++) {
    digitalWrite(cfg.us[i].trigger_pin, cfg.us[i].trigger_off);
    delayMicroseconds(10);
    digitalWrite(cfg.us[i].trigger_pin, cfg.us[i].trigger_on);
    delayMicroseconds(10);
    digitalWrite(cfg.us[i].trigger_pin, cfg.us[i].trigger_off);
    us_duration[i] = pulseIn(cfg.us[i].echo_pin, cfg.us[i].echo_on, cfg.us[i].timeout_us);
  }
}

void print_ultrasound_status() {
  for(uint8_t i = 0; i < cfg.input_count_us; i++) {
    uint32_t distance = us_duration[i] / 58;
    Serial.print("us_");
    Serial.print(cfg.us[i].sensor_name);
    Serial.print(':');
    Serial.print(us_duration[i]);
    Serial.print(':');
    Serial.print(distance);
    Serial.println(':');
  }
}

void send_mqtt_ultrasound_status() {
  for(uint8_t i = 0; i < cfg.input_count_us; i++) {
    uint32_t distance = us_duration[i] / 58;
    snprintf(mqtt_topic_buf, sizeof(mqtt_topic_buf), "CFOS/%s/us_%s/duration_microsecs", cfg.chargepoint_id, cfg.us[i].sensor_name);
    snprintf(mqtt_msg_buf, sizeof(mqtt_msg_buf), "%" PRIu32, us_duration[i]);
    mqtt_client.publish(mqtt_topic_buf, mqtt_msg_buf);
    delay(10);
    snprintf(mqtt_topic_buf, sizeof(mqtt_topic_buf), "CFOS/%s/us_%s/distance_cm", cfg.chargepoint_id, cfg.us[i].sensor_name);
    snprintf(mqtt_msg_buf, sizeof(mqtt_msg_buf), "%" PRIu32, distance);
    mqtt_client.publish(mqtt_topic_buf, mqtt_msg_buf);
    delay(10);
    snprintf(mqtt_topic_buf, sizeof(mqtt_topic_buf), "CFOS/%s/us_%s/object_detected", cfg.chargepoint_id, cfg.us[i].sensor_name);
    snprintf(mqtt_msg_buf, sizeof(mqtt_msg_buf), "%s", (distance>cfg.us[i].free_distance || distance==0)?"no":"yes");
    mqtt_client.publish(mqtt_topic_buf, mqtt_msg_buf);
    delay(10);
  }
}

/* TODO LoRa for ESP8266
void send_lora_ultrasound_status() {
  payload.us_status = 0;
  for(uint8_t i = 0; i < cfg.input_count_us; i++) {
    uint32_t distance = us_duration[i] / 58;
    if(distance<=cfg.us[i].free_distance && distance>0) {
      payload.us_status |= 1<<i;
    }
  }
}
*/

void print_evse_status() {
  uint32_t current_time = millis();
  for(uint8_t i = 0; i < cfg.input_count_ev; i++) {
    Serial.print("ev_");
    Serial.print(cfg.ev[i].pin_name);
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

void send_mqtt_evse_status() {
  uint32_t current_time = millis();
  for(uint8_t i = 0; i < cfg.input_count_ev; i++) {
    snprintf(mqtt_topic_buf, sizeof(mqtt_topic_buf), "CFOS/%s/ev_%s/status", cfg.chargepoint_id, cfg.ev[i].pin_name);
    if(evse_status[i] == EVSE_STATE_C) {
      mqtt_client.publish(mqtt_topic_buf, "charging");
    } else if(evse_status[i] == EVSE_STATE_B) {
      mqtt_client.publish(mqtt_topic_buf, "detected");
    } else {
      mqtt_client.publish(mqtt_topic_buf, "standby");
    }
    delay(10);
    snprintf(mqtt_topic_buf, sizeof(mqtt_topic_buf), "CFOS/%s/ev_%s/secs_since_last_change", cfg.chargepoint_id, cfg.ev[i].pin_name);
    snprintf(mqtt_msg_buf, sizeof(mqtt_msg_buf), "%d", ((uint32_t)(current_time - last_evse_change[i]))/1000);
    mqtt_client.publish(mqtt_topic_buf, mqtt_msg_buf);
    delay(10);
  }
}

/* TODO LoRa for ESP8266
void send_lora_connection_status() {
  uint8_t n = 0;
  payload.evse_status = 0;
  for(uint8_t i = 0; i < cfg.input_count_cp; i++) {
    if(cp_value[i] > cfg.cp[i].on_value) {      
      payload.evse_status |= EVSE_STATE_A<<(2*n);
    } else if(cp_value[i] <= cfg.cp[i].off_value) {     
      payload.evse_status |= EVSE_STATE_C<<(2*n);
    } else {      
      payload.evse_status |= EVSE_STATE_B<<(2*n);
    }
    n++;
  }
  for(uint8_t i = 0; i < cfg.input_count_ev; i++) {
    payload.evse_status |= evse_status[i]<<(2*n);
    n++;
  }
}
*/

inline void init_smartevse() {
  for(uint8_t i = 0; i < cfg.input_count_ev; i++) {
    // initialize read-only SoftwareSerial 
    evse_serial[i] = new SoftwareSerial(cfg.ev[i].pin_number, SW_SERIAL_UNUSED_PIN, false, 256);
    evse_status[i] = EVSE_STATE_A;
    last_evse_change[i] = millis();
    evse_buffer_pos[i] = 0;
    evse_serial[i]->begin(cfg.ev[i].baudrate);
      Serial.print("Configured pin number ");
      Serial.print(cfg.ev[i].pin_number);
      Serial.print(" as SmartEVSE input pin named ");
      Serial.print(cfg.ev[i].pin_name);
      Serial.print(" with baudrate ");
      Serial.println(cfg.ev[i].baudrate);
  }
}

/* TODO LoRa for ESP8266
inline void init_lora() {
  Serial.print("Join TheThingsNetwork: ");
  //debugSerial.begin(DEBUGRATE);
  loraSerial.begin(LORA_RATE); 
  //while (!debugSerial && millis() < 10000);  
  ttn.showStatus();
  ttn.join(appEui, appKey); // OTAA
}
*/

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

// read from software serial buffers and wait for the message "STATE x->y", where x and y are A, B or C
inline void read_evse_buffers() {
  for(uint8_t i = 0; i < cfg.input_count_ev; i++) {
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
}

