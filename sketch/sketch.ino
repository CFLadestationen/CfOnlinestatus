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
  webserver.on("/write", WriteSettings);
  webserver.begin();
  MDNS.begin("cfos");
  MDNS.addService("http", "tcp", 80);
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

void WriteSettings() {
  if(!webserver.hasArg("really") || webserver.arg("really") != "yes") {
    webserver.send(200, "text/plain", "error writing settings");
    return;
  }
  if(webserver.hasArg("factory_default") && webserver.arg("factory_default") == "FACTORYDEFAULT") {
    EEPROM.begin(1024);
    for(int i = 0; i < 1024; i++) {
      EEPROM.write(i, 0);
    }
    delay(200);
    EEPROM.commit();
    EEPROM.end();
    webserver.send(200, "text/plain", "writing factory defaults - CfOnlinestatus is rebooting");
    for(int i = 0; i < 100; i++) {
      webserver.handleClient();
      delay(10);
    }
    ESP.restart();
    return;
  }

  strncpy(cfg.chargepoint_id, webserver.arg("chargepoint_id").c_str(), sizeof(cfg.chargepoint_id));
  cfg.sensor_update_interval_s = webserver.arg("sensor_update_interval_s").toInt();
  cfg.serial_baudrate = webserver.arg("serial_baudrate").toInt();
  cfg.serial_output_interval_s = webserver.arg("serial_output_interval_s").toInt();
  strncpy(cfg.wifi_ssid, webserver.arg("wifi_ssid").c_str(), sizeof(cfg.wifi_ssid));
  strncpy(cfg.wifi_key, webserver.arg("wifi_key").c_str(), sizeof(cfg.wifi_key));
  strncpy(cfg.mqtt_server, webserver.arg("mqtt_server").c_str(), sizeof(cfg.mqtt_server));
  cfg.mqtt_port = webserver.arg("mqtt_port").toInt();
  strncpy(cfg.mqtt_username, webserver.arg("mqtt_username").c_str(), sizeof(cfg.mqtt_username));
  strncpy(cfg.mqtt_password, webserver.arg("mqtt_password").c_str(), sizeof(cfg.mqtt_password));
  cfg.mqtt_update_interval_s = webserver.arg("mqtt_update_interval_s").toInt();
  cfg.input_count_s0 = webserver.arg("input_count_s0").toInt();
  strncpy(cfg.s0[0].pin_name, webserver.arg("s0_0_pin_name").c_str(), sizeof(cfg.s0[0].pin_name));
  cfg.s0[0].pin_number = name2pin(webserver.arg("s0_0_pin_number"));
  cfg.s0[0].on_value = name2hilo(webserver.arg("s0_0_on_value"));
  cfg.s0[0].off_value = name2hilo(webserver.arg("s0_0_off_value"));
  cfg.s0[0].pin_mode = name2input(webserver.arg("s0_0_pinmode"));
  cfg.s0_impulses_per_kWh[0] = webserver.arg("s0_0_impulses_per_kWh").toInt();
  strncpy(cfg.s0[1].pin_name, webserver.arg("s0_1_pin_name").c_str(), sizeof(cfg.s0[1].pin_name));
  cfg.s0[1].pin_number = name2pin(webserver.arg("s0_1_pin_number"));
  cfg.s0[1].on_value = name2hilo(webserver.arg("s0_1_on_value"));
  cfg.s0[1].off_value = name2hilo(webserver.arg("s0_1_off_value"));
  cfg.s0[1].pin_mode = name2input(webserver.arg("s0_1_pinmode"));
  cfg.s0_impulses_per_kWh[1] = webserver.arg("s0_1_impulses_per_kWh").toInt();
  cfg.input_count_di = webserver.arg("input_count_di").toInt();
  strncpy(cfg.di[0].pin_name, webserver.arg("di_0_pin_name").c_str(), sizeof(cfg.di[0].pin_name));
  cfg.di[0].pin_number = name2pin(webserver.arg("di_0_pin_number"));
  cfg.di[0].on_value = name2hilo(webserver.arg("di_0_on_value"));
  cfg.di[0].off_value = name2hilo(webserver.arg("di_0_off_value"));
  cfg.di[0].pin_mode = name2input(webserver.arg("di_0_pinmode"));
  strncpy(cfg.di[1].pin_name, webserver.arg("di_1_pin_name").c_str(), sizeof(cfg.di[1].pin_name));
  cfg.di[1].pin_number = name2pin(webserver.arg("di_1_pin_number"));
  cfg.di[1].on_value = name2hilo(webserver.arg("di_1_on_value"));
  cfg.di[1].off_value = name2hilo(webserver.arg("di_1_off_value"));
  cfg.di[1].pin_mode = name2input(webserver.arg("di_1_pinmode"));
  cfg.input_count_cp = webserver.arg("input_count_cp").toInt();
  cfg.input_count_us = webserver.arg("input_count_us").toInt();
  strncpy(cfg.us[0].sensor_name, webserver.arg("us_0_pin_name").c_str(), sizeof(cfg.us[0].sensor_name));
  cfg.us[0].trigger_pin = name2pin(webserver.arg("us_0_trigger_pin"));
  cfg.us[0].echo_pin = name2pin(webserver.arg("us_0_echo_pin"));
  cfg.us[0].trigger_on = name2hilo(webserver.arg("us_0_trigger_on"));
  cfg.us[0].trigger_off = name2hilo(webserver.arg("us_0_trigger_off"));
  cfg.us[0].echo_on = name2hilo(webserver.arg("us_0_echo_on"));
  cfg.us[0].timeout_us = webserver.arg("us_0_timeout_us").toInt();
  cfg.us[0].free_distance = webserver.arg("us_0_free_distance").toInt();
  strncpy(cfg.us[1].sensor_name, webserver.arg("us_1_pin_name").c_str(), sizeof(cfg.us[1].sensor_name));
  cfg.us[1].trigger_pin = name2pin(webserver.arg("us_1_trigger_pin"));
  cfg.us[1].echo_pin = name2pin(webserver.arg("us_1_echo_pin"));
  cfg.us[1].trigger_on = name2hilo(webserver.arg("us_1_trigger_on"));
  cfg.us[1].trigger_off = name2hilo(webserver.arg("us_1_trigger_off"));
  cfg.us[1].echo_on = name2hilo(webserver.arg("us_1_echo_on"));
  cfg.us[1].timeout_us = webserver.arg("us_1_timeout_us").toInt();
  cfg.us[1].free_distance = webserver.arg("us_1_free_distance").toInt();
  cfg.input_count_ev = webserver.arg("input_count_ev").toInt();
  strncpy(cfg.ev[0].pin_name, webserver.arg("ev_0_pin_name").c_str(), sizeof(cfg.ev[0].pin_name));
  cfg.ev[0].pin_number = name2pin(webserver.arg("ev_0_pin_number"));
  cfg.ev[0].baudrate = name2pin(webserver.arg("ev_0_baudrate"));
  strncpy(cfg.ev[1].pin_name, webserver.arg("ev_1_pin_name").c_str(), sizeof(cfg.ev[1].pin_name));
  cfg.ev[1].pin_number = name2pin(webserver.arg("ev_1_pin_number"));
  cfg.ev[1].baudrate = name2pin(webserver.arg("ev_1_baudrate"));
  save_config();
  webserver.send(200, "text/plain", "config written - CfOnlinestatus is rebooting");
  for(int i = 0; i < 100; i++) {
    webserver.handleClient();
    delay(10);
  }
  ESP.restart();
  return;
}

void ConfigPage() {
  String page = "<!DOCTYPE html>"
"<html lang=\"de\">"
"  <head>"
"    <meta charset=\"utf-8\" />"
"    <meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\" />"
"    <title>"
"CfOnlinestatus configuration dialog"
"  </title>"
"  </head>"
"  <body>"
"   <h1>CfOnlinestatus configuration dialog</h1>"
"   <form action=\"/write\" method=\"POST\">"
"    <input type=\"hidden\" id=\"really\" name=\"really\" value=\"yes\" />"
"    <button type=\"reset\">Reset HTML form to start value</button>"
" <br />"
" <fieldset>"
"   <legend>General settings</legend>"
"    <label for=\"chargepoint_id\">CFOS chargepoint_id</label>"
"    <input type=\"text\" id=\"chargepoint_id\" name=\"chargepoint_id\" value=\"%%%chargepoint_id%%%\">"
" <br />"
" <label for=\"sensor_update_interval_s\">Sensor update interval (seconds)</label>"
"    <input type=\"text\" id=\"sensor_update_interval_s\" name=\"sensor_update_interval_s\" value=\"%%%sensor_update_interval_s%%%\">"
" <br />"
" <label for=\"serial_baudrate\">Serial baudrate</label>"
"    <input type=\"text\" id=\"serial_baudrate\" name=\"serial_baudrate\" value=\"%%%serial_baudrate%%%\">"
" <br />"
" <label for=\"serial_output_interval_s\">Serial output interval (seconds)</label>"
"    <input type=\"text\" id=\"serial_output_interval_s\" name=\"serial_output_interval_s\" value=\"%%%serial_output_interval_s%%%\">"
" <br />"
" <label for=\"factory_default\">Type FACTORYDEFAULT in this field and click on update to reset the complete configuration</label>"
"    <input type=\"text\" id=\"factory_default\" name=\"factory_default\" value=\"\">"
" <br />"
" </fieldset>"
" <fieldset>"
"   <legend>WiFi Settings</legend>"
" <label for=\"wifi_ssid\">WiFi SSID</label>"
"    <input type=\"text\" id=\"wifi_ssid\" name=\"wifi_ssid\" value=\"%%%wifi_ssid%%%\">"
" <br />"
" <label for=\"wifi_key\">WiFi Password</label>"
"    <input type=\"text\" id=\"wifi_key\" name=\"wifi_key\" value=\"%%%wifi_key%%%\">"
" <br />"
" </fieldset>"
""
" "
"<fieldset>"
"   <legend>MQTT settings</legend>"
" <label for=\"mqtt_server\">MQTT Servername or IP</label>"
"    <input type=\"text\" id=\"mqtt_server\" name=\"mqtt_server\" value=\"%%%mqtt_server%%%\">"
" <br />"
" <label for=\"mqtt_port\">MQTT port number</label>"
"    <input type=\"text\" id=\"mqtt_port\" name=\"mqtt_port\" value=\"%%%mqtt_port%%%\">"
" <br />"
" <label for=\"mqtt_username\">MQTT username</label>"
"    <input type=\"text\" id=\"mqtt_username\" name=\"mqtt_username\" value=\"%%%mqtt_username%%%\">"
" <br />"
" <label for=\"mqtt_password\">MQTT password (or empty)</label>"
"    <input type=\"text\" id=\"mqtt_password\" name=\"mqtt_password\" value=\"%%%mqtt_password%%%\">"
" <br />"
" <label for=\"mqtt_update_interval_s\">MQTT output interval (seconds)</label>"
"    <input type=\"text\" id=\"mqtt_update_interval_s\" name=\"mqtt_update_interval_s\" value=\"%%%mqtt_update_interval_s%%%\">"
" <br />"
"</fieldset>"
" "
" "
" <fieldset>"
"   <legend>S0 input count</legend>"
"   <input type=\"radio\" id=\"input_count_s0_0\" name=\"input_count_s0\" value=\"0\" %%%input_count_s0_0%%%>"
"   <label for=\"input_count_s0_0\"> no S0 input</label>"
"   <input type=\"radio\" id=\"input_count_s0_1\" name=\"input_count_s0\" value=\"1\" %%%input_count_s0_1%%%>"
"   <label for=\"input_count_s0_1\"> 1 S0 input</label>"
"   <input type=\"radio\" id=\"input_count_s0_2\" name=\"input_count_s0\" value=\"2\" %%%input_count_s0_2%%%>"
"   <label for=\"input_count_s0_2\"> 2 S0 inputs</label>"
" </fieldset>"
" <fieldset>"
"   <legend>first S0 input</legend>"
" <label for=\"s0_0_pin_name\">S0 input 0 name</label>"
"    <input type=\"text\" id=\"s0_0_pin_name\" name=\"s0_0_pin_name\" value=\"%%%s0_0_pin_name%%%\">"
" <br />"
" <label for=\"s0_0_pin_number\">S0 input 0 number</label>"
"    <input type=\"text\" id=\"s0_0_pin_number\" name=\"s0_0_pin_number\" value=\"%%%s0_0_pin_number%%%\">"
" <br />"
"   <input type=\"radio\" id=\"s0_0_on_value_HIGH\" name=\"s0_0_on_value\" value=\"HIGH\" %%%s0_0_on_value_HIGH%%%>"
"   <label for=\"s0_0_on_value_HIGH\"> S0_0 on=HIGH</label>"
"   <input type=\"radio\" id=\"s0_0_on_value_LOW\" name=\"s0_0_on_value\" value=\"LOW\" %%%s0_0_on_value_LOW%%%>"
"   <label for=\"s0_0_on_value_LOW\"> S0_0 on=LOW</label>"
" <br />"
"   <input type=\"radio\" id=\"s0_0_off_value_HIGH\" name=\"s0_0_off_value\" value=\"HIGH\" %%%s0_0_off_value_HIGH%%%>"
"   <label for=\"s0_0_off_value_HIGH\"> S0_0 off=HIGH</label>"
"   <input type=\"radio\" id=\"s0_0_off_value_LOW\" name=\"s0_0_off_value\" value=\"LOW\" %%%s0_0_off_value_LOW%%%>"
"   <label for=\"s0_0_off_value_LOW\"> S0_0 off=LOW</label>"
"   <br />"
"   <input type=\"radio\" id=\"s0_0_pinmode_INPUT\" name=\"s0_0_pinmode\" value=\"INPUT\" %%%s0_0_pinmode_INPUT%%%>"
"   <label for=\"s0_0_pinmode_INPUT\"> S0_0 pinmode=INPUT</label>"
"   <input type=\"radio\" id=\"s0_0_pinmode_INPUT_PULLUP\" name=\"s0_0_pinmode\" value=\"INPUT_PULLUP\" %%%s0_0_pinmode_INPUT_PULLUP%%%>"
"   <label for=\"s0_0_pinmode_INPUT_PULLUP\"> S0_0 pinmode=INPUT_PULLUP</label>"
"   <br />"
" <label for=\"s0_0_impulses_per_kWh\">S0 input 0 impulses per kWh</label>"
"    <input type=\"text\" id=\"s0_0_impulses_per_kWh\" name=\"s0_0_impulses_per_kWh\" value=\"%%%s0_0_impulses_per_kWh%%%\">"
" </fieldset>"
"   <fieldset>"
"   <legend>second S0 input</legend>"
" <label for=\"s0_1_pin_name\">S0 input 1 name</label>"
"    <input type=\"text\" id=\"s0_1_pin_name\" name=\"s0_1_pin_name\" value=\"%%%s0_1_pin_name%%%\">"
" <br />"
" <label for=\"s0_1_pin_number\">S0 input 1 number</label>"
"    <input type=\"text\" id=\"s0_1_pin_number\" name=\"s0_1_pin_number\" value=\"%%%s0_1_pin_number%%%\">"
" <br />"
"   <input type=\"radio\" id=\"s0_1_on_value_HIGH\" name=\"s0_1_on_value\" value=\"HIGH\" %%%s0_1_on_value_HIGH%%%>"
"   <label for=\"s0_1_on_value_HIGH\"> S0_1 on=HIGH</label>"
"   <input type=\"radio\" id=\"s0_1_on_value_LOW\" name=\"s0_1_on_value\" value=\"LOW\" %%%s0_1_on_value_LOW%%%>"
"   <label for=\"s0_1_on_value_LOW\"> S0_1 on=LOW</label>"
" <br />"
"   <input type=\"radio\" id=\"s0_1_off_value_HIGH\" name=\"s0_1_off_value\" value=\"HIGH\" %%%s0_1_off_value_HIGH%%%>"
"   <label for=\"s0_1_off_value_HIGH\"> S0_1 off=HIGH</label>"
"   <input type=\"radio\" id=\"s0_1_off_value_LOW\" name=\"s0_1_off_value\" value=\"LOW\" %%%s0_1_off_value_LOW%%%>"
"   <label for=\"s0_1_off_value_LOW\"> S0_1 off=LOW</label>"
"   <br />"
"   <input type=\"radio\" id=\"s0_1_pinmode_INPUT\" name=\"s0_1_pinmode\" value=\"INPUT\" %%%s0_1_pinmode_INPUT%%%>"
"   <label for=\"s0_1_pinmode_INPUT\"> S0_1 pinmode=INPUT</label>"
"   <input type=\"radio\" id=\"s0_1_pinmode_INPUT_PULLUP\" name=\"s0_1_pinmode\" value=\"INPUT_PULLUP\" %%%s0_1_pinmode_INPUT_PULLUP%%%>"
"   <label for=\"s0_1_pinmode_INPUT_PULLUP\"> S0_1 pinmode=INPUT_PULLUP</label>"
"   <br />"
" <label for=\"s0_1_impulses_per_kWh\">S0 input 1 impulses per kWh</label>"
"    <input type=\"text\" id=\"s0_1_impulses_per_kWh\" name=\"s0_1_impulses_per_kWh\" value=\"%%%s0_1_impulses_per_kWh%%%\">"
" </fieldset>"
" "
" "
" <br />"
" <fieldset>"
"   <legend>digital input count</legend>"
"   <input type=\"radio\" id=\"input_count_di_0\" name=\"input_count_di\" value=\"0\" %%%input_count_di_0%%%>"
"   <label for=\"input_count_di_0\"> no digital input</label>"
"   <input type=\"radio\" id=\"input_count_di_1\" name=\"input_count_di\" value=\"1\" %%%input_count_di_1%%%>"
"   <label for=\"input_count_di_1\"> 1 digital input</label>"
"   <input type=\"radio\" id=\"input_count_di_2\" name=\"input_count_di\" value=\"2\" %%%input_count_di_2%%%>"
"   <label for=\"input_count_di_2\"> 2 digital inputs</label>"
" </fieldset>"
"   <fieldset>"
"   <legend>first digital input</legend>"
" <label for=\"di_0_pin_name\">digital input 0 name</label>"
"    <input type=\"text\" id=\"di_0_pin_name\" name=\"di_0_pin_name\" value=\"%%%di_0_pin_name%%%\">"
" <br />"
" <label for=\"di_0_pin_number\">digital input 0 number</label>"
"    <input type=\"text\" id=\"di_0_pin_number\" name=\"di_0_pin_number\" value=\"%%%di_0_pin_number%%%\">"
" <br />"
"   <input type=\"radio\" id=\"di_0_on_value_HIGH\" name=\"di_0_on_value\" value=\"HIGH\" %%%di_0_on_value_HIGH%%%>"
"   <label for=\"di_0_on_value_HIGH\"> digital_0 on=HIGH</label>"
"   <input type=\"radio\" id=\"di_0_on_value_LOW\" name=\"di_0_on_value\" value=\"LOW\" %%%di_0_on_value_LOW%%%>"
"   <label for=\"di_0_on_value_LOW\"> digital_0 on=LOW</label>"
" <br />"
"   <input type=\"radio\" id=\"di_0_off_value_HIGH\" name=\"di_0_off_value\" value=\"HIGH\" %%%di_0_off_value_HIGH%%%>"
"   <label for=\"di_0_off_value_HIGH\"> digital_0 off=HIGH</label>"
"   <input type=\"radio\" id=\"di_0_off_value_LOW\" name=\"di_0_off_value\" value=\"LOW\" %%%di_0_off_value_LOW%%%>"
"   <label for=\"di_0_off_value_LOW\"> digital_0 off=LOW</label>"
"   <br />"
"   <input type=\"radio\" id=\"di_0_pinmode_INPUT\" name=\"di_0_pinmode\" value=\"INPUT\" %%%di_0_pinmode_INPUT%%%>"
"   <label for=\"di_0_pinmode_INPUT\"> digital_0 pinmode=INPUT</label>"
"   <input type=\"radio\" id=\"di_0_pinmode_INPUT_PULLUP\" name=\"di_0_pinmode\" value=\"INPUT_PULLUP\" %%%di_0_pinmode_INPUT_PULLUP%%%>"
"   <label for=\"di_0_pinmode_INPUT_PULLUP\"> digital_0 pinmode=INPUT_PULLUP</label>"
" </fieldset>"
" <fieldset>"
"   <legend>second digital input</legend>"
" <label for=\"di_1_pin_name\">digital input 1 name</label>"
"    <input type=\"text\" id=\"di_1_pin_name\" name=\"di_1_pin_name\" value=\"%%%di_1_pin_name%%%\">"
" <br />"
" <label for=\"di_1_pin_number\">digital input 1 number</label>"
"    <input type=\"text\" id=\"di_1_pin_number\" name=\"di_1_pin_number\" value=\"%%%di_1_pin_number%%%\">"
" <br />"
"   <input type=\"radio\" id=\"di_1_on_value_HIGH\" name=\"di_1_on_value\" value=\"HIGH\" %%%di_1_on_value_HIGH%%%>"
"   <label for=\"di_1_on_value_HIGH\"> digital_1 on=HIGH</label>"
"   <input type=\"radio\" id=\"di_1_on_value_LOW\" name=\"di_1_on_value\" value=\"LOW\" %%%di_1_on_value_LOW%%%>"
"   <label for=\"di_1_on_value_LOW\"> digital_1 on=LOW</label>"
" <br />"
"   <input type=\"radio\" id=\"di_1_off_value_HIGH\" name=\"di_1_off_value\" value=\"HIGH\" %%%di_1_off_value_HIGH%%%>"
"   <label for=\"di_1_off_value_HIGH\"> digital_1 off=HIGH</label>"
"   <input type=\"radio\" id=\"di_1_off_value_LOW\" name=\"di_1_off_value\" value=\"LOW\" %%%di_1_off_value_LOW%%%>"
"   <label for=\"di_1_off_value_LOW\"> digital_1 off=LOW</label>"
"   <br />"
"   <input type=\"radio\" id=\"di_1_pinmode_INPUT\" name=\"di_1_pinmode\" value=\"INPUT\" %%%di_1_pinmode_INPUT%%%>"
"   <label for=\"di_1_pinmode_INPUT\"> digital_1 pinmode=INPUT</label>"
"   <input type=\"radio\" id=\"di_1_pinmode_INPUT_PULLUP\" name=\"di_1_pinmode\" value=\"INPUT_PULLUP\" %%%di_1_pinmode_INPUT_PULLUP%%%>"
"   <label for=\"di_1_pinmode_INPUT_PULLUP\"> digital_1 pinmode=INPUT_PULLUP</label>"
" </fieldset>"
" <br />"
" <fieldset>"
"   <legend>CP analog input count on A0</legend>"
"   <input type=\"radio\" id=\"input_count_cp_0\" name=\"input_count_cp\" value=\"0\" %%%input_count_cp_0%%%>"
"   <label for=\"input_count_cp_0\"> no analog CP input</label>"
"   <input type=\"radio\" id=\"input_count_cp_1\" name=\"input_count_cp\" value=\"1\" %%%input_count_cp_1%%%>"
"   <label for=\"input_count_cp_1\"> 1 analog CP input</label>"
" </fieldset>"
" <br />"
" <fieldset>"
"   <legend>HC-SR04 ultrasound input count</legend>"
"   <input type=\"radio\" id=\"input_count_us_0\" name=\"input_count_us\" value=\"0\" %%%input_count_us_0%%%>"
"   <label for=\"input_count_us_0\"> no ultrasound input</label>"
"   <input type=\"radio\" id=\"input_count_us_1\" name=\"input_count_us\" value=\"1\" %%%input_count_us_1%%%>"
"   <label for=\"input_count_us_1\"> 1 ultrasound input</label>"
"   <input type=\"radio\" id=\"input_count_us_2\" name=\"input_count_us\" value=\"2\" %%%input_count_us_2%%%>"
"   <label for=\"input_count_us_2\"> 2 ultrasound inputs</label>"
" </fieldset>"
""
" <fieldset>"
"   <legend>first HC-SR04 ultrasound input</legend>"
" <label for=\"us_0_pin_name\">name</label>"
"    <input type=\"text\" id=\"us_0_pin_name\" name=\"us_0_pin_name\" value=\"%%%us_0_pin_name%%%\">"
" <br />"
" <label for=\"us_0_trigger_pin\">trigger pin</label>"
"    <input type=\"text\" id=\"us_0_trigger_pin\" name=\"us_0_trigger_pin\" value=\"%%%us_0_trigger_pin%%%\">"
" <br />"
" <input type=\"radio\" id=\"us_0_trigger_on_HIGH\" name=\"us_0_trigger_on\" value=\"HIGH\" %%%us_0_trigger_on_HIGH%%%>"
"   <label for=\"us_0_trigger_on_HIGH\">  trigger_on=HIGH</label>"
"   <input type=\"radio\" id=\"us_0_trigger_on_LOW\" name=\"us_0_trigger_on\" value=\"LOW\" %%%us_0_trigger_on_LOW%%%>"
"   <label for=\"us_0_trigger_on_LOW\">  trigger_on=LOW</label>"
" <br />"
" <input type=\"radio\" id=\"us_0_trigger_off_HIGH\" name=\"us_0_trigger_off\" value=\"HIGH\" %%%us_0_trigger_off_HIGH%%%>"
"   <label for=\"us_0_trigger_off_HIGH\">  trigger_off=HIGH</label>"
"   <input type=\"radio\" id=\"us_0_trigger_off_LOW\" name=\"us_0_trigger_off\" value=\"LOW\" %%%us_0_trigger_off_LOW%%%>"
"   <label for=\"us_0_trigger_off_LOW\">  trigger_off=LOW</label>"
" <br />"
" <label for=\"us_0_echo_pin\">echo pin</label>"
"    <input type=\"text\" id=\"us_0_echo_pin\" name=\"us_0_echo_pin\" value=\"%%%us_0_echo_pin%%%\">"
" <br />"
" <input type=\"radio\" id=\"us_0_echo_on_HIGH\" name=\"us_0_echo_on\" value=\"HIGH\" %%%us_0_echo_on_HIGH%%%>"
"   <label for=\"us_0_echo_on_HIGH\">  echo_on=HIGH</label>"
"   <input type=\"radio\" id=\"us_0_echo_on_LOW\" name=\"us_0_echo_on\" value=\"LOW\" %%%us_0_echo_on_LOW%%%>"
"   <label for=\"us_0_echo_on_LOW\">  echo_on=LOW</label>"
"     <br />"
" <label for=\"us_0_timeout_us\">timeout in microseconds (multiply max cm by 58, add 10%)</label>"
"    <input type=\"text\" id=\"us_0_timeout_us\" name=\"us_0_timeout_us\" value=\"%%%us_0_timeout_us%%%\">"
" <br />"
" <label for=\"us_0_free_distance\">how many cm = free</label>"
"    <input type=\"text\" id=\"us_0_free_distance\" name=\"us_0_free_distance\" value=\"%%%us_0_free_distance%%%\">"
" </fieldset>"
"   <fieldset>"
"   <legend>second HC-SR04 ultrasound input</legend>"
" <label for=\"us_1_pin_name\">name</label>"
"    <input type=\"text\" id=\"us_1_pin_name\" name=\"us_1_pin_name\" value=\"%%%us_1_pin_name%%%\">"
" <br />"
" <label for=\"us_1_trigger_pin\">trigger pin</label>"
"    <input type=\"text\" id=\"us_1_trigger_pin\" name=\"us_1_trigger_pin\" value=\"%%%us_1_trigger_pin%%%\">"
" <br />"
" <input type=\"radio\" id=\"us_1_trigger_on_HIGH\" name=\"us_1_trigger_on\" value=\"HIGH\" %%%us_1_trigger_on_HIGH%%%>"
"   <label for=\"us_1_trigger_on_HIGH\">  trigger_on=HIGH</label>"
"   <input type=\"radio\" id=\"us_1_trigger_on_LOW\" name=\"us_1_trigger_on\" value=\"LOW\" %%%us_1_trigger_on_LOW%%%>"
"   <label for=\"us_1_trigger_on_LOW\">  trigger_on=LOW</label>"
" <br />"
" <input type=\"radio\" id=\"us_1_trigger_off_HIGH\" name=\"us_1_trigger_off\" value=\"HIGH\" %%%us_1_trigger_off_HIGH%%%>"
"   <label for=\"us_1_trigger_off_HIGH\">  trigger_off=HIGH</label>"
"   <input type=\"radio\" id=\"us_1_trigger_off_LOW\" name=\"us_1_trigger_off\" value=\"LOW\" %%%us_1_trigger_off_LOW%%%>"
"   <label for=\"us_1_trigger_off_LOW\">  trigger_off=LOW</label>"
" <br />"
" <label for=\"us_1_echo_pin\">echo pin</label>"
"    <input type=\"text\" id=\"us_1_echo_pin\" name=\"us_1_echo_pin\" value=\"%%%us_1_echo_pin%%%\">"
" <br />"
" <input type=\"radio\" id=\"us_1_echo_on_HIGH\" name=\"us_1_echo_on\" value=\"HIGH\" %%%us_1_echo_on_HIGH%%%>"
"   <label for=\"us_1_echo_on_HIGH\">  echo_on=HIGH</label>"
"   <input type=\"radio\" id=\"us_1_echo_on_LOW\" name=\"us_1_echo_on\" value=\"LOW\" %%%us_1_echo_on_LOW%%%>"
"   <label for=\"us_1_echo_on_LOW\">  echo_on=LOW</label>"
"     <br />"
" <label for=\"us_1_timeout_us\">timeout in microseconds (multiply max cm by 58, add 10%)</label>"
"    <input type=\"text\" id=\"us_1_timeout_us\" name=\"us_1_timeout_us\" value=\"%%%us_1_timeout_us%%%\">"
" <br />"
" <label for=\"us_1_free_distance\">how many cm = free</label>"
"    <input type=\"text\" id=\"us_1_free_distance\" name=\"us_1_free_distance\" value=\"%%%us_1_free_distance%%%\">"
" </fieldset>"
" <br />"
" <fieldset>"
"   <legend>SmartEVSE serial input count</legend>"
"   <input type=\"radio\" id=\"input_count_ev_0\" name=\"input_count_ev\" value=\"0\" %%%input_count_ev_0%%%>"
"   <label for=\"input_count_ev_0\"> no SmartEVSE serial input</label>"
"   <input type=\"radio\" id=\"input_count_ev_1\" name=\"input_count_ev\" value=\"1\" %%%input_count_ev_1%%%>"
"   <label for=\"input_count_ev_1\"> 1 SmartEVSE serial input</label>"
"   <input type=\"radio\" id=\"input_count_ev_2\" name=\"input_count_ev\" value=\"2\" %%%input_count_ev_2%%%>"
"   <label for=\"input_count_ev_2\"> 2 SmartEVSE serial inputs</label>"
" </fieldset>"
"  <fieldset>"
"   <legend>first SmartEVSE serial input</legend>"
"     <label for=\"ev_0_pin_name\">name</label>"
"    <input type=\"text\" id=\"ev_0_pin_name\" name=\"ev_0_pin_name\" value=\"%%%ev_0_pin_name%%%\">"
" <br />"
" <label for=\"ev_0_pin_number\">pin number</label>"
"    <input type=\"text\" id=\"ev_0_pin_number\" name=\"ev_0_pin_number\" value=\"%%%ev_0_pin_number%%%\">"
" <br />"
" <label for=\"ev_0_baudrate\">baud rate</label>"
"    <input type=\"text\" id=\"ev_0_baudrate\" name=\"ev_0_baudrate\" value=\"%%%ev_0_baudrate%%%\">"
" <br />"
"   </fieldset>"
"     <fieldset>"
"   <legend>second SmartEVSE serial input</legend>"
"     <label for=\"ev_1_pin_name\">name</label>"
"    <input type=\"text\" id=\"ev_1_pin_name\" name=\"ev_1_pin_name\" value=\"%%%ev_1_pin_name%%%\">"
" <br />"
" <label for=\"ev_1_pin_number\">pin number</label>"
"    <input type=\"text\" id=\"ev_1_pin_number\" name=\"ev_1_pin_number\" value=\"%%%ev_1_pin_number%%%\">"
" <br />"
" <label for=\"ev_1_baudrate\">baud rate</label>"
"    <input type=\"text\" id=\"ev_1_baudrate\" name=\"ev_1_baudrate\" value=\"%%%ev_1_baudrate%%%\">"
" <br />"
"   </fieldset>"
" <button type=\"submit\">Save settings and reboot CfOnlinestatus</button>"
"   </form>"
"  </body>"
"</html>";

  page.replace("%%%chargepoint_id%%%", cfg.chargepoint_id);
  page.replace("%%%sensor_update_interval_s%%%", String(cfg.sensor_update_interval_s));
  page.replace("%%%serial_baudrate%%%", String(cfg.serial_baudrate));
  page.replace("%%%serial_output_interval_s%%%", String(cfg.serial_output_interval_s));
  page.replace("%%%wifi_ssid%%%", cfg.wifi_ssid);
  page.replace("%%%wifi_key%%%", cfg.wifi_key);
  page.replace("%%%mqtt_server%%%", cfg.mqtt_server);
  page.replace("%%%mqtt_port%%%", String(cfg.mqtt_port));
  page.replace("%%%mqtt_username%%%", cfg.mqtt_username);
  page.replace("%%%mqtt_password%%%", cfg.mqtt_password);
  page.replace("%%%mqtt_update_interval_s%%%", String(cfg.mqtt_update_interval_s));
  page.replace("%%%input_count_s0_0%%%", (cfg.input_count_s0==0)?"checked":"");
  page.replace("%%%input_count_s0_1%%%", (cfg.input_count_s0==1)?"checked":"");
  page.replace("%%%input_count_s0_2%%%", (cfg.input_count_s0==2)?"checked":"");
  page.replace("%%%s0_0_pin_name%%%", cfg.s0[0].pin_name);
  page.replace("%%%s0_0_pin_number%%%", pin2name(cfg.s0[0].pin_number));
  page.replace("%%%s0_0_on_value_HIGH%%%", (cfg.s0[0].on_value==HIGH)?"checked":"");
  page.replace("%%%s0_0_on_value_LOW%%%", (cfg.s0[0].on_value==LOW)?"checked":"");
  page.replace("%%%s0_0_off_value_HIGH%%%", (cfg.s0[0].off_value==HIGH)?"checked":"");
  page.replace("%%%s0_0_off_value_LOW%%%", (cfg.s0[0].off_value==LOW)?"checked":"");
  page.replace("%%%s0_0_pinmode_INPUT%%%", (cfg.s0[0].pin_mode==INPUT)?"checked":"");
  page.replace("%%%s0_0_pinmode_INPUT_PULLUP%%%", (cfg.s0[0].pin_mode==INPUT_PULLUP)?"checked":"");
  page.replace("%%%s0_0_impulses_per_kWh%%%", String(cfg.s0_impulses_per_kWh[0]));
  page.replace("%%%s0_1_pin_name%%%", cfg.s0[1].pin_name);
  page.replace("%%%s0_1_pin_number%%%", pin2name(cfg.s0[1].pin_number));
  page.replace("%%%s0_1_on_value_HIGH%%%", (cfg.s0[1].on_value==HIGH)?"checked":"");
  page.replace("%%%s0_1_on_value_LOW%%%", (cfg.s0[1].on_value==LOW)?"checked":"");
  page.replace("%%%s0_1_off_value_HIGH%%%", (cfg.s0[1].off_value==HIGH)?"checked":"");
  page.replace("%%%s0_1_off_value_LOW%%%", (cfg.s0[1].off_value==LOW)?"checked":"");
  page.replace("%%%s0_1_pinmode_INPUT%%%", (cfg.s0[1].pin_mode==INPUT)?"checked":"");
  page.replace("%%%s0_1_pinmode_INPUT_PULLUP%%%", (cfg.s0[1].pin_mode==INPUT_PULLUP)?"checked":"");
  page.replace("%%%s0_1_impulses_per_kWh%%%", String(cfg.s0_impulses_per_kWh[1]));
  page.replace("%%%input_count_di_0%%%", (cfg.input_count_di==0)?"checked":"");
  page.replace("%%%input_count_di_1%%%", (cfg.input_count_di==1)?"checked":"");
  page.replace("%%%input_count_di_2%%%", (cfg.input_count_di==2)?"checked":"");
  page.replace("%%%di_0_pin_name%%%", cfg.di[0].pin_name);
  page.replace("%%%di_0_pin_number%%%", pin2name(cfg.di[0].pin_number));
  page.replace("%%%di_0_on_value_HIGH%%%", (cfg.di[0].on_value==HIGH)?"checked":"");
  page.replace("%%%di_0_on_value_LOW%%%", (cfg.di[0].on_value==LOW)?"checked":"");
  page.replace("%%%di_0_off_value_HIGH%%%", (cfg.di[0].off_value==HIGH)?"checked":"");
  page.replace("%%%di_0_off_value_LOW%%%", (cfg.di[0].off_value==LOW)?"checked":"");
  page.replace("%%%di_0_pinmode_INPUT%%%", (cfg.di[0].pin_mode==INPUT)?"checked":"");
  page.replace("%%%di_0_pinmode_INPUT_PULLUP%%%", (cfg.di[0].pin_mode==INPUT_PULLUP)?"checked":"");
  page.replace("%%%di_1_pin_name%%%", cfg.di[1].pin_name);
  page.replace("%%%di_1_pin_number%%%", pin2name(cfg.di[1].pin_number));
  page.replace("%%%di_1_on_value_HIGH%%%", (cfg.di[1].on_value==HIGH)?"checked":"");
  page.replace("%%%di_1_on_value_LOW%%%", (cfg.di[1].on_value==LOW)?"checked":"");
  page.replace("%%%di_1_off_value_HIGH%%%", (cfg.di[1].off_value==HIGH)?"checked":"");
  page.replace("%%%di_1_off_value_LOW%%%", (cfg.di[1].off_value==LOW)?"checked":"");
  page.replace("%%%di_1_pinmode_INPUT%%%", (cfg.di[1].pin_mode==INPUT)?"checked":"");
  page.replace("%%%di_1_pinmode_INPUT_PULLUP%%%", (cfg.di[1].pin_mode==INPUT_PULLUP)?"checked":"");
  page.replace("%%%input_count_cp_0%%%", (cfg.input_count_cp==0)?"checked":"");
  page.replace("%%%input_count_cp_1%%%", (cfg.input_count_cp==1)?"checked":"");
  page.replace("%%%input_count_us_0%%%", (cfg.input_count_us==0)?"checked":"");
  page.replace("%%%input_count_us_1%%%", (cfg.input_count_us==1)?"checked":"");
  page.replace("%%%input_count_us_2%%%", (cfg.input_count_us==2)?"checked":"");
  page.replace("%%%us_0_pin_name%%%", cfg.us[0].sensor_name);
  page.replace("%%%us_0_trigger_pin%%%", pin2name(cfg.us[0].trigger_pin));
  page.replace("%%%us_0_echo_pin%%%", pin2name(cfg.us[0].echo_pin));
  page.replace("%%%us_0_trigger_on_HIGH%%%", (cfg.us[0].trigger_on==HIGH)?"checked":"");
  page.replace("%%%us_0_trigger_on_LOW%%%", (cfg.us[0].trigger_on==LOW)?"checked":"");
  page.replace("%%%us_0_trigger_off_HIGH%%%", (cfg.us[0].trigger_off==HIGH)?"checked":"");
  page.replace("%%%us_0_trigger_off_LOW%%%", (cfg.us[0].trigger_off==LOW)?"checked":"");
  page.replace("%%%us_0_echo_on_HIGH%%%", (cfg.us[0].echo_on==HIGH)?"checked":"");
  page.replace("%%%us_0_echo_on_LOW%%%", (cfg.us[0].echo_on==LOW)?"checked":"");
  page.replace("%%%us_0_timeout_us%%%", String(cfg.us[0].timeout_us));
  page.replace("%%%us_0_free_distance%%%", String(cfg.us[0].free_distance));
  page.replace("%%%us_1_pin_name%%%", cfg.us[1].sensor_name);
  page.replace("%%%us_1_trigger_pin%%%", pin2name(cfg.us[1].trigger_pin));
  page.replace("%%%us_1_echo_pin%%%", pin2name(cfg.us[1].echo_pin));
  page.replace("%%%us_1_trigger_on_HIGH%%%", (cfg.us[1].trigger_on==HIGH)?"checked":"");
  page.replace("%%%us_1_trigger_on_LOW%%%", (cfg.us[1].trigger_on==LOW)?"checked":"");
  page.replace("%%%us_1_trigger_off_HIGH%%%", (cfg.us[1].trigger_off==HIGH)?"checked":"");
  page.replace("%%%us_1_trigger_off_LOW%%%", (cfg.us[1].trigger_off==LOW)?"checked":"");
  page.replace("%%%us_1_echo_on_HIGH%%%", (cfg.us[1].echo_on==HIGH)?"checked":"");
  page.replace("%%%us_1_echo_on_LOW%%%", (cfg.us[1].echo_on==LOW)?"checked":"");
  page.replace("%%%us_1_timeout_us%%%", String(cfg.us[1].timeout_us));
  page.replace("%%%us_1_free_distance%%%", String(cfg.us[1].free_distance));
  page.replace("%%%input_count_ev_0%%%", (cfg.input_count_ev==0)?"checked":"");
  page.replace("%%%input_count_ev_1%%%", (cfg.input_count_ev==1)?"checked":"");
  page.replace("%%%input_count_ev_2%%%", (cfg.input_count_ev==2)?"checked":"");
  page.replace("%%%ev_0_pin_name%%%", cfg.ev[0].pin_name);
  page.replace("%%%ev_0_pin_number%%%", pin2name(cfg.ev[0].pin_number));
  page.replace("%%%ev_0_baudrate%%%", String(cfg.ev[0].baudrate));
  page.replace("%%%ev_1_pin_name%%%", cfg.ev[1].pin_name);
  page.replace("%%%ev_1_pin_number%%%", pin2name(cfg.ev[1].pin_number));
  page.replace("%%%ev_1_baudrate%%%", String(cfg.ev[1].baudrate));

  webserver.send(200, "text/html", page);
}

String pin2name(uint8_t pin) {
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
uint8_t name2pin(String name) {
  if(name=="A0") return A0;
  if(name=="D0") return D0;
  if(name=="D1") return D1;
  if(name=="D2") return D2;
  if(name=="D3") return D3;
  if(name=="D4") return D4;
  if(name=="D5") return D5;
  if(name=="D6") return D6;
  if(name=="D7") return D7;
  if(name=="D8") return D8;
  return 255;
}
uint8_t name2hilo(String name) {
  if(name=="HIGH") return HIGH;
  if(name=="LOW") return LOW;
  return 255;
}
uint8_t name2input(String name) {
  if(name=="INPUT") return HIGH;
  if(name=="INPUT_PULLUP") return INPUT_PULLUP;
  return 255;
}

