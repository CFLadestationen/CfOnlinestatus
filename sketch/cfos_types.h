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
  char    pin_name[16];
  uint8_t pin_number;
  uint8_t on_value;
  uint8_t off_value;
  uint8_t pin_mode;
}; // 20 Bytes

struct ultrasound_sensor {
  char    sensor_name[16];
  uint32_t timeout_us;
  uint16_t free_distance;
  uint8_t trigger_pin;
  uint8_t echo_pin;
  uint8_t trigger_on;
  uint8_t trigger_off;
  uint8_t echo_on;
  uint8_t dummy[5];
}; // 32 Bytes

struct smartevse_pin {
  char     pin_name[16];
  uint32_t baudrate;
  uint8_t  pin_number;
  uint8_t dummy[3];
}; // 24 Bytes

enum evse_state : uint8_t {
  EVSE_STATE_A = 1, // not connected
  EVSE_STATE_B = 2, // vehicle connected, not charging
  EVSE_STATE_C = 3  // vehicle connected, charging
};

struct lora_payload {
  uint8_t digital_status; // up to 8 digital inputs, starting rightmost
  uint8_t evse_status; // up to 4 evse statuses - analog and smartevse, starting rightmost
                       // 0 is undefined, 1 is state A, 2 is state B, 3 is state C
  uint8_t us_status; // up to 8 ultrasound inputs (vehicle present or not), starting rightmost
  uint8_t s0_watts[4]; // power in steps of 200 Watt
  uint8_t reserved; // reserved for future expansion
};

struct cfos_config {
  uint16_t  magic_marker; // must be 0x51EF
  uint16_t  config_length; // must be sizeof(cfos_config_v1)
  uint32_t  checksum; // must be 0 during CRC32 check and contain the crc32 value afterwards
  uint8_t   input_count_s0;
  uint8_t   input_count_di;
  uint8_t   input_count_cp;
  uint8_t   input_count_us;
  uint8_t   input_count_ev;
  uint8_t   output_uart;
  uint8_t   output_mqtt;
  uint8_t   output_lora;              
  uint8_t   io_reserved[4];           // 20 Bytes
  char      chargepoint_id[64];       // 84 Bytes
  uint8_t   sensor_update_interval_s;
  uint8_t   net_wifi;                 // 86 Bytes
  char      wifi_ssid[33];            // 119 Bytes
  char      wifi_key[65];             // 184 Bytes
  uint8_t   mqtt_update_interval_s;   
  char      mqtt_server[65];          // 250 Bytes
  uint16_t  mqtt_port;                // 252 Bytes
  char     mqtt_username[64];
  char     mqtt_password[64];         // 380 Bytes
  uint32_t  serial_baudrate;
  uint8_t   serial_output_interval_s; // 385 Bytes
  uint8_t   dummy_align1[3];          // 388 Bytes
  named_pin s0[2];                    // 428 Bytes
  uint32_t  s0_impulses_per_kWh[2];   // 436 Bytes
  named_pin di[2];                    // 476 Bytes
  named_pin cp;                       // 496 Bytes
  ultrasound_sensor us[2];            // 560 Bytes
  smartevse_pin ev[2];                // 608 Bytes
  uint8_t   reserved[416];            // 1024 Bytes
};

const cfos_config default_config = {
  .magic_marker = 0x51EF,
  .config_length = sizeof(cfos_config),
  .checksum = 0,
  .input_count_s0 = 0,
  .input_count_di = 0,
  .input_count_cp = 0,
  .input_count_us = 0,
  .input_count_ev = 0,
  .output_uart = 0,
  .output_mqtt = 0,
  .output_lora = 0,          
  .io_reserved = {0},
  .chargepoint_id = { '1', '0', '0', '0', '0', '0', 0},
  .sensor_update_interval_s = 15,
  .net_wifi = 0,
  .wifi_ssid = { 'W', 'i', 'F', 'i', 'S', 'S', 'I', 'D', 0},
  .wifi_key = { 'W', 'i', 'F', 'i', 'P', 'r', 'e', 's', 'h', 'a', 'r', 'e', 'd', 'K', 'e', 'y', 0},
  .mqtt_update_interval_s = 60,   
  .mqtt_server = { '4', '6', '.', '3', '8', '.', '2', '3', '2', '.', '9', '7', 0},
  .mqtt_port = 1883,
  .mqtt_username = { '1', '0', '0', '0', '0', '0', 0},
  .mqtt_password = {0},
  .serial_baudrate = 115200,
  .serial_output_interval_s = 15,
  .dummy_align1 = {0},
  .s0 = {
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
    },
  .s0_impulses_per_kWh = { 1000, 160 },
  .di = {
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
  },
  // Definition of analog input
  // CAUTION: ESP8266 only has one analog input (A0) with a voltage range of 0V - 1V
  // Connect Wemos D1 mini Pro like this: PE to GND, CP to 1N4148 (->|-) to 820k to 56k to A0 pin
  // The Wemos D1 mini Pro has an internal divider 220k to 100k
  // which means a total divider of 1100k to 100k so that +12V becomes +1V at the ESP pin
  .cp =   {
    "CPStatus",    // pin name
    A0,            // pin number
    220,           // "high" voltage (for SAE J1772: CP = Standby)
    120,           // "low" voltage (for SAE J1772: CP = charging)
    // if value is between the two limits, it is considered "mid" (CP = plugged in, but not charging)
    INPUT        // pin mode
  },
  .us = {
      {
        "CarDistanceA", // sensor name
        20000,          // Timeout in us - multiply max distance in cm by 58, add 10% safety margin
        250,            // How many cm must be free so that this space is considered free?
        D3,             // Trigger pin
        D2,             // Echo pin
        HIGH,           // Trigger ON value
        LOW,            // Trigger OFF value
        HIGH,           // Echo ON value
        {0}             // dummy
      },
      {
        "CarDistanceB", // sensor name
        20000,          // Timeout in us - multiply max distance in cm by 58, add 10% safety margin
        250,            // How many cm must be free so that this space is considered free?
        D3,             // Trigger pin
        D2,             // Echo pin
        HIGH,           // Trigger ON value
        LOW,            // Trigger OFF value
        HIGH,           // Echo ON value
        {0}             // dummy
      }
  },
  .ev = {
      {
        "Type2Left",  // SmartEVSE name
        115200,       // SmartEVSE baudrate
        D0,           // SmartEVSE input pin
        {0}           // dummy
      },
      {
        "Type2Right", // SmartEVSE name
        115200,       // SmartEVSE baudrate
        D7,           // SmartEVSE input pin
        {0}           // dummy  
      }
  },
  .reserved = {0}
};

#endif //CFOS_TYPES_H
