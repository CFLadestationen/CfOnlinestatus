/**
 * Arduino-Sketch for the online status of crowdfunded charging stations
 * Repository: https://github.com/CFLadestationen/CfOnlinestatus
 * Development thread (in German): https://www.goingelectric.de/forum/goingelectric-crowdfunding/neues-projekt-onlinestatus-fuer-crowdfunding-ladepunkte-t29325.html
 * 
 * Current features
 * - Read an arbitrary number of S0 inputs and digital inputs
 * - Print the status of those inputs on the serial console
 * - Serial format for S0: s0_<pin_name>:<ms_between_last_2_impulses>:<impulses_since_last_output>:<seconds since last impulse>:<newline \n>
 * - S0 example:
 *     s0_CounterA:300:6:0:  // s0 pin CounterA had 300 ms between the last two impulses, 6 impulses since the last output and 0 seconds since the last impulse
 *     s0_CounterB:465:0:20: // s0 pin CounterB had 465 ms between the last two impulses, 0 impulses since the last output and 20 seconds since the last impulse
 * - Serial format for digital input: di_<pin_name>:<status on or off>:<newline \n>
 * - Digital input example:
 *     di_SpaceOccupied:on:
 *     di_ContactorOn:off:
 */
struct named_pin {
  const char* pin_name;
  const uint8_t pin_number;
  const uint8_t on_value;
  const uint8_t off_value;
  const uint8_t pin_mode;
};

// ---- Begin of customizable section ----
// Send serial updates every ... ms
const uint32_t output_delay = 5000;
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
// ---- End of customizable section ----

// Internal variables
uint32_t last_output;
const uint8_t s0_pincount = sizeof(s0)/sizeof(s0[0]);
const uint8_t di_pincount = sizeof(digital_input)/sizeof(digital_input[0]);
uint32_t last_s0_millis[s0_pincount];
uint32_t last_s0_span[s0_pincount];
uint8_t  last_s0_state[s0_pincount];
uint32_t impulses_since_output[s0_pincount];

void setup() {
  Serial.begin(115200);
  // wait for serial connection
  while(!Serial) {}
  
  Serial.print("\n\nCfOnlinestatus starting with ");
  Serial.print(s0_pincount);
  Serial.print(" S0 pins and ");
  Serial.print(di_pincount);
  Serial.println(" digital input pins:");
  uint32_t current_time = millis();
  last_output = current_time;
  uint8_t val = 0;
  for(uint8_t i = 0; i < s0_pincount; i++) {
    pinMode(s0[i].pin_number, s0[i].pin_mode);
    last_s0_millis[i] = current_time;
    last_s0_span[i] = 0;
    last_s0_state[i] = s0[i].off_value;
    impulses_since_output[i] = 0;
    Serial.print("Configured pin number ");
    Serial.print(s0[i].pin_number);
    Serial.print(" as S0 pin named ");
    Serial.println(s0[i].pin_name);
  }
  for(uint8_t i = 0; i < di_pincount; i++) {
    pinMode(digital_input[i].pin_number, digital_input[i].pin_mode);
    Serial.print("Configured pin number ");
    Serial.print(digital_input[i].pin_number);
    Serial.print(" as digital input pin named ");
    Serial.println(digital_input[i].pin_name);
  }
  Serial.println("CfOnlinestatus initialisation complete.");
}

void loop() {
  uint32_t current_time = millis();
  read_s0_inputs();
  if(((uint32_t)(current_time-last_output)) >= output_delay) {
    last_output = current_time;
    print_s0_status(); 
    print_digital_input_status();
  }
}

void read_s0_inputs() {
  uint32_t current_time = millis();
  for(uint8_t i = 0; i < s0_pincount; i++) {
    uint8_t value = digitalRead(s0[i].pin_number);
    if(value == s0[i].on_value && last_s0_state[i] == s0[i].off_value) {
      // change detected
      last_s0_span[i] = (uint32_t)(current_time - last_s0_millis[i]); // cast to avoid rollover problems
      last_s0_millis[i] = current_time;
      impulses_since_output[i]++;
    }
    last_s0_state[i] = value;
  }
}

void print_s0_status() {
  uint32_t current_time = millis();
  for(uint8_t i = 0; i < s0_pincount; i++) {
    Serial.print("s0_");
    Serial.print(s0[i].pin_name);
    Serial.print(':');
    Serial.print(last_s0_span[i]);
    Serial.print(':');
    Serial.print(impulses_since_output[i]);
    impulses_since_output[i] = 0;
    Serial.print(':');
    Serial.print(((uint32_t)(current_time - last_s0_millis[i]))/1000);
    Serial.println(":");
  }
}

void print_digital_input_status() {
  for(uint8_t i = 0; i < di_pincount; i++) {
    Serial.print("di_");
    Serial.print(digital_input[i].pin_name);
    Serial.print(':');
    if(digitalRead(digital_input[i].pin_number) == digital_input[i].on_value) {
      Serial.print("on");
    } else {
      Serial.print("off");
    }
    Serial.println(":");
  }
}

