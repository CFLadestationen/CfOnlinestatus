/**
 * Arduino-Sketch for the online status of crowdfunded charging stations
 * Repository: https://github.com/CFLadestationen/CfOnlinestatus
 * Development thread (in German): https://www.goingelectric.de/forum/goingelectric-crowdfunding/neues-projekt-onlinestatus-fuer-crowdfunding-ladepunkte-t29325.html
 * 
 * Current features
 * - Read an arbitrary number of S0 inputs and print them on the serial console
 * - Serial format: s0_<number_starting_at_0>:<ms_between_last_2_impulses:<impulses_since_last_output>:<newline \n>
 * - Serial example:
 *     s0_0:300:6:
 *     s0_1:465:4:
 * 
 * Untested software - use at your own risk!
 */
// ---- Begin of customizable section ----

// Pin numbers of S0 input pins
const uint8_t s0_pins[] = {10,    11};
// High or low active? If low, switch both states
const uint8_t s0_high[] = {HIGH,  HIGH};
const uint8_t s0_low[]  = {LOW,   LOW};
// Set to INPUT_PULLUP instead of INPUT to activate the internal pull-up resistor
const uint8_t s0_mode[] = {INPUT, INPUT};
// Send serial updates every ... ms
const uint32_t output_delay = 2000;

// ---- End of customizable section ----

// Internal variables
uint32_t last_s0_millis[sizeof(s0_pins)];
uint32_t last_s0_span[sizeof(s0_pins)];
uint8_t  last_s0_state[sizeof(s0_pins)];
uint32_t last_output;
uint32_t impulses_since_output[sizeof(s0_pins)];

void setup() {
  Serial.begin(9600);
  // wait for serial connection
  while(!Serial) {}
  
  Serial.print("CfOnlinestatus starting with ");
  Serial.print(sizeof(s0_pins));
  Serial.println(" configured S0 pins.");
  
  uint32_t current_time = millis();
  last_output = current_time;
  for(uint8_t i = 0; i < sizeof(s0_pins); i++) {
    pinMode(s0_pins[i], s0_mode[i]);
    last_s0_millis[i] = current_time;
    last_s0_span[i] = 0;
    last_s0_state[i] = s0_low[i];
    impulses_since_output[i] = 0;
  }
}

void loop() {
  uint32_t current_time = millis();
  read_s0_inputs();
  if(((uint32_t)(current_time-last_output)) >= output_delay) {
    print_s0_status(); 
  }
}

void read_s0_inputs() {
  uint32_t current_time = millis();
  for(uint8_t i = 0; i < sizeof(s0_pins); i++) {
    uint8_t value = digitalRead(s0_pins[i]);
    if(value == s0_high[i] && last_s0_state[i] == s0_low[i]) {
      // change detected
      last_s0_span[i] = (uint32_t)(current_time - last_s0_millis[i]); // cast to avoid rollover problems
      last_s0_millis[i] = current_time;
      impulses_since_output[i]++;
    }
    last_s0_state[i] = value;
  }
}

void print_s0_status() {
  for(uint8_t i = 0; i < sizeof(s0_pins); i++) {
    Serial.print("s0_");
    Serial.print(i);
    Serial.print(':');
    Serial.print(last_s0_span[i]);
    Serial.print(':');
    Serial.print(impulses_since_output[i]);
    impulses_since_output[i] = 0;
    Serial.println(":");
  }
}

