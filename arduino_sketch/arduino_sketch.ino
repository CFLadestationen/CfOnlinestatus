/**
 * Arduino-Sketch for the online status of crowdfunded charging stations
 * Repository: https://github.com/CFLadestationen/CfOnlinestatus
 * Development thread (in German): https://www.goingelectric.de/forum/goingelectric-crowdfunding/neues-projekt-onlinestatus-fuer-crowdfunding-ladepunkte-t29325.html
 * 
 * Current features
 * - Read an arbitrary number of S0 inputs and print them on the serial console
 * - Serial format: s0_<number_starting_at_0>:<ms_between_last_2_impulses>:<impulses_since_last_output>:<seconds since last impulse>:<newline \n>
 * - Serial example:
 *     s0_0:300:6:0:  // s0 pin 0 had 300 ms between the last two impulses, 6 impulses since the last output and 0 seconds since the last impulse
 *     s0_1:465:0:20: // s0 pin 1 had 465 ms between the last two impulses, 0 impulses since the last output and 20 seconds since the last impulse
 * - Digital input pins can be configured with a custom name, output is di_<name>:<status on or off>:<newline \n>
 * - Digital input example:
 *     di_SpaceOccupied:on:
 *     di_ContactorOn:off:
 * 
 * Untested software - use at your own risk!
 */
// ---- Begin of customizable section ----

// Pin numbers of S0 input pins
const uint8_t s0_pin[]  = {D1,    D2};
// S0 pins high or low active? If low, switch both states
const uint8_t s0_on[]   = {HIGH,  HIGH};
const uint8_t s0_off[]  = {LOW,   LOW};
// Set to INPUT_PULLUP instead of INPUT to activate the internal pull-up resistor
const uint8_t s0_mode[] = {INPUT, INPUT};
// Pin numbers and names of digital input pins
const uint8_t digital_input_pin[]  = {D5,              D4};
const char* digital_input_name[]   = {"SpaceOccupied", "ContactorOn"};
// Digital input pins high or low active? Mode?
const uint8_t digital_input_on[]   = {HIGH,            LOW};
const uint8_t digital_input_off[]  = {LOW,             HIGH};
const uint8_t digital_input_mode[] = {INPUT,           INPUT_PULLUP};
// Send serial updates every ... ms
const uint32_t output_delay = 5000;

// ---- End of customizable section ----

// Internal variables
uint32_t last_s0_millis[sizeof(s0_pin)];
uint32_t last_s0_span[sizeof(s0_pin)];
uint8_t  last_s0_state[sizeof(s0_pin)];
uint32_t last_output;
uint32_t impulses_since_output[sizeof(s0_pin)];

void setup() {
  Serial.begin(115200);
  // wait for serial connection
  while(!Serial) {}
  
  Serial.print("CfOnlinestatus starting with ");
  Serial.print(sizeof(s0_pin));
  Serial.print(" configured S0 pins and ");
  Serial.print(sizeof(digital_input_pin));
  Serial.println(" digital input pins.");
  uint32_t current_time = millis();
  last_output = current_time;
  uint8_t val = 0;
  for(uint8_t i = 0; i < sizeof(s0_pin); i++) {
    pinMode(s0_pin[i], s0_mode[i]);
    last_s0_millis[i] = current_time;
    last_s0_span[i] = 0;
    last_s0_state[i] = s0_off[i];
    impulses_since_output[i] = 0;
  }
  for(uint8_t i = 0; i < sizeof(digital_input_pin); i++) {
    pinMode(digital_input_pin[i], digital_input_mode[i]);
    Serial.print("Configured digital input pin ");
    Serial.println(digital_input_name[i]);
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
  for(uint8_t i = 0; i < sizeof(s0_pin); i++) {
    uint8_t value = digitalRead(s0_pin[i]);
    if(value == s0_on[i] && last_s0_state[i] == s0_off[i]) {
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
  for(uint8_t i = 0; i < sizeof(s0_pin); i++) {
    Serial.print("s0_");
    Serial.print(i);
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
  for(uint8_t i = 0; i < sizeof(digital_input_pin); i++) {
    Serial.print("di_");
    Serial.print(digital_input_name[i]);
    Serial.print(':');
    if(digitalRead(digital_input_pin[i]) == digital_input_on[i]) {
      Serial.print("on");
    } else {
      Serial.print("off");
    }
    Serial.println(":");
  }
}

