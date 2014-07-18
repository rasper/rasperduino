#include <CapacitiveSensor.h>
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"

// Set up buzzer on pin 6
const int PIN_BUZZER = 6;

// Set up 10M resistor between pins 4 & 2, pin 2 is sensor pin, add a wire and or foil if desired
CapacitiveSensor capacitance_sensor = CapacitiveSensor(4,2);
const long CAPACITANCE_SAMPLES = 60;
const unsigned long AUTOCAL_TIMEOUT = 1000; // recalibrate every n milliseconds
//const unsigned long AUTOCAL_TIMEOUT = 0xFFFFFFFF; // turn off autocalibrate

// Set up nRF24L01 radio on SPI bus plus pins 9 & 10 
RF24 radio(9,10);
// nRF24L01 pipe addresses
const uint8_t CHANNEL = 76;
const uint64_t WRITING_PIPE = 0xF0F0F0F0E2LL;
const uint64_t READING_PIPE = 0xF0F0F0F0E1LL;
// nRF24L01 payload
char payload[32];
// TODO: remove? friendly
byte addresses[][6] = {"1Node","2Node"};
// Set up roles to simplify testing 
boolean role;                                    // The main role variable, holds the current role identifier
boolean role_ping_out = 1, role_pong_back = 0;   // The two different roles.

// Configuration variables
long burn_up_ms = 10000; // milliseconds per burn up cycle
long cool_down_ms = 5000; // milliseconds per cool down cycle
long sensor_threshold = 50;

// State variables
long last_ms = 0;
long burn_ms = 0; // milliseconds of burn time
long cool_ms = cool_down_ms; // milliseconds of cool time
int burn_switch = -2; // burning if > 0, cooling if <= 0
bool was_burning = false; // previous state

// Tweaking constants
const int BURN_SWITCH_OFF = -2;
const int BURN_SWITCH_ON = 3;
const int BUZZER_VOLUME = 4;
const long DELAY_MS = 250;

void setup()
{
  Serial.begin(57600);
  setup_capacitance_sensor();
  setup_buzzer();
  setup_radio();
  delay(1000);
  last_ms = millis();
}

void setup_capacitance_sensor()
{
  capacitance_sensor.set_CS_AutocaL_Millis(AUTOCAL_TIMEOUT); // recalibrate every n milliseconds
  Serial.println("Capacitance sensor up!");
}

void setup_buzzer()
{
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);
  Serial.println("Buzzer up!");
}

void setup_radio()
{
  printf_begin();
  radio.begin();
  radio.setAutoAck(1);
  radio.enableAckPayload();
  radio.enableDynamicPayloads();
  radio.setDataRate(RF24_1MBPS);
  radio.setPALevel(RF24_PA_MAX);
  radio.setRetries(15,15);
  radio.setChannel(CHANNEL);
  radio.openWritingPipe(WRITING_PIPE); 
  radio.openReadingPipe(1,READING_PIPE); 
  radio.startListening();
  radio.powerUp();
  radio.printDetails();
  Serial.println("Radio up!");
}

void loop()
{
  Serial.print(burn_ms);Serial.print("\t");Serial.print(cool_ms); // DEBUG
  process_server_messages();
  process_sensor_data();
  update_status();
  update_buzzer();
  //report_to_server();
  was_burning = burn_switch > 0;
  Serial.println(); // DEBUG
  delay(DELAY_MS);
}

void process_server_messages()
{
  while (radio.available()) {
    unsigned char length = radio.getDynamicPayloadSize();
    radio.read(payload, length);
    payload[length] = '\0'; // NULL terminate to make string processing simpler
    Serial.println(payload); // DEBUG
    (strncmp("BON", payload, 3) || buzzer_on())
    &&
    (strncmp("BOF", payload, 3) || buzzer_off())
    &&
    (strncmp("CBU", payload, 3) || config_burn_up(payload+3))
    &&
    (strncmp("CCD", payload, 3) || config_cool_down(payload+3))
    &&
    (strncmp("CST", payload, 3) || config_sensor_threshold(payload+3))
    &&
    (strncmp("TRS", payload, 3) || timer_reset())
    ;
  }
}

int buzzer_on()
{
  return 0;
}

int buzzer_off()
{
  return 0;
}

int config_burn_up(const char args[])
{
  long val = atol(args);
  if (val > 0 && val <= 86400000) burn_up_ms = val * 1000;
  return 0;
}

int config_cool_down(const char args[])
{
  long val = atol(args);
  if (val > 0 && val <= 86400000) cool_down_ms = val * 1000;
  return 0;
}

int config_sensor_threshold(const char args[])
{
  long val = atol(args);
  if (val > 0) sensor_threshold = val;
  return 0;
}

int timer_reset()
{
  return 0;
}

void process_sensor_data()
{
  long val = capacitance_sensor.capacitiveSensor(CAPACITANCE_SAMPLES);
  Serial.print("\tc:");Serial.print(val); // DEBUG
  if (val >= sensor_threshold) {
    // burning up
    if (burn_switch >= 0) {
      // snap/keep switch at on position
      burn_switch = BURN_SWITCH_ON;
    }
    else {
      // move it closer to on position
      burn_switch += 1;
    }
  }
  else {
    // cooling down
    if (burn_switch <= 1) {
      // snap/keep switch at off position
      burn_switch = BURN_SWITCH_OFF;
    }
    else {
      // move it closer to off position
      burn_switch -= 1;
    }
  }
  Serial.print("\tb:");Serial.print(burn_switch); // DEBUG
}

void update_status()
{
  long ms = millis();
  long delta = ms - last_ms;
  last_ms = ms;
  
  if (burn_switch > 0) {
    burn_ms = min(burn_ms + delta, burn_up_ms);
    if (!was_burning) {
      start_burn();
    }
  }
  else {
    cool_ms = min(cool_ms + delta, cool_down_ms);
    if (was_burning) {
      start_cool();
    }
  }
  
  // DEBUG
  Serial.print('\t');
  int b;
  if (burn_switch > 0) {
    b = burn_ms * 30 / burn_up_ms;
  }
  else {
    b = (cool_down_ms - cool_ms) * 30 / cool_down_ms;
  }
  for (; b > 0; b -= 1) {
    Serial.print('|');
  }  
}

void start_burn()
{
  // recalculate burn time from remaining cool time
  float rem = cool_down_ms - cool_ms; // get remaining cool time
  rem /= cool_down_ms; // convert to percentage
  burn_ms = burn_up_ms * rem; // scale to burn time
  Serial.print("\tB:");Serial.print(rem); // DEBUG
}

void start_cool()
{
  // recalculate cool time from remaining burn time
  float rem = burn_up_ms - burn_ms; // get remaining burn time
  rem /= burn_up_ms; // convert to percentage
  cool_ms = cool_down_ms * rem; // scale to cool time
  Serial.print("\tC:");Serial.print(rem); // DEBUG
}

void update_buzzer()
{
  if (burn_switch > 0 && burn_ms == burn_up_ms) {
    analogWrite(PIN_BUZZER, BUZZER_VOLUME);
  }
  else {
    digitalWrite(PIN_BUZZER, LOW);
  }
}
