#include <Arduino.h>
#include <LowPower.h>
#include <RFM69.h>
#include <SPI.h>

#define SERIAL_SPD 9600
#define LED_PIN 9      // led positive pin
#define LAMP_PIN 13    // lamp pin
#define SENSOR_PIN 0 

// Addresses for this node. CHANGE THESE FOR EACH NODE
#define NETWORKID 0  // Must be the same for all nodes
#define MYNODEID 1   // My node ID
#define TONODEID 2   // Destination node ID

// RFM69 frequency, uncomment the frequency of your module:
#define FREQUENCY RF69_868MHZ
#define ENCRYPT true                   // Set to "true" to use encryption
#define ENCRYPTKEY "TOPSECRETPASSWRD"  // Use the same 16-byte key on all nodes

// Use ACKnowledge when sending messages (or not):
#define USEACK true  
#define ACK_WAIT_TIME 1000
#define LOOP_WAIT_TIME 30000
#define SLEEP_TIME_SEC 24
#define READ_SENSOR_TIMEOUT 100 

#define BUFFER_SIZE 25
#define THRESHOLD_SENSOR_VAL 600  // threshold value for sensor 

float sensorValue = 0.0;  // changed to double to increase precision
char sendBuffer[BUFFER_SIZE] = {};
uint8_t sendLength = 0;  // using 8-bit unsigned integer to reduce RAM usage (max value 255)
const char message_start[] PROGMEM = "yes";
uint32_t startTime =0;

RFM69 radio;

void initSerial() {
  Serial.begin(SERIAL_SPD);
  while (!Serial) {
    ;
  }
}

void initRadio() {
  // Initialize the RFM69HCW:
  Serial.print(F("Node "));
  Serial.print(MYNODEID, DEC);
  Serial.println(F(" ready"));

  // radio.setCS(10);  //uncomment this if using Pro Micro
  radio.initialize(FREQUENCY, MYNODEID, NETWORKID);
  radio.setHighPower();  // Always use this for RFM69HCW

#if (ENCRYPT==true)
    radio.encrypt(ENCRYPTKEY);
#endif
}


// Blink an LED_PIN for a given number of ms
void blinkLed(uint8_t PIN, int DELAY_MS)
{
  digitalWrite(PIN, HIGH);
  delay(DELAY_MS);
  digitalWrite(PIN, LOW);
}

void sleepForSeconds(unsigned int seconds) {
  unsigned int sleepCycles = seconds / 8;
  for (unsigned int i = 0; i < sleepCycles; i++) {
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
  }
}

float getSensorValueAvg() {
  float sensorValueAvg=0;
  for (uint8_t i = 0; i < 100; i++) {
    sensorValueAvg += analogRead(SENSOR_PIN);
  }

  sensorValueAvg /= 100; //get average value
  float sensorVolts = sensorValueAvg / 1024 * 5.0;
  Serial.println(F("Sensor value and volts:"))
  Serial.println(sensorValueAvg);
  Serial.println(sensorVolts);
  return sensorValueAvg;
}

void radioSend(float value) {
  memset(sendBuffer, 0, BUFFER_SIZE);

  dtostrf(value, 1, 2, sendBuffer);
  sendLength = strlen(sendBuffer);  // strlen will count length until first '\0' symbol

  Serial.print(F("Sending to node "));
  Serial.print(TONODEID, DEC);
  Serial.print(F(" message ["));
  Serial.print(sendBuffer);
  Serial.print(F("]\n"));

#if (USEACK==true)
  if (radio.sendWithRetry(TONODEID, sendBuffer, sendLength)) {
    Serial.println("ACK received!");
  } else {
    Serial.println("no ACK received");
  }
#else
  radio.send(TONODEID, sendBuffer, sendLength);
#endif
  //digitalWrite(LAMP_PIN, HIGH);
}

void radioRecv(){
  if (radio.receiveDone())
  {
    // The actual message is contained in the DATA array,
    // and is DATALEN bytes in size:
    Serial.print(F("Received from node "));
    Serial.print(radio.SENDERID, DEC);
    Serial.print(F(", message ["));

    for (uint8_t i = 0; i < radio.DATALEN; i++){
      Serial.print((char)radio.DATA[i]);
    }
    // something like command handler
    if (!strcmp(radio.DATA, "read"))
    {
      radioSend(sensorValue);
    }
    // RSSI is the "Receive Signal Strength Indicator",
    // smaller numbers mean higher power.
    Serial.print(F("], RSSI "));
    Serial.println(radio.RSSI);

    // Send an ACK if requested.
    // (You don't need this code if you're not using ACKs.)
    if (radio.ACKRequested()) {
      radio.sendACK();
      Serial.println(F("ACK sent"));
    }
    blinkLed(LED_PIN, 10);
  }
}

void setup() {
  initSerial();
  initRadio();

  pinMode(LAMP_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  delay(20000); //wait to stabilize
  startTime = millis();
}

void loop() {
  radio.sendWithRetry(2, message_start, sizeof(message_start), 3, ACK_WAIT_TIME);
  startTime = millis();
  uint32_t sensorReadTime = millis();

  while (millis() - startTime < LOOP_WAIT_TIME) {

    if (millis() - sensorReadTime > READ_SENSOR_TIMEOUT) {
      sensorValue = getSensorValueAvg();
      sensorReadTime = millis();
    }

    // if sensor value is higher than threshold, send message
    if (sensorValue > THRESHOLD_SENSOR_VAL) {
      digitalWrite(LAMP_PIN, LOW);
      radioSend(sensorValue);
    } else {
      digitalWrite(LAMP_PIN, HIGH);
    }

    radioRecv();
  }
  sleepForSeconds(SLEEP_TIME_SEC);
}