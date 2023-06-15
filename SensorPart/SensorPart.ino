#include <LowPower.h>

#include <RFM69.h>
#include <SPI.h>

// Addresses for this node. CHANGE THESE FOR EACH NODE!

#define NETWORKID 0  // Must be the same for all nodes
#define MYNODEID 1   // My node ID
#define TONODEID 2   // Destination node ID

// RFM69 frequency, uncomment the frequency of your module:

#define FREQUENCY RF69_868MHZ

// AES encryption (or not):

#define ENCRYPT true                   // Set to "true" to use encryption
#define ENCRYPTKEY "TOPSECRETPASSWRD"  // Use the same 16-byte key on all nodes

// Use ACKnowledge when sending messages (or not):

#define USEACK true  // Request ACKs or not
#define ACK_WAIT_TIME 1000
// Packet sent/received indicator LED (optional):

#define LED 9  // LED positive pin
#define GND 8  // LED ground pin

// Create a library object for our RFM69HCW module:

RFM69 radio;
double sensorValue;  // changed to double to increase precision
double sensorVolts;
const int lamp = 13;  // this should be #define, not const int

int counter = 0;
/* Messaging */
#define BUFFER_SIZE 25
char sendBuffer[BUFFER_SIZE] = {};
uint8_t sendLength = 0;  // using 8-bit unsigned integer to reduce RAM usage (max value 255)
void radioSend(double value);

void setup() {
  Serial.begin(9600);
  while (!Serial)
    ;
  Serial.print("Node ");
  Serial.print(MYNODEID, DEC);
  Serial.println(" ready");

  // Initialize the RFM69HCW:
  // radio.setCS(10);  //uncomment this if using Pro Micro
  radio.initialize(FREQUENCY, MYNODEID, NETWORKID);
  radio.setHighPower();  // Always use this for RFM69HCW

  // Turn on encryption if desired:

  if (ENCRYPT)
    radio.encrypt(ENCRYPTKEY);
  Serial.begin(9600);
  delay(20000);
  pinMode(lamp, OUTPUT);
}

void loop() {
  char message[] = "yes";
  radio.sendWithRetry(2, message, sizeof(message), 3, ACK_WAIT_TIME);
  unsigned long startTime = millis();

  while (millis() - startTime < 30000) {

    for (int i = 0; i < 100; i++) {
      sensorValue = sensorValue + analogRead(0);
    }

    sensorValue = sensorValue / 100;
    sensorVolts = sensorValue / 1024 * 5.0;
    Serial.println(sensorValue);
    Serial.println(sensorVolts);
    delay(100);

    // if sensor value is higher than some threshold, send message
    if (sensorValue > 600) {
      digitalWrite(lamp, LOW);
      radioSend(sensorValue);  // sending
    } else {
      digitalWrite(lamp, HIGH);
    }


    // SENDING

    // In this section, we'll gather serial characters and
    // send them to the other node if we (1) get a carriage return,
    // or (2) the buffer is full (61 characters).

    // If there is any serial input, add it to the buffer:

    // RECEIVING

    // In this section, we'll check with the RFM69HCW to see
    // if it has received any packets:

    if (radio.receiveDone())  // Got one!
    {
      // Print out the information:

      Serial.print("received from node ");
      Serial.print(radio.SENDERID, DEC);
      Serial.print(", message [");

      // The actual message is contained in the DATA array,
      // and is DATALEN bytes in size:

      for (byte i = 0; i < radio.DATALEN; i++)
        Serial.print((char)radio.DATA[i]);

      // something like command handler
      /* command "read"
     * should send sensor value 
     */
      if (!strcmp(radio.DATA, "read"))  // not sure if this will work (radio.DATA == "read")
      {
        radioSend(sensorValue);
      }
      // RSSI is the "Receive Signal Strength Indicator",
      // smaller numbers mean higher power.

      Serial.print("], RSSI ");
      Serial.println(radio.RSSI);

      // Send an ACK if requested.
      // (You don't need this code if you're not using ACKs.)

      if (radio.ACKRequested()) {
        radio.sendACK();
        Serial.println("ACK sent");
      }
      Blink(LED, 10);
    }
  }
  sleepForSeconds(24);
}

void sleepForSeconds(unsigned int seconds) {
  unsigned int sleepCycles = seconds / 8;

  for (unsigned int i = 0; i < sleepCycles; i++) {
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
  }
}

void radioSend(double value) {
  memset(sendBuffer, 0, BUFFER_SIZE);  // clear sending buffer

  dtostrf(value, 1, 2, sendBuffer);
  sendLength = strlen(sendBuffer);  // strlen will count length until first '\0' symbol

  Serial.print("sending to node ");
  Serial.print(TONODEID, DEC);
  Serial.print(" message [");
  Serial.print(sendBuffer);
  Serial.print("]\n");

  if (USEACK) {
    if (radio.sendWithRetry(TONODEID, sendBuffer, sendLength))
      Serial.println("ACK received!");
    else
      Serial.println("no ACK received");
  } else  // don't use ACK
  {
    radio.send(TONODEID, sendBuffer, sendLength);
  }
}