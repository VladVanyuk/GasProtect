#include <LowPower.h>
#include <RFM69.h>
#include <SPI.h>

#define SERIAL_SPD 9600

// Addresses for this node. CHANGE THESE FOR EACH NODE!
#define NETWORKID 0  // Must be the same for all nodes
#define MYNODEID 2   // My node ID
#define TONODEID 1   // Destination node ID

// RFM69 frequency, uncomment the frequency of your module:
#define FREQUENCY RF69_868MHZ
#define ENCRYPT true                   // Set to "true" to use encryption
#define ENCRYPTKEY "TOPSECRETPASSWRD"  // Use the same 16-byte key on all nodes
#define USEACK true  // Use ACKnowledge when sending messages (or not):

#define LOOP_WAIT_TIME 30000

#define LED_PIN 9  // LED_PIN positive pin
const uint8_t led_pin = 13;
const uint8_t motor_pin1 = 6;
const uint8_t buttonStart_pin = 7;
const uint8_t buttonEnd_pin = 8;

float sensorValue = 0.0;
float sensorVolts = 0.0;

// Create a library object for our RFM69HCW module:
RFM69 radio;

void setup() {
  Serial.begin(SERIAL_SPD);

  Serial.print("Node ");
  Serial.print(MYNODEID, DEC);
  Serial.println(" ready");

  pinMode(buttonStart_pin, INPUT_PULLUP);
  pinMode(buttonStart_pin, INPUT_PULLUP);
  pinMode(motor_pin1, OUTPUT);
  pinMode(led_pin, OUTPUT);

  radio.initialize(FREQUENCY, MYNODEID, NETWORKID);
  radio.setHighPower();  // Always use this for RFM69HCW

  // Turn on encryption if desired:
#if (ENCRYPT==true)
    radio.encrypt(ENCRYPTKEY);
#endif
}

void loop() {
  if (radio.receiveDone()) {
    if (radio.ACKRequested()) {
      radio.sendACK();
      Serial.println("ACK sent");
    }
    unsigned long startTime = millis();

    while (millis() - startTime < LOOP_WAIT_TIME) {
      // Set up a "buffer" for characters that we'll send:
      static char sendbuffer[62];
      static int sendlength = 0;

      // SENDING
      // In this section, we'll gather serial characters and
      // send them to the other node if we (1) get a carriage return,
      // or (2) the buffer is full (61 characters).

      // If there is any serial input, add it to the buffer:
      if (Serial.available() > 0) {
        char input = Serial.read();

        if (input != '\r')  // not a carriage return
        {
          sendbuffer[sendlength] = input;
          sendlength++;
        }

        // If the input is a carriage return, or the buffer is full:
        if ((input == '\r') || (sendlength == 61))  // CR or buffer full
        {
          Serial.print("sending to node ");
          Serial.print(TONODEID, DEC);
          Serial.print(", message [");
          for (byte i = 0; i < sendlength; i++)
            Serial.print(sendbuffer[i]);
          Serial.println("]");

          if (USEACK) {
            if (radio.sendWithRetry(TONODEID, sendbuffer, sendlength))
              Serial.println("ACK received!");
            else
              Serial.println("no ACK received");
          } else  // don't use ACK
          {
            radio.send(TONODEID, sendbuffer, sendlength);
          }
          sendlength = 0;  // reset the packet
          blinkLed(LED_PIN, 10);
        }
      }

      // RECEIVING
      // In this section, we'll check with the RFM69HCW to see
      // if it has received any packets:
      if (radio.receiveDone())  // Got one!
      {
        Serial.print("received from node ");
        Serial.print(radio.SENDERID, DEC);
        Serial.print(", message [");

        // The actual message is contained in the DATA array,
        // and is DATALEN bytes in size
        for (byte i = 0; i < radio.DATALEN; i++)
          Serial.print((char)radio.DATA[i]);

        // RSSI is the "Receive Signal Strength Indicator",
        // smaller numbers mean higher power.
        Serial.print("], RSSI ");
        Serial.println(radio.RSSI);

        /* Try to read incoming string into double variable */
        double sensorValue = strtod(radio.DATA, NULL);
        Serial.println("Sensor value = ");
        Serial.print(sensorValue);
        Serial.print('\n');

        delay(100);
         if (sensorValue > 10) {
          Serial.println("TRUE");

          while (true) {
            if (digitalRead(buttonStart_pin) == HIGH) {
              Serial.println("First");
              digitalWrite(motor_pin1, HIGH);
              
            } else if (digitalRead(buttonStart_pin) == HIGH) {
              Serial.println("Third");
              digitalWrite(motor_pin1, LOW);
              break;
            }
          }
        } else {

          Serial.println("FALSE");
        }
      } else {
        delay(50);
      }

      // Send an ACK if requested.
      // (You don't need this code if you're not using ACKs.)
      if (radio.ACKRequested()) {
        radio.sendACK();
        Serial.println("ACK sent");
      }
      // blinkLed(LED_PIN, 10);
    }
    Serial.println("sleep");
    sleepForSeconds(24);
    Serial.println("work");
  }
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