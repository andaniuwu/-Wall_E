/*
Made by: Andani E. López Aréchar


  Wall-E program
  introduction: Wall-E is a system made to monitor a UV disinfectant lamps in a production line
                lamps wors at 110Vac
                This program is designed to run on an ESP32 microcontroller with LoRa WAN capabilities
                programmed using Arduino IDE

                In this case, we will have 2 systems, one system of 7 UV lamps working at 433MHz
                and other system of 9 UV lamps working at 433MHz

  General operation: 
                1. Monitor the lamp status (ON/OFF) using a light sensor
                2. If the lamp is OFF when it should be ON, send an alert via LoRa WAN
                3. If AC power is lost, send an alert via LoRa WAN

                The central system will monitor multiple Wall-E units and take action based on the received alerts
                based on Raspberry Pi with LoRa WAN gateway

  Hardware components:
                - ESP32 s3 WROOM DevKit microcontroller
                - LoRa WAN module (e.g., RFM95, in this case, Sx1278 for 433MHz)
                - Light sensor (e.g., photoresistor or photodiode)
                - AC power monitoring circuit ZMPT101B
                - Power supply for ESP32 and LoRa module
                - Enclosure for protection in industrial environment


*/

#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>


// ---------- Pin definitions (ESP32) ----------
#define UV_sensor1      34    // ADC1_CH6 (input only)
#define UV_sensor2      35    // ADC1_CH7 (input only)
#define AC_power_pin    32    // ADC1_CH4


// LoRa pins for ESP32 (adjust as per your wiring)
#define LORA_SS 18
#define LORA_RST 14
#define LORA_DIO0 26


// ---------- Node identity ----------
#define TX_ID           1     // unique 1..9 for each transmitter

// ---------- App-level protocol ----------
#define NET_ID          0xA5  // your private network/application ID
#define MSG_REQ         0x10  // coordinator's request type
#define MSG_RESP        0x90  // node's response type



// ---------- Globals ----------
uint16_t seq = 0;             // response sequence number

int UV_threshold = 2000; //threshold for UV sensor to consider lamp ON
int AC_threshold = 2000; //threshold for AC power presence

int UV_sensor1_value = 0; //variable to store UV sensor 1 value
int UV_sensor2_value = 0; //variable to store UV sensor 2 value
int AC_power_value = 0; //variable to store AC power monitoring value

//led test pin 
int ledPin = 2; // onboard LED GPIO2

/* Optional: basic averaging to reduce noise
int readAveragedADC(uint8_t pin, uint8_t samples = 4) {
  long sum = 0;
  for (uint8_t i = 0; i < samples; i++) {
    sum += analogRead(pin);
    delayMicroseconds(200); // tiny settle time
  }
  return (int)(sum / samples);
}
  */







void setup() {

  pinMode(ledPin, OUTPUT); // onboard LED to indicate running

  Serial.begin(115200);
  while (!Serial);
  Serial.println("Wall-E UV Lamp Monitor Starting...");
  // Set LoRa pins
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);

  // Keep your network private on-air (PHY filtering)
  LoRa.setSyncWord(0x21);                       // common private value
  LoRa.enableCrc();                             // PHY integrity check

  // Keep airtime short if your link allows
  LoRa.setSpreadingFactor(7);                   // valid: 7..12
  LoRa.setSignalBandwidth(125E3);               // valid: 7.8E3..500E3
  LoRa.setCodingRate4(5);                       // valid: 5,6,

  //Initialize LoRa module
  //replace the LoRa.begin(---E-) argument with your location's frequency 
  //433E6 for Asia,  This means 433 MHz (E6 stands for ×10^6, so 433E6 = 433,000,000 Hz).
  //866E6 for Europe,  866 MHz.
  //915E6 for North America, 915 MHz.
  while (!LoRa.begin(433E6)) {
  
    Serial.println("Starting LoRa failed! Retrying...");
    for (int i = 0; i < 10; i++) {
      digitalWrite(ledPin, !digitalRead(ledPin));
      delay(100);
    }
  }
  if (LoRa.begin(433E6)) { 
    Serial.println("LoRa Initialized.");
  }



}



void loop() {

  //blinkLED(); In embedded LED esp32 to test running, GPIO2
  // digitalWrite(ledPin, HIGH);
  // delay(1000);
  // digitalWrite(ledPin, LOW);


  //write to Serial Monitor for debugging and test
  Serial.println("Reading sensors...");
  
/*Process:
  ESP32 reads UV sensor values and AC power monitoring value
  Constantly checks for LoRa messages
  If received ID matches TX_ID and request == 1, send response packet: ID, AC_status, UV1_status, UV2_status
  Status: 0 = OK, 1 = Alert
*/ 


  /* To use averaging to reduce noise, **uncomment this section and the readAveragedADC function above, comment the direct analogRead lines below**
  // Sample sensors (simple average)
  int UV_sensor1_value = readAveragedADC(UV_sensor1);
  int UV_sensor2_value = readAveragedADC(UV_sensor2);
  int AC_power_value   = readAveragedADC(AC_power_pin);
  */

  UV_sensor1_value = analogRead(UV_sensor1);  //Read UV sensor values
  UV_sensor2_value = analogRead(UV_sensor2);
  AC_power_value = analogRead(AC_power_pin);  //Read AC power monitoring value

  
// ---- Parse incoming requests (polling) ----
  int packetSize = LoRa.parsePacket();          // API: LoRa.parsePacket()
  if (packetSize > 0) {
    // We expect 4 bytes: [NET_ID | MSG_REQ | TARGET_ID | REQ_CODE]
    if (packetSize != 4) {
      while (LoRa.available()) LoRa.read();     // flush unexpected
    } else {
      uint8_t net   = LoRa.read();
      uint8_t type  = LoRa.read();
      uint8_t tgtId = LoRa.read();
      uint8_t req   = LoRa.read();

      // Basic validation
      if (net == NET_ID && type == MSG_REQ && tgtId == TX_ID && req == 1) {
        // Determine statuses: 0 = OK, 1 = Alert
        uint8_t ac_status  = (AC_power_value   < AC_threshold) ? 1 : 0;
        uint8_t uv1_status = (UV_sensor1_value < UV_threshold) ? 1 : 0;
        uint8_t uv2_status = (UV_sensor2_value < UV_threshold) ? 1 : 0;

        // Small random jitter to avoid aligned replies across nodes
        delay(random(0, 80));

        // Build response:
        // [NET_ID | MSG_RESP | DEVICE_ID | SEQ_LO | SEQ_HI | AC | UV1 | UV2]
        LoRa.beginPacket();                    // API: beginPacket()
        LoRa.write(NET_ID);
        LoRa.write(MSG_RESP);
        LoRa.write(TX_ID);
        LoRa.write((uint8_t)(seq & 0xFF));
        LoRa.write((uint8_t)((seq >> 8) & 0xFF));
        LoRa.write(ac_status);
        LoRa.write(uv1_status);
        LoRa.write(uv2_status);
        LoRa.endPacket();                      // API: endPacket()

        seq++;
        Serial.printf("Response sent. ADC: AC=%d UV1=%d UV2=%d\n",
                      AC_power_value, UV_sensor1_value, UV_sensor2_value);
      }
    }
  }

  delay(50); // gentle loop
}