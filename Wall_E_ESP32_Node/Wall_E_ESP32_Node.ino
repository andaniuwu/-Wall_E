/*
================================================================================
                          Wall-E UV Lamp Monitor System
                      LoRa-based Distributed Monitoring
================================================================================

Made by: Andani E. López Aréchar

SYSTEM DESCRIPTION:
  Wall-E is a distributed monitoring system for UV disinfectant lamps in
  production lines. The system operates at 110Vac and uses ESP32 microcontrollers
  with LoRa communication on 433MHz band.

ARCHITECTURE:
  - Central Hub: Raspberry Pi 4 (receiver/coordinator)
  - Remote Nodes: Up to 9 ESP32 units (transmitters/responders)
  - Communication: LoRa point-to-point on 433MHz
  - Protocol: Request/Response with acknowledgment

MONITORING CAPABILITIES:
  1. UV Lamp Status (ON/OFF) using light sensors
  2. AC Power Presence/Absence detection
  3. Multi-sensor support (up to 2 UV lamps per node)
  4. Automatic status reporting on request

HARDWARE COMPONENTS:
  - ESP32-S3 WROOM DevKit microcontroller
  - SX1278 LoRa module (433MHz)
  - Light sensors for UV detection (photoresistor or photodiode)
  - AC power monitor (ZMPT101B or similar)
  - Power supply (5V/USB for ESP32, 3.3V for LoRa)
  - Industrial-grade enclosure

COMMUNICATION PROTOCOL:
  Request Packet (from Raspberry Pi):
    [NET_ID | MSG_REQ | TARGET_ID | REQ_CODE]
    - NET_ID: Network identifier (0xA5 for private network)
    - MSG_REQ: Message type = 0x10 (request)
    - TARGET_ID: 1-9 (device ID to query)
    - REQ_CODE: 0x01 (read sensor data)

  Response Packet (from ESP32):
    [NET_ID | MSG_RESP | DEVICE_ID | SEQ_LO | SEQ_HI | AC_STATUS | UV1_STATUS | UV2_STATUS]
    - NET_ID: Echo network ID
    - MSG_RESP: Message type = 0x90 (response)
    - DEVICE_ID: 1-9 (sender device ID)
    - SEQ: 16-bit sequence number for tracking
    - AC_STATUS: 0=OK, 1=NO_POWER
    - UV1_STATUS: 0=OK, 1=NO_LIGHT
    - UV2_STATUS: 0=OK, 1=NO_LIGHT

STATUS CODES:
  0 = OK (device/lamp working normally)
  1 = ALERT (device/lamp failure or power loss)

OPERATION FLOW:
  1. ESP32 initializes and enters listening mode
  2. Continuously monitors sensor values (ADC pins)
  3. When request arrives, validates NET_ID and TARGET_ID
  4. If match found, reads current sensor values
  5. Determines status (OK/ALERT based on thresholds)
  6. Sends response packet back to coordinator
  7. Returns to listening mode

ADJUSTMENTS PER INSTALLATION:
  - TX_ID: Set to 1-9 for each device (see line 79)
  - UV_threshold: Adjust based on sensor calibration (line 129)
  - AC_threshold: Adjust based on AC sensor calibration (line 133)
  - LoRa frequency: Use 433E6 (Asia), 866E6 (Europe), 915E6 (Americas)

================================================================================
*/

#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <Adafruit_NeoPixel.h>

// ----------------------------------------------------------------------------------
// NOTA IMPORTANTE SOBRE PINES SPI EN ESP32-S3:
// El ESP32-S3 permite asignar cualquier función SPI (MOSI, MISO, SCK, CS) a casi
// cualquier GPIO mediante software. Por eso, en el pinout/datasheet solo aparecen
// los números de GPIO y no la función SPI fija. La asignación se realiza así:
//   SPI.begin(SCK, MISO, MOSI, SS);
// En este proyecto:
//   SCK  = GPIO12 (Pin 19, FSPI_CLK)
//   MISO = GPIO13 (Pin 18, FSPI_Q)
//   MOSI = GPIO11 (Pin 17, FSPI_D)
//   NSS  = GPIO18 (Pin 9)
//   RESET= GPIO14 (Pin 13)
//   DIO0 = GPIO26 (Pin 38)
// ----------------------------------------------------------------------------------
// Tabla de conexión SX1278 ↔ ESP32-S3-DevKitC-1:
// SX1278 Pin | ESP32 GPIO | ESP32 Pin# | Función en código
// -----------|------------|------------|------------------
// SCK        | GPIO12     | Pin 19     | SPI SCK
// MISO       | GPIO13     | Pin 18     | SPI MISO
// MOSI       | GPIO11     | Pin 17     | SPI MOSI
// NSS (CS)   | GPIO18     | Pin 9      | LORA_SS
// RESET      | GPIO14     | Pin 13     | LORA_RST
// DIO0       | GPIO46     | Pin 44     | LORA_DIO0
// 3.3V       | 3V3        | Pin 2/3    | Alimentación
// GND        | GND        | Pin 1/15/16| Tierra
// ----------------------------------------------------------------------------------

// ============================================================================
// HARDWARE PIN CONFIGURATION (ESP32)
// ============================================================================

// ---- ADC Sensor Pins ----
#define UV_SENSOR1      34    // ADC1_CH6 (input only) - Main UV lamp sensor
#define UV_SENSOR2      35    // ADC1_CH7 (input only) - Secondary UV lamp sensor
#define AC_POWER_PIN    32    // ADC1_CH4 - AC presence detector (ZMPT101B or similar)

// ---- LoRa Module Pins (SX1278/RFM95) ----
#define LORA_SCK        12    // GPIO12 (Pin 19)
#define LORA_MISO       13    // GPIO13 (Pin 18)
#define LORA_MOSI       11    // GPIO11 (Pin 17)
#define LORA_SS         18    // GPIO18 (Pin 9)
#define LORA_RST        14    // GPIO14 (Pin 13)
#define LORA_DIO0       46    // GPIO46 (Pin 44)

// ---- Indicator LED ----
#define LED_PIN         2     // Onboard LED (GPIO2) - indicates system running

// WS2812 RGB LED (NeoPixel) configuration
#define NEOPIXEL_PIN    48    // GPIO48 for WS2812 addressable RGB LED
#define NEOPIXEL_COUNT  1     // Single LED on most DevKits
Adafruit_NeoPixel neopixel(NEOPIXEL_COUNT, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

// Color constants (GRB format for WS2812)
#define COLOR_OFF       neopixel.Color(0, 0, 0)
#define COLOR_RED       neopixel.Color(0, 255, 0)
#define COLOR_GREEN     neopixel.Color(255, 0, 0)
#define COLOR_BLUE      neopixel.Color(0, 0, 255)
#define COLOR_YELLOW    neopixel.Color(255, 200, 0)
#define COLOR_CYAN      neopixel.Color(255, 0, 255)
#define COLOR_WHITE     neopixel.Color(255, 255, 255)

// ============================================================================
// DEVICE CONFIGURATION
// ============================================================================

/*
 * **CRITICAL: Set TX_ID to 1-9, UNIQUE for each device**
 * 
 * Device 1: TX_ID = 1
 * Device 2: TX_ID = 2
 * Device 3: TX_ID = 3
 * ...
 * Device 9: TX_ID = 9
 *
 * Each ESP32 must have a different TX_ID to identify itself to the coordinator
 */
#define TX_ID           1     // CHANGE THIS FOR EACH DEVICE (1-9) !!!

// ============================================================================
// PROTOCOL CONSTANTS
// ============================================================================

#define NET_ID          0xA5  // Network identifier (private network marker)
#define MSG_REQ         0x10  // Message type: Request from coordinator
#define MSG_RESP        0x90  // Message type: Response from node
#define REQ_READ_DATA   0x01  // Request code: Read sensor data

// ============================================================================
// SENSOR THRESHOLDS
// ============================================================================
/*
 * ADC values are 0-4095 (12-bit resolution on ESP32).
 * Adjust these based on YOUR sensor calibration:
 *
 * UV_THRESHOLD:
 *   - Light sensor reading when UV lamp is ON
 *   - If ADC < threshold → Lamp is OFF (ALERT)
 *   - If ADC ≥ threshold → Lamp is ON (OK)
 *   - Typical range: 1500-3000 depending on sensor type
 *
 * AC_THRESHOLD:
 *   - ZMPT101B voltage monitoring output
 *   - If ADC < threshold → No AC power (ALERT)
 *   - If ADC ≥ threshold → AC power present (OK)
 *   - Typical range: 1500-2500 depending on circuit
 *
 * CALIBRATION PROCEDURE:
 * 1. Upload this code with high thresholds (3000)
 * 2. Open Serial Monitor (Tools → Serial Monitor, 115200 baud)
 * 3. Note "Raw: AC=XXXX UV1=XXXX UV2=XXXX" values in normal operation
 * 4. Set threshold to slightly below normal value
 * 5. Re-upload and test
 */

int UV_THRESHOLD = 2000;      // Adjust based on your sensor
int AC_THRESHOLD = 2000;      // Adjust based on your ZMPT101B circuit

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

uint16_t seq = 0;             // Response sequence counter (incremented each reply)

// Current sensor readings (updated each loop iteration)
int UV_sensor1_value = 0;     // Lamp 1 light intensity (0-4095)
int UV_sensor2_value = 0;     // Lamp 2 light intensity (0-4095)
int AC_power_value = 0;       // AC mains voltage presence (0-4095)

// ============================================================================
// SETUP FUNCTION - Initialization (runs once at power-on/reset)
// ============================================================================

void setup() {
  // Initialize LED for visual feedback
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Initialize UART serial for debugging output
  Serial.begin(115200);
  while (!Serial);           // Wait for Serial Monitor to open
  delay(100);
  
  // Print startup banner
  Serial.println("\n\n========================================");
  Serial.println("   Wall-E UV Monitor - Device " + String(TX_ID));
  Serial.println("========================================");
  Serial.println("Initializing LoRa module...");

  // Configure LoRa module SPI pins
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);

  // Initialize LoRa module with 433 MHz frequency
  // Using standard LoRa parameters for balanced range/speed
  if (!LoRa.begin(433E6)) {
    Serial.println("ERROR: LoRa initialization failed!");
    Serial.println("Possible causes:");
    Serial.println("  - Module not physically connected");
    Serial.println("  - Wrong pin configuration");
    Serial.println("  - SPI bus malfunction");
    
    // Error blink loop:
    // while (true) {
    //   neopixel.setPixelColor(0, COLOR_RED);
    //   neopixel.show();
    //   delay(100);
    //   neopixel.setPixelColor(0, COLOR_OFF);
    //   neopixel.show();
    //   delay(100);
    // }
    neopixel.setPixelColor(0, COLOR_RED);
    neopixel.show();
  }

  // Configure LoRa physical layer parameters
  LoRa.setSpreadingFactor(7);           // SF=7: Medium range, better speed
  LoRa.setSignalBandwidth(125E3);       // Bandwidth: 125 kHz (standard)
  LoRa.setCodingRate4(5);               // Coding rate: 4/5 (standard)
  LoRa.setSyncWord(0x21);               // Sync word: 0x21 (private network)
  LoRa.enableCrc();                     // Enable CRC error checking

  // Print initialization complete message
  Serial.println("✓ LoRa initialized successfully!");
  Serial.println("  Frequency: 433 MHz");
  Serial.println("  Spreading Factor: 7");
  Serial.println("  Bandwidth: 125 kHz");
  Serial.println("  Coding Rate: 4/5");
  Serial.println("  Waiting for requests from coordinator...");
  Serial.println("========================================\n");

  // Turn LED on to indicate ready state
  digitalWrite(LED_PIN, HIGH);
}

// ============================================================================
// HELPER FUNCTION - Optional ADC Averaging (reduce noise)
// ============================================================================
/*
 * Uncomment this function to enable averaging for more stable readings.
 * This is useful if sensors are noisy or in high electrical noise environment.
 *
 * TRADE-OFF:
 *   - Pro: Smoother readings, less false alerts
 *   - Con: Adds ~1ms per read per sample
 *
 * USAGE: Replace analogRead() calls with readAveragedADC()
 *   Example: UV_sensor1_value = readAveragedADC(UV_SENSOR1, 4);
 */

int readAveragedADC(uint8_t pin, uint8_t samples = 4) {
  long sum = 0;
  for (uint8_t i = 0; i < samples; i++) {
    sum += analogRead(pin);
    delayMicroseconds(200);  // Small delay for ADC to settle
  }
  return (int)(sum / samples);
}

// ============================================================================
// MAIN LOOP - Continuous Execution
// ============================================================================
/*
 * OPERATION FLOW:
 * 1. Read all sensor ADC values
 * 2. Check for incoming LoRa packets from coordinator
 * 3. If packet is addressed to us:
 *    a. Evaluate sensor thresholds
 *    b. Build status bytes (0=OK, 1=ALERT)
 *    c. Transmit response back
 * 4. Small delay, then repeat
 */

void loop() {
  // ========================================================================
  // STEP 1: READ SENSOR VALUES FROM ADC INPUTS
  // ========================================================================
  /*
   * Read current analog values from sensors.
   * ADC range: 0-4095 (12-bit on ESP32)
   * 
   * Option A: Direct reading (fast, may have noise)
   * Option B: Averaged reading (smooth, adds latency)
   *
   * If sensors are unstable, try averaging (see helper function above)
   */

  UV_sensor1_value = analogRead(UV_SENSOR1);    // Lamp 1 light level
  UV_sensor2_value = analogRead(UV_SENSOR2);    // Lamp 2 light level
  AC_power_value = analogRead(AC_POWER_PIN);    // AC mains presence

  // Uncomment next 3 lines for noise reduction (comment out direct readings above):
  // UV_sensor1_value = readAveragedADC(UV_SENSOR1, 4);
  // UV_sensor2_value = readAveragedADC(UV_SENSOR2, 4);
  // AC_power_value = readAveragedADC(AC_POWER_PIN, 4);

  // ========================================================================
  // STEP 2: CHECK FOR INCOMING LORA PACKETS (REQUEST FROM COORDINATOR)
  // ========================================================================
  /*
   * LoRa.parsePacket() returns packet size if data available, 0 otherwise
   * This is non-blocking - we don't wait, just check
   */

  int packetSize = LoRa.parsePacket();

  if (packetSize > 0) {
    // A packet was received. Check if it's valid (must be exactly 4 bytes).
    
    if (packetSize != 4) {
      // Invalid size - discard and flush buffer
      while (LoRa.available()) LoRa.read();
      Serial.println("⚠ Received invalid packet (wrong size)");
    } else {
      // Parse the 4-byte request packet
      uint8_t net     = LoRa.read();    // Byte 0: Network ID
      uint8_t type    = LoRa.read();    // Byte 1: Message type
      uint8_t tgtId   = LoRa.read();    // Byte 2: Target device ID
      uint8_t req     = LoRa.read();    // Byte 3: Request code

      // ====================================================================
      // STEP 3: VALIDATE REQUEST
      // ====================================================================
      /*
       * Check if this request is intended for us by validating:
       * 1. NET_ID matches (0xA5)
       * 2. Message type is REQUEST (0x10)
       * 3. Target ID matches our device ID (TX_ID = 1-9)
       * 4. Request code is READ_DATA (0x01)
       *
       * If ANY check fails, ignore the packet (not for us)
       */

      if (net == NET_ID && type == MSG_REQ && tgtId == TX_ID && req == REQ_READ_DATA) {
        
        // ================================================================
        // STEP 4: EVALUATE SENSOR STATUS
        // ================================================================
        /*
         * Compare current sensor readings against thresholds
         * Status byte: 0 = OK (normal), 1 = ALERT (problem detected)
         *
         * AC_STATUS:
         *   - 1 (ALERT) if AC_power_value < AC_THRESHOLD (no power)
         *   - 0 (OK) if AC_power_value >= AC_THRESHOLD (power present)
         *
         * UV1_STATUS, UV2_STATUS:
         *   - 1 (ALERT) if light < UV_THRESHOLD (lamp is off)
         *   - 0 (OK) if light >= UV_THRESHOLD (lamp is on)
         */

        uint8_t ac_status  = (AC_power_value < AC_THRESHOLD)   ? 1 : 0;
        uint8_t uv1_status = (UV_sensor1_value < UV_THRESHOLD) ? 1 : 0;
        uint8_t uv2_status = (UV_sensor2_value < UV_THRESHOLD) ? 1 : 0;

        // Add random delay to stagger responses from multiple devices
        // This prevents collision if coordinator queries multiple nodes simultaneously
        delay(random(10, 80));

        // ================================================================
        // STEP 5: BUILD AND TRANSMIT RESPONSE PACKET
        // ================================================================
        /*
         * Build 8-byte response packet:
         *   [NET_ID | MSG_RESP | DEVICE_ID | SEQ_LO | SEQ_HI | AC | UV1 | UV2]
         *
         * Sequence number (SEQ) is used by coordinator to:
         *   - Detect duplicate packets
         *   - Track response latency
         *   - Correlate responses with requests
         *
         * Incremented after each transmission
         */

        LoRa.beginPacket();
        LoRa.write(NET_ID);                           // Echo network ID
        LoRa.write(MSG_RESP);                         // Message type: Response (0x90)
        LoRa.write(TX_ID);                            // Our device ID
        LoRa.write((uint8_t)(seq & 0xFF));            // Sequence number (low byte)
        LoRa.write((uint8_t)((seq >> 8) & 0xFF));     // Sequence number (high byte)
        LoRa.write(ac_status);                        // AC power status (0/1)
        LoRa.write(uv1_status);                       // UV lamp 1 status (0/1)
        LoRa.write(uv2_status);                       // UV lamp 2 status (0/1)
        LoRa.endPacket();

        // Increment sequence number for next response
        seq++;

        // Print debug information to Serial Monitor
        Serial.printf("[Device %d] Response sent (Seq=%d): ", TX_ID, seq-1);
        Serial.printf("AC=%s UV1=%s UV2=%s | Raw: AC=%d UV1=%d UV2=%d\n",
          (ac_status ? "ALERT" : "OK"),
          (uv1_status ? "ALERT" : "OK"),
          (uv2_status ? "ALERT" : "OK"),
          AC_power_value, UV_sensor1_value, UV_sensor2_value);

        // Blink LED to indicate transmission
        digitalWrite(LED_PIN, LOW);
        delay(10);
        digitalWrite(LED_PIN, HIGH);
      }
    }
  }

  // ========================================================================
  // STEP 6: LOOP DELAY
  // ========================================================================
  // Small delay to avoid busy-waiting while polling LoRa module
  delay(50);
}

// ============================================================================
// END OF PROGRAM
// ============================================================================
