/*

GitHub upload mark UV lamps and voltage sense
================================================================================
                          Wall-E UV Lamp Monitor System
                      LoRa-based Distributed Monitoring
================================================================================

Made by: Andani E. López Aréchar

SYSTEM DESCRIPTION:
  Wall-E is a distributed monitoring system for UV disinfectant lamps in
  production lines. The system operates at 110Vac and uses ESP32 microcontrollers
  with LoRa communication on 433MHz band.

LAMPS DESCRIPTION:
  - UV lamps are used for disinfection air in production lines
  - Model: Microbial Area Kleaner MAK-414: It seems to has 4 UV lamps per unit, works at 110Vac, 4.2A current in total (1.05A per lamp)
  - Each lamp has a current sensor (SCT-013-030) to monitor its operation
  - AC voltage sensor (ZMPT101B) detects presence/absence of mains power


ARCHITECTURE:
  - Central Hub: Raspberry Pi 4 (receiver/coordinator)
  - Remote Nodes: Up to 9 ESP32 units (transmitters/responders)
  - Communication: LoRa point-to-point on 433MHz
  - Protocol: Request/Response with acknowledgment

MONITORING CAPABILITIES:
  1. AC Current Monitoring (4x SCT-013-030 current transformers)
  2. AC Power Presence/Absence detection (ZMPT101B voltage sensor)
  3. Multi-sensor support (4 current channels per node)
  4. Automatic status reporting on request

HARDWARE COMPONENTS:
  - ESP32-S3 WROOM DevKit microcontroller
  - SX1278 LoRa module (433MHz)
  - 4x SCT-013-030 Current Transformers (AC current monitoring)
  - AC voltage monitor (ZMPT101B voltage sensor)
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
    [NET_ID | MSG_RESP | DEVICE_ID | SEQ_LO | SEQ_HI | AC_V_SCALED | CURR1_mA | CURR2_mA | CURR3_mA | CURR4_mA]
    - NET_ID: Echo network ID
    - MSG_RESP: Message type = 0x90 (response)
    - DEVICE_ID: 1-9 (sender device ID)
    - SEQ: 16-bit sequence number for tracking
    - AC_V_SCALED: Voltage scaled 0-255 (maps to 0-130V RMS)
    - CURR1_mA: Current scaled 0-255 (maps to 0-2550 mA)
    - CURR2_mA: Current scaled 0-255 (maps to 0-2550 mA)
    - CURR3_mA: Current scaled 0-255 (maps to 0-2550 mA)
    - CURR4_mA: Current scaled 0-255 (maps to 0-2550 mA)

STATUS CODES:
  0 = OK (device/lamp working normally)
  1 = ALERT (device/lamp failure or power loss)

OPERATION FLOW:
  1. ESP32 initializes and enters listening mode
  2. Continuously monitors sensor values (ADC pins)
  3. When request arrives, validates NET_ID and TARGET_ID
  4. If match found, reads current sensor values
  5. Scales sensor values to 8-bit format for transmission
  6. Sends response packet back to coordinator
  7. Returns to listening mode

ADJUSTMENTS PER INSTALLATION:
  - TX_ID: Set to 1-9 for each device
  - CURRENT_THRESHOLD_MIN: Adjust based on lamp specifications (default 0.5A)
  - CURRENT_THRESHOLD_MAX: Adjust based on lamp specifications (default 1.2A)
  - AC_VOLTAGE_THRESHOLD: Alert if < 100V RMS (default)
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

// ---- ADC Sensor Pins (ESP32-S3 ADC1 channels) ----
// Valid ADC1 pins on ESP32-S3: GPIO0-8, GPIO14-15
#define CURRENT_SENSOR1 4     // GPIO4 (ADC1_CH3) - SCT-013-030 Channel 1
#define CURRENT_SENSOR2 5     // GPIO5 (ADC1_CH4) - SCT-013-030 Channel 2
#define CURRENT_SENSOR3 6     // GPIO6 (ADC1_CH5) - SCT-013-030 Channel 3
#define CURRENT_SENSOR4 7     // GPIO7 (ADC1_CH6) - SCT-013-030 Channel 4
#define AC_POWER_PIN    15    // GPIO15 (ADC1_CH14) - AC voltage detector (ZMPT101B)

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
#define COLOR_CYAN      neopixel.Color(0, 255, 255)
#define COLOR_WHITE     neopixel.Color(255, 255, 255)
#define COLOR_PURPLE    neopixel.Color(128, 0, 128)


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
// SENSOR CALIBRATION & CONVERSION
// ============================================================================
/*
 * SCT-013-030 Current Sensor Calibration:
 *   - Sensor ratio: 30A / 1V (output)
 *   - At 1A nominal: outputs ~33.3mV (with signal conditioning)
 *   - ESP32-S3 ADC: 12-bit (0-4095 = 0-3.3V)
 *   - ADC resolution: 3.3V / 4095 = 0.8056 mV per step
 *
 * ZMPT101B AC Voltage Sensor Calibration:
 *   - Input: 120V RMS AC mains
 *   - Peak voltage: 120V RMS × √2 = 169.7V peak
 *   - DC Offset: Fixed 1.65V (mid-point of 3.3V)
 *   - ADC output range: 1.36V min to 2.0V max
 *   - Peak AC voltage: (2.0V - 1.36V) / 2 = 0.32V peak
 *   - RMS AC voltage: 0.32V peak / √2 = 0.226V RMS
 *   - Conversion ratio: 120V RMS / 0.226V RMS = 531 V/V
 *   - Alert threshold: < 100V RMS
 *   
 * IMPORTANT: Both sensors use AC voltage oscillating around a DC center point.
 * Measurement process:
 *   1. Sample 30 times (~1-2ms, appropriate for 50/60Hz AC)
 *   2. Remove DC offset from each sample
 *   3. Calculate RMS: sqrt(sum of squares / number of samples)
 *   4. Convert RMS to actual measurement (current in A, or voltage in V)
 */

#define ADC_SAMPLES 200       // Number of samples for RMS calculation (increased for stability)

// Current sensor chain calibration (CT 100A:50mA + burden 100R + op-amp gain 91)
// V_out_rms = I_primary * (R_burden * G) / CT_TURNS_RATIO
const float CT_PRIMARY_A = 100.0f;
const float CT_SECONDARY_mA = 50.0f;
const float BURDEN_OHMS = 100.0f;
const float AMP_GAIN = 91.0f;
const float CT_TURNS_RATIO = CT_PRIMARY_A / (CT_SECONDARY_mA / 1000.0f); // 2000:1
const float V_PER_A_RMS = (BURDEN_OHMS * AMP_GAIN) / CT_TURNS_RATIO;     // V_rms at ADC per A
const float CURRENT_RATIO = 1.0f / V_PER_A_RMS;                          // A per V_rms at ADC

#define VOLTAGE_RATIO 531     // ZMPT101B conversion: 531 V/V (120V RMS / 0.226V RMS)
#define CURRENT_FLOOR_A 0.0   // Readings below this are clamped to 0A (noise suppression)

// Only sensor 1 is connected in this setup
#define USE_ONLY_SENSOR1 true

// TEST/SIMULATION MODE: Set to true to simulate sensor values for communication testing
#define SIMULATE_MODE true

// Simulated sensor values (updated every 5 seconds when SIMULATE_MODE is true)
float sim_current1_A = 0.5;
float sim_current2_A = 0.5;
float sim_current3_A = 0.5;
float sim_current4_A = 0.5;
float sim_voltage_V = 125.0;
static unsigned long lastSimUpdate = 0;

// Current sensor readings (RMS values in Amperes)
float current_sensor1_A = 0.0;
float current_sensor2_A = 0.0;
float current_sensor3_A = 0.0;
float current_sensor4_A = 0.0;

// Moving average filter for smoothing (10-sample average)
#define FILTER_SAMPLES 10
float filter_curr1[FILTER_SAMPLES] = {0};
float filter_curr2[FILTER_SAMPLES] = {0};
float filter_curr3[FILTER_SAMPLES] = {0};
float filter_curr4[FILTER_SAMPLES] = {0};
uint8_t filter_index1 = 0;
uint8_t filter_index2 = 0;
uint8_t filter_index3 = 0;
uint8_t filter_index4 = 0;

// DEBUG variables for diagnostics
float debug_raw_adc_avg = 0;
float debug_rms_curr1 = 0;
float debug_min_mV = 0;
float debug_max_mV = 0;
float AC_voltage_V = 0.0;     // AC voltage in Volts RMS


// ============================================================================
// SENSOR THRESHOLDS
// ============================================================================
/*
 * CURRENT_THRESHOLD_MIN / CURRENT_THRESHOLD_MAX (in Amperes):
 *   - Based on expected lamp current consumption
 *   - If measured current < MIN or > MAX → ALERT
 *   - If MIN ≤ measured current ≤ MAX → OK
 *   - For UV lamps: typical range 0.5A to 1.0A
 *
 * AC_THRESHOLD:
 *   - ZMPT101B voltage monitoring output
 *   - If ADC < threshold → No AC power (ALERT)
 *   - If ADC ≥ threshold → AC power present (OK)
 *   - Typical range: 1500-2500 depending on circuit
 *
 * CALIBRATION PROCEDURE:
 * 1. Upload this code with debug enabled
 * 2. Open Serial Monitor (Tools → Serial Monitor, 115200 baud)
 * 3. Monitor "Current: CURR1=XXX.XXmA CURR2=XXX.XXmA ..." values with normal loads
 * 4. Set MIN/MAX thresholds to bracket normal operation
 * 5. Test with faulty/missing lamps to verify ALERT state
 */

float CURRENT_THRESHOLD_MIN = 0.5;   // Minimum acceptable current (Amperes)
float CURRENT_THRESHOLD_MAX = 1.2;   // Maximum acceptable current (Amperes)
float AC_VOLTAGE_THRESHOLD = 100.0;  // Minimum acceptable voltage (Volts RMS) - Alert if < 100V

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

uint16_t seq = 0;             // Response sequence counter (incremented each reply)

// ============================================================================
// SETUP FUNCTION - Initialization (runs once at power-on/reset)
// ============================================================================

void setup() {
  // Initialize LED for visual feedback
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Initialize NeoPixel early (used for error/status indicators)
  neopixel.begin();
  neopixel.setPixelColor(0, COLOR_OFF);
  neopixel.show();

  // Initialize UART serial for debugging output
  Serial.begin(115200);
  // while (!Serial);           // COMENTADO: Bloqueaba el inicio cuando no hay PC conectada
  delay(500);                   // Breve espera para que Serial se inicialice si está disponible

  // Calibrate ADC for accurate millivolt readings
  analogReadResolution(12);
  analogSetPinAttenuation(CURRENT_SENSOR1, ADC_11db);
  analogSetPinAttenuation(CURRENT_SENSOR2, ADC_11db);
  analogSetPinAttenuation(CURRENT_SENSOR3, ADC_11db);
  analogSetPinAttenuation(CURRENT_SENSOR4, ADC_11db);
  analogSetPinAttenuation(AC_POWER_PIN, ADC_11db);
  
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
  LoRa.receive();                        // Ensure radio stays in RX mode

  // Print initialization complete message
  Serial.println("✓ LoRa initialized successfully!");
  Serial.println("  Frequency: 433 MHz");
  Serial.println("  Spreading Factor: 7");
  Serial.println("  Bandwidth: 125 kHz");
  Serial.println("  Coding Rate: 4/5");
  Serial.println("  Waiting for requests from coordinator...");
  Serial.println("========================================\n");

  // Standby: Azul
  neopixel.setPixelColor(0, COLOR_BLUE);
  neopixel.show();
  digitalWrite(LED_PIN, HIGH);
}

// ============================================================================
// HELPER FUNCTION - Moving Average Filter
// ============================================================================
/*
 * Applies a moving average low-pass filter to smooth noisy sensor readings.
 * Reduces oscillations without significantly delaying response.
 */
float applyMovingAverage(float newValue, float buffer[FILTER_SAMPLES], uint8_t &index) {
  // Store new value in circular buffer
  buffer[index] = newValue;
  index = (index + 1) % FILTER_SAMPLES;
  
  // Calculate average
  float sum = 0;
  for (uint8_t i = 0; i < FILTER_SAMPLES; i++) {
    sum += buffer[i];
  }
  return sum / FILTER_SAMPLES;
}


// ============================================================================
// HELPER FUNCTION - RMS Calculation for AC Current Measurement
// ============================================================================
/*
 * Samples an ADC pin multiple times and calculates RMS (Root Mean Square) value.
 * RMS is the effective AC voltage, which converts to current via the sensor ratio.
 * 
 * PARAMETERS:
 *   pin: ADC pin to sample
 *   samples: Number of samples to take (default 30)
 *   
 * RETURNS:
 *   Current in Amperes (RMS)
 *   
 * TIMING:
 *   ~30 samples = ~1-2ms (appropriate for 50/60Hz AC sampling)
 */
float readRMS_and_convertToCurrent(uint8_t pin, uint16_t samples = ADC_SAMPLES) {
  float sumSquares = 0.0f;
  float sum_mV = 0.0f;
  int min_mV = 4095;
  int max_mV = 0;

  // Take samples and accumulate squared values
  for (uint16_t i = 0; i < samples; i++) {
    int raw_mV = analogReadMilliVolts(pin);
    sum_mV += raw_mV;

    if (raw_mV < min_mV) min_mV = raw_mV;
    if (raw_mV > max_mV) max_mV = raw_mV;

    // Accumulate for RMS calculation (centered later using mean)
    sumSquares += (float)raw_mV * (float)raw_mV;

    delayMicroseconds(100);  // Small delay between samples
  }

  // Calculate RMS using mean-centering: rms = sqrt(E[x^2] - (E[x])^2)
  float mean_mV = sum_mV / (float)samples;
  float meanSquares = (sumSquares / (float)samples) - (mean_mV * mean_mV);
  if (meanSquares < 0.0f) {
    meanSquares = 0.0f;
  }
  float rms_voltage_mV = sqrtf(meanSquares);

  // Store debug info for CURR1
  if (pin == CURRENT_SENSOR1) {
    debug_raw_adc_avg = mean_mV;
    debug_rms_curr1 = rms_voltage_mV;
    debug_min_mV = (float)min_mV;
    debug_max_mV = (float)max_mV;
  }

  // Convert RMS voltage to current using sensor ratio
  float current_A = (rms_voltage_mV / 1000.0f) * CURRENT_RATIO;

  return current_A;
}

// ============================================================================
// HELPER FUNCTION - RMS Calculation for AC Voltage Measurement (ZMPT101B)
// ============================================================================
/*
 * Samples ZMPT101B pin multiple times and calculates RMS voltage value.
 * ZMPT101B has fixed 1.65V DC offset with AC signal modulation.
 * 
 * PARAMETERS:
 *   pin: ADC pin to sample
 *   samples: Number of samples to take (default 30)
 *   
 * RETURNS:
 *   Voltage in Volts RMS (0-120V range)
 *   
 * TIMING:
 *   ~30 samples = ~1-2ms (appropriate for 50/60Hz AC sampling)
 */
float readRMS_and_convertToVoltage(uint8_t pin, uint16_t samples = ADC_SAMPLES) {
  float sumSquares = 0.0f;
  float sum_mV = 0.0f;

  // Take samples and accumulate squared values
  for (uint16_t i = 0; i < samples; i++) {
    int raw_mV = analogReadMilliVolts(pin);

    sum_mV += raw_mV;
    sumSquares += (float)raw_mV * (float)raw_mV;

    delayMicroseconds(100);  // Small delay between samples
  }

  // Calculate RMS using mean-centering: rms = sqrt(E[x^2] - (E[x])^2)
  float mean_mV = sum_mV / (float)samples;
  float meanSquares = (sumSquares / (float)samples) - (mean_mV * mean_mV);
  if (meanSquares < 0.0f) {
    meanSquares = 0.0f;
  }
  float rms_voltage_mV = sqrtf(meanSquares);

  // Convert RMS voltage to actual mains voltage using calibration ratio
  float voltage_V = (rms_voltage_mV / 1000.0f) * VOLTAGE_RATIO;

  return voltage_V;
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
// SCALING FUNCTIONS - Convert measurements to 8-bit packed format
// ============================================================================
/*
 * Option 1: Minimal packet format
 * - Voltage: 0-255 represents 0-130V RMS (0.51 V/step)
 * - Current: 0-255 represents 0-2550 mA (10 mA/step)
 *
 * This allows coordinator to display actual measurements within 10-byte packet
 */

uint8_t scale_voltage(float voltage_V) {
  // Map 0-130V RMS to 0-255
  if (voltage_V < 0) return 0;
  if (voltage_V > 130.0) return 255;
  return (uint8_t)((voltage_V / 130.0) * 255.0);
}

uint8_t scale_current(float current_mA) {
  // Map 0-2550 mA to 0-255
  if (current_mA < 0) return 0;
  if (current_mA > 2550.0) return 255;
  return (uint8_t)((current_mA / 2550.0) * 255.0);
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

  // Pulso azul en standby para indicar que el loop está activo
  static unsigned long lastPulse = 0;
  static bool pulseState = false;
  unsigned long now = millis();
  if (now - lastPulse > 1000) { // cada 1 segundo
    if (!pulseState) {
      neopixel.setPixelColor(0, COLOR_OFF);
      neopixel.show();
      pulseState = true;
      lastPulse = now;
    } else {
      neopixel.setPixelColor(0, COLOR_BLUE);
      neopixel.show();
      pulseState = false;
      lastPulse = now;
    }
  }

  current_sensor1_A = readRMS_and_convertToCurrent(CURRENT_SENSOR1);
  if (USE_ONLY_SENSOR1) {
    current_sensor2_A = 0.0;
    current_sensor3_A = 0.0;
    current_sensor4_A = 0.0;
  } else {
    current_sensor2_A = readRMS_and_convertToCurrent(CURRENT_SENSOR2);
    current_sensor3_A = readRMS_and_convertToCurrent(CURRENT_SENSOR3);
    current_sensor4_A = readRMS_and_convertToCurrent(CURRENT_SENSOR4);
  }
  AC_voltage_V = readRMS_and_convertToVoltage(AC_POWER_PIN);
  
  // SIMULATION MODE: Generate random values every 5 seconds
  if (SIMULATE_MODE) {
    unsigned long now = millis();
    if (now - lastSimUpdate >= 5000) {
      // Generate new random values
      sim_current1_A = random(0, 1201) / 1000.0f;      // 0-1200 mA → 0-1.200 A
      sim_current2_A = random(0, 1201) / 1000.0f;
      sim_current3_A = random(0, 1201) / 1000.0f;
      sim_current4_A = random(0, 1201) / 1000.0f;
      sim_voltage_V = 100.0f + (random(0, 501) / 10.0f);  // 100-150 V RMS
      lastSimUpdate = now;
      Serial.printf("[SIM] New random values: I1=%.0fmA I2=%.0fmA I3=%.0fmA I4=%.0fmA V=%.1fV\n",
        sim_current1_A * 1000, sim_current2_A * 1000, sim_current3_A * 1000, sim_current4_A * 1000, sim_voltage_V);
    }
    current_sensor1_A = sim_current1_A;
    current_sensor2_A = sim_current2_A;
    current_sensor3_A = sim_current3_A;
    current_sensor4_A = sim_current4_A;
    AC_voltage_V = sim_voltage_V;
    debug_raw_adc_avg = 1610.0;
    debug_rms_curr1 = 25.0;
    debug_min_mV = 1500.0;
    debug_max_mV = 1720.0;
  }
  
  // Apply moving average filter to smooth readings
  current_sensor1_A = applyMovingAverage(current_sensor1_A, filter_curr1, filter_index1);
  if (!USE_ONLY_SENSOR1) {
    current_sensor2_A = applyMovingAverage(current_sensor2_A, filter_curr2, filter_index2);
    current_sensor3_A = applyMovingAverage(current_sensor3_A, filter_curr3, filter_index3);
    current_sensor4_A = applyMovingAverage(current_sensor4_A, filter_curr4, filter_index4);
  }
  
  // Apply noise floor: clamp weak readings to 0A
  if (current_sensor1_A < CURRENT_FLOOR_A) current_sensor1_A = 0.0;
  if (current_sensor2_A < CURRENT_FLOOR_A) current_sensor2_A = 0.0;
  if (current_sensor3_A < CURRENT_FLOOR_A) current_sensor3_A = 0.0;
  if (current_sensor4_A < CURRENT_FLOOR_A) current_sensor4_A = 0.0;
  
  // Print readings to serial monitor once per second
  static unsigned long lastPrint = 0;
  if (now - lastPrint >= 1000) {
    if (SIMULATE_MODE) {
      Serial.println("*** SIMULATION MODE ACTIVE ***");
    }
    Serial.printf("Current: CURR1=%.2fA CURR2=%.2fA CURR3=%.2fA CURR4=%.2fA | AC Voltage: %.1fV RMS\n",
      current_sensor1_A, current_sensor2_A, current_sensor3_A, current_sensor4_A, AC_voltage_V);
    float vpp_mV = debug_max_mV - debug_min_mV;
    Serial.printf("[DEBUG CURR1] AVG=%.1fmV RMS=%.2fmV Vpp=%.1fmV min=%.0fmV max=%.0fmV -> %.4fA (A/V=%.4f)\n",
      debug_raw_adc_avg, debug_rms_curr1, vpp_mV, debug_min_mV, debug_max_mV, current_sensor1_A, CURRENT_RATIO);
    lastPrint = now;
  }

  // ========================================================================
  // STEP 2: CHECK FOR INCOMING LORA PACKETS (REQUEST FROM COORDINATOR)
  // ========================================================================
  /*
   * LoRa.parsePacket() returns packet size if data available, 0 otherwise
   * This is non-blocking - we don't wait, just check
   */

  int packetSize = LoRa.parsePacket();

  if (packetSize > 0) {
    // Recibiendo paquete: morado
    neopixel.setPixelColor(0, COLOR_PURPLE);
    neopixel.show();

    Serial.printf("📡 Packet received! Size: %d bytes, RSSI: %d dBm\n", packetSize, LoRa.packetRssi());

    if (packetSize != 4) {
      // Invalid size - discard and flush buffer
      Serial.print("⚠ Invalid packet size (expected 4, got ");
      Serial.print(packetSize);
      Serial.print("): ");
      while (LoRa.available()) {
        Serial.printf("%02X ", LoRa.read());
      }
      Serial.println();
      LoRa.receive(); // Return to RX after flushing
      // Regresa a standby azul
      neopixel.setPixelColor(0, COLOR_BLUE);
      neopixel.show();
    } else {
      // Parse the 4-byte request packet
      uint8_t net     = LoRa.read();    // Byte 0: Network ID
      uint8_t type    = LoRa.read();    // Byte 1: Message type
      uint8_t tgtId   = LoRa.read();    // Byte 2: Target device ID
      uint8_t req     = LoRa.read();    // Byte 3: Request code

      // Mostrar el paquete recibido
      Serial.printf("   Packet: [%02X %02X %02X %02X]\n", net, type, tgtId, req);
      Serial.printf("   Expecting: [%02X %02X %02X %02X]\n", NET_ID, MSG_REQ, TX_ID, REQ_READ_DATA);

      // ====================================================================
      // STEP 3: VALIDATE REQUEST
      // ====================================================================
      if (net == NET_ID && type == MSG_REQ && tgtId == TX_ID && req == REQ_READ_DATA) {
        // Scale measurements to 8-bit format for transmission
        uint8_t ac_v_scaled = scale_voltage(AC_voltage_V);
        uint8_t curr1_scaled = scale_current(current_sensor1_A * 1000.0);  // Convert A to mA
        uint8_t curr2_scaled = scale_current(current_sensor2_A * 1000.0);
        uint8_t curr3_scaled = scale_current(current_sensor3_A * 1000.0);
        uint8_t curr4_scaled = scale_current(current_sensor4_A * 1000.0);

        // Espera aleatoria
        delay(random(10, 80));

        // Enviando: cyan
        neopixel.setPixelColor(0, COLOR_CYAN);
        neopixel.show();

        // Build and send response (10-byte packet with scaled values)
        LoRa.beginPacket();
        LoRa.write(NET_ID);                           // Echo network ID
        LoRa.write(MSG_RESP);                         // Message type: Response (0x90)
        LoRa.write(TX_ID);                            // Our device ID
        LoRa.write((uint8_t)(seq & 0xFF));            // Sequence number (low byte)
        LoRa.write((uint8_t)((seq >> 8) & 0xFF));     // Sequence number (high byte)
        LoRa.write(ac_v_scaled);                      // Scaled AC voltage (0-255 = 0-130V RMS)
        LoRa.write(curr1_scaled);                     // Scaled current 1 (0-255 = 0-2550 mA)
        LoRa.write(curr2_scaled);                     // Scaled current 2 (0-255 = 0-2550 mA)
        LoRa.write(curr3_scaled);                     // Scaled current 3 (0-255 = 0-2550 mA)
        LoRa.write(curr4_scaled);                     // Scaled current 4 (0-255 = 0-2550 mA)
        LoRa.endPacket();
        LoRa.receive(); // Ensure radio returns to RX mode

        // Incrementar secuencia
        seq++;

        // Debug output
        Serial.printf("[Device %d] Response sent (Seq=%d): ", TX_ID, seq-1);
        Serial.printf("AC=%.1fV(%d) CURR1=%.2fA(%d) CURR2=%.2fA(%d) CURR3=%.2fA(%d) CURR4=%.2fA(%d)\n",
          AC_voltage_V, ac_v_scaled,
          current_sensor1_A, curr1_scaled,
          current_sensor2_A, curr2_scaled,
          current_sensor3_A, curr3_scaled,
          current_sensor4_A, curr4_scaled);

        // Fin de envío: verde
        neopixel.setPixelColor(0, COLOR_GREEN);
        neopixel.show();
        delay(100);
        // Regresa a standby azul
        neopixel.setPixelColor(0, COLOR_BLUE);
        neopixel.show();
      } else {
        // No es para este nodo, regresa a standby azul
        Serial.println("   ⚠ Packet validation failed (not for this device or wrong format)");
        LoRa.receive(); // Return to RX after ignoring packet
        neopixel.setPixelColor(0, COLOR_BLUE);
        neopixel.show();
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
