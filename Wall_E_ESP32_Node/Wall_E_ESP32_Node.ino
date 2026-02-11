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
  - Each lamp has a current sensor (ACS712T-5A Hall-effect with voltage divisor) to monitor its operation
  - AC voltage sensor (ZMPT101B) monitors mains voltage presence/absence and RMS value


ARCHITECTURE:
  - Central Hub: Raspberry Pi 4 (receiver/coordinator)
  - Remote Nodes: Up to 9 ESP32 units (transmitters/responders)
  - Communication: LoRa point-to-point on 433MHz
  - Protocol: Request/Response with acknowledgment

MONITORING CAPABILITIES:
  1. Dual-mode Current Monitoring: AC (RMS) or DC (average) via ACS712T-5A (4 channels)
  2. AC Voltage monitoring (ZMPT101B) for mains presence detection and RMS voltage
  3. Multi-sensor support (4 current channels per node)
  4. Automatic measurements reporting on request with scaled 8-bit values

HARDWARE COMPONENTS:
  - ESP32-S3 WROOM DevKit microcontroller
  - SX1278 LoRa module (433MHz)
  - 4x ACS712T-5A Hall-effect current sensors with 1:2 voltage divisor (0-5V → 0-2.5V)
  - AC voltage monitor (ZMPT101B with RMS conditioning)
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
    [NET_ID | MSG_RESP | DEVICE_ID | SEQ_LO | SEQ_HI | AC_V_SCALED | CURR1_SCALED | CURR2_SCALED | CURR3_SCALED | CURR4_SCALED]
    - NET_ID: Echo network ID (0xA5)
    - MSG_RESP: Message type = 0x90 (response)
    - DEVICE_ID: 1-9 (sender device ID)
    - SEQ_LO | SEQ_HI: 16-bit sequence number for tracking
    - AC_V_SCALED: AC voltage scaled 0-255 (maps 0.0-130.0 V RMS)
    - CURR1_SCALED: Current 1 scaled 0-255 (maps 0-2550 mA or 0-2.55A)
    - CURR2_SCALED: Current 2 scaled 0-255 (maps 0-2550 mA or 0-2.55A)
    - CURR3_SCALED: Current 3 scaled 0-255 (maps 0-2550 mA or 0-2.55A)
    - CURR4_SCALED: Current 4 scaled 0-255 (maps 0-2550 mA or 0-2.55A)

STATUS INFORMATION:
  Measurements are scaled to 8-bit values to fit in the 10-byte packet:
  - Voltage: 0-255 represents 0-130V RMS with resolution 0.51 V/step
  - Current: 0-255 represents 0-2550 mA (0-2.55A) with resolution 10 mA/step
  - Coordinator can reconstruct original values from scaled values
  - No status codes (0=OK/1=ALERT) in current implementation; all measurements sent directly

OPERATION FLOW:
  1. ESP32 initializes and enters listening mode
  2. Continuously reads sensor values (ADC pins) and applies filtering
  3. When request arrives, validates NET_ID and TARGET_ID
  4. If match found, scales current measurements to 8-bit format
  5. Builds and sends response packet back to coordinator
  6. Returns to listening mode

ADJUSTMENTS PER INSTALLATION:
  - TX_ID: Set to 1-9 for each device (line 149)
  - CURRENT_MEASUREMENT_AC: Set true for AC mode (RMS×5) or false for DC mode (line 265)
  - SIMULATE_MODE: Set true to test without sensors, false for real deployment (line 268)
  - CURRENT_THRESHOLD_MIN/MAX: For optional firmware-level filtering if needed (lines 277-278)
  - AC_VOLTAGE_THRESHOLD: Alert threshold if implementing local logic (line 279)
  - LoRa frequency: 433E6 (Asia), 866E6 (Europe), 915E6 (Americas) (line 259)

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
 * ACS712T-5A Current Sensor Calibration (DC Sensor with voltage divisor):
 *   - Sensor sensitivity: 185 mV/A (at 5V operation)
 *   - DC Offset at no load: 2.5V (VCC/2 = 5V/2)
 *   - With voltage divisor (5V → 2.5V): offset = 1.25V on ESP32 ADC
 *   - Full scale range: 0-5A linear mapped to 0-2.5V (before divisor) = 0-1.25V (after divisor)
 *   - Sensitivity after divisor: 185mV/A ÷ 2 = 92.5 mV/A on ESP32 ADC
 *   - ESP32-S3 ADC: 12-bit (0-4095 = 0-3.3V)
 *   - ADC resolution: 3.3V / 4095 = 0.8056 mV per step
 *   - Safe input range: 0.625V to 1.875V (with margin from 3.3V max)
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
 * MEASUREMENT PROCESS FOR ACS712T (DC sensor):
 *   1. Sample ADC multiple times for averaging
 *   2. Calculate average voltage
 *   3. Subtract DC offset (1.25V nominal)
 *   4. Convert voltage difference to current using sensitivity
 *   5. Filter and apply noise floor
 *   
 * Measurement process for ZMPT101B (AC sensor):
 *   1. Sample 30 times (~1-2ms, appropriate for 50/60Hz AC)
 *   2. Remove DC offset from each sample
 *   3. Calculate RMS: sqrt(sum of squares / number of samples)
 *   4. Convert RMS to actual measurement (voltage in V)
 */

#define ADC_SAMPLES 200       // Number of samples per RMS calculation
#define RMS_ITERATIONS 5      // Number of RMS calculations to average (better noise rejection)

// ACS712T-5A current sensor calibration with voltage divisor
// Real installation specs:
//   - External power supply: 5.4V (measured)
//   - ACS712T sensitivity: 185 mV/A at VCC
//   - DC Offset at zero current: VCC/2 = 5.4V/2 = 2.7V
// With voltage divisor 1:2 (resistive divider to match ESP32 ADC 3.3V max):
//   - Theoretical offset after divisor: 2.7V / 2 = 1.35V
//   - ACTUAL measured offset: 1.55V (calibrated from real circuit)
//   - Difference (0.20V) caused by resistor tolerances (±5% typical)
//   - Sensitivity after divisor: 185mV/A / 2 = 92.5 mV/A
// 
// CALIBRATION NOTES:
//   - Always use measured offset value (1.55V) for accurate zero-current reference
//   - With 5.4V supply and divisor, safe ADC range: ~0.8V to 2.3V (well within 3.3V max)
//   - Maximum measurable current: 5A → voltage swing: ±462.5mV from offset
const float ACS712_DC_OFFSET_V = 1.55f;        // DC offset voltage with divisor (CALIBRATED from real measurement)
const float ACS712_SENSITIVITY_mVpA = 92.5f;   // Sensitivity in mV/A (185mV/A ÷ 2 from divisor)
const float ACS712_SENSITIVITY_VpA = ACS712_SENSITIVITY_mVpA / 1000.0f;  // Convert to V/A
const float ACS712_MAX_CURRENT_A = 5.0f;       // Maximum measurable current (5A)

#define VOLTAGE_RATIO 531     // ZMPT101B conversion: 531 V/V (120V RMS / 0.226V RMS)
#define CURRENT_FLOOR_A 0.0   // Readings below this are clamped to 0A (noise suppression)

// Enable/disable multi-sensor reading: true = CURR1 only, false = all 4 sensors
#define USE_ONLY_SENSOR1 false

// CURRENT MEASUREMENT MODE: Choose between AC and DC measurement
// Set to true for AC measurement (RMS), false for DC measurement (average)
#define CURRENT_MEASUREMENT_AC true

// TEST/SIMULATION MODE: Set to true to simulate sensor values for communication testing
// Set to false to use real sensor readings from ACS712T + ZMPT101B
#define SIMULATE_MODE false

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
// MEASUREMENT THRESHOLDS (Optional - for coordinator-side filtering)
// ============================================================================
/*
 * These thresholds are available for future firmware-level filtering/validation.
 * Currently they are stored but NOT used in this implementation.
 * The coordinator receives all measurements and handles filtering logic.
 *
 * CURRENT_THRESHOLD_MIN / CURRENT_THRESHOLD_MAX (in Amperes):
 *   - Define acceptable current range for each lamp
 *   - For UV lamps: typical range 0.5A to 1.2A per lamp
 *   - Can be used on ESP32 or coordinator depending on requirements
 *
 * AC_VOLTAGE_THRESHOLD (in Volts RMS):
 *   - Minimum acceptable mains voltage for safe operation
 *   - Below this value indicates potential power supply issue
 *   - Typical range: 95-110V RMS
 *
 * DEBUG / CALIBRATION:
 * 1. Open Serial Monitor (Tools → Serial Monitor, 115200 baud)
 * 2. Monitor "Current: CURR1=X.XXA ... | AC Voltage: XXX.XV RMS" output
 * 3. Verify measurements are in expected range with known loads
 * 4. Use these thresholds if implementing local validation
 */

float CURRENT_THRESHOLD_MIN = 0.5;   // Minimum acceptable current (Amperes)
float CURRENT_THRESHOLD_MAX = 1.2;   // Maximum acceptable current (Amperes)
float AC_VOLTAGE_THRESHOLD = 100.0;  // Minimum acceptable voltage (Volts RMS)

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
  // NOTE: while (!Serial); is commented out because:
  //   - It blocks startup if no USB/Serial Monitor is connected
  //   - Our system must run autonomously without PC connection
  //   - We use delay(500) instead to give Serial time to stabilize if available
  delay(500);

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
// HELPER FUNCTION - Single RMS Calculation (internal)
// ============================================================================
/*
 * Helper function that performs ONE RMS calculation on a set of samples.
 * Returns the RMS voltage value in millivolts.
 */
float calculateSingleRMS(uint8_t pin, uint16_t samples, int &min_mV, int &max_mV) {
  float sumSquares = 0.0f;
  float sum_mV = 0.0f;
  min_mV = 4095;
  max_mV = 0;

  // Take samples and accumulate squared values
  for (uint16_t i = 0; i < samples; i++) {
    int raw_mV = analogReadMilliVolts(pin);
    sum_mV += raw_mV;
    sumSquares += (float)raw_mV * (float)raw_mV;

    if (raw_mV < min_mV) min_mV = raw_mV;
    if (raw_mV > max_mV) max_mV = raw_mV;

    delayMicroseconds(50);  // Small delay between samples
  }

  // Calculate RMS using mean-centering: rms = sqrt(E[x^2] - (E[x])^2)
  float mean_mV = sum_mV / (float)samples;
  float meanSquares = (sumSquares / (float)samples) - (mean_mV * mean_mV);
  if (meanSquares < 0.0f) {
    meanSquares = 0.0f;
  }
  return sqrtf(meanSquares);
}

// ============================================================================
// HELPER FUNCTION - DC Current Measurement (ACS712T)
// ============================================================================
/*
 * Samples an ADC pin multiple times and calculates DC current.
 * Uses simple averaging of multiple samples for DC measurement.
 * ACS712T can also measure DC - just subtract offset and convert.
 * 
 * PARAMETERS:
 *   pin: ADC pin to sample
 *   samples: Number of samples to average (default 200)
 *   
 * RETURNS:
 *   Current in Amperes (DC)
 *   
 * TIMING:
 *   ~200 samples = ~20-30ms total
 */
float readDC_and_convertToCurrent(uint8_t pin, uint16_t samples = ADC_SAMPLES) {
  float sum_mV = 0.0f;
  int min_mV = 4095;
  int max_mV = 0;

  // Take samples and accumulate values
  for (uint16_t i = 0; i < samples; i++) {
    int raw_mV = analogReadMilliVolts(pin);
    sum_mV += raw_mV;

    if (raw_mV < min_mV) min_mV = raw_mV;
    if (raw_mV > max_mV) max_mV = raw_mV;

    delayMicroseconds(50);  // Small delay between samples
  }

  // Calculate average DC voltage
  float mean_mV = sum_mV / (float)samples;
  float voltage_diff_mV = mean_mV - (ACS712_DC_OFFSET_V * 1000.0f);  // Subtract offset

  // Store debug info for CURR1
  if (pin == CURRENT_SENSOR1) {
    debug_raw_adc_avg = mean_mV;
    debug_rms_curr1 = voltage_diff_mV;  // Voltage difference from offset
    debug_min_mV = (float)min_mV;
    debug_max_mV = (float)max_mV;
  }

  // Convert voltage difference to current using ACS712T sensitivity
  // current = voltage_diff / sensitivity
  float current_A = (voltage_diff_mV / 1000.0f) / ACS712_SENSITIVITY_VpA;

  // Clamp to valid range (ACS712T rated 0-5A)
  if (current_A < 0.0f) current_A = 0.0f;
  if (current_A > ACS712_MAX_CURRENT_A) current_A = ACS712_MAX_CURRENT_A;

  return current_A;
}

// ============================================================================
// HELPER FUNCTION - AC Current Measurement using Multiple RMS (ACS712T)
// ============================================================================
/*
 * Samples an ADC pin multiple times and calculates RMS AC current.
 * Uses MULTIPLE RMS calculations and averages them for better noise rejection.
 * ACS712T outputs a sinusoidal signal centered on DC offset (1.25V with divisor).
 * 
 * PARAMETERS:
 *   pin: ADC pin to sample
 *   samples: Number of samples per RMS calculation (default 200)
 *   iterations: Number of RMS calculations to average (default 5)
 *   
 * RETURNS:
 *   Current in Amperes (RMS - effective AC current)
 *   
 * TIMING:
 *   ~200 samples × 5 iterations = ~100-150ms total (excellent noise reduction)
 */
float readRMS_and_convertToCurrent(uint8_t pin, uint16_t samples = ADC_SAMPLES, uint8_t iterations = RMS_ITERATIONS) {
  float rms_current_sum = 0.0f;
  int min_mV_all = 4095;
  int max_mV_all = 0;

  // Take multiple RMS readings and average them
  for (uint8_t iter = 0; iter < iterations; iter++) {
    int min_mV, max_mV;
    float rms_voltage_mV = calculateSingleRMS(pin, samples, min_mV, max_mV);
    
    // Track overall min/max
    if (min_mV < min_mV_all) min_mV_all = min_mV;
    if (max_mV > max_mV_all) max_mV_all = max_mV;

    // Convert this RMS voltage to current
    float current_A = (rms_voltage_mV / 1000.0f) / ACS712_SENSITIVITY_VpA;
    if (current_A < 0.0f) current_A = 0.0f;
    if (current_A > ACS712_MAX_CURRENT_A) current_A = ACS712_MAX_CURRENT_A;
    
    rms_current_sum += current_A;
  }

  // Average all RMS current readings
  float current_A_avg = rms_current_sum / (float)iterations;

  // Store debug info for CURR1 (using last RMS calculation values for display)
  if (pin == CURRENT_SENSOR1) {
    debug_raw_adc_avg = 1250.0f;  // Expected DC offset
    debug_rms_curr1 = current_A_avg * ACS712_SENSITIVITY_mVpA;  // Show equivalent RMS voltage
    debug_min_mV = (float)min_mV_all;
    debug_max_mV = (float)max_mV_all;
  }

  return current_A_avg;
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
// SCALING FUNCTIONS - Convert measurements to 8-bit packed format
// ============================================================================
/*
 * Scales high-precision float measurements to 8-bit (0-255) for packet transmission.
 * This allows full measurements to fit in a 10-byte response packet.
 *
 * Resolution Trade-off:
 *   - Voltage: 0-255 represents 0-130V RMS = 0.51 V/step resolution
 *   - Current: 0-255 represents 0-2550 mA = 10 mA/step resolution
 *
 * The coordinator receives the scaled value and can reconstruct original range.
 * Example: scaled_current = 128 (half scale) = 1275 mA ≈ 1.28A
 */

uint8_t scale_voltage(float voltage_V) {
  // Map 0-130V RMS to 0-255 (resolution: 0.51 V/step)
  if (voltage_V < 0) return 0;
  if (voltage_V > 130.0) return 255;
  return (uint8_t)((voltage_V / 130.0) * 255.0);
}

uint8_t scale_current(float current_mA) {
  // Map 0-2550 mA to 0-255 (resolution: 10 mA/step)
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
 *    a. Scale current measurements to 8-bit format
 *    b. Build and send response packet back
 * 4. Small delay, then repeat
 *
 * NOTE: USE_ONLY_SENSOR1 Configuration
 *   - When USE_ONLY_SENSOR1 = true: Only CURRENT_SENSOR1 is active
 *   - CURRENT_SENSOR2/3/4 are set to 0.0A and unused
 *   - This saves processing time and ADC reads
 *   - Debug output shows detailed CURR1 data
 *   - Filters for sensors 2-4 are still allocated but inactive
 *   - To use all 4 sensors: Set USE_ONLY_SENSOR1 = false (line 287)
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

  // ========================================================================
  // CURRENT MEASUREMENT MODE SELECTION - Ternary Operator Example
  // ========================================================================
  // OPERATOR TERNARIO: condición ? valor_si_verdadero : valor_si_falso
  // 
  // Este es un "if/else" comprimido en una sola línea usando el operador ?:
  // 
  // Estructura:
  //   (CURRENT_MEASUREMENT_AC) ? readRMS_and_convertToCurrent(...) : readDC_and_convertToCurrent(...)
  //                 ↑                          ↑                              ↑
  //            CONDICIÓN                 SI VERDADERO                     SI FALSO
  //
  // En este caso:
  //   - Si CURRENT_MEASUREMENT_AC = true  → Usa modo AC (RMS multiple)
  //   - Si CURRENT_MEASUREMENT_AC = false → Usa modo DC (promedio simple)
  //
  // Es equivalente a:
  //   if (CURRENT_MEASUREMENT_AC) {
  //     current_sensor1_A = readRMS_and_convertToCurrent(CURRENT_SENSOR1);
  //   } else {
  //     current_sensor1_A = readDC_and_convertToCurrent(CURRENT_SENSOR1);
  //   }
  //
  // Pero en una sola línea, más compacto. Muy común en C/C++.
  // ========================================================================

  current_sensor1_A = (CURRENT_MEASUREMENT_AC) ? readRMS_and_convertToCurrent(CURRENT_SENSOR1) : readDC_and_convertToCurrent(CURRENT_SENSOR1);
  if (USE_ONLY_SENSOR1) {
    current_sensor2_A = 0.0;
    current_sensor3_A = 0.0;
    current_sensor4_A = 0.0;
  } else {
    current_sensor2_A = (CURRENT_MEASUREMENT_AC) ? readRMS_and_convertToCurrent(CURRENT_SENSOR2) : readDC_and_convertToCurrent(CURRENT_SENSOR2);
    current_sensor3_A = (CURRENT_MEASUREMENT_AC) ? readRMS_and_convertToCurrent(CURRENT_SENSOR3) : readDC_and_convertToCurrent(CURRENT_SENSOR3);
    current_sensor4_A = (CURRENT_MEASUREMENT_AC) ? readRMS_and_convertToCurrent(CURRENT_SENSOR4) : readDC_and_convertToCurrent(CURRENT_SENSOR4);
  }
  AC_voltage_V = readRMS_and_convertToVoltage(AC_POWER_PIN);
  
  // SIMULATION MODE: Generate random values every 5 seconds
  if (SIMULATE_MODE) {
    unsigned long now = millis();
    if (now - lastSimUpdate >= 5000) {
      // Generate new random values (all 4 sensors for completeness)
      sim_current1_A = random(0, 1201) / 1000.0f;      // 0-1200 mA → 0-1.200 A
      if (!USE_ONLY_SENSOR1) {
        sim_current2_A = random(0, 1201) / 1000.0f;
        sim_current3_A = random(0, 1201) / 1000.0f;
        sim_current4_A = random(0, 1201) / 1000.0f;
      }
      sim_voltage_V = 100.0f + (random(0, 501) / 10.0f);  // 100-150 V RMS
      lastSimUpdate = now;
      if (!USE_ONLY_SENSOR1) {
        Serial.printf("[SIM] New random values: I1=%.0fmA I2=%.0fmA I3=%.0fmA I4=%.0fmA V=%.1fV\n",
          sim_current1_A * 1000, sim_current2_A * 1000, sim_current3_A * 1000, sim_current4_A * 1000, sim_voltage_V);
      } else {
        Serial.printf("[SIM] New random value: I1=%.0fmA V=%.1fV\n",
          sim_current1_A * 1000, sim_voltage_V);
      }
    }
    current_sensor1_A = sim_current1_A;
    if (!USE_ONLY_SENSOR1) {
      current_sensor2_A = sim_current2_A;
      current_sensor3_A = sim_current3_A;
      current_sensor4_A = sim_current4_A;
    }
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
    
    // Detailed debug output (CURR1 only - others available if USE_ONLY_SENSOR1 is disabled)
    float vpp_mV = debug_max_mV - debug_min_mV;
    const char* mode_str = (CURRENT_MEASUREMENT_AC) ? "AC(RMS×5)" : "DC(Avg)";
    Serial.printf("[DEBUG CURR1] Offset=%.1fmV Diff=%.2fmV Vpp=%.1fmV min=%.0fmV max=%.0fmV -> %.4fA [%s]\n",
      debug_raw_adc_avg, debug_rms_curr1, vpp_mV, debug_min_mV, debug_max_mV, current_sensor1_A, mode_str);
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
