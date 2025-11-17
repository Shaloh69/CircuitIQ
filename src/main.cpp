/*
 * ESP32 Dual-Channel Electrical Safety Monitor - CircuitIQ - VERSION v6.0 SERVER-CONNECTED
 * 
 * NEW IN v6.0: SERVER CONNECTIVITY
 * ✅ WiFi connectivity with auto-reconnect
 * ✅ HTTP POST to server for real-time monitoring
 * ✅ WebSocket support for receiving commands
 * ✅ Device identification as CIRQUITIQ
 * ✅ Proper data formatting for server
 * ✅ Command processing from server
 * ✅ Network status indicators
 * ✅ ALL ORIGINAL FUNCTIONALITY PRESERVED
 */

/*
 * ESP32 Dual-Channel Electrical Safety Monitor - VERSION v6.0 ENHANCED
 * 
 * MODIFIED FOR ACTIVE BUZZER (3-24V DC)
 * - Changed tone() calls to digitalWrite() for active buzzer compatibility
 * - Line 1076-1085: Buzzer now uses simple ON/OFF instead of frequency generation
 * 
 * NEW LCD ENHANCEMENTS (v5.5):
 * ✅ ERROR DISPLAYS: Shows critical errors on LCD
 *    • NO VOLTAGE error (voltage < 50V)
 *    • SENSOR FAULT error (ACS712 issues)
 * ✅ WARNING DISPLAYS: Shows warnings on LCD
 *    • OVERVOLTAGE warning (>250V)
 *    • BROWNOUT warning (<180V)
 *    • UNDERVOLTAGE warning (<200V)
 *    • OVERCURRENT warning (>4.5A)
 * ✅ UPTIME TIMER: Shows system runtime in HH:MM:SS format
 * ✅ RELAY STATUS: Shows R1/R2 ON/OFF status on LCD
 * ✅ PRIORITY DISPLAY: Errors shown first, then warnings, then normal data
 * 
 * ALL FIXES APPLIED:
 * ✅ Voltage divider configured (10kΩ + 20kΩ)
 * ✅ ST7789 240x240 display with Adafruit library (better GMT130 support)
 * ✅ I2C LCD display (20x4 or 16x2) - shows voltage, current, kWh
 * ✅ Voltage buffer pre-fill (no false brownout)
 * ✅ Current deadband (no phantom loads)
 * ✅ Startup settling period
 * ✅ Philippine Peso cost tracking
 * ✅ AUTO-ENABLE: Both SSRs turn ON after successful initialization
 * ✅ All safety features working
 * 
 * SSR BEHAVIOR:
 * - SSRs automatically ENABLE after successful calibration
 * - SSRs only turn OFF during safety events:
 *   • Overcurrent (>4.5A for 1 second)
 *   • Overvoltage (>250V for 0.5 seconds)
 *   • Brownout (<180V for 2 seconds)
 *   • Sensor faults
 *   • Electric shock detection
 * - Manual control: Use 'on 1/2' or 'off 1/2' commands
 * 
 * WIRING:
 * - Display: GND→GND, VCC→3.3V, SCK→18, SDA→23, RES→25, DC→26, BLK→33
 * - ACS712 CH1: OUT→10kΩ→GPIO34→20kΩ→GND
 * - ACS712 CH2: OUT→10kΩ→GPIO32→20kΩ→GND
 * - ZMPT101B: OUT→GPIO35
 * - Relay CH1: GPIO16
 * - Relay CH2: GPIO17
 * - I2C LCD: SDA→21, SCL→22, VCC→5V, GND→GND
 * - LED Indicators:
 *   • RED LED (GPIO2): Short circuit / Sensor faults
 *   • YELLOW LED (GPIO27): Overload (overcurrent >4.5A)
 *   • GREEN LED (GPIO4): Normal operation
 *   • BLUE LED (GPIO5): Under/Overvoltage conditions
 */

#include <EEPROM.h>
#include <esp_task_wdt.h>
#include <FS.h>
#include <SD.h>
#include <SPI.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>

WiFiClientSecure secureClient;

#define WIFI_SSID           "YOUR_ACTUAL_WIFI_NAME"
#define WIFI_PASSWORD       "YOUR_ACTUAL_WIFI_PASSWORD"
#define SERVER_HOST         "meifhi-esp-server.onrender.com"
#define SERVER_PORT         443
#define DEVICE_ID           "CIRQUITIQ_001"  // Make unique per device
#define DEVICE_TYPE         "CIRQUITIQ"             // ✅ Added!

// Network settings
#define WIFI_CONNECT_TIMEOUT    20000
#define WIFI_RECONNECT_INTERVAL 30000
#define SERVER_POST_INTERVAL    5000    // Send data every 5 seconds
#define SERVER_TIMEOUT          10000
#define WEBSOCKET_RECONNECT_INT 10000
#define MAX_RETRY_ATTEMPTS      3       // ✅ Added!

// ==================== PIN DEFINITIONS ====================
#define ACS712_CH1_PIN      34
#define ACS712_CH2_PIN      32
#define ZMPT101B_PIN        35
#define RELAY_CH1_PIN       16
#define RELAY_CH2_PIN       17
#define RED_LED_PIN         2       // Short circuit / Sensor faults
#define YELLOW_LED_PIN      27      // Overload (overcurrent)
#define GREEN_LED_PIN       4       // Normal operation
#define BLUE_LED_PIN        5       // Under/Overvoltage
#define BUZZER_PIN          19
#define SD_CS_PIN           15
#define TFT_BL_PIN          33

// Display pins (for Adafruit library)
#define TFT_DC              26
#define TFT_RST             25
#define TFT_MOSI            23
#define TFT_SCLK            18

// I2C LCD pins
#define I2C_SDA_PIN         21
#define I2C_SCL_PIN         22
#define LCD_I2C_ADDR        0x27  // Common address, can be 0x3F

// ==================== CHANNEL CONFIGURATION ====================
#define NUM_CHANNELS        2
#define CHANNEL_1           0
#define CHANNEL_2           1

// ==================== ADC CONFIGURATION ====================
#define ADC_RESOLUTION      4096.0
#define ADC_VREF            3.6
#define ADC_SAMPLES_FAST    2
#define ADC_SAMPLES_SLOW    12

// ==================== ACS712-05B CONFIGURATION ====================
#define ACS712_SENSITIVITY  0.185
#define ACS712_ZERO_CURRENT 2.5
#define ACS712_SUPPLY_VOLTAGE 5.0
#define VOLTAGE_DIVIDER_RATIO 0.667
#define VOLTAGE_DIVIDER_SCALE 1.5
#define EXPECTED_OFFSET_MIN 1.35
#define EXPECTED_OFFSET_MAX 1.95
#define MAX_VALID_CURRENT   6.0
#define CURRENT_DEADBAND    0.08  // Ignore currents below this

// ==================== ZMPT101B CONFIGURATION ====================
#define ZMPT101B_SENSITIVITY 0.0125
#define AC_FREQUENCY        50
#define AC_PERIOD_MS        20
#define RMS_SAMPLES         100
#define RMS_SAMPLE_DELAY_US 150
#define MIN_AC_SWING        0.1
#define MIN_VALID_VOLTAGE   50.0
#define VOLTAGE_AUTO_CAL_SAMPLES 10

// ==================== RELAY CONTROL ====================
#define RELAY_ON_STATE      LOW
#define RELAY_OFF_STATE     HIGH
#define RELAY_VERIFY_INTERVAL 5000

// ==================== SAFETY THRESHOLDS ====================
#define MAX_CURRENT         4.5
#define MAX_VOLTAGE         250.0
#define MIN_VOLTAGE         200.0
#define SENSOR_FAULT_VOLTAGE 350.0
#define MAX_SENSOR_CURRENT  20.0
#define OVERCURRENT_TIME    1000
#define OVERVOLTAGE_TIME    500
#define SAFETY_DEBOUNCE_COUNT 5
#define BROWNOUT_VOLTAGE    180.0
#define BROWNOUT_TIME       2000
#define STARTUP_SETTLE_TIME 15000  // 15 seconds

// ==================== SYSTEM CONFIGURATION ====================
#define SAMPLE_RATE         1000
#define DISPLAY_INTERVAL    2000
#define CALIBRATION_SAMPLES 500
#define MOVING_AVERAGE_SIZE 10
#define WATCHDOG_TIMEOUT_S  30
#define LOG_INTERVAL_MS     5000
#define MAX_CMD_LENGTH      64
#define SERIAL_RX_BUFFER    256
#define EEPROM_SIZE         512
#define EEPROM_MAGIC        0xDEADC0DE

// ==================== DISPLAY CONFIGURATION ====================
#define TFT_UPDATE_INTERVAL 1000
#define TFT_WIDTH           240
#define TFT_HEIGHT          240
#define DISPLAY_ENABLED     true
#define TFT_BRIGHTNESS      255
#define LCD_COLS            20  // Change to 16 for 16x2 LCD
#define LCD_ROWS            4   // Change to 2 for 16x2 LCD
#define LCD_UPDATE_INTERVAL 1000  // Update I2C LCD every 1 second

// ==================== COST CALCULATION ====================
#define ELECTRICITY_RATE_PER_KWH  11.50

// ==================== POWER FACTOR ====================
#define DEFAULT_POWER_FACTOR 0.95

// ==================== PERFORMANCE TRACKING ====================
#define PERF_SAMPLES        100

// ==================== TFT COLORS (Adafruit ST77XX) ====================
#define COLOR_BG            ST77XX_BLACK
#define COLOR_TEXT          ST77XX_WHITE
#define COLOR_SUCCESS       ST77XX_GREEN
#define COLOR_WARNING       ST77XX_YELLOW
#define COLOR_ERROR         ST77XX_RED
#define COLOR_INFO          ST77XX_CYAN
#define COLOR_DISABLED      0x7BEF
#define COLOR_HEADER        0x051D

// ==================== SYSTEM STATES ====================
enum SystemState {
  STATE_INITIALIZING,
  STATE_CALIBRATING,
  STATE_MONITORING,
  STATE_MANUAL_CONTROL,
  STATE_ERROR,
  STATE_SHUTDOWN
};

enum CalibrationState {
  CAL_IDLE,
  CAL_SAMPLING,
  CAL_VERIFICATION,
  CAL_COMPLETE
};

// ==================== CHANNEL DATA ====================
struct ChannelData {
  int currentSensorPin;
  int relayPin;
  
  float currentReading;
  float powerReading;
  float apparentPower;
  float energyConsumed;
  float costAccumulated;
  
  float currentOffset;
  float currentCalibration;
  
  bool relayEnabled;
  bool relayCommandState;
  
  bool overcurrentDetected;
  bool sensorFaultDetected;
  int overcurrentDebounceCount;
  int sensorFaultDebounceCount;
  unsigned long overcurrentStartTime;
  
  float currentBuffer[MOVING_AVERAGE_SIZE];
  int bufferIndex;
  
  float maxCurrent;
  float maxPower;
  unsigned long shutdownCount;
  
  const char* name;
};

// ==================== GLOBAL VARIABLES ====================
SystemState currentState = STATE_INITIALIZING;
CalibrationState calState = CAL_IDLE;
bool manualControl = false;
bool safetyEnabled = true;
bool buzzerEnabled = true;
bool buzzerInverted = true;  // Set to true if NPN transistor causes buzzer to beep when GPIO is LOW
bool sdCardAvailable = false;
bool wifiConnected = false;
bool serverConnected = false;
bool sensorsValid = false;
bool criticalError = false;
bool displayEnabled = DISPLAY_ENABLED;
bool tftInitialized = false;
bool lcdInitialized = false;

// ✅ ADD THESE NEW VARIABLES:
int serverErrorCount = 0;
int serverSuccessCount = 0;

// Create Adafruit display object (software SPI - no CS pin for GMT130)
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_DC, TFT_RST, TFT_MOSI, TFT_SCLK);
// Create I2C LCD object (20x4 or 16x2)
LiquidCrystal_I2C lcd(LCD_I2C_ADDR, LCD_COLS, LCD_ROWS);

// Network clients
HTTPClient http;
WebSocketsClient webSocket;

ChannelData channels[NUM_CHANNELS];

float voltageReading = 0.0;
float voltageOffset = 0.0;
float voltageCalibration = 0.934;  // *** FIXED: Changed from 1.0 to 0.934 (239V / 256V)
float voltageBuffer[MOVING_AVERAGE_SIZE];
int voltageBufferIndex = 0;

bool overvoltageDetected = false;
bool undervoltageDetected = false;
bool brownoutDetected = false;
int overvoltageDebounceCount = 0;
unsigned long overvoltageStartTime = 0;
unsigned long brownoutStartTime = 0;

float maxVoltage = 0.0;
float minVoltage = 999.0;

// ==================== CALIBRATION DATA ====================
struct CalibrationData {
  uint32_t magic;
  float ch1CurrentOffset;
  float ch2CurrentOffset;
  float voltageOffset;
  float ch1CurrentCalibration;
  float ch2CurrentCalibration;
  float voltageCalibration;
  float powerFactor;
  float electricityRate;
  uint32_t crc;
};

CalibrationData calData = {
  .magic = EEPROM_MAGIC,
  .ch1CurrentOffset = ACS712_ZERO_CURRENT * VOLTAGE_DIVIDER_RATIO,
  .ch2CurrentOffset = ACS712_ZERO_CURRENT * VOLTAGE_DIVIDER_RATIO,
  .voltageOffset = 1.65,
  .ch1CurrentCalibration = 1.0,
  .ch2CurrentCalibration = 1.0,
  .voltageCalibration = 0.934,  // *** FIXED: Changed from 1.0 to 0.934
  .powerFactor = DEFAULT_POWER_FACTOR,
  .electricityRate = ELECTRICITY_RATE_PER_KWH,
  .crc = 0
};

// ==================== TIMING ====================
unsigned long lastDisplayUpdate = 0;
unsigned long lastLogUpdate = 0;
unsigned long lastRelayVerify = 0;
unsigned long systemStartTime = 0;
unsigned long lastWatchdogFeed = 0;
unsigned long lastSampleTime = 0;
unsigned long lastLoopTime = 0;
unsigned long loopCount = 0;
unsigned long lastTFTUpdate = 0;
unsigned long lastLCDUpdate = 0;
unsigned long lastServerPost = 0;
unsigned long lastWiFiCheck = 0;
unsigned long lastWebSocketReconnect = 0;

// ==================== PERFORMANCE ====================
struct PerformanceMetrics {
  unsigned long avgLoopTime;
  unsigned long maxLoopTime;
  unsigned long minLoopTime;
  uint32_t loopTimeSum;
  uint16_t sampleCount;
};
PerformanceMetrics perfMetrics = {0, 0, 999999, 0, 0};

// ==================== CALIBRATION PROGRESS ====================
struct CalibrationProgress {
  int samplesCollected;
  float ch1CurrentSum;
  float ch2CurrentSum;
  float voltageSum;
  float voltageMin;
  float voltageMax;
  unsigned long startTime;
  int currentChannel;
};
CalibrationProgress calProgress = {0, 0.0, 0.0, 0.0, 999.0, 0.0, 0, 0};

// ==================== FUNCTION PROTOTYPES ====================
void initializeChannels();
void startupSequence();
void autoCalibrateSensorsNonBlocking();
void calibrateVoltageWithReference();
void autoDetectVoltageOffset();
void readSensors();
float calculateRMSVoltage(int sensorPin);
float calculateRMSCurrent(int channel);
void updateMovingAverages();
void preFillVoltageBuffer(float voltage);
float getAverageCurrent(int channel);
float getAverageVoltage();
void performSafetyChecks();
void triggerSafetyShutdown(int channel, const char* reason);
void updateEnergyCalculation();
void updateCostCalculation();
void updateStatistics();
void updateStatusLEDs();
void setBuzzer(bool state);
void updateDisplay();
void initTFTDisplay();
void updateTFTDisplay();
void initLCDDisplay();
void updateLCDDisplay();
void drawTFTHeader();
void drawChannelStatus(int channel, int yPos);
void drawTotalStats(int yPos);
void setTFTBrightness(uint8_t brightness);
const char* getStateString();
void handleSerialCommands();
void processCommand(const char* command);
void printMenu();
void printStatistics();
void printDiagnostics();
void printMemoryUsage();
void saveCalibrationData();
bool loadCalibrationData();
void testBuzzer();
bool validateSensorReadings(int channel);
void initSDCard();
void logToSD(const char* data);
void setupWatchdog();
void feedWatchdog();
void emergencyReset();
float readRawSensorVoltage(int pin, int samples);
uint32_t calculateCRC32(const uint8_t* data, size_t length);
void verifyRelayStates();
void verifyCalibration();
void updatePerformanceMetrics(unsigned long loopTime);
bool isSafeToOperate(int channel);
void handleBrownout();
void enableChannel(int channel);
void disableChannel(int channel);
void enableAllChannels();
void disableAllChannels();
float calculateCostPerHour(float watts);
float calculateTotalCost();

// Network functions
void initWiFi();
void checkWiFiConnection();
void sendDataToServer();
void webSocketEvent(WStype_t type, uint8_t * payload, size_t length);
void processServerCommand(const char* command);
void reconnectWebSocket();
void testRawDisplayCommands();
void testAlternativeInit1();
void testAlternativeInit2();

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  Serial.setRxBufferSize(SERIAL_RX_BUFFER);
  delay(2000);
  
  Serial.println(F("\n========================================"));
  Serial.println(F("ESP32 CircuitIQ Safety Monitor"));
  Serial.println(F("VERSION v6.0 WITH I2C LCD - GMT130 FIX"));
  Serial.println(F("========================================"));
  
  EEPROM.begin(EEPROM_SIZE);
  
  initializeChannels();
  
  pinMode(channels[CHANNEL_1].relayPin, OUTPUT);
  pinMode(channels[CHANNEL_2].relayPin, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(YELLOW_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(BLUE_LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  digitalWrite(channels[CHANNEL_1].relayPin, RELAY_OFF_STATE);
  digitalWrite(channels[CHANNEL_2].relayPin, RELAY_OFF_STATE);
  digitalWrite(RED_LED_PIN, LOW);
  digitalWrite(YELLOW_LED_PIN, LOW);
  digitalWrite(GREEN_LED_PIN, LOW);
  digitalWrite(BLUE_LED_PIN, LOW);
  setBuzzer(false);  // Initialize buzzer OFF
  
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  
  for (int i = 0; i < MOVING_AVERAGE_SIZE; i++) {
    voltageBuffer[i] = 0.0;
    for (int ch = 0; ch < NUM_CHANNELS; ch++) {
      channels[ch].currentBuffer[i] = 0.0;
    }
  }
  
  setupWatchdog();
  initSDCard();
  
  if (displayEnabled) {
    initTFTDisplay();
  }
  
  // Initialize I2C LCD
  Serial.println(F("Initializing I2C LCD..."));
  initLCDDisplay();
  
  if (!loadCalibrationData()) {
    Serial.println(F("⚠️  Using default calibration values"));
  }
  
  // Initialize WiFi
  initWiFi();
  
  systemStartTime = millis();
  currentState = STATE_CALIBRATING;
  calState = CAL_IDLE;
  
  startupSequence();
  printMenu();
  printMemoryUsage();
  
  Serial.println(F("\n*** SYSTEM READY - CIRQUITIQ MODE ***"));
  Serial.println(F("Type 'help' for commands\n"));
}

// ==================== INITIALIZE CHANNELS ====================
void initializeChannels() {
  channels[CHANNEL_1].currentSensorPin = ACS712_CH1_PIN;
  channels[CHANNEL_1].relayPin = RELAY_CH1_PIN;
  channels[CHANNEL_1].name = "CH1";
  channels[CHANNEL_1].currentReading = 0.0;
  channels[CHANNEL_1].powerReading = 0.0;
  channels[CHANNEL_1].apparentPower = 0.0;
  channels[CHANNEL_1].energyConsumed = 0.0;
  channels[CHANNEL_1].costAccumulated = 0.0;
  channels[CHANNEL_1].currentOffset = ACS712_ZERO_CURRENT * VOLTAGE_DIVIDER_RATIO;
  channels[CHANNEL_1].currentCalibration = 1.0;
  channels[CHANNEL_1].relayEnabled = false;
  channels[CHANNEL_1].relayCommandState = false;
  channels[CHANNEL_1].overcurrentDetected = false;
  channels[CHANNEL_1].sensorFaultDetected = false;
  channels[CHANNEL_1].overcurrentDebounceCount = 0;
  channels[CHANNEL_1].sensorFaultDebounceCount = 0;
  channels[CHANNEL_1].overcurrentStartTime = 0;
  channels[CHANNEL_1].bufferIndex = 0;
  channels[CHANNEL_1].maxCurrent = 0.0;
  channels[CHANNEL_1].maxPower = 0.0;
  channels[CHANNEL_1].shutdownCount = 0;
  
  channels[CHANNEL_2].currentSensorPin = ACS712_CH2_PIN;
  channels[CHANNEL_2].relayPin = RELAY_CH2_PIN;
  channels[CHANNEL_2].name = "CH2";
  channels[CHANNEL_2].currentReading = 0.0;
  channels[CHANNEL_2].powerReading = 0.0;
  channels[CHANNEL_2].apparentPower = 0.0;
  channels[CHANNEL_2].energyConsumed = 0.0;
  channels[CHANNEL_2].costAccumulated = 0.0;
  channels[CHANNEL_2].currentOffset = ACS712_ZERO_CURRENT * VOLTAGE_DIVIDER_RATIO;
  channels[CHANNEL_2].currentCalibration = 1.0;
  channels[CHANNEL_2].relayEnabled = false;
  channels[CHANNEL_2].relayCommandState = false;
  channels[CHANNEL_2].overcurrentDetected = false;
  channels[CHANNEL_2].sensorFaultDetected = false;
  channels[CHANNEL_2].overcurrentDebounceCount = 0;
  channels[CHANNEL_2].sensorFaultDebounceCount = 0;
  channels[CHANNEL_2].overcurrentStartTime = 0;
  channels[CHANNEL_2].bufferIndex = 0;
  channels[CHANNEL_2].maxCurrent = 0.0;
  channels[CHANNEL_2].maxPower = 0.0;
  channels[CHANNEL_2].shutdownCount = 0;
}

// ==================== INITIALIZE TFT DISPLAY (ADAFRUIT) ====================
void initTFTDisplay() {
  Serial.print(F("Initializing GMT130 (Adafruit ST7789)... "));
  
  // Backlight ON
  pinMode(TFT_BL_PIN, OUTPUT);
  digitalWrite(TFT_BL_PIN, HIGH);
  
  // Initialize display - CRITICAL for GMT130: use init() with 240x240 and SPI_MODE0
  tft.init(240, 240, SPI_MODE0);
  
  Serial.println("Testing colors...");
  
  // Color test sequence
  tft.fillScreen(ST77XX_RED);
  Serial.println("  RED");
  delay(1000);
  
  tft.fillScreen(ST77XX_GREEN);
  Serial.println("  GREEN");
  delay(1000);
  
  tft.fillScreen(ST77XX_BLUE);
  Serial.println("  BLUE");
  delay(1000);
  
  tft.fillScreen(ST77XX_BLACK);
  
  // Text test
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(3);
  tft.setCursor(30, 100);
  tft.println("WORKING!");
  
  Serial.println("✓ Display initialized successfully");
  
  tftInitialized = true;
  delay(2000);
}

// ==================== SET TFT BRIGHTNESS ====================
void setTFTBrightness(uint8_t brightness) {
  pinMode(TFT_BL_PIN, OUTPUT);
  if (brightness > 128) {
    digitalWrite(TFT_BL_PIN, HIGH);
  } else {
    digitalWrite(TFT_BL_PIN, LOW);
  }
}

// ==================== BUZZER CONTROL WITH POLARITY SUPPORT ====================
void setBuzzer(bool state) {
  // If buzzer is inverted (NPN transistor causing active-LOW behavior), flip the state
  bool actualState = buzzerInverted ? !state : state;
  digitalWrite(BUZZER_PIN, actualState ? HIGH : LOW);
}

// ==================== INITIALIZE I2C LCD ====================
void initLCDDisplay() {
  Serial.print(F("Initializing I2C LCD... "));
  
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  lcd.init();
  lcd.backlight();
  lcdInitialized = true;
  
  // Welcome message
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("ESP32 Power Monitor");
  lcd.setCursor(0, 1);
  lcd.print("Version 5.4 LCD");
  if (LCD_ROWS > 2) {
    lcd.setCursor(0, 2);
    lcd.print("Initializing...");
  }
  
  Serial.println(F("OK"));
  delay(2000);
}

// ==================== UPDATE I2C LCD DISPLAY ====================
void updateLCDDisplay() {
  if (!lcdInitialized) return;

  // Track previous display mode to avoid unnecessary clearing
  static uint8_t prevDisplayMode = 255;  // 0=normal, 1=warning, 2=error
  uint8_t currentDisplayMode = 0;

  // Check for critical errors/warnings FIRST
  bool hasError = false;
  bool hasWarning = false;
  
  // Priority 1: Critical Errors (display immediately)
  if (voltageReading < MIN_VALID_VOLTAGE) {
    currentDisplayMode = 2;  // Error mode
    if (prevDisplayMode != currentDisplayMode) lcd.clear();

    // NO VOLTAGE ERROR
    lcd.setCursor(0, 0);
    lcd.print("** ERROR **         ");
    lcd.setCursor(0, 1);
    lcd.print("NO VOLTAGE!         ");
    lcd.setCursor(0, 2);
    lcd.print("Check ZMPT101B      ");
    lcd.setCursor(0, 3);
    char buf[21];
    snprintf(buf, sizeof(buf), "V: %.1fV             ", voltageReading);
    lcd.print(buf);
    hasError = true;
  }
  else if (channels[0].sensorFaultDetected || channels[1].sensorFaultDetected) {
    currentDisplayMode = 2;  // Error mode
    if (prevDisplayMode != currentDisplayMode) lcd.clear();

    // SENSOR FAULT ERROR
    lcd.setCursor(0, 0);
    lcd.print("** ERROR **         ");
    lcd.setCursor(0, 1);
    lcd.print("SENSOR FAULT!       ");
    lcd.setCursor(0, 2);
    if (channels[0].sensorFaultDetected) lcd.print("CH1 Sensor Fault    ");
    if (channels[1].sensorFaultDetected) lcd.print("CH2 Sensor Fault    ");
    lcd.setCursor(0, 3);
    lcd.print("Check ACS712        ");
    hasError = true;
  }
  
  // Priority 2: Warnings (if no errors)
  else if (overvoltageDetected) {
    currentDisplayMode = 1;  // Warning mode
    if (prevDisplayMode != currentDisplayMode) lcd.clear();

    // OVERVOLTAGE WARNING
    lcd.setCursor(0, 0);
    lcd.print("*** WARNING ***     ");
    lcd.setCursor(0, 1);
    lcd.print("OVERVOLTAGE!        ");
    lcd.setCursor(0, 2);
    char buf[21];
    snprintf(buf, sizeof(buf), "V: %.1fV (>250V)    ", voltageReading);
    lcd.print(buf);
    lcd.setCursor(0, 3);
    lcd.print("Relays OFF          ");
    hasWarning = true;
  }
  else if (brownoutDetected) {
    currentDisplayMode = 1;  // Warning mode
    if (prevDisplayMode != currentDisplayMode) lcd.clear();

    // BROWNOUT WARNING
    lcd.setCursor(0, 0);
    lcd.print("*** WARNING ***     ");
    lcd.setCursor(0, 1);
    lcd.print("BROWNOUT!           ");
    lcd.setCursor(0, 2);
    char buf[21];
    snprintf(buf, sizeof(buf), "V: %.1fV (<180V)    ", voltageReading);
    lcd.print(buf);
    lcd.setCursor(0, 3);
    lcd.print("Relays OFF          ");
    hasWarning = true;
  }
  else if (undervoltageDetected) {
    currentDisplayMode = 1;  // Warning mode
    if (prevDisplayMode != currentDisplayMode) lcd.clear();

    // UNDERVOLTAGE WARNING
    lcd.setCursor(0, 0);
    lcd.print("*** WARNING ***     ");
    lcd.setCursor(0, 1);
    lcd.print("UNDERVOLTAGE!       ");
    lcd.setCursor(0, 2);
    char buf[21];
    snprintf(buf, sizeof(buf), "V: %.1fV (<200V)    ", voltageReading);
    lcd.print(buf);
    lcd.setCursor(0, 3);
    lcd.print("Check voltage       ");
    hasWarning = true;
  }
  else if (channels[0].overcurrentDetected || channels[1].overcurrentDetected) {
    currentDisplayMode = 1;  // Warning mode
    if (prevDisplayMode != currentDisplayMode) lcd.clear();

    // OVERCURRENT WARNING
    lcd.setCursor(0, 0);
    lcd.print("*** WARNING ***     ");
    lcd.setCursor(0, 1);
    lcd.print("OVERCURRENT!        ");
    lcd.setCursor(0, 2);
    char buf[21];
    if (channels[0].overcurrentDetected) {
      snprintf(buf, sizeof(buf), "CH1: %.2fA          ", channels[0].currentReading);
    } else if (channels[1].overcurrentDetected) {
      snprintf(buf, sizeof(buf), "CH2: %.2fA          ", channels[1].currentReading);
    }
    lcd.print(buf);
    lcd.setCursor(0, 3);
    lcd.print("Limit: 4.5A         ");
    hasWarning = true;
  }
  
  // Priority 3: Normal Display (if no errors or warnings)
  if (!hasError && !hasWarning) {
    currentDisplayMode = 0;  // Normal mode
    if (prevDisplayMode != currentDisplayMode) lcd.clear();

    if (LCD_ROWS == 4 && LCD_COLS == 20) {
      // 20x4 LCD Layout with Timer
      float totalPower = channels[0].powerReading + channels[1].powerReading;
      float totalCost = channels[0].costAccumulated + channels[1].costAccumulated;
      char buf[21];

      // Line 0: Voltage, Power, and Network Status
      lcd.setCursor(0, 0);
      snprintf(buf, sizeof(buf), "V:%.1fV P:%.1fW      ", voltageReading, totalPower);
      buf[20] = '\0';  // Ensure null termination
      lcd.print(buf);

      // Network status indicator (last 2 chars of line 0)
      lcd.setCursor(18, 0);
      if (wifiConnected && serverConnected) {
        lcd.print("OK");
      } else if (wifiConnected) {
        lcd.print("W ");
      } else {
        lcd.print("--");
      }

      // Line 1: Channel 1 Current and Energy
      lcd.setCursor(0, 1);
      snprintf(buf, sizeof(buf), "CH1:%.2fA  %.2fkWh  ",
               channels[0].currentReading, channels[0].energyConsumed);
      buf[20] = '\0';
      lcd.print(buf);

      // Line 2: Channel 2 Current and Energy
      lcd.setCursor(0, 2);
      snprintf(buf, sizeof(buf), "CH2:%.2fA  %.2fkWh  ",
               channels[1].currentReading, channels[1].energyConsumed);
      buf[20] = '\0';
      lcd.print(buf);

      // Line 3: Total Cost and Relay Status
      lcd.setCursor(0, 3);
      snprintf(buf, sizeof(buf), "P%.2f R1:%s R2:%s ",
               totalCost,
               channels[0].relayEnabled ? "ON " : "OFF",
               channels[1].relayEnabled ? "ON " : "OFF");
      buf[20] = '\0';
      lcd.print(buf);
      
    } else if (LCD_ROWS == 2 && LCD_COLS == 16) {
      // 16x2 LCD Layout - Rotating display with timer
      static int displayPage = 0;
      static unsigned long lastPageChange = 0;
      char buf[17];

      if (millis() - lastPageChange > 3000) {
        displayPage = (displayPage + 1) % 4;  // 4 pages
        lastPageChange = millis();
        lcd.clear();  // Clear on page change for 16x2
      }

      switch (displayPage) {
        case 0: {
          // Voltage and Power
          lcd.setCursor(0, 0);
          snprintf(buf, sizeof(buf), "V:%.1fV P:%.1fW  ", voltageReading,
                   channels[0].powerReading + channels[1].powerReading);
          buf[16] = '\0';
          lcd.print(buf);

          lcd.setCursor(0, 1);
          // Network status
          if (wifiConnected && serverConnected) {
            lcd.print("Online          ");
          } else if (wifiConnected) {
            lcd.print("WiFi OK         ");
          } else {
            lcd.print("Offline         ");
          }
          break;
        }

        case 1: {
          // Channel 1
          lcd.setCursor(0, 0);
          snprintf(buf, sizeof(buf), "CH1: %.2fA      ", channels[0].currentReading);
          buf[16] = '\0';
          lcd.print(buf);

          lcd.setCursor(0, 1);
          snprintf(buf, sizeof(buf), "kWh: %.3f      ", channels[0].energyConsumed);
          buf[16] = '\0';
          lcd.print(buf);
          break;
        }

        case 2: {
          // Channel 2
          lcd.setCursor(0, 0);
          snprintf(buf, sizeof(buf), "CH2: %.2fA      ", channels[1].currentReading);
          buf[16] = '\0';
          lcd.print(buf);

          lcd.setCursor(0, 1);
          snprintf(buf, sizeof(buf), "kWh: %.3f      ", channels[1].energyConsumed);
          buf[16] = '\0';
          lcd.print(buf);
          break;
        }

        case 3: {
          // Cost and Status
          lcd.setCursor(0, 0);
          snprintf(buf, sizeof(buf), "Cost: P%.2f     ",
                   channels[0].costAccumulated + channels[1].costAccumulated);
          buf[16] = '\0';
          lcd.print(buf);

          lcd.setCursor(0, 1);
          snprintf(buf, sizeof(buf), "R1:%s R2:%s    ",
                   channels[0].relayEnabled ? "ON " : "OFF",
                   channels[1].relayEnabled ? "ON " : "OFF");
          buf[16] = '\0';
          lcd.print(buf);
          break;
        }
      }
    }
  }

  // Update previous display mode
  prevDisplayMode = currentDisplayMode;
}

// ==================== MAIN LOOP ====================
void loop() {
  unsigned long loopStart = millis();
  
  if (loopStart - lastLoopTime < 10) {
    return;
  }
  
  feedWatchdog();
  handleSerialCommands();
  
  // Network maintenance
  if (wifiConnected) {
    webSocket.loop();
    static unsigned long lastHeartbeat = 0;
    if (millis() - lastHeartbeat >= 30000) {
      if (webSocket.isConnected()) {
        StaticJsonDocument<64> doc;
        doc["type"] = "ping";
        String message;
        serializeJson(doc, message);
        webSocket.sendTXT(message);
      }
      lastHeartbeat = millis();
    }
    reconnectWebSocket();
  }
  
  checkWiFiConnection();
  
  if (currentState == STATE_CALIBRATING && calState != CAL_COMPLETE) {
    autoCalibrateSensorsNonBlocking();
    return;
  }
  
  if (sensorsValid && (loopStart - lastSampleTime >= SAMPLE_RATE)) {
    readSensors();
    updateMovingAverages();
    lastSampleTime = loopStart;
    
    for (int ch = 0; ch < NUM_CHANNELS; ch++) {
      if (!validateSensorReadings(ch)) {
        channels[ch].sensorFaultDebounceCount++;
        if (channels[ch].sensorFaultDebounceCount >= SAFETY_DEBOUNCE_COUNT && 
            !channels[ch].sensorFaultDetected) {
          char reason[32];
          snprintf(reason, sizeof(reason), "%s_SENSOR_FAULT", channels[ch].name);
          triggerSafetyShutdown(ch, reason);
        }
      } else {
        channels[ch].sensorFaultDebounceCount = 0;
      }
    }
    
    // Only enable safety checks after settling period
    if (safetyEnabled && !manualControl && currentState != STATE_ERROR && 
        (loopStart - systemStartTime > STARTUP_SETTLE_TIME)) {
      performSafetyChecks();
    }
    
    updateEnergyCalculation();
    updateCostCalculation();
  }
  
  // Send data to server
  if (wifiConnected && sensorsValid && (loopStart - lastServerPost >= SERVER_POST_INTERVAL)) {
    sendDataToServer();
    lastServerPost = loopStart;
  }
  
  if (loopStart - lastDisplayUpdate >= DISPLAY_INTERVAL) {
    updateDisplay();
    lastDisplayUpdate = loopStart;
  }
  
  if (tftInitialized && (loopStart - lastTFTUpdate >= TFT_UPDATE_INTERVAL)) {
    updateTFTDisplay();
    lastTFTUpdate = loopStart;
  }
  
  // Update LCD display
  if (lcdInitialized && (loopStart - lastLCDUpdate >= LCD_UPDATE_INTERVAL)) {
    updateLCDDisplay();
    lastLCDUpdate = loopStart;
  }
  
  if (loopStart - lastRelayVerify >= RELAY_VERIFY_INTERVAL) {
    verifyRelayStates();
    lastRelayVerify = loopStart;
  }
  
  updateStatusLEDs();
  
  if (sdCardAvailable && sensorsValid && (loopStart - lastLogUpdate >= LOG_INTERVAL_MS)) {
    char logBuffer[196];
    snprintf(logBuffer, sizeof(logBuffer), 
             "%lu,%.2f,%.3f,%.2f,%d,%.2f,%.3f,%.2f,%d,%.2f,%.4f",
             loopStart, voltageReading, 
             channels[CHANNEL_1].currentReading, channels[CHANNEL_1].powerReading, 
             channels[CHANNEL_1].relayEnabled ? 1 : 0,
             channels[CHANNEL_1].costAccumulated,
             channels[CHANNEL_2].currentReading, channels[CHANNEL_2].powerReading, 
             channels[CHANNEL_2].relayEnabled ? 1 : 0,
             channels[CHANNEL_2].costAccumulated,
             calculateTotalCost());
    logToSD(logBuffer);
    lastLogUpdate = loopStart;
  }
  
  unsigned long loopTime = millis() - loopStart;
  updatePerformanceMetrics(loopTime);
  
  lastLoopTime = loopStart;
  loopCount++;
}

// ==================== UPDATE TFT DISPLAY ====================
void updateTFTDisplay() {
  if (!tftInitialized || !displayEnabled) return;
  
  tft.fillScreen(COLOR_BG);
  drawTFTHeader();
  
  tft.setCursor(5, 25);
  tft.setTextSize(2);
  tft.setTextColor(COLOR_TEXT);
  tft.print("V: ");
  
  if (voltageReading < MIN_VOLTAGE) {
    tft.setTextColor(COLOR_ERROR);
  } else if (voltageReading > MAX_VOLTAGE) {
    tft.setTextColor(COLOR_ERROR);
  } else if (undervoltageDetected) {
    tft.setTextColor(COLOR_WARNING);
  } else {
    tft.setTextColor(COLOR_SUCCESS);
  }
  
  tft.print(voltageReading, 1);
  tft.setTextColor(COLOR_TEXT);
  tft.println("V");
  
  tft.drawFastHLine(0, 50, TFT_WIDTH, COLOR_DISABLED);
  drawChannelStatus(CHANNEL_1, 55);
  tft.drawFastHLine(0, 135, TFT_WIDTH, COLOR_DISABLED);
  drawChannelStatus(CHANNEL_2, 140);
  tft.drawFastHLine(0, 220, TFT_WIDTH, COLOR_DISABLED);
  drawTotalStats(225);
}

// ==================== DRAW TFT HEADER ====================
void drawTFTHeader() {
  tft.fillRect(0, 0, TFT_WIDTH, 20, COLOR_HEADER);
  tft.setTextSize(1);
  tft.setCursor(5, 7);
  tft.setTextColor(COLOR_TEXT);
  tft.print("ESP32 Monitor");
  
  tft.setCursor(170, 7);
  if (currentState == STATE_MONITORING) {
    bool anyOn = false;
    for (int ch = 0; ch < NUM_CHANNELS; ch++) {
      if (channels[ch].relayEnabled) anyOn = true;
    }
    if (anyOn) {
      tft.setTextColor(COLOR_SUCCESS);
      tft.print("RUN");
    } else {
      tft.setTextColor(COLOR_WARNING);
      tft.print("IDLE");
    }
  } else if (currentState == STATE_ERROR) {
    tft.setTextColor(COLOR_ERROR);
    tft.print("ERR");
  } else {
    tft.setTextColor(COLOR_INFO);
    tft.print(getStateString());
  }
}

// ==================== DRAW CHANNEL STATUS ====================
void drawChannelStatus(int channel, int yPos) {
  if (channel < 0 || channel >= NUM_CHANNELS) return;
  
  ChannelData* ch = &channels[channel];
  
  tft.setCursor(5, yPos);
  tft.setTextSize(2);
  tft.setTextColor(COLOR_TEXT);
  tft.print(ch->name);
  tft.print(": ");
  
  if (ch->relayEnabled) {
    tft.setTextColor(COLOR_SUCCESS);
    tft.println("ON ");
  } else {
    tft.setTextColor(COLOR_DISABLED);
    tft.println("OFF");
  }
  
  tft.setCursor(5, yPos + 20);
  tft.setTextSize(1);
  tft.setTextColor(COLOR_TEXT);
  tft.print("I: ");
  
  if (ch->overcurrentDetected) {
    tft.setTextColor(COLOR_ERROR);
  } else if (ch->currentReading > MAX_CURRENT * 0.8) {
    tft.setTextColor(COLOR_WARNING);
  } else {
    tft.setTextColor(COLOR_TEXT);
  }
  
  tft.print(ch->currentReading, 3);
  tft.setTextColor(COLOR_TEXT);
  tft.println(" A");
  
  tft.setCursor(5, yPos + 35);
  tft.setTextColor(COLOR_TEXT);
  tft.print("P: ");
  tft.print(ch->powerReading, 1);
  tft.println(" W");
  
  tft.setCursor(5, yPos + 50);
  tft.setTextColor(COLOR_INFO);
  float costPerHr = calculateCostPerHour(ch->powerReading);
  tft.print("P");
  tft.print(costPerHr, 2);
  tft.println("/hr");
  
  tft.setCursor(130, yPos + 50);
  tft.setTextColor(COLOR_WARNING);
  tft.print("P");
  tft.print(ch->costAccumulated, 2);
}

// ==================== DRAW TOTAL STATS ====================
void drawTotalStats(int yPos) {
  float totalPower = 0;
  float totalCost = calculateTotalCost();
  
  for (int ch = 0; ch < NUM_CHANNELS; ch++) {
    totalPower += channels[ch].powerReading;
  }
  
  tft.setCursor(5, yPos);
  tft.setTextSize(1);
  tft.setTextColor(COLOR_TEXT);
  tft.print("TOT: ");
  tft.print(totalPower, 1);
  tft.print("W  P");
  tft.print(totalCost, 2);
}

// ==================== COST CALCULATION ====================
float calculateCostPerHour(float watts) {
  return (watts / 1000.0) * calData.electricityRate;
}

float calculateTotalCost() {
  float total = 0;
  for (int ch = 0; ch < NUM_CHANNELS; ch++) {
    total += channels[ch].costAccumulated;
  }
  return total;
}

void updateCostCalculation() {
  for (int ch = 0; ch < NUM_CHANNELS; ch++) {
    channels[ch].costAccumulated = (channels[ch].energyConsumed / 1000.0) * calData.electricityRate;
  }
}

// ==================== WATCHDOG ====================
void setupWatchdog() {
  Serial.println(F("Setting up watchdog timer..."));
  esp_task_wdt_init(WATCHDOG_TIMEOUT_S, true);
  esp_task_wdt_add(NULL);
  Serial.printf("Watchdog enabled: %ds timeout\n", WATCHDOG_TIMEOUT_S);
}

void feedWatchdog() {
  if (millis() - lastWatchdogFeed > 1000) {
    esp_task_wdt_reset();
    lastWatchdogFeed = millis();
  }
}

// ==================== STARTUP SEQUENCE ====================
void startupSequence() {
  Serial.println(F("\n=== DUAL CHANNEL INITIALIZATION ==="));

  Serial.println(F("Testing LEDs..."));
  digitalWrite(RED_LED_PIN, HIGH);
  delay(200);
  digitalWrite(RED_LED_PIN, LOW);
  digitalWrite(YELLOW_LED_PIN, HIGH);
  delay(200);
  digitalWrite(YELLOW_LED_PIN, LOW);
  digitalWrite(GREEN_LED_PIN, HIGH);
  delay(200);
  digitalWrite(GREEN_LED_PIN, LOW);
  digitalWrite(BLUE_LED_PIN, HIGH);
  delay(200);
  digitalWrite(BLUE_LED_PIN, LOW);
  
  if (buzzerEnabled) {
    Serial.println(F("Testing buzzer..."));
    setBuzzer(true);
    delay(100);
    setBuzzer(false);
    delay(100);
    setBuzzer(true);
    delay(100);
    setBuzzer(false);
  }
  
  Serial.println(F("✓ Hardware test complete"));
  
  Serial.println(F("\n=== AUTO-CALIBRATING SENSORS ==="));
  Serial.println(F("IMPORTANT: Ensure NO LOAD on BOTH channels!"));
  
  calState = CAL_SAMPLING;
  calProgress.samplesCollected = 0;
  calProgress.ch1CurrentSum = 0.0;
  calProgress.ch2CurrentSum = 0.0;
  calProgress.voltageSum = 0.0;
  calProgress.voltageMin = 999.0;
  calProgress.voltageMax = 0.0;
  calProgress.startTime = millis();
}

// ==================== NON-BLOCKING CALIBRATION ====================
void autoCalibrateSensorsNonBlocking() {
  switch (calState) {
    case CAL_SAMPLING:
      if (calProgress.samplesCollected < CALIBRATION_SAMPLES) {
        float ch1V = readRawSensorVoltage(channels[CHANNEL_1].currentSensorPin, ADC_SAMPLES_SLOW);
        float ch2V = readRawSensorVoltage(channels[CHANNEL_2].currentSensorPin, ADC_SAMPLES_SLOW);
        float voltV = readRawSensorVoltage(ZMPT101B_PIN, ADC_SAMPLES_SLOW);
        
        calProgress.ch1CurrentSum += ch1V;
        calProgress.ch2CurrentSum += ch2V;
        calProgress.voltageSum += voltV;
        
        if (voltV < calProgress.voltageMin) calProgress.voltageMin = voltV;
        if (voltV > calProgress.voltageMax) calProgress.voltageMax = voltV;
        
        calProgress.samplesCollected++;
        
        if (calProgress.samplesCollected % 50 == 0) {
          Serial.print(F("."));
          feedWatchdog();
        }
      } else {
        Serial.println();
        
        calData.ch1CurrentOffset = calProgress.ch1CurrentSum / CALIBRATION_SAMPLES;
        calData.ch2CurrentOffset = calProgress.ch2CurrentSum / CALIBRATION_SAMPLES;
        
        float voltageAvg = calProgress.voltageSum / CALIBRATION_SAMPLES;
        float voltageCenter = (calProgress.voltageMin + calProgress.voltageMax) / 2.0;
        float voltageSwing = calProgress.voltageMax - calProgress.voltageMin;
        
        if (voltageSwing > MIN_AC_SWING) {
          calData.voltageOffset = voltageCenter;
          Serial.printf("Voltage offset (AC center): %.3fV\n", voltageCenter);
          Serial.printf("  AC swing: %.3fV (%.3fV to %.3fV)\n", 
                       voltageSwing, calProgress.voltageMin, calProgress.voltageMax);
        } else {
          calData.voltageOffset = voltageAvg;
          Serial.printf("Voltage offset (average): %.3fV\n", voltageAvg);
        }
        
        channels[CHANNEL_1].currentOffset = calData.ch1CurrentOffset;
        channels[CHANNEL_2].currentOffset = calData.ch2CurrentOffset;
        voltageOffset = calData.voltageOffset;
        
        Serial.printf("\nCalibration Results:\n");
        Serial.printf("  CH1 offset: %.3fV ", calData.ch1CurrentOffset);
        if (calData.ch1CurrentOffset >= EXPECTED_OFFSET_MIN && 
            calData.ch1CurrentOffset <= EXPECTED_OFFSET_MAX) {
          Serial.println(F("✓ OK"));
        } else {
          Serial.printf("⚠️  (expected ~%.2fV)\n", ACS712_ZERO_CURRENT * VOLTAGE_DIVIDER_RATIO);
        }
        
        Serial.printf("  CH2 offset: %.3fV ", calData.ch2CurrentOffset);
        if (calData.ch2CurrentOffset >= EXPECTED_OFFSET_MIN && 
            calData.ch2CurrentOffset <= EXPECTED_OFFSET_MAX) {
          Serial.println(F("✓ OK"));
        } else {
          Serial.printf("⚠️  (expected ~%.2fV)\n", ACS712_ZERO_CURRENT * VOLTAGE_DIVIDER_RATIO);
        }
        
        Serial.printf("  Voltage offset: %.3fV ✓\n", calData.voltageOffset);
        
        calState = CAL_VERIFICATION;
      }
      break;
      
    case CAL_VERIFICATION:
      {
        Serial.println(F("\n=== VALIDATING SENSORS ==="));
        
        float testCurrent1 = calculateRMSCurrent(CHANNEL_1);
        float testCurrent2 = calculateRMSCurrent(CHANNEL_2);
        float testVoltage = calculateRMSVoltage(ZMPT101B_PIN);
        
        Serial.printf("Test readings:\n");
        Serial.printf("  CH1: %.3fA\n", testCurrent1);
        Serial.printf("  CH2: %.3fA\n", testCurrent2);
        Serial.printf("  Voltage: %.1fV\n", testVoltage);
        
        bool ch1Valid = (testCurrent1 <= 1.0);
        bool ch2Valid = (testCurrent2 <= 1.0);
        bool voltageValid = (testVoltage >= MIN_VALID_VOLTAGE);
        
        if (!voltageValid) {
          Serial.println(F("⚠️  Low voltage - auto-calibrating..."));
          autoDetectVoltageOffset();
          testVoltage = calculateRMSVoltage(ZMPT101B_PIN);
          Serial.printf("  After auto-cal: %.1fV\n", testVoltage);
          voltageValid = (testVoltage >= MIN_VALID_VOLTAGE);
          
          if (!voltageValid) {
            Serial.println(F("  ⚠️  Still low - use 'cal_voltage'"));
          }
        }
        
        sensorsValid = ch1Valid && ch2Valid && voltageValid;
        
        if (sensorsValid) {
          Serial.println(F("\n✓ All sensors validated"));
          verifyCalibration();
          
          // Pre-fill voltage buffer to prevent false alarms
          preFillVoltageBuffer(testVoltage);
          
          // AUTO-ENABLE RELAYS after successful validation
          Serial.println(F("\n=== AUTO-ENABLING RELAYS ==="));
          Serial.println(F("Relays will turn ON in 3 seconds..."));
          delay(1000);
          Serial.println(F("3..."));
          delay(1000);
          Serial.println(F("2..."));
          delay(1000);
          Serial.println(F("1..."));
          delay(1000);
          
          // Enable both channels
          enableChannel(CHANNEL_1);
          enableChannel(CHANNEL_2);
          
          Serial.println(F("✓ Both relays ENABLED"));
          Serial.println(F("✓ System ACTIVE - monitoring for safety"));
          
          // Buzzer confirmation (Active Buzzer - uses digitalWrite)
          if (buzzerEnabled) {
            setBuzzer(true);   // Turn on
            delay(150);
            setBuzzer(false);  // Turn off
            delay(50);
            setBuzzer(true);   // Turn on
            delay(150);
            setBuzzer(false);  // Turn off
          }
        } else {
          Serial.println(F("\n⚠️  Sensor warnings present"));
          Serial.println(F("   System in SAFE MODE"));
          Serial.println(F("   Relays will NOT auto-enable"));
        }
        
        saveCalibrationData();

        Serial.println(F("\n✓ System ready for operation"));
        currentState = STATE_MONITORING;
        calState = CAL_COMPLETE;
        // LED will be controlled by updateStatusLEDs()
      }
      break;
      
    default:
      break;
  }
}

// ==================== PRE-FILL VOLTAGE BUFFER ====================
void preFillVoltageBuffer(float voltage) {
  for (int i = 0; i < MOVING_AVERAGE_SIZE; i++) {
    voltageBuffer[i] = voltage;
  }
  Serial.printf("Voltage buffer pre-filled: %.1fV\n", voltage);
}

// ==================== AUTO DETECT VOLTAGE OFFSET ====================
void autoDetectVoltageOffset() {
  Serial.println(F("Auto-detecting voltage offset..."));
  
  float minV = 999.0;
  float maxV = 0.0;
  float sum = 0.0;
  
  for (int i = 0; i < VOLTAGE_AUTO_CAL_SAMPLES; i++) {
    float v = readRawSensorVoltage(ZMPT101B_PIN, ADC_SAMPLES_SLOW);
    if (v < minV) minV = v;
    if (v > maxV) maxV = v;
    sum += v;
    delay(100);
    feedWatchdog();
  }
  
  float avg = sum / VOLTAGE_AUTO_CAL_SAMPLES;
  float center = (minV + maxV) / 2.0;
  float swing = maxV - minV;
  
  Serial.printf("  Range: %.3fV-%.3fV (swing: %.3fV)\n", minV, maxV, swing);
  
  if (swing > MIN_AC_SWING) {
    voltageOffset = center;
    calData.voltageOffset = center;
    Serial.printf("  ✓ Using center: %.3fV\n", center);
    
    float estimatedRMS = swing / 2.828;
    if (estimatedRMS > 0.1) {
      float estimatedCal = 220.0 / estimatedRMS;
      voltageCalibration = estimatedCal;
      calData.voltageCalibration = estimatedCal;
      Serial.printf("  ✓ Estimated cal: %.1f\n", estimatedCal);
    }
  }
}

// ==================== CALIBRATION VERIFICATION ====================
void verifyCalibration() {
  float testVoltage = calculateRMSVoltage(ZMPT101B_PIN);
  
  if (testVoltage > 100 && testVoltage < 300) {
    Serial.println(F("✓ Voltage calibration verified"));
  } else if (testVoltage > 50) {
    Serial.println(F("⚠️  Voltage may need calibration"));
    Serial.println(F("   Use 'cal_voltage' command"));
  }
}

// ==================== READ RAW SENSOR VOLTAGE ====================
float readRawSensorVoltage(int pin, int samples) {
  uint32_t sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += analogRead(pin);
  }
  float avgRaw = (float)sum / samples;
  return (avgRaw / ADC_RESOLUTION) * ADC_VREF;
}

// ==================== VOLTAGE CALIBRATION WIZARD ====================
void calibrateVoltageWithReference() {
  Serial.println(F("\n=== VOLTAGE CALIBRATION WIZARD ==="));
  Serial.println(F("1. Measure mains with multimeter"));
  Serial.println(F("2. Enter measured voltage (e.g. 220.5)"));
  Serial.print(F("\nEnter voltage: "));
  
  unsigned long startWait = millis();
  const unsigned long timeout = 20000;
  
  while(!Serial.available() && (millis() - startWait < timeout)) {
    delay(100);
    feedWatchdog();
  }
  
  if (Serial.available()) {
    float measured = Serial.parseFloat();
    Serial.println(measured);
    
    while(Serial.available()) Serial.read();
    
    if (measured > 100.0 && measured < 300.0) {
      float current = calculateRMSVoltage(ZMPT101B_PIN);
      
      if (current > 1.0) {
        calData.voltageCalibration = measured / current;
        voltageCalibration = calData.voltageCalibration;
        Serial.printf("✓ Voltage cal: %.4f\n", calData.voltageCalibration);
        Serial.printf("  %.1fV → %.1fV\n", current, measured);
        saveCalibrationData();
        verifyCalibration();
      } else {
        Serial.println(F("✗ Reading too low"));
      }
    } else {
      Serial.println(F("✗ Invalid value"));
    }
  } else {
    Serial.println(F("Timeout"));
  }
}

// ==================== SENSOR READING ====================
void readSensors() {
  voltageReading = calculateRMSVoltage(ZMPT101B_PIN);
  
  for (int ch = 0; ch < NUM_CHANNELS; ch++) {
    float rawCurrent = calculateRMSCurrent(ch);
    
    // Apply deadband
    if (rawCurrent < CURRENT_DEADBAND) {
      rawCurrent = 0.0;
    }
    
    channels[ch].currentReading = rawCurrent;
    
    if (channels[ch].currentReading > MAX_VALID_CURRENT) {
      Serial.printf("⚠️  %s: Invalid current %.2fA\n", 
                    channels[ch].name, channels[ch].currentReading);
      channels[ch].currentReading = 0.0;
      channels[ch].sensorFaultDebounceCount++;
    }
    
    channels[ch].apparentPower = voltageReading * channels[ch].currentReading;
    channels[ch].powerReading = channels[ch].apparentPower * calData.powerFactor;
  }
  
  updateStatistics();
}

float calculateRMSCurrent(int channel) {
  if (channel < 0 || channel >= NUM_CHANNELS) return 0.0;
  
  float sum = 0;
  
  for(int i = 0; i < RMS_SAMPLES; i++) {
    float voltage = readRawSensorVoltage(channels[channel].currentSensorPin, ADC_SAMPLES_FAST);
    
    float actualACS712Voltage = voltage * VOLTAGE_DIVIDER_SCALE;
    float calibratedZeroPoint = channels[channel].currentOffset * VOLTAGE_DIVIDER_SCALE;
    float currentDiff = actualACS712Voltage - calibratedZeroPoint;
    float instantCurrent = currentDiff / ACS712_SENSITIVITY;
    
    sum += instantCurrent * instantCurrent;
    
    delayMicroseconds(RMS_SAMPLE_DELAY_US);
  }
  
  float rms = sqrt(sum / RMS_SAMPLES);
  return abs(rms * channels[channel].currentCalibration);
}

float calculateRMSVoltage(int sensorPin) {
  float sum = 0;
  
  for(int i = 0; i < RMS_SAMPLES; i++) {
    float voltage = readRawSensorVoltage(sensorPin, ADC_SAMPLES_FAST);
    float acVoltage = voltage - voltageOffset;
    sum += acVoltage * acVoltage;
    delayMicroseconds(RMS_SAMPLE_DELAY_US);
  }
  
  float rms = sqrt(sum / RMS_SAMPLES);
  return rms * voltageCalibration;
}

// ==================== MOVING AVERAGE ====================
void updateMovingAverages() {
  voltageBuffer[voltageBufferIndex] = voltageReading;
  voltageBufferIndex = (voltageBufferIndex + 1) % MOVING_AVERAGE_SIZE;
  
  for (int ch = 0; ch < NUM_CHANNELS; ch++) {
    channels[ch].currentBuffer[channels[ch].bufferIndex] = channels[ch].currentReading;
    channels[ch].bufferIndex = (channels[ch].bufferIndex + 1) % MOVING_AVERAGE_SIZE;
  }
}

float getAverageCurrent(int channel) {
  if (channel < 0 || channel >= NUM_CHANNELS) return 0.0;
  
  float sum = 0.0;
  for (int i = 0; i < MOVING_AVERAGE_SIZE; i++) {
    sum += channels[channel].currentBuffer[i];
  }
  return sum / MOVING_AVERAGE_SIZE;
}

float getAverageVoltage() {
  float sum = 0.0;
  for (int i = 0; i < MOVING_AVERAGE_SIZE; i++) {
    sum += voltageBuffer[i];
  }
  return sum / MOVING_AVERAGE_SIZE;
}

// ==================== SENSOR VALIDATION ====================
bool validateSensorReadings(int channel) {
  if (channel < 0 || channel >= NUM_CHANNELS) return false;
  
  if (voltageReading > SENSOR_FAULT_VOLTAGE) {
    channels[channel].sensorFaultDetected = true;
    return false;
  }
  
  if (channels[channel].currentReading > MAX_SENSOR_CURRENT) {
    channels[channel].sensorFaultDetected = true;
    return false;
  }
  
  if (isnan(voltageReading) || isnan(channels[channel].currentReading) || 
      isinf(voltageReading) || isinf(channels[channel].currentReading)) {
    channels[channel].sensorFaultDetected = true;
    return false;
  }
  
  channels[channel].sensorFaultDetected = false;
  return true;
}

// ==================== SAFETY CHECKS ====================
void performSafetyChecks() {
  float avgVoltage = getAverageVoltage();
  
  if (!sensorsValid) return;
  
  for (int ch = 0; ch < NUM_CHANNELS; ch++) {
    float avgCurrent = getAverageCurrent(ch);
    
    if (avgCurrent > MAX_CURRENT) {
      channels[ch].overcurrentDebounceCount++;
      
      if (!channels[ch].overcurrentDetected) {
        channels[ch].overcurrentDetected = true;
        channels[ch].overcurrentStartTime = millis();
        Serial.printf("⚠️  %s Overcurrent: %.2fA\n", channels[ch].name, avgCurrent);
      }
      
      if (channels[ch].overcurrentDebounceCount >= SAFETY_DEBOUNCE_COUNT && 
          (millis() - channels[ch].overcurrentStartTime > OVERCURRENT_TIME)) {
        char reason[32];
        snprintf(reason, sizeof(reason), "%s_OVERCURRENT", channels[ch].name);
        triggerSafetyShutdown(ch, reason);
      }
    } else {
      channels[ch].overcurrentDetected = false;
      channels[ch].overcurrentDebounceCount = 0;
    }
  }
  
  if (avgVoltage > MAX_VOLTAGE) {
    overvoltageDebounceCount++;
    
    if (!overvoltageDetected) {
      overvoltageDetected = true;
      overvoltageStartTime = millis();
      Serial.printf("⚠️  Overvoltage: %.1fV\n", avgVoltage);
    }
    
    if (overvoltageDebounceCount >= SAFETY_DEBOUNCE_COUNT && 
        (millis() - overvoltageStartTime > OVERVOLTAGE_TIME)) {
      for (int ch = 0; ch < NUM_CHANNELS; ch++) {
        triggerSafetyShutdown(ch, "OVERVOLTAGE");
      }
      return;
    }
  } else {
    overvoltageDetected = false;
    overvoltageDebounceCount = 0;
  }
  
  if (avgVoltage < MIN_VOLTAGE && avgVoltage > MIN_VALID_VOLTAGE) {
    if (!undervoltageDetected) {
      undervoltageDetected = true;
      Serial.printf("⚠️  Undervoltage: %.1fV\n", avgVoltage);
    }
  } else {
    undervoltageDetected = false;
  }
  
  if (avgVoltage < BROWNOUT_VOLTAGE && avgVoltage > MIN_VALID_VOLTAGE) {
    if (!brownoutDetected) {
      brownoutDetected = true;
      brownoutStartTime = millis();
      Serial.printf("⚠️  Brownout: %.1fV\n", avgVoltage);
    }
    
    if (millis() - brownoutStartTime > BROWNOUT_TIME) {
      handleBrownout();
    }
  } else {
    brownoutDetected = false;
  }
}

// ==================== BROWNOUT HANDLER ====================
void handleBrownout() {
  Serial.println(F("⚠️  Extended brownout - protecting loads"));
  disableAllChannels();
}

// ==================== SAFETY SHUTDOWN ====================
void triggerSafetyShutdown(int channel, const char* reason) {
  if (channel < 0 || channel >= NUM_CHANNELS) return;

  Serial.printf("\n🚨 SAFETY SHUTDOWN: %s - %s\n", channels[channel].name, reason);

  disableChannel(channel);

  currentState = STATE_ERROR;
  channels[channel].shutdownCount++;

  // LEDs will be controlled by updateStatusLEDs()
  
  if (buzzerEnabled) {
    for (int i = 0; i < 3; i++) {
      setBuzzer(true);
      delay(150);
      setBuzzer(false);
      delay(100);
      feedWatchdog();
    }
  }
  
  if (sdCardAvailable) {
    char logBuffer[64];
    snprintf(logBuffer, sizeof(logBuffer), "SHUTDOWN,%s,%s,%lu", 
             channels[channel].name, reason, millis());
    logToSD(logBuffer);
  }
  
  Serial.printf("*** %s IN SAFETY SHUTDOWN ***\n", channels[channel].name);
  Serial.println(F("Type 'reset' to restart"));
}

// ==================== CHANNEL CONTROL ====================
void enableChannel(int channel) {
  if (channel < 0 || channel >= NUM_CHANNELS) return;
  
  if (!isSafeToOperate(channel)) {
    return;
  }
  
  channels[channel].relayEnabled = true;
  channels[channel].relayCommandState = true;
  digitalWrite(channels[channel].relayPin, RELAY_ON_STATE);
  delay(100);  // Let relay settle
  Serial.printf("Relay %d actual state: %s\n", 
    channel, 
    digitalRead(channels[channel].relayPin) == RELAY_ON_STATE ? "ON" : "OFF"
  );
  Serial.printf("✓ %s ENABLED\n", channels[channel].name);
}

void disableChannel(int channel) {
  if (channel < 0 || channel >= NUM_CHANNELS) return;
  
  channels[channel].relayEnabled = false;
  channels[channel].relayCommandState = false;
  digitalWrite(channels[channel].relayPin, RELAY_OFF_STATE);
  Serial.printf("✓ %s DISABLED\n", channels[channel].name);
}

void enableAllChannels() {
  Serial.println(F("Enabling all channels..."));
  for (int ch = 0; ch < NUM_CHANNELS; ch++) {
    enableChannel(ch);
  }
}

void disableAllChannels() {
  Serial.println(F("Disabling all channels..."));
  for (int ch = 0; ch < NUM_CHANNELS; ch++) {
    disableChannel(ch);
  }
}

// ==================== RELAY VERIFICATION ====================
void verifyRelayStates() {
  for (int ch = 0; ch < NUM_CHANNELS; ch++) {
    bool expected = channels[ch].relayCommandState;
    
    if (channels[ch].relayEnabled != expected && currentState != STATE_ERROR) {
      Serial.printf("⚠️  %s relay mismatch\n", channels[ch].name);
      digitalWrite(channels[ch].relayPin, expected ? RELAY_ON_STATE : RELAY_OFF_STATE);
      channels[ch].relayEnabled = expected;
    }
  }
}

// ==================== ENERGY CALCULATION ====================
void updateEnergyCalculation() {
  static unsigned long lastUpdate = 0;
  unsigned long currentTime = millis();
  
  if (lastUpdate > 0) {
    float deltaTime = (currentTime - lastUpdate) / 1000.0 / 3600.0;
    
    for (int ch = 0; ch < NUM_CHANNELS; ch++) {
      if (channels[ch].relayEnabled && channels[ch].powerReading > 0.1) {
        channels[ch].energyConsumed += channels[ch].powerReading * deltaTime;
      }
    }
  }
  
  lastUpdate = currentTime;
}

// ==================== STATISTICS ====================
void updateStatistics() {
  for (int ch = 0; ch < NUM_CHANNELS; ch++) {
    if (channels[ch].currentReading > channels[ch].maxCurrent && 
        channels[ch].currentReading < MAX_VALID_CURRENT) {
      channels[ch].maxCurrent = channels[ch].currentReading;
    }
    
    if (channels[ch].powerReading > channels[ch].maxPower) {
      channels[ch].maxPower = channels[ch].powerReading;
    }
  }
  
  if (voltageReading > maxVoltage && voltageReading < SENSOR_FAULT_VOLTAGE) {
    maxVoltage = voltageReading;
  }
  
  if (voltageReading < minVoltage && voltageReading > MIN_VALID_VOLTAGE) {
    minVoltage = voltageReading;
  }
}

// ==================== PERFORMANCE ====================
void updatePerformanceMetrics(unsigned long loopTime) {
  perfMetrics.loopTimeSum += loopTime;
  perfMetrics.sampleCount++;
  
  if (loopTime > perfMetrics.maxLoopTime) {
    perfMetrics.maxLoopTime = loopTime;
  }
  
  if (loopTime < perfMetrics.minLoopTime) {
    perfMetrics.minLoopTime = loopTime;
  }
  
  if (perfMetrics.sampleCount >= PERF_SAMPLES) {
    perfMetrics.avgLoopTime = perfMetrics.loopTimeSum / perfMetrics.sampleCount;
    perfMetrics.loopTimeSum = 0;
    perfMetrics.sampleCount = 0;
  }
}

// ==================== STATUS LEDs ====================
void updateStatusLEDs() {
  static unsigned long lastBlink = 0;
  static bool blinkState = false;

  // Turn off all LEDs first
  digitalWrite(RED_LED_PIN, LOW);
  digitalWrite(YELLOW_LED_PIN, LOW);
  digitalWrite(GREEN_LED_PIN, LOW);
  digitalWrite(BLUE_LED_PIN, LOW);

  // Check for sensor faults (highest priority - RED LED)
  bool anySensorFault = false;
  for (int ch = 0; ch < NUM_CHANNELS; ch++) {
    if (channels[ch].sensorFaultDetected) {
      anySensorFault = true;
      break;
    }
  }

  if (anySensorFault) {
    // RED LED: Blink for sensor faults/short circuit
    if (millis() - lastBlink > 250) {
      blinkState = !blinkState;
      lastBlink = millis();
    }
    digitalWrite(RED_LED_PIN, blinkState);
    return;
  }

  // Check for overcurrent (YELLOW LED - overload)
  bool anyOvercurrent = false;
  for (int ch = 0; ch < NUM_CHANNELS; ch++) {
    if (channels[ch].overcurrentDetected) {
      anyOvercurrent = true;
      break;
    }
  }

  if (anyOvercurrent) {
    // YELLOW LED: Solid for overcurrent/overload
    digitalWrite(YELLOW_LED_PIN, HIGH);
    return;
  }

  // Check for voltage issues (BLUE LED)
  float avgVoltage = getAverageVoltage();
  if (overvoltageDetected || undervoltageDetected || brownoutDetected) {
    // BLUE LED: Solid for under/overvoltage
    digitalWrite(BLUE_LED_PIN, HIGH);
    return;
  }

  // Check if any channel is enabled
  bool anyEnabled = false;
  for (int ch = 0; ch < NUM_CHANNELS; ch++) {
    if (channels[ch].relayEnabled) anyEnabled = true;
  }

  // Normal operation states
  switch (currentState) {
    case STATE_MONITORING:
      if (anyEnabled) {
        // GREEN LED: Solid for normal operation
        digitalWrite(GREEN_LED_PIN, HIGH);
      } else {
        // GREEN LED: Blink when monitoring but relays disabled
        if (millis() - lastBlink > 1000) {
          blinkState = !blinkState;
          lastBlink = millis();
        }
        digitalWrite(GREEN_LED_PIN, blinkState);
      }
      break;

    case STATE_ERROR:
      // RED LED: Blink for general errors
      if (millis() - lastBlink > 250) {
        blinkState = !blinkState;
        lastBlink = millis();
      }
      digitalWrite(RED_LED_PIN, blinkState);
      break;

    case STATE_MANUAL_CONTROL:
      // GREEN LED: Solid in manual control mode
      digitalWrite(GREEN_LED_PIN, HIGH);
      break;

    default:
      // All LEDs off for unknown states
      break;
  }
}

// ==================== DISPLAY UPDATE ====================
void updateDisplay() {
  if (!sensorsValid) {
    Serial.println(F("\n⚠️  SENSOR VALIDATION FAILED"));
    Serial.println(F("   System in SAFE MODE"));
    Serial.println(F("   Type 'test' for diagnostics\n"));
    return;
  }
  
  Serial.println(F("\n========================================"));
  Serial.printf("Status: %s | Loops: %lu\n", getStateString(), loopCount);
  Serial.println(F("========================================"));
  Serial.printf("Voltage: %.1f V (Avg: %.1f V)\n", voltageReading, getAverageVoltage());
  Serial.println(F("----------------------------------------"));
  
  for (int ch = 0; ch < NUM_CHANNELS; ch++) {
    Serial.printf("%s: %s\n", channels[ch].name, channels[ch].relayEnabled ? "ON " : "OFF");
    Serial.printf("  Current: %.3f A (Avg: %.3f A)\n", 
                  channels[ch].currentReading, getAverageCurrent(ch));
    Serial.printf("  Power: %.1f W (Apparent: %.1f VA)\n", 
                  channels[ch].powerReading, channels[ch].apparentPower);
    Serial.printf("  Energy: %.3f Wh (%.2f kWh)\n", 
                  channels[ch].energyConsumed, channels[ch].energyConsumed / 1000.0);
    Serial.printf("  Cost/hr: ₱%.2f | Total: ₱%.2f\n",
                  calculateCostPerHour(channels[ch].powerReading),
                  channels[ch].costAccumulated);
    
    if (ch < NUM_CHANNELS - 1) {
      Serial.println(F("  - - - - - - - - - - - - - - - - - - -"));
    }
  }
  
  float totalPower = 0;
  float totalEnergy = 0;
  for (int ch = 0; ch < NUM_CHANNELS; ch++) {
    totalPower += channels[ch].powerReading;
    totalEnergy += channels[ch].energyConsumed;
  }
  Serial.println(F("----------------------------------------"));
  Serial.printf("TOTAL Power: %.1f W | Energy: %.3f Wh\n", totalPower, totalEnergy);
  Serial.printf("TOTAL Cost: ₱%.2f (Rate: ₱%.2f/kWh)\n", 
                calculateTotalCost(), calData.electricityRate);
  
  if (perfMetrics.avgLoopTime > 0) {
    Serial.printf("Loop: %lu ms (min: %lu, max: %lu)\n", 
                  perfMetrics.avgLoopTime, perfMetrics.minLoopTime, perfMetrics.maxLoopTime);
  }
  
  Serial.println(F("========================================\n"));
}

const char* getStateString() {
  switch (currentState) {
    case STATE_INITIALIZING: return "INIT";
    case STATE_CALIBRATING: return "CALIB";
    case STATE_MONITORING: return "MONITOR";
    case STATE_MANUAL_CONTROL: return "MANUAL";
    case STATE_ERROR: return "ERROR";
    case STATE_SHUTDOWN: return "SHUTDOWN";
    default: return "UNKNOWN";
  }
}

// ==================== EMERGENCY RESET ====================
void emergencyReset() {
  Serial.println(F("\n🔄 EMERGENCY RESET"));
  
  currentState = STATE_CALIBRATING;
  calState = CAL_SAMPLING;
  
  disableAllChannels();
  
  for (int ch = 0; ch < NUM_CHANNELS; ch++) {
    channels[ch].overcurrentDetected = false;
    channels[ch].sensorFaultDetected = false;
    channels[ch].overcurrentDebounceCount = 0;
    channels[ch].sensorFaultDebounceCount = 0;
  }
  
  overvoltageDetected = false;
  undervoltageDetected = false;
  brownoutDetected = false;
  overvoltageDebounceCount = 0;
  criticalError = false;

  // LEDs will be controlled by updateStatusLEDs()

  Serial.println(F("Recalibrating..."));
  calProgress.samplesCollected = 0;
  calProgress.ch1CurrentSum = 0.0;
  calProgress.ch2CurrentSum = 0.0;
  calProgress.voltageSum = 0.0;
  calProgress.voltageMin = 999.0;
  calProgress.voltageMax = 0.0;
  calProgress.startTime = millis();
  
  while (calState != CAL_COMPLETE && (millis() - calProgress.startTime < 10000)) {
    autoCalibrateSensorsNonBlocking();
    feedWatchdog();
    delay(10);
  }
  
  if (calState == CAL_COMPLETE) {
    Serial.println(F("✓ Reset complete"));
  } else {
    Serial.println(F("⚠️  Reset timeout"));
  }
}

// ==================== SAFE OPERATION CHECK ====================
bool isSafeToOperate(int channel) {
  if (channel < 0 || channel >= NUM_CHANNELS) {
    Serial.println(F("✗ Invalid channel"));
    return false;
  }
  
  if (!sensorsValid) {
    Serial.printf("✗ %s: Sensors invalid\n", channels[channel].name);
    return false;
  }
  
  if (currentState == STATE_ERROR) {
    Serial.printf("✗ %s: System in ERROR\n", channels[channel].name);
    Serial.println(F("  Type 'reset' first"));
    return false;
  }
  
  float avgVoltage = getAverageVoltage();
  if (avgVoltage < MIN_VALID_VOLTAGE) {
    Serial.printf("✗ %s: No voltage\n", channels[channel].name);
    return false;
  }
  
  return true;
}

// ==================== SERIAL COMMANDS ====================
void handleSerialCommands() {
  static char cmdBuffer[MAX_CMD_LENGTH];
  static uint8_t cmdIndex = 0;
  
  while (Serial.available()) {
    char c = Serial.read();
    
    if (c == '\n' || c == '\r') {
      if (cmdIndex > 0) {
        cmdBuffer[cmdIndex] = '\0';
        
        for (uint8_t i = 0; i < cmdIndex; i++) {
          cmdBuffer[i] = tolower(cmdBuffer[i]);
        }
        
        processCommand(cmdBuffer);
        cmdIndex = 0;
      }
    } else if (cmdIndex < MAX_CMD_LENGTH - 1) {
      cmdBuffer[cmdIndex++] = c;
    } else {
      Serial.println(F("✗ Command too long"));
      cmdIndex = 0;
      while (Serial.available()) Serial.read();
    }
  }
}

void processCommand(const char* command) {
  Serial.printf("\n> %s\n", command);

  Serial.println(F("========================================"));
  Serial.printf("Command received: '%s'\n", command);
  Serial.println(F("========================================"));
  
  if (strcmp(command, "reset") == 0) {
    emergencyReset();
  }
  else if (strcmp(command, "restart") == 0) {
    Serial.println(F("Restarting ESP32..."));
    delay(1000);
    ESP.restart();
  }
  else if (strcmp(command, "help") == 0 || strcmp(command, "?") == 0) {
    printMenu();
  }
  else if (strcmp(command, "status") == 0) {
    updateDisplay();
    printStatistics();
  }
  else if (strcmp(command, "diag") == 0 || strcmp(command, "diagnostics") == 0) {
    printDiagnostics();
  }
  else if (strcmp(command, "mem") == 0 || strcmp(command, "memory") == 0) {
    printMemoryUsage();
  }
  else if (strcmp(command, "on 1") == 0) {
    enableChannel(CHANNEL_1);
  }
  else if (strcmp(command, "on 2") == 0) {
    enableChannel(CHANNEL_2);
  }
  else if (strcmp(command, "off 1") == 0) {
    disableChannel(CHANNEL_1);
  }
  else if (strcmp(command, "off 2") == 0) {
    disableChannel(CHANNEL_2);
  }
  else if (strcmp(command, "on all") == 0 || strcmp(command, "on") == 0) {
    enableAllChannels();
  }
  else if (strcmp(command, "off all") == 0 || strcmp(command, "off") == 0) {
    disableAllChannels();
  }
  else if (strcmp(command, "test") == 0) {
    Serial.println(F("\n=== SENSOR TEST ==="));
    for (int i = 0; i < 3; i++) {
      Serial.printf("\nReading %d:\n", i+1);
      
      float ch1V = readRawSensorVoltage(channels[CHANNEL_1].currentSensorPin, ADC_SAMPLES_SLOW);
      float ch2V = readRawSensorVoltage(channels[CHANNEL_2].currentSensorPin, ADC_SAMPLES_SLOW);
      float voltV = readRawSensorVoltage(ZMPT101B_PIN, ADC_SAMPLES_SLOW);
      
      float ch1I = calculateRMSCurrent(CHANNEL_1);
      float ch2I = calculateRMSCurrent(CHANNEL_2);
      float voltage = calculateRMSVoltage(ZMPT101B_PIN);
      
      Serial.printf("  CH1: %.3fV → %.3fA\n", ch1V, ch1I);
      Serial.printf("  CH2: %.3fV → %.3fA\n", ch2V, ch2I);
      Serial.printf("  Voltage: %.3fV → %.1fV\n", voltV, voltage);
      
      delay(500);
      feedWatchdog();
    }
    Serial.printf("\nCalibration:\n");
    Serial.printf("  CH1 offset: %.3fV, cal: %.3f\n", 
                  channels[CHANNEL_1].currentOffset, channels[CHANNEL_1].currentCalibration);
    Serial.printf("  CH2 offset: %.3fV, cal: %.3f\n", 
                  channels[CHANNEL_2].currentOffset, channels[CHANNEL_2].currentCalibration);
    Serial.printf("  Voltage offset: %.3fV, cal: %.3f\n", voltageOffset, voltageCalibration);
    Serial.printf("  Rate: ₱%.2f/kWh\n", calData.electricityRate);
    Serial.println(F("================================\n"));
  }
  else if (strcmp(command, "calibrate") == 0) {
    Serial.println(F("Starting calibration..."));
    currentState = STATE_CALIBRATING;
    calState = CAL_SAMPLING;
    calProgress.samplesCollected = 0;
    calProgress.ch1CurrentSum = 0.0;
    calProgress.ch2CurrentSum = 0.0;
    calProgress.voltageSum = 0.0;
    calProgress.voltageMin = 999.0;
    calProgress.voltageMax = 0.0;
    calProgress.startTime = millis();
  }
  else if (strcmp(command, "cal_voltage") == 0) {
    calibrateVoltageWithReference();
  }
  else if (strcmp(command, "stats") == 0) {
    printStatistics();
  }
  else if (strcmp(command, "clear") == 0) {
    for (int ch = 0; ch < NUM_CHANNELS; ch++) {
      channels[ch].maxCurrent = 0.0;
      channels[ch].maxPower = 0.0;
      channels[ch].energyConsumed = 0.0;
      channels[ch].costAccumulated = 0.0;
      channels[ch].shutdownCount = 0;
    }
    maxVoltage = 0.0;
    minVoltage = 999.0;
    loopCount = 0;
    systemStartTime = millis();
    perfMetrics.maxLoopTime = 0;
    perfMetrics.minLoopTime = 999999;
    Serial.println(F("✓ Statistics cleared"));
  }
  else if (strcmp(command, "manual") == 0) {
    manualControl = !manualControl;
    currentState = manualControl ? STATE_MANUAL_CONTROL : STATE_MONITORING;
    Serial.printf("Manual mode: %s\n", manualControl ? "ON" : "OFF");
  }
  else if (strcmp(command, "safety") == 0) {
    safetyEnabled = !safetyEnabled;
    Serial.printf("Safety: %s\n", safetyEnabled ? "ON" : "OFF");
  }
  else if (strcmp(command, "buzzer") == 0) {
    buzzerEnabled = !buzzerEnabled;
    Serial.printf("Buzzer: %s\n", buzzerEnabled ? "ON" : "OFF");
  }
  else if (strcmp(command, "buzzer_invert") == 0) {
    buzzerInverted = !buzzerInverted;
    Serial.printf("Buzzer Polarity: %s (set to %s if buzzer beeps continuously)\n",
                  buzzerInverted ? "INVERTED" : "NORMAL",
                  buzzerInverted ? "INVERTED" : "NORMAL");
    // Test the new setting with a quick beep
    if (buzzerEnabled) {
      setBuzzer(true);
      delay(100);
      setBuzzer(false);
    }
  }
  else if (strcmp(command, "display") == 0) {
    displayEnabled = !displayEnabled;
    Serial.printf("Display: %s\n", displayEnabled ? "ON" : "OFF");
    if (displayEnabled && !tftInitialized) {
      initTFTDisplay();
    }
  }
  else if (strcmp(command, "display_test") == 0) {
    Serial.println(F("Testing display..."));
    tft.fillScreen(ST77XX_RED);
    delay(500);
    tft.fillScreen(ST77XX_GREEN);
    delay(500);
    tft.fillScreen(ST77XX_BLUE);
    delay(500);
    tft.fillScreen(ST77XX_BLACK);
    tft.setTextColor(ST77XX_WHITE);
    tft.setTextSize(3);
    tft.setCursor(50, 100);
    tft.println("HELLO!");
    Serial.println(F("✓ Display test complete"));
  }
  else if (strcmp(command, "lcd_test") == 0) {
    Serial.println(F("Testing LCD display..."));
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("LCD TEST");
    lcd.setCursor(0, 1);
    lcd.print("1234567890ABCDEF");
    if (LCD_ROWS > 2) {
      lcd.setCursor(0, 2);
      lcd.print("Line 3 Test");
      lcd.setCursor(0, 3);
      lcd.print("Line 4 Test");
    }
    Serial.println(F("✓ LCD test pattern displayed"));
  }
  else if (strncmp(command, "rate ", 5) == 0) {
    float rate = atof(command + 5);
    if (rate >= 1.0 && rate <= 100.0) {
      calData.electricityRate = rate;
      saveCalibrationData();
      Serial.printf("✓ Rate: ₱%.2f/kWh\n", rate);
      updateCostCalculation();
    } else {
      Serial.println(F("✗ Invalid (1.0-100.0)"));
    }
  }
  else if (strncmp(command, "power_factor ", 13) == 0) {
    float pf = atof(command + 13);
    if (pf >= 0.1 && pf <= 1.0) {
      calData.powerFactor = pf;
      saveCalibrationData();
      Serial.printf("✓ Power factor: %.2f\n", pf);
    } else {
      Serial.println(F("✗ Invalid (0.1-1.0)"));
    }
  }
  else if (strncmp(command, "ch1_cal ", 8) == 0) {
    float cal = atof(command + 8);
    if (cal > 0.001 && cal < 100.0) {
      calData.ch1CurrentCalibration = cal;
      channels[CHANNEL_1].currentCalibration = cal;
      saveCalibrationData();
      Serial.printf("✓ CH1 cal: %.3f\n", cal);
    } else {
      Serial.println(F("✗ Invalid"));
    }
  }
  else if (strncmp(command, "ch2_cal ", 8) == 0) {
    float cal = atof(command + 8);
    if (cal > 0.001 && cal < 100.0) {
      calData.ch2CurrentCalibration = cal;
      channels[CHANNEL_2].currentCalibration = cal;
      saveCalibrationData();
      Serial.printf("✓ CH2 cal: %.3f\n", cal);
    } else {
      Serial.println(F("✗ Invalid"));
    }
  }
  else if (strncmp(command, "voltage_cal ", 12) == 0) {
    float cal = atof(command + 12);
    if (cal > 0.01 && cal < 10000.0) {
      calData.voltageCalibration = cal;
      voltageCalibration = cal;
      saveCalibrationData();
      Serial.printf("✓ Voltage cal: %.3f\n", cal);
    } else {
      Serial.println(F("✗ Invalid"));
    }
  }
  else if (strncmp(command, "brightness ", 11) == 0) {
    int brightness = atoi(command + 11);
    if (brightness >= 0 && brightness <= 255) {
      setTFTBrightness(brightness);
      Serial.printf("✓ Brightness: %d\n", brightness);
    } else {
      Serial.println(F("✗ Invalid (0-255)"));
    }
  }
    else if (strcmp(command, "raw_test") == 0) {
    testRawDisplayCommands();
  }
  else if (strcmp(command, "alt_test1") == 0) {
    testAlternativeInit1();
  }
  else if (strcmp(command, "alt_test2") == 0) {
    testAlternativeInit2();
  }
  else if (strcmp(command, "try_modes") == 0) {
    Serial.println(F("\n=== TESTING ALL SPI MODES ==="));
    
    for (int mode = 0; mode <= 3; mode++) {
      Serial.printf("\n--- Testing SPI_MODE%d ---\n", mode);
      
      pinMode(TFT_RST, OUTPUT);
      pinMode(TFT_DC, OUTPUT);
      pinMode(TFT_BL_PIN, OUTPUT);
      digitalWrite(TFT_BL_PIN, HIGH);
      
      digitalWrite(TFT_RST, LOW);
      delay(20);
      digitalWrite(TFT_RST, HIGH);
      delay(150);
      
      SPI.begin(TFT_SCLK, -1, TFT_MOSI, -1);
      SPI.setFrequency(10000000);
      SPI.setDataMode(mode);
      
      auto writeCommand = [](uint8_t cmd) {
        digitalWrite(TFT_DC, LOW);
        SPI.transfer(cmd);
      };
      
      auto writeData = [](uint8_t data) {
        digitalWrite(TFT_DC, HIGH);
        SPI.transfer(data);
      };
      
      // Quick init
      writeCommand(0x01); delay(150);
      writeCommand(0x11); delay(120);
      writeCommand(0x3A); writeData(0x55);
      writeCommand(0x36); writeData(0x00);
      writeCommand(0x21);
      writeCommand(0x29); delay(50);
      
      // Fill screen
      writeCommand(0x2A);
      writeData(0x00); writeData(0x00);
      writeData(0x00); writeData(0xEF);
      
      writeCommand(0x2B);
      writeData(0x00); writeData(0x00);
      writeData(0x00); writeData(0xEF);
      
      writeCommand(0x2C);
      digitalWrite(TFT_DC, HIGH);
      
      // Fill with color based on mode
      uint16_t color = 0xF800;  // RED for mode 0
      if (mode == 1) color = 0x07E0;  // GREEN
      if (mode == 2) color = 0x001F;  // BLUE
      if (mode == 3) color = 0xFFFF;  // WHITE
      
      for (int i = 0; i < 240 * 240; i++) {
        SPI.transfer16(color);
        if (i % 20000 == 0) feedWatchdog();
      }
      
      Serial.printf("MODE%d: Should see %s\n", mode, 
                    mode == 0 ? "RED" : 
                    mode == 1 ? "GREEN" : 
                    mode == 2 ? "BLUE" : "WHITE");
      
      delay(3000);
    }
    Serial.println("\n=== Which mode showed colors? ===");
  }
  else if (strcmp(command, "check_display") == 0) {
    Serial.println(F("\n=== DISPLAY HARDWARE CHECK ==="));
    
    pinMode(TFT_RST, INPUT_PULLUP);
    pinMode(TFT_DC, INPUT_PULLUP);
    pinMode(TFT_MOSI, INPUT_PULLUP);
    pinMode(TFT_SCLK, INPUT_PULLUP);
    pinMode(TFT_BL_PIN, INPUT_PULLUP);
    
    delay(100);
    
    Serial.println("\nPin states (should be HIGH with pullup):");
    Serial.printf("  RST  (GPIO %d): %s\n", TFT_RST, digitalRead(TFT_RST) ? "HIGH" : "LOW");
    Serial.printf("  DC   (GPIO %d): %s\n", TFT_DC, digitalRead(TFT_DC) ? "HIGH" : "LOW");
    Serial.printf("  MOSI (GPIO %d): %s\n", TFT_MOSI, digitalRead(TFT_MOSI) ? "HIGH" : "LOW");
    Serial.printf("  SCLK (GPIO %d): %s\n", TFT_SCLK, digitalRead(TFT_SCLK) ? "HIGH" : "LOW");
    Serial.printf("  BL   (GPIO %d): %s\n", TFT_BL_PIN, digitalRead(TFT_BL_PIN) ? "HIGH" : "LOW");
    
    Serial.println("\nIf any show LOW, check for:");
    Serial.println("  - Short circuit to ground");
    Serial.println("  - Bad solder joint");
    Serial.println("  - Incorrect pin connection");
    
    // Read display VCC if connected to ADC
    Serial.println("\nDisplay power check:");
    Serial.println("  Use multimeter: VCC should be 3.3V");
    Serial.println("  Use multimeter: GND should be 0V");
    Serial.println("  Use multimeter: BLK should be 3.3V (backlight)");
  }
  else {
    Serial.println(F("✗ Unknown command"));
    Serial.println(F("Type 'help' for commands"));
  }
}

// ==================== MENU ====================
void printMenu() {
  Serial.println(F("\n========================================"));
  Serial.println(F("    DUAL CHANNEL COMMAND MENU v5.3"));
  Serial.println(F("========================================"));
  Serial.println(F("EMERGENCY:"));
  Serial.println(F("  reset      - Emergency reset"));
  Serial.println(F("  restart    - Restart ESP32"));
  Serial.println();
  Serial.println(F("BASIC:"));
  Serial.println(F("  help       - Show menu"));
  Serial.println(F("  status     - Show status"));
  Serial.println(F("  test       - Test sensors"));
  Serial.println();
  Serial.println(F("CONTROL:"));
  Serial.println(F("  on 1/2     - Enable channel"));
  Serial.println(F("  on all     - Enable both"));
  Serial.println(F("  off 1/2    - Disable channel"));
  Serial.println(F("  off all    - Disable both"));
  Serial.println();
  Serial.println(F("DIAGNOSTICS:"));
  Serial.println(F("  diag       - Full diagnostics"));
  Serial.println(F("  mem        - Memory usage"));
  Serial.println(F("  stats      - Statistics"));
  Serial.println();
  Serial.println(F("CALIBRATION:"));
  Serial.println(F("  calibrate  - Auto-calibrate"));
  Serial.println(F("  cal_voltage- Voltage wizard"));
  Serial.println();
  Serial.println(F("SETTINGS:"));
  Serial.println(F("  manual     - Toggle manual"));
  Serial.println(F("  safety     - Toggle safety"));
  Serial.println(F("  buzzer     - Toggle buzzer"));
  Serial.println(F("  buzzer_invert - Toggle buzzer polarity (fix continuous beeping)"));
  Serial.println(F("  display    - Toggle TFT"));
  Serial.println(F("  display_test - Test display"));
  Serial.println(F("  rate X     - Set ₱/kWh"));
  Serial.println(F("  clear      - Clear stats"));
  Serial.println(F("========================================\n"));
}

void printStatistics() {
  Serial.println(F("\n=== STATISTICS ==="));
  unsigned long uptime = (millis() - systemStartTime) / 1000;
  Serial.printf("Uptime: %02lu:%02lu:%02lu\n", uptime/3600, (uptime%3600)/60, uptime%60);
  Serial.printf("Loop count: %lu\n", loopCount);
  Serial.printf("Voltage: Max %.1fV, Min %.1fV\n", maxVoltage, minVoltage);
  Serial.printf("Rate: ₱%.2f/kWh\n", calData.electricityRate);
  Serial.println(F("--------------------------------"));
  
  float totalEnergy = 0;
  float totalCost = calculateTotalCost();
  unsigned long totalShutdowns = 0;
  
  for (int ch = 0; ch < NUM_CHANNELS; ch++) {
    Serial.printf("\n%s:\n", channels[ch].name);
    Serial.printf("  Status: %s\n", channels[ch].relayEnabled ? "ON" : "OFF");
    Serial.printf("  Energy: %.3f Wh (%.2f kWh)\n", 
                  channels[ch].energyConsumed, channels[ch].energyConsumed / 1000.0);
    Serial.printf("  Cost: ₱%.2f\n", channels[ch].costAccumulated);
    Serial.printf("  Max Current: %.3f A\n", channels[ch].maxCurrent);
    Serial.printf("  Max Power: %.1f W\n", channels[ch].maxPower);
    Serial.printf("  Shutdowns: %lu\n", channels[ch].shutdownCount);
    
    totalEnergy += channels[ch].energyConsumed;
    totalShutdowns += channels[ch].shutdownCount;
  }
  
  Serial.println(F("\n--------------------------------"));
  Serial.printf("TOTAL Energy: %.3f Wh (%.2f kWh)\n", totalEnergy, totalEnergy / 1000.0);
  Serial.printf("TOTAL Cost: ₱%.2f\n", totalCost);
  Serial.printf("TOTAL Shutdowns: %lu\n", totalShutdowns);
  Serial.printf("Sensors: %s\n", sensorsValid ? "Valid" : "Invalid");
  Serial.printf("TFT: %s\n", tftInitialized ? "Active" : "Inactive");
  Serial.println(F("================================\n"));
}

// ==================== DIAGNOSTICS ====================
void printDiagnostics() {
  Serial.println(F("\n========================================"));
  Serial.println(F("   SYSTEM DIAGNOSTICS"));
  Serial.println(F("========================================"));
  
  Serial.println(F("\n--- SYSTEM INFO ---"));
  Serial.printf("Version: v5.3 ADAFRUIT\n");
  Serial.printf("Chip: %s Rev %d\n", ESP.getChipModel(), ESP.getChipRevision());
  Serial.printf("CPU: %d MHz\n", ESP.getCpuFreqMHz());
  Serial.printf("Flash: %d bytes\n", ESP.getFlashChipSize());
  
  printMemoryUsage();
  
  Serial.println(F("\n--- HARDWARE CONFIG ---"));
  Serial.printf("Display: Adafruit ST7789 (%s)\n", tftInitialized ? "OK" : "OFF");
  Serial.printf("Voltage Divider: %.3f\n", VOLTAGE_DIVIDER_RATIO);
  Serial.printf("Expected Offset: %.2fV\n", ACS712_ZERO_CURRENT * VOLTAGE_DIVIDER_RATIO);
  Serial.println(F("✅ Voltage divider installed (10kΩ+20kΩ)"));
  
  Serial.println(F("\n--- SENSOR STATUS ---"));
  Serial.printf("Sensors Valid: %s\n", sensorsValid ? "YES" : "NO");
  Serial.printf("Voltage: %.1fV (Avg: %.1fV)\n", voltageReading, getAverageVoltage());
  
  for (int ch = 0; ch < NUM_CHANNELS; ch++) {
    Serial.printf("\n%s:\n", channels[ch].name);
    Serial.printf("  Current: %.3fA (Avg: %.3fA)\n", 
                  channels[ch].currentReading, getAverageCurrent(ch));
    Serial.printf("  Power: %.1fW\n", channels[ch].powerReading);
    Serial.printf("  Cost/hr: ₱%.2f\n", calculateCostPerHour(channels[ch].powerReading));
    Serial.printf("  Fault: %s\n", channels[ch].sensorFaultDetected ? "YES" : "NO");
  }
  
  Serial.println(F("\n--- CALIBRATION ---"));
  Serial.printf("CH1: offset %.3fV, cal %.3f\n", calData.ch1CurrentOffset, calData.ch1CurrentCalibration);
  Serial.printf("CH2: offset %.3fV, cal %.3f\n", calData.ch2CurrentOffset, calData.ch2CurrentCalibration);
  Serial.printf("Voltage: offset %.3fV, cal %.3f\n", calData.voltageOffset, calData.voltageCalibration);
  Serial.printf("Power Factor: %.2f\n", calData.powerFactor);
  Serial.printf("Rate: ₱%.2f/kWh\n", calData.electricityRate);
  
  Serial.println(F("\n--- SAFETY ---"));
  Serial.printf("Safety: %s\n", safetyEnabled ? "ON" : "OFF");
  Serial.printf("Manual: %s\n", manualControl ? "YES" : "NO");
  Serial.printf("Overvoltage: %s\n", overvoltageDetected ? "DETECTED" : "OK");
  Serial.printf("Undervoltage: %s\n", undervoltageDetected ? "DETECTED" : "OK");
  Serial.printf("Brownout: %s\n", brownoutDetected ? "DETECTED" : "OK");
  
  for (int ch = 0; ch < NUM_CHANNELS; ch++) {
    Serial.printf("%s Overcurrent: %s\n", channels[ch].name, 
                  channels[ch].overcurrentDetected ? "DETECTED" : "OK");
  }
  
  Serial.println(F("\n--- RELAYS ---"));
  for (int ch = 0; ch < NUM_CHANNELS; ch++) {
    Serial.printf("%s: %s (Command: %s, Pin: %s)\n", 
                  channels[ch].name,
                  channels[ch].relayEnabled ? "ON" : "OFF",
                  channels[ch].relayCommandState ? "ON" : "OFF",
                  digitalRead(channels[ch].relayPin) == RELAY_ON_STATE ? "ON" : "OFF");
  }
  
  Serial.println(F("\n--- PERFORMANCE ---"));
  if (perfMetrics.avgLoopTime > 0) {
    Serial.printf("Avg: %lu ms\n", perfMetrics.avgLoopTime);
    Serial.printf("Min: %lu ms\n", perfMetrics.minLoopTime);
    Serial.printf("Max: %lu ms\n", perfMetrics.maxLoopTime);
    Serial.printf("Loops/sec: %.1f\n", 1000.0 / perfMetrics.avgLoopTime);
  }
  
  Serial.println(F("\n--- PERIPHERALS ---"));
  Serial.printf("SD Card: %s\n", sdCardAvailable ? "Yes" : "No");
  Serial.printf("Buzzer: %s\n", buzzerEnabled ? "Enabled" : "Disabled");
  Serial.printf("TFT: %s\n", tftInitialized ? "Active" : "Inactive");
  
  Serial.println(F("\n========================================\n"));

  Serial.println(F("\n--- NETWORK & SERVER ---"));
  Serial.printf("WiFi Connected: %s\n", wifiConnected ? "YES" : "NO");
  if (wifiConnected) {
    Serial.print(F("IP Address: "));
    Serial.println(WiFi.localIP());
    Serial.printf("Signal Strength: %d dBm\n", WiFi.RSSI());
  }
  Serial.printf("Server Connected: %s\n", serverConnected ? "YES" : "NO");
  Serial.printf("Server Host: %s\n", SERVER_HOST);
  Serial.printf("Server Port: %d (HTTPS)\n", SERVER_PORT);
  Serial.printf("Device ID: %s\n", DEVICE_ID);
  Serial.printf("Device Type: %s\n", DEVICE_TYPE);
  
  // Server statistics
  int totalRequests = serverSuccessCount + serverErrorCount;
  float successRate = totalRequests > 0 ? 
                      (float)serverSuccessCount / totalRequests * 100.0 : 0;
  
  Serial.println(F("\nServer Statistics:"));
  Serial.printf("  Successful sends: %d\n", serverSuccessCount);
  Serial.printf("  Failed sends: %d\n", serverErrorCount);
  Serial.printf("  Total requests: %d\n", totalRequests);
  Serial.printf("  Success rate: %.1f%%\n", successRate);
  
  Serial.println(F("\n========================================\n"));
}

// ==================== MEMORY ====================
void printMemoryUsage() {
  Serial.println(F("\n--- MEMORY ---"));
  Serial.printf("Free Heap: %d bytes\n", ESP.getFreeHeap());
  Serial.printf("Heap Size: %d bytes\n", ESP.getHeapSize());
  Serial.printf("Min Free: %d bytes\n", ESP.getMinFreeHeap());
  Serial.printf("Max Alloc: %d bytes\n", ESP.getMaxAllocHeap());
  
  float usage = 100.0 * (1.0 - (float)ESP.getFreeHeap() / ESP.getHeapSize());
  Serial.printf("Usage: %.1f%%\n", usage);
  
  if (usage > 80.0) {
    Serial.println(F("⚠️  High memory usage!"));
  }
}

// ==================== CRC32 ====================
uint32_t calculateCRC32(const uint8_t* data, size_t length) {
  uint32_t crc = 0xFFFFFFFF;
  
  for (size_t i = 0; i < length; i++) {
    crc ^= data[i];
    for (int j = 0; j < 8; j++) {
      crc = (crc >> 1) ^ (0xEDB88320 & -(crc & 1));
    }
  }
  
  return ~crc;
}

// ==================== EEPROM ====================
void saveCalibrationData() {
  calData.magic = EEPROM_MAGIC;
  
  calData.ch1CurrentOffset = channels[CHANNEL_1].currentOffset;
  calData.ch2CurrentOffset = channels[CHANNEL_2].currentOffset;
  calData.ch1CurrentCalibration = channels[CHANNEL_1].currentCalibration;
  calData.ch2CurrentCalibration = channels[CHANNEL_2].currentCalibration;
  calData.voltageOffset = voltageOffset;
  calData.voltageCalibration = voltageCalibration;
  
  calData.crc = calculateCRC32((uint8_t*)&calData, sizeof(CalibrationData) - sizeof(uint32_t));
  
  EEPROM.put(0, calData);
  EEPROM.commit();
  
  Serial.println(F("✓ Saved calibration to EEPROM"));
}

bool loadCalibrationData() {
  CalibrationData loaded;
  EEPROM.get(0, loaded);
  
  if (loaded.magic != EEPROM_MAGIC) {
    Serial.println(F("⚠️  Invalid EEPROM magic"));
    return false;
  }
  
  uint32_t crc = calculateCRC32((uint8_t*)&loaded, sizeof(CalibrationData) - sizeof(uint32_t));
  if (crc != loaded.crc) {
    Serial.println(F("⚠️  EEPROM CRC mismatch"));
    return false;
  }
  
  if (isnan(loaded.ch1CurrentOffset) || loaded.ch1CurrentOffset < 1.0 || loaded.ch1CurrentOffset > 2.5) {
    return false;
  }
  
  if (isnan(loaded.ch2CurrentOffset) || loaded.ch2CurrentOffset < 1.0 || loaded.ch2CurrentOffset > 2.5) {
    return false;
  }
  
  calData = loaded;
  
  channels[CHANNEL_1].currentOffset = calData.ch1CurrentOffset;
  channels[CHANNEL_2].currentOffset = calData.ch2CurrentOffset;
  channels[CHANNEL_1].currentCalibration = calData.ch1CurrentCalibration;
  channels[CHANNEL_2].currentCalibration = calData.ch2CurrentCalibration;
  voltageOffset = calData.voltageOffset;
  voltageCalibration = calData.voltageCalibration;
  
  Serial.println(F("✓ Loaded calibration from EEPROM"));
  Serial.printf("  Rate: ₱%.2f/kWh\n", calData.electricityRate);
  return true;
}

// ==================== SD CARD ====================
void initSDCard() {
  Serial.print(F("Initializing SD card... "));
  
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println(F("FAILED"));
    sdCardAvailable = false;
    return;
  }
  
  sdCardAvailable = true;
  Serial.println(F("✓ OK"));
  
  if (!SD.exists("/datalog.csv")) {
    File file = SD.open("/datalog.csv", FILE_WRITE);
    if (file) {
      file.println("Timestamp,Voltage,CH1_Current,CH1_Power,CH1_State,CH1_Cost,CH2_Current,CH2_Power,CH2_State,CH2_Cost,Total_Cost");
      file.close();
      Serial.println(F("  Created datalog.csv"));
    }
  }
}

void logToSD(const char* data) {
  if (!sdCardAvailable) return;
  
  File file = SD.open("/datalog.csv", FILE_APPEND);
  if (!file) {
    return;
  }
  
  file.println(data);
  file.close();
}

// ==================== BUZZER TEST ====================
void testBuzzer() {
  if (!buzzerEnabled) {
    Serial.println(F("Buzzer disabled"));
    return;
  }
  Serial.println(F("Testing buzzer..."));
  for (int i = 0; i < 3; i++) {
    setBuzzer(true);
    delay(200);
    setBuzzer(false);
    delay(200);
    feedWatchdog();
  }
  Serial.println(F("✓ Buzzer OK"));
}

// ==================== RAW DISPLAY TEST ====================
// Add this function anywhere in main.cpp (before setup)

void testRawDisplayCommands() {
  Serial.println(F("\n=== RAW DISPLAY TEST ==="));
  
  // Setup pins
  pinMode(TFT_RST, OUTPUT);
  pinMode(TFT_DC, OUTPUT);
  pinMode(TFT_BL_PIN, OUTPUT);
  
  // Backlight ON
  digitalWrite(TFT_BL_PIN, HIGH);
  Serial.println("Backlight ON");
  
  // Hardware reset
  digitalWrite(TFT_RST, LOW);
  delay(20);
  digitalWrite(TFT_RST, HIGH);
  delay(150);
  Serial.println("Reset complete");
  
  // Initialize SPI
  SPI.begin(TFT_SCLK, -1, TFT_MOSI, -1);
  SPI.setFrequency(10000000);  // 10MHz - safe speed
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);
  Serial.println("SPI initialized at 10MHz");
  
  // Helper functions for sending commands/data
  auto writeCommand = [](uint8_t cmd) {
    digitalWrite(TFT_DC, LOW);  // Command mode
    SPI.transfer(cmd);
  };
  
  auto writeData = [](uint8_t data) {
    digitalWrite(TFT_DC, HIGH);  // Data mode
    SPI.transfer(data);
  };
  
  auto writeData16 = [&writeData](uint16_t data) {
    writeData(data >> 8);
    writeData(data & 0xFF);
  };
  
  // ST7789 Initialization Sequence
  Serial.println("\n--- Sending ST7789 Init Commands ---");
  
  // Software Reset
  writeCommand(0x01);
  delay(150);
  Serial.println("1. Software reset");
  
  // Sleep Out
  writeCommand(0x11);
  delay(120);
  Serial.println("2. Sleep out");
  
  // Color Mode - 16bit
  writeCommand(0x3A);
  writeData(0x55);  // 16-bit/pixel
  Serial.println("3. Color mode: 16-bit");
  
  // Memory Data Access Control
  writeCommand(0x36);
  writeData(0x00);  // Try different values: 0x00, 0x60, 0xA0, 0xC0
  Serial.println("4. Memory access control");
  
  // Inversion ON (some displays need this)
  writeCommand(0x21);
  Serial.println("5. Inversion ON");
  
  // Normal Display Mode
  writeCommand(0x13);
  Serial.println("6. Normal display mode");
  
  // Display ON
  writeCommand(0x29);
  delay(50);
  Serial.println("7. Display ON");
  
  Serial.println("\n--- Testing Fill Screen RED ---");
  
  // Set column address (0 to 239)
  writeCommand(0x2A);
  writeData(0x00);
  writeData(0x00);
  writeData(0x00);
  writeData(0xEF);  // 239
  Serial.println("Column address set: 0-239");
  
  // Set row address (0 to 239)
  writeCommand(0x2B);
  writeData(0x00);
  writeData(0x00);
  writeData(0x00);
  writeData(0xEF);  // 239
  Serial.println("Row address set: 0-239");
  
  // Write to RAM
  writeCommand(0x2C);
  digitalWrite(TFT_DC, HIGH);  // Data mode
  
  Serial.println("Filling screen RED (RGB565: 0xF800)...");
  
  // Fill entire screen with RED
  for (int i = 0; i < 240 * 240; i++) {
    SPI.transfer16(0xF800);  // RED in RGB565
    if (i % 10000 == 0) {
      Serial.print(".");
      feedWatchdog();
    }
  }
  Serial.println(" Done!");
  
  delay(2000);
  
  // Try GREEN
  Serial.println("\n--- Testing Fill Screen GREEN ---");
  writeCommand(0x2C);
  digitalWrite(TFT_DC, HIGH);
  
  for (int i = 0; i < 240 * 240; i++) {
    SPI.transfer16(0x07E0);  // GREEN
    if (i % 10000 == 0) {
      Serial.print(".");
      feedWatchdog();
    }
  }
  Serial.println(" Done!");
  
  delay(2000);
  
  // Try BLUE
  Serial.println("\n--- Testing Fill Screen BLUE ---");
  writeCommand(0x2C);
  digitalWrite(TFT_DC, HIGH);
  
  for (int i = 0; i < 240 * 240; i++) {
    SPI.transfer16(0x001F);  // BLUE
    if (i % 10000 == 0) {
      Serial.print(".");
      feedWatchdog();
    }
  }
  Serial.println(" Done!");
  
  delay(2000);
  
  // Try WHITE
  Serial.println("\n--- Testing Fill Screen WHITE ---");
  writeCommand(0x2C);
  digitalWrite(TFT_DC, HIGH);
  
  for (int i = 0; i < 240 * 240; i++) {
    SPI.transfer16(0xFFFF);  // WHITE
    if (i % 10000 == 0) {
      Serial.print(".");
      feedWatchdog();
    }
  }
  Serial.println(" Done!");
  
  Serial.println("\n=== RAW TEST COMPLETE ===");
  Serial.println("Did you see colors change?");
  Serial.println("If YES: Library configuration issue");
  Serial.println("If NO: Try alternative init sequence");
}

// ==================== ALTERNATIVE INIT SEQUENCES ====================

void testAlternativeInit1() {
  Serial.println(F("\n=== TESTING ALT INIT #1 (GC9A01) ==="));
  
  pinMode(TFT_RST, OUTPUT);
  pinMode(TFT_DC, OUTPUT);
  pinMode(TFT_BL_PIN, OUTPUT);
  digitalWrite(TFT_BL_PIN, HIGH);
  
  // Reset
  digitalWrite(TFT_RST, LOW);
  delay(20);
  digitalWrite(TFT_RST, HIGH);
  delay(150);
  
  SPI.begin(TFT_SCLK, -1, TFT_MOSI, -1);
  SPI.setFrequency(10000000);
  SPI.setDataMode(SPI_MODE0);
  
  auto writeCommand = [](uint8_t cmd) {
    digitalWrite(TFT_DC, LOW);
    SPI.transfer(cmd);
  };
  
  auto writeData = [](uint8_t data) {
    digitalWrite(TFT_DC, HIGH);
    SPI.transfer(data);
  };
  
  // GC9A01 init sequence
  writeCommand(0xEF);
  writeCommand(0xEB);
  writeData(0x14);
  
  writeCommand(0xFE);
  writeCommand(0xEF);
  
  writeCommand(0xEB);
  writeData(0x14);
  
  writeCommand(0x84);
  writeData(0x40);
  
  writeCommand(0x85);
  writeData(0xFF);
  
  writeCommand(0x86);
  writeData(0xFF);
  
  writeCommand(0x87);
  writeData(0xFF);
  
  writeCommand(0x88);
  writeData(0x0A);
  
  writeCommand(0x89);
  writeData(0x21);
  
  writeCommand(0x8A);
  writeData(0x00);
  
  writeCommand(0x8B);
  writeData(0x80);
  
  writeCommand(0x8C);
  writeData(0x01);
  
  writeCommand(0x8D);
  writeData(0x01);
  
  writeCommand(0x8E);
  writeData(0xFF);
  
  writeCommand(0x8F);
  writeData(0xFF);
  
  writeCommand(0xB6);
  writeData(0x00);
  writeData(0x20);
  
  writeCommand(0x36);
  writeData(0x08);
  
  writeCommand(0x3A);
  writeData(0x05);
  
  writeCommand(0x90);
  writeData(0x08);
  writeData(0x08);
  writeData(0x08);
  writeData(0x08);
  
  writeCommand(0xBD);
  writeData(0x06);
  
  writeCommand(0xBC);
  writeData(0x00);
  
  writeCommand(0xFF);
  writeData(0x60);
  writeData(0x01);
  writeData(0x04);
  
  writeCommand(0xC3);
  writeData(0x13);
  
  writeCommand(0xC4);
  writeData(0x13);
  
  writeCommand(0xC9);
  writeData(0x22);
  
  writeCommand(0xBE);
  writeData(0x11);
  
  writeCommand(0xE1);
  writeData(0x10);
  writeData(0x0E);
  
  writeCommand(0xDF);
  writeData(0x21);
  writeData(0x0c);
  writeData(0x02);
  
  writeCommand(0x11);
  delay(120);
  
  writeCommand(0x29);
  delay(20);
  
  Serial.println("GC9A01 init complete");
  
  // Test fill
  writeCommand(0x2A);
  writeData(0x00); writeData(0x00);
  writeData(0x00); writeData(0xEF);
  
  writeCommand(0x2B);
  writeData(0x00); writeData(0x00);
  writeData(0x00); writeData(0xEF);
  
  writeCommand(0x2C);
  digitalWrite(TFT_DC, HIGH);
  
  for (int i = 0; i < 240 * 240; i++) {
    SPI.transfer16(0xF800);  // RED
  }
  
  Serial.println("Fill complete - see red?");
}

void testAlternativeInit2() {
  Serial.println(F("\n=== TESTING ALT INIT #2 (Different MADCTL) ==="));
  
  pinMode(TFT_RST, OUTPUT);
  pinMode(TFT_DC, OUTPUT);
  pinMode(TFT_BL_PIN, OUTPUT);
  digitalWrite(TFT_BL_PIN, HIGH);
  
  digitalWrite(TFT_RST, LOW);
  delay(20);
  digitalWrite(TFT_RST, HIGH);
  delay(150);
  
  SPI.begin(TFT_SCLK, -1, TFT_MOSI, -1);
  SPI.setFrequency(10000000);
  SPI.setDataMode(SPI_MODE0);
  
  auto writeCommand = [](uint8_t cmd) {
    digitalWrite(TFT_DC, LOW);
    SPI.transfer(cmd);
  };
  
  auto writeData = [](uint8_t data) {
    digitalWrite(TFT_DC, HIGH);
    SPI.transfer(data);
  };
  
  // Basic ST7789 with different MADCTL values
  writeCommand(0x01);  // Software reset
  delay(150);
  
  writeCommand(0x11);  // Sleep out
  delay(120);
  
  writeCommand(0x36);  // Memory access control
  writeData(0xA0);     // Try: 0x00, 0x60, 0xA0, 0xC0, 0x08, 0x68
  
  writeCommand(0x3A);  // Color mode
  writeData(0x55);     // 16-bit
  
  writeCommand(0x20);  // Inversion OFF (try 0x21 for ON)
  
  writeCommand(0x13);  // Normal display
  
  writeCommand(0x29);  // Display ON
  delay(50);
  
  Serial.println("Init complete with MADCTL=0xA0");
  
  // Test fill
  writeCommand(0x2A);
  writeData(0x00); writeData(0x00);
  writeData(0x00); writeData(0xEF);
  
  writeCommand(0x2B);
  writeData(0x00); writeData(0x00);
  writeData(0x00); writeData(0xEF);
  
  writeCommand(0x2C);
  digitalWrite(TFT_DC, HIGH);
  
  for (int i = 0; i < 240 * 240; i++) {
    SPI.transfer16(0x07E0);  // GREEN
  }
  
  Serial.println("Fill complete - see green?");
}
// ==================== NETWORK FUNCTIONS ====================

void initWiFi() {
  Serial.println(F("\n--- WiFi Initialization ---"));
  Serial.print(F("SSID: "));
  Serial.println(WIFI_SSID);
  Serial.print(F("Device ID: "));
  Serial.println(DEVICE_ID);
  Serial.print(F("Device Type: "));
  Serial.println(DEVICE_TYPE);
  
  if (lcdInitialized) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Connecting WiFi...");
  }
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
  unsigned long startAttempt = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startAttempt < WIFI_CONNECT_TIMEOUT) {
    delay(500);
    Serial.print(".");
    feedWatchdog();
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    wifiConnected = true;
    Serial.println(F("\nWiFi Connected!"));
    Serial.print(F("IP Address: "));
    Serial.println(WiFi.localIP());
    Serial.print(F("Signal: "));
    Serial.print(WiFi.RSSI());
    Serial.println(F(" dBm"));
    
    // ✅ CRITICAL: Configure SSL/TLS for HTTPS
    secureClient.setInsecure();  // Accept any certificate (for testing)
    // For production: secureClient.setCACert(root_ca);
    
    Serial.println(F(" SSL/TLS configured for HTTPS"));
    Serial.print(F("Server: https://"));
    Serial.println(SERVER_HOST);
    
    if (lcdInitialized) {
      lcd.setCursor(0, 1);
      lcd.print("WiFi: Connected");
      lcd.setCursor(0, 2);
      lcd.print(WiFi.localIP());
    }
    
    // ✅ FIXED: WebSocket with correct host (domain only, no https://)
    Serial.println(F("Initializing WebSocket..."));
    webSocket.beginSSL(SERVER_HOST, SERVER_PORT, "/ws"); // SSL WebSocket for HTTPS
    webSocket.onEvent(webSocketEvent);
    webSocket.setReconnectInterval(WEBSOCKET_RECONNECT_INT);
    
    delay(2000);
  } else {
    wifiConnected = false;
    Serial.println(F("\nâœ— WiFi Connection Failed!"));
    Serial.println(F("Continuing in offline mode"));
    
    if (lcdInitialized) {
      lcd.setCursor(0, 1);
      lcd.print("WiFi: Failed");
      lcd.setCursor(0, 2);
      lcd.print("Offline Mode");
    }
    
    delay(2000);
  }
}

void registerWithServer() {
     if (!webSocket.isConnected()) return;
     
     StaticJsonDocument<128> doc;
     doc["type"] = "register";
     doc["deviceId"] = DEVICE_ID;
     doc["deviceType"] = DEVICE_TYPE;
     
     String message;
     serializeJson(doc, message);
     
     webSocket.sendTXT(message);
     Serial.println(F("✓ Registration message sent"));
   }

void sendCommandAck(const char* command, bool success) {
     if (!webSocket.isConnected()) return;
     
     StaticJsonDocument<128> doc;
     doc["type"] = "commandAck";
     doc["deviceId"] = DEVICE_ID;
     doc["command"] = command;
     doc["success"] = success;
     
     String message;
     serializeJson(doc, message);
     
     webSocket.sendTXT(message);
     Serial.printf("✓ Command acknowledgment sent: %s (success=%d)\n", command, success);
   }

void checkWiFiConnection() {
  if (WiFi.status() != WL_CONNECTED) {
    if (wifiConnected) {
      Serial.println(F("✗ WiFi Disconnected!"));
      wifiConnected = false;
      serverConnected = false;
    }
    
    if (millis() - lastWiFiCheck >= WIFI_RECONNECT_INTERVAL) {
      Serial.println(F("Reconnecting WiFi..."));
      WiFi.disconnect();
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
      lastWiFiCheck = millis();
    }
  } else {
    if (!wifiConnected) {
      wifiConnected = true;
      Serial.println(F("✓ WiFi Reconnected!"));
      Serial.print(F("IP: "));
      Serial.println(WiFi.localIP());

      secureClient.setInsecure();  // Reconfigure SSL after reconnection
      Serial.println(F("✓ SSL/TLS reconfigured"));
      
      webSocket.beginSSL(SERVER_HOST, SERVER_PORT, "/ws");
    }
  }
}

void sendDataToServer() {
  if (!wifiConnected || currentState != STATE_MONITORING) {
    return;
  }
  
  if (!sensorsValid) {
    return;
  }
  
  HTTPClient https;  // Use HTTPS
  
  // ✅ Build HTTPS URL correctly
  String serverUrl = "https://";
  serverUrl += SERVER_HOST;
  serverUrl += "/api/data";  // No port needed for standard HTTPS (443)
  
  // ✅ CRITICAL: Use secure client
  https.begin(secureClient, serverUrl);
  https.addHeader("Content-Type", "application/json");
  https.setTimeout(SERVER_TIMEOUT);
  
  // ✅ Create complete JSON payload
  StaticJsonDocument<768> doc;
  
  doc["deviceId"] = DEVICE_ID;
  doc["deviceType"] = DEVICE_TYPE;  // ✅ Use constant
  doc["voltage"] = voltageReading;
  doc["state"] = getStateString();
  doc["sensors"] = sensorsValid ? "valid" : "invalid";
  
  // Channel 1 data
  JsonObject ch1 = doc.createNestedObject("channel1");
  ch1["current"] = channels[CHANNEL_1].currentReading;
  ch1["power"] = channels[CHANNEL_1].powerReading;
  ch1["energy"] = channels[CHANNEL_1].energyConsumed;
  ch1["cost"] = channels[CHANNEL_1].costAccumulated;
  ch1["relayState"] = channels[CHANNEL_1].relayEnabled;
  
  // Channel 2 data
  JsonObject ch2 = doc.createNestedObject("channel2");
  ch2["current"] = channels[CHANNEL_2].currentReading;
  ch2["power"] = channels[CHANNEL_2].powerReading;
  ch2["energy"] = channels[CHANNEL_2].energyConsumed;
  ch2["cost"] = channels[CHANNEL_2].costAccumulated;
  ch2["relayState"] = channels[CHANNEL_2].relayEnabled;
  
  // Total values
  doc["totalPower"] = channels[CHANNEL_1].powerReading + channels[CHANNEL_2].powerReading;
  doc["totalEnergy"] = channels[CHANNEL_1].energyConsumed + channels[CHANNEL_2].energyConsumed;
  doc["totalCost"] = channels[CHANNEL_1].costAccumulated + channels[CHANNEL_2].costAccumulated;
  
  String jsonPayload;
  serializeJson(doc, jsonPayload);
  
  // ✅ Add retry logic
  int attempts = 0;
  bool success = false;
  
  while (attempts < MAX_RETRY_ATTEMPTS && !success) {
    int httpCode = https.POST(jsonPayload);
    
    if (httpCode > 0) {
      if (httpCode == 200 || httpCode == HTTP_CODE_OK) {
        success = true;
        serverConnected = true;
        serverSuccessCount++;
        
        // Quiet success indicator (detailed log every 20 sends)
        if (serverSuccessCount % 20 == 0) {
          String response = https.getString();
          Serial.println(F("\nData sent to server successfully"));
          Serial.printf("  Success count: %d\n", serverSuccessCount);
          Serial.printf("  Response: %s\n", response.c_str());
        } else {
          Serial.print(F("."));  // Quiet indicator
        }
      } else {
        serverErrorCount++;
        Serial.printf("\nâš  Server returned HTTP %d (attempt %d/%d)\n", 
                      httpCode, attempts + 1, MAX_RETRY_ATTEMPTS);
        attempts++;
        if (attempts < MAX_RETRY_ATTEMPTS) {
          delay(1000);  // Wait before retry
        }
      }
    } else {
      serverErrorCount++;
      Serial.printf("\nâœ— HTTP request failed (attempt %d/%d): %s\n", 
                    attempts + 1, MAX_RETRY_ATTEMPTS,
                    https.errorToString(httpCode).c_str());
      attempts++;
      if (attempts < MAX_RETRY_ATTEMPTS) {
        delay(1000);
      }
    }
  }
  
  if (!success) {
    serverConnected = false;
    Serial.printf("âœ— Failed to send data after %d attempts\n", MAX_RETRY_ATTEMPTS);
    Serial.printf("  Error count: %d, Success count: %d\n", serverErrorCount, serverSuccessCount);
  }
  
  https.end();
}

void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
  switch(type) {
    case WStype_DISCONNECTED:
      Serial.println(F("WebSocket Disconnected"));
      serverConnected = false;
      break;
      
    case WStype_CONNECTED:
      Serial.println(F("✓ WebSocket Connected"));
      serverConnected = true;
      delay(100);  // Small delay to ensure connection is established
      registerWithServer();
      break;
      
    case WStype_TEXT:
      Serial.print(F("WebSocket Command: "));
      Serial.println((char*)payload);
      
      {
        StaticJsonDocument<256> doc;
        DeserializationError error = deserializeJson(doc, payload, length);
        
        if (!error) {
          if (doc.containsKey("command")) {
            const char* deviceId = doc["deviceId"];
            const char* command = doc["command"];
            
            if (deviceId == nullptr || strcmp(deviceId, DEVICE_ID) == 0) {
              processServerCommand(command);
            }
          }
        }
      }
      break;
      
    case WStype_ERROR:
      Serial.println(F("✗ WebSocket Error"));
      serverConnected = false;
      break;
      
    default:
      break;
  }
}

void processServerCommand(const char* command) {
  Serial.print(F("Processing server command: "));
  Serial.println(command);
     
     // Process the command
  processCommand(command);
     
     // Send acknowledgment
  sendCommandAck(command, true);
}

void reconnectWebSocket() {
  if (wifiConnected && !webSocket.isConnected()) {
    if (millis() - lastWebSocketReconnect >= WEBSOCKET_RECONNECT_INT) {
      Serial.println(F("Reconnecting WebSocket..."));
      webSocket.beginSSL(SERVER_HOST, SERVER_PORT, "/ws");
      lastWebSocketReconnect = millis();
    }
  }
}