// ================================================================
// ESP32 4WD ROVER — BLE BRIDGE + A2DP SPEAKER + FULL SENSORS
// ================================================================

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <esp_gap_ble_api.h>
#include <DHT.h>
#include "BluetoothA2DPSource.h" 

// --- BLE UUIDs ---
#define SERVICE_UUID        "19b10000-e8f2-537e-4f6c-d104768a1214"
#define COMMAND_CHAR_UUID   "19b10001-e8f2-537e-4f6c-d104768a1214"
#define TELEMETRY_CHAR_UUID "19b10002-e8f2-537e-4f6c-d104768a1214"

BLEServer* pServer = NULL;
BLECharacteristic* pTelemetryCharacteristic = NULL;
BLECharacteristic* pCommandCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;

// --- A2DP BLUETOOTH SPEAKER CONFIG ---
BluetoothA2DPSource a2dp_source;
String BLUETOOTH_SPEAKER_NAME = "Dubstep Pop 610";

// ---------------------------------------------------------------
// AUDIO DATA ARRAY (8-bit Unsigned Char)
// ---------------------------------------------------------------
const unsigned char ttsMP3_VoiceText_2026_8_21_15_3_47[] PROGMEM = { 

  // >>> PASTE YOUR ENTIRE GIANT HEX ARRAY HERE <<<

}; 

const size_t ttsMP3_VoiceText_len = sizeof(ttsMP3_VoiceText_2026_8_21_15_3_47);

// Audio streaming & resampling state variables
const unsigned char* current_audio_ptr = nullptr;
size_t current_audio_remaining = 0;
uint32_t current_sample_idx = 0;
uint32_t current_fraction = 0;
String last_gas_status = "safe";

// ---------------------------------------------------------------
// 1. PIN DEFINITIONS & GLOBALS
// ---------------------------------------------------------------
#define LEFT_R_PWM 26
#define LEFT_L_PWM 27
#define LEFT_R_EN  13
#define LEFT_L_EN  12
#define RIGHT_R_PWM 14
#define RIGHT_L_PWM 25
#define RIGHT_R_EN  4
#define RIGHT_L_EN  15

#define TRIG_PIN      5
#define ECHO_PIN      18
#define MQ2_PIN       34
#define MQ135_PIN     35
#define FLAME_PIN     32
#define ALARM_PIN     19
#define FLAME_LED_PIN 2
#define DHTPIN        33
#define DHTTYPE       DHT11

#define PWM_FREQ        5000
#define PWM_RESOLUTION  8  

int driveSpeed = 200; 
char currentDir = 'S';
int distance, mq2_val, mq135_val, flame_val = 0;
float temperature, humidity = 0.0;
unsigned long lastSensorRead, lastTelemetryPush, lastDhtRead = 0;

// RSSI Tracking Variables
int8_t ble_rssi = -100;
esp_bd_addr_t connected_bd_addr;

DHT dht(DHTPIN, DHTTYPE);

// ---------------------------------------------------------------
// 2. AUDIO & LOGIC FUNCTIONS
// ---------------------------------------------------------------
int32_t get_audio_data(Frame *channels, int32_t frame_count) {
  if (current_audio_remaining == 0 || current_audio_ptr == nullptr) {
    for (int i = 0; i < frame_count; i++) {
      channels[i].channel1 = 0;
      channels[i].channel2 = 0;
    }
    return frame_count;
  }

  uint32_t total_samples = current_audio_remaining / 2;
  int frames_played = 0;

  for (int i = 0; i < frame_count; i++) {
    if (current_sample_idx >= total_samples) {
      current_audio_remaining = 0; 
      break;
    }
    int16_t sample = (int16_t)(current_audio_ptr[current_sample_idx * 2] | (current_audio_ptr[current_sample_idx * 2 + 1] << 8));
    channels[i].channel1 = sample;
    channels[i].channel2 = sample; 

    current_fraction += 8000;
    if (current_fraction >= 44100) {
      current_fraction -= 44100;
      current_sample_idx++;
    }
    frames_played++;
  }

  for (int i = frames_played; i < frame_count; i++) {
    channels[i].channel1 = 0;
    channels[i].channel2 = 0;
  }
  return frame_count;
}

void triggerUnsafeWarning() {
  Serial.println("Playing Unsafe Warning...");
  current_audio_ptr = ttsMP3_VoiceText_2026_8_21_15_3_47 + 44;
  current_audio_remaining = ttsMP3_VoiceText_len - 44;
  current_sample_idx = 0;
  current_fraction = 0;
}

// ---------------------------------------------------------------
// 3. MOTOR & BLE CALLBACKS
// ---------------------------------------------------------------
int getPWM() {
  return map(constrain(driveSpeed, 0, 500), 0, 500, 0, 255);
}

void stopMotors() { currentDir = 'S'; ledcWrite(LEFT_R_PWM, 0); ledcWrite(LEFT_L_PWM, 0); ledcWrite(RIGHT_R_PWM, 0); ledcWrite(RIGHT_L_PWM, 0); }
void moveForward() { currentDir = 'F'; int pwm = getPWM(); ledcWrite(LEFT_R_PWM, pwm); ledcWrite(LEFT_L_PWM, 0); ledcWrite(RIGHT_R_PWM, pwm); ledcWrite(RIGHT_L_PWM, 0); }
void moveBackward() { currentDir = 'B'; int pwm = getPWM(); ledcWrite(LEFT_R_PWM, 0); ledcWrite(LEFT_L_PWM, pwm); ledcWrite(RIGHT_R_PWM, 0); ledcWrite(RIGHT_L_PWM, pwm); }
void turnLeft() { currentDir = 'L'; int pwm = getPWM(); ledcWrite(LEFT_R_PWM, 0); ledcWrite(LEFT_L_PWM, pwm); ledcWrite(RIGHT_R_PWM, pwm); ledcWrite(RIGHT_L_PWM, 0); }
void turnRight() { currentDir = 'R'; int pwm = getPWM(); ledcWrite(LEFT_R_PWM, pwm); ledcWrite(LEFT_L_PWM, 0); ledcWrite(RIGHT_R_PWM, 0); ledcWrite(RIGHT_L_PWM, pwm); }

void custom_gap_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
  if (event == ESP_GAP_BLE_READ_RSSI_COMPLETE_EVT) {
    ble_rssi = param->read_rssi_cmpl.rssi;
  }
}

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer, esp_ble_gatts_cb_param_t *param) { 
      deviceConnected = true; 
      memcpy(connected_bd_addr, param->connect.remote_bda, 6);
    }
    void onDisconnect(BLEServer* pServer) { 
      deviceConnected = false; 
      stopMotors(); 
      ble_rssi = -100;
    }
};

class MyCommandCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      String value = String(pCharacteristic->getValue().c_str());
      if (value.length() > 0) {
        char cmd = value[0];
        if (cmd == 'F') moveForward();
        else if (cmd == 'B') moveBackward();
        else if (cmd == 'L') turnLeft();
        else if (cmd == 'R') turnRight();
        else if (cmd == 'S') stopMotors();
        else if (cmd == 'V') {
          int newSpeed = value.substring(1).toInt();
          driveSpeed = constrain(newSpeed, 0, 500);
          if (currentDir == 'F') moveForward();
          else if (currentDir == 'B') moveBackward();
          else if (currentDir == 'L') turnLeft();
          else if (currentDir == 'R') turnRight();
        }
      }
    }
};

// ---------------------------------------------------------------
// 4. SETUP
// ---------------------------------------------------------------
void setup() {
  Serial.begin(115200);

  // Force ESP32 to map Analog readings to the standard 0-1023 Arduino scale
  analogReadResolution(10); 

  pinMode(LEFT_R_EN, OUTPUT);  digitalWrite(LEFT_R_EN, HIGH);
  pinMode(LEFT_L_EN, OUTPUT);  digitalWrite(LEFT_L_EN, HIGH);
  pinMode(RIGHT_R_EN, OUTPUT); digitalWrite(RIGHT_R_EN, HIGH);
  pinMode(RIGHT_L_EN, OUTPUT); digitalWrite(RIGHT_L_EN, HIGH);
  ledcAttachChannel(LEFT_R_PWM, PWM_FREQ, PWM_RESOLUTION, 0);
  ledcAttachChannel(LEFT_L_PWM, PWM_FREQ, PWM_RESOLUTION, 1);
  ledcAttachChannel(RIGHT_R_PWM, PWM_FREQ, PWM_RESOLUTION, 2);
  ledcAttachChannel(RIGHT_L_PWM, PWM_FREQ, PWM_RESOLUTION, 3);
  stopMotors();

  pinMode(TRIG_PIN, OUTPUT); pinMode(ECHO_PIN, INPUT);
  pinMode(MQ2_PIN, INPUT); pinMode(MQ135_PIN, INPUT);
  pinMode(FLAME_PIN, INPUT);
  dht.begin();

  // --- 1. INITIALIZE BLE STACK FIRST (Prevents Memory Crash) ---
  Serial.println("Initializing BLE Stack...");
  BLEDevice::init("ESP32_Rover_BLE");
  BLEDevice::setCustomGapHandler(custom_gap_handler);

  // --- 2. START A2DP SPEAKER (Avoids Antenna Contention) ---
  Serial.println("Starting A2DP Source. Searching for speaker...");
  a2dp_source.start(BLUETOOTH_SPEAKER_NAME.c_str(), get_audio_data);
  
  // Give Bluetooth Classic 5 seconds to pair without BLE interrupting the antenna
  Serial.println("Waiting 5 seconds for speaker to connect...");
  delay(5000);

  // --- 3. START BLE SERVER & ADVERTISING ---
  Serial.println("Starting BLE Server...");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);

  pCommandCharacteristic = pService->createCharacteristic(COMMAND_CHAR_UUID, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  pCommandCharacteristic->setCallbacks(new MyCommandCallbacks());

  pTelemetryCharacteristic = pService->createCharacteristic(TELEMETRY_CHAR_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  pTelemetryCharacteristic->addDescriptor(new BLE2902());

  pService->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  BLEDevice::startAdvertising();
  
  Serial.println("BLE Active. Rover Ready.");
}

// ---------------------------------------------------------------
// 5. MAIN LOOP
// ---------------------------------------------------------------
void loop() {
  
  // 1. FAST SENSORS: Read Gas, Flame, and Ultrasonic every 100ms
  if (millis() - lastSensorRead >= 100) {
    lastSensorRead = millis();

    // Read Gas and Flame
    mq2_val = analogRead(MQ2_PIN);
    mq135_val = analogRead(MQ135_PIN);
    flame_val = analogRead(FLAME_PIN);
    
    // Read Ultrasonic Distance (using Non-Blocking style ping)
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    long duration = pulseIn(ECHO_PIN, HIGH, 30000); // 30ms timeout so rover doesn't freeze
    distance = duration * 0.034 / 2;

    // Safety Trigger Logic
    int highest_gas = max(mq2_val, mq135_val);
    String current_status = "safe";
    
    if (highest_gas >= 900) {
      current_status = "unsafe";
    }
    
    if (current_status != last_gas_status) {
      if (current_status == "unsafe") {
        triggerUnsafeWarning();
      }
      last_gas_status = current_status;
    }
  }

  // 2. SLOW SENSORS: Read DHT11 every 2000ms (It crashes if read faster)
  if (millis() - lastDhtRead >= 2000) {
    lastDhtRead = millis();
    float t = dht.readTemperature();
    float h = dht.readHumidity();
    
    // Only update if the read was successful (prevent "NaN" errors)
    if (!isnan(t) && !isnan(h)) {
      temperature = t;
      humidity = h;
    }
  }

  // 3. PUSH TELEMETRY: Send data to Webpage every 500ms
  if (deviceConnected && (millis() - lastTelemetryPush >= 500)) {
    lastTelemetryPush = millis();
    
    esp_ble_gap_read_rssi(connected_bd_addr);

    // Build the complete JSON string with all sensors
    String telemetry = "{\"g\":" + String(mq2_val) + 
                       ",\"a\":" + String(mq135_val) + 
                       ",\"t\":" + String(temperature, 1) + 
                       ",\"h\":" + String(humidity, 1) + 
                       ",\"d\":" + String(distance) + 
                       ",\"f\":" + String(flame_val) + 
                       ",\"r\":" + String(ble_rssi) + "}";
                       
    pTelemetryCharacteristic->setValue(telemetry.c_str());
    pTelemetryCharacteristic->notify();
  }

  // 4. CONNECTION MANAGEMENT
  if (!deviceConnected && oldDeviceConnected) {
    delay(500); pServer->startAdvertising(); oldDeviceConnected = deviceConnected;
  }
  if (deviceConnected && !oldDeviceConnected) {
    oldDeviceConnected = deviceConnected;
  }
}