#include <Arduino.h>
#include <WiFi.h>
#include <HardwareSerial.h>
#include "imu_sdk/Arduino/Arduino_sdk/wit_c_sdk.h"

// WiFi credentials
const char* ssid = "OPTUS_663502M";
const char* password = "comic36729me";

// ESP32 Serial ports for IMU communication
HardwareSerial SerialIMU(0); // Use Serial1 for IMU communication
#define IMU_RX_PIN 3  // GPIO16 for RX
#define IMU_TX_PIN 1  // GPIO17 for TX

// IMU variables
bool imuInitialized = false;
unsigned long lastIMURead = 0;
const unsigned long IMU_READ_INTERVAL = 100; // Read every 100ms
const uint32_t c_uiBaud[8] = {0, 4800, 9600, 19200, 38400, 57600, 115200, 230400};

// Data update flags
#define ACC_UPDATE		0x01
#define GYRO_UPDATE		0x02
#define ANGLE_UPDATE	0x04
#define MAG_UPDATE		0x08
#define READ_UPDATE		0x80
static volatile char s_cDataUpdate = 0;

// Function declarations
int myFunction(int x, int y);
void initIMU();
void readIMUData();
void autoScanSensor();
void sensorUartSend(uint8_t *p_data, uint32_t uiSize);
void sensorDataUpdate(uint32_t uiReg, uint32_t uiRegNum);
void delayMsFunc(uint16_t ms);
void processIMUData();

int result;

void setup() {
  // Initialize main serial
  Serial.begin(115200);
  

  result = myFunction(2, 3);
  
  Serial.println("ESP32 IMU & WiFi Test Starting...");
  Serial.println("SSID: " + String(ssid));
  
  // Initialize IMU Serial with custom pins
  SerialIMU.begin(115200, SERIAL_8N1, IMU_RX_PIN, IMU_TX_PIN);
  Serial.printf("IMU Serial initialized on RX:%d, TX:%d\n", IMU_RX_PIN, IMU_TX_PIN);
  
  // Initialize WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  Serial.println();
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("WiFi connected!");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    Serial.print("Signal strength (RSSI): ");
    Serial.println(WiFi.RSSI());
  } else {
    Serial.println("WiFi connection failed!");
    Serial.print("WiFi status: ");
    Serial.println(WiFi.status());
    Serial.println("Continuing without WiFi...");
  }
  
  // Initialize IMU
  Serial.println("Initializing IMU SDK...");
  initIMU();
  
  Serial.println("ESP32 TEST " + String(result) + " - Setup Complete");
}

void loop() {
  // Process incoming IMU data
  while (SerialIMU.available()) {
    WitSerialDataIn(SerialIMU.read());
  }
  
  // Process IMU data updates
  processIMUData();
  
  // Check WiFi status periodically (every 5 seconds)
  static unsigned long lastWiFiCheck = 0;
  if (millis() - lastWiFiCheck >= 5000) {
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("ESP32 TEST " + String(result) + " - WiFi Connected - IP: " + WiFi.localIP().toString() + " - RSSI: " + String(WiFi.RSSI()));
    } else {
      Serial.println("ESP32 TEST " + String(result) + " - WiFi Disconnected - Status: " + String(WiFi.status()));
      
      // Try to reconnect if disconnected
      Serial.println("Attempting to reconnect...");
      WiFi.reconnect();
    }
    lastWiFiCheck = millis();
  }
  
  // Read IMU data periodically
  if (imuInitialized && (millis() - lastIMURead >= IMU_READ_INTERVAL)) {
    readIMUData();
    lastIMURead = millis();
  }
  
  delay(50); // Small delay to prevent overwhelming the system
}

// IMU initialization function
void initIMU() {
  // Initialize WIT SDK
  WitInit(WIT_PROTOCOL_NORMAL, 0x50);
  WitSerialWriteRegister(sensorUartSend);
  WitRegisterCallBack(sensorDataUpdate);
  WitDelayMsRegister(delayMsFunc);
  
  Serial.println("WIT SDK initialized, scanning for sensor...");
  
  // Auto-scan for sensor
  autoScanSensor();
}

void autoScanSensor() {
  int i, iRetry;
  
  for (i = 1; i < sizeof(c_uiBaud)/sizeof(c_uiBaud[0]); i++) {
    Serial.printf("Trying baud rate: %d\n", c_uiBaud[i]);
    SerialIMU.begin(c_uiBaud[i], SERIAL_8N1, IMU_RX_PIN, IMU_TX_PIN);
    SerialIMU.flush();
    
    iRetry = 3;
    s_cDataUpdate = 0;
    
    do {
      WitReadReg(AX, 3);
      delay(200);
      
      // Process any incoming data
      while (SerialIMU.available()) {
        WitSerialDataIn(SerialIMU.read());
      }
      
      if (s_cDataUpdate != 0) {
        Serial.printf("Sensor found at %d baud rate!\n", c_uiBaud[i]);
        imuInitialized = true;
        
        // Configure sensor
        if (WitSetOutputRate(RRATE_10HZ) == WIT_HAL_OK) {
          Serial.println("IMU output rate set to 10Hz");
        }
        
        if (WitSetContent(RSW_ACC | RSW_GYRO | RSW_ANGLE | RSW_MAG) == WIT_HAL_OK) {
          Serial.println("IMU content configured for ACC, GYRO, ANGLE, MAG");
        }
        
        return;
      }
      iRetry--;
    } while (iRetry);
  }
  
  Serial.println("Cannot find IMU sensor!");
  Serial.println("Please check your connections:");
  Serial.printf("IMU TX -> ESP32 GPIO%d (RX)\n", IMU_RX_PIN);
  Serial.printf("IMU RX -> ESP32 GPIO%d (TX)\n", IMU_TX_PIN);
  Serial.println("IMU VCC -> 3.3V or 5V");
  Serial.println("IMU GND -> GND");
}

void readIMUData() {
  // Request fresh data from sensor
  WitReadReg(AX, 3);   // Read accelerometer
  delay(10);
  WitReadReg(GX, 3);   // Read gyroscope  
  delay(10);
  WitReadReg(Roll, 3); // Read angles
  delay(10);
  WitReadReg(HX, 3);   // Read magnetometer
}

void processIMUData() {
  static unsigned long lastPrint = 0;
  float fAcc[3], fGyro[3], fAngle[3];
  int i;
  
  if (s_cDataUpdate && (millis() - lastPrint >= 1000)) { // Print every 1 second
    Serial.println("=== ESP32 IMU Data ===");
    
    // Convert raw data to physical units
    for (i = 0; i < 3; i++) {
      fAcc[i] = sReg[AX + i] / 32768.0f * 16.0f;      // ±16g range
      fGyro[i] = sReg[GX + i] / 32768.0f * 2000.0f;   // ±2000°/s range
      fAngle[i] = sReg[Roll + i] / 32768.0f * 180.0f;  // ±180° range
    }
    
    if (s_cDataUpdate & ACC_UPDATE) {
      Serial.printf("Accel: X=%.3f g, Y=%.3f g, Z=%.3f g\n", fAcc[0], fAcc[1], fAcc[2]);
      s_cDataUpdate &= ~ACC_UPDATE;
    }
    
    if (s_cDataUpdate & GYRO_UPDATE) {
      Serial.printf("Gyro: X=%.1f °/s, Y=%.1f °/s, Z=%.1f °/s\n", fGyro[0], fGyro[1], fGyro[2]);
      s_cDataUpdate &= ~GYRO_UPDATE;
    }
    
    if (s_cDataUpdate & ANGLE_UPDATE) {
      Serial.printf("Angle: Roll=%.3f°, Pitch=%.3f°, Yaw=%.3f°\n", fAngle[0], fAngle[1], fAngle[2]);
      s_cDataUpdate &= ~ANGLE_UPDATE;
    }
    
    if (s_cDataUpdate & MAG_UPDATE) {
      Serial.printf("Mag: X=%d, Y=%d, Z=%d\n", sReg[HX], sReg[HY], sReg[HZ]);
      s_cDataUpdate &= ~MAG_UPDATE;
    }
    
    // Print temperature
    float temp = (float)sReg[TEMP] / 100.0f;
    Serial.printf("Temperature: %.2f°C\n", temp);
    
    Serial.println("======================");
    lastPrint = millis();
    s_cDataUpdate = 0;
  }
}

// ESP32-specific serial write function for IMU communication
void sensorUartSend(uint8_t *p_data, uint32_t uiSize) {
  SerialIMU.write(p_data, uiSize);
  SerialIMU.flush();
}

// Register update callback - called when new IMU data is available
void sensorDataUpdate(uint32_t uiReg, uint32_t uiRegNum) {
  int i;
  for (i = 0; i < uiRegNum; i++) {
    switch (uiReg) {
      case AZ:
        s_cDataUpdate |= ACC_UPDATE;
        break;
      case GZ:
        s_cDataUpdate |= GYRO_UPDATE;
        break;
      case HZ:
        s_cDataUpdate |= MAG_UPDATE;
        break;
      case Yaw:
        s_cDataUpdate |= ANGLE_UPDATE;
        break;
      default:
        s_cDataUpdate |= READ_UPDATE;
        break;
    }
    uiReg++;
  }
}

// ESP32 delay function
void delayMsFunc(uint16_t ms) {
  delay(ms);
}

// Test function
int myFunction(int x, int y) {
  return x + y;
}