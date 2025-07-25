/*
 * ESP32 Capacitive Sensor UDP Sender
 * 
 * Make sure to create credentials.hpp with your WiFi credentials:
 * - WIFI_SSID
 * - WIFI_PASSWORD  
 * - UDP_ADDRESS
 * - UDP_PORT
 */

#include <WiFi.h>
#include <WiFiUdp.h>
#include <CapacitiveSensor.h>
#include "credentials.hpp"

// WiFi configuration (from credentials.hpp)
const char* ssid = WIFI_SSID;
const char* password = WIFI_PASSWORD;

// UDP configuration (from credentials.hpp)
const char* udpAddress = UDP_ADDRESS;
const int udpPort = UDP_PORT;

// Capacitive sensor configuration
CapacitiveSensor cs_4_2 = CapacitiveSensor(4, 2);  // 10M resistor between pins 4 & 2
CapacitiveSensor cs_4_6 = CapacitiveSensor(4, 6);  // 10M resistor between pins 4 & 6  
CapacitiveSensor cs_4_8 = CapacitiveSensor(4, 8);  // 10M resistor between pins 4 & 8

// WiFi and UDP objects
WiFiUDP udp;

// Sensor data structure (simplified - no timestamp)
struct SensorData {
  uint32_t device_id;      // 4 bytes
  uint32_t num_sensors;    // 4 bytes
  float sensor_values[8];  // 32 bytes (max 8 sensors)
} sensorData;

void setup() {
    Serial.begin(115200);
    
    // Initialize sensor data structure
    sensorData.device_id = 0x00000001;  // Device ID
    sensorData.num_sensors = 3;  // 3 capacitive sensors
    
    // Configure capacitive sensors
    cs_4_2.set_CS_AutocaL_Millis(0xFFFFFFFF);  // turn off autocalibrate
    cs_4_6.set_CS_AutocaL_Millis(0xFFFFFFFF);  // turn off autocalibrate
    cs_4_8.set_CS_AutocaL_Millis(0xFFFFFFFF);  // turn off autocalibrate
    
    // Connect to WiFi
    WiFi.begin(ssid, password);
    Serial.print("Connecting to WiFi");
    
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    
    Serial.println();
    Serial.println("WiFi connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    
    // Initialize UDP
    udp.begin(udpPort);
    Serial.println("UDP initialized");
}

void loop() {
    // Read capacitive sensor values
    long start = millis();
    long total1 = cs_4_2.capacitiveSensor(30);
    long total2 = cs_4_6.capacitiveSensor(30);
    long total3 = cs_4_8.capacitiveSensor(30);
    
    // Store raw capacitive sensor values directly
    sensorData.sensor_values[0] = (float) total1;
    sensorData.sensor_values[1] = (float) total2;
    sensorData.sensor_values[2] = (float) total3;
    
    // Send data via UDP
    udp.beginPacket(udpAddress, udpPort);
    udp.write((uint8_t*)&sensorData, sizeof(sensorData));
    udp.endPacket();
    
    // Print debug info
    Serial.print("Performance: ");
    Serial.print(millis() - start);
    Serial.print("ms | Sent: ");
    Serial.print(total1);
    Serial.print(" ");
    Serial.print(total2);
    Serial.print(" ");
    Serial.println(total3);
    
    delay(10);  // 100 Hz update rate (matching original code)
} 