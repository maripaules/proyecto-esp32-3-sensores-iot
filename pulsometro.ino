#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <WiFi.h>
#include <HTTPClient.h>

// WiFi configurations
const char* ssid = "Redmi 9";
const char* password = "17846547";

// ThingSpeak configuration
const char* apiKey = "GHDKNXWFA4Q1WL3U";  // Replace with your API Key
const char* server = "https://api.thingspeak.com/update?api_key=GHDKNXWFA4Q1WL3U&field1=0";

// Create an MPU6050 instance
Adafruit_MPU6050 mpu;

// Variables for sensor data
float temperatura = 25.0;  // Example temperature value to send to ThingSpeak

void setup(void) {
  Serial.begin(115200);
  
  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  // Configure MPU6050
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  
  // Connect to WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Conectando al WiFi...");
  }
  Serial.println("Conectado al WiFi");
}

void loop() {
  // Get new sensor events with the readings
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Print out the sensor values
  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");

  Serial.print("Rotation X: ");
  Serial.print(g.gyro.x);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y);
  Serial.print(", Z: ");
  Serial.print(g.gyro.z);
  Serial.println(" rad/s");

  // Update the temperature variable
  temperatura = temp.temperature;

  // Send data to ThingSpeak
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    String url = String(server) + "?api_key=" + apiKey + "&field1=" + String(temperatura);
    
    http.begin(url);  // Start connection to ThingSpeak
    int httpResponseCode = http.GET();  // Make the GET request
    
    if (httpResponseCode > 0) {
      String response = http.getString();  // Read the response
      Serial.println("Datos enviados a ThingSpeak: " + String(httpResponseCode));
      Serial.println("Respuesta: " + response);
    } else {
      Serial.println("Error enviando los datos: " + String(httpResponseCode));
    }
    
    http.end();  // End the connection
  }

  delay(20000);  // Wait 20 seconds before sending the next data
}
