#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <WiFi.h>
#include <PubSubClient.h>

// Instancias de sensores y MQTT
MAX30105 particleSensor;
Adafruit_MPU6050 mpu;
WiFiClient espClient;

// Configuración de Wi-Fi
const char* ssid = "Spaceunicorn";
const char* password = "2444666668888888";

// Configuración de ThingSpeak
const char* apiKey = "GHDKNXWFA4Q1WL3U";  // Tu Write API Key de ThingSpeak
const char* server = "api.thingspeak.com";

// Almacenamiento para frecuencia cardíaca
const byte RATE_SIZE = 4;
byte rates[RATE_SIZE];
byte rateSpot = 0;
long lastBeat = 0;

float beatsPerMinute;
int beatAvg;

// Definir el pin del LED
const int ledPin = 2;  // Pin 2 generalmente es el LED incorporado en ESP32

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Inicialización del sensor de frecuencia cardíaca MAX30105
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30102 no encontrado. Verifique las conexiones.");
    while (1);
  }
  particleSensor.setup();
  particleSensor.setPulseAmplitudeRed(0x0A);
  particleSensor.setPulseAmplitudeGreen(0);

  // Inicialización del sensor MPU6050
  if (!mpu.begin()) {
    Serial.println("MPU6050 no encontrado. Verifique las conexiones.");
    while (1);
  }

  // Conectar a la red Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConectado a la red Wi-Fi.");

  // Configurar el pin del LED como salida
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);  // Asegurarse de que el LED está apagado al inicio
}

void loop() {
  // Leer datos del sensor de frecuencia cardíaca
  long irValue = particleSensor.getIR();
  
  if (checkForBeat(irValue)) {
    long delta = millis() - lastBeat;
    lastBeat = millis();
    beatsPerMinute = 60 / (delta / 1000.0);

    if (beatsPerMinute < 255 && beatsPerMinute > 20) {
      rates[rateSpot++] = (byte)beatsPerMinute;
      rateSpot %= RATE_SIZE;

      beatAvg = 0;
      for (byte x = 0; x < RATE_SIZE; x++) {
        beatAvg += rates[x];
      }
      beatAvg /= RATE_SIZE;
    }
  }

  // Mostrar datos de frecuencia cardíaca
  if (irValue < 50000) {
    Serial.println("Por favor, coloque su dedo en el sensor");
  } else {
    Serial.print("Latidos por minuto: ");
    Serial.println(beatsPerMinute);
    Serial.print("Promedio BPM: ");
    Serial.println(beatAvg);
  }

  // Leer datos del sensor MPU6050
  sensors_event_t accel, gyro;
  mpu.getAccelerometerSensor()->getEvent(&accel);
  mpu.getGyroSensor()->getEvent(&gyro);

  // Imprimir los valores de acelerómetro y giroscopio
  float accX = accel.acceleration.x;
  float accY = accel.acceleration.y;
  float accZ = accel.acceleration.z;
  float gyroX = gyro.gyro.x;
  float gyroY = gyro.gyro.y;
  float gyroZ = gyro.gyro.z;

  Serial.print("Acelerómetro [X]: "); Serial.print(accX);
  Serial.print(" [Y]: "); Serial.print(accY);
  Serial.print(" [Z]: "); Serial.println(accZ);

  Serial.print("Giroscopio [X]: "); Serial.print(gyroX);
  Serial.print(" [Y]: "); Serial.print(gyroY);
  Serial.print(" [Z]: "); Serial.println(gyroZ);

  // Enviar datos a ThingSpeak
  if(WiFi.status() == WL_CONNECTED){  // Verificar si está conectado a WiFi
    WiFiClient client;
    
    if(client.connect(server, 80)){  // Conectar a ThingSpeak
      String postStr = apiKey;
      postStr += "&field1=";  // BPM
      postStr += String(beatsPerMinute);
      postStr += "&field2=";  // BPM promedio
      postStr += String(beatAvg);
      postStr += "&field3=";  // Acelerómetro X
      postStr += String(accX);
      postStr += "&field4=";  // Acelerómetro Y
      postStr += String(accY);
      postStr += "&field5=";  // Acelerómetro Z
      postStr += String(accZ);
      postStr += "&field6=";  // Giroscopio X
      postStr += String(gyroX);
      postStr += "&field7=";  // Giroscopio Y
      postStr += String(gyroY);
      postStr += "&field8=";  // Giroscopio Z
      postStr += String(gyroZ);
      postStr += "\r\n\r\n";

      client.print("POST /update HTTP/1.1\n");
      client.print("Host: api.thingspeak.com\n");
      client.print("Connection: close\n");
      client.print("X-THINGSPEAKAPIKEY: " + String(apiKey) + "\n");
      client.print("Content-Type: application/x-www-form-urlencoded\n");
      client.print("Content-Length: " + String(postStr.length()) + "\n\n");
      client.print(postStr);

      Serial.println("Datos enviados a ThingSpeak.");
    }
    client.stop();
  }

  delay(20000);  // Enviar datos cada 20 segundos
}
