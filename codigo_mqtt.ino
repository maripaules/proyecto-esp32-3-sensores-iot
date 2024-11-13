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
PubSubClient client(espClient);

// Configuración de Wi-Fi
const char* ssid = "Spaceunicorn";
const char* password = "2444666668888888";

// Configuración del broker MQTT
const char* mqttServer = "192.168.232.175";
const int mqttPort = 1883;
const char* mqttUser = "";
const char* mqttPassword = "";

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

  // Configurar el servidor MQTT
  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback);

  // Configurar el pin del LED como salida
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);  // Asegurarse de que el LED está apagado al inicio
}

void loop() {
  // Conectar al broker MQTT si no está conectado
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

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

  // Enviar datos de ambos sensores a través de MQTT
  String payload = "{\"BPM\": " + String(beatsPerMinute) + 
                   ", \"BPM_Avg\": " + String(beatAvg) + 
                   ", \"accX\": " + String(accX) + 
                   ", \"accY\": " + String(accY) + 
                   ", \"accZ\": " + String(accZ) + 
                   ", \"gyroX\": " + String(gyroX) + 
                   ", \"gyroY\": " + String(gyroY) + 
                   ", \"gyroZ\": " + String(gyroZ) + "}";
  
  client.publish("sensor/data", payload.c_str());

  delay(1000);
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Conectando al broker MQTT...");
    if (client.connect("ESP32Client", mqttUser, mqttPassword)) {
      Serial.println("Conectado.");

      // Suscribirse al tema "sensor/control"
      client.subscribe("sensor/control");

    } else {
      Serial.print("Falló, rc=");
      Serial.print(client.state());
      Serial.println(" Intentando nuevamente en 5 segundos.");
      delay(5000);
    }
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Mensaje recibido [");
  Serial.print(topic);
  Serial.print("] ");
  char payload_string[length + 1];
  memcpy(payload_string, payload, length);
  payload_string[length] = '\0';
  Serial.println(payload_string);

  // Si el mensaje recibido es "ON", enciende un LED
  if (strcmp(payload_string, "ON") == 0) {
    Serial.println("Encendiendo LED");
    digitalWrite(ledPin, HIGH); // Enciende el LED
  }
  
  // Si el mensaje es "OFF", apaga el LED
  if (strcmp(payload_string, "OFF") == 0) {
    Serial.println("Apagando LED");
    digitalWrite(ledPin, LOW); // Apaga el LED
  }
}

