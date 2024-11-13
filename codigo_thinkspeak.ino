#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <WiFi.h>
#include <HTTPClient.h>

// Configuración de la red Wi-Fi
const char* ssid = "tu_SSID";     // Reemplaza con tu SSID
const char* password = "tu_password"; // Reemplaza con tu contraseña Wi-Fi

// Configuración de ThingSpeak
const char* server = "http://api.thingspeak.com/update";
String apiKey = "TU_API_KEY";  // Reemplaza con tu Write API Key de ThingSpeak

MAX30105 particleSensor; // Inicializar el sensor MAX30102
Adafruit_MPU6050 mpu;    // Inicializar el sensor MPU6050

// Almacenamiento para la señal IR y cálculo de frecuencia cardíaca
const byte RATE_SIZE = 4; // Número de lecturas para el promedio
byte rates[RATE_SIZE]; // Almacena las últimas mediciones
byte rateSpot = 0;
long lastBeat = 0; // Almacena el tiempo de la última señal

float beatsPerMinute;
int beatAvg;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  // Inicialización del sensor MAX30102
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30102 no encontrado. Verifique las conexiones.");
    while (1);
  }

  Serial.println("Iniciando sensor MAX30102...");

  // Configuración predeterminada del sensor
  particleSensor.setup(); // Configura el sensor con valores predeterminados
  particleSensor.setPulseAmplitudeRed(0x0A); // LED Rojo para medición de SpO2
  particleSensor.setPulseAmplitudeGreen(0); // Apagar el LED verde

  // Inicialización del sensor MPU6050
  Serial.println("Inicializando MPU6050...");
  if (!mpu.begin()) {
    Serial.println("MPU6050 no encontrado. Verifica las conexiones.");
    while (1);
  }
  Serial.println("MPU6050 inicializado correctamente.");

  // Conectar a la red Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Conectado a la red Wi-Fi.");
}

void loop() {
  long irValue = particleSensor.getIR(); // Lee el valor del IR
  
  if (checkForBeat(irValue)) {
    // Calcular los latidos por minuto (BPM)
    long delta = millis() - lastBeat;
    lastBeat = millis();

    beatsPerMinute = 60 / (delta / 1000.0);
    
    if (beatsPerMinute < 255 && beatsPerMinute > 20) { // Rango aceptable
      rates[rateSpot++] = (byte)beatsPerMinute;
      rateSpot %= RATE_SIZE;
      
      // Promediar el ritmo cardíaco
      beatAvg = 0;
      for (byte x = 0; x < RATE_SIZE; x++)
        beatAvg += rates[x];
      beatAvg /= RATE_SIZE;

      // Leer los valores del acelerómetro y giroscopio
      sensors_event_t accel;
      sensors_event_t gyro;
      mpu.getAccelerometerSensor()->getEvent(&accel);
      mpu.getGyroSensor()->getEvent(&gyro);

      // Obtener los valores de aceleración y giroscopio
      float accX = accel.acceleration.x;
      float accY = accel.acceleration.y;
      float accZ = accel.acceleration.z;
      float gyroX = gyro.gyro.x;
      float gyroY = gyro.gyro.y;
      float gyroZ = gyro.gyro.z;

      // Enviar datos a ThingSpeak
      if (WiFi.status() == WL_CONNECTED) {
        HTTPClient http;

        // Construir la URL con los datos
        String url = String(server) + "?api_key=" + apiKey + 
                     "&field1=" + String(beatsPerMinute) + 
                     "&field2=" + String(accX) + 
                     "&field3=" + String(accY) + 
                     "&field4=" + String(accZ) + 
                     "&field5=" + String(gyroX) + 
                     "&field6=" + String(gyroY) + 
                     "&field7=" + String(gyroZ);

        // Enviar datos a ThingSpeak
        http.begin(url);  // Iniciar la conexión HTTP
        int httpCode = http.GET();  // Hacer la solicitud GET

        // Comprobar si la solicitud ha sido exitosa
        if (httpCode > 0) {
          Serial.println("Datos enviados a ThingSpeak correctamente");
        } else {
          Serial.println("Error al enviar los datos");
        }

        http.end();  // Finalizar la conexión HTTP
      }
    }
  }

  // Mostrar datos si la señal IR es válida
  if (irValue < 50000) {
    Serial.println("Por favor, coloque su dedo en el sensor");
  } else {
    Serial.print("Latidos por minuto: ");
    Serial.println(beatsPerMinute);
    Serial.print("Promedio BPM: ");
    Serial.println(beatAvg);
  }

  delay(10000); // Esperar 10 segundos antes de la próxima lectura
}
