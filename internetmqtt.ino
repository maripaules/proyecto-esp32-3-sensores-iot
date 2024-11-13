#include <WiFi.h>                                  //programa para conectarse al pc unsando la ESP32
#include <PubSubClient.h>

// Credenciales

const char* ssid = "Spaceunicorn";
const char* password = "2444666668888888";
const char* mqtt_server = "192.168.232.175";

//Configuración del cliente MQTT y WIFI
WiFiClient espClient;
PubSubClient client(espClient);


void setup_wifi() {
  delay(30);
  Serial.begin(9600);
  // Conectar a Wi-Fi con la SSID y password
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);   //Demora hasta que se conecta
    Serial.println("No se conecto a WiFi");
  }
  Serial.println("WiFi ok");
}
void callback(char* topic, byte* message, unsigned int length) {   // Maneja los mensajes que llegan en topico suscrito
}

void reconnect() {                                                 // Nos conectamos y lo intentamos hasta reconectarnos
  while (!client.connected()) {
    String clientId = "ESP32-";                                    //Crea un cliente ID basado en la MAC de la ESP32, dirección ip de la ESP32
    clientId += String(WiFi.macAddress());
    Serial.println(clientId);
    
    if (client.connect(clientId.c_str())) {                       // Intentamos conectar Al brocker de EMQX con el cliente ID
      client.subscribe("t1");                                     // Una vez conectado, sucribamonos al topico
      Serial.println("se conecto al Broker");
    } 
    else {
      delay(2000);                                                // SI no podemos nos conectamos otra vez en 5 seg
      Serial.println("No se conecto al Broker");
    }
  }
}

void setup() {                                                    // coneión serial
//  Serial.begin(9600);
  setup_wifi();
  client.setServer(mqtt_server, 1883);                            //La dirección del puerto en wifi para EMQX 
  client.setCallback(callback);
  Serial.println(mqtt_server);
}
void loop() {
  if (!client.connected()) {                                    
    reconnect();
  }
                                                                    // Publica los datos de la variable en topico
  float variable = 38;                                              // Replace with your variable data
  String payload = String(variable);
  client.publish("t1", payload.c_str());
  Serial.println("Publico");
//  Serial.println(variable);
  delay(4000);                                                    // Periodo de espera para intentar publicar de nuevo para evitar colición.
}