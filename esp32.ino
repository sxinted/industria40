#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>

// ****SSID y contraseña de nuestro servidor a Internet**** 
const char* ssid = "POCO M4 Pro"; 
const char* password = "python22";

// ****Dirección IP del servidor MQTT****
const char* mqtt_server = "192.168.74.51";

WiFiClient espClient;
PubSubClient client(espClient);

float temperatura = 0; //Señal que queremos publicar al broker
const int ledPin1 = 27; // Pin del LED
const int trigPin = 26; // Pin del trigger del sensor HC-SR04
const int echoPin = 25; // Pin del echo del sensor HC-SR04

long lastMsg = 0;
int value = 0;

// Dirección I2C del LM75A es 0x48 (72)
#define Addr 0x48

void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Conectando a ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi conectado");
  Serial.println("Dirección IP: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Mensaje recibido [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  // Manejar el mensaje recibido
  String msg;
  for (int i = 0; i < length; i++) {
    msg += (char)payload[i];
  }

  if (String(topic) == "esp32/output1") {
    if (msg == "ON") {
      digitalWrite(ledPin1, HIGH);
    }
    else if (msg == "OFF") {
      digitalWrite(ledPin1, LOW);
    }
  }
}

void reconnect() {
  // Bucle hasta que seamos reconectados
  while (!client.connected()) {
    Serial.print("Intentando conexión MQTT...");
    if (client.connect("ESP32Client")) {
      Serial.println("conectado");
      client.subscribe("esp32/output1");
    } else {
      Serial.print("falló, rc=");
      Serial.print(client.state());
      Serial.println(" reintento en 5 segundos");
      delay(5000);
    }
  }
}

float getDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  float distance = pulseIn(echoPin, HIGH) * 0.034 / 2; // en centímetros
  return distance;
}

void setup() {
  pinMode(ledPin1, OUTPUT);
  pinMode(trigPin, OUTPUT); // define el pin del trigger como salida
  pinMode(echoPin, INPUT); // define el pin del echo como entrada
  digitalWrite(ledPin1, LOW);
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  
  // Inicializar comunicación I2C como MASTER
  Wire.begin();
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  long now = millis();
  if (now - lastMsg > 5000) {
    lastMsg = now;
    ++value;

    // Comenzar transmisión I2C
    Wire.beginTransmission(Addr);
    // Enviar la dirección para leer
    Wire.write(0x00);
    // Parar transmisión I2C
    Wire.endTransmission();

    // Solicitar 2 bytes de datos
    Wire.requestFrom(Addr, 2);

    // Leer 2 bytes de datos
    if(Wire.available() == 2)
    {
      int data[2];
      data[0] = Wire.read();
      data[1] = Wire.read();

      // Convertir los datos a 12-bits
      float temp = ((data[0] * 256 + (data[1] & 0xF0)) / 16.0);
      if(temp > 2047)
      {
        temp -= 4096;
      }
      temperatura = temp * 0.0625;

      char tempString[8];
      dtostrf(temperatura, 1, 2, tempString);
      client.publish("esp32/temperature", tempString);
    }

    // Publicar distancia
    float distance = getDistance();
    char distanceString[10];
    dtostrf(distance, 2, 2, distanceString);
    client.publish("esp32/distance", distanceString);

    // Publicar el valor del currentFlow
    float currentFlow = analogRead(34)*(3.3/4095.0); // Lectura del potenciómetro en el pin 34
    char currentFlowString[10];
    dtostrf(currentFlow, 1, 2, currentFlowString);
    client.publish("esp32/currentFlow", currentFlowString);
  }
}

