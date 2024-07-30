#include <Arduino.h>
#include <SD.h>
#include <SPI.h>
#include <Wire.h>
#include <MPU6050.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <TinyGPS++.h>
#include <ArduinoJson.h>
#include <Time.h>


// Pines de LEDs
const int greenLED = 13;  // LED verde
const int yellowLED = 12; // LED amarillo

// Configuración del WiFi
const char* ssid = "INFINITUM2391_2.4";
const char* password = "123DonCerdito";
// ID dispos
const char* dispo = "PEBAES-010001";
const char* gpsType = "/gps";
const char* lpmType = "/lpm";
const char* stephsType = "/stephs";

// Configuración MQTT
const char* mqtt_server = "54.208.213.200";
const int mqtt_port = 1883;
const char* mqtt_user = "luis";
const char* mqtt_password = "pezcadofrito.1";

// Configuración del GPS
#define RXD2 16
#define TXD2 17

// Configuración del módulo SD
#define CS_PIN_SD 5    // Pin CS del módulo SD

// Configuración del MPU6050
MPU6050 mpu;
const int mpuChipSelect = 5; // Pin CS para la tarjeta SD
const int threshold = 1000; // Umbral para detectar un paso
bool stepDetected = false;
int stepCount = 0;

// Pines y variables para LPM
#define SENSOR_PIN 34  // Pin GPIO del ESP32 conectado al sensor
const int valorSensor_min = 1300;  // Ajustar según las pruebas
const int valorSensor_max = 3000;  // Ajustar según las pruebas
const int bpm_min = 35;  // Ajustar según las lecturas esperadas
const int bpm_max = 170; // Ajustar según las lecturas esperadas

// Intervalos
const unsigned long readInterval = 5000; // Intervalo de lectura de 5 segundos
const unsigned long averageInterval = 600000; // Intervalo para guardar el promedio de 1 minuto
const unsigned long saveIntervalGPS = 350000; // Intervalo de 10 segundos para GPS

unsigned long lastReadTime = 0;
unsigned long lastAverageTime = 0;
unsigned long lastSaveTimeGPS = 0;

// Definir variables para MQTT
unsigned long lastSendTimeMQTT = 0;
const unsigned long sendIntervalMQTT = 70000; // Intervalo de envío de 40 segundos

float totalReadings = 0;
int numReadings = 0;

HardwareSerial mechabullgps(1);
TinyGPSPlus gps;

WiFiClient wifiClient;
PubSubClient client(wifiClient);

void setup() {
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);

  //declaracion de leds
  pinMode(greenLED, OUTPUT);
  pinMode(yellowLED, OUTPUT);

  // Inicialización del GPS
  mechabullgps.begin(9600, SERIAL_8N1, RXD2, TXD2);

  // Inicialización de la tarjeta SD
  if (!SD.begin(CS_PIN_SD)) {
    Serial.println("Error al iniciar la tarjeta SD");
    return;
  }
  Serial.println("Tarjeta SD iniciada correctamente");

  // Inicializa el MPU6050
  Wire.begin(21, 22); // Inicializa I2C con SDA en GPIO 21 y SCL en GPIO 22
  Serial.println("Inicializando el MPU6050...");
  mpu.initialize();
  
  if (mpu.testConnection()) {
    Serial.println("MPU6050 conectado correctamente");
  } else {
    Serial.println("Error al conectar el MPU6050");
  }

  // Verificar si el archivo existe antes de intentar eliminarlo
  if (SD.exists("/gps_data.json")) {
    if (SD.remove("/gps_data.json")) {
      Serial.println("Archivo gps_data.json eliminado");
    } else {
      Serial.println("Error al eliminar el archivo gps_data.json");
    }
  } else {
    Serial.println("Archivo gps_data.json no encontrado, no se eliminará.");
  }
  // Verificar si el archivo existe antes de intentar eliminarlo
  if (SD.exists("/lpm_data.json")) {
    if (SD.remove("/lpm_data.json")) {
      Serial.println("Archivo lpm_data.json eliminado");
    } else {
      Serial.println("Error al eliminar el archivo lpm_data.json");
    }
  } else {
    Serial.println("Archivo lpm_data.json no encontrado, no se eliminará.");
  }
  // Verificar si el archivo existe antes de intentar eliminarlo
  if (SD.exists("/steps.json")) {
    if (SD.remove("/steps.json")) {
      Serial.println("Archivo steps.json eliminado");
    } else {
      Serial.println("Error al eliminar el archivo steps.json");
    }
  } else {
    Serial.println("Archivo steps.json no encontrado, no se eliminará.");
  }
  // Configuración del tiempo NTP
  configTime(0, 0, "pool.ntp.org"); // Configura el servidor NTP y la zona horaria UTC
}






void loop() {
   unsigned long currentMillis = millis();

  // Verificar el estado de la conexión WiFi
  if (WiFi.status() == WL_CONNECTED) {
    digitalWrite(greenLED, HIGH); // Enciende el LED si está conectado
  } else {
    digitalWrite(greenLED, LOW); // Apaga el LED si no está conectado
  }


  // Manejo del GPS
  handleGPS();

  // Leer el valor del sensor cada 5 segundos
  if (currentMillis - lastReadTime >= readInterval) {
    lastReadTime = currentMillis;
    handleLPM();
  }

  // Guardar el promedio cada minuto
  if (currentMillis - lastAverageTime >= averageInterval) {
    lastAverageTime = currentMillis;
    saveAverageLPM();
  }

  // Enviar datos a MQTT cada 40 segundos
  if (currentMillis - lastSendTimeMQTT > sendIntervalMQTT) {
    sendAllSDDataToMQTT();
    lastSendTimeMQTT = millis();
  }

  // Detectar pasos
  detectSteps();




    // Verificar y reconectar MQTT si es necesario
  if (!client.connected()) {
    reconnect();
  } else {
    client.loop(); // Mantener la conexión MQTT activa
  }

  // Enviar datos a MQTT cada minuto
  if (currentMillis - lastSendTimeMQTT >= 60000) { // 60000 ms = 1 minuto
    sendAllSDDataToMQTT();
    lastSendTimeMQTT = currentMillis; // Actualizar el temporizador
  }
}

void setup_wifi() {
  delay(1000);
  Serial.print("Conectando a: ");
  Serial.print(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi conectado");
  Serial.print("Dirección IP: ");
  Serial.println(WiFi.localIP());
}

void reconnect() {
  const int maxAttempts = 5;  // Número máximo de intentos de conexión
  int attempt = 0;

  while (!client.connected() && attempt < maxAttempts) {
    Serial.print("Conectando a MQTT...");
    
    if (client.connect("ESP32Client", mqtt_user, mqtt_password)) {
      Serial.println("Conectado!");
      return; // Salir de la función si la conexión es exitosa
    } else {
      Serial.print("Error: ");
      Serial.print(client.state());
      Serial.println(" Intentando nuevamente en 5 segundos.");
      attempt++;
      delay(5000);  // Esperar 5 segundos antes de intentar nuevamente
    }
  }

  if (attempt >= maxAttempts) {
    Serial.println("Número máximo de intentos alcanzado. Continuando sin conexión MQTT.");
  }
}


void enviarMensajeMQTT(const char* topic, const String& message) {
  if (client.connected()) {
    client.publish(topic, message.c_str());
    Serial.println("Mensaje MQTT enviado");
  } else {
    // Desactivado temporalmente
    // Serial.println("No conectado a MQTT. Intentando reconectar...");
    // reconnect();
  }
}

String getCurrentTimestamp() {
  time_t now = time(nullptr);
  struct tm* timeinfo = localtime(&now);
  char buffer[20];
  strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", timeinfo);
  return String(buffer);
}

void handleGPS() {
  boolean newData = false;
  unsigned long start = millis();
  while (millis() - start < 1000) {
    while (mechabullgps.available()) {
      if (gps.encode(mechabullgps.read())) {
        newData = true;
      }
    }
  }

  if (newData) {
    double currentLat = gps.location.lat();
    double currentLng = gps.location.lng();
    String timestamp = getCurrentTimestamp();

    if (millis() - lastSaveTimeGPS > saveIntervalGPS) {
      StaticJsonDocument<256> newDoc;
      newDoc["ID"] = dispo;
      newDoc["type"] = gpsType;
      newDoc["latitude"] = gps.location.lat();
      newDoc["longitude"] = gps.location.lng();
      newDoc["timestamp"] = timestamp;

      saveJsonToFile(newDoc, "/gps_data.json");

      unsigned long startTime = millis();
      unsigned long duration = 3000;
      while(millis() - startTime  < duration){
        digitalWrite(yellowLED, HIGH);
        delay(500);
        digitalWrite(yellowLED, LOW);
        delay(500);
      }

      lastSaveTimeGPS = millis();
    }
  } else {
    Serial.println("Sin señal GPS");
  }
}

void saveJsonToFile(JsonDocument& doc, const char* filename) {
  File file = SD.open(filename, FILE_APPEND);
  if (!file) {
    Serial.print("Error al abrir el archivo ");
    Serial.println(filename);
    return;
  }

  if (file.size() == 0) {
    file.print("[");
  } else {
    file.print(",");
  }

  String output;
  serializeJsonPretty(doc, output);
  file.print(output);

  file.close();
  Serial.print("Datos escritos en el archivo ");
  Serial.println(filename);
}

void closeJsonArray(const char* filename) {
  File file = SD.open(filename, FILE_WRITE);
  if (!file) {
    Serial.print("Error al abrir el archivo para cerrar el arreglo ");
    Serial.println(filename);
    return;
  }

  file.seek(file.size() - 1);

  char lastChar = file.read();
  if (lastChar != '}') {
    file.print('}');
  }
  char lastCharr = file.read();
  if (lastCharr != ']') {
    file.print(']');
  }
  file.close();
  Serial.print("Arreglo JSON cerrado en el archivo ");
  Serial.println(filename);
}








void sendFileToMQTT(const char* filename, const char* topic) {
  File file = SD.open(filename);
  if (!file) {
    Serial.print("Error al abrir el archivo ");
    Serial.println(filename);
    return;
  }

  const size_t bufferSize = 4548;  // Ajusta el tamaño del buffer según tus necesidades
  char buffer[bufferSize];
  String jsonLine;
  bool inObject = false;
  bool isFirstObject = true;
  bool isGPSData = false;

  // Leer y enviar datos
 while (file.available()) {
    size_t bytesRead = file.readBytes(buffer, bufferSize);
    for (size_t i = 0; i < bytesRead; ++i) {
      char ch = buffer[i];
      if (ch == '{') {
        inObject = true;
        jsonLine = "{";
      } else if (ch == '}') {
        if (inObject) {
          jsonLine += "}";
          // Enviar el objeto JSON a MQTT
           enviarMensajeMQTT(topic, jsonLine);

           unsigned long startTime = millis();
            unsigned long duration = 3000;
          while(millis() - startTime  < duration){
            digitalWrite(greenLED, HIGH);
            delay(500);
            digitalWrite(greenLED, LOW);
            delay(500);
          }

          // Imprimir el objeto JSON en consola para depuración
          Serial.println("Fragmento enviado:");
          Serial.println(jsonLine);

          // Terminar el objeto JSON
          jsonLine = "";
          inObject = false;
        }
      } else if (inObject) {
        jsonLine += ch;
      }
    }
  }


  file.close();

  // Eliminar el archivo después de enviarlo
  if (!SD.remove(filename)) {
    Serial.print("Error al eliminar el archivo ");
    Serial.println(filename);
  } else {
    Serial.print("Archivo ");
    Serial.print(filename);
    Serial.println(" eliminado");
  }
}
void sendAllSDDataToMQTT() {
  closeJsonArray("/gps_data.json");
  closeJsonArray("/lpm_data.json");
  delay(2000);  // Retraso de 2 segundos (2000 ms)

  sendFileToMQTT("/gps_data.json", "mb.divice.mqtt");
  delay(2000);  // Retraso de 2 segundos (2000 ms)

  sendFileToMQTT("/lpm_data.json", "mb.divice.mqtt");
  delay(2000);  // Retraso de 2 segundos (2000 ms)

  sendFileToMQTT("/steps.json", "mb.divice.mqtt");
  delay(2000);  // Retraso de 2 segundos (2000 ms)

  Serial.println("Todos los datos de SD enviados a MQTT");
}







void handleLPM() {
  int valorSensor = analogRead(SENSOR_PIN);
  int bpm = map(valorSensor, valorSensor_min, valorSensor_max, bpm_min, bpm_max);

  if (bpm >= bpm_min && bpm <= bpm_max) {
    totalReadings += bpm;
    numReadings++;
  } else {
    Serial.println("Lectura de sensor fuera del rango esperado");
  }
}

void saveAverageLPM() {
  if (numReadings > 0) {
    float averageBPM = totalReadings / numReadings;
    totalReadings = 0;
    numReadings = 0;

    StaticJsonDocument<256> doc;
    doc["ID"] = dispo;
    doc["type"] = lpmType;
    doc["timestamp"] = getCurrentTimestamp();
    doc["average_bpm"] = averageBPM;
    saveJsonToFile(doc, "/lpm_data.json");
     unsigned long startTime = millis();
      unsigned long duration = 3000;
      while(millis() - startTime  < duration){
        digitalWrite(yellowLED, HIGH);
        delay(500);
        digitalWrite(yellowLED, LOW);
        delay(500);
      }
  } else {
    Serial.println("No hay lecturas para calcular el promedio");
  }
}

void detectSteps() {
  int16_t ax, ay, az;
  int16_t gx, gy, gz;

  // Leer datos de aceleración y giroscopio
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Aplicar un filtro simple (promedio móvil)
  static int16_t prevAz = 0;
  int16_t filteredAz = (prevAz + az) / 2;
  prevAz = az;

  // Detectar pasos utilizando la aceleración en el eje Z filtrada
  if (abs(filteredAz) > threshold && !stepDetected) {
    stepDetected = true;
    stepCount++;
    Serial.print("Conteo de pasos: ");
    Serial.println(stepCount);

    // Crear objeto JSON con stepCount y timestamp
    StaticJsonDocument<256> doc;
    doc["ID"] = dispo;
    doc["type"] = stephsType;
    doc["stepCount"] = stepCount;
    doc["timestamp"] = getCurrentTimestamp(); // Agregar timestamp

    unsigned long startTime = millis();
      unsigned long duration = 3000;
      while(millis() - startTime  < duration){
        digitalWrite(yellowLED, HIGH);
        delay(500);
        digitalWrite(yellowLED, LOW);
        delay(500);
      }

    // Convertir JSON a string
    String output;
    serializeJson(doc, output);

    // Guardar los datos en la tarjeta SD (reescribiendo el archivo)
    File myFile = SD.open("/steps.json", FILE_WRITE);
    if (!myFile) {
      Serial.println("Error al abrir archivo");
      return;
    }
    myFile.println(output);
    myFile.close();
    Serial.println("Datos escritos en el archivo JSON");
  } else if (abs(filteredAz) < threshold) {
    stepDetected = false;
  }

  delay(100);  // Ajustar según sea necesario
}


