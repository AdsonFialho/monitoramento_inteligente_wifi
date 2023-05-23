// ######################################## Bibliotecas Utilizadas  ######################################## 
#ifdef ESP32
  #include <WiFi.h>               // Biblioteca para comunicação WiFi utilizando o ESP32
#else
  #include <ESP8266WiFi.h>        // Biblioteca para comunicação WiFi utilizando o ESP8266
#endif
#include <PubSubClient.h>         // Biblioteca para comunicação com MQTT Server (Utilizado o Broker Mosquitto)
#include <ESPmDNS.h>              // Biblioteca para criar a configuração DNS (não ha necessidade de saber o IP do Servidor MQTT)
#include <Wire.h>                 // Biblioteca para comunicação I2C
#include <Adafruit_BME280.h>      // Biblioteca com funções de leitura dos dados pelo Sensor BME280
#include <Adafruit_Sensor.h>

// ######################################## Configuração de Acesso  ######################################## 
const char* ssid     = "DIGITE O SSID DA REDE WIFI A SER UTILIZADA";
const char* password = "#DIGITA A SENHA DA REDE WIFI";
const char* mqttHostname = "rpi-inteligente";
const int mqttPort = 1883;
const char* mqttUser = "DIGITE O USUARIO PARA ACESSO AO MQTT";
const char* mqttPassword = "DIGITE A SENHA PARA ACESSO AO MQTT";

// ######################################## Inicialização das Classes das Bibliotecas utilizadas ######################################## 
WiFiClient espClient;
PubSubClient client(espClient);
Adafruit_BME280 bme; // I2C
#define SEALEVELPRESSURE_HPA (1013.25)

// ######################################## Inicialização das Variaveis ######################################## 
long lastMsg = 0;
char msg[50];
int value = 0;

float temperatureC = 0;
float temperatureF = 0;
float humidity = 0;
float pressure = 0;
float altitude = 0;

int valueLightSensor = 0;
int valueRainSensor = 0;

unsigned long previousMillis = 0; 
const long interval = 15000;        // intevalo de envio dos dados ao MQTT

// Portas do microcontrolador ESP32 DevKit v1
const int ledPin = 2;
const int pinLightSensor = 34;
const int pinRainSensor = 35;

// ######################################## Função para Receber mensagens do MQTT Server  ######################################## 
void callback(char* topic, byte* message, unsigned int lenght) {
  Serial.print("Mensagem recebida no topico: ");
  Serial.print(topic);
  Serial.print(". Menssagem: ");
  String messageTemp;
  
  for (int i = 0; i<lenght; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }

  Serial.println();

  if (String(topic) == "esp32/output") {
    Serial.print("Mudando o estado do led para: ");
    if(messageTemp == "on"){
      Serial.println("ligado");
      digitalWrite(ledPin, HIGH);
    }
    else if(messageTemp == "off"){
      Serial.println("desligado");
      digitalWrite(ledPin, LOW);
    }
  }
}

 // ######################################## Função para Conectar e/ou Reconectar ao Servidor MQTT  ######################################## 
void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Conectando ao MQTT Server... ");
    // Attempt to connect
    if (client.connect("ESP32Client", mqttUser, mqttPassword)) {
      Serial.println("Conectado!");
      Serial.println();
      // Subscribe
      client.subscribe("esp32/output");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" tentando novamente em 5 segundos");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

// ######################################## Leitura dos Sensores e Envio para o MQTT Server ########################################

void getTemperatureC() {

  // Temperature in Celsius
  temperatureC = bme.readTemperature();
    
  // Convert the value to a char array
  char tempCString[8];
  dtostrf(temperatureC, 1, 2, tempCString);
  Serial.print("Temperatura: ");
  Serial.print(tempCString);
  Serial.println(" oC");
  client.publish("esp32/temperatureC", tempCString);
}

void getTemperatureF() {

  // Temperature in Fahrenheit
  temperatureF = 1.8 * bme.readTemperature() + 32; 

  // Convert the value to a char array
  char tempFString[8];
  dtostrf(temperatureF, 1, 2, tempFString);
  Serial.print("Temperatura: ");
  Serial.print(tempFString);
  Serial.println(" oF");
  client.publish("esp32/temperatureF", tempFString);
}

void getHumidity() {

  // Fetch Humidity
  humidity = bme.readHumidity();
    
  // Convert the value to a char array
  char humString[8];
  dtostrf(humidity, 1, 2, humString);
  Serial.print("Humidade: ");
  Serial.print(humString);
  Serial.println(" %");
  client.publish("esp32/humidity", humString);
}

void getPressure() {

  // Fetch Pressure
  pressure = bme.readPressure() / 100.0F;

  // Convert the value to a char array
  char presString[8];
  dtostrf(pressure, 1, 2, presString);
  Serial.print("Pressão: ");
  Serial.print(presString);
  Serial.println(" hPa");
  client.publish("esp32/pressure", presString);
}

void getAltitude() {

  // Fetch Altiture
  altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);

  // Convert the value to a char array
  char altString[8];
  dtostrf(altitude, 1, 2, altString);
  Serial.print("Altitude: ");
  Serial.print(altString);
  Serial.println(" m");
  client.publish("esp32/altitude", altString);
}

void getLightSensor() {

  valueLightSensor = digitalRead(pinLightSensor);

  if (valueLightSensor == LOW) {
    Serial.print("Sensor de Luz: Dia ");
    //Serial.print(valueLightSensor);
    Serial.println();
    client.publish("esp32/lightSensor", "Dia");
  } else {
    Serial.print("Sensor de Luz: Noite ");
    //Serial.print(valueLightSensor);
    Serial.println();
    client.publish("esp32/lightSensor", "Noite");
  }
}

void getRainSensor() {

  valueRainSensor = digitalRead(pinRainSensor);

  if (valueRainSensor == HIGH) {
    Serial.print("Sensor de Chuva: Tempo Seco ");
    // Serial.print(valueRainSensor);
    Serial.println();
    client.publish("esp32/rainSensor", "Seco");
  } else {
    Serial.print("Sensor de Chuva: Chuva ");
    // Serial.print(valueRainSensor);
    Serial.println();
    client.publish("esp32/rainSensor", "Chuva");
  }

  Serial.println();

}

void initBME(){
  if (!bme.begin(0x76)) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }
}

// ######################################## SETUP ########################################

void setup() {
  pinMode(pinLightSensor, INPUT);
  pinMode(pinRainSensor, INPUT);
  Serial.begin(115200);
  initBME();
  delay(10);
  Serial.println();
  Serial.print("Conectando a ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while(WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.println("Wifi Conectado!");
  Serial.print("Endereço IP: ");
  Serial.println(WiFi.localIP());

  /*setup MDNS for ESP32 */
  if (!MDNS.begin("esp32")) {
    Serial.println("Error setting up MDNS responder!");
    while(1) {
      delay(1000);
    }
  }

  /* get the IP address of server by MDNS name */
  Serial.println("mDNS responder started");
  IPAddress serverIp = MDNS.queryHost(mqttHostname);
  Serial.print("IP address of server: ");
  Serial.println(serverIp.toString());
  /* configure the MQTT server with IPaddress and port */
  client.setServer(serverIp, mqttPort);
  /* this receivedCallback function will be invoked when client received subscribed topic */
  client.setCallback(callback);

  pinMode(ledPin, OUTPUT);
}


// ######################################## LOOP ########################################

void loop() {
  /* if client was disconnected then try to reconnect again */
  if (!client.connected()) {
    reconnect();
  }
  /* this function will listen for incomming subscribed topic-process-invoke receivedCallback */
  client.loop();

  // ######################################## Rotina de Leitura/Envio dos Dados ao Servidor MQTT ########################################
  long now = millis();
  if (now - lastMsg > interval) {
    lastMsg = now;

    // Fetch Temperature in Celsius
    getTemperatureC();
    
    // Fetch Temperature in Fahrenheit
    getTemperatureF();
    
    // Fetch Humidity
    getHumidity();

    // Fetch Pressure
    getPressure();

    // Fetch Altiture
    getAltitude();

    // Fetch Light Sensor
    getLightSensor();

    // Fetch Rain Sensor
    getRainSensor();

  }
}
