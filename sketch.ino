#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "DHTesp.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

#define LED_PIN 26      // Pino de controle do NeoPixel
#define NUMPIXELS 1     // Número de LEDs no strip
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUMPIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);

const int DHT_PIN = 21; // Pino do sensor DHT
DHTesp dhtSensor;

#define BUTTON_PIN 13   // Pino do pushbutton
SemaphoreHandle_t buttonSemaphore;

enum LedColor { VERDE, AMARELO, VERMELHO, AZUL };
enum LedState { DESLIGADO, LIGADO, PISCANDO_1S, PISCANDO_0_3S };
LedColor currentColor = VERDE;
LedState currentState = DESLIGADO;

float temperature = 0.0;
float humidity = 0.0;

const char *SSID = "Wokwi-GUEST"; // SSID da rede Wi-Fi
const char *PASSWORD = "";        // Senha da rede Wi-Fi
const char *BROKER_MQTT = "broker.hivemq.com"; // URL do broker MQTT
int BROKER_PORT = 1883; // Porta do Broker MQTT

#define TOPIC_SUBSCRIBE_LED "topic_led_control"
#define TOPIC_PUBLISH "topic_sensor_humidiJM"
#define ID_MQTT "esp32_mqtt"

WiFiClient espClient;
PubSubClient MQTT(espClient);

static char strTemperature[10] = {0};
static char strHumidity[10] = {0};

// Função para iniciar a conexão Wi-Fi
void initWiFi(void)
{
  vTaskDelay(pdMS_TO_TICKS(10));
  Serial.println("------Conexao WI-FI------");
  Serial.print("Conectando-se na rede: ");
  Serial.println(SSID);
  Serial.println("Aguarde");

  reconnectWiFi();
}

// Função para iniciar a conexão MQTT
void initMQTT(void)
{
  MQTT.setServer(BROKER_MQTT, BROKER_PORT); // Configura o broker e porta MQTT
  MQTT.setCallback(callbackMQTT); // Atribui função de callback para quando uma mensagem chega
}

// Função de callback chamada quando uma mensagem chega
void callbackMQTT(char *topic, byte *payload, unsigned int length)
{
  String message;
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  Serial.print("Mensagem recebida no tópico: ");
  Serial.println(topic);
  Serial.print("Mensagem: ");
  Serial.println(message);

  if (strcmp(topic, TOPIC_SUBSCRIBE_LED) == 0) {
    int sepIndex = message.indexOf(';');
    if (sepIndex != -1) {
      int color = message.substring(0, sepIndex).toInt();
      int state = message.substring(sepIndex + 1).toInt();

      if (color >= 0 && color <= 3) {
        currentColor = (LedColor)color;
        setColor(currentColor);
      }

      if (state >= 0 && state <= 3) {
        currentState = (LedState)state;
      }
    }
  }
}

// Função para reconectar ao broker MQTT
void reconnectMQTT(void)
{
  while (!MQTT.connected()) {
    Serial.print("* Tentando se conectar ao Broker MQTT: ");
    Serial.println(BROKER_MQTT);
    if (MQTT.connect(ID_MQTT)) {
      Serial.println("Conectado com sucesso ao broker MQTT!");
      MQTT.subscribe(TOPIC_SUBSCRIBE_LED);
    } else {
      Serial.println("Falha ao reconectar no broker.");
      Serial.println("Nova tentativa de conexao em 2 segundos.");
      vTaskDelay(pdMS_TO_TICKS(2000));
    }
  }
}

// Verifica e reconecta ao Wi-Fi e ao MQTT se necessário
void checkWiFIAndMQTT(void)
{
  if (!MQTT.connected())
    reconnectMQTT(); // Se não há conexão com o Broker, reconecta

  reconnectWiFi(); // Se não há conexão com o Wi-Fi, reconecta
}

// Função para reconectar ao Wi-Fi
void reconnectWiFi(void)
{
  if (WiFi.status() == WL_CONNECTED)
    return;

  WiFi.begin(SSID, PASSWORD); // Conecta na rede Wi-Fi

  while (WiFi.status() != WL_CONNECTED) {
    vTaskDelay(pdMS_TO_TICKS(100));
    Serial.print(".");
  }

  Serial.println();
  Serial.print("Conectado com sucesso na rede ");
  Serial.print(SSID);
  Serial.println("IP obtido: ");
  Serial.println(WiFi.localIP());
}

// Função para definir a cor do LED
void setColor(LedColor color) {
  switch(color) {
    case VERDE:
      strip.setPixelColor(0, strip.Color(0, 255, 0));
      break;
    case AMARELO:
      strip.setPixelColor(0, strip.Color(255, 255, 0));
      break;
    case VERMELHO:
      strip.setPixelColor(0, strip.Color(255, 0, 0));
      break;
    case AZUL:
      strip.setPixelColor(0, strip.Color(0, 0, 255));
      break;
  }
  strip.show();
}

// Função para desligar o LED
void turnOff() {
  strip.setPixelColor(0, strip.Color(0, 0, 0));
  strip.show();
}

// Task para controlar o LED
void ledTask(void *pvParameters) {
  for (;;) {
    switch (currentState) {
      case DESLIGADO:
        turnOff();
        vTaskDelay(pdMS_TO_TICKS(100));
        break;
      case LIGADO:
        setColor(currentColor);
        vTaskDelay(pdMS_TO_TICKS(100));
        break;
      case PISCANDO_1S:
        setColor(currentColor);
        vTaskDelay(pdMS_TO_TICKS(1000));
        turnOff();
        vTaskDelay(pdMS_TO_TICKS(1000));
        break;
      case PISCANDO_0_3S:
        setColor(currentColor);
        vTaskDelay(pdMS_TO_TICKS(300));
        turnOff();
        vTaskDelay(pdMS_TO_TICKS(300));
        break;
    }
  }
}

// Task para gerenciar o botão
void buttonTask(void *pvParameters) {
  TickType_t buttonPressTime = 0;
  while (1) {
    if (xSemaphoreTake(buttonSemaphore, pdMS_TO_TICKS(10)) == pdTRUE) {
      if (digitalRead(BUTTON_PIN) == LOW) { // Botão pressionado
        buttonPressTime = xTaskGetTickCount();
      } else { // Botão liberado
        TickType_t buttonReleaseTime = xTaskGetTickCount();
        TickType_t buttonPressDuration = buttonReleaseTime - buttonPressTime;
        if (buttonPressDuration < pdMS_TO_TICKS(1000)) { // Pressionamento rápido
          currentState = (LedState)((currentState + 1) % 4);
        } else if (buttonPressDuration > pdMS_TO_TICKS(3000)) { // Pressionamento longo
          currentColor = (LedColor)((currentColor + 1) % 4);
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// Task para ler dados do sensor DHT
void DhtTask(void*parametro)
{
  while(1)
  {
    TempAndHumidity data = dhtSensor.getTempAndHumidity();
    temperature = data.temperature;
    humidity = data.humidity;
    vTaskDelay(pdMS_TO_TICKS(2000));
  }
  vTaskDelete(NULL);
}

// Task para reportar dados via MQTT
void ReportTask(void *pvParameters) {
  (void) pvParameters;
  String cor = "";
  String status = "";

  for (;;) {
    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.print(" *C, Humidity: ");
    Serial.print(humidity);
    Serial.print("%, LED Color: ");

    switch(currentColor) {
      case VERDE:
        Serial.print("Verde");
        cor = "Verde";
        break;
      case AMARELO:
        Serial.print("Amarelo");
        cor = "Amarelo";
        break;
      case VERMELHO:
        Serial.print("Vermelho");
        cor = "Vermelho";
        break;
      case AZUL:
        Serial.print("Azul");
        cor = "Azul";
        break;
      default:
        Serial.print("Desconhecida");
        cor = "Desconhecida";
        break;
    }

    Serial.print(", LED State: ");
    
    switch (currentState) {
      case DESLIGADO:
        Serial.print("DESLIGADO");
        status = "DESLIGADO";
        break;
      case LIGADO:
        Serial.print("LIGADO");
        status = "LIGADO";
        break;
      case PISCANDO_1S:
        Serial.print("PISCANDO_1S");
        status = "PISCANDO_1S";
        break;
      case PISCANDO_0_3S:
        Serial.print("PISCANDO_0_3S");
        status = "PISCANDO_0_3S";
        break;
    }
    Serial.println(" ");

    StaticJsonDocument<256> doc;
    doc["temperature"] = temperature;
    doc["humidity"] = humidity;

    JsonObject ledStatus = doc.createNestedObject("ledReport");
    ledStatus["cor"] = cor.c_str();
    ledStatus["status"] = status.c_str();

    char jsonBuffer[256];
    serializeJson(doc, jsonBuffer);
    MQTT.publish(TOPIC_PUBLISH, jsonBuffer);

    vTaskDelay(pdMS_TO_TICKS(3000)); // Aguarda 3 segundos antes de imprimir o próximo relatório
  }
}

// Função de interrupção para tratar o botão
void IRAM_ATTR handleButtonPress() {
  xSemaphoreGiveFromISR(buttonSemaphore, NULL);
}

void setup() {
  Serial.begin(115200);

  Serial.println("Conectando-se à rede WiFi...");

  initWiFi();   // Inicializa a conexão Wi-Fi
  initMQTT();   // Inicializa a conexão ao broker MQTT

  strip.begin();
  strip.show(); // Inicializa todos os pixels como 'desligado'

  dhtSensor.setup(DHT_PIN, DHTesp::DHT22);

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), handleButtonPress, CHANGE);
  buttonSemaphore = xSemaphoreCreateBinary();

  // Cria a task do LED
  xTaskCreatePinnedToCore(
    ledTask,
    "LedTask",
    1000,
    NULL,
    1,
    NULL,
    1
  );

  // Cria a task do sensor DHT
  xTaskCreatePinnedToCore(
    DhtTask,
    "DHT22",
    1000,
    NULL,
    1,
    NULL,
    1
  );

  // Cria a task do botão
  xTaskCreatePinnedToCore(
    buttonTask,
    "ButtonTask",
    1000,
    NULL,
    1,
    NULL,
    0
  );

  // Cria a task de relatório
  xTaskCreatePinnedToCore(
    ReportTask,
    "ReportTask",
    4096,
    NULL,
    1,
    NULL,
    0
  );

  // Configurações iniciais do LED
  currentState = DESLIGADO;
}

void loop() {
  checkWiFIAndMQTT();
  MQTT.loop();
  vTaskDelay(pdMS_TO_TICKS(2000));
}
