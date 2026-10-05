// ============================================================
// Codi de https://github.com/UsernameDZ/Research-project-examples/
// Llicència: MIT License
//
// Comentaris del codi generats amb la IA (ChatGPT)
//
// DOIT ESP32 DEVKIT V1 (WROOM)
// ID de la placa: 2
//
// Sensors:
// AHT20 + BMP280 #1:
// SDA - GPIO18
// SCL - GPIO19
// VDD - 3.3V
// GND - GND
//
// AHT20 + BMP280 #2:
// SDA - GPIO25
// SCL - GPIO26
// VDD - 3.3V
// GND - GND
//
// Relés:
// GND - GND
// IN1 - GPIO32
// IN2 - GPIO33
// VCC - 5V
//
// Servomotor:
// GND - GND
// VCC - 5V
// SIGNAL - GPIO13
// ============================================================


#include <Arduino.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <EEPROM.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_AHTX0.h>
#include <ESP32Servo.h>


// ============================================================
// CONFIGURACIÓ DELS PINS I2C
// ============================================================

// Bus I2C #1
#define SDA_1 18
#define SCL_1 19

// Bus I2C #2
#define SDA_2 25
#define SCL_2 26

/*
TwoWire I2Cone = TwoWire(0);
TwoWire I2Ctwo = TwoWire(1);
*/

// Utilitza el bus I2C predeterminat per al primer parell de sensors
#define I2Cone Wire

// Crea un segon bus I2C per al segon parell de sensors
TwoWire I2Ctwo(1);


// ============================================================
// OBJECTES DELS SENSORS
// ============================================================

// BMP280 i AHT20 al bus I2C #1
Adafruit_BMP280 bmp1(&I2Cone);
Adafruit_AHTX0 aht1;

// BMP280 i AHT20 al bus I2C #2
Adafruit_BMP280 bmp2(&I2Ctwo);
Adafruit_AHTX0 aht2;

// Objecte del servomotor
Servo myServo;


// ============================================================
// CONFIGURACIÓ DELS PINS DE SORTIDA
// ============================================================

#define RELAY1_PIN 32
#define RELAY2_PIN 33
#define SERVO_PIN 13


// ============================================================
// CONFIGURACIÓ DE LA PLACA ESP-NOW
// ============================================================

// ID d'aquesta placa ESP32
#define BOARD_ID 2

// Canal Wi-Fi màxim utilitzat per a l'aparellament ESP-NOW
// L'11 s'utilitza habitualment a Amèrica del Nord
// El 13 està disponible a Europa
#define MAX_CHANNEL 13


// Adreça MAC del servidor
// FF:FF:FF:FF:FF:FF s'utilitza inicialment per a l'aparellament
uint8_t serverAddress[] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

// Adreça MAC d'aquesta placa ESP32
uint8_t clientMacAddress[6];


// ============================================================
// ESTRUCTURA DE DADES
// ============================================================

// Estructura utilitzada per a la comunicació entre les plaques ESP32.
// L'estructura ha de ser idèntica al transmissor i al receptor.
typedef struct struct_message {
  uint8_t msgType;
  uint8_t id;
  float tempInternal;

  // Parell de sensors AHT20 + BMP280 #1
  float tempAHT20_1;
  float humidityAHT20_1;
  float pressureBMP280_1;
  float tempBMP280_1;
  float altitudeBMP280_1;

  // Parell de sensors AHT20 + BMP280 #2
  float tempAHT20_2;
  float humidityAHT20_2;
  float pressureBMP280_2;
  float tempBMP280_2;
  float altitudeBMP280_2;

  // Camps de dades utilitzats per l'altra placa ESP32
  float tempAHT21;
  float humidityAHT21;
  float AQIENS160;
  float TVOCENS160;
  float eCO2ENS160;

  // Valors dels sensors analògics i digitals
  int MQ2;
  int MQ135;
  int hall;

  // Estat del sensor de moviment
  bool motion;

  // Estats dels relés
  bool relay1;
  bool relay2;

  // Posició del servo
  int servo;

  // Comptador de lectures
  unsigned int readingId;
} struct_message;


// Estructura utilitzada durant l'aparellament ESP-NOW
typedef struct struct_pairing {
    uint8_t msgType;
    uint8_t id;
    uint8_t macAddr[6];
    uint8_t channel;
} struct_pairing;


// Informació del dispositiu ESP-NOW
esp_now_peer_info_t peer;


// Estructura de dades de sortida
struct_message myData;

// Estructura de dades d'entrada
struct_message inData;

// Estructura de dades utilitzada per a l'aparellament
struct_pairing pairingData;


// ============================================================
// ESTAT DE L'APARELLAMENT ESP-NOW
// ============================================================

enum PairingStatus {
  NOT_PAIRED,
  PAIR_REQUEST,
  PAIR_REQUESTED,
  PAIR_PAIRED,
};

PairingStatus pairingStatus = NOT_PAIRED;


// Tipus de missatges utilitzats per ESP-NOW
enum MessageType {
  PAIRING,
  DATA,
};

MessageType messageType;


#ifdef SAVE_CHANNEL
  // Desa l'últim canal Wi-Fi utilitzat
  int lastChannel;
#endif


// Canal actual utilitzat per a l'aparellament ESP-NOW
int channel = 1;


// ============================================================
// VARIABLES DE TEMPS
// ============================================================

// Simula dades de temperatura i humitat
float t = 0;

// Desa el temps actual
unsigned long currentMillis = millis();

// Desa el temps de l'execució anterior
unsigned long previousMillis = 0;

// Interval entre les transmissions dels sensors
const long interval = 10000;

// S'utilitza per mesurar el temps necessari per a l'aparellament
unsigned long start;

// Comptador de les lectures dels sensors
unsigned int readingId = 0;


// ============================================================
// LECTURA DE L'ADREÇA MAC DE L'ESP32
// ============================================================

void readGetMacAddress(){

  uint8_t baseMac[6];

  // Llegeix l'adreça MAC de la interfície d'estació de l'ESP32
  esp_err_t ret = esp_wifi_get_mac(WIFI_IF_STA, baseMac);

  if (ret == ESP_OK) {

    // Mostra l'adreça MAC al monitor sèrie
    Serial.printf("%02x:%02x:%02x:%02x:%02x:%02x\n",
                  baseMac[0], baseMac[1], baseMac[2],
                  baseMac[3], baseMac[4], baseMac[5]);

  } else {

    Serial.println("Failed to read MAC address");
  }

  // Copia l'adreça MAC a l'array global
  clientMacAddress[0] = baseMac[0];
  clientMacAddress[1] = baseMac[1];
  clientMacAddress[2] = baseMac[2];
  clientMacAddress[3] = baseMac[3];
  clientMacAddress[4] = baseMac[4];
  clientMacAddress[5] = baseMac[5];
}


// ============================================================
// AFEGEIX UN DISPOSITIU ESP-NOW
// ============================================================

void addPeer(const uint8_t * mac_addr, uint8_t chan){

  // Estableix el canal Wi-Fi utilitzat per ESP-NOW
  ESP_ERROR_CHECK(esp_wifi_set_channel(chan ,WIFI_SECOND_CHAN_NONE));

  // Elimina el dispositiu si ja existeix
  esp_now_del_peer(mac_addr);

  // Neteja l'estructura del dispositiu
  memset(&peer, 0, sizeof(esp_now_peer_info_t));

  // Configura el dispositiu
  peer.channel = chan;
  peer.encrypt = false;

  // Copia l'adreça MAC del dispositiu
  memcpy(peer.peer_addr, mac_addr, sizeof(uint8_t[6]));

  // Afegeix el dispositiu a la llista de dispositius ESP-NOW
  if (esp_now_add_peer(&peer) != ESP_OK){

    Serial.println("Failed to add peer");
    return;
  }

  // Desa l'adreça MAC del servidor
  memcpy(serverAddress, mac_addr, sizeof(uint8_t[6]));
}


// ============================================================
// MOSTRA L'ADREÇA MAC
// ============================================================

void printMAC(const uint8_t * mac_addr){

  char macStr[18];

  // Converteix l'adreça MAC en text imprimible
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2],
           mac_addr[3], mac_addr[4], mac_addr[5]);

  Serial.print(macStr);
}


// ============================================================
// CALLBACK D'ENVIAMENT ESP-NOW
// ============================================================

// Es crida després d'enviar un paquet ESP-NOW
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {

  Serial.print("\r\nLast Packet Send Status:\t");

  // Mostra si el paquet s'ha lliurat correctament
  Serial.println(
    status == ESP_NOW_SEND_SUCCESS
    ? "Delivery Success"
    : "Delivery Fail"
  );
}


// ============================================================
// CALLBACK DE RECEPCIÓ ESP-NOW
// ============================================================

// Es crida cada vegada que es rep un paquet ESP-NOW
void OnDataRecv(
  const uint8_t * mac_addr,
  const uint8_t *incomingData,
  int len
) {

  Serial.print("Packet received with ");
  Serial.print("data size = ");

  Serial.println(sizeof(incomingData));

  // El primer byte conté el tipus de missatge
  uint8_t type = incomingData[0];

  switch (type) {

    // --------------------------------------------------------
    // MISSATGE DE DADES
    // --------------------------------------------------------

    case DATA:

      // Copia les dades rebudes a l'estructura de dades d'entrada
      memcpy(&inData, incomingData, sizeof(inData));

      Serial.print("ID = ");
      Serial.println(inData.id);

      Serial.print("Relay 1 = ");
      Serial.println(inData.relay1);

      Serial.print("Relay 2 = ");
      Serial.println(inData.relay2);

      Serial.print("Servo Angle = ");
      Serial.println(inData.servo);

      Serial.print("Setpoint temp = ");
      Serial.println(inData.tempInternal);

      Serial.print("reading Id  = ");
      Serial.println(inData.readingId);


      // Les sortides dels relés són actives en LOW
      digitalWrite(
        RELAY1_PIN,
        inData.relay1 ? LOW : HIGH
      );

      digitalWrite(
        RELAY2_PIN,
        inData.relay2 ? LOW : HIGH
      );


      // Estableix l'angle del servo
      myServo.write(inData.servo);


      // Canvia el LED integrat segons l'ID de lectura
      if (inData.readingId % 2 == 1){

        digitalWrite(LED_BUILTIN, LOW);

      } else {

        digitalWrite(LED_BUILTIN, HIGH);
      }

      break;


    // --------------------------------------------------------
    // MISSATGE D'APARELLAMENT
    // --------------------------------------------------------

    case PAIRING:

      // Copia la informació de l'aparellament rebuda
      memcpy(
        &pairingData,
        incomingData,
        sizeof(pairingData)
      );

      // L'ID 0 identifica el servidor
      if (pairingData.id == 0) {

        Serial.print("Pairing done for MAC Address: ");

        printMAC(pairingData.macAddr);

        Serial.print(" on channel ");

        // Mostra el canal seleccionat pel servidor
        Serial.print(pairingData.channel);

        Serial.print(" in ");

        // Mostra el temps necessari per a l'aparellament
        Serial.print(millis()-start);

        Serial.println("ms");


        // Afegeix el servidor a la llista de dispositius ESP-NOW
        addPeer(
          pairingData.macAddr,
          pairingData.channel
        );


        #ifdef SAVE_CHANNEL

          // Desa el canal a l'EEPROM
          lastChannel = pairingData.channel;

          EEPROM.write(
            0,
            pairingData.channel
          );

          EEPROM.commit();

        #endif


        // Marca la placa com a aparellada
        pairingStatus = PAIR_PAIRED;
      }

      break;
  }
}


// ============================================================
// APARELLAMENT AUTOMÀTIC ESP-NOW
// ============================================================

PairingStatus autoPairing(){

  switch(pairingStatus) {


    // --------------------------------------------------------
    // ENVIA LA SOL·LICITUD D'APARELLAMENT
    // --------------------------------------------------------

    case PAIR_REQUEST:

      Serial.print("Pairing request on channel  ");
      Serial.println(channel);


      // Estableix el canal Wi-Fi actual
      ESP_ERROR_CHECK(
        esp_wifi_set_channel(
          channel,
          WIFI_SECOND_CHAN_NONE
        )
      );


      // Inicialitza ESP-NOW
      if (esp_now_init() != ESP_OK) {

        Serial.println("Error initializing ESP-NOW");
      }


      // Registra els callbacks d'ESP-NOW
      esp_now_register_send_cb(
        esp_now_send_cb_t(OnDataSent)
      );

      esp_now_register_recv_cb(
        esp_now_recv_cb_t(OnDataRecv)
      );


      // Prepara la informació de l'aparellament
      pairingData.msgType = PAIRING;

      pairingData.id = BOARD_ID;

      pairingData.channel = channel;


      // Copia l'adreça MAC d'aquesta placa
      pairingData.macAddr[0] = clientMacAddress[0];
      pairingData.macAddr[1] = clientMacAddress[1];
      pairingData.macAddr[2] = clientMacAddress[2];
      pairingData.macAddr[3] = clientMacAddress[3];
      pairingData.macAddr[4] = clientMacAddress[4];
      pairingData.macAddr[5] = clientMacAddress[5];


      // Afegeix el servidor com a dispositiu i envia la sol·licitud d'aparellament
      addPeer(serverAddress, channel);

      esp_now_send(
        serverAddress,
        (uint8_t *) &pairingData,
        sizeof(pairingData)
      );


      // Inicia el temporitzador de temps d'espera de l'aparellament
      previousMillis = millis();

      pairingStatus = PAIR_REQUESTED;

      break;


    // --------------------------------------------------------
    // ESPERA LA RESPOSTA DE L'APARELLAMENT
    // --------------------------------------------------------

    case PAIR_REQUESTED:

      // Comprova quant de temps ha passat
      currentMillis = millis();

      if(currentMillis - previousMillis > 1000) {

        previousMillis = currentMillis;

        // Passa al següent canal Wi-Fi
        channel ++;

        // Torna al canal 1 després d'arribar al màxim
        if (channel > MAX_CHANNEL){

          channel = 1;
        }

        // Envia una altra sol·licitud d'aparellament
        pairingStatus = PAIR_REQUEST;
      }

      break;


    // --------------------------------------------------------
    // APARELLAT
    // --------------------------------------------------------

    case PAIR_PAIRED:

      // No cal fer res mentre està aparellat

      break;
  }

  return pairingStatus;
}


// ============================================================
// CONFIGURACIÓ
// ============================================================

void setup() {

  // Dona una mica de temps a la placa per iniciar-se
  delay(1000);

  // Inicia la comunicació sèrie
  Serial.begin(115200);

  Serial.println();


  // Configura el LED integrat
  pinMode(LED_BUILTIN, OUTPUT);


  // Configura els pins dels relés
  pinMode(RELAY1_PIN, OUTPUT);
  pinMode(RELAY2_PIN, OUTPUT);


  // Apaga inicialment els dos relés
  digitalWrite(RELAY1_PIN, HIGH);
  digitalWrite(RELAY2_PIN, HIGH);


  // Connecta el servo al seu pin GPIO
  myServo.attach(SERVO_PIN);

  // Estableix la posició inicial del servo
  myServo.write(0);


  // Inicialitza el primer bus I2C
  I2Cone.begin(
    SDA_1,
    SCL_1,
    50000
  );


  // Inicialitza el segon bus I2C
  I2Ctwo.begin(
    SDA_2,
    SCL_2,
    50000
  );


  // Estableix els valors de temps d'espera d'I2C
  I2Cone.setTimeOut(500);
  I2Ctwo.setTimeOut(500);


  // ----------------------------------------------------------
  // AHT20 #1
  // ----------------------------------------------------------

  bool status1 = aht1.begin(&I2Cone);

  if(!status1){

    Serial.println(F("Error: AHT20 1"));

    while (1) delay(1000);
  }

  Serial.println(F("AHT20 1 okay"));


  // ----------------------------------------------------------
  // BMP280 #1
  // ----------------------------------------------------------

  bool status2 = bmp1.begin(0x77);

  if(!status2){

    Serial.println(F("Error: BMP280 1"));

    while (1) delay(1000);
  }

  Serial.println(F("BMP280 1 okay"));


  // ----------------------------------------------------------
  // AHT20 #2
  // ----------------------------------------------------------

  bool status3 = aht2.begin(&I2Ctwo);

  if(!status3){

    Serial.println(F("Error: AHT20 2"));

    while (1) delay(1000);
  }

  Serial.println(F("AHT20 2 okay"));


  // ----------------------------------------------------------
  // BMP280 #2
  // ----------------------------------------------------------

  bool status4 = bmp2.begin(0x77);

  if(!status4){

    Serial.println(F("Error: BMP280 2"));

    while (1) delay(1000);
  }

  Serial.println(F("BMP280 2 okay"));


  // ----------------------------------------------------------
  // INICIALITZACIÓ WI-FI / ESP-NOW
  // ----------------------------------------------------------

  // Configura l'ESP32 com a estació Wi-Fi
  WiFi.mode(WIFI_STA);

  WiFi.STA.begin();


  // Mostra l'adreça MAC d'aquesta placa
  Serial.print("Client Board MAC Address:  ");

  readGetMacAddress();


  // Desconnecta el Wi-Fi abans de l'aparellament ESP-NOW
  WiFi.disconnect();


  // Desa el temps d'inici de l'aparellament
  start = millis();


  #ifdef SAVE_CHANNEL

    // Inicialitza l'EEPROM
    EEPROM.begin(10);

    // Llegeix el canal desat anteriorment
    lastChannel = EEPROM.read(0);

    Serial.println(lastChannel);


    // Utilitza el canal desat si és vàlid
    if (lastChannel >= 1 && lastChannel <= MAX_CHANNEL) {

      channel = lastChannel;
    }

    Serial.println(channel);

  #endif


  // Inicia el procés d'aparellament automàtic
  pairingStatus = PAIR_REQUEST;
}


// ============================================================
// BUCLE PRINCIPAL
// ============================================================

void loop() {

  // Continua només després d'un aparellament ESP-NOW correcte
  if (autoPairing() == PAIR_PAIRED) {

    unsigned long currentMillis = millis();


    // Comprova si és el moment d'enviar una nova lectura
    if (currentMillis - previousMillis >= interval) {
      if (bmp1.readPressure() < 80000) {
        ESP.restart();
      }
      // Desa el temps d'aquesta lectura
      previousMillis = currentMillis;


      // ------------------------------------------------------
      // LECTURA DEL SENSOR AHT20 #1
      // ------------------------------------------------------ 

      sensors_event_t humidity1, temperature1;

      // Llegeix la temperatura i la humitat
      aht1.getEvent(
        &humidity1,
        &temperature1
      );


      // ------------------------------------------------------
      // LECTURA DEL SENSOR AHT20 #2
      // ------------------------------------------------------

      sensors_event_t humidity2, temperature2;

      // Llegeix la temperatura i la humitat
      aht2.getEvent(
        &humidity2,
        &temperature2
      );


      // ------------------------------------------------------
      // PREPARACIÓ DEL PAQUET DE DADES
      // ------------------------------------------------------

      // Tipus de missatge
      myData.msgType = DATA;

      // ID de la placa
      myData.id = BOARD_ID;

      // Temperatura interna de l'ESP32
      myData.tempInternal = temperatureRead();


      // Parell de sensors #1
      myData.tempAHT20_1 = temperature1.temperature;
      myData.humidityAHT20_1 = humidity1.relative_humidity;

      myData.pressureBMP280_1 = bmp1.readPressure();
      myData.tempBMP280_1 = bmp1.readTemperature();
      myData.altitudeBMP280_1 = bmp1.readAltitude();


      // Parell de sensors #2
      myData.tempAHT20_2 = temperature2.temperature;
      myData.humidityAHT20_2 = humidity2.relative_humidity;

      myData.pressureBMP280_2 = bmp2.readPressure();
      myData.tempBMP280_2 = bmp2.readTemperature();
      myData.altitudeBMP280_2 = bmp2.readAltitude();


      // Incrementa l'ID de lectura
      myData.readingId = readingId++;


      // Envia el paquet complet de dades dels sensors al servidor
      esp_err_t result = esp_now_send(
        serverAddress,
        (uint8_t *) &myData,
        sizeof(myData)
      );
    }
  }
}
