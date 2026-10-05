// ============================================================
// Codi de https://github.com/UsernameDZ/Research-project-examples/
// Llicència: MIT License
//
// Comentaris del codi generats amb la IA (ChatGPT)
//
// DOIT ESP32 C6
// ID de la placa: 1
//
// AHT21 + ENS160:
// SCL - GPIO7
// SDA - GPIO6
// VDD - 3.3V
// GND - GND
//
// MQ2:
// AO -  0
// GND - GND
// VCC - VCC/5V
//
// MQ135:
// AO -  1
// GND - GND
// VCC - VCC/5V
//
// HALL SENSOR:
// VCC - VCC
// SIGNAL - 2
// GND - GND
//
// PANTALLA LCD:
// VCC - VCC/5V
// SDA - 6
// SCL - 7
// GND - GND
// ============================================================


#include <Arduino.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <EEPROM.h>

#include <Wire.h>
#include <Adafruit_AHTX0.h>
#include "ScioSense_ENS160.h"
#include <LiquidCrystal_I2C.h>


// ============================================================
// CONFIGURACIÓ DELS PINS DELS SENSORS
// ============================================================

// Entrada analògica MQ2
const int MQ2_PIN = 0;

// Entrada analògica MQ135
const int MQ135_PIN = 1;

// Entrada analògica del sensor Hall
const int HALL_SENSOR_PIN = 2;


// ============================================================
// OBJECTES DELS SENSORS
// ============================================================

// Objecte del sensor AHT21
Adafruit_AHTX0 aht;


// Objecte del sensor ENS160
// Adreça 0x52:
// ScioSense_ENS160 ens160(ENS160_I2CADDR_0);

// Adreça 0x53
ScioSense_ENS160 ens160(ENS160_I2CADDR_1);


// Pantalla LCD I2C
LiquidCrystal_I2C lcd(0x27, 16, 2);


// Variable d'estat del reinici
int restartStatus = 0;


// ============================================================
// CONFIGURACIÓ ESP-NOW
// ============================================================

// ID de la placa
#define BOARD_ID 1

// Canal Wi-Fi màxim utilitzat durant l'aparellament
#define MAX_CHANNEL 13


// Adreça MAC del servidor
uint8_t serverAddress[] = {
  0xFF,0xFF,0xFF,0xFF,0xFF,0xFF
};


// Adreça MAC d'aquesta placa
uint8_t clientMacAddress[6];


// ============================================================
// ESTRUCTURA DE DADES
// ============================================================

// Estructura utilitzada per a la comunicació entre les plaques ESP32.
// Ha de coincidir amb l'estructura utilitzada pel transmissor/receptor.
typedef struct struct_message {

  uint8_t msgType;
  uint8_t id;
  float tempInternal;

  // AHT20 + BMP280 #1
  float tempAHT20_1;
  float humidityAHT20_1;
  float pressureBMP280_1;
  float tempBMP280_1;
  float altitudeBMP280_1;

  // AHT20 + BMP280 #2
  float tempAHT20_2;
  float humidityAHT20_2;
  float pressureBMP280_2;
  float tempBMP280_2;
  float altitudeBMP280_2;

  // AHT21 + ENS160
  float tempAHT21;
  float humidityAHT21;
  float AQIENS160;
  float TVOCENS160;
  float eCO2ENS160;

  // Sensors analògics
  int MQ2;
  int MQ135;
  int hall;

  // Estat del moviment
  bool motion;

  // Estats dels relés
  bool relay1;
  bool relay2;

  // Angle del servo
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


// Dades enviades al servidor
struct_message myData;

// Dades rebudes del servidor
struct_message inData;

// Informació de l'aparellament
struct_pairing pairingData;


// ============================================================
// ESTAT DE L'APARELLAMENT
// ============================================================

enum PairingStatus {
  NOT_PAIRED,
  PAIR_REQUEST,
  PAIR_REQUESTED,
  PAIR_PAIRED,
};

PairingStatus pairingStatus = NOT_PAIRED;


// Tipus de missatges
enum MessageType {
  PAIRING,
  DATA,
};

MessageType messageType;


#ifdef SAVE_CHANNEL

  // Últim canal Wi-Fi desat
  int lastChannel;

#endif


// Canal Wi-Fi actual utilitzat durant l'aparellament
int channel = 1;


// ============================================================
// VARIABLES DE TEMPS I LECTURES
// ============================================================

// Variable simulada de temperatura/humitat
float t = 0;

// Temps actual
unsigned long currentMillis = millis();

// Temps de l'execució anterior
unsigned long previousMillis = 0;

// Interval de lectura dels sensors
const long interval = 10000;

// Temps d'inici de l'aparellament
unsigned long start;

// Comptador de lectures dels sensors
unsigned int readingId = 0;


// ============================================================
// LECTURA DE L'ADREÇA MAC
// ============================================================

void readGetMacAddress(){

  uint8_t baseMac[6];

  // Llegeix l'adreça MAC de l'estació
  esp_err_t ret =
    esp_wifi_get_mac(
      WIFI_IF_STA,
      baseMac
    );


  if (ret == ESP_OK) {

    // Mostra l'adreça MAC
    Serial.printf(
      "%02x:%02x:%02x:%02x:%02x:%02x\n",
      baseMac[0],
      baseMac[1],
      baseMac[2],
      baseMac[3],
      baseMac[4],
      baseMac[5]
    );

  } else {

    Serial.println(
      "Failed to read MAC address"
    );
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

void addPeer(
  const uint8_t * mac_addr,
  uint8_t chan
){

  // Estableix el canal Wi-Fi
  ESP_ERROR_CHECK(
    esp_wifi_set_channel(
      chan,
      WIFI_SECOND_CHAN_NONE
    )
  );


  // Elimina el dispositiu existent
  esp_now_del_peer(mac_addr);


  // Neteja l'estructura del dispositiu
  memset(
    &peer,
    0,
    sizeof(esp_now_peer_info_t)
  );


  // Configura el dispositiu
  peer.channel = chan;

  peer.encrypt = false;


  // Copia l'adreça MAC del dispositiu
  memcpy(
    peer.peer_addr,
    mac_addr,
    sizeof(uint8_t[6])
  );


  // Afegeix el dispositiu
  if (
    esp_now_add_peer(&peer) != ESP_OK
  ){

    Serial.println(
      "Failed to add peer"
    );

    return;
  }


  // Desa l'adreça MAC del servidor
  memcpy(
    serverAddress,
    mac_addr,
    sizeof(uint8_t[6])
  );
}


// ============================================================
// MOSTRA L'ADREÇA MAC
// ============================================================

void printMAC(
  const uint8_t * mac_addr
){

  char macStr[18];


  // Converteix l'adreça MAC en una cadena de text
  snprintf(
    macStr,
    sizeof(macStr),
    "%02x:%02x:%02x:%02x:%02x:%02x",
    mac_addr[0],
    mac_addr[1],
    mac_addr[2],
    mac_addr[3],
    mac_addr[4],
    mac_addr[5]
  );


  Serial.print(macStr);
}


// ============================================================
// CALLBACK D'ENVIAMENT ESP-NOW
// ============================================================

void OnDataSent(
  const uint8_t *mac_addr,
  esp_now_send_status_t status
){

  Serial.print(
    "\r\nLast Packet Send Status:\t"
  );


  // Mostra el resultat de la transmissió
  Serial.println(
    status == ESP_NOW_SEND_SUCCESS
    ? "Delivery Success"
    : "Delivery Fail"
  );
}


// ============================================================
// CALLBACK DE RECEPCIÓ ESP-NOW
// ============================================================

void OnDataRecv(
  const uint8_t * mac_addr,
  const uint8_t *incomingData,
  int len
){

  Serial.print(
    "Packet received with "
  );

  Serial.print(
    "data size = "
  );


  // Mostra la mida de les dades rebudes
  Serial.println(
    sizeof(incomingData)
  );


  // El primer byte identifica el tipus de missatge
  uint8_t type =
    incomingData[0];


  switch (type) {


    // --------------------------------------------------------
    // MISSATGE DE DADES
    // --------------------------------------------------------

    case DATA:

      // Copia les dades rebudes a l'estructura
      memcpy(
        &inData,
        incomingData,
        sizeof(inData)
      );


      Serial.print("ID  = ");
      Serial.println(inData.id);


      Serial.print("Setpoint temp = ");
      Serial.println(inData.tempInternal);


      Serial.print("Temp AHT20_1  = ");
      Serial.println(inData.tempAHT20_1);


      Serial.print("Humidity AHT20_1 = ");
      Serial.println(
        inData.humidityAHT20_1
      );


      Serial.print("Pressure BMP280_1  = ");
      Serial.println(
        inData.pressureBMP280_1
      );


      Serial.print("Temp AHT20_2  = ");
      Serial.println(
        inData.tempAHT20_2
      );


      Serial.print("Humidity AHT20_2 = ");
      Serial.println(
        inData.humidityAHT20_2
      );


      Serial.print("Pressure BMP280_2  = ");
      Serial.println(
        inData.pressureBMP280_2
      );


      Serial.print("reading Id  = ");
      Serial.println(
        inData.readingId
      );


      /*
      // Indicació opcional mitjançant LED
      if (inData.readingId % 2 == 1){
        digitalWrite(LED_BUILTIN, LOW);
      } else {
        digitalWrite(LED_BUILTIN, HIGH);
      }
      */

      break;


    // --------------------------------------------------------
    // MISSATGE D'APARELLAMENT
    // --------------------------------------------------------

    case PAIRING:

      // Copia la informació de l'aparellament
      memcpy(
        &pairingData,
        incomingData,
        sizeof(pairingData)
      );


      // L'ID 0 indica que el missatge prové del servidor
      if (pairingData.id == 0) {

        Serial.print(
          "Pairing done for MAC Address: "
        );

        printMAC(
          pairingData.macAddr
        );


        Serial.print(
          " on channel "
        );


        // Mostra el canal del servidor
        Serial.print(
          pairingData.channel
        );


        Serial.print(
          " in "
        );


        // Mostra el temps de l'aparellament
        Serial.print(
          millis()-start
        );


        Serial.println("ms");


        // Afegeix el servidor a la llista de dispositius
        addPeer(
          pairingData.macAddr,
          pairingData.channel
        );


        #ifdef SAVE_CHANNEL

          // Desa el canal seleccionat
          lastChannel =
            pairingData.channel;

          EEPROM.write(
            0,
            pairingData.channel
          );

          EEPROM.commit();

        #endif


        // Marca l'aparellament com a completat
        pairingStatus =
          PAIR_PAIRED;
      }

      break;
  }
}


// ============================================================
// APARELLAMENT AUTOMÀTIC
// ============================================================

PairingStatus autoPairing(){

  switch(pairingStatus) {


    // --------------------------------------------------------
    // ENVIA LA SOL·LICITUD D'APARELLAMENT
    // --------------------------------------------------------

    case PAIR_REQUEST:

      Serial.print(
        "Pairing request on channel  "
      );

      Serial.println(channel);


      // Estableix el canal Wi-Fi
      ESP_ERROR_CHECK(
        esp_wifi_set_channel(
          channel,
          WIFI_SECOND_CHAN_NONE
        )
      );


      // Inicialitza ESP-NOW
      if (
        esp_now_init() != ESP_OK
      ){

        Serial.println(
          "Error initializing ESP-NOW"
        );
      }


      // Registra els callbacks
      esp_now_register_send_cb(
        esp_now_send_cb_t(OnDataSent)
      );

      esp_now_register_recv_cb(
        esp_now_recv_cb_t(OnDataRecv)
      );


      // Prepara les dades d'aparellament
      pairingData.msgType =
        PAIRING;

      pairingData.id =
        BOARD_ID;

      pairingData.channel =
        channel;


      // Copia l'adreça MAC d'aquesta placa
      pairingData.macAddr[0] =
        clientMacAddress[0];

      pairingData.macAddr[1] =
        clientMacAddress[1];

      pairingData.macAddr[2] =
        clientMacAddress[2];

      pairingData.macAddr[3] =
        clientMacAddress[3];

      pairingData.macAddr[4] =
        clientMacAddress[4];

      pairingData.macAddr[5] =
        clientMacAddress[5];


      // Afegeix el servidor com a dispositiu
      addPeer(
        serverAddress,
        channel
      );


      // Envia la sol·licitud d'aparellament
      esp_now_send(
        serverAddress,
        (uint8_t *) &pairingData,
        sizeof(pairingData)
      );


      // Inicia el temporitzador d'espera
      previousMillis =
        millis();


      pairingStatus =
        PAIR_REQUESTED;

      break;


    // --------------------------------------------------------
    // ESPERA LA RESPOSTA DE L'APARELLAMENT
    // --------------------------------------------------------

    case PAIR_REQUESTED:

      currentMillis =
        millis();


      // Espera un segon per obtenir una resposta
      if (
        currentMillis -
        previousMillis >
        1000
      ){

        previousMillis =
          currentMillis;


        // Prova el següent canal Wi-Fi
        channel ++;


        // Torna al canal 1 després del màxim
        if (
          channel > MAX_CHANNEL
        ){

          channel = 1;
        }


        // Envia una altra sol·licitud d'aparellament
        pairingStatus =
          PAIR_REQUEST;
      }

      break;


    // --------------------------------------------------------
    // APARELLAT
    // --------------------------------------------------------

    case PAIR_PAIRED:

      // No cal fer res aquí

      break;
  }


  return pairingStatus;
}


// ============================================================
// CONFIGURACIÓ
// ============================================================

void setup() {

  // Inicia el monitor sèrie
  Serial.begin(115200);

  Serial.println();


  // Configura el LED integrat
  pinMode(
    LED_BUILTIN,
    OUTPUT
  );


  // ----------------------------------------------------------
  // I2C
  // ----------------------------------------------------------

  // Inicialitza el bus I2C
  // SDA = GPIO6
  // SCL = GPIO7
  Wire.begin(
    6,
    7
  );


  // ----------------------------------------------------------
  // AHT21
  // ----------------------------------------------------------

  // Inicialitza l'AHT21
  aht.begin();


  // ----------------------------------------------------------
  // ENS160
  // ----------------------------------------------------------

  // Inicialitza l'ENS160
  ens160.begin();


  // Mostra la revisió de l'ENS160
  Serial.print("\tRev: ");

  Serial.print(
    ens160.getMajorRev()
  );

  Serial.print(".");

  Serial.print(
    ens160.getMinorRev()
  );

  Serial.print(".");

  Serial.println(
    ens160.getBuild()
  );


  // Configura l'ENS160 en mode de funcionament estàndard
  Serial.print(
    "\tStandard mode "
  );

  Serial.println(
    ens160.setMode(
      ENS160_OPMODE_STD
    )
    ? "done."
    : "failed!"
  );


  // ----------------------------------------------------------
  // SENSOR HALL
  // ----------------------------------------------------------

  pinMode(
    HALL_SENSOR_PIN,
    INPUT_PULLUP
  );


  // ----------------------------------------------------------
  // LCD
  // ----------------------------------------------------------

  // Inicialitza la pantalla LCD
  lcd.init();

  // Activa la il·luminació de fons de la pantalla LCD
  lcd.backlight();


  // ----------------------------------------------------------
  // WI-FI / ESP-NOW
  // ----------------------------------------------------------

  WiFi.mode(
    WIFI_STA
  );

  WiFi.STA.begin();


  // Mostra l'adreça MAC de la placa
  Serial.print(
    "Client Board MAC Address:  "
  );

  readGetMacAddress();


  // Desconnecta el Wi-Fi abans de l'aparellament
  WiFi.disconnect();


  // Desa el temps d'inici de l'aparellament
  start =
    millis();


  #ifdef SAVE_CHANNEL

    // Inicialitza l'EEPROM
    EEPROM.begin(10);


    // Llegeix el canal desat
    lastChannel =
      EEPROM.read(0);


    Serial.println(
      lastChannel
    );


    // Utilitza el canal desat si és vàlid
    if (
      lastChannel >= 1 &&
      lastChannel <= MAX_CHANNEL
    ){

      channel =
        lastChannel;
    }


    Serial.println(channel);

  #endif


  // Inicia l'aparellament automàtic
  pairingStatus =
    PAIR_REQUEST;
}


// ============================================================
// BUCLE PRINCIPAL
// ============================================================

void loop() {

  // Continua amb el funcionament normal després de l'aparellament
  if (
    autoPairing() ==
    PAIR_PAIRED
  ){

    unsigned long currentMillis =
      millis();


    // Llegeix i transmet els sensors cada 10 segons
    if (
      currentMillis -
      previousMillis >=
      interval
    ){

      // Desa el temps actual
      previousMillis =
        currentMillis;


      // ------------------------------------------------------
      // LECTURA DE L'AHT21
      // ------------------------------------------------------

      sensors_event_t
        humidity_aht,
        temperature_aht;


      // Llegeix la temperatura i la humitat
      aht.getEvent(
        &humidity_aht,
        &temperature_aht
      );


      // ------------------------------------------------------
      // LECTURA DE L'ENS160
      // ------------------------------------------------------

      // Proporciona la temperatura i la humitat a l'ENS160
      ens160.set_envdata(
        temperature_aht.temperature,
        humidity_aht.relative_humidity
      );


      // Realitza la mesura de l'ENS160
      ens160.measure(true);

      ens160.measureRaw(true);


      // ------------------------------------------------------
      // LECTURA DELS SENSORS ANALÒGICS
      // ------------------------------------------------------

      float mq2Readings =
        analogRead(MQ2_PIN);

      float mq135Readings =
        analogRead(MQ135_PIN);

      int hallSensorReadings =
        analogRead(HALL_SENSOR_PIN);


      // ------------------------------------------------------
      // PREPARACIÓ DEL PAQUET DE DADES
      // ------------------------------------------------------

      // Tipus de missatge
      myData.msgType =
        DATA;


      // ID de la placa
      myData.id =
        BOARD_ID;


      // Temperatura interna de l'ESP32
      myData.tempInternal =
        temperatureRead();


      // Lectures de l'AHT21
      myData.tempAHT21 =
        temperature_aht.temperature;

      myData.humidityAHT21 =
        humidity_aht.relative_humidity;


      // Lectures de l'ENS160
      myData.AQIENS160 =
        ens160.getAQI();

      myData.TVOCENS160 =
        ens160.getTVOC();

      myData.eCO2ENS160 =
        ens160.geteCO2();


      // Lectures dels sensors analògics
      myData.MQ2 =
        mq2Readings;

      myData.MQ135 =
        mq135Readings;

      myData.hall =
        hallSensorReadings;


      // Incrementa l'ID de lectura
      myData.readingId =
        readingId++;


      // Envia les dades dels sensors al servidor
      esp_err_t result =
        esp_now_send(
          serverAddress,
          (uint8_t *) &myData,
          sizeof(myData)
        );


      // Mostra les lectures analògiques
      Serial.println(
        mq2Readings
      );

      Serial.println(
        mq135Readings
      );

      Serial.println(
        hallSensorReadings
      );


      // ------------------------------------------------------
      // PANTALLA LCD
      // ------------------------------------------------------

      // Mostra les dades AHT20 rebudes quan estan disponibles
      if (
        inData.humidityAHT20_1 != 0.00
      ){

        // Alterna entre diferents informacions
        if (
          inData.readingId % 2 == 0
        ){

          lcd.clear();

          lcd.setCursor(0, 0);

          lcd.print("Ti:");

          lcd.print(
            inData.tempAHT20_1,
            1
          );

          lcd.print("C Hi:");

          lcd.print(
            inData.humidityAHT20_1,
            0
          );

          lcd.print("%");


          lcd.setCursor(0, 1);

          lcd.print("AQI:");

          lcd.print(
            ens160.getAQI()
          );

          lcd.print(" CO2:");

          lcd.print(
            ens160.geteCO2()
          );


        } else {

          lcd.clear();

          lcd.setCursor(0, 0);

          lcd.print("To:");

          lcd.print(
            inData.tempAHT20_2,
            1
          );

          lcd.print("C Ho:");

          lcd.print(
            inData.humidityAHT20_2,
            0
          );

          lcd.print("%");


          lcd.setCursor(0, 1);

          lcd.print("Pi:");

          lcd.print(
            inData.pressureBMP280_1 /
            101325.0
          );

          lcd.print(" Po:");

          lcd.print(
            inData.pressureBMP280_2 /
            101325.0
          );
        }


      } else {

        // Mostra les dades locals de l'AHT21 i l'ENS160
        lcd.clear();

        lcd.setCursor(0, 0);

        lcd.print("T:");

        lcd.print(
          temperature_aht.temperature,
          1
        );

        lcd.print("C H:");

        lcd.print(
          humidity_aht.relative_humidity,
          0
        );

        lcd.print("%");


        lcd.setCursor(0, 1);

        lcd.print("AQI:");

        lcd.print(
          ens160.getAQI()
        );

        lcd.print(" CO2:");

        lcd.print(
          ens160.geteCO2()
        );
      }
    }


  } else {

    // --------------------------------------------------------
    // MOSTRA ELS VALORS DELS SENSORS MENTRE NO ESTÀ APARELLAT
    // --------------------------------------------------------

    sensors_event_t
      humidity_aht,
      temperature_aht;


    // Llegeix l'AHT21
    aht.getEvent(
      &humidity_aht,
      &temperature_aht
    );


    // Proporciona les dades ambientals a l'ENS160
    ens160.set_envdata(
      temperature_aht.temperature,
      humidity_aht.relative_humidity
    );


    // Realitza la mesura de l'ENS160
    ens160.measure(true);

    ens160.measureRaw(true);


    // --------------------------------------------------------
    // MOSTRA LES DADES DELS SENSORS LOCALS
    // --------------------------------------------------------

    lcd.clear();

    lcd.setCursor(0, 0);

    lcd.print("T:");

    lcd.print(
      temperature_aht.temperature,
      1
    );

    lcd.print("C H:");

    lcd.print(
      humidity_aht.relative_humidity,
      0
    );

    lcd.print("%");


    lcd.setCursor(0, 1);

    lcd.print("AQI:");

    lcd.print(
      ens160.getAQI()
    );

    lcd.print(" CO2:");

    lcd.print(
      ens160.geteCO2()
    );
  }
}
