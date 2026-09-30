// ============================================================
// DOIT ESP32 C6
// Board ID: 1
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
// LCD DISPLAY:
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
// SENSOR PIN CONFIGURATION
// ============================================================

// MQ2 analog input
const int MQ2_PIN = 0;

// MQ135 analog input
const int MQ135_PIN = 1;

// Hall sensor analog input
const int HALL_SENSOR_PIN = 2;


// ============================================================
// SENSOR OBJECTS
// ============================================================

// AHT21 sensor object
Adafruit_AHTX0 aht;


// ENS160 sensor object
// Address 0x52:
// ScioSense_ENS160 ens160(ENS160_I2CADDR_0);

// Address 0x53
ScioSense_ENS160 ens160(ENS160_I2CADDR_1);


// I2C LCD display
LiquidCrystal_I2C lcd(0x27, 16, 2);


// Restart status variable
int restartStatus = 0;


// ============================================================
// ESP-NOW CONFIGURATION
// ============================================================

// Board ID
#define BOARD_ID 1

// Maximum Wi-Fi channel used during pairing
#define MAX_CHANNEL 13


// Server MAC address
uint8_t serverAddress[] = {
  0xFF,0xFF,0xFF,0xFF,0xFF,0xFF
};


// MAC address of this board
uint8_t clientMacAddress[6];


// ============================================================
// DATA STRUCTURE
// ============================================================

// Structure used for communication between ESP32 boards.
// It must match the structure used by the sender/receiver.
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

  // Analog sensors
  int MQ2;
  int MQ135;
  int hall;

  // Motion state
  bool motion;

  // Relay states
  bool relay1;
  bool relay2;

  // Servo angle
  int servo;

  // Reading counter
  unsigned int readingId;

} struct_message;


// Structure used during ESP-NOW pairing
typedef struct struct_pairing {

    uint8_t msgType;
    uint8_t id;
    uint8_t macAddr[6];
    uint8_t channel;

} struct_pairing;


// ESP-NOW peer information
esp_now_peer_info_t peer;


// Data sent to the server
struct_message myData;

// Data received from the server
struct_message inData;

// Pairing information
struct_pairing pairingData;


// ============================================================
// PAIRING STATUS
// ============================================================

enum PairingStatus {
  NOT_PAIRED,
  PAIR_REQUEST,
  PAIR_REQUESTED,
  PAIR_PAIRED,
};

PairingStatus pairingStatus = NOT_PAIRED;


// Message types
enum MessageType {
  PAIRING,
  DATA,
};

MessageType messageType;


#ifdef SAVE_CHANNEL

  // Last saved Wi-Fi channel
  int lastChannel;

#endif


// Current Wi-Fi channel used during pairing
int channel = 1;


// ============================================================
// TIMING AND READING VARIABLES
// ============================================================

// Simulated temperature/humidity variable
float t = 0;

// Current time
unsigned long currentMillis = millis();

// Previous execution time
unsigned long previousMillis = 0;

// Sensor reading interval
const long interval = 10000;

// Pairing start time
unsigned long start;

// Sensor reading counter
unsigned int readingId = 0;


// ============================================================
// READ MAC ADDRESS
// ============================================================

void readGetMacAddress(){

  uint8_t baseMac[6];

  // Read station MAC address
  esp_err_t ret =
    esp_wifi_get_mac(
      WIFI_IF_STA,
      baseMac
    );


  if (ret == ESP_OK) {

    // Print MAC address
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


  // Copy MAC address to the global array
  clientMacAddress[0] = baseMac[0];
  clientMacAddress[1] = baseMac[1];
  clientMacAddress[2] = baseMac[2];
  clientMacAddress[3] = baseMac[3];
  clientMacAddress[4] = baseMac[4];
  clientMacAddress[5] = baseMac[5];
}


// ============================================================
// ADD ESP-NOW PEER
// ============================================================

void addPeer(
  const uint8_t * mac_addr,
  uint8_t chan
){

  // Set Wi-Fi channel
  ESP_ERROR_CHECK(
    esp_wifi_set_channel(
      chan,
      WIFI_SECOND_CHAN_NONE
    )
  );


  // Remove existing peer
  esp_now_del_peer(mac_addr);


  // Clear peer structure
  memset(
    &peer,
    0,
    sizeof(esp_now_peer_info_t)
  );


  // Configure peer
  peer.channel = chan;

  peer.encrypt = false;


  // Copy peer MAC address
  memcpy(
    peer.peer_addr,
    mac_addr,
    sizeof(uint8_t[6])
  );


  // Add peer
  if (
    esp_now_add_peer(&peer) != ESP_OK
  ){

    Serial.println(
      "Failed to add peer"
    );

    return;
  }


  // Save server MAC address
  memcpy(
    serverAddress,
    mac_addr,
    sizeof(uint8_t[6])
  );
}


// ============================================================
// PRINT MAC ADDRESS
// ============================================================

void printMAC(
  const uint8_t * mac_addr
){

  char macStr[18];


  // Convert MAC address to string
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
// ESP-NOW SEND CALLBACK
// ============================================================

void OnDataSent(
  const uint8_t *mac_addr,
  esp_now_send_status_t status
){

  Serial.print(
    "\r\nLast Packet Send Status:\t"
  );


  // Display transmission result
  Serial.println(
    status == ESP_NOW_SEND_SUCCESS
    ? "Delivery Success"
    : "Delivery Fail"
  );
}


// ============================================================
// ESP-NOW RECEIVE CALLBACK
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


  // Display size of received data
  Serial.println(
    sizeof(incomingData)
  );


  // First byte identifies message type
  uint8_t type =
    incomingData[0];


  switch (type) {


    // --------------------------------------------------------
    // DATA MESSAGE
    // --------------------------------------------------------

    case DATA:

      // Copy incoming data into the structure
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
      // Optional LED indication
      if (inData.readingId % 2 == 1){
        digitalWrite(LED_BUILTIN, LOW);
      } else {
        digitalWrite(LED_BUILTIN, HIGH);
      }
      */

      break;


    // --------------------------------------------------------
    // PAIRING MESSAGE
    // --------------------------------------------------------

    case PAIRING:

      // Copy pairing information
      memcpy(
        &pairingData,
        incomingData,
        sizeof(pairingData)
      );


      // ID 0 means the message comes from the server
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


        // Display server channel
        Serial.print(
          pairingData.channel
        );


        Serial.print(
          " in "
        );


        // Display pairing time
        Serial.print(
          millis()-start
        );


        Serial.println("ms");


        // Add server to peer list
        addPeer(
          pairingData.macAddr,
          pairingData.channel
        );


        #ifdef SAVE_CHANNEL

          // Save selected channel
          lastChannel =
            pairingData.channel;

          EEPROM.write(
            0,
            pairingData.channel
          );

          EEPROM.commit();

        #endif


        // Mark pairing as completed
        pairingStatus =
          PAIR_PAIRED;
      }

      break;
  }
}


// ============================================================
// AUTOMATIC PAIRING
// ============================================================

PairingStatus autoPairing(){

  switch(pairingStatus) {


    // --------------------------------------------------------
    // SEND PAIRING REQUEST
    // --------------------------------------------------------

    case PAIR_REQUEST:

      Serial.print(
        "Pairing request on channel  "
      );

      Serial.println(channel);


      // Set Wi-Fi channel
      ESP_ERROR_CHECK(
        esp_wifi_set_channel(
          channel,
          WIFI_SECOND_CHAN_NONE
        )
      );


      // Initialize ESP-NOW
      if (
        esp_now_init() != ESP_OK
      ){

        Serial.println(
          "Error initializing ESP-NOW"
        );
      }


      // Register callbacks
      esp_now_register_send_cb(
        esp_now_send_cb_t(OnDataSent)
      );

      esp_now_register_recv_cb(
        esp_now_recv_cb_t(OnDataRecv)
      );


      // Prepare pairing data
      pairingData.msgType =
        PAIRING;

      pairingData.id =
        BOARD_ID;

      pairingData.channel =
        channel;


      // Copy this board's MAC address
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


      // Add server as peer
      addPeer(
        serverAddress,
        channel
      );


      // Send pairing request
      esp_now_send(
        serverAddress,
        (uint8_t *) &pairingData,
        sizeof(pairingData)
      );


      // Start timeout timer
      previousMillis =
        millis();


      pairingStatus =
        PAIR_REQUESTED;

      break;


    // --------------------------------------------------------
    // WAIT FOR PAIRING RESPONSE
    // --------------------------------------------------------

    case PAIR_REQUESTED:

      currentMillis =
        millis();


      // Wait one second for a response
      if (
        currentMillis -
        previousMillis >
        1000
      ){

        previousMillis =
          currentMillis;


        // Try the next Wi-Fi channel
        channel ++;


        // Return to channel 1 after the maximum
        if (
          channel > MAX_CHANNEL
        ){

          channel = 1;
        }


        // Send another pairing request
        pairingStatus =
          PAIR_REQUEST;
      }

      break;


    // --------------------------------------------------------
    // PAIRED
    // --------------------------------------------------------

    case PAIR_PAIRED:

      // Nothing to do here

      break;
  }


  return pairingStatus;
}


// ============================================================
// SETUP
// ============================================================

void setup() {

  // Start Serial Monitor
  Serial.begin(115200);

  Serial.println();


  // Configure built-in LED
  pinMode(
    LED_BUILTIN,
    OUTPUT
  );


  // ----------------------------------------------------------
  // I2C
  // ----------------------------------------------------------

  // Initialize I2C bus
  // SDA = GPIO6
  // SCL = GPIO7
  Wire.begin(
    6,
    7
  );


  // ----------------------------------------------------------
  // AHT21
  // ----------------------------------------------------------

  // Initialize AHT21
  aht.begin();


  // ----------------------------------------------------------
  // ENS160
  // ----------------------------------------------------------

  // Initialize ENS160
  ens160.begin();


  // Print ENS160 revision
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


  // Set ENS160 to standard operating mode
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
  // HALL SENSOR
  // ----------------------------------------------------------

  pinMode(
    HALL_SENSOR_PIN,
    INPUT_PULLUP
  );


  // ----------------------------------------------------------
  // LCD
  // ----------------------------------------------------------

  // Initialize LCD
  lcd.init();

  // Turn on LCD backlight
  lcd.backlight();


  // ----------------------------------------------------------
  // WI-FI / ESP-NOW
  // ----------------------------------------------------------

  WiFi.mode(
    WIFI_STA
  );

  WiFi.STA.begin();


  // Print board MAC address
  Serial.print(
    "Client Board MAC Address:  "
  );

  readGetMacAddress();


  // Disconnect from Wi-Fi before pairing
  WiFi.disconnect();


  // Store pairing start time
  start =
    millis();


  #ifdef SAVE_CHANNEL

    // Initialize EEPROM
    EEPROM.begin(10);


    // Read stored channel
    lastChannel =
      EEPROM.read(0);


    Serial.println(
      lastChannel
    );


    // Use stored channel if valid
    if (
      lastChannel >= 1 &&
      lastChannel <= MAX_CHANNEL
    ){

      channel =
        lastChannel;
    }


    Serial.println(channel);

  #endif


  // Start automatic pairing
  pairingStatus =
    PAIR_REQUEST;
}


// ============================================================
// MAIN LOOP
// ============================================================

void loop() {

  // Continue normal operation after pairing
  if (
    autoPairing() ==
    PAIR_PAIRED
  ){

    unsigned long currentMillis =
      millis();


    // Read and transmit sensors every 10 seconds
    if (
      currentMillis -
      previousMillis >=
      interval
    ){

      // Save the current time
      previousMillis =
        currentMillis;


      // ------------------------------------------------------
      // READ AHT21
      // ------------------------------------------------------

      sensors_event_t
        humidity_aht,
        temperature_aht;


      // Read temperature and humidity
      aht.getEvent(
        &humidity_aht,
        &temperature_aht
      );


      // ------------------------------------------------------
      // READ ENS160
      // ------------------------------------------------------

      // Provide temperature and humidity to ENS160
      ens160.set_envdata(
        temperature_aht.temperature,
        humidity_aht.relative_humidity
      );


      // Perform ENS160 measurement
      ens160.measure(true);

      ens160.measureRaw(true);


      // ------------------------------------------------------
      // READ ANALOG SENSORS
      // ------------------------------------------------------

      float mq2Readings =
        analogRead(MQ2_PIN);

      float mq135Readings =
        analogRead(MQ135_PIN);

      int hallSensorReadings =
        analogRead(HALL_SENSOR_PIN);


      // ------------------------------------------------------
      // PREPARE DATA PACKET
      // ------------------------------------------------------

      // Message type
      myData.msgType =
        DATA;


      // Board ID
      myData.id =
        BOARD_ID;


      // ESP32 internal temperature
      myData.tempInternal =
        temperatureRead();


      // AHT21 readings
      myData.tempAHT21 =
        temperature_aht.temperature;

      myData.humidityAHT21 =
        humidity_aht.relative_humidity;


      // ENS160 readings
      myData.AQIENS160 =
        ens160.getAQI();

      myData.TVOCENS160 =
        ens160.getTVOC();

      myData.eCO2ENS160 =
        ens160.geteCO2();


      // Analog sensor readings
      myData.MQ2 =
        mq2Readings;

      myData.MQ135 =
        mq135Readings;

      myData.hall =
        hallSensorReadings;


      // Increment reading ID
      myData.readingId =
        readingId++;


      // Send sensor data to server
      esp_err_t result =
        esp_now_send(
          serverAddress,
          (uint8_t *) &myData,
          sizeof(myData)
        );


      // Print analog readings
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
      // LCD DISPLAY
      // ------------------------------------------------------

      // Display received AHT20 data when available
      if (
        inData.humidityAHT20_1 != 0.00
      ){

        // Alternate between different information
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

        // Display local AHT21 and ENS160 data
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
    // DISPLAY SENSOR VALUES WHILE NOT PAIRED
    // --------------------------------------------------------

    sensors_event_t
      humidity_aht,
      temperature_aht;


    // Read AHT21
    aht.getEvent(
      &humidity_aht,
      &temperature_aht
    );


    // Provide environmental data to ENS160
    ens160.set_envdata(
      temperature_aht.temperature,
      humidity_aht.relative_humidity
    );


    // Perform ENS160 measurement
    ens160.measure(true);

    ens160.measureRaw(true);


    // --------------------------------------------------------
    // DISPLAY LOCAL SENSOR DATA
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