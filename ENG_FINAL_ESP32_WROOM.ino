// ============================================================
// DOIT ESP32 DEVKIT V1 (WROOM)
// Board ID: 2
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
// SCL- GPIO26
// VDD - 3.3V
// GND - GND
//
// Relays:
// GND - GND
// IN1 - GPIO32
// IN2 - GPIO33
// VCC - 5V
//
// Servo motor:
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
// I2C PIN CONFIGURATION
// ============================================================

// I2C bus #1
#define SDA_1 18
#define SCL_1 19

// I2C bus #2
#define SDA_2 25
#define SCL_2 26

/*
TwoWire I2Cone = TwoWire(0);
TwoWire I2Ctwo = TwoWire(1);
*/

// Use the default I2C bus for the first sensor pair
#define I2Cone Wire

// Create a second I2C bus for the second sensor pair
TwoWire I2Ctwo(1);


// ============================================================
// SENSOR OBJECTS
// ============================================================

// BMP280 and AHT20 on I2C bus #1
Adafruit_BMP280 bmp1(&I2Cone);
Adafruit_AHTX0 aht1;

// BMP280 and AHT20 on I2C bus #2
Adafruit_BMP280 bmp2(&I2Ctwo);
Adafruit_AHTX0 aht2;

// Servo motor object
Servo myServo;


// ============================================================
// OUTPUT PIN CONFIGURATION
// ============================================================

#define RELAY1_PIN 32
#define RELAY2_PIN 33
#define SERVO_PIN 13


// ============================================================
// ESP-NOW BOARD CONFIGURATION
// ============================================================

// ID of this ESP32 board
#define BOARD_ID 2

// Maximum Wi-Fi channel used for ESP-NOW pairing
// 11 is commonly used in North America
// 13 is available in Europe
#define MAX_CHANNEL 13


// Server MAC address
// FF:FF:FF:FF:FF:FF is used initially for pairing
uint8_t serverAddress[] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

// MAC address of this ESP32 board
uint8_t clientMacAddress[6];


// ============================================================
// DATA STRUCTURE
// ============================================================

// Structure used for communication between the ESP32 boards.
// The structure must be identical on the sender and receiver.
typedef struct struct_message {
  uint8_t msgType;
  uint8_t id;
  float tempInternal;

  // AHT20 + BMP280 sensor pair #1
  float tempAHT20_1;
  float humidityAHT20_1;
  float pressureBMP280_1;
  float tempBMP280_1;
  float altitudeBMP280_1;

  // AHT20 + BMP280 sensor pair #2
  float tempAHT20_2;
  float humidityAHT20_2;
  float pressureBMP280_2;
  float tempBMP280_2;
  float altitudeBMP280_2;

  // Data fields used by the other ESP32 board
  float tempAHT21;
  float humidityAHT21;
  float AQIENS160;
  float TVOCENS160;
  float eCO2ENS160;

  // Analog and digital sensor values
  int MQ2;
  int MQ135;
  int hall;

  // Motion sensor state
  bool motion;

  // Relay states
  bool relay1;
  bool relay2;

  // Servo position
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


// Data structure for outgoing data
struct_message myData;

// Data structure for incoming data
struct_message inData;

// Data structure used for pairing
struct_pairing pairingData;


// ============================================================
// ESP-NOW PAIRING STATUS
// ============================================================

enum PairingStatus {
  NOT_PAIRED,
  PAIR_REQUEST,
  PAIR_REQUESTED,
  PAIR_PAIRED,
};

PairingStatus pairingStatus = NOT_PAIRED;


// Message types used by ESP-NOW
enum MessageType {
  PAIRING,
  DATA,
};

MessageType messageType;


#ifdef SAVE_CHANNEL
  // Stores the previously used Wi-Fi channel
  int lastChannel;
#endif


// Current channel used for ESP-NOW pairing
int channel = 1;


// ============================================================
// TIMING VARIABLES
// ============================================================

// Simulate temperature and humidity data
float t = 0;

// Stores the current time
unsigned long currentMillis = millis();

// Stores the previous execution time
unsigned long previousMillis = 0;

// Interval between sensor transmissions
const long interval = 10000;

// Used to measure the time required for pairing
unsigned long start;

// Counter for sensor readings
unsigned int readingId = 0;


// ============================================================
// READ ESP32 MAC ADDRESS
// ============================================================

void readGetMacAddress(){

  uint8_t baseMac[6];

  // Read the MAC address of the ESP32 station interface
  esp_err_t ret = esp_wifi_get_mac(WIFI_IF_STA, baseMac);

  if (ret == ESP_OK) {

    // Print the MAC address to Serial Monitor
    Serial.printf("%02x:%02x:%02x:%02x:%02x:%02x\n",
                  baseMac[0], baseMac[1], baseMac[2],
                  baseMac[3], baseMac[4], baseMac[5]);

  } else {

    Serial.println("Failed to read MAC address");
  }

  // Copy the MAC address into the global array
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

void addPeer(const uint8_t * mac_addr, uint8_t chan){

  // Set the Wi-Fi channel used by ESP-NOW
  ESP_ERROR_CHECK(esp_wifi_set_channel(chan ,WIFI_SECOND_CHAN_NONE));

  // Remove the peer if it already exists
  esp_now_del_peer(mac_addr);

  // Clear the peer structure
  memset(&peer, 0, sizeof(esp_now_peer_info_t));

  // Configure the peer
  peer.channel = chan;
  peer.encrypt = false;

  // Copy the peer MAC address
  memcpy(peer.peer_addr, mac_addr, sizeof(uint8_t[6]));

  // Add the peer to the ESP-NOW peer list
  if (esp_now_add_peer(&peer) != ESP_OK){

    Serial.println("Failed to add peer");
    return;
  }

  // Store the server MAC address
  memcpy(serverAddress, mac_addr, sizeof(uint8_t[6]));
}


// ============================================================
// PRINT MAC ADDRESS
// ============================================================

void printMAC(const uint8_t * mac_addr){

  char macStr[18];

  // Convert MAC address into printable text
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2],
           mac_addr[3], mac_addr[4], mac_addr[5]);

  Serial.print(macStr);
}


// ============================================================
// ESP-NOW SEND CALLBACK
// ============================================================

// Called after an ESP-NOW packet has been sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {

  Serial.print("\r\nLast Packet Send Status:\t");

  // Print whether the packet was delivered successfully
  Serial.println(
    status == ESP_NOW_SEND_SUCCESS
    ? "Delivery Success"
    : "Delivery Fail"
  );
}


// ============================================================
// ESP-NOW RECEIVE CALLBACK
// ============================================================

// Called whenever an ESP-NOW packet is received
void OnDataRecv(
  const uint8_t * mac_addr,
  const uint8_t *incomingData,
  int len
) {

  Serial.print("Packet received with ");
  Serial.print("data size = ");

  Serial.println(sizeof(incomingData));

  // The first byte contains the message type
  uint8_t type = incomingData[0];

  switch (type) {

    // --------------------------------------------------------
    // DATA MESSAGE
    // --------------------------------------------------------

    case DATA:

      // Copy received data into the incoming data structure
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


      // Relay outputs are active LOW
      digitalWrite(
        RELAY1_PIN,
        inData.relay1 ? LOW : HIGH
      );

      digitalWrite(
        RELAY2_PIN,
        inData.relay2 ? LOW : HIGH
      );


      // Set the servo angle
      myServo.write(inData.servo);


      // Change the built-in LED according to the reading ID
      if (inData.readingId % 2 == 1){

        digitalWrite(LED_BUILTIN, LOW);

      } else {

        digitalWrite(LED_BUILTIN, HIGH);
      }

      break;


    // --------------------------------------------------------
    // PAIRING MESSAGE
    // --------------------------------------------------------

    case PAIRING:

      // Copy the received pairing information
      memcpy(
        &pairingData,
        incomingData,
        sizeof(pairingData)
      );

      // ID 0 identifies the server
      if (pairingData.id == 0) {

        Serial.print("Pairing done for MAC Address: ");

        printMAC(pairingData.macAddr);

        Serial.print(" on channel ");

        // Display the channel selected by the server
        Serial.print(pairingData.channel);

        Serial.print(" in ");

        // Display the time required for pairing
        Serial.print(millis()-start);

        Serial.println("ms");


        // Add the server to the ESP-NOW peer list
        addPeer(
          pairingData.macAddr,
          pairingData.channel
        );


        #ifdef SAVE_CHANNEL

          // Save the channel to EEPROM
          lastChannel = pairingData.channel;

          EEPROM.write(
            0,
            pairingData.channel
          );

          EEPROM.commit();

        #endif


        // Mark the board as paired
        pairingStatus = PAIR_PAIRED;
      }

      break;
  }
}


// ============================================================
// AUTOMATIC ESP-NOW PAIRING
// ============================================================

PairingStatus autoPairing(){

  switch(pairingStatus) {


    // --------------------------------------------------------
    // SEND PAIRING REQUEST
    // --------------------------------------------------------

    case PAIR_REQUEST:

      Serial.print("Pairing request on channel  ");
      Serial.println(channel);


      // Set the current Wi-Fi channel
      ESP_ERROR_CHECK(
        esp_wifi_set_channel(
          channel,
          WIFI_SECOND_CHAN_NONE
        )
      );


      // Initialize ESP-NOW
      if (esp_now_init() != ESP_OK) {

        Serial.println("Error initializing ESP-NOW");
      }


      // Register ESP-NOW callbacks
      esp_now_register_send_cb(
        esp_now_send_cb_t(OnDataSent)
      );

      esp_now_register_recv_cb(
        esp_now_recv_cb_t(OnDataRecv)
      );


      // Prepare pairing information
      pairingData.msgType = PAIRING;

      pairingData.id = BOARD_ID;

      pairingData.channel = channel;


      // Copy this board's MAC address
      pairingData.macAddr[0] = clientMacAddress[0];
      pairingData.macAddr[1] = clientMacAddress[1];
      pairingData.macAddr[2] = clientMacAddress[2];
      pairingData.macAddr[3] = clientMacAddress[3];
      pairingData.macAddr[4] = clientMacAddress[4];
      pairingData.macAddr[5] = clientMacAddress[5];


      // Add the server as a peer and send the pairing request
      addPeer(serverAddress, channel);

      esp_now_send(
        serverAddress,
        (uint8_t *) &pairingData,
        sizeof(pairingData)
      );


      // Start the pairing timeout timer
      previousMillis = millis();

      pairingStatus = PAIR_REQUESTED;

      break;


    // --------------------------------------------------------
    // WAIT FOR PAIRING RESPONSE
    // --------------------------------------------------------

    case PAIR_REQUESTED:

      // Check how much time has passed
      currentMillis = millis();

      if(currentMillis - previousMillis > 1000) {

        previousMillis = currentMillis;

        // Move to the next Wi-Fi channel
        channel ++;

        // Return to channel 1 after reaching the maximum
        if (channel > MAX_CHANNEL){

          channel = 1;
        }

        // Send another pairing request
        pairingStatus = PAIR_REQUEST;
      }

      break;


    // --------------------------------------------------------
    // PAIRED
    // --------------------------------------------------------

    case PAIR_PAIRED:

      // Nothing to do while paired

      break;
  }

  return pairingStatus;
}


// ============================================================
// SETUP
// ============================================================

void setup() {

  // Give the board some time to start
  delay(1000);

  // Start Serial communication
  Serial.begin(115200);

  Serial.println();


  // Configure the built-in LED
  pinMode(LED_BUILTIN, OUTPUT);


  // Configure relay pins
  pinMode(RELAY1_PIN, OUTPUT);
  pinMode(RELAY2_PIN, OUTPUT);


  // Turn both relays off initially
  digitalWrite(RELAY1_PIN, HIGH);
  digitalWrite(RELAY2_PIN, HIGH);


  // Attach the servo to its GPIO pin
  myServo.attach(SERVO_PIN);

  // Set the initial servo position
  myServo.write(0);


  // Initialize the first I2C bus
  I2Cone.begin(
    SDA_1,
    SCL_1,
    50000
  );


  // Initialize the second I2C bus
  I2Ctwo.begin(
    SDA_2,
    SCL_2,
    50000
  );


  // Set I2C timeout values
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
  // Wi-Fi / ESP-NOW INITIALIZATION
  // ----------------------------------------------------------

  // Configure the ESP32 as a Wi-Fi station
  WiFi.mode(WIFI_STA);

  WiFi.STA.begin();


  // Print this board's MAC address
  Serial.print("Client Board MAC Address:  ");

  readGetMacAddress();


  // Disconnect from Wi-Fi before ESP-NOW pairing
  WiFi.disconnect();


  // Store the pairing start time
  start = millis();


  #ifdef SAVE_CHANNEL

    // Initialize EEPROM
    EEPROM.begin(10);

    // Read the previously stored channel
    lastChannel = EEPROM.read(0);

    Serial.println(lastChannel);


    // Use the stored channel if it is valid
    if (lastChannel >= 1 && lastChannel <= MAX_CHANNEL) {

      channel = lastChannel;
    }

    Serial.println(channel);

  #endif


  // Start the automatic pairing process
  pairingStatus = PAIR_REQUEST;
}


// ============================================================
// MAIN LOOP
// ============================================================

void loop() {

  // Continue only after successful ESP-NOW pairing
  if (autoPairing() == PAIR_PAIRED) {

    unsigned long currentMillis = millis();


    // Check whether it is time to send another reading
    if (currentMillis - previousMillis >= interval) {
      if (bmp1.readPressure() < )
      // Save the time of this reading
      previousMillis = currentMillis;


      // ------------------------------------------------------
      // READ AHT20 SENSOR #1
      // ------------------------------------------------------ 

      sensors_event_t humidity1, temperature1;

      // Read temperature and humidity
      aht1.getEvent(
        &humidity1,
        &temperature1
      );


      // ------------------------------------------------------
      // READ AHT20 SENSOR #2
      // ------------------------------------------------------

      sensors_event_t humidity2, temperature2;

      // Read temperature and humidity
      aht2.getEvent(
        &humidity2,
        &temperature2
      );


      // ------------------------------------------------------
      // PREPARE DATA PACKET
      // ------------------------------------------------------

      // Message type
      myData.msgType = DATA;

      // Board ID
      myData.id = BOARD_ID;

      // ESP32 internal temperature
      myData.tempInternal = temperatureRead();


      // Sensor pair #1
      myData.tempAHT20_1 = temperature1.temperature;
      myData.humidityAHT20_1 = humidity1.relative_humidity;

      myData.pressureBMP280_1 = bmp1.readPressure();
      myData.tempBMP280_1 = bmp1.readTemperature();
      myData.altitudeBMP280_1 = bmp1.readAltitude();


      // Sensor pair #2
      myData.tempAHT20_2 = temperature2.temperature;
      myData.humidityAHT20_2 = humidity2.relative_humidity;

      myData.pressureBMP280_2 = bmp2.readPressure();
      myData.tempBMP280_2 = bmp2.readTemperature();
      myData.altitudeBMP280_2 = bmp2.readAltitude();


      // Increment the reading ID
      myData.readingId = readingId++;


      // Send the complete sensor packet to the server
      esp_err_t result = esp_now_send(
        serverAddress,
        (uint8_t *) &myData,
        sizeof(myData)
      );
    }
  }
}
