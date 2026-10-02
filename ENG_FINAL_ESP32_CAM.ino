// ============================================================
// AI Thinker ESP32-CAM
//
// PIR sensor:
// VCC    - 3.3V
// SIGNAL - GPIO13
// GND    - GND
// ============================================================


#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include "ESPAsyncWebServer.h"
#include "AsyncTCP.h"
#include <ArduinoJson.h>

#include <WiFiClientSecure.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "esp_camera.h"
#include <UniversalTelegramBot.h>


// ============================================================
// WI-FI CONFIGURATION
// ============================================================

const char* ssid = "XXX";
const char* password = "XXX";


// Telegram bot token and chat ID
String BOTtoken = "XXX";
String CHAT_ID = "XXX";


// ============================================================
// GLOBAL STATE VARIABLES
// ============================================================

bool sendPhoto = false;
bool autoMode = false;
bool windowOpened = false;
bool windowOpenedBefore = false;


unsigned int restartBoard;
String sendToUser;
String sendToUserBefore;
String lastAutoStatus = "";

// PIR motion sensor configuration
const int motionSensor = 13; //was 4, then 13, then 14

// Volatile because this variable is changed inside an interrupt
volatile bool motionDetected = false; //was bool motionDetected = false;


// Secure TCP client used to communicate with Telegram
WiFiClientSecure clientTCP;

// Telegram bot object
UniversalTelegramBot bot(BOTtoken, clientTCP);


// ============================================================
// FLASH LED
// ============================================================

#define FLASH_LED_PIN 4

bool flashState = LOW;


// Check for new Telegram messages every second
int botRequestDelay = 1000;
unsigned long lastTimeBotRan;


// ============================================================
// ESP32-CAM AI THINKER PIN CONFIGURATION
// ============================================================

// Camera power-down pin
#define PWDN_GPIO_NUM     32

// Camera reset pin
#define RESET_GPIO_NUM    -1

// Camera clock pin
#define XCLK_GPIO_NUM      0

// Camera SCCB/I2C pins
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

// Camera data pins
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5

// Camera synchronization pins
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22


// ============================================================
// CAMERA INITIALIZATION
// ============================================================

void configInitCamera(){

  // Create camera configuration structure
  camera_config_t config;


  // Configure the camera LEDC channel and timer
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;


  // Configure camera data pins
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;


  // Configure camera control pins
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;

  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;

  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;


  // Camera clock frequency
  config.xclk_freq_hz = 20000000;


  // Use JPEG image format
  config.pixel_format = PIXFORMAT_JPEG;

  // Get the latest available camera frame
  config.grab_mode = CAMERA_GRAB_LATEST;


  // ----------------------------------------------------------
  // CAMERA MEMORY CONFIGURATION
  // ----------------------------------------------------------

  // Initialize with higher specifications when PSRAM is available
  if(psramFound()){

    config.frame_size = FRAMESIZE_VGA; //was FRAMESIZE_UXGA

    // 0-63: lower number means higher image quality
    config.jpeg_quality = 12;  //was 10

    config.fb_count = 1;

  } else {

    config.frame_size = FRAMESIZE_SVGA;

    config.jpeg_quality = 12;

    config.fb_count = 1;
  }


  // ----------------------------------------------------------
  // CAMERA INITIALIZATION
  // ----------------------------------------------------------

  esp_err_t err = esp_camera_init(&config);

  if (err != ESP_OK) {

    Serial.printf(
      "Camera init failed with error 0x%x",
      err
    );

    delay(1000);

    // Restart the ESP32 if camera initialization fails
    ESP.restart();
  }
}


// ============================================================
// ESP-NOW CONFIGURATION
// ============================================================

// ESP-NOW peer structure
esp_now_peer_info_t slave;

// Current Wi-Fi channel
int chan;


// Message types used by ESP-NOW
enum MessageType {
  PAIRING,
  DATA,
};

MessageType messageType;


// Counter used for outgoing messages
int counter = 0;


// MAC address of the connected client
uint8_t clientMacAddress[6];


// ============================================================
// DATA STRUCTURE
// ============================================================

// Structure used to receive data from the sensor boards.
// It must match the structure used by the sender.
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

  // AHT21 and ENS160 values
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


// Structure used for ESP-NOW pairing
typedef struct struct_pairing {

    uint8_t msgType;
    uint8_t id;
    uint8_t macAddr[6];
    uint8_t channel;

} struct_pairing;


// Received sensor readings
struct_message incomingReadings;

// Data sent back to the sensor boards
struct_message outgoingSetpoints;

// Pairing information
struct_pairing pairingData;


// ============================================================
// WEB SERVER
// ============================================================

// Asynchronous HTTP server running on port 80
AsyncWebServer server(80);

// Server-Sent Events endpoint
AsyncEventSource events("/events");


// ============================================================
// WEB DASHBOARD
// ============================================================

// HTML page served by the ESP32-CAM
// ============================================================
// WEB DASHBOARD
// ============================================================

// HTML page served by the ESP32-CAM
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <title>ESP-NOW DASHBOARD</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body { font-family: Arial, sans-serif; margin: 0; padding: 20px; background-color: #f4f4f4; color: #333; }
    h3 { text-align: center; margin-top: 0; padding-bottom: 10px; border-bottom: 1px solid #ccc; }
    .grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(320px, 1fr)); gap: 20px; max-width: 1200px; margin: 0 auto; }
    .card { background: #fff; padding: 20px; border-radius: 4px; box-shadow: 0 1px 3px rgba(0,0,0,0.1); }
    .data-row { display: flex; justify-content: space-between; align-items: center; padding: 6px 0; border-bottom: 1px solid #eee; font-size: 0.95rem; }
    .data-row:last-child { border-bottom: none; }
    .data-row span:last-child { font-weight: bold; }
    button { background-color: #007BFF; color: white; border: none; padding: 6px 12px; border-radius: 4px; cursor: pointer; }
    button:active { background-color: #0056b3; }
    input[type="number"] { padding: 5px; width: 60px; border: 1px solid #ccc; border-radius: 4px; margin-right: 5px; }
  </style>
</head>
<body>
  <div class="grid">
    
    <!-- CONTROLS CARD -->
    <div class="card" id="controls">
      <h3>CONTROLS</h3>
      <div class="data-row"><span>Auto Mode:</span> <button onclick="sendCommand('/toggle_auto')">Toggle Auto</button></div>
      <div class="data-row"><span>Relay 1:</span> <button onclick="sendCommand('/toggle_relay1')">Toggle Relay 1</button></div>
      <div class="data-row"><span>Relay 2:</span> <button onclick="sendCommand('/toggle_relay2')">Toggle Relay 2</button></div>
      <div class="data-row">
        <span>Servo Angle:</span> 
        <span>
          <input type="number" id="servoInput" min="0" max="180" value="90">
          <button onclick="setServo()">Set</button>
        </span>
      </div>
    </div>

    <div class="card" id="board1">
      <h3>BOARD #1</h3>
      <div class="data-row"><span>Reading ID:</span> <span id="r_1">-</span></div>
      <div class="data-row"><span>Internal Temp:</span> <span id="tInt_1">-</span> &deg;C</div>
      <div class="data-row"><span>AHT20 (1) Temp / Hum:</span> <span><span id="tAHT1_1">-</span> &deg;C / <span id="hAHT1_1">-</span> %</span></div>
      <div class="data-row"><span>BMP280 (1) Temp:</span> <span id="tBMP1_1">-</span> &deg;C</div>
      <div class="data-row"><span>BMP280 (1) Press / Alt:</span> <span><span id="pBMP1_1">-</span> Pa / <span id="aBMP1_1">-</span> m</span></div>
      <div class="data-row"><span>AHT20 (2) Temp / Hum:</span> <span><span id="tAHT2_1">-</span> &deg;C / <span id="hAHT2_1">-</span> %</span></div>
      <div class="data-row"><span>BMP280 (2) Temp:</span> <span id="tBMP2_1">-</span> &deg;C</div>
      <div class="data-row"><span>BMP280 (2) Press / Alt:</span> <span><span id="pBMP2_1">-</span> Pa / <span id="aBMP2_1">-</span> m</span></div>
      <div class="data-row"><span>AHT21 Temp / Hum:</span> <span><span id="tAHT21_1">-</span> &deg;C / <span id="hAHT21_1">-</span> %</span></div>
      <div class="data-row"><span>ENS160 AQI / TVOC / eCO2:</span> <span><span id="aqi_1">-</span> / <span id="tvoc_1">-</span> / <span id="eco2_1">-</span></span></div>
      <div class="data-row"><span>MQ2 / MQ135:</span> <span><span id="mq2_1">-</span> / <span id="mq135_1">-</span></span></div>
      <div class="data-row"><span>Hall Sensor:</span> <span id="hall_1">-</span></div>
      <div class="data-row"><span>Motion Detected:</span> <span id="motion_1">-</span></div>
      <div class="data-row"><span>Relay 1 / Relay 2:</span> <span><span id="relay1_1">-</span> / <span id="relay2_1">-</span></span></div>
      <div class="data-row"><span>Servo Angle:</span> <span id="servo_1">-</span> &deg;</div>
    </div>

    <div class="card" id="board2">
      <h3>BOARD #2</h3>
      <div class="data-row"><span>Reading ID:</span> <span id="r_2">-</span></div>
      <div class="data-row"><span>Internal Temp:</span> <span id="tInt_2">-</span> &deg;C</div>
      <div class="data-row"><span>AHT20 (1) Temp / Hum:</span> <span><span id="tAHT1_2">-</span> &deg;C / <span id="hAHT1_2">-</span> %</span></div>
      <div class="data-row"><span>BMP280 (1) Temp:</span> <span id="tBMP1_2">-</span> &deg;C</div>
      <div class="data-row"><span>BMP280 (1) Press / Alt:</span> <span><span id="pBMP1_2">-</span> Pa / <span id="aBMP1_2">-</span> m</span></div>
      <div class="data-row"><span>AHT20 (2) Temp / Hum:</span> <span><span id="tAHT2_2">-</span> &deg;C / <span id="hAHT2_2">-</span> %</span></div>
      <div class="data-row"><span>BMP280 (2) Temp:</span> <span id="tBMP2_2">-</span> &deg;C</div>
      <div class="data-row"><span>BMP280 (2) Press / Alt:</span> <span><span id="pBMP2_2">-</span> Pa / <span id="aBMP2_2">-</span> m</span></div>
      <div class="data-row"><span>AHT21 Temp / Hum:</span> <span><span id="tAHT21_2">-</span> &deg;C / <span id="hAHT21_2">-</span> %</span></div>
      <div class="data-row"><span>ENS160 AQI / TVOC / eCO2:</span> <span><span id="aqi_2">-</span> / <span id="tvoc_2">-</span> / <span id="eco2_2">-</span></span></div>
      <div class="data-row"><span>MQ2 / MQ135:</span> <span><span id="mq2_2">-</span> / <span id="mq135_2">-</span></span></div>
      <div class="data-row"><span>Hall Sensor:</span> <span id="hall_2">-</span></div>
      <div class="data-row"><span>Motion Detected:</span> <span id="motion_2">-</span></div>
      <div class="data-row"><span>Relay 1 / Relay 2:</span> <span><span id="relay1_2">-</span> / <span id="relay2_2">-</span></span></div>
      <div class="data-row"><span>Servo Angle:</span> <span id="servo_2">-</span> &deg;</div>
    </div>
  </div>

<script>
// Send standard toggle commands
function sendCommand(url) {
  fetch(url);
}

// Send servo angle
function setServo() {
  var val = document.getElementById("servoInput").value;
  fetch('/set_servo?value=' + val);
}

// Establish a Server-Sent Events connection
if (!!window.EventSource) {

 var source = new EventSource('/events');

 source.addEventListener('open', function(e) {
   console.log("Events Connected");
 }, false);

 source.addEventListener('error', function(e) {
  if (e.target.readyState != EventSource.OPEN) {
    console.log("Events Disconnected");
  }
 }, false);


 // Receive new sensor readings
 source.addEventListener('new_readings', function(e) {
  var obj = JSON.parse(e.data);
  var id = obj.id;

  if(id !== 1 && id !== 2) return;

  var set = function(elId, val, isFloat) {
    var el = document.getElementById(elId + "_" + id);
    if(el && val !== undefined)
      el.innerText = isFloat ? val.toFixed(2) : val;
  };

  set("r", obj.readingId, false);
  set("tInt", obj.temperatureInternal, true);
  set("tAHT1", obj.tempAHT20_1, true);
  set("hAHT1", obj.humidityAHT20_1, true);
  set("tBMP1", obj.tempBMP280_1, true);
  set("pBMP1", obj.pressureBMP280_1, false);
  set("aBMP1", obj.altitudeBMP280_1, true);
  set("tAHT2", obj.tempAHT20_2, true);
  set("hAHT2", obj.humidityAHT20_2, true);
  set("tBMP2", obj.tempBMP280_2, true);
  set("pBMP2", obj.pressureBMP280_2, false);
  set("aBMP2", obj.altitudeBMP280_2, true);
  set("tAHT21", obj.tempAHT21, true);
  set("hAHT21", obj.humidityAHT21, true);
  set("aqi", obj.AQIENS160, false);
  set("tvoc", obj.TVOCENS160, false);
  set("eco2", obj.eCO2ENS160, false);
  set("mq2", obj.MQ2, false);
  set("mq135", obj.MQ135, false);
  set("hall", obj.hall, false);
  set("servo", obj.servo, false);

  var mEl = document.getElementById("motion_" + id);
  if(mEl && obj.motion !== undefined) mEl.innerText = obj.motion ? "Yes" : "No";

  var r1El = document.getElementById("relay1_" + id);
  if(r1El && obj.relay1 !== undefined) r1El.innerText = obj.relay1 ? "ON" : "OFF";

  var r2El = document.getElementById("relay2_" + id);
  if(r2El && obj.relay2 !== undefined) r2El.innerText = obj.relay2 ? "ON" : "OFF";

 }, false);
}
</script>
</body>
</html>)rawliteral";


void autoModeVoid() {

  if (!autoMode) {
    return;
  }

  // =========================================================
  // BOARD 2 - TEMPERATURE AND HUMIDITY
  // =========================================================

  if (incomingReadings.id == 2) {

    // ---------------------------------------------------------
    // LOW TEMPERATURE -> HEATING ON
    // ---------------------------------------------------------

    if (incomingReadings.tempAHT20_1 < 18) {

      outgoingSetpoints.relay2 = true;
      sendToUser = "Low temperature: HEATING ON";
    }

    // ---------------------------------------------------------
    // HIGH TEMPERATURE
    // Open window if outside is cooler
    // Otherwise use the fan
    // ---------------------------------------------------------

    else if (incomingReadings.tempAHT20_1 > 27) {

      if (incomingReadings.tempAHT20_2 <
          incomingReadings.tempAHT20_1) {

        outgoingSetpoints.servo = 90;

        if (!windowOpened) {
          sendToUser = "High temperature: WINDOW OPEN";
        }
      }

      else {

        outgoingSetpoints.relay1 = true;
        sendToUser = "High temperature: FAN ON";
      }
    }


    // ---------------------------------------------------------
    // HIGH HUMIDITY
    // ---------------------------------------------------------

    if (incomingReadings.humidityAHT20_1 > 60) {


      // If outside is less humid, open the window
      if (incomingReadings.humidityAHT20_2 <
          incomingReadings.humidityAHT20_1) {

        outgoingSetpoints.servo = 90;

        if (!windowOpened) {
          sendToUser = "High humidity: WINDOW OPEN";
        }
      }

      // Otherwise use the fan
      else {

        outgoingSetpoints.relay1 = true;
        sendToUser = "High humidity: FAN ON";
      }
    }


    // ---------------------------------------------------------
    // NORMAL CONDITIONS
    // ---------------------------------------------------------

    if (incomingReadings.tempAHT20_1 > 20 &&
        incomingReadings.tempAHT20_1 < 27 &&
        incomingReadings.humidityAHT20_1 < 55) {

      outgoingSetpoints.relay2 = false;
      outgoingSetpoints.relay1 = false;
      outgoingSetpoints.servo = 0;

      sendToUser = "Temperature and humidity normal";
    }
  }


  // =========================================================
  // BOARD 1 - AIR QUALITY
  // =========================================================

  if (incomingReadings.id == 1) {

    // ---------------------------------------------------------
    // VERY BAD AIR
    // ---------------------------------------------------------

    if (incomingReadings.eCO2ENS160 > 1500 ||
        incomingReadings.MQ2 > 2500) {

      outgoingSetpoints.relay1 = true;
      outgoingSetpoints.servo = 90;

      sendToUser = "WARNING: VERY BAD AIR QUALITY!";
    }

    // ---------------------------------------------------------
    // BAD AIR
    // ---------------------------------------------------------

    else if (incomingReadings.eCO2ENS160 > 1000 ||
             incomingReadings.MQ2 > 1500) {

      outgoingSetpoints.relay1 = true;
      outgoingSetpoints.servo = 90;

      sendToUser = "Bad air quality: FAN ON, WINDOW OPEN";
    }


    // ---------------------------------------------------------
    // HALL SENSOR
    // ---------------------------------------------------------

    if (incomingReadings.hall < 2000) {
      windowOpened = false;
    }

    else {
      windowOpened = true;
    }
  }
}
// ============================================================
// SEND CONTROL PACKET
// ============================================================

void sendControlPacket() {

  // Set packet type
  outgoingSetpoints.msgType = DATA;

  // ID 0 identifies the server
  outgoingSetpoints.id = 0;

  // Read ESP32 internal temperature
  outgoingSetpoints.tempInternal = temperatureRead();

  // Read PIR state
  outgoingSetpoints.motion = motionDetected;

  // Increment packet counter
  outgoingSetpoints.readingId = counter++;


  // Send the control packet to the connected ESP-NOW peers
  esp_now_send(
    NULL,
    (uint8_t *) &outgoingSetpoints,
    sizeof(outgoingSetpoints)
  );
}


// ============================================================
// TELEGRAM MESSAGE HANDLER
// ============================================================

void handleNewMessages(int numNewMessages) {

  Serial.print("Handle New Messages: ");
  Serial.println(numNewMessages);


  // Process every new Telegram message
  for (int i = 0; i < numNewMessages; i++) {

    // Get the sender's chat ID
    String chat_id = String(bot.messages[i].chat_id);


    // Reject unauthorized users
    if (chat_id != CHAT_ID){

      bot.sendMessage(
        chat_id,
        "Unauthorized user",
        ""
      );

      continue;
    }


    // Print the received command
    String text = bot.messages[i].text;

    Serial.println(text);


    // Get the sender's name
    String from_name = bot.messages[i].from_name;


    // --------------------------------------------------------
    // GET SYSTEM STATE
    // --------------------------------------------------------

    if (
      text == "/get_state@FridayHomeAssistantBot" ||
      text == "/get_state"
    ) {

      bot.sendChatAction(chat_id, "typing");

      float temp = temperatureRead();

      String state =
        "Hello , " + from_name +
        ". The system is working \n";

      state +=
        "ESP32-CAM internal temperature is " +
        String(temp) +
        "°C \n";

      state +=
        "ESP32-CAM flash state is " +
        String(flashState) +
        "\n";

      state +=
        "PIR state is " +
        String(motionDetected) +
        "\n";

      bot.sendMessage(
        CHAT_ID,
        state,
        ""
      );
    }


    // --------------------------------------------------------
    // SWITCH FLASH
    // --------------------------------------------------------

    if (
      text == "/switch_flash@FridayHomeAssistantBot" ||
      text == "/switch_flash"
    ) {

      bot.sendChatAction(chat_id, "typing");

      // Toggle flash state
      flashState = !flashState;

      digitalWrite(
        FLASH_LED_PIN,
        flashState
      );


      String flash =
        "Change flash LED state to " +
        String(flashState);

      bot.sendMessage(
        CHAT_ID,
        flash,
        ""
      );

      Serial.println(
        "Change flash LED state to " +
        String(flashState)
      );
    }


    // --------------------------------------------------------
    // GET SENSOR READINGS
    // --------------------------------------------------------

    if (
      text == "/get_sensors_readings@FridayHomeAssistantBot" ||
      text == "/get_sensors_readings"
    ) {

      bot.sendChatAction(chat_id, "typing");

      float temp = temperatureRead();

      String doorOpened;


      // Determine door state using the hall sensor
      if (incomingReadings.hall < 2000) {

        doorOpened = "closed";

      } else {

        doorOpened = "opened";
      }


      // Create the sensor readings message
      String readings =
        "ESP32-CAM internal temperature is " +
        String(temp) +
        "°C \n";

      readings +=
        "ESP32-CAM flash state is " +
        String(flashState) +
        "\n";

      readings +=
        "PIR state is " +
        String(motionDetected) +
        "\n";

      readings += "\n";


      readings +=
        "From board with ID " +
        String(incomingReadings.id) +
        "int. temperature is " +
        String(incomingReadings.tempInternal) +
        "°C\n";


      readings +=
        "From AHT20: " +
        String(incomingReadings.tempAHT20_1) +
        "°C, " +
        String(incomingReadings.humidityAHT20_1) +
        "%\n";


      readings +=
        "From BMP280: " +
        String((incomingReadings.pressureBMP280_1)/101325.0) +
        "atm (" +
        String(incomingReadings.pressureBMP280_1) +
        "kp), " +
        String(incomingReadings.tempBMP280_1) +
        "°C, " +
        String(incomingReadings.altitudeBMP280_1) +
        "m\n";


      /*readings +=
        "From AHT20(2): " +
        String(incomingReadings.tempAHT20_2) +
        "°C, " +
        String(incomingReadings.humidityAHT20_2) +
        "%\n";


      readings +=
        "From BMP280(2): " +
        String((incomingReadings.pressureBMP280_2)/101325.0) +
        "atm (" +
        String(incomingReadings.pressureBMP280_2) +
        "kp), " +
        String(incomingReadings.tempBMP280_2) +
        "°C, " +
        String(incomingReadings.altitudeBMP280_2) +
        "m\n";*/


      readings +=
        "From AHT21: " +
        String(incomingReadings.tempAHT21) +
        "°C, " +
        String(incomingReadings.humidityAHT21) +
        "%\n";


      readings +=
        "From ENS160: " +
        String(incomingReadings.AQIENS160) +
        " AQI, " +
        String(incomingReadings.TVOCENS160) +
        " TVOC, " +
        String(incomingReadings.eCO2ENS160) +
        " eCO2\n";


      readings +=
        "From MQ2: " +
        String(incomingReadings.MQ2) +
        ", MQ135: " +
        String(incomingReadings.MQ135);


      readings +=
        "From Hall Sensor: " +
        String(incomingReadings.hall) +
        ", door is " +
        String(doorOpened);


      readings +=
        "(readings ID: " +
        String(incomingReadings.readingId) +
        ")\n";


      // Send the readings to Telegram
      bot.sendMessage(
        CHAT_ID,
        readings,
        ""
      );
    }


    // --------------------------------------------------------
    // SWITCH AUTOMATIC MODE
    // --------------------------------------------------------

    if (
      text == "/switch_auto@FridayHomeAssistantBot" ||
      text == "/switch_auto"
    ) {

      bot.sendChatAction(chat_id, "typing");

      // Toggle automatic control
      autoMode = !autoMode;

      String autoModeChange =
        "Auto mode switched to " +
        String(autoMode);

      bot.sendMessage(
        CHAT_ID,
        autoModeChange,
        ""
      );
    }


    // --------------------------------------------------------
    // SWITCH RELAY 1
    // --------------------------------------------------------

    if (
      text == "/switch_relay1@FridayHomeAssistantBot" ||
      text == "/switch_relay1"
    ) {

      bot.sendChatAction(chat_id, "typing");

      // Manual control disables automatic mode
      autoMode = false;

      // Toggle relay 1
      outgoingSetpoints.relay1 =
        !outgoingSetpoints.relay1;

      // Send the new relay state
      sendControlPacket();

      String relay1Change =
        "Relay 1 switched to " +
        String(outgoingSetpoints.relay1);

      bot.sendMessage(
        CHAT_ID,
        relay1Change,
        ""
      );
    }


    // --------------------------------------------------------
    // SWITCH RELAY 2
    // --------------------------------------------------------

    if (
      text == "/switch_relay2@FridayHomeAssistantBot" ||
      text == "/switch_relay2"
    ) {

      bot.sendChatAction(chat_id, "typing");

      // Manual control disables automatic mode
      autoMode = false;

      // Toggle relay 2
      outgoingSetpoints.relay2 =
        !outgoingSetpoints.relay2;

      // Send the new relay state
      sendControlPacket();

      String relay2Change =
        "Relay 2 switched to " +
        String(outgoingSetpoints.relay2);

      bot.sendMessage(
        CHAT_ID,
        relay2Change,
        ""
      );
    }


    // --------------------------------------------------------
    // SET SERVO ANGLE
    // --------------------------------------------------------

    if (text.startsWith("/servo ")) {

      // Manual servo control disables automatic mode
      autoMode = false;

      // Extract the requested angle
      int val = text.substring(7).toInt();

      // Limit the angle to 0-180 degrees
      outgoingSetpoints.servo =
        constrain(val, 0, 180);

      // Send the new servo position
      sendControlPacket();

      bot.sendMessage(
        CHAT_ID,
        "Servo angle set to " +
        String(outgoingSetpoints.servo),
        ""
      );
    }


    // --------------------------------------------------------
    // REQUEST PHOTO
    // --------------------------------------------------------

    /*if (text == "/weather@FridayHomeAssistantBot" || text == "/weather") {
      bot.sendChatAction(chat_id, "typing");
      if (incomingReadings.tempAHT20_2) {

      }
      String autoModeChange = "Auto mode switched to " + String(autoMode);
      bot.sendMessage(CHAT_ID, autoModeChange, "");
    }*/

    if (
      text == "/get_photo@FridayHomeAssistantBot" ||
      text == "/get_photo"
    ) {

      bot.sendChatAction(
        chat_id,
        "upload_photo"
      );

      // Set the photo request flag
      sendPhoto = true;

      Serial.println("New photo request");
    }


    // --------------------------------------------------------
    // REQUEST RESTART
    // --------------------------------------------------------

    if (
      text == "/restart@FridayHomeAssistantBot" ||
      text == "/restart"
    ) {

      bot.sendChatAction(
        chat_id,
        "typing"
      );


      /*String sure = "Are you sure?\n";
      sure += "/yes\n\n";
      sure += "/no\n";
      bot.sendMessage(chat_id, sure, "Markdown");*/


      // Reply keyboard asking the user to confirm restart
      String sure =
        "[[\"/yes\", \"/no\"],[\"/get_state\"]]";

      bot.sendMessageWithReplyKeyboard(
        chat_id,
        "Are you sure?",
        "",
        sure,
        true
      );
    }


    // --------------------------------------------------------
    // CONFIRM RESTART
    // --------------------------------------------------------

    if (
      text == "/yes@FridayHomeAssistantBot" ||
      text == "/yes"
    ) {

      bot.sendChatAction(
        chat_id,
        "typing"
      );


      // Prevent restart immediately after boot
      if (millis() > 10000) {

        String restart =
          "Restarting the board...";

        bot.sendMessage(
          CHAT_ID,
          restart,
          ""
        );

        bot.sendChatAction(
          chat_id,
          "typing"
        );

        Serial.println("Restarting");

        // Restart the ESP32
        ESP.restart();

      } else {

        String restartSuccess =
          "Board restared";

        bot.sendMessage(
          CHAT_ID,
          restartSuccess,
          ""
        );
      }
    }
  }
}


// ============================================================
// TELEGRAM PHOTO FUNCTION
// ============================================================

String sendPhotoTelegram() {

  const char* myDomain =
    "api.telegram.org";

  String getAll = "";
  String getBody = "";


  // ----------------------------------------------------------
  // DISCARD FIRST FRAME
  // ----------------------------------------------------------

  // Dispose first picture because of bad quality
  camera_fb_t * fb = NULL;

  fb = esp_camera_fb_get();

  esp_camera_fb_return(fb);


  // ----------------------------------------------------------
  // CAPTURE NEW PHOTO
  // ----------------------------------------------------------

  fb = NULL;

  fb = esp_camera_fb_get();

  if(!fb) {

    Serial.println("Camera capture failed");

    delay(1000);

    ESP.restart();

    return "Camera capture failed";
  }


  Serial.println(
    "Connect to " +
    String(myDomain)
  );


  // ----------------------------------------------------------
  // CONNECT TO TELEGRAM
  // ----------------------------------------------------------

  if (clientTCP.connect(myDomain, 443)) {

    Serial.println("Connection successful");


    // Multipart form-data header
    String head =
      "--Friday\r\n"
      "Content-Disposition: form-data; name=\"chat_id\"; \r\n\r\n" +
      CHAT_ID +
      "\r\n"
      "--Friday\r\n"
      "Content-Disposition: form-data; name=\"photo\"; filename=\"esp32-cam.jpg\"\r\n"
      "Content-Type: image/jpeg\r\n\r\n";


    // Multipart form-data footer
    String tail =
      "\r\n--Friday--\r\n";


    // Calculate total request size
    size_t imageLen = fb->len;

    size_t extraLen =
      head.length() +
      tail.length();

    size_t totalLen =
      imageLen +
      extraLen;


    // Send HTTP POST request
    clientTCP.println(
      "POST /bot" +
      BOTtoken +
      "/sendPhoto HTTP/1.1"
    );

    clientTCP.println(
      "Host: " +
      String(myDomain)
    );

    clientTCP.println(
      "Content-Length: " +
      String(totalLen)
    );

    clientTCP.println(
      "Content-Type: multipart/form-data; boundary=Friday"
    );

    clientTCP.println();


    // Send multipart header
    clientTCP.print(head);


    // Send image data in chunks
    uint8_t *fbBuf = fb->buf;

    size_t fbLen = fb->len;


    for (
      size_t n=0;
      n<fbLen;
      n=n+1024
    ) {

      if (n+1024<fbLen) {

        clientTCP.write(
          fbBuf,
          1024
        );

        fbBuf += 1024;

      }

      else if (fbLen%1024>0) {

        size_t remainder =
          fbLen%1024;

        clientTCP.write(
          fbBuf,
          remainder
        );
      }
    }


    // Send multipart footer
    clientTCP.print(tail);


    // Release camera frame buffer
    esp_camera_fb_return(fb);


    // Wait for Telegram response
    int waitTime = 10000;

    long startTimer = millis();

    boolean state = false;


    while (
      (startTimer + waitTime) >
      millis()
    ){

      Serial.print(".");

      delay(100);


      while (clientTCP.available()) {

        char c = clientTCP.read();

        if (state==true)
          getBody += String(c);


        if (c == '\n') {

          if (getAll.length()==0)
            state=true;

          getAll = "";

        }

        else if (c != '\r') {

          getAll += String(c);
        }


        startTimer = millis();
      }


      if (getBody.length()>0)
        break;
    }


    // Close the connection
    clientTCP.stop();

    Serial.println(getBody);

  }

  else {

    getBody =
      "Connected to api.telegram.org failed.";

    Serial.println(
      "Connected to api.telegram.org failed."
    );
  }


  return getBody;
}


// ============================================================
// READ ESP32-CAM MAC ADDRESS
// ============================================================

void readMacAddress(){

  uint8_t baseMac[6];

  esp_err_t ret =
    esp_wifi_get_mac(
      WIFI_IF_STA,
      baseMac
    );


  if (ret == ESP_OK) {

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
}

void IRAM_ATTR detectsMovement() {
  motionDetected = true;
}
// ============================================================
// PREPARE CONTROL DATA
// ============================================================

void readDataToSend() {

  // Set message type
  outgoingSetpoints.msgType = DATA;

  // ID 0 identifies the server
  outgoingSetpoints.id = 0;

  // ESP32 internal temperature
  outgoingSetpoints.tempInternal =
    temperatureRead();

  // Copy sensor values from the received structure
  outgoingSetpoints.tempAHT20_1 =
    incomingReadings.tempAHT20_1;

  outgoingSetpoints.humidityAHT20_1 =
    incomingReadings.humidityAHT20_1;

  outgoingSetpoints.tempAHT20_2 =
    incomingReadings.tempAHT20_2;

  outgoingSetpoints.humidityAHT20_2 =
    incomingReadings.humidityAHT20_2;

  outgoingSetpoints.pressureBMP280_1 =
    incomingReadings.pressureBMP280_1;

  outgoingSetpoints.pressureBMP280_2 =
    incomingReadings.pressureBMP280_2;

  // Current motion state
  outgoingSetpoints.motion =
    motionDetected;

  // Increment the packet counter
  outgoingSetpoints.readingId =
    counter++;
}


// ============================================================
// ESP-NOW
// ============================================================

void printMAC(const uint8_t * mac_addr){

  char macStr[18];

  // Convert MAC address to a printable string
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
// ADD ESP-NOW PEER
// ============================================================

bool addPeer(const uint8_t *peer_addr) {

  // Clear peer information
  memset(
    &slave,
    0,
    sizeof(slave)
  );


  const esp_now_peer_info_t *peer =
    &slave;


  // Copy peer MAC address
  memcpy(
    slave.peer_addr,
    peer_addr,
    6
  );


  // Use the current Wi-Fi channel
  slave.channel = chan;

  // Disable encryption
  slave.encrypt = 0;


  // Check whether the peer already exists
  bool exists =
    esp_now_is_peer_exist(
      slave.peer_addr
    );


  if (exists) {

    // Peer is already paired
    Serial.println("Already Paired");

    return true;

  }

  else {

    // Add the peer
    esp_err_t addStatus =
      esp_now_add_peer(peer);


    if (addStatus == ESP_OK) {

      // Pairing succeeded
      Serial.println("Pair success");

      return true;

    }

    else {

      // Pairing failed
      Serial.println("Pair failed");

      return false;
    }
  }
}


// ============================================================
// ESP-NOW SEND CALLBACK
// ============================================================

// Called when an ESP-NOW packet has been sent
void OnDataSent(
  const wifi_tx_info_t* mac_addr,
  esp_now_send_status_t status
) {

  char macStr[18];


  Serial.print("Last Packet Send Status: ");

  Serial.print(
    status == ESP_NOW_SEND_SUCCESS
    ? "Delivery Success to "
    : "Delivery Fail to "
  );


  // Copies the receiver MAC address to a string
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

  Serial.println();
}


// ============================================================
// ESP-NOW RECEIVE CALLBACK
// ============================================================

void OnDataRecv(
  const uint8_t * mac_addr,
  const uint8_t *incomingData,
  int len
) {

  // Print the size of the received packet
  Serial.print(len);

  Serial.println(
    " bytes of new data received."
  );


  // Create a JSON document for web dashboard data
  StaticJsonDocument<1000> root;

  String payload;


  // First byte identifies the message type
  uint8_t type = incomingData[0];


  switch (type) {


    // --------------------------------------------------------
    // DATA MESSAGE
    // --------------------------------------------------------

    case DATA:

      // Copy received packet into the data structure
      memcpy(
        &incomingReadings,
        incomingData,
        sizeof(incomingReadings)
      );


      // ------------------------------------------------------
      // CREATE JSON DATA
      // ------------------------------------------------------

      // Board identification
      root["id"] =
        incomingReadings.id;

      // Internal temperature
      root["temperatureInternal"] =
        incomingReadings.tempInternal;


      // AHT20 #1
      root["tempAHT20_1"] =
        incomingReadings.tempAHT20_1;

      root["humidityAHT20_1"] =
        incomingReadings.humidityAHT20_1;


      // BMP280 #1
      root["pressureBMP280_1"] =
        incomingReadings.pressureBMP280_1;

      root["tempBMP280_1"] =
        incomingReadings.tempBMP280_1;

      root["altitudeBMP280_1"] =
        incomingReadings.altitudeBMP280_1;


      // AHT20 #2
      root["tempAHT20_2"] =
        incomingReadings.tempAHT20_2;

      root["humidityAHT20_2"] =
        incomingReadings.humidityAHT20_2;


      // BMP280 #2
      root["pressureBMP280_2"] =
        incomingReadings.pressureBMP280_2;

      root["tempBMP280_2"] =
        incomingReadings.tempBMP280_2;

      root["altitudeBMP280_2"] =
        incomingReadings.altitudeBMP280_2;


      // AHT21
      root["tempAHT21"] =
        incomingReadings.tempAHT21;

      root["humidityAHT21"] =
        incomingReadings.humidityAHT21;


      // ENS160
      root["AQIENS160"] =
        incomingReadings.AQIENS160;

      root["TVOCENS160"] =
        incomingReadings.TVOCENS160;

      root["eCO2ENS160"] =
        incomingReadings.eCO2ENS160;


      // Analog sensors
      root["MQ2"] =
        incomingReadings.MQ2;

      root["MQ135"] =
        incomingReadings.MQ135;

      root["hall"] =
        incomingReadings.hall;


      // Motion sensor
      root["motion"] =
        incomingReadings.motion;


      // Relay states
      root["relay1"] =
        incomingReadings.relay1;

      root["relay2"] =
        incomingReadings.relay2;


      // Servo angle
      root["servo"] =
        incomingReadings.servo;


      // Reading counter
      root["readingId"] =
        String(incomingReadings.readingId);


      // Convert JSON document into a string
      serializeJson(
        root,
        payload
      );


      // Print the generated event
      Serial.print("event send :");

      serializeJson(
        root,
        Serial
      );


      // Send the JSON data to connected web clients
      events.send(
        payload.c_str(),
        "new_readings",
        millis()
      );


      // Run automatic control logic
      autoModeVoid();

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


      Serial.println(
        pairingData.msgType
      );

      Serial.println(
        pairingData.id
      );


      Serial.print(
        "Pairing request from MAC Address: "
      );

      printMAC(
        pairingData.macAddr
      );


      Serial.print(" on channel ");

      Serial.println(
        pairingData.channel
      );


      // Copy client MAC address
      clientMacAddress[0] =
        pairingData.macAddr[0];

      clientMacAddress[1] =
        pairingData.macAddr[1];

      clientMacAddress[2] =
        pairingData.macAddr[2];

      clientMacAddress[3] =
        pairingData.macAddr[3];

      clientMacAddress[4] =
        pairingData.macAddr[4];

      clientMacAddress[5] =
        pairingData.macAddr[5];


      // Ignore messages from the server itself
      if (pairingData.id > 0) {

        if (pairingData.msgType == PAIRING) {

          // ID 0 identifies the server
          pairingData.id = 0;


          // Server is in AP_STA mode.
          // Clients need the Soft AP MAC address.
          WiFi.softAPmacAddress(
            pairingData.macAddr
          );


          Serial.print(
            "Pairing MAC Address: "
          );

          printMAC(
            clientMacAddress
          );


          // Use the current Wi-Fi channel
          pairingData.channel = chan;

          Serial.println(
            " send response"
          );


          // Send pairing response to the client
          esp_err_t result =
            esp_now_send(
              clientMacAddress,
              (uint8_t *) &pairingData,
              sizeof(pairingData)
            );


          // Add the client as an ESP-NOW peer
          addPeer(
            clientMacAddress
          );
        }
      }

      break;
  }
}


// ============================================================
// INITIALIZE ESP-NOW
// ============================================================

void initESP_NOW(){

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {

    Serial.println(
      "Error initializing ESP-NOW"
    );

    return;
  }


  // Register ESP-NOW callbacks
  esp_now_register_send_cb(
    esp_now_send_cb_t(OnDataSent)
  );

  esp_now_register_recv_cb(
    esp_now_recv_cb_t(OnDataRecv)
  );
}


// ============================================================
// SETUP
// ============================================================

void setup() {

  // Disable brownout detection
  // This prevents resets when the board experiences
  // a temporary voltage drop.
  WRITE_PERI_REG(
    RTC_CNTL_BROWN_OUT_REG,
    0
  );


  // Initialize Serial Monitor
  Serial.begin(115200);


  // ----------------------------------------------------------
  // PIR MOTION SENSOR
  // ----------------------------------------------------------

  pinMode(
    motionSensor,
    INPUT_PULLUP
  );

  attachInterrupt(digitalPinToInterrupt(motionSensor), detectsMovement, RISING);


  // ----------------------------------------------------------
  // FLASH LED
  // ----------------------------------------------------------

  pinMode(
    FLASH_LED_PIN,
    OUTPUT
  );

  digitalWrite(
    FLASH_LED_PIN,
    flashState
  );


  // ----------------------------------------------------------
  // CAMERA
  // ----------------------------------------------------------

  configInitCamera();


  // ----------------------------------------------------------
  // WI-FI
  // ----------------------------------------------------------

  // Configure the ESP32 as both station and access point
  WiFi.mode(WIFI_AP_STA);

  WiFi.STA.begin();


  // Print server MAC address
  Serial.print(
    "Server MAC Address: "
  );

  readMacAddress();


  // Set the device as Station + Soft AP
  WiFi.mode(WIFI_AP_STA);


  // Connect to the configured Wi-Fi network
  WiFi.begin(
    ssid,
    password
  );


  // Wait until Wi-Fi connection is established
  while (
    WiFi.status() != WL_CONNECTED
  ) {

    Serial.print(".");

    delay(500);
  }


  // Store the current Wi-Fi channel
  chan = WiFi.channel();


  // Configure Telegram certificate
  clientTCP.setCACert(
    TELEGRAM_CERTIFICATE_ROOT
  );


  Serial.println();


  // Print the local IP address
  Serial.print(
    "ESP32-CAM IP Address: "
  );

  Serial.println(
    WiFi.localIP()
  );


  // Send initialization message to Telegram
  String init =
    "Friday system initialized";

  bot.sendMessage(
    CHAT_ID,
    init,
    ""
  );


  // Print Soft AP MAC address
  Serial.print(
    "Server SOFT AP MAC Address:  "
  );

  Serial.println(
    WiFi.softAPmacAddress()
  );


  // Store Wi-Fi channel
  chan = WiFi.channel();


  // Print station IP address
  Serial.print(
    "Station IP Address: "
  );

  Serial.println(
    WiFi.localIP()
  );


  // Print current Wi-Fi channel
  Serial.print(
    "Wi-Fi Channel: "
  );

  Serial.println(
    WiFi.channel()
  );


  // ----------------------------------------------------------
  // ESP-NOW
  // ----------------------------------------------------------

  initESP_NOW();


  // ----------------------------------------------------------
  // WEB SERVER
  // ----------------------------------------------------------

  // Serve the dashboard HTML page
  server.on(
    "/",
    HTTP_GET,
    [](AsyncWebServerRequest *request){

      request->send(
        200,
        "text/html",
        index_html
      );
    }
  );


  // ----------------------------------------------------------
  // SERVER-SENT EVENTS
  // ----------------------------------------------------------

  events.onConnect(
    [](AsyncEventSourceClient *client){

      if(client->lastId()){

        Serial.printf(
          "Client reconnected! Last message ID that it got is: %u\n",
          client->lastId()
        );
      }


      // Send a hello event and configure reconnect delay
      client->send(
        "hello!",
        NULL,
        millis(),
        10000
      );
    }
  );


  // Add event handler to the web server
  server.addHandler(&events);

  // Auto Mode Toggle
  server.on("/toggle_auto", HTTP_GET, [](AsyncWebServerRequest *request){
    autoMode = !autoMode;
    request->send(200, "text/plain", "OK");
  });

  // Relay 1 Toggle
  server.on("/toggle_relay1", HTTP_GET, [](AsyncWebServerRequest *request){
    autoMode = false; // Manual override disables auto
    outgoingSetpoints.relay1 = !outgoingSetpoints.relay1;
    sendControlPacket();
    request->send(200, "text/plain", "OK");
  });

  // Relay 2 Toggle
  server.on("/toggle_relay2", HTTP_GET, [](AsyncWebServerRequest *request){
    autoMode = false; // Manual override disables auto
    outgoingSetpoints.relay2 = !outgoingSetpoints.relay2;
    sendControlPacket();
    request->send(200, "text/plain", "OK");
  });

  // Servo Control
  server.on("/set_servo", HTTP_GET, [](AsyncWebServerRequest *request){
    if (request->hasParam("value")) {
      autoMode = false; // Manual override disables auto
      String val = request->getParam("value")->value();
      outgoingSetpoints.servo = constrain(val.toInt(), 0, 180);
      sendControlPacket();
    }
    request->send(200, "text/plain", "OK");
  });
  // Start the web server
  server.begin();
}


// ============================================================
// MAIN LOOP
// ============================================================

void loop() {

  // Track time between dashboard/control events
  static unsigned long lastEventTime =
    millis();

  static const unsigned long EVENT_INTERVAL_MS =
    5000;


  // Send events and control packets every 5 seconds
  if (
    (millis() - lastEventTime) >
    EVENT_INTERVAL_MS
  ) {

    // Send keep-alive event
    events.send(
      "ping",
      NULL,
      millis()
    );


    // Update timer
    lastEventTime = millis();


    // Prepare outgoing control data
    readDataToSend();


    // Send control packet to ESP-NOW peers
    esp_now_send(
      NULL,
      (uint8_t *) &outgoingSetpoints,
      sizeof(outgoingSetpoints)
    );
  }


  // ----------------------------------------------------------
  // TELEGRAM PHOTO REQUEST
  // ----------------------------------------------------------

  if (sendPhoto) {

    Serial.println(
      "Preparing photo"
    );


    // Capture and send photo
    sendPhotoTelegram();


    // Clear the request flag
    sendPhoto = false;
  }


  // ----------------------------------------------------------
  // MOTION DETECTION
  // ----------------------------------------------------------

  if (motionDetected){

    Serial.println(
      "Motion detected"
    );


    // Notify the Telegram chat
    bot.sendMessage(
      CHAT_ID,
      "Motion detected",
      ""
    );


    // Capture and send a photo
    sendPhotoTelegram();


    // Reset motion flag
    motionDetected = false;


    Serial.println(
      "Motion detected"
    );
  }


  // ----------------------------------------------------------
  // TELEGRAM MESSAGES
  // ----------------------------------------------------------
  if (sendToUser != sendToUserBefore) {
    
    bot.sendChatAction(CHAT_ID, "typing");
    bot.sendMessage(CHAT_ID, sendToUser, "");

    sendToUserBefore = sendToUser;
    
  }
  if (windowOpened != windowOpenedBefore) {
    bot.sendChatAction(CHAT_ID, "typing");
    String windowState = windowOpened ? "opened" : "closed";
    bot.sendMessage(CHAT_ID, windowState, "");
    windowOpenedBefore = windowOpened;
  }
  // Check Telegram for new messages
  if (
    millis() >
    lastTimeBotRan +
    botRequestDelay
  ) {

    int numNewMessages =
      bot.getUpdates(
        bot.last_message_received + 1
      );


    // Process all available messages
    while (numNewMessages) {

      Serial.println(
        "got response"
      );


      // Handle received commands
      handleNewMessages(
        numNewMessages
      );


      // Check for additional messages
      numNewMessages =
        bot.getUpdates(
          bot.last_message_received + 1
        );
    }


    // Store the last Telegram polling time
    lastTimeBotRan = millis();
  }
}
