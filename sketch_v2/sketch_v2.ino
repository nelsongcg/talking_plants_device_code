#undef IRAM_ATTR
#define IRAM_ATTR   // no-op

#include "secrets.h" 
#include <WiFiProv.h>
#include <WiFi.h>
#include <nvs_flash.h>
#include <esp_sleep.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_SHTC3.h>
#include <BH1750.h>
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <WiFiClientSecure.h>  // for TLS connection
#include <PubSubClient.h>      // MQTT client
#include <ArduinoJson.h>
#include <esp_system.h>
#include <esp_adc_cal.h>
// ----------------------------------------------------------------------------
//  Configuration
// ----------------------------------------------------------------------------

// Wifi provisioning config
const char *POP            = "ABC123";
const char *SERVICE_NAME   = "ITTP";

// Define screen dimensions
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

// Define I2C pins for ESP32
#define OLED_SDA 21
#define OLED_SCL 22

// control the energy to transistor
const int sensorSwitchPin = 25;

// sleep time lenght
constexpr uint64_t SLEEP_US = 30ULL * 60ULL * 1000000ULL;

// sendor variables
// String sensortime ;
float temperature;
float soilmoisture;
float luminosity;
float relativeHumidity;
// char idStr[18];
uint8_t factoryMac[6];



// Definitions for battery readings
const int  BAT_ADC_PIN = 35;        // GPIO 35 = ADC1_CH7
const int  ADC_BITS    = 12;        // 0–4095
const float R1 = 10000.0;           // ohms (top)
const float R2 =  1000.0;           // ohms (bottom)
const float DIV_MULT = (R1 + R2) / R2;   // = 11.0
static esp_adc_cal_characteristics_t adc_chars;


// display setup
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// luminosity sensor setup
BH1750 lightMeter;

// co2 humidity and temperature sensor setup
Adafruit_SHTC3 shtc3;
static char errorMessage[64];
static int16_t error;

#ifdef NO_ERROR
#undef NO_ERROR
#endif
#define NO_ERROR 0


// seoil moisture setup
#define SOIL_SENSOR_PIN 34


// AWS setup
#define AWS_IOT_PUBLISH_TOPIC   "esp32/pub"
#define AWS_IOT_SUBSCRIBE_TOPIC "esp32/sub"

// use the onboard BOOT button; wire button → GPIO0, other side → GND
#define CLEAR_BTN_PIN 27


WiFiClientSecure net = WiFiClientSecure();
PubSubClient client(net);

// ----------------------------------------------------------------------------
//  Forward declarations
// ----------------------------------------------------------------------------
void scanNetworks();
bool attemptWiFiConnect();
void startProvisioning();
void onWiFiConnected();
void SysProvEvent(arduino_event_t *e);

// ----------------------------------------------------------------------------
//  helper functions
// ----------------------------------------------------------------------------
void PrintUint64(uint64_t& value) {
    Serial.print("0x");
    Serial.print((uint32_t)(value >> 32), HEX);
    Serial.print((uint32_t)(value & 0xFFFFFFFF), HEX);
}

void messageHandler(char* topic, byte* payload, unsigned int length) {
  Serial.print("incoming: ");
  Serial.println(topic);
 
  JsonDocument doc;
  deserializeJson(doc, payload);
  const char* message = doc["message"];
  Serial.println(message);
}

void forgetCredentialsAndReboot() {
  Serial.println("Erasing NVS…");
  ESP_ERROR_CHECK( nvs_flash_erase() );
  ESP_ERROR_CHECK( nvs_flash_init() );  
  Serial.println("Rebooting…");
  esp_restart();
}

// ----------------------------------------------------------------------------
//  Helper: scan and print all visible 2.4 GHz networks
// ----------------------------------------------------------------------------
void scanNetworks() {
  Serial.println("\n[SCAN] Scanning for Wi-Fi networks…");
  int n = WiFi.scanNetworks();
  if (n == 0) {
    Serial.println("[SCAN] No networks found");
  } else {
    for (int i = 0; i < n; ++i) {
      Serial.printf("[%2d] %-32s  RSSI:%4d  Ch:%2d  %s\n",
        i,
        WiFi.SSID(i).c_str(),
        WiFi.RSSI(i),
        WiFi.channel(i),
        (WiFi.encryptionType(i) == WIFI_AUTH_OPEN) ? "Open" : "Encrypted"
      );
    }
  }
  Serial.println("[SCAN] Done\n");
}

// ----------------------------------------------------------------------------
//  Connect to AWS
// ----------------------------------------------------------------------------
void connectAWS() {
 
  // Configure WiFiClientSecure to use the AWS IoT device credentials
  net.setCACert(AWS_CERT_CA);
  net.setCertificate(AWS_CERT_CRT);
  net.setPrivateKey(AWS_CERT_PRIVATE);
 
  // Connect to the MQTT broker on the AWS endpoint we defined earlier
  client.setServer(AWS_IOT_ENDPOINT, 8883);
 
  // Create a message handler
  client.setCallback(messageHandler);
 
  Serial.println("Connecting to AWS IOT");
 
  while (!client.connect(THINGNAME))
  {
    Serial.print(".");
    delay(100);
  }
 
  if (!client.connected())
  {
    Serial.println("AWS IoT Timeout!");
    return;
  }
 
  // Subscribe to a topic
  client.subscribe(AWS_IOT_SUBSCRIBE_TOPIC);
 
  Serial.println("AWS IoT Connected!");
}

// ----------------------------------------------------------------------------
//  Try to connect with stored credentials (returns true on success)
// ----------------------------------------------------------------------------
bool attemptWiFiConnect() {
  Serial.println("[SETUP] Attempting Wi-Fi connect with stored credentials…");
  oledMessage("Attempting Wi-Fi connection",1);
  WiFi.begin();
  unsigned long start = millis();
  while (millis() - start < 15000) {  // 15s timeout
    if (WiFi.status() == WL_CONNECTED) {
      return true;
    }
    delay(500);
  }
  Serial.println("[SETUP] Stored Wi-Fi connect FAILED");
  return false;
}

// ----------------------------------------------------------------------------
//  Kick off BLE provisioning (resetting any prior state)
// ----------------------------------------------------------------------------
void startProvisioning() {
  Serial.println("[SETUP] Starting BLE provisioning…");
  
  WiFiProv.beginProvision(
    NETWORK_PROV_SCHEME_BLE,
    NETWORK_PROV_SCHEME_HANDLER_FREE_BLE,
    NETWORK_PROV_SECURITY_1,
    POP,
    SERVICE_NAME,
    nullptr, nullptr,
    true  // reset any prior provisioning
  );
  WiFiProv.printQR(SERVICE_NAME, POP, "ble");

  Serial.println("[SETUP] Scan the QR code above with the ESP-BLE-Provisioning app…");
  display.clearDisplay();
  display.setTextSize(2);
  oledMessage(POP,2);
}

// ----------------------------------------------------------------------------
//  Called whenever we actually get an IP (either from stored creds or after prov)
// ----------------------------------------------------------------------------
void onWiFiConnected() {

  Serial.print("Reading battery values");
  // set up for battery reading
  analogReadResolution(12);  
  analogSetPinAttenuation(BAT_ADC_PIN, ADC_11db);
  esp_adc_cal_characterize(
        ADC_UNIT_1,
        ADC_ATTEN_DB_11,
        ADC_WIDTH_BIT_12,
        1100,          // fallback Vref (mV) if chip has no eFuse data
        &adc_chars);
  float bat_vb  = readBatteryVoltage();
  float bat_pct = voltageToPercent(bat_vb);



  Serial.print("[WIFI] Connected, IP: ");
  Serial.println(WiFi.localIP());

  display.clearDisplay();
  display.setTextSize(1);
  oledMessage("Start sensot setup",0);

  Wire.begin(21, 22);


  // Initialize the luminosity sensor
  if (lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE)) {
      Serial.println("BH1750 initialized");
  } else {
      Serial.println("Error initializing BH1750");
  }


  // Temperature & Humidity (SHTC3)
  if (! shtc3.begin() ) {
    Serial.println("Error: couldn't find SHTC3 sensor");
    while (1) delay(10);
  }
  Serial.println("SHTC3 initialized");


  oledMessage("Sensor setup concluded",1);
  oledMessage("Start sensor reading",2);
  delay(3000);


  bool dataReady = false;
  temperature = 0.0;
  relativeHumidity = 0.0;


  // Read temperature & humidity once:
  sensors_event_t humidity_event, temp_event;
  shtc3.getEvent(&humidity_event, &temp_event);
  temperature       = temp_event.temperature;      // °C
  relativeHumidity  = humidity_event.relative_humidity;  // %RH


  int raw_value = analogRead(SOIL_SENSOR_PIN);  // Read analog value (0-4095)
  // Convert to percentage (adjust values based on calibration)
  soilmoisture = map(raw_value, 1350, 3050, 100, 0);
  soilmoisture = constrain(soilmoisture, 0, 100);


  Serial.print("Soil Moisture: ");
  Serial.print(soilmoisture);
  Serial.println("%");

  luminosity = lightMeter.readLightLevel(); // Read light intensity in lux
  Serial.print("Light Intensity [lx]: ");
  Serial.print(luminosity);
  Serial.println();

  //
  // Print results in physical units.
  Serial.print("CO2 concentration [ppm]: ");
  Serial.println();
  Serial.print("Temperature [°C]: ");
  Serial.print(temperature);
  Serial.println();
  Serial.print("Relative Humidity [RH]: ");
  Serial.print(relativeHumidity);
  Serial.println();
  Serial.print("Battery voltage: ");
  Serial.print(bat_vb);
  Serial.println();
  Serial.print("Battery percent: ");
  Serial.print(bat_pct);


  Serial.println("*********");


  display.clearDisplay();
  oledMessage(("SM: " + String(soilmoisture)).c_str(), 0);
  oledMessage(("LI: " + String(luminosity)).c_str(), 1);
  oledMessage(("T: " + String(temperature)).c_str(), 2);
  oledMessage(("RH: " + String(relativeHumidity)).c_str(), 3);
  oledMessage(("BAT V: " + String(bat_vb)).c_str(), 4);
  oledMessage(("BAT PCT: " + String(bat_pct)).c_str(), 5);


  delay(3000);
  display.clearDisplay();

  String idStr = WiFi.macAddress();

  // esp_efuse_mac_get_default(factoryMac);  // Gets factory MAC from efuse
  // sprintf(idStr, "%02X:%02X:%02X:%02X:%02X:%02X", factoryMac[0], factoryMac[1], factoryMac[2],factoryMac[3], factoryMac[4], factoryMac[5]);
  oledMessage("Connecting to AWS", 3);
  connectAWS();
  JsonDocument doc;
  // doc["sensortime"] = "TIME";
  doc["temperature"] = temperature;
  doc["relativeHumidity"] = relativeHumidity;
  doc["luminosity"] = luminosity;
  doc["soilmoisture"] = soilmoisture;
  doc["sensoridentification"] = idStr;
  doc["batvoltage"] = bat_vb;
  doc["batpercent"] = bat_pct;
  
  char jsonBuffer[256];
  serializeJson(doc, jsonBuffer); // print to client
  Serial.println(jsonBuffer);
  client.publish(AWS_IOT_PUBLISH_TOPIC, jsonBuffer);

  client.loop();
  oledMessage("Data sent to AWS", 4);
  Serial.println("[APP] Sending data to AWS");
  delay(3000);

  display.clearDisplay();
  oledMessage("Going to sleep",3);
  delay(2000);
  Serial.println("[APP] Going to deep sleep for 30 minutes…");
  display.ssd1306_command(SSD1306_DISPLAYOFF);
  esp_deep_sleep(SLEEP_US);
}

// ----------------------------------------------------------------------------
//  Event handler for provisioning & Wi-Fi events
// ----------------------------------------------------------------------------
void SysProvEvent(arduino_event_t *e) {
  switch (e->event_id) {
    case ARDUINO_EVENT_PROV_START:
      Serial.println("\n[PROV] start");
      break;

    case ARDUINO_EVENT_PROV_CRED_RECV: {
      auto &cred = e->event_info.prov_cred_recv;
      Serial.println("\n[PROV] got credentials:");
      Serial.print("   SSID = "); Serial.println((char*)cred.ssid);
      Serial.print("   PSK  = "); Serial.println((char*)cred.password);
      display.clearDisplay();
      display.setTextSize(1);
      oledMessage("Connecting to WiFi...", 0);
      break;
    }

    case ARDUINO_EVENT_PROV_CRED_FAIL: {
      int reason = e->event_info.prov_fail_reason;
      Serial.printf("\n[PROV] cred fail (code %d)\n", reason);
      if (reason == NETWORK_PROV_WIFI_STA_AUTH_ERROR) {
        Serial.println("   ↳ Auth error (bad password)");
      } else if (reason == NETWORK_PROV_WIFI_STA_AP_NOT_FOUND) {
        Serial.println("   ↳ AP not found (wrong SSID / hidden / 5GHz?)");
      } 
      display.clearDisplay();
      display.setTextSize(1);
      oledMessage("WiFi setup failed!", 0);
      oledMessage("Device will reboot", 1);
      delay(3000);
      WiFi.disconnect(true);
      esp_deep_sleep(500000ULL);
    }

    case ARDUINO_EVENT_PROV_CRED_SUCCESS:
      Serial.println("\n[PROV] success");
      break;

    case ARDUINO_EVENT_PROV_END:
      Serial.println("\n[PROV] end");
      break;

    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED: {
      // <-- open a new scope here
      wifi_event_sta_disconnected_t info = e->event_info.wifi_sta_disconnected;
      Serial.printf("[WIFI] disconnected, reason=%d\n", info.reason);

      if (info.reason == WIFI_REASON_AUTH_EXPIRE ||
          info.reason == WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT ||
          info.reason == WIFI_REASON_HANDSHAKE_TIMEOUT) {
        Serial.println("↳ Auth failure—rebooting");
        display.clearDisplay();
        oledMessage("WiFi setup failed!", 0);
        oledMessage("Device will reboot", 1);
        delay(3000);
        WiFi.disconnect(true);
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ESP_ERROR_CHECK( nvs_flash_init() );  
        esp_deep_sleep(500000ULL);
      }
      break;
    } // <-- close the scope here

    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      // Once we get an IP, run our connected logic
      onWiFiConnected();
      break;

    default:
      break;
  }
}

// ----------------------------------------------------------------------------
//  Handler for messages to oled
// ----------------------------------------------------------------------------
void oledMessage(const char* message, int line) {
  display.setCursor(0, line * 10); // Move cursor to the right line
  display.print(message);
  display.display();
}

float readBatteryVoltage() {
  uint32_t raw = analogRead(BAT_ADC_PIN);
  uint32_t mv  = esp_adc_cal_raw_to_voltage(raw, &adc_chars); // mV at divider
  return (mv / 1000.0) * 11.0;   // undo divider
}

float voltageToPercent(float vb) {
  // crude linear 2.75 V → 0 %, 4.20 V → 100 %
  float pct = (vb - 2.75) / (4.20 - 2.75) * 100.0;
  return constrain(pct, 0, 100);
}

// ----------------------------------------------------------------------------
//  Main entry points
// ----------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(5000);

  Serial.println("********* START CODE **********");

  pinMode(CLEAR_BTN_PIN, INPUT_PULLDOWN);
  Serial.println("Hold clear-creds button (GPIO27) for 3 s...");
  uint32_t t0 = millis();
  while (millis() - t0 < 3000) {
    if (digitalRead(CLEAR_BTN_PIN) == HIGH) {
      Serial.println("⚠️ Clearing Wi-Fi creds and rebooting…");
      forgetCredentialsAndReboot();
    }
    delay(50);
  }



  pinMode(sensorSwitchPin, OUTPUT);
  digitalWrite(sensorSwitchPin, HIGH);
  delay(1000);

  Wire.begin(OLED_SDA, OLED_SCL);

  // Initialize OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
      Serial.println("SSD1306 allocation failed");
      while (1);
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  oledMessage("I TALK TO PLANTS!", 0);

  // 1) Init (and erase if needed) NVS
  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
      err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    err = nvs_flash_init();
  }
  ESP_ERROR_CHECK(err);

  // 2) Clear any automatic Wi-Fi state, then scan
  WiFi.mode(WIFI_MODE_STA);
  WiFi.disconnect(true);
  scanNetworks();

  // 3) Register all event callbacks
  WiFi.onEvent(SysProvEvent, ARDUINO_EVENT_PROV_START);
  WiFi.onEvent(SysProvEvent, ARDUINO_EVENT_PROV_CRED_RECV);
  WiFi.onEvent(SysProvEvent, ARDUINO_EVENT_PROV_CRED_FAIL);
  WiFi.onEvent(SysProvEvent, ARDUINO_EVENT_PROV_CRED_SUCCESS);
  WiFi.onEvent(SysProvEvent, ARDUINO_EVENT_PROV_END);
  WiFi.onEvent(SysProvEvent, ARDUINO_EVENT_WIFI_STA_DISCONNECTED);
  WiFi.onEvent(SysProvEvent, ARDUINO_EVENT_WIFI_STA_GOT_IP);

  // 4) Try stored credentials first; otherwise provision
  if (!attemptWiFiConnect()) {
    startProvisioning();
  }
}

void loop() {

  // check for reset of wifi credentials
  // static uint32_t pressStart = 0;
  // if (digitalRead(BUTTON_PIN_BOOT) == LOW) {
  //   if (pressStart == 0) pressStart = millis();
  //   else if (millis() - pressStart > 2000) {
  //     // long-press detected
  //     nvs_flash_erase();
  //     nvs_flash_init();
  //     esp_restart();
  //   }
  // } else {
  //   pressStart = 0;
  // }
}