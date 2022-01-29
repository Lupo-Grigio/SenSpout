#include <Arduino.h>
#include <WiFi.h>
// For more info on how to set up the Firebase stuff, see: https://github.com/mobizt/Firebase-ESP-Client
#include <Firebase_ESP_Client.h>
#include <addons/TokenHelper.h>
#include <addons/RTDBHelper.h>

#include "config.h"

// TODO: Should probably be in its own file
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>

#define FIREBASE_NODE "/readouts/" + FURBALL_ID

// Define Firebase Data object
FirebaseData fbdo;

FirebaseAuth auth;

FirebaseConfig config;

unsigned long sendDataPrevMillis = 0;

unsigned long count = 0;

// TODO: Should probably be in its own file similar to Furball
Adafruit_BME680 bme;



void setup() {

// Set up Firebase, WiFi, etc...
  Serial.begin(115200);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.print("Connecting to WiFi.");
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.println("...");
  }

  Serial.print("Connected to: ");
  Serial.println(WiFi.localIP());
  Serial.println();

  Serial.printf("Firebase Client v%s\n\n", FIREBASE_CLIENT_VERSION);

  /* Assign the api key (required) */
  config.api_key = API_KEY;

  /* Assign the user sign in credentials */
  auth.user.email = USER_EMAIL;
  auth.user.password = USER_PASSWORD;

  /* Assign the RTDB URL (required) */
  config.database_url = DATABASE_URL;

  /* Assign the callback function for the long running token generation task */
  config.token_status_callback = tokenStatusCallback; //see addons/TokenHelper.h

  Firebase.begin(&config, &auth);

  //Comment or pass false value when WiFi reconnection will control by your code or third party library
  Firebase.reconnectWiFi(true);

  Firebase.setDoubleDigits(5);

  /** Timeout options.
  //WiFi reconnect timeout (interval) in ms (10 sec - 5 min) when WiFi disconnected.
  config.timeout.wifiReconnect = 10 * 1000;
  //Socket connection and SSL handshake timeout in ms (1 sec - 1 min).
  config.timeout.socketConnection = 10 * 1000;
  //Server response read timeout in ms (1 sec - 1 min).
  config.timeout.serverResponse = 10 * 1000;
  //RTDB Stream keep-alive timeout in ms (20 sec - 2 min) when no server's keep-alive event data received.
  config.timeout.rtdbKeepAlive = 45 * 1000;
  //RTDB Stream reconnect timeout (interval) in ms (1 sec - 1 min) when RTDB Stream closed and want to resume.
  config.timeout.rtdbStreamReconnect = 1 * 1000;
  //RTDB Stream error notification timeout (interval) in ms (3 sec - 30 sec). It determines how often the readStream
  //will return false (error) when it called repeatedly in loop.
  config.timeout.rtdbStreamError = 3 * 1000;
  Note:
  The function that starting the new TCP session i.e. first time server connection or previous session was closed, the function won't exit until the 
  time of config.timeout.socketConnection.
  You can also set the TCP data sending retry with
  config.tcp_data_sending_retry = 1;
  */

 // Set up bme680
 //! some boards may use bme280 instead
 // TODO: put this in its own file, in a void begin()
  if (!bme.begin()) {
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
    return;
  }

  bme.setTemperatureOversampling(BME680_OS_8X);


}

// the loop function runs over and over again forever
void loop(){

  //Firebase.ready works for authentication management and should be called repeatedly in the loop.

  if (Firebase.ready() && (millis() - sendDataPrevMillis > 5000 || sendDataPrevMillis == 0))
  {
    sendDataPrevMillis = millis();

  //! BME stuff should probably go in bme files as handlers like in Furball
  FirebaseJson bmeReadings;
  float temp = bme.readTemperature();

  bmeReadings.set("bme/temperature", temp);
  bmeReadings.set("timestamp/.sv", "timestamp");

  Serial.printf("Push data with timestamp... %s\n", Firebase.RTDB.pushJSON(&fbdo, READOUTS_NODE, &bmeReadings) ? "ok" : fbdo.errorReason().c_str());
  // bmeReadings.set("timestamp/.sv", "timestamp");
  // bmeReadings.add("temperature/" + String(count), temp);
  // Firebase.RTDB.updateNode(&fbdo, F("/bme"), &bmeReadings);
  // Serial.print(F("Temperature = "));
  // Serial.print(temp);
  // Serial.print(F(" *C and "));
  // Serial.print(temp * 9/5 + 32);
  // Serial.println(F(" *F"));

    count++;
  }
}
