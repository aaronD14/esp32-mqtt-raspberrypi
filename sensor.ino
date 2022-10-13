//#include <WiFi.h>
//#include "PubSubClient.h" // Connect and publish to the MQTT broker

#define TRIG_PIN 0 // ESP32 pin GIOP23 connected to Ultrasonic Sensor's TRIG pin
#define ECHO_PIN 1 // ESP32 pin GIOP22 connected to Ultrasonic Sensor's ECHO pin


// SENSOR
float duration_us, distance_cm;

// WIFI
const char* ssid = "";
const char* password = "pass";


// MQTT
const char* mqtt_server = "192.168.0.8";  // IP of the MQTT broker
const char* sensor_topic = "home/sensor";
const char* mqtt_username = "cdavid"; // MQTT username
const char* mqtt_password = "cdavid"; // MQTT password
const char* clientID = "client"; // MQTT client ID


// Initialise the WiFi and MQTT Client objects

WiFiClient wifiClient;

// 1883 is the listener port for the Broker

PubSubClient client(mqtt_server, 1883, wifiClient); 




void connect_MQTT(){
  Serial.print("Connecting to ");
  Serial.println(ssid);

  // Connect to the WiFi
  WiFi.begin(ssid, wifi_password);

  // Wait until the connection has been confirmed before continuing
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  // Connect to MQTT Broker
  // client.connect returns a boolean value to let us know if the connection was successful.
  // If the connection is failing, make sure you are using the correct MQTT Username and Password (Setup Earlier in the Instructable)
  if (client.connect(clientID, mqtt_username, mqtt_password)) {
    Serial.println("Connected to MQTT Broker!");
  }
  else {
    Serial.println("Connection to MQTT Broker failed...");
  }
}



void setup() {
  // begin serial port
  Serial.begin (9600);
  // configure the trigger pin to output mode
  pinMode(TRIG_PIN, OUTPUT);
  // configure the echo pin to input mode
  pinMode(ECHO_PIN, INPUT);
  
}

void loop() {
  //connect_MQTT();
  // generate 10-microsecond pulse to TRIG pin
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // measure duration of pulse from ECHO pin
  duration_us = pulseIn(ECHO_PIN, HIGH);

  // calculate the distance
  distance_cm = 0.017 * duration_us;


 // PUBLISH to the MQTT Broker (topic = Temperature, defined at the beginning)
 
  if (client.publish(sensor_topic, String(distance_cm).c_str())) {
    Serial.println("distance sent!");
  }
  // Again, client.publish will return a boolean value depending on whether it succeded or not.
  // If the message failed to send, we will try again, as the connection may have broken.
  else {
    Serial.println("distance failed to send. Reconnecting to MQTT Broker and trying again");
    client.connect(clientID, mqtt_username, mqtt_password);
    delay(10); // This delay ensures that client.publish doesn't clash with the client.connect call
    client.publish(sensor_topic, String(t).c_str());
  }



  // print the value to Serial Monitor
  Serial.print("distance: ");
  Serial.print(distance_cm);
  Serial.println(" cm");
  
  //client.disconnect();
  delay(500);
}
