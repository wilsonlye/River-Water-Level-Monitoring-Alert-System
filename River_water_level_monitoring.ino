#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
// Wi-Fi Credentials
const char* ssid = "HUAWEI nova 2i";  // Your Wi-Fi SSID
const char* password = "wilsonlye123"; // Your Wi-Fi password

// MQTT Server Details
const char* mqtt_server = "192.168.61.77";  // Your MQTT server IP address
const char* mqtt_topic = "distance/mqtt";    // MQTT Topic to publish distance data

// Initialize the LCD with I2C address and screen size
LiquidCrystal_I2C lcd(0x27, 16, 2);  // I2C address 0x27 and 16x2 screen

// Define pins for the Ultrasonic Sensor
const int trigPin = D6;  // GPIO14 (D6 on NodeMCU)
const int echoPin = D5;  // GPIO12 (D5 on NodeMCU)

// Define pins for the Buzzer and LEDs
const int buzzerPin = D3;   // GPIO0 (D3 on NodeMCU)
const int blueLEDPin = D4;  // GPIO2 (D4 on NodeMCU)
const int yellowLEDPin = D7;  // GPIO13 (D7 on NodeMCU)

WiFiClient espClient;
PubSubClient client(espClient);

// Setup Wi-Fi
void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(100);
    Serial.print(".");
  }

  Serial.println("");
  Serial.print("WiFi connected - IP address: ");
  Serial.println(WiFi.localIP());
}

// MQTT Setup
void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");

    if (client.connect("ESP8266Client")) {
      Serial.println("connected");
      client.subscribe(mqtt_topic);  // Subscribe to the topic if needed
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" Try again in 5 seconds");
      delay(5000);
    }
  }
}

// Setup function
void setup() {
  Serial.begin(115200);

  // Set pin modes for the ultrasonic sensor and LEDs
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(buzzerPin, OUTPUT);
  pinMode(blueLEDPin, OUTPUT);
  pinMode(yellowLEDPin, OUTPUT);

  // Initialize the LCD
  lcd.init();
  lcd.backlight();

  // Connect to Wi-Fi and MQTT
  setup_wifi();
  client.setServer(mqtt_server, 1883);
}

// MQTT callback function (not used in this example)
void callback(char* topic, byte* payload, unsigned int length) {
  // Implement functionality based on received MQTT messages if needed.
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }

  client.loop();  // Handle MQTT communication

  // Send pulse to ultrasonic sensor
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Measure the duration of the pulse
  long duration = pulseIn(echoPin, HIGH);

  // Calculate the distance in centimeters
  long distance = (duration / 2) * 0.0343;  // Speed of sound = 0.0343 cm/µs

  // Display the distance on the LCD
  lcd.setCursor(0, 0);
  lcd.print("Distance: ");
  lcd.setCursor(0, 1);
  lcd.print(distance);
  lcd.print(" cm   ");  // Extra spaces to clear previous data

  // Print the distance to Serial Monitor
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  // Publish the distance to MQTT
  String distanceStr = String(distance);
  client.publish(mqtt_topic, distanceStr.c_str());  // Publish the distance value to the MQTT topic

  // Check if the distance is less than 5 cm
  if (distance < 5) {
    // Activate the buzzer and blue LED if the distance is less than 5 cm
    digitalWrite(buzzerPin, HIGH);
    digitalWrite(blueLEDPin, HIGH);
    digitalWrite(yellowLEDPin, LOW);  // Turn off yellow LED
  } else {
    // Deactivate the buzzer and activate the yellow LED if distance is 5 cm or more
    digitalWrite(buzzerPin, LOW);
    digitalWrite(blueLEDPin, LOW);    // Turn off blue LED
    digitalWrite(yellowLEDPin, HIGH); // Turn on yellow LED
  }
  delay(500);  // Delay for 0.5 second before the next reading
}
