#include <WiFi.h>
#include <WiFiClient.h>

// WiFi parameters
const char* ssid = "SiteTech";        // WiFi SSID
const char* password = "INSERT WIFI PASSWORD";// WiFi password
const char* host_ip = "INSERT HOST IP";// Server IP adress
const int port = 65442;               // Server port

// LED for showing connection status
const int led_pin = 2;

// Buffer to receive the bytes sent by the server
const int buffer_size = 10;
uint8_t buffer[sizeof(float) * buffer_size];
uint8_t byteArray[sizeof(float) * buffer_size];

// Motor forces
float motor_forces[buffer_size];

// Timing
const int sampling_interval = 1000;  // Sampling interval in ms
unsigned long current_time = 0;
unsigned long last_time = 0;

void setup() {
  // Serial for debugging
  Serial.begin(460800);
  // Serial for data transfer to motor controller
  Serial2.begin(460800, SERIAL_8N1, 16, 17);    

  // Connect to Wi-Fi network with SSID and password
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  pinMode(led_pin, OUTPUT);    // LED to indicate if wifi is connected
  digitalWrite(led_pin, LOW);  // Turn off LED initially
}

int connection_attempts = 0;
const int max_attempts = 3;

void loop() {
  // Connect to server
  Serial.println("Establishing connection to server");
  WiFiClient client;
  while (!client.connect(host_ip, port)) {
    connection_attempts++;  // Increase connection attempts
    if (connection_attempts >= max_attempts){
      connection_attempts = 0;
      Serial.println("Connection timed out " + String(max_attempts) + " times... Restarting!");
      ESP.restart();
    } else {
      delay(1000);
      Serial.println("Connection timed out... Retrying!");
    }
  }
  digitalWrite(led_pin, HIGH);  // Turn on LED when connected
  // Continue listening until connetion is lost
  while (client.connected()) {
    current_time = millis();
    if (current_time >= last_time + sampling_interval){
      Serial.print("Sampling frequency: ");
      Serial.print(1000/(float(current_time)-float(last_time)), 5);
      Serial.println(" Hz");
      last_time = current_time;
      // Read bytes sent from server and store in buffer
      size_t len = client.readBytes(buffer, sizeof(buffer));
      //client.flush();

      // Convert the bytes to a list of float values
      for (int i = 0; i < buffer_size; i++) {
        motor_forces[i] = *((float*) (buffer + i * sizeof(float)));
      }
      
      Serial.println("Received data: ");
      for (int j = 0; j < buffer_size/10; j++){
        for (int i = 0; i < 5; i++){
          Serial.print(motor_forces[i+j*5], 2);
          Serial.print("\t");
          Serial.print(motor_forces[i+int(buffer_size/2)+j*5], 6);
          Serial.print("\t");
        }
        Serial.println();
      }
      
      
      

      // Prepare byte array for serial transmit
      //memcpy(byteArray, motor_forces, sizeof(byteArray));
      //Serial2.write(byteArray, sizeof(byteArray));
      Serial2.write(buffer, sizeof(buffer));
      Serial2.flush();
      //Serial.println("Sent data");
    }
  }
  Serial.println("Lost connection to server");
  client.stop();
  digitalWrite(led_pin, LOW);  // Turn off LED when connection is lost
}
