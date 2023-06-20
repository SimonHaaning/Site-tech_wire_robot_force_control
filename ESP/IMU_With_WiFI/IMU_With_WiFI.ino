
#define WIRE_NR 9  // Wire number (change when uploading code to ESP connected to second IMU)

#include <WiFi.h>
#include <WiFiClient.h>
const char* ssid = "SiteTech";          // WiFi SSID
const char* password = "INSERT WIFI PASSWORD";  // WiFi password
const char* host_ip = "INSERT HOST IP";  // Server IP adress
#if WIRE_NR == 1
const int port = 65439;  // Server port for wire 1
#elif WIRE_NR == 3
const int port = 65440;  // Server port for wire 3
#else
const int port = 65441;  // Server port for angle measure
#endif
const int timeout_length = 100;  // Timeout length for server response (in milliseconds)
unsigned long timeout;           // Timeout variable for server response
int connection_attempts = 0;
const int max_attempts = 3;
const int led_pin = 2;

const int sampling_interval = 100;  // Sampling interval in milliseconds
unsigned long last_sample_time = 0;
unsigned long current_time = 0;

// Parameters for gravity to direction conversion
float staticGravity[3] = { 0, 1, 0 };       // Actual gravity in static frame
float localWireDirection[3] = { 1, 0, 0 };  // Wire direction in the IMU frame
float measuredGravity[3] = { 0, 0, 0 };     // IMU gravity vector output
float actualWireDirection[3] = { 0 };       // Calculated wire direction
void transformWireDirection2D(float measured_gravity[3], float actual_gravity[3], float* wire_direction_static_frame);

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

MPU6050 mpu;

#define OUTPUT_READABLE_EULER

// MPU control/status vars
bool dmpReady = false;   // set true if DMP init was successful
uint8_t mpuIntStatus;    // holds actual interrupt status byte from MPU
uint8_t devStatus;       // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;     // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;      // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];  // FIFO storage buffer

// orientation/motion vars
Quaternion q;         // [w, x, y, z]         quaternion container
VectorInt16 aa;       // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;   // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;  // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;  // [x, y, z]            gravity vector
//float euler[3];         // [psi, theta, phi]    Euler angle container
//float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float prevTime;
float previousEuler[3];
float resultMatrix[3][3];
float euler[3] = { 0, 0, 0 };    // [psi, theta, phi]    Euler angle container
unsigned long previousTime = 0;  // time of the last action
unsigned long interval = 100;    // time between actions in milliseconds

//setup of the code
void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000);  // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
  // initialize serial communication
  Serial.begin(115200);  // (115200 chosen because it is required for Teapot Demo output, but it's really up to you depending on your project)
  delay(10);

  while (!Serial)
    ;  // wait for Leonardo enumeration, others continue immediately

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  if (WIRE_NR == 1) {
    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(87);
    mpu.setYGyroOffset(-27);
    mpu.setZGyroOffset(66);
    mpu.setZAccelOffset(1095);
  } else if(WIRE_NR == 3) {
    //IMU2
    mpu.setXGyroOffset(-15);
    mpu.setYGyroOffset(-35);
    mpu.setZGyroOffset(-3);
    mpu.setZAccelOffset(1288);
  } else {
    mpu.setXGyroOffset(-62);
    mpu.setYGyroOffset(101);
    mpu.setZGyroOffset(5);
    mpu.setZAccelOffset(956);
  }

  Serial.println("MPU6050 initialized!");
  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    /*
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    */
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
  // ================================================================
  // ===                      Setup WiFI                          ===
  // ================================================================
  pinMode(led_pin, OUTPUT);    // LED to indicate if wifi is connected
  digitalWrite(led_pin, LOW);  // Turn off LED initially
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
  // ================================================================
  // ===                     Setup WiFI                           ===
  // ================================================================
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
void loop() {
  // Connect to server
  Serial.println("Establishing connection to server");
  WiFiClient client;
  while (!client.connect(host_ip, port)) {
    connection_attempts++;  // Increase connection attempts
    if (connection_attempts >= max_attempts) {
      connection_attempts = 0;
      Serial.println("Connection timed out " + String(max_attempts) + " times... Restarting!");
      ESP.restart();
    } else {
      delay(1000);
      Serial.println("Connection timed out... Retrying!");
    }
  }
  digitalWrite(led_pin, HIGH);  // Turn on LED when connected
  while (client.connected()) {
    current_time = millis();
    if (current_time >= last_sample_time + sampling_interval) {
      Serial.print("Sampling frequency: ");
      Serial.print(1000 / (float(current_time) - float(last_sample_time)), 5);
      Serial.println(" Hz");
      last_sample_time = current_time;
      // if programming failed, don't try to do anything
      if (!dmpReady) return;
      if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {  // Get the Latest packet
        // Update gravity vector with newest measurement
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);

        // Calculate the direction of the wire
        float measuredGravity[3] = { gravity.x, gravity.y, gravity.z };
        //calculateActualWireDirection(measuredGravity, staticGravity, localWireDirection, actualWireDirection);
        transformWireDirection2D(measuredGravity, staticGravity, actualWireDirection);

        // Print the unit vector to the Serial Monitor
        Serial.print("Grav vector: x= ");
        Serial.print(measuredGravity[0], 10);
        Serial.print(", y= ");
        Serial.print(measuredGravity[1], 10);
        Serial.print(", z= ");
        Serial.println(measuredGravity[2], 10);
        Serial.print("Unit vector: x= ");
        Serial.print(actualWireDirection[0], 10);
        Serial.print(", y= ");
        Serial.print(actualWireDirection[1], 10);
        Serial.print(", z= ");
        Serial.println(actualWireDirection[2], 10);

        // Send data with TCP
        String data_str;
        data_str = String(actualWireDirection[0], 10) + "," + String(actualWireDirection[1], 10) + "," + String(actualWireDirection[2], 10) + ",";
        // Pad the data and send it
        while (data_str.length() < 64) {
          data_str.concat(",");
        }
        client.print(data_str);  // Send the three float values to the server
        client.flush();
      }
    }
  }
  Serial.println("Lost connection to server");
  client.stop();
  digitalWrite(led_pin, LOW);  // Turn off LED when connection is lost
}

// Function to calculate wire direction for gravity (Only calculates as planar case so Z component is missing)
void transformWireDirection2D(float measured_gravity[3], float actual_gravity[3], float* wire_direction_static_frame) {
  float wire_direction_dynamic_frame[2] = { 1.0, 0.0 };

  float measured_gravity_2D[3] = { measured_gravity[1], measured_gravity[0], 0 };

  // Normalize the input vectors
  float measured_gravity_norm = sqrt(pow(measured_gravity_2D[0], 2) + pow(measured_gravity_2D[1], 2));
  float actual_gravity_norm = sqrt(pow(actual_gravity[0], 2) + pow(actual_gravity[1], 2));

  measured_gravity_2D[0] /= measured_gravity_norm;
  measured_gravity_2D[1] /= measured_gravity_norm;
  actual_gravity[0] /= actual_gravity_norm;
  actual_gravity[1] /= actual_gravity_norm;

  // Calculate the cross product
  float c[3] = {
    measured_gravity_2D[1] * actual_gravity[2] - actual_gravity[1] * measured_gravity_2D[2],
    actual_gravity[0] * measured_gravity_2D[2] - measured_gravity_2D[0] * actual_gravity[2],
    measured_gravity_2D[0] * actual_gravity[1] - actual_gravity[0] * measured_gravity_2D[1]
  };

  // Calculate the dot product
  float d = measured_gravity_2D[0] * actual_gravity[0] + measured_gravity_2D[1] * actual_gravity[1];

  // Check if the vectors are parallel
  float c_squared = pow(c[0], 2) + pow(c[1], 2) + pow(c[2], 2);
  if (c_squared == 0) {
    Serial.println("Vectors are parallel. Cannot determine transformation.");
    return;
  }

  // Calculate the transformation angle
  float theta = atan2(c[2], d);

  // Calculate the transformation matrix
  float T[2][2] = {
    { cos(theta), -sin(theta) },
    { sin(theta), cos(theta) }
  };

  // Perform matrix multiplication
  wire_direction_static_frame[0] = T[1][0] * wire_direction_dynamic_frame[0] + T[1][1] * wire_direction_dynamic_frame[1];
  wire_direction_static_frame[1] = T[0][0] * wire_direction_dynamic_frame[0] + T[0][1] * wire_direction_dynamic_frame[1];
  wire_direction_static_frame[2] = 0;
}