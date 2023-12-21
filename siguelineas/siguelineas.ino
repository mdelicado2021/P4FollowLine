/*
  p4: FollowLine
  Code for line following on Arduino using FreeRTOS.
*/

#include <Arduino_FreeRTOS.h>
#include "FastLED.h"
#include <Servo.h>
#include <SoftwareSerial.h>
#include <string.h>


//------------------- Variable Definitions -------------------


// Pins for the ultrasonic sensor
#define TRIG_PIN 13  
#define ECHO_PIN 12 

// Pins for the infrared sensor
#define PIN_ITR20001_LEFT   A2
#define PIN_ITR20001_MIDDLE A1
#define PIN_ITR20001_RIGHT  A0

// Pins for the motors
#define PIN_Motor_STBY 3
#define PIN_Motor_AIN_1 7
#define PIN_Motor_PWMA 5
#define PIN_Motor_BIN_1 8
#define PIN_Motor_PWMB 6

// Pins for the LED
#define PIN_RBGLED 4
#define NUM_LEDS 1
CRGB leds[NUM_LEDS];

// Base velocity
#define VEL 150

// Definitions for messages to ESP32
#define START_LAP 0
#define END_LAP 1
#define OBSTACLE_DETECTED 2
#define LINE_LOST 3
#define PING 4
#define INIT_LINE_SEARCH 5
#define STOP_LINE_SEARCH 6
#define LINE_FOUND 7
#define VISIBLE_LINE 8
#define SEND_PING_INTERVAL 4000

// PD controller variables
float Kp = 0.55;
float Kd = 0.83;
const int MAX_VEL = 185;
const int MIN_VEL = 170;
float error_left = 0;
float error_right = 0;
float previous_error = 0;
float previous_error_right = 0;
float previous_error_left = 0;
float delta_error_left = 0;
float delta_error_right = 0;
float PD_output = 0;

// Baud rate
const int SPEED = 9600;

// Initialized with a value greater than 8 to avoid false positives
int distance = 100.00;
const int distance_threshold = 8;

// Booleans to control message passing
bool message_sent = false;
bool obs_detected = false;
bool message_8_sent = false;
bool ping_sent = false;

// Threshold for line detection
const int threshold = 500;

// Infrared sensor initialization values
int left_sensor_value;
int middle_sensor_value;
int right_sensor_value;

// Variables for recovery
int last_infrared_sensor = 0;
bool turned_green = true;
bool turned_red = false;

// Variables to calculate the percentage of line
int total_readings = 0;
int line_readings = 0;

// Variables to calculate time
long begin_millis;
long last_millis;


//------------------- Functions -------------------


// Function to go straight
void straight(int left_speed, int right_speed){
  digitalWrite(PIN_Motor_AIN_1, right_speed);
  analogWrite(PIN_Motor_PWMA, VEL);
  
  digitalWrite(PIN_Motor_BIN_1, left_speed);
  analogWrite(PIN_Motor_PWMB, VEL);
}

// Function to turn right
void right(int left_speed, int right_speed){
  digitalWrite(PIN_Motor_AIN_1, LOW);
  analogWrite(PIN_Motor_PWMA, 0);
  
  digitalWrite(PIN_Motor_BIN_1, HIGH);
  analogWrite(PIN_Motor_PWMB, left_speed);
}

// Function to turn left
void left(int left_speed, int right_speed){
  digitalWrite(PIN_Motor_AIN_1, HIGH);
  analogWrite(PIN_Motor_PWMA, right_speed);
  
  digitalWrite(PIN_Motor_BIN_1, LOW);
  analogWrite(PIN_Motor_PWMB, 0);
}

// PD controller function
void calculatePD(int left_sensor_value, int middle_sensor_value, int right_sensor_value, 
                int threshold, int &rightVelocity, int &leftVelocity) {
  // Improved error calculation incorporating the middle sensor
  float error = (right_sensor_value - left_sensor_value) + (middle_sensor_value - threshold); 

  // PD calculation
  float delta_error = error - previous_error;
  float PD_output = Kp * error + Kd * delta_error;
  previous_error = error;

  // Dynamic adjustment of the base velocity
  int baseVel = adjustBaseVelocity(error);

  // Calculate velocities
  rightVelocity = baseVel + PD_output;
  leftVelocity = baseVel - PD_output;

  // Limit velocities to be within allowed ranges
  rightVelocity = constrain(rightVelocity, MIN_VEL, MAX_VEL);
  leftVelocity = constrain(leftVelocity, MIN_VEL, MAX_VEL);
}


// Adjust velocities for PD control
int adjustBaseVelocity(float error) {
    // Define error bounds for velocity adjustment
    const float minError = 10.0;
    const float maxError = 50.0;

    // Calculate velocity adjustment factor based on error
    float velocityAdjustmentFactor = 1.0;
    if (abs(error) < minError) {
        // If error is small, increase velocity
        velocityAdjustmentFactor = 1.0 + (minError - abs(error)) / minError;
    } else if (abs(error) > maxError) {
        // If error is large, decrease velocity
        velocityAdjustmentFactor = 1.0 - (abs(error) - maxError) / maxError;
    }

    // Adjust base velocity within specified limits
    int adjustedVelocity = VEL * velocityAdjustmentFactor;
    return (adjustedVelocity);
}


// Main function for line following
void follower(){
  // Read values from infrared sensors
  left_sensor_value = analogRead(PIN_ITR20001_LEFT);
  middle_sensor_value = analogRead(PIN_ITR20001_MIDDLE);
  right_sensor_value = analogRead(PIN_ITR20001_RIGHT);

  // Calculate motor velocities
  int rightVelocity, leftVelocity;
  calculatePD(left_sensor_value, middle_sensor_value, right_sensor_value, threshold, rightVelocity, leftVelocity);

  // Calculate the total sensor readings
  total_readings++;
  
  // Sending pings
  if (!obs_detected) {
    long current_millis = millis();
    if (!ping_sent && current_millis - last_millis >= SEND_PING_INTERVAL) {
      // Send a ping if one has not been sent recently and the interval has passed
      last_millis = current_millis; // Save the time when the ping was sent
      long timeElapsed = current_millis - begin_millis;
      String timeElapsed_s = String(timeElapsed);
      Serial.print("4");
      Serial.println(timeElapsed_s);
      ping_sent = true;
    } else if (ping_sent && current_millis - last_millis >= SEND_PING_INTERVAL) {
      // Reset the flag to allow the next ping
      ping_sent = false;
    }
  }

  // Line detection calculation with any of the infrared sensors
  if (middle_sensor_value >= threshold || right_sensor_value >= threshold || left_sensor_value >= threshold) {
    line_readings++;
  }

  // Robot movement
  if (distance > distance_threshold){
    if (middle_sensor_value > threshold) {
      straight(leftVelocity, rightVelocity);
    }
    else if (right_sensor_value > threshold) {
      right(leftVelocity, rightVelocity);
      last_infrared_sensor = 3;
    }
    else if (left_sensor_value > threshold) {
      left(leftVelocity, rightVelocity);
      last_infrared_sensor = 2;
    }

    // Recovery
    else if (middle_sensor_value < threshold && right_sensor_value < threshold && left_sensor_value < threshold){
      if (last_infrared_sensor == 2){
        left(leftVelocity, rightVelocity);
      }
      else if (last_infrared_sensor == 3){
        right(leftVelocity, rightVelocity);
      }
    }
  }

  // Obstacle detection
  else {
    digitalWrite(PIN_Motor_AIN_1, LOW);
    analogWrite(PIN_Motor_PWMA, 0);
    
    digitalWrite(PIN_Motor_BIN_1, LOW);
    analogWrite(PIN_Motor_PWMB, 0);
    obs_detected = true;

    // Send the line percentage
    if(!message_8_sent){
      linePercentage();
    }

    // Send the "END_LAP" message only once
    if (obs_detected && !message_sent) {
      String distance_str = (String)distance;

      Serial.print("2");
      Serial.println(distance_str);
      Serial.print("1");

      long current_millis = millis();
      long lap_time = current_millis - begin_millis;

      String lap_time_str = String(lap_time);
      Serial.println(lap_time_str);
      
      // Ensure the message is sent only once
      message_sent = true;
    }
  }
}

// Function to use the ultrasonic sensor
void ultrasonic(){
  long t;
  long d;

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  t = pulseIn(ECHO_PIN, HIGH);
  distance = t/59;
}

// Send the line percentage
void linePercentage(){
  float linePercentage = (float)line_readings / total_readings * 100.00;
  String linePercentage_s = String(linePercentage);
  Serial.print("8");
  Serial.println(linePercentage_s);
  message_8_sent = true;  
}


//------------------- Tasks -------------------


// Task for line follower
void TaskLineFollower(void *pvParameters) {
  (void) pvParameters;
  for (;;) {
    follower();
  }
}


// Task for LED illumination
void TaskLedBlink(void *pvParameters) {
  (void) pvParameters;
  for (;;) {
    // Read the value of the infrared sensor
    int middle_sensor_value = analogRead(PIN_ITR20001_MIDDLE);
    int left_sensor_value = analogRead(PIN_ITR20001_LEFT);
    int right_sensor_value = analogRead(PIN_ITR20001_RIGHT);

    // Determine the LED color
    CRGB ledColor;
    if (distance < distance_threshold) {
      ledColor = CRGB::Blue;
    } 
    else if ((middle_sensor_value >= threshold) || (left_sensor_value >= threshold) || (right_sensor_value >= threshold)) {
      ledColor = CRGB::Green;
        if (turned_red) {
          Serial.print("7");
          Serial.print("6");
          turned_green = true;
          turned_red = false;
        }
    } 
    else if ((middle_sensor_value < threshold) && (left_sensor_value < threshold) && (right_sensor_value < threshold)){
      ledColor = CRGB::Red;
       if (turned_green) {
        Serial.print("3");
        Serial.print("5");
        turned_red = true;
        turned_green = false;
      }
    }
    FastLED.showColor(ledColor);
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}


// Task for the ultrasonic sensor
void TaskUltrasonicSensor(void *pvParameters) {
  (void) pvParameters;
  for (;;) {
    ultrasonic();
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}


//------------------- Setup -------------------


void setup() {
  Serial.begin(9600);

  // Ultrasonic sensor pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);

  // Configure motor pins as output
  pinMode(PIN_Motor_STBY, OUTPUT);
  pinMode(PIN_Motor_AIN_1, OUTPUT);
  pinMode(PIN_Motor_PWMA, OUTPUT);
  pinMode(PIN_Motor_BIN_1, OUTPUT);
  pinMode(PIN_Motor_PWMB, OUTPUT);

  // Enable motors
  digitalWrite(PIN_Motor_STBY, HIGH);

  // LED
  FastLED.addLeds<NEOPIXEL, PIN_RBGLED>(leds, NUM_LEDS);
  FastLED.setBrightness(20);

  // Configure infrared sensor pins
  pinMode(PIN_ITR20001_LEFT, INPUT);
  pinMode(PIN_ITR20001_MIDDLE, INPUT);
  pinMode(PIN_ITR20001_RIGHT, INPUT);

  // Wait for communication
  while(1){
    if (Serial.available()){
      break;
    }
  }

  // Calculate times
  begin_millis = millis();
  last_millis = millis();

  // Create tasks for line following and ultrasonic sensing
  xTaskCreate(TaskLineFollower, "LineFollower", 128, NULL, 1, NULL);
  xTaskCreate(TaskUltrasonicSensor, "Ultrasonic", 128, NULL, 1, NULL);
  xTaskCreate(TaskLedBlink, "LED", 128, NULL, 1, NULL);  
}


//------------------- Loop -------------------


void loop() {
  // Empty loop - tasks are running in their own loop
}
