/*
  p4: FollowLine
  De momento código para siguelíneas.
*/

#include <Arduino_FreeRTOS.h>
#include "FastLED.h"
#include <Servo.h>
#include <SoftwareSerial.h>


//------------------- Definición de variables -------------------

// Ultrasonido
#define TRIG_PIN 13  
#define ECHO_PIN 12 

// Sensor infrarrojo
#define PIN_ITR20001_LEFT   A2
#define PIN_ITR20001_MIDDLE A1
#define PIN_ITR20001_RIGHT  A0

// Motores
// Enable/Disable motor control
//  HIGH: motor control enabled
//  LOW: motor control disabled
#define PIN_Motor_STBY 3

// Group A Motors (Right Side)
// PIN_Motor_AIN_1: Digital output. HIGH: Forward, LOW: Backward
#define PIN_Motor_AIN_1 7
// PIN_Motor_PWMA: Analog output [0-255]. It provides speed.
#define PIN_Motor_PWMA 5

// Group B Motors (Left Side)
// PIN_Motor_BIN_1: Digital output. HIGH: Forward, LOW: Backward
#define PIN_Motor_BIN_1 8
// PIN_Motor_PWMB: Analog output [0-255]. It provides speed.
#define PIN_Motor_PWMB 6


#define PIN_RBGLED 4
#define NUM_LEDS 1
CRGB leds[NUM_LEDS];

#define VEL 100

#define COM_ESP2ARD 0

// Variables del PD
float Kp = 1.05; // Constante proporcional
float Kd = 0.85; // Constante derivativa

float error = 0; // Error actual
float error_anterior = 0; // Error anterior
float delta_error = 0; // Cambio en el error

float salidaPD = 0; // Salida del control PD


const int SPEED = 9600;

int distance;
int pulse_time;

//const int umbral = 650;
const int umbral = 50;

int leftSensorValue;
int middleSensorValue;
int rightSensorValue;



//------------------- Funciones -------------------
void recto(int velizq, int veldcha){
  digitalWrite(PIN_Motor_AIN_1, veldcha);
  analogWrite(PIN_Motor_PWMA, VEL);
  
  digitalWrite(PIN_Motor_BIN_1, velizq);
  analogWrite(PIN_Motor_PWMB, VEL);
}

void dcha(int velizq, int veldcha){
  digitalWrite(PIN_Motor_AIN_1, LOW);
  analogWrite(PIN_Motor_PWMA, 0);
  
  digitalWrite(PIN_Motor_BIN_1, HIGH);
  analogWrite(PIN_Motor_PWMB, velizq);
}

void izq(int velizq, int veldcha){
  digitalWrite(PIN_Motor_AIN_1, HIGH);
  analogWrite(PIN_Motor_PWMA, veldcha);
  
  digitalWrite(PIN_Motor_BIN_1, LOW);
  analogWrite(PIN_Motor_PWMB, 0);
}


void calcularPD(int valorSensorIzquierdo, int valorSensorCentral, int valorSensorDerecho, int umbral, int &velocidadDerecha, int &velocidadIzquierda) {
  // Calcula el error basado en los sensores y el umbral
  // Nota: Este es un ejemplo simple, puedes ajustar la lógica según tus necesidades
  int error = valorSensorCentral - umbral;

  // Cálculo del PD
  int delta_error = error - error_anterior;
  float salidaPD = Kp * error + Kd * delta_error;

  // Actualiza el error anterior
  error_anterior = error;

  // Calcula las velocidades
  velocidadDerecha = VEL + salidaPD;
  velocidadIzquierda = VEL - salidaPD;

  // Limita las velocidades para que estén dentro de los rangos permitidos
  velocidadDerecha = constrain(velocidadDerecha, 0, 255);
  velocidadIzquierda = constrain(velocidadIzquierda, 0, 255);
}


void seguidor(){
  // Lectura de valores de los sensores infrarrojos
  leftSensorValue = analogRead(PIN_ITR20001_LEFT);
  middleSensorValue = analogRead(PIN_ITR20001_MIDDLE);
  rightSensorValue = analogRead(PIN_ITR20001_RIGHT);

  // Calcula las velocidades de los motores
  int velocidadDerecha, velocidadIzquierda;
  calcularPD(leftSensorValue, middleSensorValue, rightSensorValue, umbral, velocidadDerecha, velocidadIzquierda);

  if (distance >= 8){
    if (middleSensorValue > umbral) {
      recto(velocidadIzquierda, velocidadDerecha);
    }
    else if (rightSensorValue > umbral) {
      dcha(velocidadIzquierda, velocidadDerecha);
    }
    else if (leftSensorValue > umbral) {
      izq(velocidadIzquierda, velocidadDerecha);
    }
  } 
  else {
    digitalWrite(PIN_Motor_AIN_1, LOW);
    analogWrite(PIN_Motor_PWMA, 0);
    
    digitalWrite(PIN_Motor_BIN_1, LOW);
    analogWrite(PIN_Motor_PWMB, 0);
    Serial.write(2);
  }
}


void ultrasonido(){
  long t; //timepo que demora en llegar el eco
  long d; //distancia en centimetros

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);          //Enviamos un pulso de 10us
  digitalWrite(TRIG_PIN, LOW);
  
  t = pulseIn(ECHO_PIN, HIGH); //obtenemos el ancho del pulso
  distance = t/59;             //escalamos el tiempo a una distancia en cm
  
  Serial.print("Distancia: ");
  Serial.print(distance);      //Enviamos serialmente el valor de la distancia
  Serial.print("cm");
  Serial.println();
}

// Task for line following
void TaskLineFollower(void *pvParameters) {
  (void) pvParameters;

  for (;;) {
    seguidor();
  }
}

// Task for line following
void TaskLedBlink(void *pvParameters) {
  (void) pvParameters;

  for (;;) {
      // Leer el valor del sensor infrarrojo
      int middleSensorValue = analogRead(PIN_ITR20001_MIDDLE);

      if (middleSensorValue > umbral) {
        // Dentro de la línea (verde)
        FastLED.showColor(CRGB::Green);
      } else {
        // Fuera de la línea (rojo)
        FastLED.showColor(CRGB::Red);
      }

      FastLED.show();
  }
}

// Task for ultrasonic sensor reading
void TaskUltrasonicSensor(void *pvParameters) {
  (void) pvParameters;

  for (;;) {
    ultrasonido();
    vTaskDelay(100 / portTICK_PERIOD_MS); // Adjust the delay as necessary
  }
}




//------------------- Setup -------------------

void setup() {
  Serial.begin(9600);

  pinMode(TRIG_PIN, OUTPUT); //pin como salida
  pinMode(ECHO_PIN, INPUT);  //pin como entrada
  digitalWrite(TRIG_PIN, LOW); //Inicializamos el pin con 0

  // Configurar pines de motores como salida
  pinMode(PIN_Motor_STBY, OUTPUT);
  pinMode(PIN_Motor_AIN_1, OUTPUT);
  pinMode(PIN_Motor_PWMA, OUTPUT);
  pinMode(PIN_Motor_BIN_1, OUTPUT);
  pinMode(PIN_Motor_PWMB, OUTPUT);

  digitalWrite(PIN_Motor_STBY, HIGH);

  // LED
  FastLED.addLeds<NEOPIXEL, PIN_RBGLED>(leds, NUM_LEDS);
  FastLED.setBrightness(20);

  // Configuración pines de infrarrojo
  pinMode(PIN_ITR20001_LEFT, INPUT);
  pinMode(PIN_ITR20001_MIDDLE, INPUT);
  pinMode(PIN_ITR20001_RIGHT, INPUT);

  while(1){
    if (Serial.available()){
      Serial.println(Serial.read());
      break;
    }
  }

  // Create tasks for line following and ultrasonic sensing
  xTaskCreate(TaskLineFollower, "LineFollower", 128, NULL, 1, NULL);
  xTaskCreate(TaskUltrasonicSensor, "Ultrasonic", 128, NULL, 1, NULL);
  xTaskCreate(TaskLedBlink, "LED", 128, NULL, 1, NULL);
  
}

void loop() {
  // Empty loop - tasks are running in their own loop
}
