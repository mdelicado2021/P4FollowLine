/*
  p4: FollowLine
  De momento código para siguelíneas.
*/

// LED
#include "FastLED.h"
#include <Servo.h>


//------------------- Definición de variables -------------------

// Ultrasonido
#define TRIG_PIN 13  
#define ECHO_PIN 12 

// Sensor infrarrojo
#define PIN_ITR20001_LEFT   A2
#define PIN_ITR20001_MIDDLE A1
#define PIN_ITR20001_RIGHT  A0

// Motores
// Enable/Disable motor control.
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

#define VEL 30

const int SPEED = 9600;

int distance;
int pulse_time;

const int umbral = 850;


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

  // Configuración pines de infrarrojo
  pinMode(PIN_ITR20001_LEFT, INPUT);
  pinMode(PIN_ITR20001_MIDDLE, INPUT);
  pinMode(PIN_ITR20001_RIGHT, INPUT);


}

//------------------- Funciones -------------------
void recto(){
  digitalWrite(PIN_Motor_AIN_1, HIGH);
  analogWrite(PIN_Motor_PWMA, VEL);
  
  digitalWrite(PIN_Motor_BIN_1, HIGH);
  analogWrite(PIN_Motor_PWMB, VEL);
}

void dcha(){
  digitalWrite(PIN_Motor_AIN_1, LOW);
  analogWrite(PIN_Motor_PWMA, VEL);
  
  digitalWrite(PIN_Motor_BIN_1, HIGH);
  analogWrite(PIN_Motor_PWMB, VEL);
}

void izq(){
  digitalWrite(PIN_Motor_AIN_1, HIGH);
  analogWrite(PIN_Motor_PWMA, VEL);
  
  digitalWrite(PIN_Motor_BIN_1, LOW);
  analogWrite(PIN_Motor_PWMB, VEL);
}

void seguidor(){
  // Lectura de valores de los sensores infrarrojos
  int leftSensorValue = analogRead(PIN_ITR20001_LEFT);
  int middleSensorValue = analogRead(PIN_ITR20001_MIDDLE);
  int rightSensorValue = analogRead(PIN_ITR20001_RIGHT);

  if (leftSensorValue < umbral && middleSensorValue > umbral && rightSensorValue < umbral) {
  recto();
  }
  else if (leftSensorValue < umbral && middleSensorValue < umbral && rightSensorValue > umbral) {
  dcha();
  }
  else if (leftSensorValue > umbral && middleSensorValue < umbral && rightSensorValue < umbral) {
  izq();
  }
}


void ultrasonido(){
  long t;
  long d;
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10); // enviar un pulso de 10us
  digitalWrite(TRIG_PIN, LOW);
  t = pulseIn(ECHO_PIN, HIGH);
  d = t/59;
  Serial.println(d);
  delay(100);
}



void loop() {
  
  // ultrasonido();

  // seguidor();

}
