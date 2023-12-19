/*
  p4: FollowLine
  De momento código para siguelíneas.
*/

#include <Arduino_FreeRTOS.h>
#include "FastLED.h"
#include <Servo.h>
#include <SoftwareSerial.h>
#include <string.h>




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

#define VEL 150

#define COM_ESP2ARD 0

#define START_LAP 0
#define END_LAP 1
#define OBSTACLE_DETECTED 2
#define LINE_LOST 3
#define PING 4
#define INIT_LINE_SEARCH 5
#define STOP_LINE_SEARCH 6
#define LINE_FOUND 7
#define VISIBLE_LINE 8

// Variables del PD
float Kp = 0.55; // Constante proporcional
float Kd = 0.83; // Constante derivativa

const int MAX_VEL = 185;
const int MIN_VEL = 170;

float error_izq = 0; // Error actual
float error_dch = 0; // Error actual
float error_anterior = 0;
float error_anterior_dch = 0; // Error anterior
float error_anterior_izq = 0; // Error anterior
float delta_error_izq = 0; // Cambio en el error
float delta_error_dch = 0; // Cambio en el error

float salidaPD = 0; // Salida del control PD


const int SPEED = 9600;

int distance = 100; // Inicializa con un valor mayor que 8 para evitar falsos positivos
bool mensajeEnviado = false;
bool obs_detected = false;
int pulse_time;

const int umbral = 500;
//const int umbral = 50;

int leftSensorValue;
int middleSensorValue;
int rightSensorValue;

String outputbuff = "";

int ultimo_sensor_infrarrojo = 0;

bool viene_de_ponerse_en_verde = false;
bool viene_de_ponerse_en_rojo = true;

int counter = 0; 

const int umbral_distancia = 10; //cm

int totalLecturasInfrarrojos = 0;
int lecturasLineaDetectada = 0;

bool mensaje8_enviado = false;


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
  // Mejora en el cálculo del error
  float error = (valorSensorDerecho - valorSensorIzquierdo) + (valorSensorCentral - umbral); // Incorpora el sensor central

  // Cálculo del PD
  float delta_error = error - error_anterior;
  float salidaPD = Kp * error + Kd * delta_error;
  error_anterior = error;

  // Actualiza el error anterior
  // error_anterior = error;

  // Ajuste dinámico de la velocidad base (opcional)
  int velBase = ajustarVelocidadBase(error);

  // Calcula las velocidades
  velocidadDerecha = velBase + salidaPD;
  velocidadIzquierda = velBase - salidaPD;

  // Limita las velocidades para que estén dentro de los rangos permitidos
  velocidadDerecha = constrain(velocidadDerecha, MIN_VEL, MAX_VEL);
  velocidadIzquierda = constrain(velocidadIzquierda, MIN_VEL, MAX_VEL);
}



int ajustarVelocidadBase(float error) {
    // Definir los límites del error para ajustar la velocidad
    const float errorMinimo = 10.0; // Define un umbral para el error mínimo
    const float errorMaximo = 50.0; // Define un umbral para el error máximo

    // Calcula el factor de ajuste de velocidad basado en el error
    float factorAjuste = 1.0;
    if (abs(error) < errorMinimo) {
        // Si el error es pequeño, aumenta la velocidad
        factorAjuste = 1.0 + (errorMinimo - abs(error)) / errorMinimo;
    } else if (abs(error) > errorMaximo) {
        // Si el error es grande, reduce la velocidad
        factorAjuste = 1.0 - (abs(error) - errorMaximo) / errorMaximo;
    }

    // Ajusta la velocidad base dentro de los límites establecidos
    int velocidadAjustada = VEL * factorAjuste;
    return (velocidadAjustada);
}




void seguidor(){
  // Lectura de valores de los sensores infrarrojos
  leftSensorValue = analogRead(PIN_ITR20001_LEFT);
  middleSensorValue = analogRead(PIN_ITR20001_MIDDLE);
  rightSensorValue = analogRead(PIN_ITR20001_RIGHT);
  //Serial.println(middleSensorValue);

  // Calcula las velocidades de los motores
  int velocidadDerecha, velocidadIzquierda;
  calcularPD(leftSensorValue, middleSensorValue, rightSensorValue, umbral, velocidadDerecha, velocidadIzquierda);
  // Serial.println(velocidadDerecha);
  // Serial.println(velocidadIzquierda);
  totalLecturasInfrarrojos++;
  
  if (middleSensorValue >= umbral || rightSensorValue >= umbral || leftSensorValue >= umbral) {
    lecturasLineaDetectada++;
  }

  if (distance >= umbral_distancia){
    if (middleSensorValue > umbral) {
      recto(velocidadIzquierda, velocidadDerecha);
    }
    else if (rightSensorValue > umbral) {
      dcha(velocidadIzquierda, velocidadDerecha);
      ultimo_sensor_infrarrojo = 3;
    }
    else if (leftSensorValue > umbral) {
      izq(velocidadIzquierda, velocidadDerecha);
      ultimo_sensor_infrarrojo = 2;
    }
    else if (middleSensorValue < umbral && rightSensorValue < umbral && leftSensorValue < umbral){
      if (ultimo_sensor_infrarrojo == 2){
        izq(velocidadIzquierda, velocidadDerecha);
      }
      else if (ultimo_sensor_infrarrojo == 3){
        dcha(velocidadIzquierda, velocidadDerecha);
      }
    }
    //mensajeEnviado = false;
  }
  else {
    digitalWrite(PIN_Motor_AIN_1, LOW);
    analogWrite(PIN_Motor_PWMA, 0);
    
    digitalWrite(PIN_Motor_BIN_1, LOW);
    analogWrite(PIN_Motor_PWMB, 0);
    obs_detected = true;

    if(!mensaje8_enviado){
      porcentaje_linea();
    }

    // Serial.println("END_LAP");

    // Enviar el mensaje "END_LAP" solo una vez
    if (obs_detected && !mensajeEnviado) {
      // outputbuff
      //Serial.write("\n");
      //Serial.println(distance);
      Serial.print("2");
      Serial.print("1");
      
      // Serial.println((String)distance);
      mensajeEnviado = true; // Asegurar que el mensaje solo se envíe una vez
    }
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
  
  // Serial.print("Distancia: ");
  // Serial.print(distance);      //Enviamos serialmente el valor de la distancia
  // Serial.print("cm");
  // Serial.println();
}


void porcentaje_linea(){
  float porcentajeLinea = (float)lecturasLineaDetectada / totalLecturasInfrarrojos * 100.0;
  //Serial.print("Porcentaje de línea detectada: ");
  //Serial.print(porcentajeLinea);
  //Serial.println("%");
  Serial.print(8);
  mensaje8_enviado = true;  
}




//------------------- Tareas -------------------





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
    int leftSensorValue = analogRead(PIN_ITR20001_LEFT);
    int rightSensorValue = analogRead(PIN_ITR20001_RIGHT);

    // Determinar el color del LED
    CRGB ledColor;
    if (distance < umbral_distancia) {
      ledColor = CRGB::Blue; // Cambiar a azul si hay un obstáculo
    } 
    else if ((middleSensorValue >= umbral) || (leftSensorValue >= umbral) || (rightSensorValue >= umbral)) {
      ledColor = CRGB::Green; // Cambiar a verde si está sobre la línea
        if (viene_de_ponerse_en_rojo) {
          // Mensaje de LINE_FOUND
          Serial.println(7);
          // Mensaje de STOP_LINE_SEARCH
          Serial.println(6);
        }
      viene_de_ponerse_en_verde = true;
      viene_de_ponerse_en_rojo = false;
    } 
    else {
      ledColor = CRGB::Red; // Cambiar a rojo si está fuera de la línea
       if (viene_de_ponerse_en_verde) {
        // Mensaje de LINE_LOST
        Serial.println(3);
        // Mensaje de INIT_LINE_SEARCH
        Serial.println(5);
      }
      viene_de_ponerse_en_rojo = true;
      viene_de_ponerse_en_verde = false;
    }

    // Cambiar el color del LED
    FastLED.showColor(ledColor);

    // Retraso mínimo para permitir el procesamiento de los sensores y la actualización del LED
    vTaskDelay(10 / portTICK_PERIOD_MS);
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

  /*
  while(1){
    if (Serial.available()){
      // Serial.println(Serial.read());
      break;
    }
  }
  */

  // Mensaje de INIT_LAP
  Serial.println(0);

  // Create tasks for line following and ultrasonic sensing
  xTaskCreate(TaskLineFollower, "LineFollower", 128, NULL, 1, NULL);
  xTaskCreate(TaskUltrasonicSensor, "Ultrasonic", 128, NULL, 1, NULL);
  xTaskCreate(TaskLedBlink, "LED", 128, NULL, 1, NULL);
  
}

void loop() {
  // Empty loop - tasks are running in their own loop
}
