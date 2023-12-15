/*
  p4: FollowLine
  De momento código para siguelíneas.
*/


#include "FastLED.h"
#include <Servo.h>
#include <Arduino_FreeRTOS.h>
#include <FreeRTOSConfig.h>
#include <task.h>
#include <semphr.h>


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

#define VEL 100

#define PIN_RBGLED 4
#define NUM_LEDS 1
CRGB leds[NUM_LEDS];


const int SPEED = 9600;

int distance;
int pulse_time;

const int umbral = 650;
int leftSensorValue;
int middleSensorValue;
int rightSensorValue;

// Declaración de semáforos para sincronización
SemaphoreHandle_t semaforoInfrarrojo;
SemaphoreHandle_t semaforoUltrasonido;

// Prototipos de las funciones de las tareas
void tareaLED(void *parameter);
void tareaInfrarrojo(void *parameter);
void tareaUltrasonido(void *parameter);


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

  // LED
  FastLED.addLeds<NEOPIXEL, PIN_RBGLED>(leds, NUM_LEDS);
  FastLED.setBrightness(20);

  // Crear los semáforos
  semaforoInfrarrojo = xSemaphoreCreateBinary();
  semaforoUltrasonido = xSemaphoreCreateBinary();

  // Crear tareas
  xTaskCreate(tareaLED, "LEDTask", 128, NULL, 1, NULL);
  xTaskCreate(tareaInfrarrojo, "InfrarrojoTask", 128, NULL, 2, NULL);
  xTaskCreate(tareaUltrasonido, "UltrasonidoTask", 128, NULL, 3, NULL);

  vTaskStartScheduler(); // Iniciar el planificador de tareas de FreeRTOS


}

//------------------- Funciones -------------------
void tareaInfrarrojo(void *parameter) {
  while (1) {
    // Leer los valores de los sensores infrarrojos
    leftSensorValue = analogRead(PIN_ITR20001_LEFT);
    middleSensorValue = analogRead(PIN_ITR20001_MIDDLE);
    rightSensorValue = analogRead(PIN_ITR20001_RIGHT);

    // Señalar a la tarea del LED
    xSemaphoreGive(semaforoInfrarrojo);

    vTaskDelay(50 / portTICK_PERIOD_MS); // Ajusta el tiempo según sea necesario
    Serial.println("Infrarrojo");
  }
}


void tareaLED(void *parameter) {
  while (1) {
    if (xSemaphoreTake(semaforoInfrarrojo, portMAX_DELAY) == pdTRUE) {
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

    vTaskDelay(50 / portTICK_PERIOD_MS); // Ajusta el tiempo de espera según sea necesario
    Serial.printl("LED");
  }
}


void recto(){
  digitalWrite(PIN_Motor_AIN_1, HIGH);
  analogWrite(PIN_Motor_PWMA, VEL);
  
  digitalWrite(PIN_Motor_BIN_1, HIGH);
  analogWrite(PIN_Motor_PWMB, VEL);
}

void dcha(){
  digitalWrite(PIN_Motor_AIN_1, LOW);
  analogWrite(PIN_Motor_PWMA, VEL/1.5);
  
  digitalWrite(PIN_Motor_BIN_1, HIGH);
  analogWrite(PIN_Motor_PWMB, VEL/1.5);
}

void izq(){
  digitalWrite(PIN_Motor_AIN_1, HIGH);
  analogWrite(PIN_Motor_PWMA, VEL/1.5);
  
  digitalWrite(PIN_Motor_BIN_1, LOW);
  analogWrite(PIN_Motor_PWMB, VEL/1.5);
}


void tareaSeguidor(void *parameter) {
  while (1) {
    // Tu lógica de seguimiento de línea aquí
    // Lectura de valores de los sensores infrarrojos
    int leftSensorValue = analogRead(PIN_ITR20001_LEFT);
    int middleSensorValue = analogRead(PIN_ITR20001_MIDDLE);
    int rightSensorValue = analogRead(PIN_ITR20001_RIGHT);

    if (middleSensorValue > umbral) {
      recto();
    } else if (rightSensorValue > umbral) {
      dcha();
    } else if (leftSensorValue > umbral) {
      izq();
    }

    vTaskDelay(100 / portTICK_PERIOD_MS); // Ajusta el tiempo de espera según sea necesario
    Serial.println("seguidor");
  }
}




/*
uint32_t Color(uint8_t r, uint8_t g, uint8_t b)
{
  return (((uint32_t)r << 16) | ((uint32_t)g << 8) | b);
}

void myColor()
{
  int r=255,g=0,b=0;
    for(int i=0;i<255;i++)
    {
      FastLED.showColor(Color(r, g, b));
      r-=1;
      g+=1;
      delay(10);
    }
    r=0,g=0,b=255;
    
    for(int i=0;i<255;i++)
    {
      FastLED.showColor(Color(r, g, b));
      r+=1;
      b-=1;
      delay(10);
    }
    r=0,g=255,b=0;

    for(int i=0;i<255;i++)
    {
      FastLED.showColor(Color(r, g, b));
      g-=1;
      b+=1;
      delay(10);
    }
    r=0,g=0,b=0;
}
*/



void seguidor(){
  // Lectura de valores de los sensores infrarrojos
  leftSensorValue = analogRead(PIN_ITR20001_LEFT);
  middleSensorValue = analogRead(PIN_ITR20001_MIDDLE);
  rightSensorValue = analogRead(PIN_ITR20001_RIGHT);

  if (middleSensorValue > umbral) {
    recto();
  }
  else if (rightSensorValue > umbral) {
    dcha();
  }
  else if (leftSensorValue > umbral) {
    izq();
  }
}

/*
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
*/

void tareaUltrasonido(void *parameter) {
  while (1) {
    // Realizar la lógica del sensor de ultrasonido
    long t;
    long d;
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    t = pulseIn(ECHO_PIN, HIGH);
    d = t / 59;
    Serial.println(d);
    delay(100);

    vTaskDelay(200 / portTICK_PERIOD_MS); // Ajusta el tiempo según sea necesario
  }
}



void loop() {
  
  // ultrasonido();

  //seguidor();
  
  //myColor();

}
