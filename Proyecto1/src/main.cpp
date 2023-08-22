
//****************************************************************
// UVG BE3015-Digital 2
// Laboratorio 4
// Ingebor Ayleen Rubio Vasquez - 19003
//****************************************************************
//****************************************************************
// Librerías
//****************************************************************
#include <Arduino.h>
#include "driver/ledc.h"
#include <ESP32Servo.h>
//****************************************************************
// Definición de etiquetas
//****************************************************************
// Paso 1: selección de parámetros de la señal PWM
#define pwmChannel 0 // 16 canales 0-15
#define ledRChannel 1
#define ledGChannel 2
#define ledAChannel 3
#define pwmChannelServo 4
#define freqPWM 5000  // Frecuencia en Hz
#define freqPWM1 1000 // Frecuencia en Hz
#define freqPWM2 500  // Frecuencia en Hz
#define freqPWMServo 50
#define resolution 8 // 1-16 bits de resolución

// Pines de entrada
#define pinLedR 27
#define pinLedG 12
#define pinLedA 14
#define pinPWM 15 // GPIO 2 para tener la salida del PWM

#define boton1 2

#define pinServo 13

#define A 34
#define B 35
#define C 23
#define D 21
#define E 22
#define F 32
#define G 33

#define GND1 18
#define GND2 19
#define GND3 5

//******************Sensor de temperatura*******************
// Codigo de: https://esp32io.com/tutorials/esp32-lm35-temperature-sensor
//**********************************************************

// #define ADC_VREF_mV 3300.0 // in millivolt
// #define ADC_RESOLUTION 4096.0
// #define PIN_LM35 34 // ESP32 pin GPIO36 (ADC0) connected to LM35

//****************************************************************
// Prototipos de funciones
//****************************************************************
void configurarPWM(void);
float obtenerTemp(void);
void mostrarDig(int dig);
void mostrarCompleto (int numero);
//****************************************************************
// Variables Globales
//****************************************************************

// int dutycycle = 0;
int color = 0; // 0 para rojo 1 para verde y 2 para azul
int dcR = 256;
int dcG = 256;
int dcB = 256;

int caso = 0;

Servo myservo;

float temp = 0.0;
int pos = 0;

//****************************************************************
// ISR: Interrupciones
//****************************************************************
// void IRAM_ATTR ISR(){
//
//}
//****************************************************************
// Configuración
//****************************************************************
void setup()
{
  Serial.begin(115200);
  configurarPWM();
  pinMode(boton1, INPUT_PULLDOWN);
  myservo.attach(pinServo);
  myservo.write(pos);
}
//****************************************************************
// Loop Principal
//****************************************************************
void loop()
{
  int estadob1 = digitalRead(boton1); // lEER ESTADO DE LOS BOTONES
  if (estadob1 == 1)
  {
    temp = obtenerTemp();
    Serial.println("Obteniendo temperatura: ");
    Serial.println(temp);
    if (temp <= 37.0)
    {
      caso = 0;
      Serial.println("Temp menor a 37");
      myservo.write(25);
      delay(1000);
    }
    else if (temp > 37.0 && temp <= 37.5)
    {
      caso = 1;
      Serial.println("Temp entre 37 y 37.5");
    }
    else if (temp > 37)
    {
      caso = 2;
      Serial.println("Temp mayor a 37");
    }

    switch (caso)
    {
    case 0:                       // Servo a 30° y modificar LED verde
      ledcWrite(ledRChannel, 0);  // Apagar LED rojo
      ledcWrite(ledAChannel, 0);  // Apagar LED amarillo
      ledcWrite(pwmChannel, dcG); // Encender LED verde
      ledcWrite(ledGChannel, dcG);
      myservo.write(30);
      delay(10);
      break;
    case 1:                       // Servo a 90° y modificar LED amarillo
      ledcWrite(ledRChannel, 0);  // Apagar LED rojo
      ledcWrite(ledGChannel, 0);  // Apagar LED verde
      ledcWrite(pwmChannel, dcB); // Encender LED amarillo
      ledcWrite(ledAChannel, dcB);
      myservo.write(90);
      delay(10);

      break;
    case 2:                       // Servo a 180° y modificar LED rojo
      ledcWrite(ledGChannel, 0);  // Apagar led verde
      ledcWrite(ledAChannel, 0);  // Apagar led amarillo
      ledcWrite(pwmChannel, dcR); // Encender led rojo
      ledcWrite(ledRChannel, dcR);
      myservo.write(150);
      delay(10);
      break;
    }
  }
}
//****************************************************************
// Función para configurar módulo PWM
//****************************************************************
void configurarPWM(void)
{
  // Paso 1: Configurar el módulo PWM
  ledcSetup(pwmChannel, freqPWM, resolution);
  ledcSetup(ledRChannel, freqPWM1, resolution);
  ledcSetup(ledGChannel, freqPWM2, resolution);
  ledcSetup(ledAChannel, freqPWM, resolution);
  // ledcSetup(pwmChannelServo, freqPWMServo, resolution);
  //  Paso 2: seleccionar en que GPIO tendremos nuestra señal PWM
  ledcAttachPin(pinPWM, pwmChannel);
  ledcAttachPin(pinLedR, ledRChannel);
  ledcAttachPin(pinLedG, ledGChannel);
  ledcAttachPin(pinLedA, ledAChannel);
  // ledcAttachPin(pinServo, pwmChannelServo);
}

float obtenerTemp(void)
{
  //******************************Descomentar al tener sensor******************
  /*
  // read the ADC value from the temperature sensor
  int adcVal = analogRead(PIN_LM35);
  // convert the ADC value to voltage in millivolt
  float milliVolt = adcVal * (ADC_VREF_mV / ADC_RESOLUTION);
  // convert the voltage to the temperature in °C
  float tempC = milliVolt / 10;
  // convert the °C to °F
  float tempF = tempC * 9 / 5 + 32;

  // print the temperature in the Serial Monitor:
  Serial.print("Temperature: ");
  Serial.print(tempC);   // print the temperature in °C
  Serial.print("°C");
  Serial.print("  ~  "); // separator between °C and °F
  Serial.print(tempF);   // print the temperature in °F
  Serial.println("°F");
  */

  //*****************************************************

  // Temporal en lo que no se tiene el sensor
  float tempC = 36.5;

  delay(500);
  return tempC;
}

// Funciones para digitos

void mostrarDig(int dig)
{
  switch (dig)
  {
  case 0:
    digitalWrite(A, HIGH);
    digitalWrite(B, HIGH);
    digitalWrite(C, HIGH);
    digitalWrite(D, HIGH);
    digitalWrite(E, HIGH);
    digitalWrite(F, HIGH);
    digitalWrite(G, LOW);
    break;
  case 1:
    digitalWrite(A, LOW);
    digitalWrite(B, HIGH);
    digitalWrite(C, HIGH);
    digitalWrite(D, LOW);
    digitalWrite(E, LOW);
    digitalWrite(F, LOW);
    digitalWrite(G, LOW);
    break;
  case 2:
    digitalWrite(A, HIGH);
    digitalWrite(B, HIGH);
    digitalWrite(C, LOW);
    digitalWrite(D, HIGH);
    digitalWrite(E, HIGH);
    digitalWrite(F, LOW);
    digitalWrite(G, HIGH);
    break;
  case 3:
    digitalWrite(A, HIGH);
    digitalWrite(B, HIGH);
    digitalWrite(C, HIGH);
    digitalWrite(D, HIGH);
    digitalWrite(E, LOW);
    digitalWrite(F, LOW);
    digitalWrite(G, HIGH);
    break;
  case 4:
    digitalWrite(A, LOW);
    digitalWrite(B, HIGH);
    digitalWrite(C, HIGH);
    digitalWrite(D, LOW);
    digitalWrite(E, LOW);
    digitalWrite(F, HIGH);
    digitalWrite(G, HIGH);
    break;
  case 5:
    digitalWrite(A, HIGH);
    digitalWrite(B, LOW);
    digitalWrite(C, HIGH);
    digitalWrite(D, HIGH);
    digitalWrite(E, LOW);
    digitalWrite(F, HIGH);
    digitalWrite(G, HIGH);
    break;
  case 6:
    digitalWrite(A, HIGH);
    digitalWrite(B, LOW);
    digitalWrite(C, HIGH);
    digitalWrite(D, HIGH);
    digitalWrite(E, HIGH);
    digitalWrite(F, HIGH);
    digitalWrite(G, HIGH);
    break;
  case 7:
    digitalWrite(A, HIGH);
    digitalWrite(B, HIGH);
    digitalWrite(C, HIGH);
    digitalWrite(D, LOW);
    digitalWrite(E, LOW);
    digitalWrite(F, LOW);
    digitalWrite(G, LOW);
    break;
  case 8:
    digitalWrite(A, HIGH);
    digitalWrite(B, HIGH);
    digitalWrite(C, HIGH);
    digitalWrite(D, HIGH);
    digitalWrite(E, HIGH);
    digitalWrite(F, HIGH);
    digitalWrite(G, HIGH);
    break;
  case 9:
    digitalWrite(A, HIGH);
    digitalWrite(B, HIGH);
    digitalWrite(C, HIGH);
    digitalWrite(D, HIGH);
    digitalWrite(E, LOW);
    digitalWrite(F, HIGH);
    digitalWrite(G, HIGH);
    break;
  }
}

void mostrarCompleto (int numero){
  mostrarDig(numero/100);
  digitalWrite(GND1, HIGH); // first digit on,
  digitalWrite(GND2, LOW); // other off
  digitalWrite(GND3, LOW);
  delay (1);

  numero = numero%100;  // remainder of 1234/1000 is 234
  digitalWrite(GND1, LOW); // first digit is off
  mostrarDig(numero/10); //// segments are set to display "2"
  digitalWrite(GND2, HIGH); // second digit is on
  delay (1); // and so on....
}
