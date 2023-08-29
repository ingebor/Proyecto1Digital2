
//****************************************************************
// UVG BE3015-Digital 2
// Proyecto 1
// Ingebor Ayleen Rubio Vasquez - 19003
//****************************************************************
//****************************************************************
// Librerías
//****************************************************************
#include <Arduino.h>
#include "driver/ledc.h"
#include "config.h"
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
#define reServo 10

// Pines de entrada
#define pinLedR 27
#define pinLedG 12
#define pinLedA 14
#define pinPWM 15 // GPIO 2 para tener la salida del PWM
#define PIN_LM35 35

#define boton1 25

#define pinServo 13

#define A 2
#define B 4
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

#define ADC_VREF_mV 3300.0 // in millivolt
#define ADC_RESOLUTION 4095.0

//****************************************************************
// Prototipos de funciones
//****************************************************************
void configurarPWM(void);
float obtenerTemp(void);
void mostrarDig(int dig);
void mostrarCompleto(int numero);
//****************************************************************
// Variables Globales
//****************************************************************

// int dutycycle = 0;
int color = 0; // 0 para rojo 1 para verde y 2 para azul
int dcR = 256;
int dcG = 256;
int dcB = 256;
Servo myservo;
int pos = 0;
int caso = 0;
float temp = 0.0;
int tempDisplay = 0;
int estadoSemaforo = 0;

//****************************************************************
// ISR: Interrupciones
//****************************************************************
// void IRAM_ATTR ISR(){
//
//}

//*********************************************************
//  Configuraciones adafruit
//********************************************************
AdafruitIO_Feed *tempCanal = io.feed("Proyecto1Temperatura");
AdafruitIO_Feed *estadoLed = io.feed("ValorLed");

//****************************************************************
// Configuración
//****************************************************************
void setup()
{
  Serial.begin(115200);
  configurarPWM();
  pinMode(boton1, INPUT_PULLDOWN);
  pinMode(PIN_LM35, INPUT);
  pinMode(pinServo, OUTPUT);
  pinMode(A, OUTPUT);
  pinMode(B, OUTPUT);
  pinMode(C, OUTPUT);
  pinMode(D, OUTPUT);
  pinMode(E, OUTPUT);
  pinMode(F, OUTPUT);
  pinMode(G, OUTPUT);
  myservo.attach(pinServo);
  myservo.write(pos);
  pinMode(GND1, OUTPUT);
  pinMode(GND2, OUTPUT);
  pinMode(GND3, OUTPUT);

  // ledcWrite(0, map(180,0,180,0,1023));

  // Adafruit
  while (!Serial)
    ;
  Serial.print("Connecting to Adafruit IO");
  io.connect();
  // wait for a connection
  while (io.status() < AIO_CONNECTED)
  {
    Serial.print(".");
    delay(500);
  }
  // we are connected
  Serial.println();
  Serial.println(io.statusText());
}
//****************************************************************
// Loop Principal
//****************************************************************
void loop()
{

  int estadob1 = digitalRead(boton1); // lEER ESTADO DE LOS BOTONES
  if (estadob1 == 1)
  {
    io.run();
    temp = obtenerTemp();
    tempDisplay = temp * 10;
    Serial.println("**************\nObteniendo temperatura: ");
    Serial.println(temp);
    if (temp <= 15.8)
    {
      caso = 0;
      Serial.println("Temp menor a 15.8");
    }
    else if (temp > 15.8 && temp <= 16.3)
    {
      caso = 1;
      Serial.println("Temp entre 15.8 y 16.3");
    }
    else if (temp > 16.3)
    {
      caso = 2;
      Serial.println("Temp mayor a 16.3");
    }

    switch (caso)
    {
    case 0: // Servo a 30° y modificar LED verde
      estadoSemaforo = 0;
      ledcWrite(ledRChannel, 0);  // Apagar LED rojo
      ledcWrite(ledAChannel, 0);  // Apagar LED amarillo
      ledcWrite(pwmChannel, dcG); // Encender LED verde
      ledcWrite(ledGChannel, dcG);
      ledcWrite(pwmChannel, map(30, 0, 180, 30, 115));
      ledcWrite(pwmChannelServo, map(30, 0, 180, 30, 115));
      delay(10);
      break;
    case 1: // Servo a 90° y modificar LED amarillo
      estadoSemaforo = 1;
      ledcWrite(ledRChannel, 0);  // Apagar LED rojo
      ledcWrite(ledGChannel, 0);  // Apagar LED verde
      ledcWrite(pwmChannel, dcB); // Encender LED amarillo
      ledcWrite(ledAChannel, dcB);
      ledcWrite(pwmChannel, map(90, 0, 180, 30, 115));
      ledcWrite(pwmChannelServo, map(90, 0, 180, 30, 115));
      delay(10);

      break;
    case 2: // Servo a 180° y modificar LED rojo
      estadoSemaforo = 2;
      ledcWrite(ledGChannel, 0);  // Apagar led verde
      ledcWrite(ledAChannel, 0);  // Apagar led amarillo
      ledcWrite(pwmChannel, dcR); // Encender led rojo
      ledcWrite(ledRChannel, dcR);
      ledcWrite(pwmChannel, map(150, 0, 180, 30, 115));
      ledcWrite(pwmChannelServo, map(150, 0, 180, 30, 115));
      delay(10);
      break;
    }
    tempCanal->save(temp);           // Enviar temperatura a adafruit
    estadoLed->save(estadoSemaforo); // enviar estado del semáforo
    delay(3000);
  }

  mostrarCompleto(tempDisplay);
  delay(1);
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
  ledcSetup(pwmChannelServo, freqPWMServo, reServo);
  //  Paso 2: seleccionar en que GPIO tendremos nuestra señal PWM
  ledcAttachPin(pinPWM, pwmChannel);
  ledcAttachPin(pinLedR, ledRChannel);
  ledcAttachPin(pinLedG, ledGChannel);
  ledcAttachPin(pinLedA, ledAChannel);
  ledcAttachPin(pinServo, pwmChannelServo);
  ledcWrite(0, map(180, 0, 180, 0, 1023));
}

float obtenerTemp(void)
{
  //******************************Descomentar al tener sensor******************

  // read the ADC value from the temperature sensor
  int adcVal = analogRead(PIN_LM35);
  // convert the ADC value to voltage in millivolt
  float milliVolt = adcVal * (ADC_VREF_mV / ADC_RESOLUTION);
  // convert the voltage to the temperature in °C
  float tempC = milliVolt / 10;

  // print the temperature in the Serial Monitor:
  Serial.print("Temperature: ");
  Serial.print(tempC); // print the temperature in °C
  Serial.println("°C");

  //*****************************************************

  // Temporal en lo que no se tiene el sensor
  // float tempC = 38.3;

  delay(500);
  return tempC;
}

// Funciones para digitos

void mostrarDig(int dig) // Encender los pines correspondientes al dígito
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

void mostrarCompleto(int numero)
{
  mostrarDig(numero / 100);
  digitalWrite(GND1, HIGH); // encender el primer dígito
  digitalWrite(GND2, LOW);  // apagar segundo dígito
  digitalWrite(GND3, LOW);  // apagar tercer dígito
  delay(1);

  numero = numero % 100;    // obtener el segundo dígito
  digitalWrite(GND1, LOW);  // apagar el primer digito
  mostrarDig(numero / 10);  // mostrar el segundo dígito
  digitalWrite(GND2, HIGH); // encender el segundo dígito
  delay(1);

  numero = numero % 10;     // Obtener el tercer dígito
  digitalWrite(GND2, LOW);  // apagar el segundo dígito
  mostrarDig(numero);       // mostrar el tercer dígito
  digitalWrite(GND3, HIGH); // encender el tercer dígito
  delay(1);
}
