/* -------------------------------------------------------------------- */
//                     MÓDULO 02 - ATMEGA328P
// || SENSOR  ||                                       || CÓDIGO  ||                               ||  TESTE  ||
//  Sensor de posição da suspensão traseira esquerda     DS2201
//  Sensor de posição da suspensão traseira direita      DS2201
//  Sensor de tensão na bateria                          
//  Sensor de velocidade                                 Original moto
//  Sensor de corrente na bateria                
//  Acelerômetro 2                                       GPY-521/MPU6050
//  Acelerômetro 3                                       GPY-521/MPU6050
/* -------------------------------------------------------------------- */
/**********************************************/
/*Obervacoes
    
*/

//Definicao das portas
#define LED_CPU 8

#define CAN_SCK 13 //Pino SCK da CAN
#define CAN_SO 12  //Pino SO da CAN
#define CAN_SI 11  //Pino SI da CAN
#define CAN_CS 10  //Pino CS da CAN


//Definicao do Timer
#define TMR_BASE 100000


//Definicao CAN_ID


//Bibliotecas
#include <Arduino.h>
#include <Adafruit_MLX90614.h>
#include <SPI.h>
#include <TimerOne.h>
#include <EK305CAN.h>
#include <mcp2515.h>

//Prototipos de funcoes Setup
void setupCAN();
void setupInit();

//Prototipos de funcoes Task
void taskScheduler(void);

//Variaveis globais de controle das tasks


//Pacotes CAN


//Inicializacoes
MCP2515 mcp2515(CAN_CS);

void setup()
{
  setupCAN();
  setupInit();
}

void loop()
{

}

void setupCAN()
{
  digitalWrite(LED_CPU, HIGH);
  CAN_Init(&mcp2515, CAN_500KBPS);
  digitalWrite(LED_CPU, LOW);
  
}

void setupInit()
{
  Serial.begin(9600);
  pinMode(LED_CPU, OUTPUT);

  Timer1.initialize(TMR_BASE);
  Timer1.attachInterrupt(taskScheduler);

  digitalWrite(LED_CPU, HIGH);
  delay(100);
  digitalWrite(LED_CPU, LOW);
  delay(100);
}

void taskScheduler(void)
{

}

