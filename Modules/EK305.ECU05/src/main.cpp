/* -------------------------------------------------------------------- */
//                     MÓDULO 03 - ATMEGA328P
// || SENSOR  ||                               || CÓDIGO  ||            ||  TESTE  ||
//  Pressão de óleo                          	PS10
//  Sensor de Rotação	                         Original Moto
//  Sonda Lambda	                            MTE 9700.50.150
//  Temperatura de óleo	                         MTE 5043
//  Temperatura de Água	                         MTE 5043
//  Temperatura de ar na entrada do Intercooler	 MTE 5053
/* -------------------------------------------------------------------- */
/*************************************************************************/

//Definicao das portas
#define LED_CPU 8

#define CAN_SCK 13 //Pino SCK da CAN
#define CAN_SO 12  //Pino SO da CAN
#define CAN_SI 11  //Pino SI da CAN
#define CAN_CS 10  //Pino CS da CAN

//Definicao dos timers
#define TMR_BASE 100000

//Definicao CAN_ID


//Definicao CAN_ID

//Demais definicoes


//Blibiotecas
#include <Arduino.h>
#include <EK305CAN.h>
#include <mcp2515.h>
#include <TimerOne.h>
#include <SPI.h>

//Prototipos das funcoes Setup
void SetupCAN(void);
void SetupInit(void);

//Prototipos das funcoes task

//Prototipo de funcoes auxiliares

//Variaveis globais de controle das funcoes task

//Demais variaveis

//Pacotes CAN

//Inicializacoes
MCP2515 mcp2515(CAN_CS);

void setup(){

  SetupInit();
  SetupCAN();
}

void loop() {

}

void SetupCAN()
{
  digitalWrite(LED_CPU, HIGH);
  //CAN_Init(&mcp2515, CAN_500KBPS); //O codigo nao roda sem a CAN, so se comentar aqui
  digitalWrite(LED_CPU, LOW);

}

void SetupInit()
{
  Serial.begin(9600);

  pinMode(Intercooler_IN_AirPress_PIN, INPUT);
  pinMode(Intercooler_IN_AirTemp_PIN, INPUT);
  pinMode(Intercooler_OUT_AirPress_PIN, INPUT);
  pinMode(Intercooler_OUT_AirTemp_PIN, INPUT);

  Timer1.initialize(TMR_BASE);
  Timer1.attachInterrupt(taskScheduler);

}

void taskScheduler()
{

}

