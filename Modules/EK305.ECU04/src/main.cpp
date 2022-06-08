/* -------------------------------------------------------------------- */
//                     MÓDULO 04 - ATMEGA328P
// || SENSOR  ||                    || CÓDIGO  ||            ||  TESTE  ||
// Temperatura Restritor/Turbo	    MTE5053
// Temperatura Turbo/Intercooler	  MTE5053
// Temperatura Saída Intercooler	  MTE5053
// Pressão Restritor/Turbo	        MTE7133
// Pressão Turbo/Intercooler	      MTE7133
// Pressão Saída Intercooler	      MTE7133
// Indicador de Marcha	            Original Moto
/* -------------------------------------------------------------------- */

//Definicao das portas
#define LED_CPU 8

#define CAN_SCK 13 //Pino SCK da CAN
#define CAN_SO 12  //Pino SO da CAN
#define CAN_SI 11  //Pino SI da CAN
#define CAN_CS 10  //Pino CS da CAN

#define Intercooler_IN_AirPress_PIN A1 //Pino que recebe o sinal do sensor de pressao posicionado antes do intercooler 
#define Intercooler_IN_AirTemp_PIN A2 //Pino que recebe o sinal do sensor de temperatura posicionado antes do intercooler
#define Intercooler_OUT_AirPress_PIN A3 //Pino que recebe o sinal do sensor de pressao posicionado depois do intercooler
#define Intercooler_OUT_AirTemp_PIN A4 //Pino que recebe o sinal do sensor de temperatura posicionado depois do intercooler

//Definicao dos timers
#define TMR_BASE 100000
#define TMR_IN_AIR_PRESS 200000
#define TMR_IN_AIR_TEMP 200000
#define TMR_OUT_AIR_PRESS 200000
#define TMR_OUT_AIR_TEMP 200000

//Definicao CAN_ID
#define IN_AirPress_CAN_ID 0xA4
#define IN_AirTemp_CAN_ID 0xB4
#define OUT_AirPress_CAN_ID 0xC
#define OUT_AirTemp_CAN_ID 0xD4

//Definicao CAN_ID
#define IN_AirPress_CAN_DLC 1
#define IN_AirTemp_CAN_DLC 1
#define OUT_AirPress_CAN_DLC 1
#define OUT_AirTemp_CAN_DLC 1

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
void task_IN_AirPress(); //Funcao que le e envia os dados referentes a pressao do ar na entrada do intercooler
void task_IN_AirTemp(); //Funcao que le e envia os dados referentes a temperatura do ar na entrada do intercooler
void taskScheduler();
void task_OUT_AirPress(); //Funcao que le e envia os dados referentes a pressao do ar na saida do intercooler
void task_OUT_AirTemp(); //Funcao que le e envia os dados referentes a temperatura do ar na saida do intercooler

//Prototipo de funcoes auxiliares
float turboPressure1(); //Funcao que ajuda a calcular o valor da pressao na entrada do intercooler
float turboPressure2(); //Funcao que ajuda a calcular o valor da pressao na saida do intercooler

//Variaveis globais de controle das funcoes task
bool tmr_IN_AirPress_Overflow = false;
bool tmr_IN_AirPress_Enable = false;
int tmr_IN_AirPress_Count = 0;

bool tmr_IN_AirTemp_Overflow = false;
bool tmr_IN_AirTemp_Enable = false;
int tmr_IN_AirTemp_Count = 0;

bool tmr_OUT_AirPress_Overflow = false;
bool tmr_OUT_AirPress_Enable = false;
int tmr_OUT_AirPress_Count = 0;

bool tmr_OUT_AirTemp_Overflow = false;
bool tmr_OUT_AirTemp_Enable = false;
int tmr_OUT_AirTemp_Count = 0;

//Demais variaveis
const float Conversion =  0.0048875855; // 5/1023 v/bit

//Pacotes CAN
can_frame IN_AirPress;
can_frame IN_AirTemp;
can_frame OUT_AirPress;
can_frame OUT_AirTemp;

//Inicializacoes
MCP2515 mcp2515(CAN_CS);

void setup(){

  SetupInit();
  SetupCAN();
}

void loop() {
  task_IN_AirPress();
  task_IN_AirTemp();
  task_OUT_AirPress();
  task_OUT_AirTemp();
}

void SetupCAN()
{
  digitalWrite(LED_CPU, HIGH);
  //CAN_Init(&mcp2515, CAN_500KBPS); //O codigo nao roda sem a CAN, so se comentar aqui
  digitalWrite(LED_CPU, LOW);

  IN_AirPress.can_id = IN_AirPress_CAN_ID;
  IN_AirTemp.can_id = IN_AirTemp_CAN_ID;
  OUT_AirPress.can_id = OUT_AirPress_CAN_ID;
  OUT_AirTemp.can_id = OUT_AirTemp_CAN_ID;

  IN_AirPress.can_dlc = IN_AirPress_CAN_DLC;
  IN_AirTemp.can_dlc = IN_AirTemp_CAN_DLC;
  OUT_AirPress.can_dlc = OUT_AirPress_CAN_DLC;
  OUT_AirTemp.can_dlc = OUT_AirTemp_CAN_DLC;
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

  tmr_IN_AirPress_Enable = false;
  tmr_IN_AirTemp_Enable = true;
  tmr_OUT_AirPress_Enable = false;
  tmr_OUT_AirTemp_Enable = false;
}

void taskScheduler()
{
  if(tmr_IN_AirPress_Enable)
  {
    tmr_IN_AirPress_Count++;
    if(tmr_IN_AirPress_Count >= TMR_IN_AIR_PRESS/TMR_BASE)
    {
      tmr_IN_AirPress_Overflow = true;
      tmr_IN_AirPress_Count = 0;
    }
  }

  if(tmr_IN_AirTemp_Enable)
  {
    tmr_IN_AirTemp_Count++;
    if(tmr_IN_AirTemp_Count >= TMR_IN_AIR_TEMP/TMR_BASE)
    {
      tmr_IN_AirTemp_Overflow = true;
      tmr_IN_AirTemp_Count = 0;
    }
  }

  if(tmr_OUT_AirPress_Enable)
  {
    tmr_OUT_AirPress_Count++;
    if(tmr_OUT_AirPress_Count >= TMR_IN_AIR_PRESS/TMR_BASE)
    {
      tmr_OUT_AirPress_Overflow = true;
      tmr_OUT_AirPress_Count = 0;
    }
  }

  if(tmr_OUT_AirTemp_Enable)
  {
    tmr_OUT_AirTemp_Count++;
    if(tmr_OUT_AirTemp_Count >= TMR_OUT_AIR_TEMP/TMR_BASE)
    {
      tmr_OUT_AirTemp_Overflow = true;
      tmr_OUT_AirTemp_Count = 0;
    }
  }
}

void task_IN_AirPress()
{
  if(tmr_IN_AirPress_Overflow)
  {

    IN_AirPress.data[0] = pres;
    if(mcp2515.sendMessage(&IN_AirPress)!=MCP2515::ERROR::ERROR_OK)
    {
      
    }
    else
    {
      Serial.println("Deu ruim IN_AirPress");
    }

    tmr_IN_AirPress_Overflow = false;
  }
}


void task_IN_AirTemp()
{
  if(tmr_IN_AirTemp_Overflow)
  {
    float temperatura;
    float tensao = 0;
    unsigned int temp;

    for(int i = 0; i < 40; i++)
    {
      tensao = tensao + analogRead(Intercooler_IN_AirTemp_PIN) * Conversion;
    }
    tensao = tensao/40;//Media de 40 leituras para evitar flutuacoes absurdas
    Serial.println(tensao);

    temperatura = tensao*39.125 + 3.140;//Formula obtida a partir dos dados lidos do sensor. Usou-se o Exce(0.0509 - 0.0982)
    
    Serial.println(temperatura);

    tensao = 0;

    IN_AirTemp.data[0] = temperatura;
    if(mcp2515.sendMessage(&IN_AirTemp)!=MCP2515::ERROR::ERROR_OK)
    {

    }
    else
    {
      //Serial.println("Deu ruim IN_AirTemp");
    }

    tmr_IN_AirTemp_Overflow = false;
  }
}

void task_OUT_AirPress()
{
  if(tmr_OUT_AirPress_Overflow)
  {
    OUT_AirPress.data[0] = pres;
    if(mcp2515.sendMessage(&OUT_AirPress)!=MCP2515::ERROR::ERROR_OK)
    {
      
    }
    else
    {
      Serial.println("Deu ruim OUT_AirPress");
    }

    tmr_OUT_AirPress_Overflow = false;
  }
}

void task_OUT_AirTemp()
{
  if(tmr_OUT_AirTemp_Overflow)
  {
    float temperatura;
    float tensao = 0;
    unsigned int temp;

    for(int i = 0; i < 40; i++)
    {
      tensao = tensao + analogRead(Intercooler_IN_AirTemp_PIN) * Conversion;
    }
    tensao = tensao/40;//Media de 40 leituras para evitar flutuacoes absurdas
    Serial.println(tensao);

    temperatura = tensao*39.125 + 3.140;//Formula obtida a partir dos dados lidos do sensor. Usou-se o Exce(0.0509 - 0.0982)
    
    Serial.println(temperatura);

    tensao = 0;
    OUT_AirTemp.data[0] = temperatura;
    if(mcp2515.sendMessage(&OUT_AirTemp)!=MCP2515::ERROR::ERROR_OK)
    {

    }
    else
    {
      Serial.println("Deu ruim OUT_AirTemp");
    }

    tmr_OUT_AirTemp_Overflow = false;
  }
}