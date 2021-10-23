/* -------------------------------------------------------------------- */
//                     MÓDULO 03 - ATMEGA328P
// || SENSOR  ||                               || CÓDIGO  ||            ||  TESTE  ||
// Pressão do ar na saída do intercooler	        MTE 7133
// Temperatura do ar na saída do intercooler      MTE 5053
// Pressão do ar na saída do intercooler          MTE 7133
// Temperatura do ar na saída do intercooler      MTE 5033   
/* -------------------------------------------------------------------- */
/*************************************************************************/
/*Observacoes 
  1 - Verificar procedencia da formula de conversao do sinal dos sensore
  2 - Verificar se os nomes das variaveis nao estao confusos
*/

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
  CAN_Init(&mcp2515, CAN_500KBPS);
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
    float pressao;
    unsigned int pres;

    pressao = turboPressure1();

    pres = (unsigned int)(pressao);
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

float turboPressure1()
{
  float pressao;
  float a = 53.743;
  float b = -1.0935;

  pressao = ((5.0 * analogRead(Intercooler_IN_AirPress_PIN)) / 1023.0);
  pressao = pressao * a + b;

  if (pressao < 0)
  {
    pressao = 0;
  }

  return pressao;
}

void task_IN_AirTemp()
{
  if(tmr_IN_AirTemp_Overflow)
  {
    float temperatura;
    unsigned int temp;

    float tensao = analogRead(Intercooler_IN_AirTemp_PIN) * (5.0 / 1023.0);

    Serial.println(tensao);

    temperatura = (tensao - 0.34)*100;

    //temperatura = ((1.0 / -0.0404) * log((tensao * 1000.0) / (7021.0 * (5.0 - tensao))));
    //temp = (unsigned int)(temperatura);

    Serial.println(temperatura);

    IN_AirTemp.data[0] = temp;
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
    float pressao;
    unsigned int pres;

    pressao = turboPressure1();

    pres = (unsigned int)(pressao);
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

float turboPressure2()
{
  float pressao;
  float a = 53.743;
  float b = -1.0935;

  pressao = ((5.0 * analogRead(Intercooler_OUT_AirPress_PIN)) / 1023.0);
  pressao = pressao * a + b;

  if (pressao < 0)
  {
    pressao = 0;
  }

  return pressao;
}

void task_OUT_AirTemp()
{
  if(tmr_OUT_AirTemp_Overflow)
  {
    float temperatura;
    unsigned int temp;

    float tensao = analogRead(Intercooler_OUT_AirTemp_PIN) * (5.0 / 1023.0);

    temperatura = ((1.0 / -0.0404) * log((tensao * 1000.0) / (7021.0 * (5.0 - tensao))));
    temp = (unsigned int)(temperatura);
    OUT_AirTemp.data[0] = temp;
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