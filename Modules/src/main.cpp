/* -------------------------------------------------------------------- */
//                     MÓDULO 01 - ATMEGA328P
// || SENSOR  ||                      || CÓDIGO  ||            ||  TESTE  ||
// Acelerômetro 01	                  GY-521 - MPU-6050
// Posição Suspensão Frontal Direita	DS2201
// Posição Suspensão Frontal Esquerda	DS2201
// Posição do Pedal do Freio	        DS2217
// Posição do Pedal do Acelerador	    DS2217
/* -------------------------------------------------------------------- */

//DEFINICAO DAS PORTAS
#define PIN_SUSP_DIREITA A1  //Porta para o sensor da suspensao direita
#define PIN_SUSP_ESQUERDA A0 //Porta para o sensor da suspensao esquerda
#define PIN_ACR_PEDAL_POS A2     //Porta do sensor de posicao do pedal do acelerador
#define PIN_BRAKE_PEDAL_POS A3   //Porta do sensor de posicao do pedal de freio
#define LED_CPU 8            //Porta para o LED do módulo

#define CAN_SCK 13 //Pino SCK da CAN
#define CAN_SO 12  //Pino SO da CAN
#define CAN_SI 11  //Pino SI da CAN
#define CAN_CS 10  //Pino CS da CAN


/*
//VARIAVEIS GLOBAIS
#define VALOR_MIN_LEITURA_SUSP 126 //Minimo valor de leitura na porta analogica
#define VALOR_MAX_LEITURA_SUSP 876 //Maximo valor de leitura na porta analogica
*/

// TIMERS

#define TMR_BASE 100000   //Clock base para os multiplicadores
#define TMR_SUSP 100000   //Timer para gravar dados da suspensão
#define TMR_ACC 100000    //Timer para gravar e enviar dados do acelerômetro 1
#define TMR_ACELE2 100000 //Timer para gravar e enviar dados do acelerômetro 2
#define TMR_BLINK 100000  //Timer para piscar o led
#define TMR_ACR_PEDAL 100000  //Timer para sensor do pedal do acelerador
#define TMR_BRAKE_PEDAL 100000  //Timer para sensor do curso do pedal de freio

//ENDERECO CAN
#define ACRPedal_CAN_ID 0x01
#define BrakePedal_CAN_ID 0x02

//BIBLIOTECAS
#include <SPI.h>
#include <mcp2515.h>
#include <Arduino.h>
#include <Wire.h>
#include <TimerOne.h>
#include <EK305CAN.h>

//PROTOTIPOS DE FUNCOES

void setupCAN();
void setupWIRE();

//CRIACAO DO PROTÓTIPO DE TASKS
void taskAcc(void);       //Task do acelerometro
void taskSusp(void);      //Task para leitura do curso de suspensão (no balancim?)
void taskACRPedalPos(void);     //Task para leitura e calcular curso do pedal
void taskBrakePedalPos(void);   //Task para leitura e calcular curso do pedal
void taskScheduler(void); //Task do escalonador
void taskBlink(void);     //Task de piscar o led
void setupInit(void);

//ENDERECOS DOS MODULOS
const int MPU1 = 0x68; // Se o pino ADO for conectado em GND o modulo assume esse endereço
const int MPU2 = 0x69; // Se o pino ADO for conectado em 5V ou 3,3V o modulo assume esse endereço

//Variáveis Globais

bool estadoLed = false;

//Variáveis para controle de Tarefas

bool tmrBlinkOverflow = false;
bool tmrBlinkEnable = false;
int tmrBlinkCount = 0;

bool tmrSuspOverflow = false;
bool tmrSuspEnable = false;
int tmrSuspCount = 0;

bool tmrAccOverflow = false;
bool tmrAccEnable = false;
int tmrAccCount = 0;

bool tmrACRPedalPos_Overflow = false;
bool tmrACRPedalPos_Enable = false;
int tmrACRPedalPos_Count = 0;

bool tmrBrakePedalPos_Overflow = false;
bool tmrBrakePedalPos_Enable = false;
int tmrBrakePedalPos_Count = 0;

//CAN
can_frame Acc;
can_frame Gyro;
can_frame ACR_Pedal_Pos;
can_frame Brake_Pedal_Pos;
can_frame Suspensao;

MCP2515 mcp2515(CAN_CS); //Pino 10 é o Slave

void setup()
{
  setupInit();

  setupCAN();

  setupWIRE();

  SPI.begin();
  Serial.begin(9600); //Usar somente para teste
}

void loop()
{
  taskAcc();
  taskSusp();
  taskACRPedalPos();
  taskBrakePedalPos();
  taskBlink();
}

//VERIFICAR SE E NECESSARIO ESSA PARTE

void setupCAN()
{
  //Configura a CAN
  digitalWrite(LED_CPU, HIGH);
  //CAN_Init(&mcp2515, CAN_500KBPS);
  digitalWrite(LED_CPU, LOW);

  //ACELERÔMETRO 01
  Acc.can_id = EK305CAN_ID_ACC_01;
  Acc.can_dlc = 6;

  //SUSPENSAO
  Suspensao.can_id = EK305CAN_ID_SUSP_FRONT;
  Suspensao.can_dlc = 2;

  //POSICAO DO PEDAL DO ACELERADOR
  ACR_Pedal_Pos.can_id = ACRPedal_CAN_ID;
  ACR_Pedal_Pos.can_dlc = 1;

  //POSICAO DO PEDAL DE FREIO
  Brake_Pedal_Pos.can_id = BrakePedal_CAN_ID;
  Brake_Pedal_Pos.can_dlc = 1;

}

void setupInit()
{
  
  //Configura o TimerOne
  Timer1.initialize(TMR_BASE);
  Timer1.attachInterrupt(taskScheduler);

  tmrSuspEnable = true;
  tmrACRPedalPos_Enable = false;
  tmrBrakePedalPos_Enable = false;
  tmrBlinkEnable = false;
  tmrAccEnable = false;

  digitalWrite(LED_CPU, HIGH);
  delay(100);
  digitalWrite(LED_CPU, LOW);
  delay(100);

  pinMode(LED_CPU, OUTPUT);
  pinMode(PIN_BRAKE_PEDAL_POS, INPUT);
  pinMode(PIN_ACR_PEDAL_POS, INPUT);
  pinMode(PIN_SUSP_DIREITA, INPUT);
  pinMode(PIN_SUSP_ESQUERDA, INPUT);
}

void taskScheduler(void)
{
  if (tmrSuspEnable)
  {
    tmrSuspCount++;
    if (tmrSuspCount >= TMR_SUSP / TMR_BASE)
    {
      tmrSuspCount = 0;
      tmrSuspOverflow = true;
    }
  }

  if (tmrAccEnable)
  {
    tmrAccCount++;
    if (tmrAccCount >= TMR_ACC / TMR_BASE)
    {
      tmrAccCount = 0;
      tmrAccOverflow = true;
    }
  }

  if (tmrACRPedalPos_Enable)
  {
    tmrACRPedalPos_Count++;
    if (tmrACRPedalPos_Count >= TMR_ACR_PEDAL / TMR_BASE)
    {
      tmrACRPedalPos_Count = 0;
      tmrACRPedalPos_Overflow = true;
    }
  }

  if (tmrBrakePedalPos_Enable)
  {
    tmrBrakePedalPos_Count++;
    if (tmrBrakePedalPos_Count >= TMR_BRAKE_PEDAL / TMR_BASE)
    {
      tmrBrakePedalPos_Count = 0;
      tmrBrakePedalPos_Overflow = true;
    }
  }

  if (tmrBlinkEnable)
  {
    tmrBlinkCount++;
    if (tmrBlinkCount >= TMR_BLINK / TMR_BASE)
    {
      tmrBlinkCount = 0;
      tmrBlinkOverflow = true;
    }
  }
  else
  {
    tmrBlinkCount++;
    if (tmrBlinkCount >= 10 * TMR_BLINK / TMR_BASE)
    {
      tmrBlinkCount = 0;
      tmrBlinkOverflow = true;
    }
  }
}

void taskBlink(void)
{
  if (tmrBlinkOverflow)
  {
    digitalWrite(LED_CPU, estadoLed);
    estadoLed != estadoLed;
    tmrBlinkOverflow = false;
  }
}

// ACELERÔMETRO 01
void taskAcc(void)
{

}

void taskSusp()
{
  if(tmrSuspOverflow)
  {

    if(mcp2515.sendMessage(&Suspensao)!=MCP2515::ERROR::ERROR_OK)
    {
    
    }
  }

  Suspensao.data[0] = //var&0xFF
  tmrSuspOverflow = false;
}

void taskACRPedalPos()
{
  if(tmrACRPedalPos_Overflow)
  {


    ACR_Pedal_Pos.data[0] = //variavel&0xFF;

    if(mcp2515.sendMessage(&ACR_Pedal_Pos)!=MCP2515::ERROR::ERROR_OK)
    {
    
    }
  }
  tmrACRPedalPos_Overflow = false;
}

void taskBrakePedalPos()
{
  if(tmrBrakePedalPos_Overflow)
  {

    Brake_Pedal_Pos.data[0] = position&0xFF;

    if(mcp2515.sendMessage(&Brake_Pedal_Pos)!=MCP2515::ERROR::ERROR_OK)
    {
    
    }
  }
  tmrBrakePedalPos_Overflow = false;
}
