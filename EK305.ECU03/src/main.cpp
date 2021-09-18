/* -------------------------------------------------------------------- */
//                     MÓDULO 03 - ATMEGA328P
// || SENSOR  ||                  || CÓDIGO  ||            ||  TESTE  ||
// Posição do Pedal de Freio	      DS2217
// Posição da Válvula Borboleta     TPS
// Sonda Lambda                     MTE 9700.50.150 
// Sensor de RPM                    Original Moto
// Posição do Volante               Potenciômetro 3590S                                
// Velocidade	                      Original Moto
/* -------------------------------------------------------------------- */

/*************************************************************************/
/*Observacoes
  1 - A maneira de contagem de RPM precisa ser refeita
  2 - As funções taskBrakePedalPos, taskSteeringWheelPos e taskTPS estão de uma maneira genérica para leitura de potenciômetro; precisa-se verificar como transformar os daddos
  3 - Verificar se a multiplicação de um tipo int por (5.0/1024) não faz-se perder as informações 
*/


//Definicao das portas
#define LED_CPU 8

#define CAN_SCK 13 //Pino SCK da CAN
#define CAN_SO 12  //Pino SO da CAN
#define CAN_SI 11  //Pino SI da CAN
#define CAN_CS 10  //Pino CS da CAN

#define LambdaPIN A1 //Pino do sensor Lambda
#define RPMPIN 3 //Pino do sensor de RPM
#define BrakePedalPosPIN A2 //Pino do sensor de posicao do pedal de freio
#define SteeringWheelPosPIN A3 //Pino do sensor de posicao do volante
#define TPSPIN A4 //Pino do sensor de posicao da borboleta


//Definicao do Timer
#define TMR_BASE 100000
#define TMR_BRAKE_PEDAL_POS 200000
#define TMR_LAMBDA_SENSOR 200000
#define TMR_RPM 200000
#define TMR_STEERING_WHEEL_POS 200000
#define TMR_SPEED 200000
#define TMR_TPS 200000

//Definicao CAN_ID
#define BrakePedalPos_CAN_ID 0xC2
#define LambdaSensor_CAN_ID 0x101
#define RPM_CAN_ID 0x2
#define SteeringWheelPos_CAN_ID 0xA
#define Speed_CAN_ID 0x301
#define TPS_CAN_ID 0x103

//Demais definicoes
#define NUM_AMOSTRAGEM 5
#define NUM_SPARKS 2

//Blibiotecas
#include <Arduino.h>
#include <EK305CAN.h>
#include <mcp2515.h>
#include <TimerOne.h>
#include <SPI.h>

//Prototipo das funcoes Setup
void SetupCAN(void);
void SetupInit(void);

//Prototipo das funcoes task
void taskLambdaSensor(); //Funcao que le e envia os dados do sensor Lambda
void taskScheduler(); //Funcao que gerencia o overflow
void taskBrakePedalPos(); //Funcao que le e envia a posicao do pedal do freio
void taskRPM(); //Funcao que le e envia RPM
void taskSteeringWheelPos(); //Funcao que le e envia a posicao do volante
void taskSpeed(); //Funcao que le e envia a velocidade
void taskTPS();

//Prototipo de funcoes auxiliares
int getRPM(void);
void ISR_RPM(void);

//Variaveis globais para o controle das tasks
bool tmrLambdaSensor_Overflow = false;
bool tmrLambdaSensor_Enable = false;
int tmrLambdaSensor_Count = 0;

bool tmrBrakePedalPos_Overflow = false;
bool tmrBrakePedalPos_Enable = false;
int tmrBrakePedalPos_Count = 0;

bool tmrRPM_Overflow = false;
bool tmrRPM_Enable = false;
int tmrRPM_Count = 0;

bool tmrSteeringWheelPos_Overflow = false;
bool tmrSteeringWheelPos_Enable = false;
int tmrSteeringWheelPos_Count = 0;

bool tmrSpeed_Overflow = false;
bool tmrSpeed_Enable = false;
int tmrSpeed_Count = 0;

bool tmrTPS_Overflow = false;
bool tmrTPS_Enable = false;
int tmrTPS_Count = false;

//Variaveis para a interrupcao do RPM
unsigned long InitialTime;
unsigned long TimeDif;
volatile bool flagRPM = false;
volatile int numPulses = 0;

//Pacotes CAN
can_frame LambdaSensor;
can_frame BrakePedalPos;
can_frame RPM;
can_frame SteeringWheelPos;
can_frame Speed;
can_frame TPS;

//Inicialização
MCP2515 mcp2515(CAN_CS);

void setup()
{
  SetupCAN();
  SetupInit();
}

void loop()
{
  taskLambdaSensor();
  taskBrakePedalPos();
  taskSteeringWheelPos();
  taskTPS();
}

void SetupCAN()
{
  digitalWrite(LED_CPU, HIGH);
  CAN_Init(&mcp2515, CAN_500KBPS);
  digitalWrite(LED_CPU, LOW);

  LambdaSensor.can_id = LambdaSensor_CAN_ID;
  BrakePedalPos.can_id = BrakePedalPos_CAN_ID;
  RPM.can_id = RPM_CAN_ID;
  SteeringWheelPos.can_id = SteeringWheelPos_CAN_ID;
  Speed.can_id = Speed_CAN_ID;

  LambdaSensor.can_dlc = 1;
  BrakePedalPos.can_dlc = 1;
  RPM.can_dlc = 2;
  SteeringWheelPos.can_dlc = 2;
  Speed.can_dlc = 1;
}

void SetupInit()
{
  Serial.begin(9600);

  pinMode(LED_CPU, OUTPUT);
  pinMode(LambdaPIN, INPUT);
  pinMode(RPMPIN, INPUT_PULLUP);
  pinMode(BrakePedalPosPIN, INPUT);
  pinMode(SteeringWheelPosPIN, INPUT);
  pinMode(TPSPIN, INPUT);


  Timer1.initialize(TMR_BASE);
  Timer1.attachInterrupt(taskScheduler);

  //attachInterrupt(digitalPinToInterrupt(RPMPIN), ISR_RPM, RISING); //toda vez que o pino referente ao sensor de RPM vai de LOW para HIGH, ocorre uma interrupcao

  tmrLambdaSensor_Enable = true;
  tmrBrakePedalPos_Enable = true;
  tmrRPM_Enable = false; 
  tmrSteeringWheelPos_Enable = true;
  tmrSpeed_Enable = false;
  tmrTPS_Enable = true;

  digitalWrite(LED_CPU, HIGH);
  delay(100);
  digitalWrite(LED_CPU, LOW);
  delay(100);
}

void taskScheduler()
{
  if(tmrLambdaSensor_Enable)
  {
    tmrLambdaSensor_Count++;
    if(tmrLambdaSensor_Count >= TMR_LAMBDA_SENSOR/TMR_BASE)
    {
      tmrLambdaSensor_Overflow = true;
      tmrLambdaSensor_Count = 0;
    }
  }

  if(tmrBrakePedalPos_Enable)
  {
    tmrBrakePedalPos_Count++;
    if(tmrBrakePedalPos_Count >= TMR_BRAKE_PEDAL_POS/TMR_BASE)
    {
      tmrBrakePedalPos_Overflow = true;
      tmrBrakePedalPos_Count = 0;
    }
  }

  if(tmrRPM_Enable)
  {
    tmrRPM_Count++;
    if(tmrRPM_Count >= TMR_RPM/TMR_BASE)
    {
      tmrRPM_Overflow = true;
      tmrRPM_Count = 0;
    }
  }

  if(tmrSteeringWheelPos_Enable)
  {
    tmrSteeringWheelPos_Count++;
    if(tmrSteeringWheelPos_Count >= TMR_STEERING_WHEEL_POS/TMR_BASE)
    {
      tmrSteeringWheelPos_Overflow = true;
      tmrSteeringWheelPos_Count = 0;
    }
  }

  if(tmrSpeed_Enable)
  {
    tmrSpeed_Count++;
    if(tmrSpeed_Count >= TMR_BRAKE_PEDAL_POS/TMR_BASE)
    {
      tmrSpeed_Overflow = true;
      tmrSpeed_Count = 0;
    }
  }

  if(tmrTPS_Enable)
  {
    tmrTPS_Count++;
    if(tmrTPS_Count >= TMR_TPS/TMR_BASE)
    {
      tmrTPS_Overflow = true;
      tmrTPS_Count = 0;
    }
  }
}

void taskLambdaSensor()
{
  int LambdaReadValue = analogRead(LambdaPIN);
  LambdaSensor.data[0] = (((LambdaReadValue - 0.2) * 0.65 / 4.6) + 0.65) * 100; //valor X100 para torna-lo inteiro //Converte a para fator Lambda
  if(mcp2515.sendMessage(&LambdaSensor) != MCP2515::ERROR::ERROR_OK)
  { // envia os dados de um CAN_Frame na CAN
    //tmrBlinkEnable = false;
  }
  tmrLambdaSensor_Overflow = false;
}

/*void taskRPM()//Precisa refazer
{
  int average = getRPM();
  RPM.data[1] = average & 0xFF << 8;
  RPM.data[0] = average & 0xFF;
  if (mcp2515.sendMessage(&RPM) != MCP2515::ERROR::ERROR_OK)
  { // envia os dados de um CAN_Frame na CAN
    //tmrBlinkEnable = false;
  }
  tmrRPM_Overflow = false;

  //tmrBlinkEnable = true;
  
}

int getRPM()
{
  unsigned long TimeDif; //Valor da diferenca de tempo entre dois pulsos
  unsigned int average;  //Media entre alguns RPMs para ter um valor com menos interferencias
  unsigned long RPM;     //RPM nao suavizado - o suavizado é a media
  unsigned int counter;  //Contador para fazer a media
  
  if (flagRPM)
  {
    flagRPM = false;     //Desativa a flag que chama a função
    counter = numPulses; //Coloca o valor de numPulses numa variável que não pertence ao interrupt
    numPulses = 0;       //Faz o numero de pulsos que ocorreu voltar para 0

    //Calcula os pulsos para virar rotação por min
    TimeDif = micros() - InitialTime; //Calcula a diferenca de tempo entre dois pulsos
    RPM = 60000000 / TimeDif;         //Faz o perídodo virar frequencia e multiplica por 60 a frequencia para ter rotacoes por MINUTO
    average = RPM * counter;          //Multiplica o RPM por (idealmente) NUM_AMOSTRAGEM para ter a media entre os NUM_AMOSTRAGEM periodos
    average = average / NUM_SPARKS;   //Divide a media pelo numero de centelhas numa revolução

    InitialTime = micros(); //Armazena o valor atual para calcular a diferença a próxima vez que for chamado
  }
  else
  {
    average = 0; //Caso a flag não esteja true é porque não teve pulso, ou seja, rotação
  }

    return average;  
}

void ISR_RPM()
{
  flagRPM = true;
  numPulses++;
}
*/
void taskBrakePedalPos()
{
  if(tmrBrakePedalPos_Overflow)
  {
    int position = analogRead(BrakePedalPosPIN); //Le o valor de posicao do pedal de freio

    position = position*(5.0/1024);

    BrakePedalPos.data[0] = position&0xFF;

    if(mcp2515.sendMessage(&BrakePedalPos)!=MCP2515::ERROR::ERROR_OK)
    {
    
    }
  }
  tmrBrakePedalPos_Overflow = false;
}

void taskSteeringWheelPos()
{
  if(tmrSteeringWheelPos_Overflow)
  {
    int position = analogRead(SteeringWheelPosPIN); //Le o valor de posicao do pedal de freio

    position = position*(5.0/1024);

    SteeringWheelPos.data[1] = (position>>8)&0xFF;/************************Problema com o tipo de variavel***************************/
    SteeringWheelPos.data[0] = position&0xFF;

    if(mcp2515.sendMessage(&SteeringWheelPos)!=MCP2515::ERROR::ERROR_OK)
    {
    
    }
  }
  tmrSteeringWheelPos_Overflow = false;
}

void taskTPS()
{
  if(tmrTPS_Overflow)
  {
    int position = analogRead(TPSPIN); //Le o valor de posicao do pedal de freio

    position = position*(5.0/1024);

    TPS.data[0] = position&0xFF;

    if(mcp2515.sendMessage(&TPS)!=MCP2515::ERROR::ERROR_OK)
    {
    
    }
  }
  tmrTPS_Overflow = false;
}