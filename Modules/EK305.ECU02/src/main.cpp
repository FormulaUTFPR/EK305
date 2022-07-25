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
Se a canoa não virar,
Olê olê olê olá
Eu chego lá
*/

//Definicao das portas
#define LED_CPU 8  //Pisca pisca

#define CAN_SCK 13 //Pino SCK da CAN
#define CAN_SO 12  //Pino SO da CAN
#define CAN_SI 11  //Pino SI da CAN
#define CAN_CS 10  //Pino CS da CAN

#define VEL_PIN 3 // Pino do sensor de velocidade

//Definicao do Timer
#define TMR_SPEED 200

//Definicao CAN_ID
#define Speed_CAN_ID 0x301

//Demais definicoes
#define NUM_AMOSTRAGEM 5
#define NUM_PER_REV 2

//Bibliotecas
#include <Arduino.h>
#include <SPI.h>
#include <EK305CAN.h>
#include <mcp2515.h>

//Prototipos de funcoes Setup
void setupCAN();
void setupInit();

//Prototipo de funcoes auxiliares
int getVEL(void);
void ISR_VEL(void);

//Prototipos de funcoes Task
// void taskScheduler(void);
void taskVEL();

//Variaveis globais de controle das tasks

//Variaveis para a interrupcao da Velocidade
unsigned long InitialTime;
unsigned long TimeDif;
volatile bool flagVEL = false;
volatile int numPulses = 0;

//Pacotes CAN
can_frame Speed;

//Inicializacoes
MCP2515 mcp2515(CAN_CS);

void setup()
{
  setupCAN();
  setupInit();
}

void loop()
{
  if(mcp2515.sendMessage(&Speed)!=MCP2515::ERROR::ERROR_OK)
  {  
  }
  taskVEL();
  delay(100);
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

  pinMode(VEL_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(VEL_PIN), ISR_VEL, RISING);

/*
  Timer1.initialize(TMR_BASE);
  Timer1.attachInterrupt(taskScheduler);
*/

  digitalWrite(LED_CPU, HIGH);
  delay(100);
  digitalWrite(LED_CPU, LOW);
  delay(100);

  
  Speed.can_id = Speed_CAN_ID;
  Speed.can_dlc = 1;
}

void taskScheduler(void)
{

}

void taskVEL()
{
  int average = getVEL();
  Speed.data[0] = average & 0xFF;
  if (mcp2515.sendMessage(&Speed) != MCP2515::ERROR::ERROR_OK)
  { // envia os dados de um CAN_Frame na CAN
    //tmrBlinkEnable = false;
  }

  //tmrBlinkEnable = true;
  
}

int getVEL()
{
  unsigned long TimeDif; //Valor da diferenca de tempo entre dois pulsos
  unsigned int average;  //Media entre alguns RPMs para ter um valor com menos interferencias
  unsigned long VEL;     //RPM nao suavizado - o suavizado é a media
  unsigned int counter;  //Contador para fazer a media
  
  if (flagVEL)
  {
    flagVEL = false;     //Desativa a flag que chama a função
    counter = numPulses; //Coloca o valor de numPulses numa variável que não pertence ao interrupt (não que vá fazer diferença...)
    numPulses = 0;       //Faz o numero de pulsos que ocorreu voltar para 0

    //Calcula os pulsos para virar rotação por min
    TimeDif = micros() - InitialTime;   //Calcula a diferenca de tempo entre dois pulsos
    VEL = 3600000000 / TimeDif;         //Faz o perídodo virar frequencia e multiplica por 3600 a frequencia para ter rotacoes por HORA
    average = VEL * counter;            //Multiplica a velocidade por (idealmente) NUM_AMOSTRAGEM para ter a media entre os NUM_AMOSTRAGEM periodos
    average = average / NUM_PER_REV;    //Divide a media pelo numero de sinais numa revolução

    InitialTime = micros(); //Armazena o valor atual para calcular a diferença a próxima vez que for chamado
  }
  else
  {
    average = 0; //Caso a flag não esteja true é porque não teve pulso, ou seja, rotação no sensor
  }

    return average;  
}

void ISR_VEL()
{
  flagVEL = true;
  numPulses++;
}