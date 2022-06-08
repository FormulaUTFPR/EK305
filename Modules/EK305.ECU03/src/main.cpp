/* -------------------------------------------------------------------- */
//                     MÓDULO 03 - ATMEGA328P
// || SENSOR  ||                  || CÓDIGO  ||            ||  TESTE  ||
//  MAP	                        Integrado na Central
//  Sensor de Temperatura	            MTE5053
//  TPS	                             MTE 7248
//  Sensor de Posição do Volante	Potenciômetro 3590S
/* -------------------------------------------------------------------- */

/*************************************************************************/


//Definicao das portas
#define LED_CPU 8

#define CAN_SCK 13 //Pino SCK da CAN
#define CAN_SO 12  //Pino SO da CAN
#define CAN_SI 11  //Pino SI da CAN
#define CAN_CS 10  //Pino CS da CAN

#define TPSPIN A4 //Pino do sensor de posicao da borboleta


//Definicao do Timer
#define TMR_BASE 100000
#define TMR_TPS 200000

//Definicao CAN_ID
#define BrakePedalPos_CAN_ID 0xC2
#define LambdaSensor_CAN_ID 0x101
#define RPM_CAN_ID 0x2
#define SteeringWheelPos_CAN_ID 0xA
#define Speed_CAN_ID 0x301
#define TPS_CAN_ID 0x103


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
void taskTPS();

//Prototipo de funcoes auxiliares


//Variaveis globais para o controle das tasks
bool tmrTPS_Overflow = false;
bool tmrTPS_Enable = false;
int tmrTPS_Count = false;

//Pacotes CAN
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
  taskTPS();
}

void SetupCAN()
{
  digitalWrite(LED_CPU, HIGH);
  //CAN_Init(&mcp2515, CAN_500KBPS);
  digitalWrite(LED_CPU, LOW);


}

void SetupInit()
{
  Serial.begin(9600);

  pinMode(LED_CPU, OUTPUT);
  pinMode(TPSPIN, INPUT);


  Timer1.initialize(TMR_BASE);
  Timer1.attachInterrupt(taskScheduler);

  //attachInterrupt(digitalPinToInterrupt(RPMPIN), ISR_RPM, RISING); //toda vez que o pino referente ao sensor de RPM vai de LOW para HIGH, ocorre uma interrupcao

  tmrTPS_Enable = false;

  digitalWrite(LED_CPU, HIGH);
  delay(100);
  digitalWrite(LED_CPU, LOW);
  delay(100);
}

void taskScheduler()
{

}

void taskLambdaSensor()
{

}

void taskTPS()
{
  if(tmrTPS_Overflow)
  {
    int position = analogRead(TPSPIN); //Le o valor de posicao do pedal de freio

    position = map(position, 0, 1023, 0, 100);//Converte os bits na abertura da borboleta(0% a 100%)

    TPS.data[0] = position&0xFF;

    if(mcp2515.sendMessage(&TPS)!=MCP2515::ERROR::ERROR_OK)
    {
    
    }
  }
  tmrTPS_Overflow = false;
}