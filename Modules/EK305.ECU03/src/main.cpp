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
#define AIR_TEMP_PIN A0
#define STEER_WHEEL_POS_PIN A1

//Definicao do Timer
#define TMR_BASE 100000
#define TMR_TPS 200000
#define TMR_AIR_TEMP 100000
#define TMR_STEER_WHEEL_POS 100000

//Definicao CAN_ID
#define STEER_WHEEL_POS_CAN_ID 0xA
#define AIR_TEMP_CAN_ID 0xB
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
void taskAirTemp();
void taskSteerWheelPos();

//Prototipo de funcoes auxiliares


//Variaveis globais para o controle das tasks
bool tmrTPS_Overflow = false;
bool tmrTPS_Enable = false;
int tmrTPS_Count = false;

bool tmrAirTemp_Overflow = false; 
bool tmrAirTemp_Enable = false;
int tmrAirTemp_count = false;

bool tmrSteerWheelPos_Overflow = false; 
bool tmrSteerWheelPos_Enable = false;
int tmrSteerWheelPos_count = false;


//Pacotes CAN
can_frame TPS;
can_frame AirTemp;
can_frame SteerWheelPos;

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
  taskAirTemp();
  taskSteerWheelPos();
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
  pinMode(TPS_PIN, INPUT);
  pinMode(AIR_TEMP_PIN, INPUT);
  pinMode(STEER_WHEEL_POS_PIN, INPUT);


  Timer1.initialize(TMR_BASE);
  Timer1.attachInterrupt(taskScheduler);

  //attachInterrupt(digitalPinToInterrupt(RPMPIN), ISR_RPM, RISING); //toda vez que o pino referente ao sensor de RPM vai de LOW para HIGH, ocorre uma interrupcao

  tmrTPS_Enable = false;
  tmrSteerWheelPos_Enable = false;
  tmrAirTemp_Enable = false;

  digitalWrite(LED_CPU, HIGH);
  delay(100);
  digitalWrite(LED_CPU, LOW);
  delay(100);
}

void taskScheduler()
{
  
  if(tmrTPS_Enable)
  {
    tmrTPS_count++;
    if(tmrTPS_Count >= TMR_TPS / TMR_BASE)
    {
      tmrTPS_Overflow = true;
      tmrTPS_Count = 0;} 
  }

  if(tmrAirTemp_Enable)
  {
    tmrAirTemp_count++;
    if(tmrAirTemp_Count >= TMR_AIR_TEMP / TMR_BASE)
    {
      tmrAirTemp_Overflow = true;
      tmrAirTemp_Count = 0;
    } 
  }
  
  if(tmrSteerWheelPos_Enable)
  {
    tmrSteerWheelPos_count++;
    if(tmrSteerWheelPos_Count >= TMR_STEER_WHEEL_POS / TMR_BASE)
    {
      tmrSteerWheelPos_Overflow = true;
      tmrSterWheelPos_Count = 0;
    } 
  }
}

void taskLambdaSensor()
{

}

void taskAirTemp()
{
    float temperatura;
    float tensao = 0;
    unsigned int temp;

    for(int i = 0; i < 40; i++)
    {
      tensao = tensao + analogRead(AIR_TEMP_PIN) * Conversion;
    }
    tensao = tensao/40;//Media de 40 leituras para evitar flutuacoes absurdas
    Serial.println(tensao);

    temperatura = tensao*39.125 + 3.140;//Formula obtida a partir dos dados lidos do sensor. Usou-se o Exce(0.0509 - 0.0982)

    tensao = 0; 
  
    AirTemp.data[0]  = temperatura;
  
    if(mcp2515.sendMessage(&AirTemp)!=MCP2515::ERROR::ERROR_OK)
    {
    }
  
  tmrAirTemp_Overflow = false;
}

void taskSteerWheelPos()
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
