/* -------------------------------------------------------------------- */
//                     MÓDULO 02 - ATMEGA328P
// || SENSOR  ||                             || CÓDIGO  ||                               ||  TESTE  ||
//  Temperatura do disco de freio frontal     IR MLX90614
//  Pressão do fluído de freio frontal
//  Temperatura do disco de freio traseiro    IR MLX90614
//  Pressão do fluído de freio traseiro
/* -------------------------------------------------------------------- */

//Definicao das portas
#define LED_CPU 8

#define CAN_SCK 13 //Pino SCK da CAN
#define CAN_SO 12  //Pino SO da CAN
#define CAN_SI 11  //Pino SI da CAN
#define CAN_CS 10  //Pino CS da CAN


//Definicao do Timer
#define TMR_BASE 100000
#define TMR_FRONT_BRAKE_TEMP 200000
#define TMR_REAR_BRAKE_TEMP 200000

//Definicao CAN_ID
#define FrontBrakeTemp_CAN_ID 0xB2
#define RearBrakeTemp_CAN_ID 0xA2  

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
void taskFrontBrakeTemp(void);
void taskRearBrakeTemp(void);
void taskScheduler(void);

//Variaveis globais de controle das tasks
bool tmrFrontBrakeTemp_Overflow = false;
bool tmrFrontBrakeTemp_Enable = false;
int tmrFrontBrakeTemp_Count = 0;

bool tmrRearBrakeTemp_Overflow = false;
bool tmrRearBrakeTemp_Enable = false;
int tmrRearBrakeTemp_Count = 0;


//Pacotes CAN
can_frame FrontBrakeTemp;
can_frame RearBrakeTemp;

//Inicializacoes
MCP2515 mcp2515(CAN_CS);
Adafruit_MLX90614 mlx = Adafruit_MLX90614();

void setup()
{
  setupCAN();
  setupInit();
}

void loop()
{
  taskFrontBrakeTemp();
  taskRearBrakeTemp();
}

void setupCAN()
{
  digitalWrite(LED_CPU, HIGH);
  CAN_Init(&mcp2515, CAN_500KBPS);
  digitalWrite(LED_CPU, LOW);

  FrontBrakeTemp.can_id = FrontBrakeTemp_CAN_ID;
  RearBrakeTemp.can_id = RearBrakeTemp_CAN_ID; 

  FrontBrakeTemp.can_dlc = 2;
  RearBrakeTemp.can_dlc = 2;  
}

void setupInit()
{
  Serial.begin(9600);
  mlx.begin();
  pinMode(LED_CPU, OUTPUT);

  Timer1.initialize(TMR_BASE);
  Timer1.attachInterrupt(taskScheduler);

  tmrFrontBrakeTemp_Enable = true;
  tmrRearBrakeTemp_Enable = true;

  digitalWrite(LED_CPU, HIGH);
  delay(100);
  digitalWrite(LED_CPU, LOW);
  delay(100);
}

void taskScheduler(void)
{
  if(tmrFrontBrakeTemp_Enable)
  {
    tmrFrontBrakeTemp_Count++;
    if(tmrFrontBrakeTemp_Count == TMR_FRONT_BRAKE_TEMP/TMR_BASE)
    {
      tmrFrontBrakeTemp_Overflow = true;
      tmrFrontBrakeTemp_Count = 0;
    }
  }

  if(tmrRearBrakeTemp_Enable)
  {
    tmrRearBrakeTemp_Count++;
    if(tmrRearBrakeTemp_Count == TMR_REAR_BRAKE_TEMP/TMR_BASE)
    {
      tmrRearBrakeTemp_Overflow = true;
      tmrRearBrakeTemp_Count = 0;
    }
  }
}

void taskFrontBrakeTemp(void)
{
  if(tmrFrontBrakeTemp_Overflow)
  {

  unsigned int temp1 = 0;//problema com o retorno da funcao readObject
  temp1 = mlx.readObjectTempC();

  FrontBrakeTemp.data[0] = (temp1>>8)&0xFF;
  FrontBrakeTemp.data[1] = temp1&0xFF;

  if(mcp2515.sendMessage(&FrontBrakeTemp) != MCP2515::ERROR::ERROR_OK)
  {

  }
  else 
  {
    Serial.println("Deu ruim: CAN FrontBrakeTemp");
  }

  tmrFrontBrakeTemp_Overflow = false;
  }
}

void taskRearBrakeTemp(void)
{
  if(tmrRearBrakeTemp_Overflow)
  {

  unsigned int temp2 = 0;//problema com o retorno da funcao readObject
  temp2 = mlx.readObjectTempC();

  RearBrakeTemp.data[0] = (temp2>>8)&0xFF;
  RearBrakeTemp.data[1] = temp2&0xFF;

  if(mcp2515.sendMessage(&RearBrakeTemp) != MCP2515::ERROR::ERROR_OK)
  {

  }
  else 
  {
    Serial.println("Deu ruim: CAN RearBrakeTemp");
  }

  tmrRearBrakeTemp_Overflow = false;
  }
}