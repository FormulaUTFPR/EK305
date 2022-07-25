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
#define SUSP_RIGHT_PIN A1  //Porta para o sensor da suspensao direita
#define SUSP_LEFT_PIN A0 //Porta para o sensor da suspensao esquerda
#define ACR_PEDAL_POS_PIN A2     //Porta do sensor de posicao do pedal do acelerador
#define BRAKE_PEDAL_POS_PIN A3   //Porta do sensor de posicao do pedal de freio
//#define ACC_PIN A4            //Porta do acelerometro/giroscopio
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
#define TMR_BLINK 100000  //Timer para piscar o led
#define TMR_ACR_PEDAL 100000  //Timer para sensor do pedal do acelerador
#define TMR_BRAKE_PEDAL 100000  //Timer para sensor do curso do pedal de freio

//ENDERECO CAN
#define ACR_PEDAL_POS_CAN_ID 0x01
#define BRAKE_PEDAL_POS_CAN_ID 0x02
#define SUSP_RIGHT_CAN_ID 0x03
#define SUSP_LEFT_CAN_ID 0x04
#define ACC_CAN_ID 0x05

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

float AccX, AccY, AccZ, Temp, GyrX, GyrY, GyrZ;
bool estadoLed = false;

//Variáveis para controle de Tarefas

bool tmrBlink_Overflow = false;
bool tmrBlink_Enable = false;
int tmrBlink_Count = 0;

bool tmrSusp_Overflow = false;
bool tmrSusp_Enable = false;
int tmrSusp_Count = 0;

bool tmrAcc_Overflow = false;
bool tmrAcc_Enable = false;
int tmrAcc_Count = 0;

bool tmrACRPedalPos_Overflow = false;
bool tmrACRPedalPos_Enable = false;
int tmrACRPedalPos_Count = 0;

bool tmrBrakePedalPos_Overflow = false;
bool tmrBrakePedalPos_Enable = false;
int tmrBrakePedalPos_Count = 0;

//CAN
can_frame Acc;
can_frame Gyro;
can_frame ACRPedalPos;
can_frame BrakePedalPos;
can_frame Susp;

MCP2515 mcp2515(CAN_CS); //Pino 10 é o Slave

void setup()
{
  setupInit();

  setupCAN();

  //setupWIRE();

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

void setupCAN()
{
  //Configura a CAN
  digitalWrite(LED_CPU, HIGH);
  //CAN_Init(&mcp2515, CAN_500KBPS);
  digitalWrite(LED_CPU, LOW);

  //ACELERÔMETRO 01
  Acc.can_id = ACC_CAN_ID;
  Acc.can_dlc = 6;

  //SUSPENSAO
  Susp.can_id = SUSP_RIGHT_CAN_ID;
  Susp.can_dlc = 2;

  //POSICAO DO PEDAL DO ACELERADOR
  ACRPedalPos.can_id = ACR_PEDAL_POS_CAN_ID;
  ACRPedalPos.can_dlc = 1;

  //POSICAO DO PEDAL DE FREIO
  BrakePedalPos.can_id = BRAKE_PEDAL_POS_CAN_ID;
  BrakePedalPos.can_dlc = 1;

}

void setupInit()
{
  
  //Configura o TimerOne
  Timer1.initialize(TMR_BASE);
  Timer1.attachInterrupt(taskScheduler);

  //Inicializa o acelerometro
  Wire.begin();
  Wire.beginTransmission(MPU1);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  //Configura o fundo de escala do giroscopio
  Wire.beginTransmission(MPU1);
  Wire.write(0x1B);
  Wire.write(0x00011000);  // Trocar esse comando para fundo de escala desejado conforme acima
  Wire.endTransmission();

  //Configura o fundo de escala do acelerometro
  Wire.beginTransmission(MPU1);
  Wire.write(0x1C);
  Wire.write(0b00011000);  // Trocar esse comando para fundo de escala desejado conforme acima
  Wire.endTransmission();

  tmrSusp_Enable = true;
  tmrACRPedalPos_Enable = false;
  tmrBrakePedalPos_Enable = false;
  tmrBlink_Enable = false;
  tmrAcc_Enable = false;

  digitalWrite(LED_CPU, HIGH);
  delay(100);
  digitalWrite(LED_CPU, LOW);
  delay(100);

  pinMode(LED_CPU, OUTPUT);
  pinMode(BRAKE_PEDAL_POS_PIN, INPUT);
  pinMode(ACR_PEDAL_POS_PIN, INPUT);
  pinMode(SUSP_RIGHT_PIN, INPUT);
  pinMode(SUSP_LEFT_PIN, INPUT);
  //pinMode(ACC_PIN, INPUT);
}

void taskScheduler(void)
{
  if (tmrSusp_Enable)
  {
    tmrSusp_Count++;
    if (tmrSusp_Count >= TMR_SUSP / TMR_BASE)
    {
      tmrSusp_Count = 0;
      tmrSusp_Overflow = true;
    }
  }

  if (tmrAcc_Enable)
  {
    tmrAcc_Count++;
    if (tmrAcc_Count >= TMR_ACC / TMR_BASE)
    {
      tmrAcc_Count = 0;
      tmrAcc_Overflow = true;
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

  if (tmrBlink_Enable)
  {
    tmrBlink_Count++;
    if (tmrBlink_Count >= TMR_BLINK / TMR_BASE)
    {
      tmrBlink_Count = 0;
      tmrBlink_Overflow = true;
    }
  }
  else
  {
    tmrBlink_Count++;
    if (tmrBlink_Count >= 10 * TMR_BLINK / TMR_BASE)
    {
      tmrBlink_Count = 0;
      tmrBlink_Overflow = true;
    }
  }
}

void taskBlink(void)
{
  if (tmrBlink_Overflow)
  {
    digitalWrite(LED_CPU, estadoLed);
    estadoLed != estadoLed;
    tmrBlink_Overflow = false;
  }
}

// ACELERÔMETRO 01
void taskAcc(void)
{

  Wire.beginTransmission(1);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU1, 14, true); // Solicita os dados ao sensor

  // Armazena o valor dos sensores nas variaveis correspondentes
  AccX = Wire.read() << 8 | Wire.read(); //0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AccY = Wire.read() << 8 | Wire.read(); //0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AccZ = Wire.read() << 8 | Wire.read(); //0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Temp = Wire.read() << 8 | Wire.read(); //0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyrX = Wire.read() << 8 | Wire.read(); //0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyrY = Wire.read() << 8 | Wire.read(); //0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyrZ = Wire.read() << 8 | Wire.read(); //0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  // Imprime na Serial os valores obtidos
  /* Alterar divisão conforme fundo de escala escolhido:
      Acelerômetro
      +/-2g = 16384
      +/-4g = 8192
      +/-8g = 4096
      +/-16g = 2048

      Giroscópio
      +/-250°/s = 131
      +/-500°/s = 65.6
      +/-1000°/s = 32.8
      +/-2000°/s = 16.4
  */

  Serial.print(AccX / 2048);
  Serial.print(" ");
  Serial.print(AccY / 2048);
  Serial.print(" ");
  Serial.println(AccZ / 2048);

  Acc.data[0] = (int(AccX)&0xFF << 8);
  Acc.data[1] = AccX;
  Acc.data[2] = (int(AccY)&0xFF << 8);
  Acc.data[3] = AccY;
  Acc.data[4] = (int(AccZ)&0xFF << 8);
  Acc.data[5] = AccZ;

  if(mcp2515.sendMessage(&Acc)!=MCP2515::ERROR::ERROR_OK){}

  tmrAcc_Overflow = false;


}

void taskSusp()
{
  if(tmrSusp_Overflow)
  {
    int position = analogRead(SUSP_RIGHT_PIN);

    position = map(position, 0, 1023, 0, 100);
    
    Susp.data[0] = position&0xFF;
    
    if(mcp2515.sendMessage(&Susp)!=MCP2515::ERROR::ERROR_OK)
    {
    
    }
  }
  
  tmrSusp_Overflow = false;
}

void taskACRPedalPos()
{
  if(tmrACRPedalPos_Overflow)
  {

    int position = analogRead(ACR_PEDAL_POS_PIN);

    position = map(position, 0, 1023, 0, 100);

    ACRPedalPos.data[0] = position&0xFF;

    if(mcp2515.sendMessage(&ACRPedalPos)!=MCP2515::ERROR::ERROR_OK)
    {
    
    }
  }
  tmrACRPedalPos_Overflow = false;
}

void taskBrakePedalPos()
{
  if(tmrBrakePedalPos_Overflow)
  {

    int position = analogRead(BRAKE_PEDAL_POS_PIN);

    position = map(position, 0, 1023, 0, 100);
 
    BrakePedalPos.data[0] = position&0xFF;

    if(mcp2515.sendMessage(&BrakePedalPos)!=MCP2515::ERROR::ERROR_OK)
    {
    
    }
  }
  tmrBrakePedalPos_Overflow = false;
}
