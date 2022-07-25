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
    
*/

//Definicao das portas
#define REAR_SUSP_POS_RIGHT_PIN A0
#define REAR_SUSP_POS_LEFT_PIN A1
#define VOLTAGE_SENSOR_PIN A2
#define CURRENT_SENSOR_PIN A3
#define LED_CPU 8

#define CAN_SCK 13 //Pino SCK da CAN
#define CAN_SO 12  //Pino SO da CAN
#define CAN_SI 11  //Pino SI da CAN
#define CAN_CS 10  //Pino CS da CAN


//Definicao do Timer
#define TMR_BASE 100000
#define TMR_ACC1 100000
#define TMR_ACC2 100000
#define TMR_REAR_SUSP_POS_RIGHT 100000
#define TMR_REAR_SUSP_POS_LEFT 100000
#define TMR_SPEED 100000
#define TMR_VOLTAGE_SENSOR 100000
#define TMR_CURRENT_SENSOR 100000

//Definicao CAN_ID
#define ACC1_CAN_ID 0x06
#define ACC2_CAN_ID 0x07
#define REAR_SUSP_POS_RIGHT_CAN_ID 0x08
#define REAR_SUSP_POS_LEFT_CAN_ID 0x09
#define SPEED_CAN_ID 0x0A
#define VOLTAGE_SENSOR_CAN_ID 0x0B
#define CURRENT_SENSOR_CAN_ID 0x0C

//Bibliotecas
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <TimerOne.h>
#include <EK305CAN.h>
#include <mcp2515.h>

//Prototipos de funcoes Setup
void setupCAN();
void setupInit();

//Prototipos de funcoes Task
void taskScheduler(void);
void taskRearSuspPosRight(void);
void taskRearSuspPosLeft(void);
void taskAcc1(void);
void taskAcc2(void);
void taskVoltageSensor(void);
void taskCurrentSensor(void);

float AccX, AccY, AccZ, Temp, GyrX, GyrY, GyrZ;

//Endereco dos Acc
const int MPU1 = 0x68;

//Variaveis globais de controle das tasks
bool tmrAcc1_Enable = false;
int tmrAcc1_Count = 0;
bool tmrAcc1_Overflow = false;

bool tmrAcc2_Enable = false;
int tmrAcc2_Count = 0;
bool tmrAcc2_Overflow = false;

bool tmrRearSuspPosRight_Enable = false;
int tmrRearSuspPosRight_Count = 0;
bool tmrRearSuspPosRight_Overflow = false;

bool tmrRearSuspPosLeft_Enable = false;
int tmrRearSuspPosLeft_Count = 0;
bool tmrRearSuspPosLeft_Overflow = false;

bool tmrVoltageSensor_Enable = false;
int tmrVoltageSensor_Count = 0;
bool tmrVoltageSensor_Overflow = false;

bool tmrCurrentSensor_Enable = false;
int tmrCurrentSensor_Count = 0;
bool tmrCurrentSensor_Overflow = false;

//Pacotes CAN
can_frame Acc1;
can_frame Acc2;
can_frame SpeedSensor;
can_frame VoltageSensor;
can_frame CurrentSensor;
can_frame RearSuspPosRight;
can_frame RearSuspPosLeft;

//Inicializacoes
MCP2515 mcp2515(CAN_CS);

void setup()
{
  setupCAN();
  setupInit();
}

void loop()
{
    void taskRearSuspPosRight();
    void taskRearSuspPosLeft();
    void taskAcc1();
    void taskAcc2();
    void taskCurrentSensor();
    void taskVoltageSensor();
}

void setupCAN()
{
  digitalWrite(LED_CPU, HIGH);
  CAN_Init(&mcp2515, CAN_500KBPS);
  digitalWrite(LED_CPU, LOW);


  Acc1.can_id = ACC1_CAN_ID;
  Acc2.can_id = ACC2_CAN_ID;
  RearSuspPosRight.can_id = REAR_SUSP_POS_RIGHT_CAN_ID;
  RearSuspPosLeft.can_id = REAR_SUSP_POS_LEFT_CAN_ID;
  VoltageSensor.can_id = VOLTAGE_SENSOR_CAN_ID;
  CurrentSensor.can_id = CURRENT_SENSOR_CAN_ID;

  Acc1.can_dlc = 7;
  Acc2.can_dlc = 7;
  RearSuspPosRight.can_dlc = 1;
  RearSuspPosLeft.can_dlc = 1;
  VoltageSensor.can_dlc = 1;
  CurrentSensor.can_dlc = 1;
}

void setupInit()
{
  Serial.begin(9600);
  pinMode(LED_CPU, OUTPUT);

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

  pinMode(REAR_SUSP_POS_RIGHT_PIN, INPUT);
  pinMode(REAR_SUSP_POS_LEFT_PIN, INPUT);
  pinMode(VOLTAGE_SENSOR_PIN, INPUT);
  pinMode(CURRENT_SENSOR_PIN, INPUT);

  tmrAcc1_Enable = false;
  tmrAcc2_Enable = false;
  tmrRearSuspPosRight_Enable = false;
  tmrRearSuspPosLeft_Enable = false;
  tmrVoltageSensor_Enable = false;
  tmrCurrentSensor_Enable = false;

  digitalWrite(LED_CPU, HIGH);
  delay(100);
  digitalWrite(LED_CPU, LOW);
  delay(100);
}

void taskScheduler(void)
{

  if(tmrAcc1_Enable){

    tmrAcc1_Count++;
    if(tmrAcc1_Count >= TMR_ACC1/TMR_BASE){
      tmrAcc1_Overflow = true;
      tmrAcc1_Count = 0;
    }

  }

  if(tmrAcc2_Enable){

    tmrAcc2_Count++;
    if(tmrAcc2_Count >= TMR_ACC2/TMR_BASE){
      tmrAcc2_Overflow = true;
      tmrAcc2_Count = 0;
    }

  }

  if(tmrRearSuspPosRight_Enable){

    tmrRearSuspPosRight_Count++;
    if(tmrRearSuspPosRight_Count >= TMR_REAR_SUSP_POS_RIGHT/TMR_BASE){
      tmrRearSuspPosRight_Overflow = true;
      tmrRearSuspPosRight_Count = 0;
    }

  }

  if(tmrRearSuspPosLeft_Enable){

    tmrRearSuspPosLeft_Count++;
    if(tmrRearSuspPosLeft_Count >= TMR_REAR_SUSP_POS_LEFT/TMR_BASE){
      tmrRearSuspPosLeft_Overflow = true;
      tmrRearSuspPosLeft_Count = 0;
    }

  }

  if(tmrVoltageSensor_Enable){

    tmrVoltageSensor_Count++;
    if(tmrVoltageSensor_Count >= TMR_VOLTAGE_SENSOR/TMR_BASE){
      tmrVoltageSensor_Overflow = true;
      tmrVoltageSensor_Count = 0;
    }

  }

  if(tmrCurrentSensor_Enable){

    tmrCurrentSensor_Count++;
    if(tmrCurrentSensor_Count >= TMR_CURRENT_SENSOR/TMR_BASE){
      tmrCurrentSensor_Overflow = true;
      tmrCurrentSensor_Count = 0;
    }

  }


}

void taskAcc1(){

  if(tmrAcc1_Overflow){
  Wire.beginTransmission(MPU1);
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

  // Imprime na Serial o valore obtido
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

  Acc1.data[0] = (int(AccX)&0xFF << 8);
  Acc1.data[1] = AccX;
  Acc1.data[2] = (int(AccY)&0xFF << 8);
  Acc1.data[3] = AccY;
  Acc1.data[4] = (int(AccZ)&0xFF << 8);
  Acc1.data[5] = AccZ;

  if(mcp2515.sendMessage(&Acc1)!=MCP2515::ERROR::ERROR_OK){}

  tmrAcc1_Overflow = false;


  }
}

void taskRearSuspPosRight(void){
  if(tmrRearSuspPosRight_Overflow){
    int position = analogRead(REAR_SUSP_POS_RIGHT_PIN);

    position = map(position, 0, 1023, 0, 100);

    if(mcp2515.sendMessage(&RearSuspPosRight)!=MCP2515::ERROR::ERROR_OK){}

  }
  tmrRearSuspPosRight_Overflow = false;
}

void taskRearSuspPosLeft(void){
  if(tmrRearSuspPosLeft_Overflow){
    int position = analogRead(REAR_SUSP_POS_LEFT_PIN);

    position = map(position, 0, 1023, 0, 100);

    if(mcp2515.sendMessage(&RearSuspPosLeft)!=MCP2515::ERROR::ERROR_OK){}
  }
  tmrRearSuspPosLeft_Overflow = false;
}
