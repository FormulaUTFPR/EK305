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

//Acc Variables -- VARIÁVEIS NOVAS -- ATUALIZAR
int acc_error = 0;                          //We use this variable to only calculate once the Acc data error
float rad_to_deg = 180 / 3.141592654;       //This value is for pasing from radians to degrees values
float Acc_rawX, Acc_rawY, Acc_rawZ;         //Here we store the raw data read
float Acc_angle_x, Acc_angle_y;             //Here we store the angle value obtained with Acc data
float Acc_angle_error_x, Acc_angle_error_y; //Here we store the initial Acc data error

float Total_angle_x, Total_angle_y;

//Gyro Variables
float elapsedTime, time, timePrev;        //Variables for time control
int gyro_error = 0;                       //We use this variable to only calculate once the gyro data error
float Gyr_rawX, Gyr_rawY, Gyr_rawZ;       //Here we store the raw data read
float Gyro_angle_x, Gyro_angle_y;         //Here we store the angle value obtained with Gyro data
float Gyro_raw_error_x, Gyro_raw_error_y; //Here we store the initial gyro data error

// AQUI ACABAM AS VARIÁVEIS NOVAS

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
void setupWIRE()
{
  Wire.begin();                 //begin the wire comunication
  Wire.beginTransmission(0x68); //begin, Send the slave adress (in this case 68)
  Wire.write(0x6B);             //make the reset (place a 0 into the 6B register)
  Wire.write(0x00);
  Wire.endTransmission(false); //end the transmission
  //Gyro config
  Wire.beginTransmission(0x68); //begin, Send the slave adress (in this case 68)
  Wire.write(0x1B);             //We want to write to the GYRO_CONFIG register (1B hex)
  Wire.write(0x10);             //Set the register bits as 00010000 (1000dps full scale)
  Wire.endTransmission(false);  //End the transmission with the gyro
  //Acc config
  Wire.beginTransmission(0x68); //Start communication with the address found during search.
  Wire.write(0x1C);             //We want to write to the ACCEL_CONFIG register
  Wire.write(0x10);             //Set the register bits as 00010000 (+/- 8g full scale range)
  Wire.endTransmission(true);
}

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
  tmrACRPedalPos_Enable = true;
  tmrBrakePedalPos_Enable = true;
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
  if (tmrAccOverflow)
  {
    Wire.beginTransmission(0x68);
    Wire.write(0x3B); //Ask for the 0x3B register- correspond to AcX
    Wire.endTransmission(false);
    Wire.requestFrom(0x68, 6, true);

    Acc_rawX = (Wire.read() << 8 | Wire.read()) / 4096.0; //each value needs two registres
    Acc_rawY = (Wire.read() << 8 | Wire.read()) / 4096.0;
    Acc_rawZ = (Wire.read() << 8 | Wire.read()) / 4096.0;

    Wire.beginTransmission(0x68); //begin, Send the slave adress (in this case 68)
    Wire.write(0x43);             //First adress of the Gyro data
    Wire.endTransmission(false);
    Wire.requestFrom(0x68, 6, true); //We ask for just 4 registers

    Gyr_rawX = Wire.read() << 8 | Wire.read(); //Once again we shif and sum
    Gyr_rawY = Wire.read() << 8 | Wire.read();
    Gyr_rawZ = Wire.read() << 8 | Wire.read();

    int iAcx2 = int(Acc_rawX); // Nova escala de 0 a 200
    int iAcy2 = int(Acc_rawY); // Essa escala se refere a -1 a 1 G
    int iAcz2 = int(Acc_rawZ); // Aproximadamente 0 se refere a -1 G.

    int iGyx2 = int(Gyr_rawX); // Nova escala de 0 a 250.
    int iGyy2 = int(Gyr_rawY); // Essa escala se refere a -250 a 250 graus/s.
    int iGyz2 = int(Gyr_rawZ); // Aproximadamente 0 se refere a -250 graus/s.

    Acc.data[0] = (iAcx2 >> 8) & 0xFF;
    Acc.data[1] = iAcx2 & 0x0F;
    Acc.data[2] = (iAcy2 >> 8) & 0xFF;
    Acc.data[3] = iAcy2 & 0x0F;
    Acc.data[4] = (iAcz2 >> 8) & 0xFF;
    Acc.data[5] = iAcz2 & 0x0F;

    Gyro.data[0] = (iGyx2 >> 8) & 0xFF;
    Gyro.data[1] = iGyx2 & 0x0F;
    Gyro.data[2] = (iGyy2 >> 8) & 0xFF;
    Gyro.data[3] = iGyy2 & 0x0F;
    Gyro.data[4] = (iGyz2 >> 8) & 0xFF;
    Gyro.data[5] = iGyz2 & 0x0F;

    tmrAccOverflow = false;

    if (mcp2515.sendMessage(&Acc) != MCP2515::ERROR::ERROR_OK)
    { // envia os dados de um CAN_Frame na CAN
      tmrBlinkEnable = false;
    }
    if (mcp2515.sendMessage(&Gyro) != MCP2515::ERROR::ERROR_OK)
    { // envia os dados de um CAN_Frame na CAN
      tmrBlinkEnable = false;
    }
  }
}

void taskSusp()
{
  if(tmrSuspOverflow)
  {
    int position = analogRead(PIN_SUSP_DIREITA); //Le o valor de posicao do pedal de freio

    Serial.println(position);

    position = map(position, 0, 1023, 100, 0);

    Suspensao.data[0] = position&0xFF;

    if(mcp2515.sendMessage(&Suspensao)!=MCP2515::ERROR::ERROR_OK)
    {
    
    }
  }
  tmrSuspOverflow = false;
}

void taskACRPedalPos()
{
  if(tmrACRPedalPos_Overflow)
  {
    int position = analogRead(PIN_ACR_PEDAL_POS); //Le o valor de posicao do pedal de freio

    position = map(position, 0, 1023, 100, 0);

    ACR_Pedal_Pos.data[0] = position&0xFF;

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
    int position = analogRead(PIN_BRAKE_PEDAL_POS); //Le o valor de posicao do pedal de freio

    position = map(position, 0, 1023, 100, 0);

    Brake_Pedal_Pos.data[0] = position&0xFF;

    if(mcp2515.sendMessage(&Brake_Pedal_Pos)!=MCP2515::ERROR::ERROR_OK)
    {
    
    }
  }
  tmrBrakePedalPos_Overflow = false;
}

//CODIGO ANTIGO DO PEDAL DO ACELERADOR, DO PEDAL DE FREIO E DA POSICAO DA SUSPENSAO(APAGAR SE CODIGO NOVO FUNCIONAR)

/*
// SUSPENSAO FRONTAL
void taskSusp(void)
{
  if (tmrSuspOverflow)
  {
    unsigned int sender1 = analogRead(PIN_SUSP_DIREITA);

    unsigned int sender2 = analogRead(PIN_SUSP_ESQUERDA);

    Suspensao.data[0] = (sender1 >> 8) & 0xFF;
    Suspensao.data[1] = sender1 & 0xFF;

    Suspensao.data[2] = (sender2 >> 8) & 0xFF;
    Suspensao.data[3] = sender2 & 0xFF;

    if (mcp2515.sendMessage(&Suspensao) != MCP2515::ERROR::ERROR_OK)
    { // envia os dados de um CAN_Frame na CAN
      tmrBlinkEnable = false;
    }

    tmrSuspOverflow = false;
  }
}

// PEDAIS
void taskPedal(void)
{
  if (tmrPedalOverflow)
  {
    unsigned int sender1 = analogRead(PIN_SUSP_DIREITA);

    unsigned int sender2 = analogRead(PIN_SUSP_ESQUERDA);

    Pedal.data[0] = (sender1 / 4) & 0xFF;

    Pedal.data[1] = (sender2 / 2) & 0xFF;

    if (mcp2515.sendMessage(&Pedal) != MCP2515::ERROR::ERROR_OK)
    { // envia os dados de um CAN_Frame na CAN
      tmrBlinkEnable = false;
    }

    tmrPedalOverflow = false;
  }
}
*/
