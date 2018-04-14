#include <StackArray.h>
#include <math.h>

//////////////////////////////////////////////LCD////////////////////////////////////////////////
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 16, 2);
int lastMillis, actualMillis;

//////////////////////////////////////////////LEDs////////////////////////////////////////////////
//UNO
//#define ledRight 13
//#define ledLeft 12
//MEGA
#define ledRed 13
#define ledGreen 12
#define ledBlue 45
//PLACA
//#define ledRed 53
//#define ledGreen 47
//#define ledBlue 45

/////////////////////////////////////////////MOTORS///////////////////////////////////////////////
//UNO
#define motorL1 3
#define motorL2 2
#define motorR1 5 
#define motorR2 4 //Forward
//PLACA
//#define M2Back 2 //Back
//#define M2Front 3  //Front
//#define M2PWM 4  //PWM
//#define M1Back 5 //Back
//#define M1Front 6  //Front
//#define M1PWM 7  //PWM
//#define M3Back 48 //Back
//#define M3Front 46  //Front
//#define M3PWM 8  //PWM
//#define M4Back 52 //Back
//#define M4Front 50  //Front
//#define M4PWM 9  //PWM
double velGenDer = 90;
double velGenIzq = 75;
double velGenDerBack = 45;
double velGenIzqBack = 45;

/////////////////////////////////////////////ENCODERS///////////////////////////////////////////////
#define limitSwitchIzq A9
#define limitSwitchDer A3

/////////////////////////////////////////////ENCODERS///////////////////////////////////////////////
#define encoderM1Front 13
#define encoderM1Back 12
#define encoderM2Front A13
#define encoderM2Back A12
#define encoderM3Front 11
#define encoderM3Back 10
#define encoderM4Front A15
#define encoderM4Back A14

///////////////////////////////////////////ULTRASONICS////////////////////////////////////////////
#include <NewPing.h>
//UNO
//#define echoRight 4
//#define trigRight 5
//#define echoLeft 7
//#define trigLeft 8
//#define echoFront 3
//#define trigFront 2  
//MEGA
#define echoRight 24
#define trigRight 25
#define echoLeft 26
#define trigLeft 27
#define echoFront 22
#define trigFront 23  
//PLACA
//#define echoRight 25
//#define trigRight 23
//#define echoLeft 41
//#define trigLeft 39
//#define echoFront 33
//#define trigFront 31
bool special, ultraNegativoSide;
double stepDistance = 24, backStepDistance = 34;
double MAX_DISTANCE = 250;  //Prevents from waiting too long on pulseIn()
NewPing pingFront(trigFront, echoFront, MAX_DISTANCE);
NewPing pingRight(trigRight, echoRight, MAX_DISTANCE);
NewPing pingLeft(trigLeft, echoLeft, MAX_DISTANCE);
String bits="000000";

///////////////////////////////////////////BNO///////////////////////////////////////////////////
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
Adafruit_BNO055 bno = Adafruit_BNO055(55);
sensors_event_t event; 

///////////////////////////////////////////IMU///////////////////////////////////////////////////
#include <MPU6050.h>
MPU6050 mpu;
Vector rawGyro;
Vector normGyro;

///////////////////////////////////////////PID///////////////////////////////////////////////////
#include <utility/imumaths.h>
#include <PID_v1.h>
double leftAlignKp=4, leftAlignKi=0, leftAlignKd=0;
double leftTurnKp=2, leftTurnKi=0, leftTurnKd=0;
double leftConsKp=7.3, leftConsKi=0, leftConsKd=0;
double leftGenKp=leftConsKp, leftGenKi=leftConsKi, leftGenKd=leftConsKd;
double leftError=0;
double rightAlignKp=4, rightAlignKi=0, rightAlignKd=0;
double rightTurnKp=2, rightTurnKi=0, rightTurnKd=0;
double rightConsKp=9, rightConsKi=0, rightConsKd=0;
double rightGenKp=rightConsKp, rightGenKi=rightConsKi, rightGenKd=rightConsKd;
double rightError=0;
double Offset, Setpoint, fakeSetpoint, leftOutput, rightOutput, Input, rawInput, fakeInput, lastSetpoint;
double outputDifference = 5;
PID leftPID(&fakeInput, &leftOutput, &fakeSetpoint, leftGenKp, leftGenKi, leftGenKd, DIRECT); // (Values>0)
PID rightPID(&fakeInput, &rightOutput, &fakeSetpoint, rightGenKp, rightGenKi, rightGenKd, REVERSE); // (Values<0)

//////////////////////////////////////////////ALGORITHM/////////////////////////////////////////////////
const int mazeSize=9;
int actualRow=0, actualCol=0;
char comeFrom='S';
String maze[mazeSize][mazeSize];

///////////////////////////////////////////////COLOR///////////////////////////////////////////////////
#define LED_red 22
#define LED_blue 23
#define LED_green 24
#define S0 41
#define S1 40
#define S2 37
#define S3 40
#define sensorOut 36
double r = 0, g = 0, b = 0;
const int num_col = 4;
const int range = 7;
int color_position;           //  0        1       2       3    
String color_names[num_col] = {"rojo", "azul", "verde", "negro"};
//color color_position_arr[num_col];

///////////////////////////////////////////ULTRA KALMAN FILTER///////////////////////////////////////////
struct UltraKalman{
  UltraKalman(){
    varSensor = 3e-6; //Variance of sensor. The LESS, the MORE it looks like the raw input.
    varProcess = 1e-7; 
    P = 1.0;
    Pc = 0.0;
    G = 0.0;
    Xp = 0.0;
    Zp = 0.0;
    Xe = 0.0;
    duration = 0.0;
    distance = 0.0;
    side = false;
  }
  float varSensor; //Variance of sensor. The LESS, the MORE it looks like the raw input
  float varProcess; // The greater the variance, faster the sensor response
  float P;
  float Pc;
  float G;
  float Xp;
  float Zp;
  float Xe;
  double duration;
  double distance;
  bool side;
};
UltraKalman ultraRight;
UltraKalman ultraLeft;
UltraKalman ultraFront;

///////////////////////////////////////////xBNO KALMAN FILTER///////////////////////////////////////////
float xBNOVarSensor = 10e-6; //Variance of sensor. The LESS, the MORE it looks like the raw input
float xBNOVarProcess = 1e-7; // The greater the variance, faster the sensor response
float xBNOP = 1.0;
float xBNOPc = 0.0;
float xBNOG = 0.0;
float xBNOXp = 0.0;
float xBNOZp = 0.0;
float xBNOXe = 0.0; // Xe

///////////////////////////////////////////xIMU KALMAN FILTER///////////////////////////////////////////
float xIMUVarSensor = 10e-4; //Variance of sensor. The LESS, the MORE it looks like the raw input
float xIMUVarProcess = 1e-7; // The greater the variance, faster the sensor response
float xIMUP = 1.0;
float xIMUPc = 0.0;
float xIMUG = 0.0;
float xIMUXp = 0.0;
float xIMUZp = 0.0;
float xIMUXe = 0.0; // Xe

/////////////////////////////////////////////SETUP////////////////////////////////////////////////
void setup() {
  Serial.begin(9600);
//  Serial.begin(115200);
  pinMode(motorR1,OUTPUT);
  pinMode(motorR2,OUTPUT);
  pinMode(motorL1,OUTPUT);
  pinMode(motorL2,OUTPUT);
//  pinMode(M1Back,OUTPUT);
//  pinMode(M1Front,OUTPUT);
//  pinMode(M1PWM,OUTPUT);
//  pinMode(M2Back,OUTPUT);
//  pinMode(M2Front,OUTPUT);
//  pinMode(M2PWM,OUTPUT);
//  pinMode(M3Back,OUTPUT);
//  pinMode(M3Front,OUTPUT);
//  pinMode(M3PWM,OUTPUT);
//  pinMode(M4Back,OUTPUT);
//  pinMode(M4Front,OUTPUT);
//  pinMode(M4PWM,OUTPUT);
  pinMode(ledGreen,OUTPUT);
  pinMode(ledRed,OUTPUT);
  pinMode(ledBlue,OUTPUT);  
  pinMode(limitSwitchDer,INPUT);
  pinMode(limitSwitchIzq,INPUT);
  pinMode(encoderM1Front,INPUT);
  pinMode(encoderM1Back,INPUT);
  pinMode(encoderM2Front,INPUT);
  pinMode(encoderM2Back,INPUT);
  pinMode(encoderM3Front,INPUT);
  pinMode(encoderM3Back,INPUT);
  pinMode(encoderM4Front,INPUT);
  pinMode(encoderM4Back,INPUT);  
  pinMode(trigRight, OUTPUT);
  pinMode(echoLeft, INPUT);
  pinMode(trigLeft, OUTPUT);
  pinMode(echoFront, INPUT);
  pinMode(trigFront, OUTPUT);
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(sensorOut, INPUT);
  digitalWrite(S0,HIGH);
  digitalWrite(S1,LOW);    
//// MPU
//  Serial.println("Initializing MPU6050...");
//  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
//  {
//    Serial.println("MPU6050 has been initiated!");
//    delay(500);
//  }
//  mpu.calibrateGyro();
//  mpu.setThreshold(3);
//// BNO
  if(!bno.begin())  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or L2C ADDR!");
    while(1);
  } 
  else
    Serial.println("BNO STARTED");
//  setCal();
//  delay(3000);
//  setFakeSetpoint();
  bno.setExtCrystalUse(true);
//// PID
  Setpoint=0; //  Set reference point to 0
  leftPID.SetSampleTime(1); // Set Sample Time
  leftPID.SetMode(AUTOMATIC);
  leftPID.SetOutputLimits(0, 255); 
  rightPID.SetSampleTime(1); // Set Sample Time
  rightPID.SetMode(AUTOMATIC);
  rightPID.SetOutputLimits(0, 255);
//// LCD
//  lcd.backlight();
  lcd.begin();
  delay(500);
}

void loop(){
//   forwardPID(bno, event, mpu);
//   filtrateDistances(ultraFront, ultraRight, ultraLeft);
//   if(ultraFront.side){
//       spinPID(bno, event, mpu, 90, false);
//       filtrateDistances(ultraFront, ultraRight, ultraLeft);      
//   }

  rightPriotity(ultraFront, ultraRight, ultraLeft);

//  mazeAlgorithm();
//  writeStringLCD("HOLA", 0, 1);
  
//  filtrateDistances(ultraFront, ultraRight, ultraLeft);
//  Serial.println(getBitWithValues(bits));

//  readPosition(bno, event, mpu, 'B');
//  filtrateDistances(ultraFront, ultraRight, ultraLeft);
//  oneStep(ultraFront, ultraRight, ultraLeft, 35);
//  spinPID(bno, event, mpu, 90);
//backPID(bno, event, mpu);

//go();
//digitalWrite(ledGreen, HIGH);

//  filtrateDistances(ultraFront, ultraRight, ultraLeft);
//  Serial.print(ultraLeft.distance);
//  Serial.print("\t");
//  Serial.print(ultraFront.distance);
//  Serial.print("\t");
//  Serial.println(ultraRight.distance);

//  xBNO_RawKalman(bno, event);
//  ultra_RawKalman(ultraFront, pingFront);
//  ultra_RawKalman(ultraLeft, pingLeft);
//  ultra_RawKalman(ultraRight, pingRight);
}
