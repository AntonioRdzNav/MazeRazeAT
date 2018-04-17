#include <StackArray.h>
#include <math.h>

///////////////////////////////////////////////COLOR///////////////////////////////////////////////////
//#define S0 41
//#define S1 42
//#define S2 37
//#define S3 40
//#define sensorOut 36
#define sensorOut 32
#define S2 30
#define S3 28
#define S1 38
#define S0 36
#define OE 40
bool colorRedDetected=false, colorGreenDetected=false, colorBlackDetected=false;
double r = 0, g = 0, b = 0;
const int num_col = 4;
const int range = 15;
int color_position;           //  0        1       2       3    
String color_names[num_col] = {"rojo", "azul", "verde", "negro"};
struct color{
  String nombre;
  double red;
  double green;
  double blue;
};
color color_position_arr[num_col];

//////////////////////////////////////////////LCD////////////////////////////////////////////////
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 16, 2);
int lastMillis, actualMillis;

//////////////////////////////////////////////LEDs////////////////////////////////////////////////
//UNO
//#define ledRight 13
//#define ledLeft 12
//gregMovil
//#define ledRed 43
//#define ledGreen 28
//#define ledBlue 29
//AT
#define ledRed 53
#define ledGreen 47
#define ledBlue 45

/////////////////////////////////////////////MOTORS///////////////////////////////////////////////
//gregMovil
//#define motorL1 3
//#define motorL2 2
//#define motorR1 5 
//#define motorR2 4 //Forward
//AT
//#define M2Back 3 //Back
//#define M2Front 2  //Front
//#define M2PWM 4  //PWM
//#define M1Back 5 //Back
//#define M1Front 6  //Front
//#define M1PWM 7  //PWM
//#define motorL_PWM 8  //PWM
//#define motorD_PWM 9  //PWM
//#define motorL1 46 //Back
//#define motorL2 48  //Front
//#define motorR1 50 //Back
//#define motorR2 52  //Front
#define motorL_PWM 7  //PWM
#define motorD_PWM 4  //PWM
#define motorL1 5 //Back
#define motorL2 6  //Front
#define motorR1 2 //Back
#define motorR2 3  //Front

double velGenDer = 140;
double velGenIzq = 140-25;
double velGenDerBack = 100;
double velGenIzqBack = 100-25;

/////////////////////////////////////////////ENCODERS///////////////////////////////////////////////
#define limitSwitchIzq A9
#define limitSwitchDer A3

/////////////////////////////////////////////ENCODERS///////////////////////////////////////////////
//AT
//#define encoderM1Front 13
//#define encoderM1Back 12
//#define encoderM2Front A13
//#define encoderM2Back A12
//#define encoderM3Front 11
//#define encoderM3Back 10
//#define encoderM4Front A15
//#define encoderM4Back A14

///////////////////////////////////////////ULTRASONICS////////////////////////////////////////////
#include <NewPing.h>
//UNO
//#define echoRight 4
//#define trigRight 5
//#define echoLeft 7
//#define trigLeft 8
//#define echoFront 3
//#define trigFront 2  
//GregMovil
//#define echoRight 25
//#define trigRight 24
//#define echoLeft 27
//#define trigLeft 26
//#define echoFront 22
//#define trigFront 23  
//AT
#define echoRight 25
#define trigRight 23
#define echoLeft 41
#define trigLeft 39
#define echoFront 31
#define trigFront 29
bool special, ultraNegativoSide;
double stepDistance = 25, backStepDistance = 36;
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
double leftAlignKp=5.5, leftAlignKi=0, leftAlignKd=0;
double leftTurnKp=3.4, leftTurnKi=0, leftTurnKd=0;
double leftConsKp=16, leftConsKi=0, leftConsKd=0;
double leftGenKp=leftConsKp, leftGenKi=leftConsKi, leftGenKd=leftConsKd;
double leftError=0;
double rightAlignKp=5.5, rightAlignKi=0, rightAlignKd=0;
double rightTurnKp= 3.4, rightTurnKi=0, rightTurnKd=0;
double rightConsKp=16, rightConsKi=0, rightConsKd=0;
double rightGenKp=rightConsKp, rightGenKi=rightConsKi, rightGenKd=rightConsKd;
double rightError=0;
double Offset, Setpoint, fakeSetpoint, leftOutput, rightOutput, Input, rawInput, fakeInput, lastSetpoint;
double outputDifference = 5;
PID leftPID(&fakeInput, &leftOutput, &fakeSetpoint, leftGenKp, leftGenKi, leftGenKd, DIRECT); // (Values>0)
PID rightPID(&fakeInput, &rightOutput, &fakeSetpoint, rightGenKp, rightGenKi, rightGenKd, REVERSE); // (Values<0)
bool LARC = false;

//////////////////////////////////////////////ALGORITHM/////////////////////////////////////////////////
const int mazeSize=9;
int actualRow=0, actualCol=0;
char comeFrom='S';
String maze[mazeSize][mazeSize];

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
  pinMode(motorL_PWM,OUTPUT);
  pinMode(motorD_PWM,OUTPUT);
  pinMode(ledGreen,OUTPUT);
  pinMode(ledRed,OUTPUT);
  pinMode(ledBlue,OUTPUT);    
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
//  AT
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
//  pinMode(limitSwitchDer,INPUT);
//  pinMode(limitSwitchIzq,INPUT);
//  pinMode(encoderM1Front,INPUT);
//  pinMode(encoderM1Back,INPUT);
//  pinMode(encoderM2Front,INPUT);
//  pinMode(encoderM2Back,INPUT);
//  pinMode(encoderM3Front,INPUT);
//  pinMode(encoderM3Back,INPUT);
//  pinMode(encoderM4Front,INPUT);
//  pinMode(encoderM4Back,INPUT);
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
//   if(ultraNegativoSide){
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
//  digitalWrite(ledGreen, HIGH);

//  filtrateDistances(ultraFront, ultraRight, ultraLeft);
//  Serial.print(ultraLeft.distance);
//  Serial.print("\t");
//  Serial.print(ultraFront.distance);
//  Serial.print("\t");
//  Serial.println(ultraRight.distance);

//  firstChallenge();

//  calibrarColores(0);
//  while(1){
//    readColor(r, g, b);
//    Serial.print(r);
//    Serial.print("\t");
//    Serial.print(g);
//    Serial.print("\t");
//    Serial.println(b); 
//  } 

//  xBNO_RawKalman(bno, event);
//  ultra_RawKalman(ultraFront, pingFront);
//  ultra_RawKalman(ultraLeft, pingLeft);
//  ultra_RawKalman(ultraRight, pingRight);

  //  0        1       2       3    
//String color_names[num_col] = {"rojo", "azul", "verde", "negro"};

//  calibrarColores(0);
//  if(currentColor() == 0)
//    Serial.println("Rojo");
//  if(currentColor() == 1)
//    Serial.println("Azul");
//  if(currentColor() == 2)
//    Serial.println("Verde");
//  if(currentColor() == 3)
//    Serial.println("Negro");
}
