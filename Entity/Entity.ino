#include <StackArray.h>
#include <math.h>

//////////////////////////////////////////////LCD////////////////////////////////////////////////
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 16, 2);
int lastMillis, actualMillis;

//////////////////////////////////////////////LEDs////////////////////////////////////////////////
//#define ledRight 13
//#define ledLeft 12
#define ledRight 28
#define ledLeft 29

/////////////////////////////////////////////MOTORS///////////////////////////////////////////////
#define motorL1 3 //Forward
#define motorL2 2
#define motorR1 5 
#define motorR2 4 //Forward
double velGenDer = 80;
double velGenIzq = 80;
double velGenDerBack = 45;
double velGenIzqBack = 45;

///////////////////////////////////////////ULTRASONICS////////////////////////////////////////////
#include <NewPing.h>
//#define echoRight 4
//#define trigRight 5
//#define echoLeft 7
//#define trigLeft 8
//#define echoFront 3
//#define trigFront 2  
#define echoRight 25
#define trigRight 24
#define echoLeft 27
#define trigLeft 26
#define echoFront 22
#define trigFront 23  
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
//#define BotonColores 38
//#define LED_red 22
//#define LED_blue 23
//#define LED_green 24
//#define S0 34
//#define S1 35
//#define S2 53
//#define S3 52
//#define sensorOut 51
//double r = 0, g = 0, b = 0;
//const int num_col = 8;
//const int range = 7;
//int color_position;           //  0        1       2       3         4          5          6        7   
//String color_names[num_col] = {"blanco", "rosa", "rojo", "azul", "naranja", "amarillo", "negro", "verde"};
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
  pinMode(ledLeft,OUTPUT);
  pinMode(ledRight,OUTPUT);
  pinMode(echoRight, INPUT);
  pinMode(trigRight, OUTPUT);
  pinMode(echoLeft, INPUT);
  pinMode(trigLeft, OUTPUT);
  pinMode(echoFront, INPUT);
  pinMode(trigFront, OUTPUT);
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

//  rightPriotity(ultraFront, ultraRight, ultraLeft);

  mazeAlgorithm();
//  writeStringLCD("HOLA", 0, 1);
  
//  filtrateDistances(ultraFront, ultraRight, ultraLeft);
//  Serial.println(getBitWithValues(bits));

//  readPosition(bno, event, mpu, 'B');
//  filtrateDistances(ultraFront, ultraRight, ultraLeft);
//  oneStep(ultraFront, ultraRight, ultraLeft, 35);
//  spinPID(bno, event, mpu, 90);
//backPID(bno, event, mpu);


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
