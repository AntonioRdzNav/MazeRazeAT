#include <StackArray.h>
#include <math.h>

//////////////////////////////////////////////LEDs////////////////////////////////////////////////
#define ledRight 13
#define ledLeft 12

/////////////////////////////////////////////MOTORS///////////////////////////////////////////////
#define motorL1 9 //Forward
#define motorL2 6
#define motorR1 11 //Forward
#define motorR2 10
double velGenDer = 85;
double velGenIzq = 85;
double velGenDerBack = 55;
double velGenIzqBack = 55;

///////////////////////////////////////////ULTRASONICS////////////////////////////////////////////
#include <NewPing.h>
#define echoRight 4
#define trigRight 5
#define echoLeft 7
#define trigLeft 8
#define echoFront 3
#define trigFront 2  
bool special, ultraNegativoSide;
double stepDistance = 24, backStepDistance = 34;
double MAX_DISTANCE = 250;  //Prevents from waiting too long on pulseIn()
NewPing pingFront(trigFront, echoFront, MAX_DISTANCE);
NewPing pingRight(trigRight, echoRight, MAX_DISTANCE);
NewPing pingLeft(trigLeft, echoLeft, MAX_DISTANCE);

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
double leftTurnKp=1.9, leftTurnKi=0, leftTurnKd=0;
double leftConsKp=5.9, leftConsKi=0, leftConsKd=0;
double leftGenKp=leftConsKp, leftGenKi=leftConsKi, leftGenKd=leftConsKd;
double leftError=0;
double rightAlignKp=4, rightAlignKi=0, rightAlignKd=0;
double rightTurnKp=1.9, rightTurnKi=0, rightTurnKd=0;
double rightConsKp=5.2, rightConsKi=0, rightConsKd=0;
double rightGenKp=rightConsKp, rightGenKi=rightConsKi, rightGenKd=rightConsKd;
double rightError=0;
double Offset, Setpoint, fakeSetpoint, leftOutput, rightOutput, Input, rawInput, fakeInput, lastSetpoint;
double outputDifference = 5;
PID leftPID(&fakeInput, &leftOutput, &fakeSetpoint, leftGenKp, leftGenKi, leftGenKd, DIRECT); // (Values>0)
PID rightPID(&fakeInput, &rightOutput, &fakeSetpoint, rightGenKp, rightGenKi, rightGenKd, REVERSE); // (Values<0)

//////////////////////////////////////////////ALGORITHM/////////////////////////////////////////////////
const int mazeSize=9;
int actualRow=8, actualCol=6;
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

//  readPosition(bno, event, mpu, 'B');
//  filtrateDistances(ultraFront, ultraRight, ultraLeft);
//  oneStep(ultraFront, ultraRight, ultraLeft, 35);
//  spinPID(bno, event, mpu, 90);


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
