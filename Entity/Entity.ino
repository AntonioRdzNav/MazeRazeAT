//////////////////////////////////////////////LEDs////////////////////////////////////////////////
#define ledRight 24
#define ledLeft 25

/////////////////////////////////////////////MOTORS///////////////////////////////////////////////
#define motorL1 5 //Forward
#define motorL2 4
#define motorR1 2 //Forward
#define motorR2 3
double velGen = 65;

///////////////////////////////////////////ULTRASONICS////////////////////////////////////////////
#include <NewPing.h>
#define echoRight 26
#define trigRight 27
#define echoLeft 28
#define trigLeft 29
#define echoFront 30
#define trigFront 31  
bool special;
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
double leftAggKp=0, leftAggKi=0, leftAggKd=0;
double leftConsKp=3, leftConsKi=0, leftConsKd=0;
double leftError=0;
double rightAggKp=0, rightAggKi=0, rightAggKd=0;
double rightConsKp=3, rightConsKi=0, rightConsKd=0;
double rightError=0;
double Setpoint, leftOutput, rightOutput, Input;
double outputDifference = 5;
PID leftPID(&Input, &leftOutput, &Setpoint, leftConsKp, leftConsKi, leftConsKd, DIRECT); // (Values>0)
PID rightPID(&Input, &rightOutput, &Setpoint, rightConsKp, rightConsKi, rightConsKd, REVERSE); // (Values<0)

///////////////////////////////////////////ULTRA KALMAN FILTER///////////////////////////////////////////
struct UltraKalman{
  UltraKalman(){
    varSensor = 0.1e-3; //Variance of sensor. The LESS, the MORE it looks like the raw input.
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
}

void loop(){
//  Setpoint = calculateNewSetpoint(-90);
  
//  forwardPID(bno, event, mpu);
//  ledsPID();
//  filtrateDistances(ultraFront, ultraRight, ultraLeft);
//  Serial.print(ultraFront.Xe);
//  Serial.print("   ");
//  Serial.println(xBNOXe);
//  if(ultraFront.side){
      Setpoint = calculateNewSetpoint(-90);
      turnLeftPID(bno, event, mpu);
      stop();
      delay(5000);
//  }
  Serial.println(Setpoint);

// Serial.print(leftOutput);
// Serial.print("\t\t");
// Serial.print(rightOutput);
// Serial.print("\t\t");
// Serial.println(outputDifference);
// turnLeftPID(bno, event, mpu);
// delay(2000);
// ledsPID();
  
//  ultraFront_RawKalman(ultraFront);
//  ultraLeft_RawKalman(ultraLeft);
//  ultraRight_RawKalman(ultraRight);
}
