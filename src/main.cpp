#include <PS4Controller.h>
unsigned long lastTimeStamp = 0;
int maxSpeed = 255;
int minSpeed = maxSpeed * -1;

//Right motor
int enableRightMotor=22; 
int rightMotorPin1=16;
int rightMotorPin2=17;
//Left motor
int enableLeftMotor=23;
int leftMotorPin1=18;
int leftMotorPin2=19;

const int PWMFreq = 1000; /* 1 KHz */
const int PWMResolution = 8;
const int rightMotorPWMSpeedChannel = 4;
const int leftMotorPWMSpeedChannel = 5;


void setMotorDirections(int motorPin1, int motorPin2, int speed) {
  int pin1State = (speed > 0) ? HIGH : LOW;
  int pin2State = (speed < 0) ? HIGH : LOW;
  
  digitalWrite(motorPin1, pin1State);
  digitalWrite(motorPin2, pin2State);
}

void rotateMotor(int rightMotorSpeed, int leftMotorSpeed)
{
  setMotorDirections(rightMotorPin1, rightMotorPin2, rightMotorSpeed);
  setMotorDirections(leftMotorPin1, leftMotorPin2, leftMotorSpeed);
  
//   if (rightMotorSpeed < 0)
//   {
//     digitalWrite(rightMotorPin1,LOW);
//     digitalWrite(rightMotorPin2,HIGH);    
//   }
//   else if (rightMotorSpeed > 0)
//   {
//     digitalWrite(rightMotorPin1,HIGH);
//     digitalWrite(rightMotorPin2,LOW);      
//   }
//   else
//   {
//     digitalWrite(rightMotorPin1,LOW);
//     digitalWrite(rightMotorPin2,LOW);      
//   }
  
//   if (leftMotorSpeed < 0)
//   {
//     digitalWrite(leftMotorPin1,LOW);
//     digitalWrite(leftMotorPin2,HIGH);    
//   }
//   else if (leftMotorSpeed > 0)
//   {
//     digitalWrite(leftMotorPin1,HIGH);
//     digitalWrite(leftMotorPin2,LOW);      
//   }
//   else
//   {
//     digitalWrite(leftMotorPin1,LOW);
//     digitalWrite(leftMotorPin2,LOW);      
//   } 
  ledcWrite(rightMotorPWMSpeedChannel, abs(rightMotorSpeed));
  ledcWrite(leftMotorPWMSpeedChannel, abs(leftMotorSpeed));   
}

void setUpPinModes()
{
  pinMode(enableRightMotor,OUTPUT);
  pinMode(rightMotorPin1,OUTPUT);
  pinMode(rightMotorPin2,OUTPUT);
  
  pinMode(enableLeftMotor,OUTPUT);
  pinMode(leftMotorPin1,OUTPUT);
  pinMode(leftMotorPin2,OUTPUT);

  //Set up PWM for motor speed
  ledcSetup(rightMotorPWMSpeedChannel, PWMFreq, PWMResolution);
  ledcSetup(leftMotorPWMSpeedChannel, PWMFreq, PWMResolution);  
  ledcAttachPin(enableRightMotor, rightMotorPWMSpeedChannel);
  ledcAttachPin(enableLeftMotor, leftMotorPWMSpeedChannel);  
  
  rotateMotor(0, 0);
}


void notify()
{
  if(millis() - lastTimeStamp > 100) {
  int rightMotorSpeed, leftMotorSpeed;
  int stickY, stickX;
  int rightMotorSpeedRaw, leftMotorSpeedRaw;

  // if(PS4.Triangle() && millis() - lastTimeStamp > 500) {
  //   maxSpeed  += 5;
  //   if(maxSpeed > 255) {
  //     maxSpeed = 220;
  //   }
   // minSpeed = maxSpeed * -1;
  //   lastTimeStamp = millis();
  //   Serial.printf("New Max Speed: %4d", maxSpeed);   

  // }


  stickY = PS4.RStickY() ;
  stickX = PS4.RStickX() / 3;
  rightMotorSpeedRaw = constrain(stickY - stickX,  -127, 127);
  leftMotorSpeedRaw = constrain(stickY + stickX,  -127, 127);

  rightMotorSpeed = map( rightMotorSpeedRaw, -127, 127, minSpeed, maxSpeed); //Left stick  - y axis - forward/backward left motor movement
  leftMotorSpeed = map( leftMotorSpeedRaw, -127, 127, minSpeed, maxSpeed);  //Right stick - y axis - forward/backward right motor movement


//Only needed to print the message properly on serial monitor. Else we dont need it.
//   if (millis() - lastTimeStamp > 200)
//   {
//     Serial.printf("StickY: %4d, StickX: %4d, right-raw: %4d, left-raw: %4d, right: %4d, left: %4d \n", stickY, stickX, rightMotorSpeedRaw, leftMotorSpeedRaw, rightMotorSpeed, leftMotorSpeed);   
//     lastTimeStamp = millis();
//   }
    
 //original: working
//   rightMotorSpeed = map( PS4.RStickY(), -127, 127, -220, 220); //Left stick  - y axis - forward/backward left motor movement
//   leftMotorSpeed = map( PS4.LStickY(), -127, 127, -220, 220);  //Right stick - y axis - forward/backward right motor movement


  rightMotorSpeed = constrain(rightMotorSpeed, minSpeed, maxSpeed);
  leftMotorSpeed = constrain(leftMotorSpeed, minSpeed, maxSpeed);

  rotateMotor(rightMotorSpeed, leftMotorSpeed);
  lastTimeStamp = millis();
  }
    
}



void onConnect()
{
  Serial.println("Connected!.");
}

void onDisConnect()
{
  rotateMotor(0, 0);
  Serial.println("Disconnected!.");    
}

void setup()
{
  setUpPinModes();
  Serial.begin(9600);
  PS4.attach(notify);
  PS4.attachOnConnect(onConnect);
  PS4.attachOnDisconnect(onDisConnect);
  PS4.begin();
  Serial.println("Ready.");
}

void loop()
{
}