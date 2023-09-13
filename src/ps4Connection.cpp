#include <PS4Controller.h>

unsigned long lastTimeStamp_ = 0;

void notify_()
{
  char messageString[200];
sprintf(messageString, "%4d,%4d,%4d,%4d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d",
  PS4.LStickX(),
  PS4.LStickY(),
  PS4.RStickX(),
  PS4.RStickY(),
  PS4.Left(),
  PS4.Down(),
  PS4.Right(),
  PS4.Up(),
  PS4.Square(),
  PS4.Cross(),
  PS4.Circle(),
  PS4.Triangle(),
  PS4.L1(),
  PS4.R1(),
  PS4.L2(),
  PS4.R2(),  
  PS4.Share(),
  PS4.Options(),
  PS4.PSButton(),
  PS4.Touchpad(),
  PS4.Charging(),
  PS4.Audio(),
  PS4.Mic(),
  PS4.Battery());

  //Only needed to print the message properly on serial monitor. Else we dont need it.
  if (millis() - lastTimeStamp_ > 50)
  {
    Serial.println(messageString);
    lastTimeStamp_ = millis();
  }
}

void onConnect_()
{
  Serial.println("Connected!.");
}

void onDisConnect_()
{
  Serial.println("Disconnected!.");    
}

void setup_() 
{
  Serial.begin(9600);
  PS4.attach(notify_);
  PS4.attachOnConnect(onConnect_);
  PS4.attachOnDisconnect(onDisConnect_);
  PS4.begin();
  Serial.println("Ready.");
}

void loop_() 
{

}