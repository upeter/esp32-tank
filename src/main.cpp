#include <PS4Controller.h>
unsigned long lastTimeStamp = 0;
int maxSpeed = 255;
int minSpeed = maxSpeed * -1;

// Right motor
int enableRightMotor = 22;
int rightMotorPin1 = 16;
int rightMotorPin2 = 17;
// Left motor
int enableLeftMotor = 23;
int leftMotorPin1 = 18;
int leftMotorPin2 = 19;

const int PWMFreq = 1000; /* 1 KHz */
const int PWMResolution = 8;
const int rightMotorPWMSpeedChannel = 4;
const int leftMotorPWMSpeedChannel = 5;

const int lightSensorPin = 36;
const int ledPin1 = 13;
const int buzzerPin1 = 12;

class Motors
{

  void setMotorDirections(int motorPin1, int motorPin2, int speed)
  {
    int pin1State = (speed > 0) ? HIGH : LOW;
    int pin2State = (speed < 0) ? HIGH : LOW;

    digitalWrite(motorPin1, pin1State);
    digitalWrite(motorPin2, pin2State);
  }

public:
  void rotateMotor(int rightMotorSpeed, int leftMotorSpeed)
  {
    setMotorDirections(rightMotorPin1, rightMotorPin2, rightMotorSpeed);
    setMotorDirections(leftMotorPin1, leftMotorPin2, leftMotorSpeed);
    ledcWrite(rightMotorPWMSpeedChannel, abs(rightMotorSpeed));
    ledcWrite(leftMotorPWMSpeedChannel, abs(leftMotorSpeed));
  }

  void setUpPinModes()
  {
    pinMode(enableRightMotor, OUTPUT);
    pinMode(rightMotorPin1, OUTPUT);
    pinMode(rightMotorPin2, OUTPUT);

    pinMode(enableLeftMotor, OUTPUT);
    pinMode(leftMotorPin1, OUTPUT);
    pinMode(leftMotorPin2, OUTPUT);

    // Set up PWM for motor speed
    ledcSetup(rightMotorPWMSpeedChannel, PWMFreq, PWMResolution);
    ledcSetup(leftMotorPWMSpeedChannel, PWMFreq, PWMResolution);
    ledcAttachPin(enableRightMotor, rightMotorPWMSpeedChannel);
    ledcAttachPin(enableLeftMotor, leftMotorPWMSpeedChannel);

    rotateMotor(0, 0);
  }
};

class MotorChaosMonkey
{

  unsigned long startMillis; // will store last time LED was updated
  volatile boolean active = false;
  boolean phaseOneExecuted;
  boolean phaseTwoExecuted;
  unsigned long chaosDurationMillis = 800;
  unsigned long coolDownMillis = 3000;
  Motors *motors;

public:
  MotorChaosMonkey(Motors *motors_)
  {
    motors = motors_;
  }

  void Start()
  {
     unsigned long currentMillis = millis();
    if (!active && (currentMillis - startMillis) >= coolDownMillis)
    {
      active = true;
      startMillis = millis();
      Serial.println((String) "chaos started");
    }
    else
    {
      Serial.println((String) "chaos won't start because it is already active");
    }
  }

  boolean isActive()
  {
    return active;
  }

  void Update()
  {
    unsigned long currentMillis = millis();
    if (active)
    {
      if ((currentMillis - startMillis) >= chaosDurationMillis)
      {
        active = false;
        startMillis = 0;
        phaseOneExecuted = false;
        phaseTwoExecuted = false;
        Serial.println((String) "chaos terminated after: " + chaosDurationMillis);
      }
      else
      {
        if (!phaseOneExecuted)
        {
          motors->rotateMotor(maxSpeed, -maxSpeed);
          phaseOneExecuted = true;
        }
        if (phaseOneExecuted && !phaseTwoExecuted && currentMillis - startMillis >= (chaosDurationMillis / 2))
        {
          motors->rotateMotor(-maxSpeed, maxSpeed);
          phaseTwoExecuted = true;
        }
      }
    }
  }
};

class Flasher
{
  // Class Member Variables
  // These are initialized at startup
  int ledPin;     // the number of the LED pin
  long OnTime;    // milliseconds of on-time
  long OffTime;   // milliseconds of off-time
  boolean active; // active or not

  // These maintain the current state
  int ledState;                 // ledState used to set the LED
  unsigned long previousMillis; // will store last time LED was updated

public:
  Flasher() {}
  // Constructor - creates a Flasher
  // and initializes the member variables and state
public:
  Flasher(int pin, long on, long off)
  {
    ledPin = pin;
    pinMode(ledPin, OUTPUT);

    OnTime = on;
    OffTime = off;

    ledState = LOW;
    previousMillis = 0;
    active = false;
  }

  void ToggleActive()
  {
    active = !active;
  }
  void Activate()
  {
    active = true;
  }

  void Deactivate()
  {
    active = false;
  }

  void Update()
  {
    // check to see if it's time to change the state of the LED
    unsigned long currentMillis = millis();
    if (active)
    {
      if ((ledState == HIGH) && (currentMillis - previousMillis >= OnTime))
      {
        ledState = LOW;                 // Turn it off
        previousMillis = currentMillis; // Remember the time
        digitalWrite(ledPin, ledState); // Update the actual LED
      }
      else if ((ledState == LOW) && (currentMillis - previousMillis >= OffTime))
      {
        ledState = HIGH;                // turn it on
        previousMillis = currentMillis; // Remember the time
        digitalWrite(ledPin, ledState); // Update the actual LED
      }
    }
    else
    {
      previousMillis = 0;
      digitalWrite(ledPin, LOW);
    }
  }
};

class Buzzer
{
  // Class Member Variables
  // These are initialized at startup
  int buzzerPin;   // the number of the Buzzer pin
  long SoundMs;    // milliseconds sound buzzer for give frequency
  long FreqStepHz; // change in frequency in Hz per step
  boolean active;  // active or not
  long freq;       // frequency
  // These maintain the current state
  unsigned long activatedMillis; // will store last time Buzzer was updated

public:
  Buzzer() {}

  // Constructor - creates a Flasher
  // and initializes the member variables and state
public:
  Buzzer(int pin, long soundMs, int frequency)
  {
    buzzerPin = pin;
    pinMode(buzzerPin, OUTPUT);
    active = false;
    freq = frequency;
    SoundMs = soundMs;
  }

  void Activate()
  {
    active = true;
    activatedMillis = millis();
  }

  void Deactivate()
  {
    active = false;
    activatedMillis = 0;
  }

  void Update()
  {
    // check to see if it's time to change the state of the LED
    // Serial.println((String)"buzzer active?: " + active);
    unsigned long currentMillis = millis();
    if (active)
    {
      // exceeded sound ms
      if (((currentMillis - activatedMillis) >= SoundMs))
      {
        noTone(buzzerPin);
        Deactivate();
      }
      else
      {
        tone(buzzerPin, freq);
      }
    }
  }
};
class Sensor
{
  String id;
  int ldrPin;
  int sensitivity;

public:
  int hitCount;
  Flasher flasher;
  Buzzer buzzer;
  long DurationTime; // milliseconds of on-time
  boolean active;    // active or not
  MotorChaosMonkey *chaosMonkey;

  // These maintain the current state
  unsigned long previousMillis; // will store last time Sensor was updated

public:
  Sensor(String id_, int ldrPin_, int sensitivity_, Flasher flash, Buzzer buz, MotorChaosMonkey *chaosMonkey_, long durationTime)
  {
    id = id_;
    ldrPin = ldrPin_;
    sensitivity = sensitivity_;
    buzzer = buz;
    flasher = flash;
    DurationTime = durationTime;
    active = false;
    hitCount = 0;
    chaosMonkey = chaosMonkey_;
    pinMode(ldrPin, INPUT);
  }

  void Update()
  {

    unsigned long currentMillis = millis();
    if (active)
    {
      // Serial.println((String)"timing: " + (currentMillis - previousMillis) + " " + DurationTime);
      if ((currentMillis - previousMillis) >= DurationTime)
      {
        // Serial.println((String)"deactivate: " + (currentMillis - previousMillis) + " " + DurationTime);
        flasher.Deactivate();
        // buzzer.Deactivate();
        active = false;
        previousMillis = 0; // Remember the time
      }
      // previousMillis = currentMillis;  // Remember the time
    }
    else
    {
      int ldrStatus = analogRead(ldrPin);
      Serial.println((String) "" + id + ": Sensitity: " + (sensitivity) + " ldr: " + ldrStatus);

      // low ldr = hit
      if (ldrStatus < sensitivity)
      {
        hitCount++;
        flasher.Activate();
        buzzer.Activate();
        active = true;
        Serial.print("Hit: ");
        Serial.println(hitCount);
        try
        {
          chaosMonkey->Start();
        }
        catch (const std::exception &e)
        {
          Serial.print("Exception caught: ");
          Serial.println(e.what());
        }
         Serial.print("Started Chaos: ");
      }
    }
    flasher.Update();
    buzzer.Update();
  }

  void incrementSensitivity(int inc)
  {
    sensitivity += inc;
    Serial.println((String) "" + id + ": New Sensitity: " + (sensitivity));
  }
  void decrementSensitivity(int inc)
  {
    sensitivity -= inc;
    Serial.println((String) "" + id + ": New Sensitity: " + (sensitivity));
  }
};

Motors *motors;
Flasher flasher1(ledPin1, 200, 200);
Buzzer buzzer1(buzzerPin1, 300, 1400);
MotorChaosMonkey *chaosMonkey;
Sensor * sensor1;

volatile long hitMillis;

void notify()
{
  try {
    if (millis() - lastTimeStamp > 100)
    {
      int rightMotorSpeed, leftMotorSpeed;
      int stickY, stickX;
      int rightMotorSpeedRaw, leftMotorSpeedRaw;

      if (PS4.Triangle())
      {
        chaosMonkey->Start();
      }

      if (PS4.Circle())
      {
        sensor1->incrementSensitivity(5);
      }
      if (PS4.Square())
      {
        sensor1->decrementSensitivity(5);
      }

      stickY = PS4.RStickY();
      stickX = PS4.RStickX() / 3;
      rightMotorSpeedRaw = constrain(stickY - stickX, -127, 127);
      leftMotorSpeedRaw = constrain(stickY + stickX, -127, 127);

      rightMotorSpeed = map(rightMotorSpeedRaw, -127, 127, minSpeed, maxSpeed); // Left stick  - y axis - forward/backward left motor movement
      leftMotorSpeed = map(leftMotorSpeedRaw, -127, 127, minSpeed, maxSpeed);   // Right stick - y axis - forward/backward right motor movement

      // Only needed to print the message properly on serial monitor. Else we dont need it.
      // Serial.printf("StickY: %4d, StickX: %4d, right-raw: %4d, left-raw: %4d, right: %4d, left: %4d \n", stickY, stickX, rightMotorSpeedRaw, leftMotorSpeedRaw, rightMotorSpeed, leftMotorSpeed);

      // original: working
      //   rightMotorSpeed = map( PS4.RStickY(), -127, 127, -220, 220); //Left stick  - y axis - forward/backward left motor movement
      //   leftMotorSpeed = map( PS4.LStickY(), -127, 127, -220, 220);  //Right stick - y axis - forward/backward right motor movement

      rightMotorSpeed = constrain(rightMotorSpeed, minSpeed, maxSpeed);
      leftMotorSpeed = constrain(leftMotorSpeed, minSpeed, maxSpeed);

      // motorLeft->setMotorDirections(leftMotorSpeed);
      // motorRight->setMotorDirections(rightMotorSpeed);
      // motorLeft->rotateMotor(leftMotorSpeed);
      // motorRight->rotateMotor(rightMotorSpeed);

      // rotateMotor(rightMotorSpeed, leftMotorSpeed);
      //  int remainingHitMillis = millis() - hitMillis;
      //  if(remainingHitMillis < 600) {
      //    motors->rotateMotor(maxSpeed, -maxSpeed);
      //  } else if(remainingHitMillis < 600 && remainingHitMillis > 300) {
      //    motors->rotateMotor(-maxSpeed, maxSpeed);
      //  } else {
      //    motors->rotateMotor(rightMotorSpeed, leftMotorSpeed);
      //  }
      if (!chaosMonkey->isActive())
      {
        motors->rotateMotor(rightMotorSpeed, leftMotorSpeed);
      }

      lastTimeStamp = millis();
    }

  }  catch (const std::exception &e)
    {
      Serial.print("Exception caught: ");
      Serial.println(e.what());
    }
  }

  void onConnect()
  {
    Serial.println("Connected!.");
  }

  void onDisConnect()
  {
    motors->rotateMotor(0, 0);
    Serial.println("Disconnected!.");
  }

  void setup()
  {

    Serial.begin(9600);
    Serial.println("Initialize motors...");

    motors = new Motors();
    chaosMonkey = new MotorChaosMonkey(motors);
    sensor1 = new Sensor("t1", lightSensorPin, 200, flasher1, buzzer1, chaosMonkey, 400);
    motors->setUpPinModes();


    Serial.println("Motors initialized");
    PS4.attach(notify);
    PS4.attachOnConnect(onConnect);
    PS4.attachOnDisconnect(onDisConnect);
    PS4.begin();
    Serial.println("Ready.");
  }

  void loop()
  {
    try
    {
      sensor1->Update();
      chaosMonkey->Update();
    }
    catch (const std::exception &e)
    {
      Serial.print("Exception caught: ");
      Serial.println(e.what());
    }
  }