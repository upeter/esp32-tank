#include <PS4Controller.h>
#include <ESP32Servo.h>
#include <vector>
#include <initializer_list>

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

// PWM
const int PWMFreq = 1000; /* 1 KHz */
const int PWMResolution = 8;
const int rightMotorPWMSpeedChannel = 4;
const int leftMotorPWMSpeedChannel = 5;
// expo
const float expo = 0.5;
const float nullFactor = 100.0;
const int maxX = 35;

// Servo
const int baseServoPin = 13;
// Sensor
const int sensitivity = 20;
const int lightSensorPin1 = 36;
const int lightSensorPin2 = 39;
const int lightSensorPin3 = 34;
// Lighter
const int lighterPin1 = 5;
const int ledPin1 = 13;
const int buzzerPin1 = 12;

volatile int trimX = 0;

float withExpo(int x)
{
  if (x == 0)
  {
    return 1.0;
  }
  else
  {
    x = abs(x);
    float n = (float(x)) / nullFactor;
    float y = expo * pow(n, 3) + ((1.0 - expo) * n);
    return y;
  }
}

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

  boolean isActive()
  {
    return active;
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

class Lighter
{
  long OnTime; // milliseconds of on-time
  Flasher flasher1;
  unsigned long previousMillis; // will store last time LED was updated

public:
  Lighter() {}

public:
  Lighter(Flasher &flasher1_, long on)
  {
    OnTime = on;
    previousMillis = 0;
    flasher1 = flasher1_;
  }

  void Activate()
  {
    if (!flasher1.isActive())
    {
      Serial.println("Activate flasher");
      previousMillis = millis();
      flasher1.Activate();
    }
  }

  void Deactivate()
  {
    flasher1.Deactivate();
  }

  void Update()
  {
    // check to see if it's time to change the state of the LED
    unsigned long currentMillis = millis();
    if (flasher1.isActive())
    {
      flasher1.Update();
      if (currentMillis - previousMillis >= OnTime)
      {
        Serial.println("Deactivate flasher");
        flasher1.Deactivate();
        flasher1.Update();
        previousMillis = 0;
      }
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
      // Serial.println((String) "" + id + ": Sensitity: " + (sensitivity) + " ldr: " + ldrStatus);

      // low ldr = hit
      if (ldrStatus < sensitivity)
      {
        hitCount++;
        flasher.Activate();
        buzzer.Activate();
        active = true;
        Serial.print((String) "" + id + " hit: ");
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

class Sensors
{
private:
  std::vector<Sensor *> sensorCollection;

public:
  Sensors(std::initializer_list<Sensor *> sensors) : sensorCollection(sensors)
  {
  }

  void Update()
  {
    for (Sensor *sensor : sensorCollection)
    {
      sensor->Update();
    }
  }

  void incrementSensitivity(int inc)
  {
    for (Sensor *sensor : sensorCollection)
    {
      sensor->incrementSensitivity(inc);
    }
  }
  void decrementSensitivity(int dec)
  {
    for (Sensor *sensor : sensorCollection)
    {
      sensor->decrementSensitivity(dec);
    }
  }
};

Motors *motors;
Flasher flasher1(ledPin1, 1000, 0);
Flasher lighterFlasher1(lighterPin1, 100, 100);
Lighter lighter1(lighterFlasher1, 2000);
Buzzer buzzer1(buzzerPin1, 300, 1400);
MotorChaosMonkey *chaosMonkey;
Sensor *sensor1;
Sensor *sensor2;
Sensor *sensor3;

Sensors *sensors;

Servo *baseServo;

volatile long hitMillis;

void notify()
{
  try
  {
    if (millis() - lastTimeStamp > 100)
    {
      int rightMotorSpeed, leftMotorSpeed;
      int stickY, stickX, stickXWithExpo, stickXWithExpoFloat;
      int rightMotorSpeedRaw, leftMotorSpeedRaw;
      int baseX;
      int baseServoPosition;

      baseX = PS4.LStickX();
      baseServoPosition = map(baseX, -127, 127, 0, 180);
      baseServo->write(baseServoPosition);
      // chaos monkey
      if (PS4.Triangle())
      {
        chaosMonkey->Start();
      }
      // shoot
      if (PS4.R1() || PS4.L1())
      {
        Serial.println("Shoot");
        lighter1.Activate();
      }

      // special functions
      if (PS4.L2())
      {
        // trim
        if (PS4.Right())
        {
          trimX++;
          Serial.println("Trim++: " + (String)trimX);
        }
        if (PS4.Left())
        {
          trimX--;
          Serial.println("Trim--: " + (String)trimX);
        }
        // change sensitivity
        if (PS4.Up())
        {
          sensors->incrementSensitivity(1);
        }
        if (PS4.Down())
        {
          sensors->decrementSensitivity(1);
        }
      }

      stickY = PS4.LStickY();
      stickX = PS4.RStickX();
      float stickXExpo = withExpo(stickX);
      // stickX = (toggle ? (int)((float)stickX * stickXExpo) : stickX / 3) * -1;
      stickXWithExpo = constrain((int)((float(stickX) * stickXExpo) + trimX) * -1, maxX * -1, maxX);

      rightMotorSpeedRaw = constrain((stickY - stickXWithExpo), -127, 127);
      leftMotorSpeedRaw = constrain((stickY + stickXWithExpo), -127, 127);

      rightMotorSpeed = map(rightMotorSpeedRaw, -127, 127, minSpeed, maxSpeed); // Left stick  - y axis - forward/backward left motor movement
      leftMotorSpeed = map(leftMotorSpeedRaw, -127, 127, minSpeed, maxSpeed);   // Right stick - y axis - forward/backward right motor movement

      // Only needed to print the message properly on serial monitor. Else we dont need it.
      // Serial.printf("StickY: %4d, StickX: %4d, right-raw: %4d, left-raw: %4d, right: %4d, left: %4d \n", stickY, stickX, rightMotorSpeedRaw, leftMotorSpeedRaw, rightMotorSpeed, leftMotorSpeed);

      // original: working
      //   rightMotorSpeed = map( PS4.RStickY(), -127, 127, -220, 220); //Left stick  - y axis - forward/backward left motor movement
      //   leftMotorSpeed = map( PS4.LStickY(), -127, 127, -220, 220);  //Right stick - y axis - forward/backward right motor movement

      rightMotorSpeed = constrain(rightMotorSpeed, minSpeed, maxSpeed);

      leftMotorSpeed = constrain(leftMotorSpeed, minSpeed, maxSpeed);

      //   Serial.printf("StickY: %4d, StickX: %4d, StickXExpo: %f, StickXWithExpo: %d, trimX: %d ", stickY, stickX, stickXExpo, stickXWithExpo, trimX);
      //   Serial.printf("right-raw: %d,  left-raw: %d, right: %d, left: %d \n", rightMotorSpeedRaw, leftMotorSpeedRaw, rightMotorSpeed, leftMotorSpeed);

      if (!chaosMonkey->isActive())
      {
        motors->rotateMotor(rightMotorSpeed, leftMotorSpeed);
      }

      lastTimeStamp = millis();
    }
  }
  catch (const std::exception &e)
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
  sensor1 = new Sensor("t1", lightSensorPin1, sensitivity, flasher1, buzzer1, chaosMonkey, 400);
  sensor2 = new Sensor("t2", lightSensorPin2, sensitivity, flasher1, buzzer1, chaosMonkey, 400);
  sensor3 = new Sensor("t3", lightSensorPin3, sensitivity, flasher1, buzzer1, chaosMonkey, 400);
  sensors = new Sensors({sensor1, sensor2, sensor3});
  motors->setUpPinModes();
  baseServo = new Servo();

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  baseServo->setPeriodHertz(50);              // standard 50 hz servo
  baseServo->attach(baseServoPin, 500, 2400); //

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
    sensors->Update();
    chaosMonkey->Update();
    lighter1.Update();
  }
  catch (const std::exception &e)
  {
    Serial.print("Exception caught: ");
    Serial.println(e.what());
  }
}