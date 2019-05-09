// Firmware of Poke-Vibration multimodal tactile display ver. 2 (After CHI'19 submission)

// LRA motor driver pin assignment
const byte motor9_F = A0;
const byte motor9_R = A1;
const byte motor8_F = A2;
const byte motor8_R = A3;
const byte motor7_F = A4;
const byte motor7_R = A5;
const byte motor6_F = A6;
const byte motor6_R = A7;
const byte motor5_F = A8;
const byte motor5_R = A9;
const byte motor4_F = A10;
const byte motor4_R = A11;
const byte motor3_F = A12;
const byte motor3_R = A13;
const int motor2_F = 44;
const int motor2_R = 42;
const int motor1_F = 48;
const int motor1_R = 46;

// Linear servo PWM pin assignment
const int pokePin[9] = {5, 6, 7, 8, 9, 10, 11, 12, 13};

bool stringComplete = false;
char inData[1000];
int dataIdx = 0;
bool pokeOn[9] = {false, false, false, false, false, false, false, false, false};
bool motorOn[9] = {false, false, false, false, false, false, false, false, false};
const bool initBool[9] = {false, false, false, false, false, false, false, false, false};

int curTime = 0; // Timer starts from 0 when pattern starts
const int duration = 275;// Pattern's duration(ms)
const int breakTime = 275;
const int pokeOffTime = duration - 33;
int initDepth = 170; // Minimum depth when foot contacts the display's bottom
int inDepth[9] = {170, 170, 170, 170, 170, 170, 170, 170, 170};
float degMmRatio = 20.0; // servo motor movement control (deg/mm)
int inoutDiff = (int)(1.5f * degMmRatio); // initiate poking depth to 1.5 mm (deg)

bool patternOn = false;
char inputP[4] = {'n', 'n', 'n', 'n'};
bool cmdDelivered[5] = {false, false, false, false, false};
bool initCmdD[5] = {false, false, false, false, false};

void setup() {
  pinMode (motor1_F, OUTPUT);
  pinMode (motor1_R, OUTPUT);
  pinMode (motor2_F, OUTPUT);
  pinMode (motor2_R, OUTPUT);
  pinMode (motor3_F, OUTPUT);
  pinMode (motor3_R, OUTPUT);
  pinMode (motor4_F, OUTPUT);
  pinMode (motor4_R, OUTPUT);
  pinMode (motor5_F, OUTPUT);
  pinMode (motor5_R, OUTPUT);
  pinMode (motor6_F, OUTPUT);
  pinMode (motor6_R, OUTPUT);
  pinMode (motor7_F, OUTPUT);
  pinMode (motor7_R, OUTPUT);
  pinMode (motor8_F, OUTPUT);
  pinMode (motor8_R, OUTPUT);
  pinMode (motor9_F, OUTPUT);
  pinMode (motor9_R, OUTPUT);

  for(int i=0;i<9;i++)
  {
    pinMode(pokePin[i], OUTPUT);
  }

  Serial.begin(115200);
  while (! Serial);
  Serial.println("Poke-Vibration Multimodal Tactile Display...");

  for(int i=0;i<9;i++)
  {
    servoPoke(i, inDepth[i]);
  }
}

void loop() {
  loopPatternManager();
  loopSerial();
  loopMotorOnOff();
}

void serialEvent()
{
  while(Serial.available() && stringComplete == false)
  {
    char inChar = Serial.read();
    inData[dataIdx++] = inChar;

    if(inChar == '\n')
    {
      dataIdx = 0;
      stringComplete = true;
    }
  }
}

// Function: loopSerial
void loopSerial()
{
  if(stringComplete)
  {
    char line[1000];
    int lineIdx = 0;
    
    // Count command chars & init inData (error prone)
    while(inData[lineIdx] != '\n' && lineIdx < 100)
    {
      line[lineIdx] = inData[lineIdx];
      inData[lineIdx] = NULL;
      lineIdx++;
    }
    
    char c1 = line[0], c2 = line[1], c3 = line[2], c4 = line[3];
    int pokeNum = 0;
    int motorNum = 0;
    float inPositionMm = 0.0;
    float tmpDepth = 0.0;
    
    switch(c1)
    {
      case 'p':
        pokeNum = (int)c2 - 49;
        if(0 <= pokeNum && pokeNum < 9)
        {
          pokeOn[pokeNum] = !pokeOn[pokeNum];
          Serial.print("Poke");
          Serial.print(c2);
          if(pokeOn[pokeNum])
          {
            Serial.print(": OUT\n");
            servoPoke(pokeNum, inDepth[pokeNum] - inoutDiff);
          }
          else
          {
             Serial.print(": IN\n");
             servoPoke(pokeNum, inDepth[pokeNum]);
          }
          Serial.flush();
        }
        break;
      case 'm':
        motorNum = (int)c2 - 49;
        if(0 <= motorNum && motorNum < 9)
        {
          motorOn[motorNum] = !motorOn[motorNum];
          Serial.print("Motor");
          Serial.print(c2);
          if(motorOn[motorNum])
          {
            Serial.print(": ON\n");
          }
          else
          {
             Serial.print(": OFF\n");
          }
          Serial.flush();
        }
        break;
      case 's':
        motorNum = (int)c2 - 49;
        if(0 <= motorNum && motorNum < 9)
        {
          Serial.print("Pulse");
          Serial.println(c2);
          motorPulse(motorNum);
          Serial.flush();
        }
        break;
      case 'z':
        Serial.println("Stop all");
        for(int i=0;i<9;i++)
        {
          pokeOn[i] = false;
          motorOn[i] = false;
          servoPoke(i, inDepth[i]);
        }
        Serial.flush();
        break;
      case 't':
        pokeNum = (int)c2 - 49;
        if(0 <= pokeNum && pokeNum < 9)
        {
          inPositionMm = ((c3 - 48) * 10 + (c4 - 48)) / 10.0; // unit: mm
          int inPositionDeg = (int)(initDepth - inPositionMm * degMmRatio);
          if (inPositionDeg >= (float)inoutDiff)
          {
            Serial.print("Tactor ");
            Serial.print(c2);
            Serial.print(" in-position: ");
            Serial.print(inPositionMm);
            Serial.println(" mm");
            inDepth[pokeNum] = inPositionDeg;
            servoPoke(pokeNum, inDepth[pokeNum]);
          }
          else
          {
            Serial.print("Error out of range: ");
            Serial.print(inPositionMm);
            Serial.println(" mm");
          }
          Serial.flush();
        }
        break;
      case 'l':
        tmpDepth = ((c2 - 48) * 10 + (c3 - 48)) / 10.0;
        inoutDiff = tmpDepth * degMmRatio;
        Serial.print("Poking depth: ");
        Serial.print(tmpDepth);
        Serial.println(" mm");
        Serial.flush();
        break;
      case 'r':
        Serial.println("Remove foot");
        Serial.flush();
        for(int i=0;i<9;i++)
        {
          pokeOn[i] = false;
          servoPoke(i, 0);
        }
        break;
      case 'a':
      case 'b':
      case 'c':
      case 'd':
      case 'n':
        inputP[0] = c1;
        inputP[1] = c2;
        inputP[2] = c3;
        inputP[3] = c4;
        patternOn = true;
        curTime = 0;
        memcpy(cmdDelivered, initCmdD, 5 * sizeof(bool));
        Serial.println("Pattern: " + String(c1) + String(c2) + String(c3) + String(c4));
        break;
      default:
        break;
    }
    stringComplete = false;
  }
}

// Function: loopPatternManager
// Check if the counter exceeds the duration and change or turn off the pattern bools
void loopPatternManager()
{
  if(patternOn)
  {
    if(curTime > duration * 2 + breakTime)
    {
      memcpy(motorOn, initBool, 9 * sizeof(bool)); 
        
      patternOn = false;
    }
    else if(curTime > duration + breakTime + pokeOffTime && !cmdDelivered[4])
    {
      actPoke(inputP[2], false);
      cmdDelivered[4] = true;
    }
    else if(curTime > duration + breakTime && !cmdDelivered[3])
    {
      actPoke(inputP[2], true);
      actMotor(inputP[3], true);
      cmdDelivered[3] = true;
    }
    else if(curTime > duration && !cmdDelivered[2])
    {
      actMotor(inputP[1], false);
      cmdDelivered[2] = true;
    }
    else if(curTime > pokeOffTime && !cmdDelivered[1])
    {
      actPoke(inputP[0], false);
      cmdDelivered[1] = true;
    }
    else if(curTime > 0 && !cmdDelivered[0])
    {
      actPoke(inputP[0], true);
      actMotor(inputP[1], true);
      cmdDelivered[0] = true;
    }
  }
}

void actPoke(char cmd, bool onoff){
  switch(cmd)
  {
    case 'a':
      if(onoff)
      {
        servoPoke(2, inDepth[2] - inoutDiff);
      }
      else
      {
        servoPoke(2, inDepth[2]);
      }
      break;
    case 'b':
      if(onoff)
      {
        servoPoke(8, inDepth[5] - inoutDiff);
      }
      else
      {
        servoPoke(8, inDepth[5]);
      }
      break;
    case 'c':
      if(onoff)
      {
        servoPoke(0, inDepth[1] - inoutDiff);
      }
      else
      {
        servoPoke(0, inDepth[1]);
      }
      break;
    case 'd':
      if(onoff)
      {
        servoPoke(6, inDepth[4] - inoutDiff);
      }
      else
      {
        servoPoke(6, inDepth[4]);
      }
      break;
    default:
      break;
  }
}

void actMotor(char cmd, bool onoff){
  switch(cmd)
  {
    case 'a':
      if(onoff)
      {
        motorOn[2] = true;
      }
      else
      {
        motorOn[2] =false;
      }
      break;
    case 'b':
      if(onoff)
      {
        motorOn[8] = true;
      }
      else
      {
        motorOn[8] =false;
      }
      break;
    case 'c':
      if(onoff)
      {
        motorOn[0] = true;
      }
      else
      {
        motorOn[0] =false;
      }
      break;
    case 'd':
      if(onoff)
      {
        motorOn[6] = true;
      }
      else
      {
        motorOn[6] =false;
      }
      break;
    default:
      break;
  }
}

// Function: servoPoke
// Write the angle on the specific servo
void servoPoke (int servoNum, int angle)
{
  digitalWrite(pokePin[servoNum], HIGH);
  delayMicroseconds(map(angle, 0, 180, 500, 2300));
  digitalWrite(pokePin[servoNum], LOW);
}

// Function: loopMotorOnOff
// Turn on LRA if true (166Hz full-powered)
void loopMotorOnOff ()
{
  digitalWrite(motor1_F, LOW);
  digitalWrite(motor1_R, LOW);
  digitalWrite(motor2_F, LOW);
  digitalWrite(motor2_R, LOW);
  digitalWrite(motor3_F, LOW);
  digitalWrite(motor3_R, LOW);
  digitalWrite(motor4_F, LOW);
  digitalWrite(motor4_R, LOW);
  digitalWrite(motor5_F, LOW);
  digitalWrite(motor5_R, LOW);
  digitalWrite(motor6_F, LOW);
  digitalWrite(motor6_R, LOW);
  digitalWrite(motor7_F, LOW);
  digitalWrite(motor7_R, LOW);
  digitalWrite(motor8_F, LOW);
  digitalWrite(motor8_R, LOW);
  digitalWrite(motor9_F, LOW);
  digitalWrite(motor9_R, LOW);
  
  //Forward
  if(motorOn[0])
  {
    digitalWrite(motor1_F, HIGH);
    digitalWrite(motor1_R, LOW);
  }
  if(motorOn[1])
  {
    digitalWrite(motor2_F, HIGH);
    digitalWrite(motor2_R, LOW);
  }
  if(motorOn[2])
  {
    digitalWrite(motor3_F, HIGH);
    digitalWrite(motor3_R, LOW);
  }
  if(motorOn[3])
  {
    digitalWrite(motor4_F, HIGH);
    digitalWrite(motor4_R, LOW);
  }
  if(motorOn[4])
  {
    digitalWrite(motor5_F, HIGH);
    digitalWrite(motor5_R, LOW);
  }
  if(motorOn[5])
  {
    digitalWrite(motor6_F, HIGH);
    digitalWrite(motor6_R, LOW);
  }
  if(motorOn[6])
  {
    digitalWrite(motor7_F, HIGH);
    digitalWrite(motor7_R, LOW);
  }
  if(motorOn[7])
  {
    digitalWrite(motor8_F, HIGH);
    digitalWrite(motor8_R, LOW);
  }
  if(motorOn[8])
  {
    digitalWrite(motor9_F, HIGH);
    digitalWrite(motor9_R, LOW);
  }
  
  delayCount(3);
  
  if(motorOn[0])
  {
    digitalWrite(motor1_F, LOW);
    digitalWrite(motor1_R, LOW);
  }
  if(motorOn[1])
  {
    digitalWrite(motor2_F, LOW);
    digitalWrite(motor2_R, LOW);
  }
  if(motorOn[2])
  {
    digitalWrite(motor3_F, LOW);
    digitalWrite(motor3_R, LOW);
  }
  if(motorOn[3])
  {
    digitalWrite(motor4_F, LOW);
    digitalWrite(motor4_R, LOW);
  }
  if(motorOn[4])
  {
    digitalWrite(motor5_F, LOW);
    digitalWrite(motor5_R, LOW);
  }
  if(motorOn[5])
  {
    digitalWrite(motor6_F, LOW);
    digitalWrite(motor6_R, LOW);
  }
  if(motorOn[6])
  {
    digitalWrite(motor7_F, LOW);
    digitalWrite(motor7_R, LOW);
  }
  if(motorOn[7])
  {
    digitalWrite(motor8_F, LOW);
    digitalWrite(motor8_R, LOW);
  }
  if(motorOn[8])
  {
    digitalWrite(motor9_F, LOW);
    digitalWrite(motor9_R, LOW);
  }
    
  //Reverse
  if(motorOn[0])
  {
    digitalWrite(motor1_F, LOW);
    digitalWrite(motor1_R, HIGH);
  }
  if(motorOn[1])
  {
    digitalWrite(motor2_F, LOW);
    digitalWrite(motor2_R, HIGH);
  }
  if(motorOn[2])
  {
    digitalWrite(motor3_F, LOW);
    digitalWrite(motor3_R, HIGH);
  }
  if(motorOn[3])
  {
    digitalWrite(motor4_F, LOW);
    digitalWrite(motor4_R, HIGH);
  }
  if(motorOn[4])
  {
    digitalWrite(motor5_F, LOW);
    digitalWrite(motor5_R, HIGH);
  }
  if(motorOn[5])
  {
    digitalWrite(motor6_F, LOW);
    digitalWrite(motor6_R, HIGH);
  }
  if(motorOn[6])
  {
    digitalWrite(motor7_F, LOW);
    digitalWrite(motor7_R, HIGH);
  }
  if(motorOn[7])
  {
    digitalWrite(motor8_F, LOW);
    digitalWrite(motor8_R, HIGH);
  }
  if(motorOn[8])
  {
    digitalWrite(motor9_F, LOW);
    digitalWrite(motor9_R, HIGH);
  }

  delayCount(3);
  
  if(motorOn[0])
  {
    digitalWrite(motor1_F, LOW);
    digitalWrite(motor1_R, LOW);
  }
  if(motorOn[1])
  {
    digitalWrite(motor2_F, LOW);
    digitalWrite(motor2_R, LOW);
  }
  if(motorOn[2])
  {
    digitalWrite(motor3_F, LOW);
    digitalWrite(motor3_R, LOW);
  }
  if(motorOn[3])
  {
    digitalWrite(motor4_F, LOW);
    digitalWrite(motor4_R, LOW);
  }
  if(motorOn[4])
  {
    digitalWrite(motor5_F, LOW);
    digitalWrite(motor5_R, LOW);
  }
  if(motorOn[5])
  {
    digitalWrite(motor6_F, LOW);
    digitalWrite(motor6_R, LOW);
  }
  if(motorOn[6])
  {
    digitalWrite(motor7_F, LOW);
    digitalWrite(motor7_R, LOW);
  }
  if(motorOn[7])
  {
    digitalWrite(motor8_F, LOW);
    digitalWrite(motor8_R, LOW);
  }
  if(motorOn[8])
  {
    digitalWrite(motor9_F, LOW);
    digitalWrite(motor9_R, LOW);
  }
}

void motorPulse(int motor_Num)
{
  byte motor_F, motor_R;

  switch(motor_Num)
  {
    case 0:
      motor_F = motor1_F;
      motor_R = motor1_R;
      break;
    case 1:
      motor_F = motor2_F;
      motor_R = motor2_R;
      break;
    case 2:
      motor_F = motor3_F;
      motor_R = motor3_R;
      break;
    case 3:
      motor_F = motor4_F;
      motor_R = motor4_R;
      break;
    case 4:
      motor_F = motor5_F;
      motor_R = motor5_R;
      break;
    case 5:
      motor_F = motor6_F;
      motor_R = motor6_R;
      break;
    case 6:
      motor_F = motor7_F;
      motor_R = motor7_R;
      break;
    case 7:
      motor_F = motor8_F;
      motor_R = motor8_R;
      break;
    case 8:
      motor_F = motor9_F;
      motor_R = motor9_R;
      break;
    default:
      break;
  }
  
  digitalWrite(motor_F, LOW);
  digitalWrite(motor_R, LOW);
  
  //Forward
  digitalWrite(motor_F, HIGH);
  digitalWrite(motor_R, LOW);
  
  delayCount(3);
  
  digitalWrite(motor_F, LOW);
  digitalWrite(motor_R, LOW);
  
  //Reverse
  digitalWrite(motor_F, LOW);
  digitalWrite(motor_R, HIGH);

  delayCount(3);

  digitalWrite(motor_F, LOW);
  digitalWrite(motor_R, LOW);
}

// Function: delayCount
// Delay time and count up currTime
void delayCount(int time)
{
  curTime = curTime + time;
  delay(time);
}
