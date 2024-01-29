#include <ESP32Servo.h>
#include "AS5600.h"
#include "Wire.h"

#define stepPin 26
#define dirPin 25
#define en      14
#define lwr    23
#define uppr    27
#define m1      4
#define m2     16
#define e1     17
#define e1channel 6
#define m3      5
#define m4     18
#define e2     19
#define e2channel 7
#define touchSensor 36
#define trigPin  15
#define echoPin 2




hw_timer_t * timer = NULL;
Servo servoMotor;
int pos = 180;
int state = 1;
char input[6]; 
char lastinput[6]; 
int size;
int x = 0;
int targetAngle = 0;
int currentAngle = 0;
int servoMove = 0;
int upflag = 0;
long long lastTime = 0;
int currentRobotAngle = 0;
int targetRobotAngle = 0;
int startStopRobot = 0;
int rotAngleL=0;
int rotAngleR=0;
int rotTargetAngle=0; 
bool goUp = true;
bool goDown = true;
long duration;
int distance;

AS5600 as5600_0(&Wire);
 
AS5600 as5600_1(&Wire1);

void IRAM_ATTR onTimer() {
  digitalWrite(stepPin,!digitalRead(stepPin));
}

void movRev(){
  rotAngleL = as5600_0.getCumulativePosition() * AS5600_RAW_TO_DEGREES;
  Serial.print(rotAngleL);
  // Serial.print("\t");
  // Serial.println(rotTargetAngle);
  // input[0] = 'z';
  // input[1] = '\n';
  // input[2] = '\n';
  // input[3] = '\n';
  // input[4] = '\n';
  // input[5] = '\n';
  if(rotAngleL == rotTargetAngle)
  {
    startStopRobot = 1;
    input[0] = 'z';
    digitalWrite(m1,LOW);
    digitalWrite(m2,LOW);
    digitalWrite(m3,LOW);
    digitalWrite(m4,LOW);
    input[1] = '\n';
    input[2] = '\n';
    input[3] = '\n';
    input[4] = '\n';
    input[5] = '\n';
    Serial.println("equal");
  }
  if(rotAngleL != rotTargetAngle && startStopRobot == 0)
  {
    if(rotTargetAngle - rotAngleL > 0)
    {
      digitalWrite(m1,HIGH);
      digitalWrite(m2,LOW);
      ledcWrite(e1channel,200);
      digitalWrite(m3,HIGH);
      digitalWrite(m4,LOW);
      ledcWrite(e2channel,200);
      Serial.println("forward");
    }
    else if(rotTargetAngle - rotAngleL < 0)
    {
      digitalWrite(m1,LOW);
      digitalWrite(m2,HIGH);
      ledcWrite(e1channel,200);
      digitalWrite(m3,LOW);
      digitalWrite(m4,HIGH);
      ledcWrite(e2channel,200);
      Serial.println("backward");
    }
  }
}


// void rotateRobot(){
//   currentRobotAngle = int(round(ypr[0]* 180 / M_PI));
//   Serial.print(currentRobotAngle);
//   Serial.print("\t");
//   Serial.println(targetRobotAngle);
//   if(currentRobotAngle == targetRobotAngle)
//   {
//     startStopRobot = 1;
//     input[0] = 'z';
//     digitalWrite(m1,LOW);
//     digitalWrite(m2,LOW);
//     digitalWrite(m3,LOW);
//     digitalWrite(m4,LOW);
//     input[1] = '\n';
//     input[2] = '\n';
//     input[3] = '\n';
//     input[4] = '\n';
//     input[5] = '\n';
//   }
//   if(currentRobotAngle != targetRobotAngle && startStopRobot == 0)
//   {
//     if(targetRobotAngle - currentRobotAngle > 0)
//     {
//       digitalWrite(m1,LOW);
//       digitalWrite(m2,HIGH);
//       ledcWrite(e1channel,255);
//       digitalWrite(m3,HIGH);
//       digitalWrite(m4,LOW);
//       ledcWrite(e2channel,255);
//     }
//     else if(targetRobotAngle - currentRobotAngle < 0)
//     {
//       digitalWrite(m1,HIGH);
//       digitalWrite(m2,LOW);
//       ledcWrite(e1channel,255);
//       digitalWrite(m3,LOW);
//       digitalWrite(m4,HIGH);
//       ledcWrite(e2channel,255);
//     }
//   }
// }

void measureAngle()
{
  Serial.print(as5600_0.getCumulativePosition() * AS5600_RAW_TO_DEGREES);
  Serial.print("\t");
  Serial.print(as5600_1.getCumulativePosition() * -AS5600_RAW_TO_DEGREES);
  Serial.print("\t");
  Serial.println(currentRobotAngle);
  Serial.print("\n");
}

void moveServo(){
  if(targetAngle == 0){
    servoMove = 0;
    currentAngle = 0;
    servoMotor.write(0);
  }
  else if(targetAngle == 180){
    
    if(servoMove == 1)
      return;
    
    while(currentAngle < targetAngle /*&& (millis()-lastTime > 10)*/){
      currentAngle = currentAngle + 1;
      servoMotor.write(currentAngle);
      Serial.println(currentAngle);
      // lastTime = millis();   
      Serial.println(analogRead(touchSensor));
      if(analogRead(touchSensor) > 0){
        // Serial.println("larger");
        servoMove = 1;
        Serial.print(analogRead(touchSensor));
        Serial.println("here");
        return;
      }
      delay(10);
    }
  }  
}

void moveStepperUp(){
  digitalWrite(en,LOW);
  timerAlarmEnable(timer);
  // Serial.println("Moving up");
  digitalWrite(dirPin,LOW);
}

void moveStepperDown(){
   if(digitalRead(lwr) == LOW)
  {
    timerAlarmDisable(timer);
    digitalWrite(en,HIGH);   
    return;
  }
  digitalWrite(en,LOW);
  timerAlarmEnable(timer);
  // Serial.println("Moving down");
  digitalWrite(dirPin,HIGH);
}

void moveForward(){
  digitalWrite(m1,HIGH);
  digitalWrite(m2,LOW);
  ledcWrite(e1channel,255);
  digitalWrite(m3,HIGH);
  digitalWrite(m4,LOW);
  ledcWrite(e2channel,255);
  Serial.println("Forward");
}

void moveBackward(){
  digitalWrite(m1,LOW);
  digitalWrite(m2,HIGH);
  ledcWrite(e1channel,255);
  digitalWrite(m3,LOW);
  digitalWrite(m4,HIGH);
  ledcWrite(e2channel,255);
  // Serial.println("Backward");
}
void moveLeft(){
  digitalWrite(m1,LOW);

  digitalWrite(m2,HIGH);
  ledcWrite(e1channel,255);
  digitalWrite(m3,HIGH);
  digitalWrite(m4,LOW);
  ledcWrite(e2channel,255);
  // Serial.println("Left");
}

void moveRight(){
  digitalWrite(m1,HIGH);
  digitalWrite(m2,LOW);
  ledcWrite(e1channel,255);
  digitalWrite(m3,LOW);
  digitalWrite(m4,HIGH);
  ledcWrite(e2channel,255);
  // Serial.println("Right");
}
void ultrasonic(){
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distance = duration * 0.034 / 2;
  // Prints the distance on the Serial Monitor
  Serial.print("Distance: ");
  Serial.println(distance);
  delay(10);
}


void setup_esp() {
  pinMode(stepPin,OUTPUT); 
  pinMode(dirPin,OUTPUT);
  pinMode(touchSensor,INPUT);
  pinMode(lwr, INPUT);
  pinMode(uppr, INPUT);
  pinMode(m1,OUTPUT);
  pinMode(m2,OUTPUT);
  pinMode(e1,OUTPUT);
  pinMode(e2,OUTPUT);
  pinMode(en,OUTPUT);
  ledcSetup(e1channel,5000,8);
  ledcAttachPin(e1,e1channel);
  ledcSetup(e2channel,5000,8);
  ledcAttachPin(e2,e2channel);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT); 
  pinMode(m3,OUTPUT);
  pinMode(m4,OUTPUT);
  digitalWrite(en,HIGH);
  Serial.begin(115200);
  Wire.begin();
  Wire1.begin(33,32);
  // mpu.initialize();
  // Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  //mpu.dmpInitialize();
  // mpu.setXGyroOffset(40);
  // mpu.setYGyroOffset(8);
  // mpu.setZGyroOffset(21);
  // mpu.CalibrateGyro(6);
  // mpu.setDMPEnabled(true);
  // packetSize = mpu.dmpGetFIFOPacketSize();
  servoMotor.attach(13);
  servoMotor.write(0);
  Serial.println();
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 800, true);
  as5600_0.getCumulativePosition();
  as5600_0.resetPosition();
  as5600_1.getCumulativePosition();
  as5600_1.resetPosition();
  rotAngleL = as5600_0.getCumulativePosition() * AS5600_RAW_TO_DEGREES;
  rotAngleR = as5600_1.getCumulativePosition() * AS5600_RAW_TO_DEGREES;

}
void change_state(const void * msgin) {
  const std_msgs__msg__Char * msg = (const std_msgs__msg__Char*)msgin;


  if(msg->data == 'q' && goUp == true)
  {
    moveStepperUp();
    upflag = 0;
    // Serial.println("here");
    lastinput[0] = 'q';
    goDown = true;
  }
  else if(msg->data == 'e' && goDown == true)
  {
    moveStepperDown();
    upflag = 1;
    lastinput[0] = 'e';
    goUp = true;
  }
  else if(msg->data == 'i' || lastinput[0] == 'i' && msg->data == 'z')
  {
    measureAngle();
    lastinput[0] = 'i';
  }
  else if(msg->data == 't' || lastinput[0] == 't' && msg->data == 'z')
  {
    targetAngle = 0;
    moveServo();
    lastinput[0] = 't';
  }
  else if(msg->data == 'y' || lastinput[0] == 'y' && msg->data == 'z')
  {
    targetAngle = 180;
    moveServo();
    lastinput[0] = 'y';
  }
  else if(msg->data == 'w')
  { 
    moveForward();
    lastinput[0] = 'w';
  }
  else if(msg->data == 's')
  {
    moveBackward();
    lastinput[0] = 's';
  }
  else if(msg->data == 'a')
  {
    moveLeft();
    lastinput[0] = 'a';
  }
  else if(msg->data == 'd')
  {
    moveRight();
    lastinput[0] = 'd';
  }
  else if(msg->data == 'x'){
    ultrasonic();
    lastinput[0] = 'x';
  }

  else if(msg->data == 'r')
  {
    startStopRobot = 0;
    sscanf(input, "%1c%4d",&lastinput, &targetRobotAngle);
    targetRobotAngle = targetRobotAngle + currentRobotAngle;
    input[0] = 'z';
    Serial.println("Rotate to angle");
    Serial.println(targetRobotAngle);
  }
  else if(input[0] == 'm')
  {
    rotAngleL = as5600_0.getCumulativePosition() * AS5600_RAW_TO_DEGREES;
    startStopRobot = 0;
    sscanf(input, "%1c%4d",&lastinput, &rotTargetAngle);
    Serial.println(rotTargetAngle);
    rotTargetAngle = rotTargetAngle * 360;
    rotTargetAngle = rotTargetAngle + rotAngleL;
    input[0] = 'z';
    Serial.println("Rotate in angle");
    Serial.println(rotTargetAngle);
    movRev();
  }
  else if(lastinput[0] == 'm' && msg->data == 'z' && startStopRobot == 0)
  {
    movRev();
  }
  else
  {
    lastinput[0] = 'z';
    digitalWrite(m1,LOW);
    digitalWrite(m2,LOW);
    digitalWrite(m3,LOW);
    digitalWrite(m4,LOW);
    Serial.println("off");
  }
}
void brud(){

  if(digitalRead(lwr) == LOW && upflag != 0)
  { 
    // Serial.println("low");
    timerAlarmDisable(timer);
    digitalWrite(en,HIGH);
    goUp = true;
    goDown = false;
  }

  if(digitalRead(uppr) == LOW && upflag != 1)
  { 
    // Serial.println("low");
    timerAlarmDisable(timer);
    digitalWrite(en,HIGH);
    goUp = false;
    goDown = true;
  }

  // if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { 
  // mpu.dmpGetQuaternion(&q, fifoBuffer);
  // mpu.dmpGetGravity(&gravity, &q);
  // mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  // currentRobotAngle = int(round(ypr[0]* 180 / M_PI));
  // // Serial.print("y\t");
  // // Serial.println(ypr[0] * 180 / M_PI);
  // } 

}