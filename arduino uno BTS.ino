 #include <math.h>

#include <SoftwareSerial.h>

SoftwareSerial btSerial(0, 1); // TX, RX                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      

/*BTS7960 Motor Driver Carrier*/
const int MotorRight_R_EN = 4; // Pin to control the clockwise direction of Right Motor
const int MotorRight_L_EN = 5; // Pin to control the counterclockwise direction of Right Motor 
const int MotorLeft_R_EN = 8; // Pin to control the clockwise direction of Left Motor
const int MotorLeft_L_EN = 9; // Pin to control the counterclockwise direction of Left Motor
const int Rpwm1 = 6; // pwm output - motor A
const int Lpwm1 = 7; // pwm output - motor B
const int Rpwm2 = 10; // pwm output - motor A
const int Lpwm2 = 11; // pwm output - motor B
int mspeed = 255;
int robotControlState;

char c;
void setup(){
  
  //Setup Right Motors
  pinMode(MotorRight_R_EN, OUTPUT); 
  pinMode(MotorRight_L_EN, OUTPUT);

  //Setup Left Motors
  pinMode(MotorLeft_R_EN, OUTPUT); 
  pinMode(MotorLeft_L_EN, OUTPUT);
  
  //Setup PWM pins as Outputs
  pinMode(Rpwm1, OUTPUT);
  pinMode(Lpwm1, OUTPUT);
  pinMode(Rpwm2, OUTPUT);
  pinMode(Lpwm2, OUTPUT);
  
  Serial.begin(9600);
  btSerial.begin(9600);
  
  stop_Robot();
}// void setup()

void loop(){
  if (btSerial.available()) {

    processInput();
  }
}// void loop()

void processInput (){
  static long receivedNumber = 0;
  static boolean negative = false;
  Serial.write(c);
  c=btSerial.read();
  Serial.println(c);
  switch (c){

  case 'F': // Go FORWARD
    go_Forward(mspeed);
    //Serial.println("forward");
    break;

  case 'B': // Go BACK
    go_Backwad(mspeed);
    break;

  case 'R':
    turn_Right(mspeed);
    break;

  case 'L':
    turn_Left(mspeed);
    break;

  case 'I': // Top Right
    move_RightForward(255);
    break; 

  case 'G': // Top Left
    move_LeftForward(mspeed);
    break;  

  case 'J': // Bottom Right
    move_RightBackward(mspeed);
    break; 

  case 'H': // Bottom Left
    move_LeftBackward(mspeed);
    break;  

  case 'S':
    stop_Robot();
    break;

  case '1':
  mspeed = 100;
  
    break;
    case '2':
  mspeed = 120;
  
    break;
    case '3':
  mspeed = 150;
  
    break;
    case '4':
  mspeed = 180;
  
    break;
    case '5':
  mspeed = 190;
  
    break;
    case '6':
  mspeed = 200;
  
    break;
    case '7':
  mspeed = 210;
  
    break;
    case '8':
  mspeed = 220;
  
    break;
    case '9':
  mspeed = 240;
  
    break;
    case 'q':
  mspeed = 255;
  
    break;

  case '-':
    negative = true;
    break;
  } 
} 

void stop_Robot(){ 
    //SetMotors(2); 
    analogWrite(Rpwm1, 0);
    analogWrite(Lpwm1, 0);
    analogWrite(Rpwm2, 0);
    analogWrite(Lpwm2, 0);
 
}// void stopRobot()

void turn_Right(int mspeed){ 
    SetMotors(1);
    analogWrite(Rpwm1, 0);
    analogWrite(Lpwm1, mspeed);
    analogWrite(Rpwm2, mspeed);
    analogWrite(Lpwm2, 0);
  
}// void turn_Right(int mspeed)

void turn_Left(int mspeed){ 
    SetMotors(1);
    analogWrite(Rpwm1, mspeed);
    analogWrite(Lpwm1, 0);
    analogWrite(Rpwm2, 0);
    analogWrite(Lpwm2, mspeed);

}// void turn_Left(int mspeed)

void go_Forward(int mspeed){ 
    SetMotors(1);
    analogWrite(Rpwm1, mspeed);
    analogWrite(Lpwm1, 0);
    analogWrite(Rpwm2, mspeed);
    analogWrite(Lpwm2, 0);
   
}// void goForward(int mspeed)

void go_Backwad(int mspeed){ 
    SetMotors(1);
    analogWrite(Rpwm1, 0);
    analogWrite(Lpwm1, mspeed);
    analogWrite(Rpwm2, 0);
    analogWrite(Lpwm2, mspeed);
}// void goBackwad(int mspeed)

void move_RightForward(int mspeed){ 
    SetMotors(1);
    analogWrite(Rpwm1, mspeed*0.6);
    analogWrite(Lpwm1, 0);
    analogWrite(Rpwm2, mspeed);
    analogWrite(Lpwm2, 0);
  
}// void move_RightForward(int mspeed)

void move_LeftForward(int mspeed){ 
    SetMotors(1);
    analogWrite(Rpwm1, mspeed);
    analogWrite(Lpwm1, 0);
    analogWrite(Rpwm2, mspeed*0.6);
    analogWrite(Lpwm2, 0);
}// move_LeftForward(int mspeed)

void move_RightBackward(int mspeed){ 
    SetMotors(1);
    analogWrite(Rpwm1, 0);
    analogWrite(Lpwm1, 0);
    analogWrite(Rpwm2, 0);
    analogWrite(Lpwm2, mspeed);
}// void move_RightBackward(int mspeed)

void move_LeftBackward(int mspeed){ 
    SetMotors(1);
    analogWrite(Rpwm1, 0);
    analogWrite(Lpwm1, mspeed);
    analogWrite(Rpwm2, 0);
    analogWrite(Lpwm2, 0);   
}// void move_LeftBackward(int mspeed)

void stopRobot(int delay_ms){
  SetMotors(2);
  analogWrite(Rpwm1, 0);
  analogWrite(Lpwm1, 0);
  analogWrite(Rpwm2, 0);
  analogWrite(Lpwm2, 0);
  delay(delay_ms);
}// void stopRobot(int delay_ms)

void SetMotors(int controlCase){
  switch(controlCase){
    case 1:
      digitalWrite(MotorRight_R_EN, HIGH);  
      digitalWrite(MotorRight_L_EN, HIGH); 
      digitalWrite(MotorLeft_R_EN, HIGH);  
      digitalWrite(MotorLeft_L_EN, HIGH); 
    break;
    case 2:
      digitalWrite(MotorRight_R_EN, LOW);  
      digitalWrite(MotorRight_L_EN, LOW); 
      digitalWrite(MotorLeft_R_EN, LOW);  
      digitalWrite(MotorLeft_L_EN, LOW); 
    break;
  } 
}// void SetMotors(int controlCase)
