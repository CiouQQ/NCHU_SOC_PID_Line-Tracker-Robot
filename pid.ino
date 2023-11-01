#include <LiquidCrystal_PCF8574.h>
#include <Wire.h>
#include<SoftwareSerial.h>
LiquidCrystal_PCF8574 lcd(0x27);
SoftwareSerial BTSerial(2, 3); // RX | TX
char ch;

const int IN_A0 = A0; 
const int IN_A2 = A2; 
const int IN_A1 = A1; //L-Left
// const int IN_A3 = A3; //R-Right
// const int IN_4 = 4; 

int detect[3];
int a = 0;
int t =0;
int t1 = 0;

const byte LEFT1 = 8;
const byte LEFT2 = 9;
const byte LEFT_PWM = 10;

const byte RIGHT1 = 7;
const byte RIGHT2 = 6;
const byte RIGHT_PWM = 5;

byte motorSpeed = 130;


//pid
float Kp = 50 , Ki = 0, Kd = 0;                   //pid弯道参数参数
float error = 0, P = 0, I = 0, D = 0, PID_value = 0;//pid直道参数
float previous_error = 0, previous_I = 0;           //误差值
int sensor[3] = {0, 0, 0};  
static int initial_motor_speed = 100;                //初始速度

void read_sensor_values()
{
  
  sensor[0] = digitalRead(IN_A0);
  sensor[1] = digitalRead(IN_A2);
  sensor[2] = digitalRead(IN_A1);
  if (detect[1] == 0 && detect[2] == 1&& detect[0] == 0) { //010
    error = 0;
  }
  else if (detect[2] == 1 && detect[1] ==1 && detect[0] == 1 ) { //111
    error = 0;
  }
  else if (detect[2] == 1 && detect[1] == 1 ) { //x11
    error = 1;
  }
  else if (detect[2] == 1 && detect[0] == 1 ) { //11x
    error = -1;
  }
  else if ( detect[1] == 1 ) { //xx1
    error = 3;
  }
  else if ( detect[0] == 1 ) { //1xx
    error = -3;
  }
  
  // else if (detect[2] == 0 && detect[1] ==0 && detect[0] == 0 )
  // {
  //   t = 1;
  //   backward();
  // }





  if (sensor[0]==1 && sensor[1]==0 && sensor[2]==0) {
    error = -4;//100
  } else if (sensor[0]==1 && sensor[1]==1 && sensor[2]==0) {
    error = -1;//110
  } else if (sensor[0]==0 && sensor[1]==1 && sensor[2]==0) {
    error = 0;//010
  } else if (sensor[0]==0 && sensor[1]==1 && sensor[2]==1) {
    error = 1;//011
  } else if (sensor[0]==0 && sensor[1]==0 && sensor[2]==1) {
    error = 4;//001
  }
}

void calc_pid()
{
  P = error;
  I = I + error;
  D = error - previous_error;
  PID_value = (Kp * P) + (Ki * I) + (Kd * D);
  previous_error = error;
}

void motor_control()
{
  int left_motor_speed ;
  int right_motor_speed ;
  left_motor_speed = initial_motor_speed + PID_value;
  right_motor_speed = initial_motor_speed - PID_value;
  if (left_motor_speed < -180) {
    left_motor_speed = -180;
  }
  if (left_motor_speed > 180) {
    left_motor_speed = 180;
  }
  if (right_motor_speed < -180) {
    right_motor_speed = -180;
  }
  if (right_motor_speed > 180) {
    right_motor_speed = 180;
  }
  motorsWrite(left_motor_speed, right_motor_speed);
}

void motorsWrite(int speedL, int speedR)
{
  if (speedL > 0) {
    digitalWrite(LEFT1, HIGH); 
    digitalWrite(LEFT2, LOW);
    analogWrite(LEFT_PWM,speedL);
  } else {
    digitalWrite(LEFT1, LOW); 
    digitalWrite(LEFT2, HIGH);
    analogWrite(LEFT_PWM,speedL);
  }
  if (speedR > 0) {
    digitalWrite(RIGHT1, LOW); 
    digitalWrite(RIGHT2, HIGH);
    analogWrite(RIGHT_PWM,speedR);
  } else {
    digitalWrite(RIGHT1, HIGH); 
    digitalWrite(RIGHT2, LOW);
    analogWrite(RIGHT_PWM,speedR);
  }
}



void LC(){
 
  
  


  lcd.setCursor(0, 0);
  
  lcd.print(detect[0]);
  lcd.print(", ");
  lcd.print(detect[2]);
  lcd.print(", ");
  lcd.print(detect[1]);
  lcd.setCursor(0, 1);
  lcd.print("L ");
  lcd.setCursor(14, 1);
  lcd.print("R ");

}



void stop()
{
  analogWrite(LEFT_PWM,0);
  analogWrite(RIGHT_PWM,0);
}
void backward()
{
  digitalWrite(LEFT1, LOW); 
  digitalWrite(LEFT2, HIGH);
  analogWrite(LEFT_PWM,115);
  digitalWrite(RIGHT1, HIGH); 
  digitalWrite(RIGHT2, LOW);
  analogWrite(RIGHT_PWM,130);
}
void forward(int Lspeed,int Rspeed)
{
  digitalWrite(LEFT1, HIGH); 
  digitalWrite(LEFT2, LOW);
  analogWrite(LEFT_PWM,Rspeed);
  digitalWrite(RIGHT1, LOW); 
  digitalWrite(RIGHT2, HIGH);
  analogWrite(RIGHT_PWM,Rspeed);
}
void turnRight(int Lspeed,int Rspeed)
{
  digitalWrite(LEFT1, HIGH); 
  digitalWrite(LEFT2, LOW);
  analogWrite(LEFT_PWM,Lspeed);
  digitalWrite(RIGHT1, HIGH); 
  digitalWrite(RIGHT2, LOW);
  analogWrite(RIGHT_PWM,Rspeed);
}
void turnLeft(int Lspeed,int Rspeed)
{
  digitalWrite(LEFT1, LOW); 
  digitalWrite(LEFT2, HIGH);
  analogWrite(LEFT_PWM,Lspeed);
  digitalWrite(RIGHT1, LOW); 
  digitalWrite(RIGHT2, HIGH);
  analogWrite(RIGHT_PWM,Rspeed);
}

void track(){
  if (detect[1] == 0 && detect[2] == 1&& detect[0] == 0) { //010
    forward(120,120); 
    t = 0 ;
    t1 --;
  }
  else if (detect[2] == 1 && detect[1] ==1 && detect[0] == 1 ) { //111
    forward(100,100); 
    t = 0;
    t1--;
  }
  else if (detect[2] == 1 && detect[1] == 1 ) { //x11
    turnRight(120,100) ;
    t = 0 ;
    t1 --;
  }
  else if (detect[2] == 1 && detect[0] == 1 ) { //11x
    turnLeft(100,120) ;
    t = 0 ;
    t1 --;
  }
  else if ( detect[1] == 1 ) { //xx1
    turnRight(120,120) ;
    t = 0 ;
    t1 --;
  }
  else if ( detect[0] == 1 ) { //1xx
    turnLeft(120,120) ;
    t = 0 ;
    t1 --;
  }
  else if (detect[2] == 0 && detect[1] ==0 && detect[0] == 0 &&t1 < 30000 && t == 0 ) { //111
    t1++;
    // forward(80,80);
    
  }
  else if (detect[2] == 0 && detect[1] ==0 && detect[0] == 0 )
  {
    t = 1;
    backward();
  }
  
  
 
  
  
}

void setup() {
  pinMode(IN_A0, INPUT);
  pinMode(IN_A2, INPUT);
  pinMode(IN_A1, INPUT);
  // pinMode(IN_A3, INPUT);
  // pinMode(IN_4, INPUT);
  lcd.begin(16, 2); // initialize the lcd
  lcd.setBacklight(10);
  
  Serial.begin(9600);
//  Serial.println("Enter AT commands:");
  BTSerial.begin(9600); // HC-05 default speed in AT command more
}

void loop() {
  read_sensor_values();         //循迹小车
  calc_pid();
  motor_control();

  // put your main code here, to run repeatedly:
  // while(BTSerial.available()) 
  // {
  //   delay(3);
  //   ch = BTSerial.read();
  //   if( (ch != 'w') && (ch != 's') && (ch != 'x') && (ch != 'a') && (ch != 'd') && (ch != 'z') && (ch != 'c') && (ch != 'w')) {
  //      continue;
  //   }
  //   else
  //   {
  //     Serial.println(ch);
  //     break;
  //   }
  // }
  // ch = BTSerial.read();
  // if( ch == 'w') {
  //      a=1;
  //   }
  // else if(ch == 's'){
  //   a = 0;
  //   stop();
  // }
  // detect[0] = digitalRead(IN_A0);
  // detect[1] = digitalRead(IN_A1);//Center
  // detect[2] = digitalRead(IN_A2);//Center
  // if(a == 0){
  //   track();
  // }
  // track();
  // LC();
  
  
  // delay(10);

}
