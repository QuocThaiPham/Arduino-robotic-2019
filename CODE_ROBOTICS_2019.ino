#include <Servo.h>
Servo myServo;
int servoPin = 48;
#include <NewPing.h>
#define TRIGGER_PIN  46
#define ECHO_PIN     44
#define MAX_DISTANCE 80
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
int distance;
unsigned long time;
int m;
int pin_mode = 50;
const int analogInPin = A5;
const int speedpin = A13;
int speedsensorValue = 0;
int sensorValue = 0;
int outputValue = 0;
const int line_pin[5] = {A0, A1, A2, A3, A4};
int  LFSensor[5] = {0};
int i;
const int line_back_pin[5] = {A6, A7, A8, A9, A10};
int  backLFSensor[5] = {0};
int j;
int mode;
int mode2;
int error2;
// Motor Variables
int ENA = 6;//trai
int motorInput1 = 5;
int motorInput2 = 7;
int motorInput3 = 4;
int motorInput4 = 8;
int ENB = 3;
int initial_motor_speed  ;
int motor_speed;
float Kp = 369;
float Ki = 0.5;
float Kd = 5;
int led1 = 11;
int led2 = 12;
int led3 = 14;
float error = 0, P = 0, I = 0, D = 0, PID_value = 0;
float previous_error = 0, previous_I = 0;
int kill = 0;
int trai;
int phai;
int san = 1;//den=0
int line = 0;
int k;
int fix;
int fix2;
int c;
int a;
int c2;
int a2;
int l;
int ob;
int f;
int h;
void setup ()
{ Serial.begin(9600);
  pinMode(A11, INPUT_PULLUP); //truoc
  pinMode(A12, INPUT_PULLUP); //sau
  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(13, OUTPUT);
  pinMode(14, OUTPUT);
  myServo.attach(servoPin);
  flat_off();
  Serial.println("Started !!");
  // delay(500);
  digitalWrite(13, LOW);
  int trai = 0;
  int phai = 0;
}
void read_mode()
{
  m = digitalRead(pin_mode);
}
void find_opponent()
{
  delay(50);
  distance = sonar.ping_cm();
  if ((distance > 40) && (distance < 75)) {
    ob = 80;
    led3_on();
  }
  else if ((distance > 10) && (distance < 40)) {
    ob = 60;
    led3_blink();
  }
  else if ((distance == 0) || (distance > 75)) {
    ob = 85;
    led3_off();
  }
  else if ((distance > 0) && (distance < 7)) {
    ob = 1;
    led3_blink();
  }
}
void led3_on()
{
  digitalWrite(14, HIGH);
}
void led3_off()
{
  digitalWrite(14, LOW);
}
void led3_blink()
{
  digitalWrite(14, HIGH);
  delay(100);
  digitalWrite(14, LOW);
  delay(100);
}
void led13_on()
{
  digitalWrite(13, HIGH);
}
void led13_off()
{
  digitalWrite(13, LOW);
}
void led13_blink()
{
  digitalWrite(13, HIGH);
  delay(100);
  digitalWrite(13, LOW);
  delay(100);
}
void flat_off()
{
  myServo.write(0);
}
void flat_on()
{
  myServo.write(105);
}
void led1_on()
{
  digitalWrite(led1, HIGH);
}
void led1_off()
{
  digitalWrite(led1, LOW);
}
void led1_blink()
{ digitalWrite(led1, HIGH);
  delay(100);
  digitalWrite(led1, LOW);
  delay(100);
}
void led2_on()
{
  digitalWrite(led2, HIGH);
}
void led2_off()
{
  digitalWrite(led2, LOW);
}
void led2_blink()
{ digitalWrite(led2, HIGH);
  delay(50);
  digitalWrite(led2, LOW);
  delay(50);
}
void read_center1()
{
  for (i = 0; i < 5; i++)
  {
    LFSensor[i] = digitalRead(line_pin[i]);
  }
  if ((     LFSensor[0] == san ) && (LFSensor[1] == line ) && (LFSensor[2] == line ) && (LFSensor[3] == line ) && (LFSensor[4] == line ))  {
    fix = -4; //phai
  }
  else if ((LFSensor[0] == line ) && (LFSensor[1] == line ) && (LFSensor[2] == line ) && (LFSensor[3] == line ) && (LFSensor[4] == san ))  {
    fix = 4; //trai
  }
  else if ((LFSensor[0] == line ) && (LFSensor[1] == line ) && (LFSensor[2] == line ) && (LFSensor[3] == line ) && (LFSensor[4] == line ))  {
    fix = 103;
  }
  else if ((LFSensor[0] == san ) && (LFSensor[1] == san ) && (LFSensor[2] == san ) && (LFSensor[3] == san ) && (LFSensor[4] == san ))  {
    fix = 102;
  }
}
void read_center2()
{
  for (j = 0; j < 5; j++)
  {
    backLFSensor[j] = digitalRead(line_back_pin[j]);
  }
  if ((     backLFSensor[0] == san ) && (backLFSensor[1] == line ) && (backLFSensor[2] == line ) && (backLFSensor[3] == line ) && (backLFSensor[4] == line ))  {
    fix2 = -4; //phai
  }
  else if ((backLFSensor[0] == line ) && (backLFSensor[1] == line ) && (backLFSensor[2] == line ) && (backLFSensor[3] == line ) && (backLFSensor[4] == san ))  {
    fix2 = 4; //trai
  }
  else if ((backLFSensor[0] == line ) && (backLFSensor[1] == line ) && (backLFSensor[2] == line ) && (backLFSensor[3] == line ) && (backLFSensor[4] == line ))  {
    fix2 = 103;
  }
  else if ((backLFSensor[0] == san ) && (backLFSensor[1] == san ) && (backLFSensor[2] == san ) && (backLFSensor[3] == san ) && (backLFSensor[4] == san ))  {
    fix2 = 102;
  }
}
void read_center()
{
  read_center1();
  read_center2();
}
void read_sensor_values()
{
  for (i = 0; i < 5; i++)
  {
    LFSensor[i] = digitalRead(line_pin[i]);
  }

  if ((     LFSensor[0] == san ) && (LFSensor[1] == san ) && (LFSensor[2] == san ) && (LFSensor[3] == san ) && (LFSensor[4] == line ))  {
    mode = 0;  //FOLLOWING_LINE 0
    error = -4;
  }
  else if ((LFSensor[0] == san ) && (LFSensor[1] == san ) && (LFSensor[2] == san ) && (LFSensor[3] == line ) && (LFSensor[4] == line ))  {
    mode = 0;  //FOLLOWING_LINE
    error = -3;
  }
  else if ((LFSensor[0] == san ) && (LFSensor[1] == san ) && (LFSensor[2] == san ) && (LFSensor[3] == line ) && (LFSensor[4] == san ))  {
    mode = 0;  //FOLLOWING_LINE
    error = -2;
  }
  else if ((LFSensor[0] == san ) && (LFSensor[1] == san ) && (LFSensor[2] == line ) && (LFSensor[3] == line ) && (LFSensor[4] == san ))  {
    mode = 0;  //FOLLOWING_LINE
    error = -1;
  }
  else if ((LFSensor[0] == san ) && (LFSensor[1] == san ) && (LFSensor[2] == line ) && (LFSensor[3] == san ) && (LFSensor[4] == san ))  {
    mode = 0;  //FOLLOWING_LINE
    error = 0;
  }
  else if ((LFSensor[0] == san ) && (LFSensor[1] == line ) && (LFSensor[2] == line ) && (LFSensor[3] == san ) && (LFSensor[4] == san ))  {
    mode = 0;  //FOLLOWING_LINE
    error = 1;
  }
  else if ((LFSensor[0] == san ) && (LFSensor[1] == line ) && (LFSensor[2] == san ) && (LFSensor[3] == san ) && (LFSensor[4] == san ))  {
    mode = 0;  //FOLLOWING_LINE
    error = 2;
  }
  else if ((LFSensor[0] == line ) && (LFSensor[1] == line ) && (LFSensor[2] == san ) && (LFSensor[3] == san ) && (LFSensor[4] == san ))  {
    mode = 0;  //FOLLOWING_LINE
    error = 3;
  }
  else if ((LFSensor[0] == line ) && (LFSensor[1] == san ) && (LFSensor[2] == san ) && (LFSensor[3] == san ) && (LFSensor[4] == san ))  {
    mode = 0;  //FOLLOWING_LINE//
    error = 4;
  }
  else if ((LFSensor[0] == line ) && (LFSensor[1] == line ) && (LFSensor[2] == line ) && (LFSensor[3] == san ) && (LFSensor[4] == san ))  {
    mode = 0;  // TURN LEFT
    error = 100;
  }
  else if ((LFSensor[0] == san ) && (LFSensor[1] == san ) && (LFSensor[2] == line ) && (LFSensor[3] == line ) && (LFSensor[4] == line ))  {
    mode = 0;  // TURN RIGHT
    error = 101;
  }
  else if ((LFSensor[0] == san ) && (LFSensor[1] == san ) && (LFSensor[2] == san ) && (LFSensor[3] == san ) && (LFSensor[4] == san ))  {
    mode = 1;  //NO_LINE 2 VONG DAU9
    error = 102;
  }
  else if ((LFSensor[0] == line ) && (LFSensor[1] == line ) && (LFSensor[2] == line ) && (LFSensor[3] == line ) && (LFSensor[4] == line ))  {
    mode = 0;  //STOPPED 1 BIEN
    error = 103;
  }
}
void read_back_sensor_values()
{
  for (j = 0; j < 5; j++)
  {
    backLFSensor[j] = digitalRead(line_back_pin[j]);
  }

  if ((     backLFSensor[0] == san ) && (backLFSensor[1] == san ) && (backLFSensor[2] == san ) && (backLFSensor[3] == san ) && (backLFSensor[4] == line ))  {
    mode2 = 0;  //FOLLOWING_LINE 0
    error2 = -4;
  }
  else if ((backLFSensor[0] == san ) && (backLFSensor[1] == san ) && (backLFSensor[2] == san ) && (backLFSensor[3] == line ) && (backLFSensor[4] == line ))  {
    mode2 = 0;  //FOLLOWING_LINE
    error2 = -3;
  }
  else if ((backLFSensor[0] == san ) && (backLFSensor[1] == san ) && (backLFSensor[2] == san ) && (backLFSensor[3] == line ) && (backLFSensor[4] == san ))  {
    mode2 = 0;  //FOLLOWING_LINE
    error2 = -2;
  }
  else if ((backLFSensor[0] == san ) && (backLFSensor[1] == san ) && (backLFSensor[2] == line ) && (backLFSensor[3] == line ) && (backLFSensor[4] == san ))  {
    mode2 = 0;  //FOLLOWING_LINE
    error2 = -1;
  }
  else if ((backLFSensor[0] == san ) && (backLFSensor[1] == san ) && (backLFSensor[2] == line ) && (backLFSensor[3] == san ) && (backLFSensor[4] == san ))  {
    mode2 = 0;  //FOLLOWING_LINE
    error2 = 0;
  }
  else if ((backLFSensor[0] == san ) && (backLFSensor[1] == line ) && (backLFSensor[2] == line ) && (backLFSensor[3] == san ) && (backLFSensor[4] == san ))  {
    mode2 = 0;  //FOLLOWING_LINE
    error2 = 1;
  }
  else if ((backLFSensor[0] == san ) && (backLFSensor[1] == line ) && (backLFSensor[2] == san ) && (backLFSensor[3] == san ) && (backLFSensor[4] == san ))  {
    mode2 = 0;  //FOLLOWING_LINE
    error2 = 2;
  }
  else if ((backLFSensor[0] == line ) && (backLFSensor[1] == line ) && (backLFSensor[2] == san ) && (backLFSensor[3] == san ) && (backLFSensor[4] == san ))  {
    mode2 = 0;  //FOLLOWING_LINE
    error2 = 3;
  }
  else if ((backLFSensor[0] == line ) && (backLFSensor[1] == san ) && (backLFSensor[2] == san ) && (backLFSensor[3] == san ) && (backLFSensor[4] == san ))  {
    mode2 = 0;  //FOLLOWING_LINE//
    error2 = 4;
  }
  else if ((backLFSensor[0] == line ) && (backLFSensor[1] == line ) && (backLFSensor[2] == line ) && (backLFSensor[3] == san ) && (backLFSensor[4] == san ))  {
    mode2 = 0;  // TURN LEFT
    error2 = 100;
  }
  else if ((backLFSensor[0] == san ) && (backLFSensor[1] == san ) && (backLFSensor[2] == line ) && (backLFSensor[3] == line ) && (backLFSensor[4] == line ))  {
    mode2 = 0;  // TURN RIGHT
    error2 = 101;
  }
  else if ((backLFSensor[0] == san ) && (backLFSensor[1] == san ) && (backLFSensor[2] == san ) && (backLFSensor[3] == san ) && (backLFSensor[4] == san ))  {
    mode2 = 2;  //NO_LINE 2 VONG DAU9
    error2 = 102;
  }
  else if ((backLFSensor[0] == line ) && (backLFSensor[1] == line ) && (backLFSensor[2] == line ) && (backLFSensor[3] == line ) && (backLFSensor[4] == line ))  {
    mode2 = 0;  //STOPPED 1 BIEN
    error2 = 103;
  }

}
void calculate_pid()
{
  P = error;
  I = I + previous_I;
  D = error - previous_error;

  PID_value = (Kp * P) + (Ki * I) + (Kd * D);

  previous_I = I;
  previous_error = error;
}
void motor_control()
{
  // Calculating the effective motor speed:
  int left_motor_speed = initial_motor_speed - PID_value;
  int right_motor_speed = initial_motor_speed + PID_value;
  if (right_motor_speed > initial_motor_speed ) right_motor_speed = initial_motor_speed; // prevent the motor from going beyond max speed
  if (left_motor_speed > initial_motor_speed ) left_motor_speed = initial_motor_speed; // prevent the motor from going beyond max speed
  if (right_motor_speed < 0) right_motor_speed = 0; // keep the motor speed positive
  if (left_motor_speed < 0) left_motor_speed = 0; // keep the motor speed positive

  // The motor speed should not exceed the max PWM value
  left_motor_speed = constrain(left_motor_speed, 0, initial_motor_speed);
  right_motor_speed = constrain(right_motor_speed, 0, initial_motor_speed);

  // Serial.print(PID_value);
  //  Serial.print("\t");
  //  Serial.print(left_motor_speed);
  //   Serial.print("\t");
  //   Serial.println(right_motor_speed);
  analogWrite(ENA, left_motor_speed); //Left Motor Speed
  analogWrite(ENB, right_motor_speed ); //Right Motor Speed

  //following lines of code are to make the bot move forward
  forward();
}

void reverse()
{
  /*The pin numbers and high, low values might be different depending on your connections */

  digitalWrite(motorInput1, HIGH);
  digitalWrite(motorInput2, LOW);
  digitalWrite(motorInput3, HIGH);
  digitalWrite(motorInput4, LOW);
}
void forward()
{
  /*The pin numbers and high, low values might be different depending on your connections */
  digitalWrite(motorInput1, LOW);
  digitalWrite(motorInput2, HIGH);
  digitalWrite(motorInput3, LOW);
  digitalWrite(motorInput4, HIGH);
}
void left()
{
  /*The pin numbers and high, low values might be different depending on your connections */

  digitalWrite(motorInput1, LOW);
  digitalWrite(motorInput2, LOW);
  digitalWrite(motorInput3, LOW);
  digitalWrite(motorInput4, HIGH);
}
void right()
{
  /*The pin numbers and high, low values might be different depending on your connections */

  digitalWrite(motorInput1, LOW);
  digitalWrite(motorInput2, HIGH);
  digitalWrite(motorInput3, LOW);
  digitalWrite(motorInput4, LOW);
}
void sharpLeftTurn() {
  /*The pin numbers and high, low values might be different depending on your connections */
  digitalWrite(motorInput1, HIGH);
  digitalWrite(motorInput2, LOW);
  digitalWrite(motorInput3, LOW);
  digitalWrite(motorInput4, HIGH);
}
void sharpRightTurn() {
  /*The pin numbers and high, low values might be different depending on your connections */


  digitalWrite(motorInput1, LOW);
  digitalWrite(motorInput2, HIGH);
  digitalWrite(motorInput3, HIGH);
  digitalWrite(motorInput4, LOW);
}
void stop_bot()
{
  /*The pin numbers and high, low values might be different depending on your connections */
  digitalWrite(motorInput1, LOW);
  digitalWrite(motorInput2, LOW);
  digitalWrite(motorInput3, LOW);
  digitalWrite(motorInput4, LOW);
  analogWrite(ENA, 0); //Left Motor Speed
  analogWrite(ENB, 0 ); //Right Motor Speed

}
void read_speed() {
  speedsensorValue = analogRead(speedpin);
  initial_motor_speed  = map(speedsensorValue, 0, 1023, 0, 255);
}
void read_kp() {
  sensorValue = analogRead(analogInPin);
  Kp  = map(sensorValue, 0, 1023, 0, 600);
}
void read_speed2() {
  sensorValue = analogRead(analogInPin);
  motor_speed  = map(sensorValue, 0, 1023, 0, 255);
}


void chay_line()
{
  read_sensor_values();
  read_back_sensor_values();
  if (error == 102) {
    if ((trai == 1) && (phai == 0)) {
      stop_bot();
      sharpLeftTurn();
      analogWrite(ENB, 50); //Left Motor Speed
      analogWrite(ENA, 200); //Right Motor Speed
      read_sensor_values();
      // digitalWrite(led,HIGH);
      Serial.print("trai   ");
      Serial.println(error);
    }
    if ((phai == 1) && (trai == 0)) {
      stop_bot();
      sharpRightTurn();
      analogWrite(ENA, 50); //Left Motor Speed
      analogWrite(ENB, 200); //Right Motor Speed
      //digitalWrite(led,HIGH);
      read_sensor_values();
      Serial.print("phai   ");
      Serial.println(error);
    }
  }

  if (error == 103)
  { kill = kill + 1;
  }
  else {
    calculate_pid();
    motor_control();
  }
}

void vao_tam(int action)
{
  switch (action)
  {
    case 1:
      stop_bot();
      read_center();
      break;
    case 2:
      //stop_bot();
      forward();
      analogWrite(ENA, motor_speed); //Left Motor Speed
      analogWrite(ENB, motor_speed); //Right Motor Speed
      read_center();
      break;
    case 3:
      stop_bot();
      reverse();
      analogWrite(ENA, motor_speed); //Left Motor Speed
      analogWrite(ENB, motor_speed); //Right Motor Speed
      read_center();
      break;
    case 4:
      stop_bot();
      sharpRightTurn();
      analogWrite(ENA, motor_speed ); //Left Motor Speed
      analogWrite(ENB, motor_speed); //Right Motor Speed
      read_center();
      break;
    case 5:
      stop_bot();
      sharpLeftTurn();
      analogWrite(ENA, motor_speed); //Left Motor Speed
      analogWrite(ENB, motor_speed); //Right Motor Speed
      read_center();
      break;

  }
}
void stop_forward()
{
  stop_bot();
  reverse();
  analogWrite(ENB, motor_speed); //Left Motor Speed
  analogWrite(ENA, motor_speed); //Right Motor
  delay(50);
  stop_bot();
}
void stop_reverse()
{
  stop_bot();
  forward();
  analogWrite(ENB, motor_speed); //Left Motor Speed
  analogWrite(ENA, motor_speed); //Right Motor
  delay(50);
  stop_bot();
}
void center ()
{
  read_center();
  if ((fix == 103) && (fix2 == 102)) {
    vao_tam(2);
    led2_blink();
    led1_off();
  }
  else if ((fix2 == 103) && (fix == 102)) {
    vao_tam(3);
    led2_blink();
    led1_off();
  }
  else if ((fix == 4) || (fix2 == 4)) {
    vao_tam(5);
    led2_blink();
    led1_off();
  }
  else if ((fix == -4) || (fix2 == -4)) {
    vao_tam(4);
    led2_blink();
    led1_off();
  }
  else if ((fix == 4) && (fix2 == 102)) {
    vao_tam(5);
    led2_blink();
    led1_off();
  }
  else if ((fix2 == -4) && (fix == 102)) {
    vao_tam(4);
    led2_blink();
    led1_off();
  }
  else if ((fix == 103) && (fix2 == 103)) {
    vao_tam(1);
    led2_on();
    led1_off();
    flat_on();
  }
  else if ((fix == 102) && (fix2 == 102)) {
    vao_tam(2);
    led1_on();
    k = 0;
  }

}
void read_kl()
{ if ((digitalRead(A11) == 1) && (digitalRead(A12) == 0))(k = 1);
  if ((digitalRead(A11) == 0) && (digitalRead(A12) == 1))(k = 1);
}
void chinh_line()
{
  if ((error == 103) && (error2 == 102))
  {
    reverse();
    analogWrite(ENB, motor_speed); //Left Motor Speed
    analogWrite(ENA, motor_speed); led1_on(); //Right Motor
    delay(100);
    read_line();
    read_kl();
  };
  if ((error2 == 103) && (error == 102))
  {
    forward();
    analogWrite(ENB, motor_speed); //Left Motor Speed
    analogWrite(ENA, motor_speed); led1_on(); //Right Motor
    delay(100);
    read_line();
    read_kl();
  } ;
  if ((error == 4) && (error2 == 102))
  {
    /*forward();
      analogWrite(ENB, motor_speed); //Left Motor Speed
      analogWrite(ENA, motor_speed);led1_on(); //Right Motor
      delay(50);*/


    left();
    analogWrite(ENB, 20);
    analogWrite(ENA, 0); led1_blink;
    delay(100);
    read_line();
    read_kl();
  };
  if ((error == -4) && (error2 == 102))
  {
    /*forward();
      analogWrite(ENB, motor_speed); //Left Motor Speed
      analogWrite(ENA, motor_speed);led1_on(); //Right Motor
      delay(50);*/


    right();
    analogWrite(ENB, 0); //Left Motor Speed
    analogWrite(ENA, 20); led1_blink(); //Right Motor
    delay(100);
    read_line();
    read_kl();
  };
  if ((error2 == -4) && (error == 102))
  {
    /*reverse();
      analogWrite(ENB, motor_speed); //Left Motor Speed
      analogWrite(ENA, motor_speed);led1_on(); //Right Motor
      delay(50);*/

    digitalWrite(motorInput1, HIGH);
    digitalWrite(motorInput2, LOW);
    digitalWrite(motorInput3, LOW);
    digitalWrite(motorInput4, LOW);
    analogWrite(ENB, 0); //Left Motor Speed
    analogWrite(ENA, 20); led1_blink(); //Right Motor
    delay(100);
    read_line();
    read_kl();
  };
  if ((error2 == 4) && (error == 102))
  { /*reverse();
      analogWrite(ENB, motor_speed); //Left Motor Speed
      analogWrite(ENA, motor_speed);led1_on(); //Right Motor
      delay(50);*/

    digitalWrite(motorInput1, LOW);
    digitalWrite(motorInput2, LOW);
    digitalWrite(motorInput3, HIGH);
    digitalWrite(motorInput4, LOW);
    analogWrite(ENB, 20); //Left Motor Speed
    analogWrite(ENA, 0); led1_blink(); //Right Motor
    delay(100);
    read_line();
    read_kl();
  };
}
void read_line()
{ read_sensor_values();
  read_back_sensor_values();
}
void cross()
{
  stop_bot();
  delay(50);
  reverse();
  analogWrite(ENB, initial_motor_speed + 5);
  analogWrite(ENA, initial_motor_speed + 5);
  delay(1100);
  stop_bot();
  //delay(50);
  sharpRightTurn();
  analogWrite(ENA, initial_motor_speed + 5);
  analogWrite(ENB, initial_motor_speed + 5);
  delay(1200);
  stop_bot();
  delay(50);
  forward();
  analogWrite(ENA, initial_motor_speed);
  analogWrite(ENB, initial_motor_speed);
  delay(50);
}
void cross_line()
{

  read_speed();
  read_speed2();
  read_line();
  if (kill == 0) {
    chay_line();
  };
  if (kill == 1)
  {
    if (c == 0) {
      cross();
      c = 1;
    }
    if (c == 1)
    {
      read_line();
      if ((error == 102) && (a == 0)) {
        a = 1;
        read_line();
      }
      if ((error == 103) && (a == 1)) {
        a = 2;
        read_line();
      }
      if ((error == 102) && (a == 2)) {
        a = 3;
        read_line();
      }
      if ((mode == 0) && (a == 3)) {
        a = 4;
        read_line();
        led2_blink();
      }
      if (a == 4) {
        stop_bot();
        sharpLeftTurn();
        analogWrite(ENA, initial_motor_speed + 5);
        analogWrite(ENB, initial_motor_speed + 5);
        delay(700);
        stop_bot();
        a = 5;
      }
      if (a == 5)  {
        chay_line();
        led1_blink();
        led2_off();
      }
    }
  }
  if (kill == 2)   {
    if (l == 0) {
      reverse(); led1_on();
      analogWrite(ENB, initial_motor_speed + 5); //Left Motor Speed
      analogWrite(ENA, initial_motor_speed + 5); //Right Motor
      delay(1200);
      stop_bot(); l = 1;
      forward();
      analogWrite(ENB, initial_motor_speed); //Left Motor Speed
      analogWrite(ENA, initial_motor_speed); //Right Motor
      delay(20);
      stop_bot();
    }
    find_opponent();
    led1_off();
    led2_blink();
    if ((time >= (85000 - 1300)) && (f == 0)) {
      forward();
      analogWrite(ENB, motor_speed); //Left Motor Speed
      analogWrite(ENA, motor_speed); //Right Motor
      delay(50); f = 1; h = 1;
    }
    if ((ob == 1) && (f == 0)) {
      forward();
      analogWrite(ENB, motor_speed); //Left Motor Speed
      analogWrite(ENA, motor_speed); //Right Motor
      delay(50); f = 1; h = 1;
    }
    if ((ob == 60) && (time >= (60000 - 1300)) && (f == 0)) {
      forward();
      analogWrite(ENB, motor_speed); //Left Motor Speed
      analogWrite(ENA, motor_speed); //Right Motor
      delay(50); f = 1; h = 1;
    }


    if ((ob == 80) && (time >= (80000 - 1300)) && (f == 0)) {
      forward();
      analogWrite(ENB, motor_speed); //Left Motor Speed
      analogWrite(ENA, motor_speed); //Right Motor
      delay(50); f = 1; h = 1;
    }
    if (h == 1) {
      read_line();
      if (error2 == 103) {
        led1_blink();
        kill = kill + 1;
      }
    }
  }

  if (kill == 3)    {
    read_kl();
    //find_obtacle();
    if (k == 0) {
      read_line(); chinh_line();
    }
    if (k == 1) {
      center();
    }
  }

}
void normal()
{
  read_speed();
  read_speed2();
  read_line();
  if (kill == 0) {
    chay_line();
  };
  if (kill == 1)   {
    forward(); led1_on();
    analogWrite(ENB, motor_speed); //Left Motor Speed
    analogWrite(ENA, motor_speed); //Right Motor
    read_line();
    if ((error == 102) && (error2 == 103)) {
      led1_blink();

      kill = 2;
    }
  }
  if (kill == 2)    {
    read_kl();

    if (k == 0) {
      read_line(); chinh_line();

    }
    if (k == 1) {
      center();
    }
  }


}
void loop()
{
  time = millis();
  read_mode();
  if (m == 1) {
    led13_on();
    cross_line();
  }
  else {
    led13_off();
    normal();
  }
}
