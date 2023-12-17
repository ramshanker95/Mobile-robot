/* BLDC Hall Sensor read and calculation program for Teensy 3.5 in the Arduino IDE (Ver.1). Digi-Key Electronics*/

/***************************** Variables *********************************/
#include <Ultrasonic.h>
#define INTERRUPT_MODE FALLING

#define CW -1  // Assign a value to represent clock wise rotation
#define CCW 1  // Assign a value to represent counter-clock wise rotation

int direct = 1;  // Integer variable to store BLDC rotation direction
int direct_1 = 1;

struct Point {
    int xval;
    int yval;
    };

#define ro_pin PB1
#define gd_pin PB0
#define yc_pin PA7 

const int trigPin1 = PC14;
const int echoPin1 = PC15;
int Limit_Dist = 65;
Ultrasonic ultrasonic1(trigPin1, echoPin1);
#define PWM_left PB7
#define PWM_right PB8      //       pwm  
#define dir_left PB12
#define en_left PB13
#define dir_right PB14
#define en_right PB15

#define encoderpin_left_u PA6
#define encoderpin_left_v PA5
#define encoderpin_left_w PA4

#define encoderpin_right_u PA3
#define encoderpin_right_v PA2
#define encoderpin_right_w PA1

#define obj_c PB4
#define obj_l PB3
#define obj_r PA15
#define no_obj PB5
#define dest PB6


volatile long pulse_count[] = {0, 0};
volatile long pulse_count_pre[] = {0, 0};

bool HSU_Val = digitalRead(encoderpin_left_u);  // Set the U sensor value as boolean and read initial state
bool HSV_Val = digitalRead(encoderpin_left_v);  // Set the V sensor value as boolean and read initial state
bool HSW_Val = digitalRead(encoderpin_left_w);  // Set the W sensor value as boolean and read initial state

bool HSU_Val_1 = digitalRead(encoderpin_right_u);
bool HSV_Val_1 = digitalRead(encoderpin_right_v);
bool HSW_Val_1 = digitalRead(encoderpin_right_w); 


void left_motor_dir(char ch)
{
  if (ch == 'f') //forward
  {
    digitalWrite(dir_left, LOW); //low=f, high=b
  }
  else  if (ch == 'b') //backward
  {
    digitalWrite(dir_left, HIGH); //low=f, high=b
  }
}

void right_motor_dir(char ch)
{
  if (ch == 'f')
  {
    digitalWrite(dir_right, HIGH);
  }
  else  if (ch == 'b')
  {
    digitalWrite(dir_right, LOW);
  }
}

void set_pwm(int pwm1, int pwm2)
{
  analogWrite(PWM_right, pwm2);
  analogWrite(PWM_left, pwm1);
}

void enable()
{
  digitalWrite(en_right, LOW);
  digitalWrite(en_left, LOW);
}

void desable()
{
  digitalWrite(en_right, HIGH);
  digitalWrite(en_left, HIGH);
}

// Function to calculate the angle between two coordinates
float calculateAngle(float x1, float y1, float x2, float y2) {
  // Calculate the angle in radians
  float angleRad = atan2(y2 - y1, x2 - x1);

  // Convert the angle from radians to degrees
  float angleDeg = angleRad * 180.0 / M_PI;

  // Ensure the angle is positive
  if (angleDeg < 0) {
    angleDeg += 360.0;
  }

  return angleDeg;
}

//-----------------------------------

void HallSensorW() {
  HSW_Val = digitalRead(encoderpin_left_w);      // Read the current W hall sensor value
  HSV_Val = digitalRead(encoderpin_left_v);      // Read the current V (or U) hall sensor value
  direct = (HSW_Val == HSV_Val) ? CCW : CW;  // Determine rotation direction (ternary if statement)
  pulse_count[0] = pulse_count[0] + (1 * direct);    // Add 1 to the pulse count
}
void HallSensorV() {
  HSV_Val = digitalRead(encoderpin_left_v);
  HSU_Val = digitalRead(encoderpin_left_u);  // Read the current U (or W) hall sensor value
  direct = (HSV_Val == HSU_Val) ? CCW : CW;
  //  Serial.println(direct);
  pulse_count[0] = pulse_count[0] + (1 * direct);
}
void HallSensorU() {
  HSU_Val = digitalRead(encoderpin_left_u);
  HSW_Val = digitalRead(encoderpin_left_w);  // Read the current W (or V) hall sensor value
  direct = (HSU_Val == HSW_Val) ? CCW : CW;
  pulse_count[0] = pulse_count[0] + (1 * direct);
}
/************************ Interrupt Functions 2***************************/
void HallSensorW_1() {
  HSW_Val_1 = digitalRead(encoderpin_right_w);          // Read the current W hall sensor value
  HSV_Val_1 = digitalRead(encoderpin_right_v);          // Read the current V (or U) hall sensor value
  direct_1 = (HSW_Val_1 == HSV_Val_1) ? CW : CCW;  // Determine rotation direction (ternary if statement)
  pulse_count[1] = pulse_count[1] + (1 * direct_1);    // Add 1 to the pulse count
}
void HallSensorV_1() {
  HSV_Val_1 = digitalRead(encoderpin_right_v);
  HSU_Val_1 = digitalRead(encoderpin_right_u);  // Read the current U (or W) hall sensor value
  direct_1 = (HSV_Val_1 == HSU_Val_1) ? CW : CCW;
  pulse_count[1] = pulse_count[1] + (1 * direct_1);
}
void HallSensorU_1() {
  HSU_Val_1 = digitalRead(encoderpin_right_u);
  HSW_Val_1 = digitalRead(encoderpin_right_w);  // Read the current W (or V) hall sensor value
  direct_1 = (HSU_Val_1 == HSW_Val_1) ? CW : CCW;
  pulse_count[1] = pulse_count[1] + (1 * direct_1);
}
//----------------------------------
void stopmotor()
{
  for(int i=8;i>0;i--)
  {
    set_pwm(i*i, (i-(i/30.332))*(i-(i/30.332)));
    delay(50);
  }
  desable();
  delay(100);
}

void fstopmotor()
{
  for(int i=12;i>0;i--)
  {
    set_pwm(i*i,(i-(i/60.3))*(i-(i/60.3)));
    delay(50);
  }
  desable();
  delay(100);
}
void smotor()
{
  set_pwm(0, 0);
  desable();
  delay(100);
}

void forward()
{
  right_motor_dir('f');
  left_motor_dir('f');
  enable();
  for(int i=2;i<=13;i++)
  {
    set_pwm(i*i,(i+(i/60.2))*(i+(i/60.2)));// i+(i/33.34)
    delay(50);
  }
}
void backward()
{
  right_motor_dir('b');
  left_motor_dir('b');
  set_pwm(70, 70);
  enable();
}

void left()
{
  right_motor_dir('f');
  left_motor_dir('b');
  set_pwm(50, 50);
  enable();
}

void right()
{
  right_motor_dir('b');
  left_motor_dir('f');
  set_pwm(50, 50);
  enable();
}
