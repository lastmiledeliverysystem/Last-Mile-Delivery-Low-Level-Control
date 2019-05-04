// the sensor communicates using SPI, so include the library:

#include <math.h>


// Global Variables
int forward_right = 0;
int forward_left = 0;
float v,w;

int data = 0;
float received [4];
int counter = 0;

#define L 0.25
#define R  0.06

#define maxVoltR 8.16
#define maxVoltL 8.18

#define minVoltR 0.96
#define minVoltL 0.96

// m/s
#define maxWheelVelocity 1.0327
#define minWheelVelocity 0.25

// rad/s
#define maxAngularVelocity 9.58498
#define minAngularVelocity 0.690324

// Motors Forward & Backward Enable pins
#define MOTOR_RIGHT_F 2
#define MOTOR_RIGHT_B 4
#define MOTOR_LEFT_F  7
#define MOTOR_LEFT_B  8

// PWM pins
#define MOTOR_RIGHT_PWM_F 5
#define MOTOR_RIGHT_PWM_B 3
#define MOTOR_LEFT_PWM_F 9
#define MOTOR_LEFT_PWM_B 6


void setup() {
  
  Serial.begin(9600);

  // Setting PWM pins as output
 pinMode(MOTOR_RIGHT_PWM_F, OUTPUT);
 pinMode(MOTOR_RIGHT_PWM_B, OUTPUT);
 pinMode(MOTOR_LEFT_PWM_F, OUTPUT);
 pinMode(MOTOR_LEFT_PWM_B, OUTPUT);

  //Setting Enable pins as output
 pinMode(MOTOR_RIGHT_F, OUTPUT);
 pinMode(MOTOR_RIGHT_B, OUTPUT);
 pinMode(MOTOR_LEFT_F, OUTPUT);
 pinMode(MOTOR_LEFT_B, OUTPUT);
 
  //Making all Enable pins High
 digitalWrite(MOTOR_RIGHT_F, HIGH);
 digitalWrite(MOTOR_RIGHT_B, HIGH);
 digitalWrite(MOTOR_LEFT_F, HIGH);
 digitalWrite(MOTOR_LEFT_B, HIGH);
  
 // give the sensor time to set up:
  delay(100);
}


void loop() {

  if (Serial.available() > 0){ 
   
    data = Serial.read();
    received [counter++] = data;
   
    if (counter == 4)
    {

      counter = 0;
      
      v = (float)(received [0] / (float) 100);
      if  (received [1] == 1)
      {
        v = -1 * v;
      }
      
      w = (float)(received[2] / (float) 10 );
      if  (received [3] == 1)
      {
        w = -1 * w;
       
      }
      calculate_velocities(v,w);
    }
  }
}

void calculate_velocities(float v,float w)
{

  float rightVelocity ,leftVelocity; // m/s
  float norm;

  if (v == 0.0 && w == 0.0) //Stop Motors
  {
    rightVelocity = leftVelocity = 0.0;
  }
  else
  { 
    // Ready to be applied (m/s)
    rightVelocity = (v + (w * L /2.0));  
    leftVelocity = (v - (w * L /2.0));

    if (fabsf(rightVelocity) < minWheelVelocity || fabsf(leftVelocity) < minWheelVelocity)
    {
      norm = minWheelVelocity / min(fabsf(rightVelocity),fabsf(leftVelocity));
      rightVelocity = rightVelocity * norm;
      leftVelocity = leftVelocity * norm;
    }
    if (fabsf(rightVelocity) > maxWheelVelocity || fabsf(leftVelocity) > maxWheelVelocity)
    {
      norm = maxWheelVelocity / max(fabsf(rightVelocity),fabsf(leftVelocity));
      rightVelocity = rightVelocity * norm;
      leftVelocity = leftVelocity * norm;
    }
    
    if (rightVelocity < 0.0)
    {
       forward_right = 0;
    }
    else 
    {
      forward_right = 1;
    }

    if (leftVelocity < 0.0)
    {
      forward_left = 0;
    }
    else
    {
      forward_left = 1;
    }
    
  }
  
  calculate_voltages(fabsf(rightVelocity),fabsf(leftVelocity));
}

void calculate_voltages(float velocityR,float velocityL)
{
  float voltR,voltL;
 
  voltR = (0.2042 * pow(velocityR,2)) + (6.2299 * velocityR) + 0.4462;
  voltL = (0.2047 * pow(velocityL,2)) + (6.2452 * velocityL) + 0.4473;
  
  if ( velocityR == 0.0 )
  {
    voltR = 0.0;
  }
  if ( velocityL == 0.0 )
  {
    voltL = 0.0;
  }

  calculate_pwms(voltR, voltL);
}


void calculate_pwms (float voltR, float voltL)
{  
  int pwmR, pwmL;
  
  pwmR = (int)((voltR / maxVoltR) * 255);
  pwmL = (int)((voltL / maxVoltL) * 255);
  
  // Applying pwm to the the Right-side motors
   if (forward_right)
  {
    analogWrite(MOTOR_RIGHT_PWM_F, pwmR);
    digitalWrite(MOTOR_RIGHT_PWM_B, LOW);
  }
  else
  {
    analogWrite(MOTOR_RIGHT_PWM_B, pwmR);
    digitalWrite(MOTOR_RIGHT_PWM_F, LOW);
  }

  // Applying pwm to the the Left-side motors
  if (forward_left)
  {
    analogWrite(MOTOR_LEFT_PWM_F, pwmL);
    digitalWrite(MOTOR_LEFT_PWM_B, LOW);
  }
  else 
  {
    analogWrite(MOTOR_LEFT_PWM_B, pwmL);
    digitalWrite(MOTOR_LEFT_PWM_F, LOW);
  }
  
}

