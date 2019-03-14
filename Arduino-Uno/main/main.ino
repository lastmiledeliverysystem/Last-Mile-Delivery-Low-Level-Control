// the sensor communicates using SPI, so include the library:

#include <math.h>


#define L 0.25
#define R  0.06

#define maxVolt 6.3
#define minVolt 1.95

// m/s
#define maxWheelVelocity 0.797
#define minWheelVelocity 0.239

// rad/s
#define maxAngularVelocity 6.376
#define minAngularVelocity 1.912

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

// Global Variables
int forward_right = 0;
int forward_left = 0;
float v,w;
float previousW = 0.0;
float preiousV = 0.0;
float msg;
float received [4];
int counter = 0;



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
  
  // start the SPI library in slave mode
  //pinMode(MISO, OUTPUT);
  pinMode(MOSI, INPUT);
  pinMode(SS,   INPUT);
  pinMode(SCK,  INPUT);
  
  SPCR |= _BV(SPE);  
  
  // give the sensor time to set up:
  delay(100);
}


void loop() {
  if((SPSR & (1 << SPIF)) != 0)
  {
    msg = SPDR;
    
    received [counter++] = msg;
   
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

      calculate_velocities(v,w);
    }
  }
}


void calculate_velocities(float v,float w)
{
  float rightVelocity ,leftVelocity; // rpm
  float vR, vL; // m/s

  if (v == 0.0 && w == 0.0) //Stop Motors
  {
    rightVelocity = leftVelocity = 0.0;
  }
  else
  { 
    if (v == 0) // Pure Angular Velocity
    {
      // out of allowable boundaries
      if ( fabsf(w) < minAngularVelocity || fabsf(w) > maxAngularVelocity)
      {
        // apply previous action
//        v = preiousV;
//        w = previousW;
      }
    }
    else if (w == 0) // Pure Translation
    {
      // out of allowable boundaries
      if ( fabsf(v) < minWheelVelocity || fabsf(v) > maxWheelVelocity)
      {
        // apply previous action
//        v = preiousV;
//        w = previousW;
      }
    }

    vR = (v + (w * L /2.0));  
    vL = (v - (w * L /2.0));

    if (vR == 0) // taking curve with the only left wheels 
    {
      // out of allowable boundaries
      if (fabsf(vL) < minWheelVelocity || fabsf(vL) > maxWheelVelocity)
      {
        //apply previous action
//        v = preiousV;
//        w = previousW;
      }
    }
    else if (vL == 0) // taking curve with the only right wheels 
    {
      // out of allowable boundaries
      if (fabsf(vR) < minWheelVelocity || fabsf(vR) > maxWheelVelocity)
      {
        //apply previous action
//        v = preiousV;
//        w = previousW;
      }
    }
    else // regular movement, both sides are moving with v and w
    {
      if (fabsf(vL) < minWheelVelocity || fabsf(vR) > maxWheelVelocity)
      {
        //apply previous action
//        v = preiousV;
//        w = previousW;  
      }
    }
    
    // Ready to be applied (rpm)
    rightVelocity = (v + (w * L /2.0)) * (30.0 / (PI * R) );  
    leftVelocity = (v - (w * L /2.0)) * (30.0 /  (PI * R) );

    // Saving State
    preiousV = v;
    previousW = w;
    
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
  
  voltR = (0.04175 * velocityR) + 0.59855;
  voltL = (0.04205 * velocityL) + 0.6013;
  
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
  
  pwmR = (int)((voltR /  maxVolt) * 255);

  pwmL = (int)((voltL /  maxVolt) * 255);
  
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
