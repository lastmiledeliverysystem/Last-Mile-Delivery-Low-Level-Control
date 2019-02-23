// the sensor communicates using SPI, so include the library:


#define L 0.42
#define R  0.06

#define maxVolt 6.3
#define maxLinearVelocity 110
#define maxVelocity 127
#define maxW (PI/3.0)

// Motors Forward & Backward Enable pins
#define MOTOR_RIGHT_F 2
#define MOTOR_RIGHT_B 4
#define MOTOR_LEFT_F  12
#define MOTOR_LEFT_B  13


// PWM pins
#define MOTOR_RIGHT_PWM_F 3
#define MOTOR_RIGHT_PWM_B 5
#define MOTOR_LEFT_PWM_F 10
#define MOTOR_LEFT_PWM_B 11

// Global Variables
int forward_right = 0;
int forward_left = 0;
float v,w;
float msg;
float received [4];
int counter = 0;



void setup() {
  Serial.begin(9600);

  // Setting PWM pins as output
//  pinMode(MOTOR_RIGHT_PWM_F, OUTPUT);
//  pinMode(MOTOR_RIGHT_PWM_B, OUTPUT);
//  pinMode(MOTOR_LEFT_PWM_F, OUTPUT);
//  pinMode(MOTOR_LEFT_PWM_B, OUTPUT);

  // Setting Enable pins as output
//  pinMode(MOTOR_RIGHT_F, OUTPUT);
//  pinMode(MOTOR_RIGHT_B, OUTPUT);
//  pinMode(MOTOR_LEFT_F, OUTPUT);
//  pinMode(MOTOR_LEFT_B, OUTPUT);
//  
  // Making all Enable pins High
//  digitalWrite(MOTOR_RIGHT_F, HIGH);
//  digitalWrite(MOTOR_RIGHT_B, HIGH);
//  digitalWrite(MOTOR_LEFT_F, HIGH);
//  digitalWrite(MOTOR_LEFT_B, HIGH);
  
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
//    Serial.print(msg);
//    Serial.print("\n");
    received [counter++] = msg;
   
    if (counter == 4)
    {
      counter = 0;
      
      v = received [0];
      if  (received [1] == 1)
      {
        v = -1 * v;
      }
      
      w = received[2];
      w = w * PI / 180.0;
      if  (received [3] == 1)
      {
        w = -1 * w;
      }
      Serial.print(v);
      Serial.print("\n");
      Serial.print(w);
      Serial.print("\n");
      calculate_velocities(v,w);
    }
  }
}


void calculate_velocities(float v,float w)
{
  float rightVelocity ,leftVelocity;
  
  //Stop Motors
  if (v == 0.0 && w == 0.0)
  {
    rightVelocity = leftVelocity = 0.0;
  }
  else
  { 
    //threshold
    constrain(v, -maxLinearVelocity, maxLinearVelocity);
    constrain(w, -maxW, maxW);

    // Output --> RPM
    rightVelocity = (v + (w * L /2.0)) * (30.0 / (PI * R)); 
    leftVelocity = (v - (w * L /2.0)) * (30.0 / (PI * R)); 

    // Output --> m/s
    //rightVelocity = ((v * PI * R /30.0 ) + (w * L /2.0)) * (30.0 / (PI * R));
    //leftVelocity = ((v * PI * R /30.0 ) - (w * L /2.0)) * (30.0 / (PI * R));
  
    if (rightVelocity <= 0.0)
    {
       forward_right = 0;
    }
    else
    {
      forward_right = 1;
    }
    if (leftVelocity <= 0.0)
    {
      forward_left = 0;
    }
    else
    {
      forward_left = 1;
    }
    
    //threshold
    if (rightVelocity > maxVelocity)
    {
      rightVelocity = maxVelocity;
    }
    else if (rightVelocity < (-1 * maxVelocity)) 
    {
      rightVelocity = -1 * maxVelocity;
    }
    if (leftVelocity > maxVelocity) 
    {
      leftVelocity = maxVelocity;
    }
    if (leftVelocity < (-1 * maxVelocity)) 
    {
      leftVelocity = -1 * maxVelocity;
    }
    
  }
  
  calculate_voltages(fabsf(rightVelocity),fabsf(leftVelocity));
}

void calculate_voltages(float velocityR,float velocityL)
{
  float voltR,voltL;
  
  voltL = (0.04175 * velocityR) + 0.59855;
  voltR = (0.04205 * velocityL) + 0.6013;
  
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
