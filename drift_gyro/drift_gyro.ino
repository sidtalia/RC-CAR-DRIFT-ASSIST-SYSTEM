#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

volatile unsigned long timer[3];
volatile byte last_channel[2]={0,0};
volatile int input[2]={1500,1500};

int steer,throttle;  //servo objects
long esc_timer;
long timestamp;
#define STEERINGPIN B11101111
#define THROTTLEPIN B11110111
#define STEERING_NULL 1460
#define THROTTLE_NULL 1460
#define Kd 1

//-----ACCEL-GYRO STUFF BEGINS---------------------

MPU6050 accelgyro;

int16_t ax, ay, az;  //accelerations from mpu6050
int16_t gx, gy, gz;  //gyration rates from mpu6050

float G[3],lastA[3],offsetG[3]; //x=0,y=1,z=2, T=tilt,V=velocity ,X=position on X axis,Y=position on Y axis,Ha=horizontall acceleration along Y direction
int i;

#define SAMPLE 2000

int c;

inline void motorWrite()
{
  if(millis()-timestamp>=20)
  {
    timestamp=millis();
    PORTD |= B00011000;
    esc_timer=micros();
    while(PORTD>8)
    {
      if(micros()>=(steer+esc_timer))
      {
       PORTD &= STEERINGPIN;
      }
      if(micros()>=(throttle+esc_timer))
      {
       PORTD &= THROTTLEPIN;
      }
    }
  }
}

void setup() 
{    // join I2C bus (I2Cdev library doesn't do this automatically)
  //Serial.begin(9600);
  PCICR |= (1 << PCIE0);   
  PCMSK0 |= (1 << PCINT0); //8
  PCMSK0 |= (1 << PCINT1); //9

  //-----------THROTTLE STEERING SETUP DONE-------
  DDRD |= B00011000;    //setting pins 3 and 4 as output
  PORTD |= B00011000;   //pull them high and make them fly? 
  delayMicroseconds(THROTTLE_NULL);    
  PORTD &= B11100111;    //pull them low  
  timestamp=millis();
  throttle=THROTTLE_NULL;
  steer=STEERING_NULL;
  //-----------ACCELGYRO SETUP BEGINS-------------
  Wire.setClock(800000);
  Wire.begin();
//  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();
  
  // verify connection
//  Serial.println("Testing device connections...");
  accelgyro.testConnection() ? c=1 : c=0;
  //calculating offsets
  for(i=0;i<SAMPLE;i++)
  {
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    offsetG[2]+=gz;
    motorWrite();
  }
  for(i=0;i<3;i++)
  {
    offsetG[i]/=SAMPLE;
  }
}


float mod(float a)
{
  return sqrt(a*a);
}

long loop_timer;

void loop() 
{   
    loop_timer = millis()+20;
    while(!c)
    { 
      accelgyro.testConnection()? c=1 : c=0;
    }    
    callimu();
    steer=input[0]+Kd*G[2];
    throttle=input[1]-mod(G[2]);
    motorWrite();
    while(millis()-loop_timer<0); //50hz update rate. 
}


ISR(PCINT0_vect)
{
  timer[0]=micros();
  //channel 1 ----
  
  if(last_channel[0]==0&& PINB & B00000001) //makes sure that the first pin was initially low and is now high
  {                                         //PINB & B00000001 is equivalent to digitalRead but faster
    last_channel[0]=1;
    timer[1]=timer[0];          
  }
  else if(last_channel[0]==1 && !(PINB & B00000001))
  {
    last_channel[0]=0;
    input[0]=timer[0]-timer[1];
  }

  //channel 2---
  if(last_channel[1]==0&& PINB & B00000010) //makes sure that the first pin was initially low and is now high
  {                                         //PINB & B00000001 is equivalent to digitalRead but faster
    last_channel[1]=1;
    timer[2]=timer[0];          
  }
  else if(last_channel[1]==1 && !(PINB & B00000010))
  {
    last_channel[1]=0;
    input[1]=timer[0]-timer[2];
  }
}

