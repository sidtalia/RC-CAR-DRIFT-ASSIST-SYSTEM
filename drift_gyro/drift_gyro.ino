#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

volatile unsigned long timer[3];
volatile byte last_channel[2]={0,0};
volatile int input[2]={1500,1500};

int steer,throttle;  //the pid outputs
long esc_timer; //timer for the loop 
long timestamp; //timestamp
#define STEERINGPIN B11101111 //pin 4
#define THROTTLEPIN B11110111 //pin 3
#define STEERING_NULL 1500//you might have to change these if your neutral signal is different
#define THROTTLE_NULL 1500
#define Kd 1 //change this to increase or decrease the amount of "assist"

//-----ACCEL-GYRO STUFF BEGINS---------------------

MPU6050 accelgyro;

int16_t ax, ay, az;  //accelerations from mpu6050
int16_t gx, gy, gz;  //gyration rates from mpu6050

float G[3],lastA[3],offsetG[3]; //x=0,y=1,z=2.
int i;

#define SAMPLE 2000

int c;

void motorWrite()
{
  if(millis()-timestamp>=20)
  {
    timestamp=millis();//make sure we don't send the signal too soon. so take a time stamp now and compare again in the next cycle
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
{ 
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
  accelgyro.initialize();
  accelgyro.testConnection() ? c=1 : c=0;
  //calculating offsets
  for(i=0;i<SAMPLE;i++)
  {
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    offsetG[2]+=gz;
    motorWrite();//keep writing neutral signal 
  }
  offsetG[2]/=SAMPLE;
}

float mod(float a)
{
  if(a<0)
  {
    return -a;
  }
  return a;
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
    steer=input[0]+Kd*G[2];//open loop plus D controller.
    throttle=input[1]-mod(G[2]); //open loop plus D controller. 
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

