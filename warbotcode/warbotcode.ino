#define WINDOW_SIZE 3 // average value of // 10
 #define WINDOW_SIZED 3
int INDEX = 0;
float VALUE = 0;
float SUM = 0;
float READINGS[WINDOW_SIZE];
float AVERAGED = 0;
int onval=0;
int INDEXD = 0;
float VALUED = 0;
float SUMD = 0;
float READINGSD[WINDOW_SIZED];
float f_d_term = 0;
#define PPM_INPUT_PIN 7 // pwm pin to receiver ch1
#define PPM_CHANNELS 6
#include <Servo.h>
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
volatile unsigned int ppm[PPM_CHANNELS];
volatile byte currentChannel;
MPU6050 accelgyro;
int yaw_rate_max=600;
Servo rightmotor;
Servo leftmotor;
Servo throttle;
int throttleval;
int16_t ax, ay, az;
int16_t gx, gy, gz;
double gz_scaled=0;
float demanded_yaw=0;
int rollppm=1500;
int rollerror=0;
int rightmotorval=90;
int leftmotorval=90;
float roll_kp=1; // 0.1
float roll_kd=0;
float roll_ki=0.005;
float i_sum=0;
float i_limit=10;
float p_term=0;
float d_term=0;
int pitch=0;
double gyro_z[2]={0,0};
double fgyro_z[2]={0,0};
double omega = 2*3.1415926535897932384626433832795*0.001; //where f is the cutoff frequency in Hz and omega is in rad/sec
double prev_time=millis();
double sampletime=0;
void setup() {
  Wire.begin();
  Wire.setClock(400000);
//  Wire.setWireTimeout(10000,true);
  accelgyro.initialize();
  rightmotor.attach(5); // pwm pins to brushed ESC
  leftmotor.attach(6);// pwm pins to brushed ESC
  throttle.attach(9);// weapon motor
  rightmotor.write(90);
  leftmotor.write(90);
  //throttle.write(0);
  pinMode(PPM_INPUT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PPM_INPUT_PIN), readPPM, RISING);
  //Serial.begin(115200);

  //Serial.setTimeout(1);


leftmotor.writeMicroseconds(1500);
rightmotor.writeMicroseconds(1500);
throttle.writeMicroseconds(1000);
delay (5000);

  
}

void loop() {
  // Your code here
 
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  sampletime=(millis()-prev_time)/1000;
  prev_time=millis();
  throttleval=ppm[2];

if (ppm[5]<100)
{
  rightmotor.writeMicroseconds(1500);
  leftmotor.writeMicroseconds(1500);
  
}



if (900<ppm[5]<1050)

 {
  if (ppm[4]<1500)
   //if (az >0)
  {
  gz_scaled=-((double)gz+240)/131;
  }
  else
  {
  gz_scaled=((double)gz+240)/131;
  }
 
}

 if (ppm[5]>1800)
  {
    gz_scaled=0;
     //Serial.println(gz_scaled);
    }
  SUM = SUM - READINGS[INDEX];       // Remove the oldest entry from the sum
  VALUE = gz_scaled;  // Read the next sensor
 
 
  READINGS[INDEX] = VALUE;           // Add the newest reading to the window
  SUM = SUM + VALUE;                 // Add the newest reading to the sum
  INDEX = (INDEX+1) % WINDOW_SIZE;   // Increment the index, and wrap to 0 if it exceeds the window size
  fgyro_z[0] = SUM / WINDOW_SIZE;      // Divide the sum of the window by the window size for the result-
  d_term=(fgyro_z[0]-fgyro_z[1])*roll_kd;

  SUMD = SUMD - READINGSD[INDEXD];       // Remove the oldest entry from the sum
  VALUED = d_term;        // Read the next sensor value
  READINGSD[INDEXD] = VALUED;           // Add the newest reading to the window
  SUMD = SUMD + VALUED;                 // Add the newest reading to the sum
  INDEXD = (INDEXD+1) % WINDOW_SIZED;   // Increment the index, and wrap to 0 if it exceeds the window size
  f_d_term = SUMD / WINDOW_SIZED;      // Divide the sum of the window by the window size for the result-
  //Serial.println(d_term,6);
  fgyro_z[1]=fgyro_z[0];
  gyro_z[1]=gyro_z[0];
  //Serial.print("Roll-");
 // Serial.println(ppm[5]);
 
 
 
   rollppm=map(ppm[0],1972,1032,-yaw_rate_max,yaw_rate_max);// normal
 
 
  demanded_yaw=(float)rollppm;
  p_term=(demanded_yaw-fgyro_z[0]-3)*roll_kp;
  //i_sum=i_sum+((demanded_yaw-fgyro_z[0]-3)*roll_ki);
  if(i_sum>i_limit)
  {
    i_sum=i_limit;
  }
  if(i_sum<-i_limit)
  {
    i_sum=-i_limit;
  }
  rollerror=p_term-f_d_term+i_sum;
  //Serial.print(d_term);Serial.print(',');Serial.println(f_d_term);
  //if (az >0)
  if (ppm[4]>1500)
  {
  pitch=map(ppm[1],1976,1020,-500,500);
   gz_scaled=-((double)gz+240)/131;
  }
  
  else
  {
  pitch=map(ppm[1],1976,1020,500,-500);
  }
  rightmotorval=1500-pitch;
    if(rightmotorval<1000)
  {
    rightmotorval=1000;
  }
  if(rightmotorval>2000)
  {
    rightmotorval=2000;
  }


 
  rightmotorval+=rollerror;


 
  if(rightmotorval<1000)
  {
    rightmotorval=1000;
  }
  if(rightmotorval>2000)
  {
    rightmotorval=2000;
  }
  leftmotorval=1500-pitch;
  if(leftmotorval<1000)
  {
    leftmotorval=1000;
  }
  if(leftmotorval>2000)
  {
    leftmotorval=2000;
  }

   
  leftmotorval-=rollerror;
 

 
  if(leftmotorval<1000)
  {
    leftmotorval=1000;
  }
  if(leftmotorval>2000)
  {
    leftmotorval=2000;
  }

     
   if (ppm[5]>100)
   {
  rightmotor.writeMicroseconds(rightmotorval);
  leftmotor.writeMicroseconds(leftmotorval);
  throttle.write(throttleval);
   }
  //Serial.println(throttleval);//Serial.println(az);
  //Serial.println(gz_scaled);
  //Serial.print(fgyro_z[0],6);Serial.print(',');Serial.println(gz_scaled,6);
  //delay(10);
 
 }

void readPPM() {
  static unsigned int startTime;
  unsigned int pulseWidth = micros() - startTime;
  startTime = micros();

  if (pulseWidth > 3000) {
    currentChannel = 0;
  } else if (currentChannel < PPM_CHANNELS) {
    ppm[currentChannel++] = pulseWidth;
  }
}
float lpf(double prev_filtered,double prev_unfiltered,double current_val,double prev_val, double T, double omega)
{
 
float alpha = (2-T*omega)/(2+T*omega); //Where T is sample time in seconds
float beta = T*omega/(2+T*omega);
//Low pass filter equation:
float current_filtered = alpha*prev_val + beta*(current_val+prev_unfiltered); //y is filtered value and x is unfiltered sample
return current_filtered;
}