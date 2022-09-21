 
/*************************************************** 
  This is an example for our Adafruit 16-channel PWM & Servo driver
  Servo test - this will drive 8 servos, one after the other on the
  first 8 pins of the PCA9685

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products/815
  
  These drivers use I2C to communicate, 2 pins are required to  
  interface.

  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// you can also call it with a different address you want
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);
// you can also call it with a different address and I2C interface
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire);

// Depending on your servo make, the pulse width min and max may vary, you 
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!
#define SERVOMIN  0 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  535 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

// our servo # counter
// x,y,z three dimensional coordinate
///A,B,C three angles of triangle made bye leg
// a,b constant length of leg
// c is initial height
//d is final length/side of triangle
//z is the height from ground

float c[4],d[4],x[4],z[4],A[4],B[4],C[4],D[4],thetaX[4],phi[4],y[4],n, ellipse_a=4 , ellipse_b=2 ;  // s=radius, A,B,C triangle angles, a,b,c triangle sides, z is height from base
int servoposition[16]={180,180,170,0,180,180,90,0,180,80,90,0,180,180,187,0};
int a=7.5, b=13,i;

void setup() {
  Serial.begin(9600);

  pwm.begin();
  
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
  c[0]=c[1]=c[2]=c[3]=14;
  x[0]=x[1]=x[2]=x[3]=0;
  z[0]=z[1]=z[2]=z[3]=0;
  y[0]=y[1]=y[2]=y[3]=0;
  Serial.println("read for command");

  delay(10);
}


int angletopulse(int ang){
  int pulse= map(ang,0,270,SERVOMIN,SERVOMAX);
  return pulse;
}

int mg995(int ang){
  int pulse=map(ang,0,200,70,510);
  return pulse;
}

int IK(){
  
  
  for(int n=0;n<4;n++)
  {
    thetaX[n]=atan(x[n]/c[n]);
    phi[n]=atan(y[n]/c[n]);
    d[n]=sqrt(pow(x[n],2)+pow(y[n],2)+pow((c[n]-z[n]),2));
  }
  
  for(int n=0;n<4;n++)
  {
  A[n]=acos((pow(b,2)+pow(d[n],2)-pow(a,2))/(2*b*d[n]));
  B[n]=acos((pow(a,2)+pow(d[n],2)-pow(b,2))/(2*a*d[n]));
  C[n]=acos((pow(b,2)+pow(a,2)-pow(d[n],2))/(2*b*a));
  
  
  B[n]=-thetaX[n]+B[n]; //Komabo B and add korbo theta
  
  }


   pwm.setPWM(12,0,angletopulse(servoposition[12]+(phi[3]*180/3.1416)));
    pwm.setPWM(13,0,angletopulse(servoposition[13]-(B[3]*180/3.1416)));
    pwm.setPWM(14,0,angletopulse(servoposition[14]-180+(C[3]*180/3.1416)));

    pwm.setPWM(4,0,angletopulse(servoposition[4]+(phi[1]*180/3.1416)));
    pwm.setPWM(5,0,angletopulse(servoposition[5]+(B[1]*180/3.1416)));
    pwm.setPWM(6,0,angletopulse(servoposition[6]+180-(C[1]*180/3.1416)));


    pwm.setPWM(0,0,angletopulse(servoposition[0]-(phi[0]*180/3.1416)));
    pwm.setPWM(1,0,angletopulse(servoposition[1]-(B[0]*180/3.1416)));
    pwm.setPWM(2,0,angletopulse(servoposition[2]-180+(C[0]*180/3.1416)));

   
    pwm.setPWM(8,0,angletopulse(servoposition[8]-(phi[2]*180/3.1416)));
    pwm.setPWM(9,0,angletopulse(servoposition[9]+(B[2]*180/3.1416)));
    pwm.setPWM(10,0,angletopulse(servoposition[10]+180-(C[2]*180/3.1416)));
  
}

// You can use this function if you'd like to set the pulse length in seconds
// e.g. setServoPulse(0, 0.001) is a ~1 millisecond pulse width. It's not precise!
void setServoPulse(uint8_t n, double pulse) {
  double pulselength;
  
  pulselength = 1000000;   // 1,000,000 us per second
  pulselength /= SERVO_FREQ;   // Analog servos run at ~60 Hz updates
  Serial.print(pulselength); Serial.println(" us per period"); 
  pulselength /= 4096;  // 12 bits of resolution
  Serial.print(pulselength); Serial.println(" us per bit"); 
  pulse *= 1000000;  // convert input seconds to us
  pulse /= pulselength;
  Serial.println(pulse);
  pwm.setPWM(n, 0, pulse);
}






void loop() {
  if(Serial.available()==0)
  {
    IK();
    
  }
  else
  {
    char command=Serial.read();
    if(command=='m') //WALK FORWARD
    {
      
     
      
      for(x[0]=-ellipse_a,x[1]=-ellipse_a,x[2]=ellipse_a,x[3]=ellipse_a;x[0]<=ellipse_a,x[1]<=ellipse_a,x[2]>=-ellipse_a,x[3]>=-ellipse_a;x[0]+=0.5,x[1]+=0.5,x[2]-=0.5,x[3]-=0.5)
      {
        z[0]=ellipse_b*sqrt(1-(pow(x[0],2)/pow(ellipse_a,2)));
        z[1]=ellipse_b*sqrt(1-(pow(x[1],2)/pow(ellipse_a,2)));
        IK();
        delay(5);
        
        
      }
      
      delay(200);
      
      

      for(x[0]=ellipse_a,x[1]=ellipse_a,x[2]=-ellipse_a,x[3]=-ellipse_a;x[0]>=-ellipse_a,x[1]>=-ellipse_a,x[2]<=ellipse_a,x[3]<=ellipse_a;x[0]-=0.5,x[1]-=0.5,x[2]+=0.5,x[3]+=0.5)
      {
        z[2]=ellipse_b*sqrt(1-(pow(x[2],2)/pow(ellipse_a,2)));
        z[3]=ellipse_b*sqrt(1-(pow(x[3],2)/pow(ellipse_a,2)));
        IK();
        
        
        delay(5);
        
      }
      delay(200);
     
    }

    
    else if(command=='n') //NORMAL
    {
      x[0]=x[1]=x[2]=x[3]=0;
  z[0]=z[1]=z[2]=z[3]=0;
  y[0]=y[1]=y[2]=y[3]=0;
  IK;
    }

   else if( command=='h') //high position
    {
      c[0]=c[1]=c[2]=c[3]=15;
      IK();
      delay(200);
    }

    else if( command=='l') //low position
    {
      c[0]=c[1]=c[2]=c[3]=13;
      IK();
      delay(200);
    }


    else if( command=='4') //medium position
    {
      c[0]=c[1]=c[2]=c[3]=14;
      IK();
      delay(200);
    }
   

    else if(command=='r') //DANCE SIDE BY SIDE
    {
      for(y[0]=0,y[1]=0,y[2]=0,y[3]=0;y[0]<=4,y[1]<=4,y[2]<=4,y[3]<=4;y[0]+=.5,y[1]+=.5,y[2]+=.5,y[3]+=.5)
      {
        IK();
        delay(30);
      }
      for(y[0]=4,y[1]=4,y[2]=4,y[3]=4;y[0]>=-4,y[1]>=-4,y[2]>=-4,y[3]>=-4;y[0]-=.5,y[1]-=.5,y[2]-=.5,y[3]-=.5)
      {
        IK();
        delay(30);
      }
      for(y[0]=-4,y[1]=-4,y[2]=-4,y[3]=-4;y[0]<=0,y[1]<=0,y[2]<=0,y[3]<=0;y[0]+=.5,y[1]+=.5,y[2]+=.5,y[3]+=.5)
      {
        IK();
        delay(30);
      }
      
      }
      

      else if(command=='b') //BACKWARD
    {
      
     
      
      for(x[0]=ellipse_a,x[1]=ellipse_a,x[2]=-ellipse_a,x[3]=-ellipse_a;x[0]>=-ellipse_a,x[1]>=-ellipse_a,x[2]<=ellipse_a,x[3]<=ellipse_a;x[0]-=0.5,x[1]-=0.5,x[2]+=0.5,x[3]+=0.5)
      {
        z[0]=ellipse_b*sqrt(1-(pow(x[0],2)/pow(ellipse_a,2)));
        z[1]=ellipse_b*sqrt(1-(pow(x[1],2)/pow(ellipse_a,2)));
        IK();
        delay(5);
        
        
      }
      
      delay(200);
      
      

      for(x[0]=-ellipse_a,x[1]=-ellipse_a,x[2]=ellipse_a,x[3]=ellipse_a;x[0]<=ellipse_a,x[1]<=ellipse_a,x[2]>=-ellipse_a,x[3]>=-ellipse_a;x[0]+=0.5,x[1]+=0.5,x[2]-=0.5,x[3]-=0.5)
      {
        z[2]=ellipse_b*sqrt(1-(pow(x[2],2)/pow(ellipse_a,2)));
        z[3]=ellipse_b*sqrt(1-(pow(x[3],2)/pow(ellipse_a,2)));
        IK();
        
        
        delay(5);
        
      }
      delay(200);
     
    }
    
   
   

    
    else if(command=='s') //stand at initial angle
    { pwm.setPWM(0, 0, angletopulse(180));
    
    pwm.setPWM(4, 0, angletopulse(180));
    
    pwm.setPWM(8, 0, angletopulse(180));
    
    pwm.setPWM(12, 0, angletopulse(180));
    
    
    pwm.setPWM(1, 0, angletopulse(180));
    
    pwm.setPWM(5, 0, angletopulse(185));
  
    pwm.setPWM(9, 0, angletopulse(90));
   
    pwm.setPWM(13, 0, angletopulse(175));
    
    
    pwm.setPWM(2, 0, angletopulse(180));
    
    pwm.setPWM(6, 0, angletopulse(90));
    
    pwm.setPWM(10, 0, angletopulse(90)); //MG995 0 to 180 equals 70 to 510
    
    pwm.setPWM(14, 0, angletopulse(190));
     delay(5000);
    }


    
    else if(command=='f') //Backward
    {
      
     
      
      for(y[0]=-ellipse_a,y[1]=-ellipse_a,y[2]=ellipse_a,y[3]=ellipse_a;y[0]<=ellipse_a,y[1]<=ellipse_a,y[2]>=-ellipse_a,y[3]>=-ellipse_a;y[0]+=0.5,y[1]+=0.5,y[2]-=0.5,y[3]-=0.5)
      {
        z[0]=ellipse_b*sqrt(1-(pow(y[0],2)/pow(ellipse_a,2)));
        z[1]=ellipse_b*sqrt(1-(pow(y[1],2)/pow(ellipse_a,2)));
        IK();
        delay(5);
        
        
      }
      
      delay(200);
      
      

      for(y[0]=ellipse_a,y[1]=ellipse_a,y[2]=-ellipse_a,y[3]=-ellipse_a;y[0]>=-ellipse_a,y[1]>=-ellipse_a,y[2]<=ellipse_a,y[3]<=ellipse_a;y[0]-=0.5,y[1]-=0.5,y[2]+=0.5,y[3]+=0.5)
      {
        z[2]=ellipse_b*sqrt(1-(pow(y[2],2)/pow(ellipse_a,2)));
        z[3]=ellipse_b*sqrt(1-(pow(y[3],2)/pow(ellipse_a,2)));
        IK();
        
        
        delay(5);
        
      }
      delay(200);
     
    }
  

}
}
