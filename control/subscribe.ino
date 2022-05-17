#include <ros.h>
#include <std_msgs/Int32.h>
ros::NodeHandle nh;
                                                                              
                
int angle;
int percent;

int Dir1A  = 4;
int Dir1B  = 5;

int Dir2A  = 6;
int Dir2B  = 7;

int Dir3A  = 8;
int Dir3B  = 9;

int Dir4A  = 10;
int Dir4B  = 11;

int PWM1     =  12;
int PWM2     =  13; 
int PWM3     =  14;
int PWM4     =  15; 

  
void messageCb(const std_msgs::Int32& msg)
{
//  Serial.print(msg.data);
    angle = msg.data;
    percent = msg.data;
    //Serial.println(msg.data);
}       

int duty=percent*8.4;

ros::Subscriber<std_msgs::Int32> sub("chatter", &messageCb);
void motor();
void go_forward()       
{
  digitalWrite( Dir1A, HIGH );
  digitalWrite( PWM1,  LOW );
  analogWrite( Dir1A, duty );
  digitalWrite( Dir2A,  LOW );
  digitalWrite( Dir2B, HIGH );
  analogWrite( PWM2, duty );
  digitalWrite( Dir3A, HIGH );
  digitalWrite( Dir3B,  LOW );
  analogWrite( PWM3, duty );
  digitalWrite( Dir4A,  LOW );
  digitalWrite( Dir4B, HIGH );
  analogWrite( PWM4, duty );
}




void turn_left()       
{
  digitalWrite( Dir1A,  LOW );
  digitalWrite( Dir1B, HIGH );
  analogWrite( PWM1, duty );
  digitalWrite( Dir2A,  LOW );
  digitalWrite( Dir2B, HIGH );
  analogWrite( PWM2, duty );
  digitalWrite( Dir3A,  LOW );
  digitalWrite( Dir3B, HIGH );
  analogWrite( PWM3, duty );
  digitalWrite( Dir4A,  LOW );
  digitalWrite( Dir4B, HIGH );
  analogWrite( PWM4, duty );
  
  

}


void go_back()
{
  digitalWrite( Dir1A,  LOW );
  digitalWrite( Dir1B, HIGH );
  analogWrite( PWM1, duty );
  digitalWrite( Dir2A, HIGH );
  digitalWrite( Dir2B,  LOW );
  analogWrite( PWM2, duty );
  digitalWrite( Dir3A,  LOW );
  digitalWrite( Dir3B, HIGH );
  analogWrite( PWM3, duty );
  digitalWrite( Dir4A, HIGH );
  digitalWrite( Dir4B,  LOW );
  analogWrite( PWM4, duty );
}










void turn_right()     
{
  digitalWrite( Dir1A, HIGH );
  digitalWrite( Dir1B,  LOW );
  analogWrite( PWM1, duty );
  digitalWrite( Dir2A, HIGH );
  digitalWrite( Dir2B,  LOW );
  analogWrite( PWM2, duty );
  digitalWrite( Dir3A, HIGH );
  digitalWrite( Dir3B,  LOW );
  analogWrite( PWM3, duty );
  digitalWrite( Dir4A, HIGH );
  digitalWrite( Dir4B,  LOW );
  analogWrite( PWM4, duty );
}

void left()       
{
  digitalWrite( Dir1A,  LOW );
  digitalWrite( Dir1B, HIGH );
  analogWrite( PWM1, duty );
  digitalWrite( Dir2A,  LOW );
  digitalWrite( Dir2B, HIGH );
  analogWrite( PWM2, duty );
  digitalWrite( Dir3A, HIGH );
  digitalWrite( Dir3B,  LOW );
  analogWrite( PWM3, duty );
  digitalWrite( Dir4A,  LOW );
  digitalWrite( Dir4B, HIGH );
  analogWrite( PWM4, duty );
}



void right()     
{
  digitalWrite( Dir1A, HIGH );
  digitalWrite( Dir1B,  LOW );
  analogWrite( PWM1, duty );
  digitalWrite( Dir2A, HIGH );
  digitalWrite( Dir2B,  LOW );
  analogWrite( PWM2, duty );
  digitalWrite( Dir3A,  LOW );
  digitalWrite( Dir3B, HIGH );
  analogWrite( PWM3, duty );
  digitalWrite( Dir4A,  LOW );
  digitalWrite( Dir4B, HIGH );
  analogWrite( PWM4, duty );
}

void go_left()      
{
  digitalWrite( Dir1A, HIGH );
  digitalWrite( Dir1B, HIGH );
  analogWrite( PWM1, duty );
  digitalWrite( Dir2A,  LOW );
  digitalWrite( Dir2B, HIGH );
  analogWrite( PWM2, duty );
  digitalWrite( Dir3A, HIGH );
  digitalWrite( Dir3B,  LOW );
  analogWrite( PWM3, duty );
  digitalWrite( Dir4A, HIGH );
  digitalWrite( Dir4B, HIGH );
  analogWrite( PWM4, duty );
}



void go_right()     
{
  digitalWrite( Dir1A, HIGH );
  digitalWrite( Dir1B,  LOW );
  analogWrite( PWM1, duty );
  digitalWrite( Dir2A, HIGH );
  digitalWrite( Dir2B, HIGH );
  analogWrite( PWM2, duty );
  digitalWrite( Dir3A, HIGH ); 
  digitalWrite( Dir3B, HIGH );
  analogWrite( PWM3, duty );
  digitalWrite( Dir4A,  LOW );
  digitalWrite( Dir4B, HIGH );
  analogWrite( PWM4, duty );
}

void back_left()       
{
  digitalWrite( Dir1A,  LOW );
  digitalWrite( Dir1B, HIGH );
  analogWrite( PWM1, duty );
  digitalWrite( Dir2A, HIGH );
  digitalWrite( Dir2B, HIGH );
  analogWrite( PWM2, duty );
  digitalWrite( Dir3A, HIGH );
  digitalWrite( Dir3B, HIGH );
  analogWrite( PWM3, duty );
  digitalWrite( Dir4A, HIGH );
  digitalWrite( Dir4B,  LOW );
  analogWrite( PWM4, duty );
}



void back_right()     
{
  digitalWrite( Dir1A, HIGH );
  digitalWrite( Dir1B, HIGH );
  analogWrite( PWM1, duty );
  digitalWrite( Dir2A, HIGH );
  digitalWrite( Dir2B,  LOW );
  analogWrite( PWM2, duty );
  digitalWrite( Dir3A,  LOW );
  digitalWrite( Dir3B, HIGH );
  analogWrite( PWM3, duty );
  digitalWrite( Dir4A, HIGH );
  digitalWrite( Dir4B, HIGH );
  analogWrite( PWM4, duty );
}



void break_on()      
{
  digitalWrite( Dir1A, HIGH );
  digitalWrite( Dir1B, HIGH );
  digitalWrite( Dir2A, HIGH );
  digitalWrite( Dir2B, HIGH );
  digitalWrite( Dir3A, HIGH );
  digitalWrite( Dir3B, HIGH );
  digitalWrite( Dir4A, HIGH );
  digitalWrite( Dir4B, HIGH );
}

void motor(){
  if ( angle == 0){
      go_forward();
      Serial.println("go");
    }
    else if ( angle == 180){
      go_back();
      Serial.println("back");
    }
    else if ( angle<0 && angle >-90 ){
      go_left();
      Serial.println("go left");
    }
    else if ( angle>0 && angle<90 ){
      go_right();
      Serial.println("go right");
    }
    else if ( angle<-90 && angle>-180 ){
      back_left();
      Serial.println("back left");
    }
    else if ( angle>90 && angle<180 ){
      back_right();
      Serial.println("back right");
    }
    else if ( angle == -90 ){
      left();
      Serial.println("left");
    }
    else if ( angle == 90 ){
      right();
      Serial.println("right");
    }
    
      
     
}
void setup()
{
  pinMode( Dir1A, OUTPUT );
  pinMode( Dir1B, OUTPUT );
  pinMode( Dir2A, OUTPUT );
  pinMode( Dir2B, OUTPUT );
  pinMode( Dir3A, OUTPUT );
  pinMode( Dir3B, OUTPUT );
  pinMode( Dir4A, OUTPUT );
  pinMode( Dir4B, OUTPUT );
  pinMode(  PWM1, OUTPUT );
  pinMode(  PWM2, OUTPUT );
  pinMode(  PWM3, OUTPUT );
  pinMode(  PWM4, OUTPUT );

 
  break_on();





  Serial.begin( 38400 );

  nh.initNode();
  nh.subscribe(sub);
}
void loop()
{

  nh.spinOnce();
  delay(10);
} 
