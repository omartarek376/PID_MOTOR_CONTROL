#include <PID_v1.h>
#include <TimerOne.h>
double desired_spd = 100;
double Input, Output,Input2,Output2;
double Kp=0.2, Ki=1.04, Kd=0;
volatile float rotation1 =0 , rotation2=0;
PID myPID(&Input, &Output, &desired_spd, Kp, Ki, Kd, DIRECT);
PID myPID2(&Input2, &Output2, &desired_spd, Kp, Ki, Kd, DIRECT);
const byte MOTOR1 = 2;                    //interrupt pin for encoder
const byte MOTOR2 = 3;                    //interrupt pin for encoder
unsigned int counter1 = 0;
unsigned int counter2 = 0;
float diskslots = 10; 
void ISR_count1()  
{
  counter1++;  
} 

void ISR_count2()  
{
  counter2++;  
} 
void ISR_timerone()
{
  Timer1.detachInterrupt();  
  //Serial.print("Motor Speed 1: "); 
  rotation1 = (counter1 / diskslots) * 60.00;  
  if (rotation1 >= 200)
  {
  rotation1 = 200;  
  }
  Input = rotation1;
  counter1 = 0;  
  rotation2 = (counter2 / diskslots) * 60.00;  
  if (rotation2 >= 200)
  {
  rotation2 = 200;  
  }
  Input2 = rotation2;  
  counter2 = 0;  
  Timer1.attachInterrupt( ISR_timerone );  
}
char t;
const int M1_A=7,M1_B=8,M2_A=12,M2_B=4;
const int en2 = 11;
const int en1 = 9;
void setup()
{
Serial.begin(9600);
Timer1.initialize(100000); // set timer for 1sec
attachInterrupt(digitalPinToInterrupt (MOTOR1), ISR_count1, RISING);  
attachInterrupt(digitalPinToInterrupt (MOTOR2), ISR_count2, RISING);  
Timer1.attachInterrupt( ISR_timerone ); 
pinMode(M1_A,OUTPUT);   
pinMode(M1_B,OUTPUT);   
pinMode(M2_A,OUTPUT);   
pinMode(M2_B,OUTPUT);   
pinMode(en1,OUTPUT);
pinMode(en2,OUTPUT);
myPID.SetMode(AUTOMATIC);
myPID2.SetMode(AUTOMATIC);
analogWrite(en2, 150);                        //PWM value to start the motors (will be changed when PID runs)
analogWrite(en1, 150);
}
 
void loop() {
if(Serial.available())
{
 t = Serial.read();                           //to take commands from mobile app
}
Serial.print("Motor Speed 1: ");              //shows speed of motor (comment it if u want to control car using bluetooth)
Serial.print(rotation1);  
Serial.println(" RPM - ");
//Serial.print("Motor Speed 2: ");
//Serial.print(rotation2);  
//Serial.println(" RPM - ");    
myPID.Compute();
myPID2.Compute();
analogWrite(en2, Output);
analogWrite(en1, Output2);
if(t =='F'){            //move forward(all motors rotate in forward direction)
  digitalWrite(M1_A,HIGH);
  digitalWrite(M1_B,LOW);
  digitalWrite(M2_A,HIGH);
  digitalWrite(M2_B,LOW);
}
 
else if(t == 'B'){      //move reverse (all motors rotate in reverse direction)
  digitalWrite(M1_A,LOW);
  digitalWrite(M1_B,HIGH);
  digitalWrite(M2_A,LOW);
  digitalWrite(M2_B,HIGH);
}
 
else if(t == 'L'){      //turn right (left side motors rotate in forward direction,)
  digitalWrite(M1_A,HIGH);
  digitalWrite(M1_B,LOW);
  digitalWrite(M2_A,LOW);
  digitalWrite(M2_B,HIGH);
}
 
else if(t == 'R'){      //turn left (right side motors rotate in forward direction,)
  digitalWrite(M1_A,LOW);
  digitalWrite(M1_B,HIGH);
  digitalWrite(M2_A,HIGH);
  digitalWrite(M2_B,LOW);
}
 
else if(t == 'S'){      //STOP (all motors stop)
  digitalWrite(M1_A,LOW);
  digitalWrite(M1_B,LOW);
  digitalWrite(M2_A,LOW);
  digitalWrite(M2_B,LOW);
}
}
