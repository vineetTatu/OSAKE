#include <Encoder.h>
#include <Arduino.h>
#include "CytronMotorDriver.h"
#include "ACS712.h"

const int encoderPin1A = 32;   // Encoder signal A (Link 1)
const int encoderPin1B = 33;   // Encoder signal B (Link 1)
const int encoderPin2A = 25;   // Encoder signal A (Link 2)
const int encoderPin2B = 26;   // Encoder signal B (Link 2)
const int pulsesPerRevolution = 8000;  // PPR for encoder

volatile int encoderPosition1 = 0;
volatile int encoderPosition2 = 0;

CytronMD motor1(PWM_DIR, 13, 12);    // PWM 1 = Pin D13, DIR 1 = Pin D12.
CytronMD motor2(PWM_DIR, 14, 27);    // PWM 2 = Pin D14, DIR 2 = Pin D27.
ACS712  ACS(34, 5, 4095, 100);   // ---> ESP
ACS712  ACS(35, 5, 4095, 100);   // ---> ESP

// Variables
long prevT1, prevT2 = 0;
float prevE1, prevE2, prevE3 = 0;
float E_integral1, E_integral2, E_integral3 = 0;
float E1, Delta_E1, E2, Delta_E2, E3, Delta_E3 = 0;
float u1,u2, degrees1, degrees2 = 0;

// PID constants
// motor 1
float kp1 = 1;
float kd1 = 0.3;
float ki1 = 0.003;
//motor 2
float kp2 = 1;
float kd2 = 0.7;
float ki2 = 0.003;
// for current 
float kp3 = 1;
float kd3 = 0.4;
float ki3 = 0.002;

float kt = 20;      // Torque constant
float T_des = 1 ;  // desired torque


void setup() {
  // put your setup code here, to run once:
  
  Serial.begin (9600);

  pinMode(encoderPin1A, INPUT_PULLUP);
  pinMode(encoderPin1B, INPUT_PULLUP);

  pinMode(encoderPin2A, INPUT_PULLUP);
  pinMode(encoderPin2B, INPUT_PULLUP);

  // Attach interrupt handlers for encoder pulses
  attachInterrupt(digitalPinToInterrupt(encoderPin1A), handleEncoder1Interrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPin2A), handleEncoder2Interrupt, CHANGE);

  Serial.begin(115200);
  ACS.autoMidPoint();
  
}

void loop() {
  // Convert encoder position to degrees
  float degrees1 = (static_cast<float>(encoderPosition1) / pulsesPerRevolution) * 360.0;
  float degrees2 = (static_cast<float>(encoderPosition2) / pulsesPerRevolution) * 360.0;
  // Print the current encoder position in degrees
  Serial.print("1 Encoder Position (Degrees): ");
  Serial.println(degrees1);
  Serial.print("2 Encoder Position (Degrees): ");
  Serial.println(degrees2);
  delay(100); // Delay for readability (adjust as needed)

     
  // put your main code here, to run repeatedly:
  int  target1 = 90 ;   // set target angle
  // pid constants
  int  target2 = 180;
  int  I_des = (T_des/ kt)  ;   // current desired for the desired torque
  E1 =  ( target1-degrees1) ;
  E2 =  ( target2-degrees2) ;
  E3 =  (I_des - I_cur) ;

  Delta_E1 = ( E1 - prevE1)    ;  //derivative of the error
  prevE1 = E1 ;                   // updating the error
  E_integral1 = E_integral1 + E1 ;

  Delta_E2 = ( E2 - prevE2)    ;  //derivative of the error
  prevE2 = E2 ;                   // updating the error
  E_integral2 = E_integral2 + E2 ;

  Delta_E3 = ( E3 - prevE3)    ;  //derivative of the error
  prevE3 = E3 ;                   // updating the error
  E_integral3 = E_integral3 + E3 ;

  u1 = kp1*E1 + kd1* Delta_E1 + ki1 * E_integral1 ;
  u2 = kp2*E2 + kd2* Delta_E2 + ki2 * E_integral2 ;
  u3 = kp3*E3 + kd3* Delta_E3 + ki3 * E_integral3 :
  Serial.print("u1 -> ");
  Serial.println(u1);
  Serial.print("u2 -> ");
  Serial.println(u2);
  Serial.print("u3 -> ");
  Serial.println(u3);
  // If-Else conditions are used for setting maximum PWM value
  // The motor rotation direction is opposite to error value so used If-Else to adjust values
   //for the motor 1
  if(E1<0){
    if(u1<-70){
      motor1.setSpeed(70);
    }
    else{
      motor1.setSpeed((-1)*u1);
    }
  }
  else{
    if(u1>70){
      motor1.setSpeed(-70);
    }
    else{
      motor1.setSpeed((-1)*u1);
    }
  }


// for the motor 2
  if(E2<0){
    if(u2<-80){
      motor2.setSpeed(90);
    }
    else{
      motor2.setSpeed((-1)*u2);
    }
  }
  else{
    if(u2>80){
      motor2.setSpeed(-90);
    }
    else{
      motor2.setSpeed((-1)*u2);      
    }
  }

  
  int I_cur = ACS.mA_DC();
  Serial.println(I_cur);
  delay(1000);


  // for torque control
  if (u3 > 0) {
    motor1.setSpeed(-u3);
    
  } else {
    motor1.setSpeed(u3);
    
}
 
   

void handleEncoder1Interrupt() {
  // Read the current state of encoderPin1A
  int state1A = digitalRead(encoderPin1A);
  // Read the current state of encoderPin1B
  int state1B = digitalRead(encoderPin1B);
  // Determine the direction of rotation based on the states of A and B
  int direction1 = (state1A == state1B) ? 1 : -1;
  // Update the encoder position
  encoderPosition1 += direction1;

}


void handleEncoder2Interrupt() {
  // Read the current state of encoderPin1A
  int state2A = digitalRead(encoderPin2A);
  // Read the current state of encoderPin1B
  int state2B = digitalRead(encoderPin2B);
  // Determine the direction of rotation based on the states of A and B
  int direction2 = (state2A == state2B) ? 1 : -1;
  // Update the encoder position
  encoderPosition2 += direction2;
}
