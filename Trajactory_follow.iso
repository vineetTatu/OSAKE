#include <Encoder.h>
#include <Arduino.h>
#include "CytronMotorDriver.h"

const int encoderPin1A = 32;   // Encoder signal A (Link 1)
const int encoderPin1B = 33;   // Encoder signal B (Link 1)
const int encoderPin2A = 25;   // Encoder signal A (Link 2)
const int encoderPin2B = 26;   // Encoder signal B (Link 2)
const int pulsesPerRevolution = 8000;  // PPR for encoder

volatile int encoderPosition1 = 0;
volatile int encoderPosition2 = 0;

CytronMD motor1(PWM_DIR, 13, 12);    // PWM 1 = Pin D13, DIR 1 = Pin D12.
CytronMD motor2(PWM_DIR, 14, 27);    // PWM 2 = Pin D14, DIR 2 = Pin D27.


// Variables
long prevT1, prevT2 = 0;
float prevE1, prevE2 = 0;
float E_integral1, E_integral2 = 0;
float E1, Delta_E1, E2, Delta_E2 = 0;
float u1,u2, degrees1, degrees2 = 0;

// PID constants
float kp1 = 1;
float kd1 = 0.3;
float ki1 = 0.003;
float kp2 = 1;
float kd2 = 0.7;
float ki2 = 0.003;


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

  E1 =  ( target1-degrees1) ;
  E2 =  ( target2-degrees2) ;

  Delta_E1 = ( E1 - prevE1)    ;  //derivative of the error
  prevE1 = E1 ;                   // updating the error
  E_integral1 = E_integral1 + E1 ;

  Delta_E2 = ( E2 - prevE2)    ;  //derivative of the error
  prevE2 = E2 ;                   // updating the error
  E_integral2 = E_integral2 + E2 ;

  u1 = kp1*E1 + kd1* Delta_E1 + ki1 * E_integral1 ;
  u2 = kp2*E2 + kd2* Delta_E2 + ki2 * E_integral2 ;
  Serial.print("u1 -> ");
  Serial.println(u1);
  Serial.print("u2 -> ");
  Serial.println(u2);
  
  // If-Else conditions are used for setting maximum PWM value
  // The motor rotation direction is opposite to error value so used If-Else to adjust values
   
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
