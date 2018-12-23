#include <PID_v1.h>
#include <Stepper.h>

#define encoder2PinA 18
#define encoder2PinB 19
#define encoder3PinA 20
#define encoder3PinB 21
#define encoder5PinA 31
#define encoder5PinB 30

#define STEPS_PER_MOTOR_REVOLUTION 32 
#define STEPS_PER_OUTPUT_REVOLUTION 32 * 64  //2048 

//volatile unsigned 
int encoder2Pos = 0;
int encoder3Pos = 0;
int encoder5Pos = 0;

double Setpoint2, Input2, Output2=0;
double Setpoint3, Input3, Output3=0;
PID pid2(&Input2, &Output2, &Setpoint2, 5, 2, 0, DIRECT);
PID pid3(&Input3, &Output3, &Setpoint3, 5, 0, 0, DIRECT);

#include <Arduino.h>

#include "A4988.h"

#include <LiquidCrystal.h>

// Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/step
#define MOTOR_STEPS 200

// All the wires needed for full functionality
#define DIR1 3
#define STEP1 2
#define DIR2 5
#define STEP2 4
#define DIR3 7
#define STEP3 6
#define DIR4 9
#define STEP4 8
#define DIR5 16
#define STEP5 14
  
int s1=0;
int s4=0;
int Pos1=0;
int Pos4=0;

// 2-wire basic config, microstepping is hardwired on the driver
// BasicStepperDriver stepper(DIR, STEP);

// microstep control for A4988

 A4988 stepper1(MOTOR_STEPS, DIR1, STEP1);
 A4988 stepper2(MOTOR_STEPS, DIR2, STEP2);
 A4988 stepper3(MOTOR_STEPS, DIR3, STEP3);
 A4988 stepper4(MOTOR_STEPS, DIR4, STEP4);
 A4988 stepper5(MOTOR_STEPS, DIR5, STEP5);

 Stepper small_stepper(STEPS_PER_MOTOR_REVOLUTION, 10, 12, 11, 13);

/*-----( Declare Variables )-----*/
int  Steps2Take;

LiquidCrystal lcd(48, 49, 51, 50, 53, 52);

void stepper1Move(int error){
  stepper1.move(-error);
  s1=0;
  Pos1= Pos1+error;
  }

void stepper4Move(int error){
  stepper4.move(error);
  s4=0;
  Pos4= Pos4+error;
  }
  
// Encoder Interrupt Channel A
void doEncoder2A(){

  // look for a low-to-high on channel A
  if (digitalRead(encoder2PinA) == HIGH) { 
    // check channel B to see which way encoder is turning
    if (digitalRead(encoder2PinB) == LOW) {  
      encoder2Pos = encoder2Pos + 1;         // CW
    } 
    else {
      encoder2Pos = encoder2Pos - 1;         // CCW
    }
  }
  else   // must be a high-to-low edge on channel A                                       
  { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(encoder2PinB) == HIGH) {   
      encoder2Pos = encoder2Pos + 1;          // CW
    } 
    else {
      encoder2Pos = encoder2Pos - 1;          // CCW
    }
  }
          
  // use for debugging - remember to comment out
}

void doEncoder2B(){

  // look for a low-to-high on channel B
  if (digitalRead(encoder2PinB) == HIGH) {   
   // check channel A to see which way encoder is turning
    if (digitalRead(encoder2PinA) == HIGH) {  
      encoder2Pos = encoder2Pos + 1;         // CW
    } 
    else {
      encoder2Pos = encoder2Pos - 1;         // CCW
    }
  }
  // Look for a high-to-low on channel B
  else { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(encoder2PinA) == LOW) {   
      encoder2Pos = encoder2Pos + 1;          // CW
    } 
    else {
      encoder2Pos = encoder2Pos - 1;          // CCW
    }
  }
}

void doEncoder3A(){

  // look for a low-to-high on channel A
  if (digitalRead(encoder3PinA) == HIGH) { 
    // check channel B to see which way encoder is turning
    if (digitalRead(encoder3PinB) == LOW) {  
      encoder3Pos = encoder3Pos + 1;         // CW
    } 
    else {
      encoder3Pos = encoder3Pos - 1;         // CCW
    }
  }
  else   // must be a high-to-low edge on channel A                                       
  { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(encoder3PinB) == HIGH) {   
      encoder3Pos = encoder3Pos + 1;          // CW
    } 
    else {
      encoder3Pos = encoder3Pos - 1;          // CCW
    }
  }
          
  // use for debugging - remember to comment out
}

void doEncoder3B(){

  // look for a low-to-high on channel B
  if (digitalRead(encoder3PinB) == HIGH) {   
   // check channel A to see which way encoder is turning
    if (digitalRead(encoder3PinA) == HIGH) {  
      encoder3Pos = encoder3Pos + 1;         // CW
    } 
    else {
      encoder3Pos = encoder3Pos - 1;         // CCW
    }
  }
  // Look for a high-to-low on channel B
  else { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(encoder3PinA) == LOW) {   
      encoder3Pos = encoder3Pos + 1;          // CW
    } 
    else {
      encoder3Pos = encoder3Pos - 1;          // CCW
    }
  }
}

void doEncoder5A(){

  // look for a low-to-high on channel A
  if (digitalRead(encoder5PinA) == HIGH) { 
    // check channel B to see which way encoder is turning
    if (digitalRead(encoder5PinB) == LOW) {  
      encoder5Pos = encoder5Pos + 1;         // CW
    } 
    else {
      encoder5Pos = encoder5Pos - 1;         // CCW
    }
  }
  else   // must be a high-to-low edge on channel A                                       
  { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(encoder5PinB) == HIGH) {   
      encoder5Pos = encoder5Pos + 1;          // CW
    } 
    else {
      encoder5Pos = encoder5Pos - 1;          // CCW
    }
  }
          
  // use for debugging - remember to comment out
}

void doEncoder5B(){

  // look for a low-to-high on channel B
  if (digitalRead(encoder5PinB) == HIGH) {   
   // check channel A to see which way encoder is turning
    if (digitalRead(encoder5PinA) == HIGH) {  
      encoder5Pos = encoder5Pos + 1;         // CW
    } 
    else {
      encoder5Pos = encoder5Pos - 1;         // CCW
    }
  }
  // Look for a high-to-low on channel B
  else { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(encoder5PinA) == LOW) {   
      encoder5Pos = encoder5Pos + 1;          // CW
    } 
    else {
      encoder5Pos = encoder5Pos - 1;          // CCW
    }
  }
}

String matlabData;
 
void setup() {
// analogReadResolution(12);
  lcd.begin(16, 2);

  pinMode(14, OUTPUT);
  pinMode(16, OUTPUT); 
  
  stepper1.setMicrostep(8); // make sure we are in full speed mode   
  stepper1.setRPM(16);
  
  stepper2.setMicrostep(8); // make sure we are in full speed mode   
  stepper2.setRPM(32);

  stepper3.setMicrostep(8); // make sure we are in full speed mode   
  stepper3.setRPM(32);

  stepper4.setMicrostep(8); // make sure we are in full speed mode   
  stepper4.setRPM(16);

  stepper5.setMicrostep(8); // make sure we are in full speed mode   
  stepper5.setRPM(16);

  Setpoint2=0; 
  Setpoint3=0; 
  pid2.SetOutputLimits(-800, 800);
  pid3.SetOutputLimits(-8800, 8800);
  pinMode(encoder2PinA, INPUT); 
  digitalWrite(encoder2PinA, HIGH);       // turn on pull-up resistor
  pinMode(encoder2PinB, INPUT); 
  digitalWrite(encoder2PinB, HIGH);       // turn on pull-up resistor

// encoder pin on interrupt 6

  attachInterrupt(encoder2PinA, doEncoder2A, CHANGE);
// encoder pin on interrupt 7

  attachInterrupt(encoder2PinB, doEncoder2B, CHANGE);  

  pinMode(encoder3PinA, INPUT); 
  digitalWrite(encoder3PinA, HIGH);       // turn on pull-up resistor
  pinMode(encoder3PinB, INPUT); 
  digitalWrite(encoder3PinB, HIGH);       // turn on pull-up resistor

// encoder pin on interrupt 8

  attachInterrupt(encoder3PinA, doEncoder3A, CHANGE);
// encoder pin on interrupt 9

  attachInterrupt(encoder3PinB, doEncoder3B, CHANGE);  
  Serial.begin (9600);

    pinMode(encoder5PinA, INPUT); 
  digitalWrite(encoder5PinA, HIGH);       // turn on pull-up resistor
  pinMode(encoder5PinB, INPUT); 
  digitalWrite(encoder5PinB, HIGH);       // turn on pull-up resistor

// encoder pin on interrupt 3

  attachInterrupt(encoder5PinA, doEncoder5A, CHANGE);
// encoder pin on interrupt 2

  attachInterrupt(encoder5PinB, doEncoder5B, CHANGE); 

}
 
void loop() {
  if(Serial.available() > 0) {
    matlabData = Serial.readString(); // read data
    s1=1;
    s4=1;

  }

if (matlabData=="GC"){
  Steps2Take  =  -STEPS_PER_OUTPUT_REVOLUTION/4 ;  // Rotate CW 1 turn
  small_stepper.setSpeed(500);   
  small_stepper.step(Steps2Take);
  }

else if(matlabData=="GO"){
  Steps2Take  = STEPS_PER_OUTPUT_REVOLUTION/4 ;  // Rotate CW 1 turn
  small_stepper.setSpeed(500);   
  small_stepper.step(Steps2Take);  
  }
// Split string from python code to Steps to give to motors
else{
   int a= matlabData.indexOf('@');
   int b= matlabData.indexOf('#');
   int c= matlabData.indexOf('$');
   int d= matlabData.indexOf('%');
   int e= matlabData.length();

   String ang1= matlabData.substring(0, a);
   String ang2= matlabData.substring(a+1, b);
   String ang3= matlabData.substring(b+1, c);
   String ang4= matlabData.substring(c+1, d);
   String ang5= matlabData.substring(d+1, e);

   float angle1= ang1.toFloat();
   float angle2= ang2.toFloat();
   float angle3= ang3.toFloat();
   float angle4= ang4.toFloat();
   float angle5= ang5.toFloat();

   float steps1= (angle1/(2*3.142))*1600;
  Setpoint2= (angle2/(2*3.142))*1600;
  Setpoint3= (angle3/(2*3.142))*1600;
   float steps4= (angle4/(2*3.142))*1600;
   float steps5= (angle5/(2*3.142))*1600;

   int error1= round(8*steps1)-Pos1;
   int error2= round(Setpoint2-encoder2Pos);
   int error3= round(11.5385*Setpoint3-11.5385*encoder3Pos);
   int error4= round(steps4-Pos4);
   int error5= round(4.4*steps5-4.4*encoder5Pos);
//

   if (error2<5 & error2>-5) stepper2.move(0);
//   else stepper2.move(error2);
else{     Input2= encoder2Pos;
   pid2.SetMode(AUTOMATIC);
   pid2.Compute();
   stepper2.move(Output2);
}
      
   if (error3<13 & error3>-13) stepper3.move(0);
//   else stepper2.move(error2);
else{     Input3= encoder3Pos;
   pid3.SetMode(AUTOMATIC);
   pid3.Compute();
   stepper3.move(Output3);
}

    lcd.setCursor(0, 1);
    //  lcd.print(error5);
    //  lcd.print(" ");
      lcd.print(matlabData);

   if (s4==1)stepper4Move(error4);
   else stepper4.move(0);
   
   if (error5<3 & error5>-2) stepper5.move(0);
   else stepper5.move(error5);

   if (s1==1)stepper1Move(error1);
   else stepper1.move(0);
}
}
