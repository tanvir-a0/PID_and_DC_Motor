#include <Arduino.h>
void run_motor(int speed); 
void calculate_counter(); 
int calculate_pid(int setpoint, int position);

#define MAX_PWM 150
#define integral_error_count 10

int lpwm = 10;
int rpwm = 9;


 #define outputA 7
 #define outputB 6
 int counter = 0; 
 int aState;
 int aLastState;

void setup() {
  Serial.begin(9600);
  pinMode(lpwm, OUTPUT);
  pinMode(rpwm, OUTPUT);
  pinMode(outputA, INPUT);
  pinMode(outputB, INPUT);

}


float kp = 1;
float ki = 0;
float kd = 0.1;
float error = 0;
float last_error = 0;

float integral_error_value = 0;
int integral_error_index = 0;


void loop() {
  calculate_counter();
  int motr_speed_val = calculate_pid(200, counter);
  run_motor(motr_speed_val);
  //run_motor(100);
  //Serial.println(counter);
  Serial.println(motr_speed_val);
  
  
}

int calculate_pid(int setpoint, int position){
  error = setpoint - position;
  float P = kp * error;

  integral_error_value += error;
  if(integral_error_index < integral_error_count){
    integral_error_index++;
  }else{
    integral_error_index = 0;
    integral_error_value = 0;
  }
  float I = ki * integral_error_value;


  float D = kd * (error - last_error);
  last_error = error;
  float pid = P + I + D;
  // Serial.print("PID: ");
  //Serial.println((int)pid);
  // Serial.print("P: ");
  // Serial.print(P);
  // Serial.print("  I: ");
  // Serial.print(I);
  // Serial.print("  D: ");
  // Serial.println(D);
  if(pid > 0){
    pid = min(pid, MAX_PWM);
  }else{
    pid = max(pid, -MAX_PWM);
  }

  return (int)pid;
}

void calculate_counter()
{
  aState = digitalRead(outputA); // Reads the "current" state of the outputA
   // If the previous and the current state of the outputA are different, that means a Pulse has occured
   if (aState != aLastState){     
     // If the outputB state is different to the outputA state, that means the encoder is rotating clockwise
     if (digitalRead(outputB) != aState) { 
       counter ++;
     } else {
       counter --;
     }
     //Serial.print("Position: ");
     //Serial.println(counter);
   } 
   aLastState = aState; // Updates the previous state of the outputA with the current state
}


void run_motor(int speed){
  if(speed == 0){
    analogWrite(lpwm, 0);
    analogWrite(rpwm, 0);
  }
  if(speed > 0){
    speed = min(speed, MAX_PWM);
    analogWrite(lpwm, speed);
    analogWrite(rpwm, 0);
    //Serial.print("Forward  ");
    //Serial.println(speed);
  }else if(speed < 0){
    speed = -speed;
    speed = min(speed, MAX_PWM);
    analogWrite(lpwm, 0);
    analogWrite(rpwm, speed);
    //Serial.print("Backward  ");
    //Serial.println(speed);
  }
}