#include <Servo.h>

//PITCH
Servo pitchServo;
const int pitchPin = 3;
const int pitchPot = 0;
int pitchVal;


//YAW
Servo yawServo;
const int yawPin = 5;
const int yawPot = 1;
int yawVal;

//INTAKE / INDEXER
Servo intakeServo;
const int intakePin = 6;
const int intakeButton = 12;

//Shooter
Servo leftShooterMotor;
Servo rightShooterMotor;
const int leftShooterPin = 10;
const int rightShooterPin = 9;
const int shooterButton = 13;

void setup()
{
 
 pitchServo.attach(pitchPin);
 yawServo.attach(yawPin);
 intakeServo.attach(intakePin);
 
 leftShooterMotor.attach(leftShooterPin, 1000, 2000);
 rightShooterMotor.attach(rightShooterPin, 1000, 2000); 
  
 pinMode(intakeButton, INPUT);
 
 Serial.begin(9600);
 Serial.println("Running Setup...");
 
 
}

void loop(){

  yawVal = analogRead(yawPot);
  yawVal = map(yawVal, 0, 1023, 0, 180); //TODO change to max angle of servo
  yawServo.write(yawVal);
  
  pitchVal = analogRead(pitchPot);
  pitchVal = map(pitchVal, 0, 1023, 0, 180); //TODO change to max angle of servo
  pitchServo.write(pitchVal);
  
  if(digitalRead(intakeButton) == LOW){
    intakeServo.write(180); //TODO find real val
    Serial.println("Intake button hit...");
  }else{
    intakeServo.write(0); //TODO find real val
  }
  
  if(digitalRead(shooterButton) == LOW){
    setMotorSpeed(100); //TODO: Change to real val;
    Serial.println("Shooter button hit...");
  }else{
    setMotorSpeed(0);
  }
}
  
void setMotorSpeed(int val){
    leftShooterMotor.write(val); 
    rightShooterMotor.write(val); 
}
