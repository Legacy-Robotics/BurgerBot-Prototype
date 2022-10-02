// C++ code
//
#include <stdio.h>
#include <Servo.h>
#include <math.h>

Servo servoYaw; // the third one
Servo servoPitch; // the second one
Servo servoIndexer; // the first one

float pitchAngle;
float yawAngle;
float indexAngle = 0;
float timer = 300;
float step = 54;
float temp;

bool running = false;
bool indexerRunning = true;

// Receive a command from ROS 2
float ROSCMD[4] = {45, 15, 0, 1}; // [yaw, pitch, fire, indexer], hardcode first


void setup()
{
  pinMode(13, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  servoYaw.attach(9);
  servoPitch.attach(10);
  servoIndexer.attach(11);
  
}

// define a function for rotating indexer
void rotateIndexer(){
  if (indexerRunning == true){
  	for (int i = 0; i< 3; i++){
    temp = timer/3;
    indexAngle = indexAngle + (step/timer * temp);
    if (indexAngle > 180){
    	indexAngle = 0;
    	}
  	servoIndexer.write(indexAngle);
    delay(temp);
  	}
    servoIndexer.write(0);
    indexerRunning = false;
    
  }
}

// define a function for setting motor speed.
/*void setMotorSpeed(int val){
  	analogWrite(13,val);
  	analogWrite(12,val);
    analogWrite(8,val);
  	analogWrite(7,val);
}*/

void loop()
{
  if (running == false){
  	running = true;
    servoYaw.write(ROSCMD[0]);
  	servoPitch.write(ROSCMD[1]);
    //servoIndexer.write(0);
  
  	if ((ROSCMD[2] == 1) && (ROSCMD[3] == 0)){
    	digitalWrite(13, LOW);
    	digitalWrite(12, HIGH);
    	digitalWrite(8, LOW);
    	digitalWrite(7, HIGH);
    	//setMotorSpeed(100);
    	rotateIndexer();
      	
    	ROSCMD[2] = 0;
    	//setMotorSpeed(0);
  	}
  	else if ((ROSCMD[2] == 0) && (ROSCMD[3] == 1)){
  		rotateIndexer();
      	
  	}
    
    running = false;
  
  
  
  }
  /*else {
  
  }*/
  
}