#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#define NUM_SERVOS 2
#define SERVO_MIN 150
#define SERVO_MAX 600
#define NUM_ANGLES 12
int angles[12];
int servoPins[NUM_SERVOS] = {0,1};
int angleValue;
Adafruit_PWMServoDriver pwm;
int led = 5;

void setup() {
  pinMode(led, OUTPUT);
  Serial.begin(115200);
  pwm.begin();
  pwm.setPWMFreq(50);
  for(int i = 0; i < NUM_SERVOS; i++){
    pwm.setPWM(servoPins[i],0,350);
  }
  delay(10);
}

uint16_t degreeToPwm(int degree){
  degree = constrain(degree,0,180);
  return map(degree,0,180,SERVO_MIN,SERVO_MAX);
}

void loop() {
  if(Serial.available()>0){
    String data = Serial.readStringUntil('\n');
    int startIndex = 0;
    int commandIndex;
    for (int i = 0; i < NUM_ANGLES; i++){
      commandIndex = data.indexOf(',',startIndex);
      if(commandIndex!=-1){
        angles[i] = data.substring(startIndex,commandIndex).toInt();
        startIndex = commandIndex + 1;
      }else{
        angles[i] = data.substring(startIndex).toInt();
      }
    }
    if(angles[0]==0 && angles[1]==54 && angles[2]==-83 &&
    angles[3]==0 && angles[4]==44 && angles[5]==-90 && 
    angles[6]==0 && angles[7]==30 && angles[8]==-89 &&
    angles[9]==0 && angles[10]==16 && angles[11]==-84){
      digitalWrite(led,HIGH);
      delay(20);
    }
    if(angles[0]==0 && angles[1]==55 && angles[2]==-83 &&
    angles[3]==0 && angles[4]==44 && angles[5]==-90 && 
    angles[6]==0 && angles[7]==30 && angles[8]==-89 &&
    angles[9]==0 && angles[10]==15 && angles[11]==-84){
      digitalWrite(led,LOW);
      delay(1000);
      digitalWrite(led,HIGH);
    }

    //int angles[2] = {90 + data.substring(0,commandIndex).toInt(),90 + data.substring(commandIndex+1).toInt()};
    //int theta1 = data.substring(0,commandIndex).toInt();
    //int theta2 = data.substring(commandIndex+1).toInt();
    
    //for(int i = 0; i < NUM_SERVOS ; i++){
    //  pwm.setPWM(servoPins[i],0,degreeToPwm(angles[i]));
    //}

    //delay(5000);
    //for(int i = 0; i < NUM_SERVOS; i++){
    //pwm.setPWM(servoPins[i],0,350);
    //delay(500);
  }
  /*for (int pos = SERVO_MIN;pos<=SERVO_MAX;pos+=1){
    for(int i = 0; i < NUM_SERVOS; i++){
      pwm.setPWM(servoPins[i],0,pos);
    }
    delay(15);
  }
  delay(1000);
  for (int pos = SERVO_MAX;pos>=SERVO_MAX;pos-=1){
    for(int i = 0; i < NUM_SERVOS; i++){
      pwm.setPWM(servoPins[i],0,pos);
    }
    delay(15);
  }*/
}
