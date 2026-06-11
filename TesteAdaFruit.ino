#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <string.h>
#include <stdlib.h>
#include <LiquidCrystal.h>
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);
uint16_t limTicks(float us);
uint16_t degreeToPwm(int degree);
void anglesRecived(const char *data, int *angles, uint8_t size);
void sendingAnglesToServos(int *angles);
#define NUM_SERVOS 12
#define SERVOMIN 172
#define SERVOMAX 530
#define ZERO_POSITION_TICK static_cast<int>((SERVOMIN+SERVOMAX)/2)
#define NUM_ANGLES 12
uint16_t angles[NUM_ANGLES];
uint8_t servoPins[NUM_SERVOS] = {0,1,2,3,4,5,6,7,8,9,10,11};
Adafruit_PWMServoDriver pwm;

void setup() {
  Serial.begin(115200);
  pwm.begin();
  pwm.setPWMFreq(50);
  //Calibra os servos
  for(int j = 0; j < NUM_SERVOS; j++){
      pwm.setPWM(servoPins[j],0,ZERO_POSITION_TICK);
  }
}
void loop() {
  if(Serial.available()>0){
    String data = Serial.readStringUntil('\n');
    char buffer[100];  // precisa caber toda a string
    data.toCharArray(buffer, sizeof(buffer));
    anglesRecived(buffer,angles,12);
    sendingAnglesToServos(angles);
  }
}
uint16_t degreeToPwm(int degree){
  degree = constrain(degree,0,180);
  return map(degree,0,180,SERVOMIN,SERVOMAX);
}
uint16_t degreeToPwmInverse(int degree){
  degree = constrain(degree,0,180);
  return map(degree,180,0,SERVOMIN,SERVOMAX);
}
uint16_t limTicks(float us){
  return (uint16_t)((us*4096)/20*pow(10,3));
}
void anglesRecived( char *data,uint16_t* angles,uint8_t size){
  char *pt = strtok(data,",");
  uint8_t i=0;
  while(pt!=NULL && i<size){
    angles[i] = atoi(pt);
    pt = strtok(NULL,",");
    i++;
  }
}
void sendingAnglesToServos(int *angles){
  //Atribuindo os valores dos ângulos recebidos aos servo motores
  for(int j = 0; j < NUM_ANGLES; j++){
    if(j>=0 && j<3 || j>5 && j<9){
        if(j==0){
          pwm.setPWM(servoPins[0],0,degreeToPwm(angles[0]));
        }else{
          pwm.setPWM(servoPins[j],0,degreeToPwmInverse(angles[j]));
        }
      }
    
    if(j>2 && j<6 || j>=9 && j<=11){
      if(j==9){
        pwm.setPWM(servoPins[j],0,degreeToPwmInverse(angles[j]));
      }else{
        pwm.setPWM(servoPins[j],0,degreeToPwm(angles[j]));
      }
    }
  } 
}



