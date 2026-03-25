#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <string.h>
#include <stdlib.h>
#include <LiquidCrystal.h>
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);
uint16_t limTicks(float us);
uint16_t degreeToPwm(int degree);
void anglesRecived(const char *data, int *angles);
void sendingAnglesToServos(int *angles);
//uint16_t ticks[2] = {limTicks(1000),limTicks(2000)};
#define NUM_SERVOS 12
//#define SERVO_MIN ticks[0]
//#define SERVO_MAX ticks[1]
#define SERVOMIN 172
#define SERVOMAX 530
#define ZERO_POSITION_TICK static_cast<int>((SERVOMIN+SERVOMAX)/2)
#define NUM_ANGLES 12
uint16_t angles[NUM_ANGLES];
uint8_t servoPins[NUM_SERVOS] = {0,1,2,3,4,5,6,7,8,9,10,11};
Adafruit_PWMServoDriver pwm;
//uint8_t led = 5;

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
    //lcd.clear();
    String data = Serial.readStringUntil('\n');
    char buffer[100];  // precisa caber toda a string
    data.toCharArray(buffer, sizeof(buffer));
    //lcd.print(data);
    //delay(500);
    //for (int posicao = 0; posicao < 21; posicao++)
    //{
    //lcd.scrollDisplayLeft();
    //delay(500);
    //}
    anglesRecived(buffer,angles,12);
    sendingAnglesToServos(angles);
    //Salvando os valores dos angulos no vetor do mesmo.
    /*for (int i = 0; i < NUM_ANGLES; i++){
      commandIndex = data.indexOf(',',startIndex);
      if(commandIndex!=-1){
        angles[i] = data.substring(startIndex,commandIndex).toInt();
        startIndex = commandIndex + 1;
      }else{
        angles[i] = data.substring(startIndex).toInt();
      }
    }*/
    //Atribuindo os valores dos ângulos recebidos aos servo motores
    //for(int j = 0; j < NUM_ANGLES; j++){
    //  pwm.setPWM(servoPin[i],0,degreeToPwm(angles[i]));
    //}
    /*if(angles[0]==0 && angles[1]==54 && angles[2]==-83 &&
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
    */
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
  //delay(20);
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
  //lcd.setCursor(0,0);
  char *pt = strtok(data,",");
  uint8_t i=0;
  //uint8_t size = sizeof(angles)/sizeof(angles[0]);
  while(pt!=NULL && i<size){
    angles[i] = atoi(pt);
    //lcd.print(angles[i]);
    //lcd.print(" ");
    pt = strtok(NULL,",");
    i++;
  }
  //delay(500);
  //for (int posicao = 0; posicao < 21; posicao++)
  //  {
  //  lcd.scrollDisplayLeft();
  //  delay(500);
  //  }
}
void sendingAnglesToServos(int *angles){
  //Atribuindo os valores dos ângulos recebidos aos servo motores
  for(int j = 0; j < NUM_ANGLES; j++){
    if(j>0 && j<3 || j>5 && j<9){
      pwm.setPWM(servoPins[j],0,degreeToPwmInverse(angles[j]));
    }
    if(j>2 && j<6 || j>8 && j<12){
      pwm.setPWM(servoPins[j],0,degreeToPwm(angles[j]));
    }
  }
  //j>2 && j<6 ||
  //j>0 && j<3 ||
  // || j>5 && j<9
  // || j>8 && j<12
}

