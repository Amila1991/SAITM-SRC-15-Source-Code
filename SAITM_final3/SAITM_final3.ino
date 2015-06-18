#include <QTRSensors.h>
#include <Encoder.h>
#include<Servo.h>
#include<EEPROM.h>

#define ENCODER_DO_NOT_USE_INTERRUPTS // check without this

#define NUM_SENSORS   8     // number of sensors used
#define TIMEOUT       2500  // waits for 2500 microseconds for sensor outputs to go low
#define EMITTER_PIN   2     // emitter is controlled by digital pin 2

// sensors 0 through 7 are connected to digital pins 3 through 10, respectively
QTRSensorsRC qtrrc((unsigned char[]) {37, 35, 33, 31, 29, 27, 25, 23},
  NUM_SENSORS, TIMEOUT, EMITTER_PIN); 
  
Encoder leftEnc(20, 20);  // Assign Encoders
Encoder rightEnc(21, 21);

Servo grabServo;
Servo armServo;

int calibratePin=36;
int buzzerPin=41;
int digtalFrontSensor=47;
int digtalFrontSensorLeft=49;
int digtalFrontSensorRight=51;


//Node 1 and 2 Encoder position
double node1Enpos=0;
double node2Enpos=0;
double distanceBetween2node=0;

int frontPath[]={0,1,2,12,22};
int backPath[]={22,12,2,1,0};
//int paths[]={0,1,11,21};
//int paths2[]={21,11,1,0};

int currentNode=0;
int driveDirection=0; // 0-front, 1-right, 2-back,3-left
//int path[]={0,1,2,12,11,1};
int count=0;

unsigned int sensorValues[NUM_SENSORS];
int sensorRead[8] ={0,0,0,0,0,0,0,0};
int error=0, previous_error=0, activeSensor=0,P=0,I=0,D=0;
float Kp=0.058, Ki=0, Kd=0.14, PID_value=0;

float Kpmin=0.06, Kimin=0, Kdmin=0.06, PID_valueMin=0;

//Motor Pins
int motor1[]={38,40};
int motor2[]={42,44};
int motorPWM[]={5,6}; 
 
int grabPos=0;
int armPos=180;

int gameMode=0;
int ballPlace=3;
int analogIRMax=0;
 
 
  
void setup(){
  for(int i=0;i<2;i++){
    pinMode(motor1[i],OUTPUT);
    pinMode(motor2[i],OUTPUT);
    pinMode(motorPWM[i],OUTPUT);
  }
  pinMode(calibratePin,INPUT);
  pinMode(digtalFrontSensor,INPUT);
  pinMode(digtalFrontSensorLeft,INPUT);
  pinMode(digtalFrontSensorRight,INPUT);
  //pinMode(buzzerPin,OUTPUT);
  pinMode(13,OUTPUT);
  Serial.begin(9600);
  Serial3.begin(9600);
  
  armServo.attach(8);
  grabServo.attach(9);
  grabPos=10;
  armPos=180;
  armServo.write(armPos);
  grabServo.write(grabPos);
  //if(digitalRead(calibratePin)==HIGH){
  // setQTRValues();
  //}else{
    sensorCalibrate(); 
 // }
  Serial3.println("start");
 
}

void loop(){
  gamePlay();
} 

void gamePlay(){
  if(gameMode==0){
      goForward(250,230);
      alignLine();
      gameMode++;
  }else if(gameMode==1){
      int sizeOfPath=sizeof(frontPath)/sizeof(int);
      Serial.println(sizeOfPath);
      findPlace(frontPath,sizeOfPath); 
      if(currentNode==frontPath[sizeOfPath-2] && digitalRead(digtalFrontSensor)==0){ // check ball place 1
          alignLine();
          if(digitalRead(digtalFrontSensor)==0){
             backwardDrive();
             analogWrite(motorPWM[0],255);   //Left Motor Speed
             analogWrite(motorPWM[1],255);
             delay(25);
            double newPos = (rightEnc.read()+leftEnc.read())/2;
            double gap=abs(((rightEnc.read()+leftEnc.read())/2)-newPos);
          while(digitalRead(digtalFrontSensor)!=1 && gap<300){
             analogWrite(motorPWM[0],170);   //Left Motor Speed
             analogWrite(motorPWM[1],170);
             readSensor();
             calculateError();
             gap=abs(((rightEnc.read()+leftEnc.read())/2)-newPos);
          }
          alignLine();
          if(digitalRead(digtalFrontSensor)==1){
          goBackardWithEncoders(250);
          while(digitalRead(digtalFrontSensor)!=0){
             readSensor();
             calculateError();
             if(activeSensor==8){
                 forwardDrive();
                 analogWrite(motorPWM[0],255);   //Left Motor Speed
                 analogWrite(motorPWM[1],255);
                 delay(25);
                 analogWrite(motorPWM[0],120);   //Left Motor Speed
                 analogWrite(motorPWM[1],120);
             }else{
                lineFollowSlow();
             }   
          }
          goForwardWithEncodersSlow(50);
          alignLine();
      }
      }
      
      ballPlace=1;
      //granb ball l
          grabOut(40);
          armDown(60);
          grabIn(0);
          armUp(180);
          
          //buzz(buzzerPin,5000,500); //indicate buzzer
          readSensor();
          calculateError();
          currentNode=12;
          if(activeSensor!=8){
            goForward(220,220);
          } 
          goForwardWithEncoders(150);
          lineFollow();
      }
      
      if(ballPlace!=1 && digitalRead(digtalFrontSensorLeft)==0){
        ballPlace=0;
      }
      
      if(currentNode==frontPath[sizeOfPath-1]){
          alignLine();  
          gameMode++;
      }else{
        lineFollow();
      }
  }else if(gameMode==2){
      while(activeSensor==8){
        if(digitalRead(digtalFrontSensorRight)==0){
             ballPlace=2; 
          }
          forwardDrive();
          analogWrite(motorPWM[0],190);   //Left Motor Speed
          analogWrite(motorPWM[1],170);
          readSensor();
          calculateError();
       //checkRightAnalogSensor check ball place 2 or check ball place 3  
       }
       if(digitalRead(digtalFrontSensorRight)==0){
             ballPlace=2;

          }

       alignLine();  
       Serial3.write("BAll Place 2");
       gameMode++;
  }else if(gameMode==3){
      Serial3.write("BAll Place :");
      Serial3.println(ballPlace);
       if(ballPlace==1){
         //goForwardWithEncodersWithoutAlign(100);
         double newPos = (rightEnc.read()+leftEnc.read())/2;
         double gap=abs(((rightEnc.read()+leftEnc.read())/2)-newPos);
         while(gap<200){
             lineFollow();
             gap=abs(((rightEnc.read()+leftEnc.read())/2)-newPos);
         }
      // grab process and  atfer come to the node 22 
       }else{
           if(ballPlace==0){
               turnLeftWithoutForward();
               alignLine();
               while(digitalRead(digtalFrontSensor)==1){
                 lineFollowSlow();
               }
               alignLine();
               if(digitalRead(digtalFrontSensor)==0){
                 backwardDrive();
                 analogWrite(motorPWM[0],255);   //Left Motor Speed
                 analogWrite(motorPWM[1],255);
                 delay(25);
                 double newPos = (rightEnc.read()+leftEnc.read())/2;
                 double gap=abs(((rightEnc.read()+leftEnc.read())/2)-newPos);
                while(digitalRead(digtalFrontSensor)!=1 && gap<300){
                   analogWrite(motorPWM[0],100);   //Left Motor Speed
                   analogWrite(motorPWM[1],100);
                   readSensor();
                   calculateError();
                   gap=abs(((rightEnc.read()+leftEnc.read())/2)-newPos);
                }
                
              }
                //3 250
              if(digitalRead(digtalFrontSensor)==1){  
                alignLine();
                  goBackardWithEncodersSlow(150);
                  while(digitalRead(digtalFrontSensor)!=0){
                     readSensor();
                     calculateError();
                     if(activeSensor==8){
                       forwardDrive();
                       analogWrite(motorPWM[0],100);   //Left Motor Speed
                       analogWrite(motorPWM[1],100);
                   }else{
                      lineFollowSlow();
                   }   
                }
                goForwardWithEncoders(100);
            }
                
                stopDrive();
            //granb ball l
                grabOut(50);
                armDown(60);
                grabIn(0);
                armUp(180);
                
                goForwardWithEncoders(400);
                turnBackRight();
                alignLine();
            }else if(ballPlace==2){
             goForwardWithEncodersSlow(150);
             semiTurnRight();
             goBackardWithEncodersSlow(150);
             semiTurnRight();
             alignLine();
             while(digitalRead(digtalFrontSensor)==1){
               lineFollowSlow();
             }
             if(digitalRead(digtalFrontSensor)==0){
                   backwardDrive();
                   analogWrite(motorPWM[0],255);   //Left Motor Speed
                   analogWrite(motorPWM[1],255);
                   delay(25);
                   double newPos = (rightEnc.read()+leftEnc.read())/2;
                   double gap=abs(((rightEnc.read()+leftEnc.read())/2)-newPos);
                while(digitalRead(digtalFrontSensor)!=1 && gap<200){
                   analogWrite(motorPWM[0],100);   //Left Motor Speed
                   analogWrite(motorPWM[1],100);
                   readSensor();
                   calculateError();
                   gap=abs(((rightEnc.read()+leftEnc.read())/2)-newPos);
                }
                alignLine();
                //3 250
                //goBackardWithEncodersSlow(50);
                if(digitalRead(digtalFrontSensor)==1){
                while(digitalRead(digtalFrontSensor)!=0){
                   readSensor();
                   calculateError();
                   if(activeSensor==8){
                       forwardDrive();
                       analogWrite(motorPWM[0],255);   //Left Motor Speed
                       analogWrite(motorPWM[1],255);
                       delay(25);
                       analogWrite(motorPWM[0],100);   //Left Motor Speed
                       analogWrite(motorPWM[1],100);
                   }else{
                      lineFollowSlow();
                   }   
                }
                }
            }
 
                alignLine();
            //granb ball l
                grabOut(70);
                armDown(60);
                grabIn(0);
                armUp(180);

                
                goForwardWithEncoders(300);
                turnBackRight();
                alignLine();          
          }else if(ballPlace==3){
            Serial3.println("ball Place 3");
                goForwardWithEncodersSlow(75);
                 semiTurnRight();
                 goForwardWithEncodersSlow(50);
                 while(digitalRead(digtalFrontSensor)==1){
                   lineFollowSlow();
                 }
                 alignLine();
                 if(digitalRead(digtalFrontSensor)==0){
                       backwardDrive();
                       analogWrite(motorPWM[0],255);   //Left Motor Speed
                       analogWrite(motorPWM[1],255);
                       delay(25);
                       double newPos = (rightEnc.read()+leftEnc.read())/2;
                       double gap=abs(((rightEnc.read()+leftEnc.read())/2)-newPos);
                    while(digitalRead(digtalFrontSensor)!=1 && gap<300){
                       analogWrite(motorPWM[0],100);   //Left Motor Speed
                       analogWrite(motorPWM[1],100);
                       readSensor();
                       calculateError();
                       gap=abs(((rightEnc.read()+leftEnc.read())/2)-newPos);
                    }
                 }
                    if(digitalRead(digtalFrontSensor)==1){
                    while(digitalRead(digtalFrontSensor)!=0){
                       readSensor();
                       calculateError();
                       if(activeSensor==8){
                           forwardDrive();
                           analogWrite(motorPWM[0],100);   //Left Motor Speed
                           analogWrite(motorPWM[1],100);
                       }else{
                          lineFollowSlow();
                       }   
                    }
                    goForwardWithEncodersSlow(100);
                }
                    stopDrive();
                //granb ball l
                    grabOut(50);
                    armDown(60);
                    grabIn(0);
                    armUp(180);
                    
                    goForwardWithEncoders(450);
                    turnBackLeft();
                    alignLine();
           }
            
            while(activeSensor!=8){
              lineFollow();
            }
            if(ballPlace==0){
              goForwardWithEncoders(100);
              turnLeft();
            }else if(ballPlace==2){
              goForward(190,170);
              semiTurnRight();
            }else if(ballPlace==3){
              //goForward(190,170);
              turnRight();
            }
            
            alignLine();
       }
       gameMode++;
  }else if(gameMode==4){
    while(activeSensor!=8){
      lineFollow();
    }
    Serial3.write("Ball throw");
    goForwardWithEncoders(50);
    //ball grab out
    armDown(100);
    grabOut(120);
    armUpSpeed(180);
    grabIn(10); 
    gameMode++;
  }else if(gameMode==5){
    //goBackardWithEncoders(50);
    turnBackRightWithoutForward();   
    goForward(230,210);
    gameMode++;
  }else if(gameMode==6){
    lineFollow();
    if(activeSensor==0){
      stopDrive();
      //delay(25);
   double newPos = (rightEnc.read()+leftEnc.read())/2;
  double gap=abs(((rightEnc.read()+leftEnc.read())/2)-newPos);
  readSensor();
  calculateError();  
  int leftPWM=180;
  int rightPWM=180;
  while(gap<50 && (sensorRead[2]!=1 && sensorRead[3]!=1 && sensorRead[4]!=1 && sensorRead[5]!=1)){
          leftforwardDrive();
          rightbackwardDrive();
          analogWrite(motorPWM[0],leftPWM);   //Left Motor Speed
          analogWrite(motorPWM[1],rightPWM);
          gap=abs(((rightEnc.read()+leftEnc.read())/2)-newPos);
          readSensor();
          calculateError();
          
          
  }
  gameMode++;
  }
  }else if(gameMode==7){
    lineFollow();
    if(activeSensor==0){
     turnLeftWithoutForward();
    }
    }else if(activeSensor==8){
      if(goForwardCheck(250,250)){
        gameMode++;
      }
      
    
  }else if(gameMode==8){
    while(activeSensor!=8){
      lineFollow();
    }
    //goForwardWithEncoders(150);
    stopDrive();
    gameMode++;
  }
  
  
  
}


//////////////////////////////////////////////////////////////////////

//0,1,2,12,22
void findPlace(int path[],int sizeOfPath){
  double newPos = (rightEnc.read()+leftEnc.read())/2;
  double gap=abs(((rightEnc.read()+leftEnc.read())/2)-newPos);
  while(currentNode!=path[sizeOfPath-2] && currentNode!=path[sizeOfPath-1]){
    calibrateDistanceBetween2Node();  
    if(activeSensor==8){
    newPos = gap;
  double gap=abs(((rightEnc.read()+leftEnc.read())/2)-newPos);  
    calibrateDistanceBetween2Node();
    if(gap-newPos >=(distanceBetween2node)/2){
      setCurrentNode();
    }
    nextNodeToDrive(path[count+2]);
    count++;            
    }
    lineFollow();
  }
  if(activeSensor==8 && count!=0){
  setCurrentNode();
  count=0;
  }
}

/////////////////////////////////////////////////

void lineFollow(){
  readSensor();
  calculateError();
  if(activeSensor!=8){
  motor_control();
  }
  Serial.println(error);
  Serial.println(PID_value);
  Serial.println();
 
}

/////////////////////////////////////////////////////////////////////

void lineFollowSlow(){
  readSensor();
  calculateError();
  if(activeSensor!=8){
    motor_controlSlow();
  }
}

/////////////////////////////////////////////////////////////////////

void sensorCalibrate(){
  digitalWrite(13, HIGH);    // turn on Arduino's LED to indicate we are in calibration mode
  for (int i = 0; i < 400; i++)  // make the calibration take about 10 seconds
  {
    qtrrc.calibrate();       // reads all sensors 10 times at 2500 us per read (i.e. ~25 ms per call)
  }
  digitalWrite(13, LOW);     // turn off Arduino's LED to indicate we are through with calibration

  //writeEEPROMQTRValues();
  // print the calibration minimum values measured when emitters were on
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtrrc.calibratedMinimumOn[i]);
    Serial.print(' ');
  }
  Serial.println();
  
  // print the calibration maximum values measured when emitters were on
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtrrc.calibratedMaximumOn[i]);
    Serial.print(' ');
  }
}

///////////////////////////////////////////////////////////////////////////////

void writeEEPROMQTRValues(){
  for ( int i = 0 ; i < EEPROM.length() ; i++ ){
    EEPROM.write(i, 0);
  }
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    EEPROM.write(i,qtrrc.calibratedMinimumOn[i]);
    EEPROM.write(8+i,qtrrc.calibratedMinimumOff[i]);
    EEPROM.write(16+i,qtrrc.calibratedMaximumOn[i]);
    EEPROM.write(24+i,qtrrc.calibratedMaximumOff[i]);
  }
}

void setQTRValues(){
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    qtrrc.calibratedMinimumOn[i]=EEPROM.read(i);
    qtrrc.calibratedMinimumOff[i]=EEPROM.read(8+i);
    qtrrc.calibratedMaximumOn[i]=EEPROM.read(16+i);
    qtrrc.calibratedMaximumOff[i]=EEPROM.read(24+i);
  }
}

//////////////////////////////////////////////////////////////////////////////

void readSensor(){
  int blackline=0;
    // read calibrated sensor values and obtain a measure of the line position from 0 to 5000
  // To get raw sensor values, call:
  //  qtrrc.read(sensorValues); instead of unsigned int position = qtrrc.readLine(sensorValues);
  unsigned int position = qtrrc.readLine(sensorValues);

  // print the sensor values as numbers from 0 to 1000, where 0 means maximum reflectance and
  // 1000 means minimum reflectance, followed by the line position
  for (unsigned char i = 0; i < NUM_SENSORS; i++)
  {
    if(sensorValues[i]<200){
        sensorRead[i]=0;
    }else{
      blackline++;
      sensorRead[i]=1;
    }
  }
}

/////////////////////////////////////////////////////////////////////////////////

void calculateError(){
  activeSensor=0;
  error=0;
  for(int i=0;i<NUM_SENSORS/2;i++){
    if(sensorRead[i]==1){
      activeSensor++;  
    }
    
    error+=sensorRead[i]*((NUM_SENSORS/2)-i);
  }
  for(int i=NUM_SENSORS-1;i>=NUM_SENSORS/2;i--){
     if(sensorRead[i]==1){
      activeSensor++;
    }
    error-=sensorRead[i]*(i-(NUM_SENSORS/2)+1);
    error=qtrrc.readLine(sensorValues)-3500;
  }
  if((error>0 && error<400) || error>3100 ){
    error=0;
    Serial.println(activeSensor);
   //Find new Node
    
  }
  if(activeSensor==8){
                      
  }else if(activeSensor==7){
    activeSensor=8;
  }else if((activeSensor==6 && (sensorRead[0]==1 && sensorRead[1]==1 
    && sensorRead[2]==1 && sensorRead[3]==1 && sensorRead[4]==1 && sensorRead[5]==1)) || (activeSensor==5 && (sensorRead[0]==1 && sensorRead[1]==1 
    && sensorRead[2]==1 && sensorRead[3]==1 && sensorRead[4]==1))){
      activeSensor=8;
   }else if((activeSensor==6 && (sensorRead[2]==1 && sensorRead[3]==1 
    && sensorRead[4]==1 && sensorRead[5]==1 && sensorRead[6]==1 && sensorRead[7]==1)) || (activeSensor==5 && (sensorRead[3]==1 
    && sensorRead[4]==1 && sensorRead[5]==1 && sensorRead[6]==1 && sensorRead[7]==1))){
      activeSensor=8;
  }
}

////////////////////////////////////////////////////////////////////////////

void calculatePID(){
  calculateError();
   P = error;
   I = I + error;
   D = error - previous_error;

    
   PID_value = (Kp*P) + (Ki*I) + (Kd*D);
    
   previous_error=error;
}

////////////////////////////////////////////////////////////////////////////

void calculatePIDMin(){
  calculateError();
   P = error;
   I = I + error;
   D = error - previous_error;

    
   PID_valueMin = (Kpmin*P) + (Kimin*I) + (Kdmin*D);
    
   previous_error=error;
}

/////////////////////////////////////////////////////////////////////

void motor_control()
{
   calculatePID();
   int initial_motor_speed=255;
   // Calculating the effective motor speed:
   int left_motor_speed = initial_motor_speed-PID_value;
   int right_motor_speed = initial_motor_speed+PID_value;
    
   // The moto\speed should not exceed the max PWM value
   left_motor_speed=constrain(left_motor_speed,0,255);
   right_motor_speed=constrain(right_motor_speed,0,255);
   Serial.println(left_motor_speed);
   Serial.println(right_motor_speed);
    
   forwardDrive();
   analogWrite(motorPWM[0],left_motor_speed);   //Left Motor Speed
   analogWrite(motorPWM [1],right_motor_speed);  //Right Motor Speed
}
///////////////////////////////////////////////////////////////////////////

void motor_controlSlow()
{
   calculatePIDMin();
   int initial_motor_speed=120;
   // Calculating the effective motor speed:
   int left_motor_speed = initial_motor_speed-PID_valueMin;
   int right_motor_speed = initial_motor_speed+PID_valueMin;
    
   // The moto\speed should not exceed the max PWM value
   left_motor_speed=constrain(left_motor_speed,0,255);
   right_motor_speed=constrain(right_motor_speed,0,255);
   Serial.println(left_motor_speed);
   Serial.println(right_motor_speed);
    
   forwardDrive();
   analogWrite(motorPWM[0],left_motor_speed);   //Left Motor Speed
   analogWrite(motorPWM [1],right_motor_speed);  //Right Motor Speed
}

///////////////////////////////////////////////////////////////////////////

void setCurrentNode(){
  if(driveDirection==0){
    currentNode++;
  }else if(driveDirection==1){
    currentNode+=10;
  }else if(driveDirection==2){
    currentNode--;
  }else{
    currentNode-=10;
  } 
}

/////////////////////////////////////////////////////////////////////

int getNextNodeDirection(int nextNode){
  int nextDirection=nextNode-currentNode;
  if(nextDirection==1){
    return 0;
  }else if(nextDirection==10){
    return 1;
  }else if(nextDirection==-1){
    return 2; 
  }else if(nextDirection==-10){
    return 3;
  }
  return driveDirection;
}

////////////////////////////////////////////////////////////////////////

void nextNodeToDrive(int nextNode){
  Serial.println(nextNode);
  int turnDirection=getNextNodeDirection(nextNode)-driveDirection;
  Serial.println(getNextNodeDirection(nextNode));
  if(turnDirection==0){
    //stopDrive();
    goForward(250,230);
  }else if(turnDirection==1){
      turnRight();
      alignLine();
  }else if(turnDirection==2){
      turnRight();
      turnRight();
      alignLine();
  }else{
      turnLeft();
      alignLine();
    }
}

//////////////////////////////////////////////////////////////////////////////

void goForward(int leftPWM,int rightPWM){
     readSensor();
     calculateError();
     forwardDrive();
     analogWrite(motorPWM[0],255);   //Left Motor Speed
     analogWrite(motorPWM[1],255);
     delay(25);
     while(activeSensor==8){
       analogWrite(motorPWM[0],leftPWM);   //Left Motor Speed
       analogWrite(motorPWM[1],rightPWM);
       readSensor();
       calculateError();
     }
}
//////////////////////////////////////////////////////////////////////////////

bool goForwardCheck(int leftPWM,int rightPWM){
     readSensor();
     calculateError();
     forwardDrive();
     
     analogWrite(motorPWM[0],255);   //Left Motor Speed
     analogWrite(motorPWM[1],255);
     delay(25);
     double newPos = (rightEnc.read()+leftEnc.read())/2;
    double gap=abs(((rightEnc.read()+leftEnc.read())/2)-newPos);
     while(activeSensor==8 && gap<300){
       analogWrite(motorPWM[0],leftPWM);   //Left Motor Speed
       analogWrite(motorPWM[1],rightPWM);
       readSensor();
       calculateError();
       gap=abs(((rightEnc.read()+leftEnc.read())/2)-newPos);
     }
     if(gap>=300){
       return true;
     }
     return false;
}

/////////////////////////////////////////////////////////////////////////////

void goBackward(int leftPWM,int rightPWM){
     readSensor();
     calculateError();
     backwardDrive();
     analogWrite(motorPWM[0],255);   //Left Motor Speed
     analogWrite(motorPWM[1],255);
     delay(25);
     while(activeSensor==8){
       analogWrite(motorPWM[0],leftPWM);   //Left Motor Speed
       analogWrite(motorPWM[1],rightPWM);
       readSensor();
       calculateError();
     }
}

///////////////////////////////////////////////////////////////////////////////

void goForwardWithEncoders(int NoofEncoder){
  stopDrive();
  double newPos = (rightEnc.read()+leftEnc.read())/2;
  double gap=abs(((rightEnc.read()+leftEnc.read())/2)-newPos);
  readSensor();
       calculateError();
     while(gap<NoofEncoder){
          gap=abs(((rightEnc.read()+leftEnc.read())/2)-newPos);
          if(activeSensor!=8){
            lineFollow();
          }else{
            forwardDrive();
            analogWrite(motorPWM[0],200);   //Left Motor Speed
            analogWrite(motorPWM[1],200);
            readSensor();
            calculateError();
          }
    } 
    readSensor();
    calculateError();
   if(activeSensor!=0){ 
    alignLine();
   }
}

/////////////////////////////////////////////////////////////////////////////////

void goForwardWithEncodersWithoutAlign(int NoofEncoder){
  stopDrive();
  double newPos = (rightEnc.read()+leftEnc.read())/2;
  double gap=abs(((rightEnc.read()+leftEnc.read())/2)-newPos);
  readSensor();
       calculateError();
     while(gap<NoofEncoder){
          gap=abs(((rightEnc.read()+leftEnc.read())/2)-newPos);
          if(activeSensor!=8){
            lineFollow();
          }else{
            forwardDrive();
            analogWrite(motorPWM[0],200);   //Left Motor Speed
            analogWrite(motorPWM[1],200);
            readSensor();
            calculateError();
          }
    } 
}

///////////////////////////////////////////////////////////////////////////////

void goForwardWithEncodersSlow(int NoofEncoder){
  stopDrive();
  double newPos = (rightEnc.read()+leftEnc.read())/2;
  double gap=abs(((rightEnc.read()+leftEnc.read())/2)-newPos);
  readSensor();
       calculateError();
       forwardDrive();
       delay(25);
     while(gap<NoofEncoder){
          gap=abs(((rightEnc.read()+leftEnc.read())/2)-newPos);
          if(activeSensor!=8){
            lineFollowSlow();
          }else{
            analogWrite(motorPWM[0],120);   //Left Motor Speed
            analogWrite(motorPWM[1],120);
            readSensor();
            calculateError();
          }
    }  
    alignLine();
}

/////////////////////////////////////////////////////////////////////////////////

void goBackardWithEncoders(int NoofEncoder){
  stopDrive();
  double newPos = (rightEnc.read()+leftEnc.read())/2;
  double gap=abs(((rightEnc.read()+leftEnc.read())/2)-newPos);
  readSensor();
       calculateError();
     while(gap<NoofEncoder){
          gap=abs(((rightEnc.read()+leftEnc.read())/2)-newPos);
          backwardDrive();
          analogWrite(motorPWM[0],200);   //Left Motor Speed
          analogWrite(motorPWM[1],200);
          readSensor();
          calculateError();
          
    }  
    //alignLine();
}


/////////////////////////////////////////////////////////////////////////////////

void goBackardWithEncodersSlow(int NoofEncoder){
  stopDrive();
  double newPos = (rightEnc.read()+leftEnc.read())/2;
  double gap=abs(((rightEnc.read()+leftEnc.read())/2)-newPos);
  readSensor();
     calculateError();
     backwardDrive();
     delay(25);
     while(gap<NoofEncoder){
          gap=abs(((rightEnc.read()+leftEnc.read())/2)-newPos);
          analogWrite(motorPWM[0],120);   //Left Motor Speed
          analogWrite(motorPWM[1],120);
          readSensor();
          calculateError();
          
    }  
    //alignLine();
}


////////////////////////////////////////////////////////////////////////////

void turnRight(){
  stopDrive();
  driveDirection++;
  driveDirection%=4;
  goForward(190,170);
  goForwardWithEncoders(150);
     double newPos = (rightEnc.read()+leftEnc.read())/2;
     double gap=abs(((rightEnc.read()+leftEnc.read())/2)-newPos);
     newPos = (rightEnc.read()+leftEnc.read())/2;
     gap=abs(((rightEnc.read()+leftEnc.read())/2)-newPos);
     readSensor();
     calculateError();   
     int leftPWM=220;
     int rightPWM=220;
     //right 475,
    while(gap<450){
          leftbackwardDrive();
          rightforwardDrive();
          analogWrite(motorPWM[0],leftPWM);   //Left Motor Speed
          analogWrite(motorPWM[1],rightPWM);
          gap=abs(((rightEnc.read()+leftEnc.read())/2)-newPos);
          readSensor();
          calculateError();
        }  
     leftPWM=120;
     rightPWM=120;
     while(gap<50 ||(sensorRead[2]!=1 && sensorRead[3]!=1 && sensorRead[4]!=1 && sensorRead[5]!=1)){
          leftbackwardDrive();
          rightforwardDrive();
          analogWrite(motorPWM[0],leftPWM);   //Left Motor Speed
          analogWrite(motorPWM[1],rightPWM);
          gap=abs(((rightEnc.read()+leftEnc.read())/2)-newPos);
          readSensor();
          calculateError();
        }
        previous_error=0;
}

/////////////////////////////////////////////////////////////////////////////////

void semiTurnRight(){
  stopDrive();
  driveDirection++;
  driveDirection%=4;
     double newPos = (rightEnc.read()+leftEnc.read())/2;
     double gap=abs(((rightEnc.read()+leftEnc.read())/2)-newPos);
     newPos = (rightEnc.read()+leftEnc.read())/2;
     gap=abs(((rightEnc.read()+leftEnc.read())/2)-newPos);
     readSensor();
     calculateError();   
     int leftPWM=180;
     int rightPWM=180;
     leftbackwardDrive();
     rightforwardDrive();
     delay(20);
     //right 475,
    while(gap<200){
          leftbackwardDrive();
          rightforwardDrive();
          analogWrite(motorPWM[0],leftPWM);   //Left Motor Speed
          analogWrite(motorPWM[1],rightPWM);
          gap=abs(((rightEnc.read()+leftEnc.read())/2)-newPos);
          readSensor();
          calculateError();
        }  
     leftPWM=100;
     rightPWM=100;
     while(gap<50 || (sensorRead[2]!=1 && sensorRead[3]!=1 && sensorRead[4]!=1 && sensorRead[5]!=1)){
          leftbackwardDrive();
          rightforwardDrive();
          analogWrite(motorPWM[0],leftPWM);   //Left Motor Speed
          analogWrite(motorPWM[1],rightPWM);
          gap=abs(((rightEnc.read()+leftEnc.read())/2)-newPos);
          readSensor();
          calculateError();
        }
        previous_error=0;
}

/////////////////////////////////////////////////////////////////////////////////


void turnBackRight(){
  stopDrive();
  driveDirection++;
  driveDirection%=4;
  goForward(190,170);
  goForwardWithEncoders(200);
     double newPos = (rightEnc.read()+leftEnc.read())/2;
     double gap=abs(((rightEnc.read()+leftEnc.read())/2)-newPos);
     readSensor();
     calculateError(); 
    int leftPWM=220;
    int rightPWM=220;
    leftbackwardDrive();
    rightforwardDrive();
    delay(25);
     //right 475,
    while(gap<1100){
          leftbackwardDrive();
          rightforwardDrive();
          analogWrite(motorPWM[0],leftPWM);   //Left Motor Speed
          analogWrite(motorPWM[1],rightPWM);
          gap=abs(((rightEnc.read()+leftEnc.read())/2)-newPos);
          readSensor();
          calculateError();
        }  
     leftPWM=180;
     rightPWM=180;
     while(gap<50 ||(sensorRead[2]!=1 && sensorRead[3]!=1 && sensorRead[4]!=1 && sensorRead[5]!=1)){
          leftbackwardDrive();
          rightforwardDrive();
          analogWrite(motorPWM[0],leftPWM);   //Left Motor Speed
          analogWrite(motorPWM[1],rightPWM);
          gap=abs(((rightEnc.read()+leftEnc.read())/2)-newPos);
          readSensor();
          calculateError();
        }
        //alignLine();
        previous_error=0;
}
///////////////////////////////////////////////////////////////////////////////////////////////

void turnBackRightWithoutForward(){
  stopDrive();
  driveDirection++;
  driveDirection%=4;
     double newPos = (rightEnc.read()+leftEnc.read())/2;
     double gap=abs(((rightEnc.read()+leftEnc.read())/2)-newPos);
     readSensor();
     calculateError(); 
    int leftPWM=220;
    int rightPWM=220;
    leftbackwardDrive();
    rightforwardDrive();
    delay(25);
     //right 475,
    while(gap<1300){
          leftbackwardDrive();
          rightforwardDrive();
          analogWrite(motorPWM[0],leftPWM);   //Left Motor Speed
          analogWrite(motorPWM[1],rightPWM);
          gap=abs(((rightEnc.read()+leftEnc.read())/2)-newPos);
          readSensor();
          calculateError();
        }  
     leftPWM=120;
     rightPWM=120;
     while(gap<50 ||(sensorRead[2]!=1 && sensorRead[3]!=1 && sensorRead[4]!=1 && sensorRead[5]!=1)){
          leftbackwardDrive();
          rightforwardDrive();
          analogWrite(motorPWM[0],leftPWM);   //Left Motor Speed
          analogWrite(motorPWM[1],rightPWM);
          gap=abs(((rightEnc.read()+leftEnc.read())/2)-newPos);
          readSensor();
          calculateError();
        }
        //alignLine();
        previous_error=0;
}

/////////////////////////////////////////////////////////////////////////////////////////////

void turnLeft(){
  stopDrive();
  driveDirection--;
  error=error<0?-error:error;
  if(driveDirection<0){
    driveDirection+=4;
  }
  goForward(190,170);
  goForwardWithEncoders(150);
  double newPos = (rightEnc.read()+leftEnc.read())/2;
  double gap=abs(((rightEnc.read()+leftEnc.read())/2)-newPos);
  readSensor();
  calculateError();  
  int leftPWM=220;
  int rightPWM=220;
  leftforwardDrive();
   rightbackwardDrive();
   delay(25);
  //left 500
  while(gap<450){
   leftforwardDrive();
   rightbackwardDrive();
   analogWrite(motorPWM[0],leftPWM);   //Left Motor Speed
   analogWrite(motorPWM[1],rightPWM);
   gap=abs(((rightEnc.read()+leftEnc.read())/2)-newPos);
   readSensor();
   calculateError();
  }  
  leftPWM=180;
  rightPWM=180;
  while(gap<50 ||(sensorRead[2]!=1 && sensorRead[3]!=1 && sensorRead[4]!=1 && sensorRead[5]!=1)){
          leftforwardDrive();
          rightbackwardDrive();
          analogWrite(motorPWM[0],leftPWM);   //Left Motor Speed
          analogWrite(motorPWM[1],rightPWM);
          gap=abs(((rightEnc.read()+leftEnc.read())/2)-newPos);
          readSensor();
          calculateError();
        }
        readSensor();
        calculateError();
        
        previous_error=0;  
}

/////////////////////////////////////////////////////////////////////////////
void turnLeftWithoutForward(){
  stopDrive();
  driveDirection--;
  error=error<0?-error:error;
  if(driveDirection<0){
    driveDirection+=4;
  }
  goForward(190,170);
  //goForwardWithEncoders(150);
  double newPos = (rightEnc.read()+leftEnc.read())/2;
  double gap=abs(((rightEnc.read()+leftEnc.read())/2)-newPos);
  readSensor();
  calculateError();  
  int leftPWM=220;
  int rightPWM=220;
  leftforwardDrive();
   rightbackwardDrive();
   delay(25);
  //left 500
  while(gap<450){
   leftforwardDrive();
   rightbackwardDrive();
   analogWrite(motorPWM[0],leftPWM);   //Left Motor Speed
   analogWrite(motorPWM[1],rightPWM);
   gap=abs(((rightEnc.read()+leftEnc.read())/2)-newPos);
   readSensor();
   calculateError();
  }  
  leftPWM=180;
  rightPWM=180;
  while(gap<50 ||(sensorRead[2]!=1 && sensorRead[3]!=1 && sensorRead[4]!=1 && sensorRead[5]!=1)){
          leftforwardDrive();
          rightbackwardDrive();
          analogWrite(motorPWM[0],leftPWM);   //Left Motor Speed
          analogWrite(motorPWM[1],rightPWM);
          gap=abs(((rightEnc.read()+leftEnc.read())/2)-newPos);
          readSensor();
          calculateError();
        }
        readSensor();
        calculateError();
        
        previous_error=0;  
}

/////////////////////////////////////////////////////////////////////////////

void semiTurnLeft(){
  stopDrive();
  driveDirection--;
  if(driveDirection<0){
    driveDirection+=4;
  }
  goForward(190,170);
  double newPos = (rightEnc.read()+leftEnc.read())/2;
  double gap=abs(((rightEnc.read()+leftEnc.read())/2)-newPos);
  readSensor();
  calculateError();  
  int leftPWM=180;
  int rightPWM=180;
  leftforwardDrive();
   rightbackwardDrive();
   delay(25);
  //left 500
  while(gap<200){
   leftforwardDrive();
   rightbackwardDrive();
   analogWrite(motorPWM[0],leftPWM);   //Left Motor Speed
   analogWrite(motorPWM[1],rightPWM);
   gap=abs(((rightEnc.read()+leftEnc.read())/2)-newPos);
   readSensor();
   calculateError();
  }  
  leftPWM=100;
  rightPWM=100;
  while(gap<50 ||(sensorRead[2]!=1 && sensorRead[3]!=1 && sensorRead[4]!=1 && sensorRead[5]!=1)){
          leftforwardDrive();
          rightbackwardDrive();
          analogWrite(motorPWM[0],leftPWM);   //Left Motor Speed
          analogWrite(motorPWM[1],rightPWM);
          gap=abs(((rightEnc.read()+leftEnc.read())/2)-newPos);
          readSensor();
          calculateError();
        }
        previous_error=0;  
}

/////////////////////////////////////////////////////////////////////////////

void turnBackLeft(){
  stopDrive();
  driveDirection++;
  driveDirection%=4;
  goForward(190,170);
  goForwardWithEncoders(100);
     double newPos = (rightEnc.read()+leftEnc.read())/2;
     double gap=abs(((rightEnc.read()+leftEnc.read())/2)-newPos);
     readSensor();
     calculateError(); 
    int leftPWM=220;
    int rightPWM=220;
     //right 475,
    while(gap<1100){
          leftforwardDrive();
          rightbackwardDrive();
          analogWrite(motorPWM[0],leftPWM);   //Left Motor Speed
          analogWrite(motorPWM[1],rightPWM);
          gap=abs(((rightEnc.read()+leftEnc.read())/2)-newPos);
          readSensor();
          calculateError();
        }  
     leftPWM=180;
     rightPWM=180;
     while(gap<50 ||(sensorRead[2]!=1 && sensorRead[3]!=1 && sensorRead[4]!=1 && sensorRead[5]!=1)){
          leftforwardDrive();
          rightbackwardDrive();
          analogWrite(motorPWM[0],leftPWM);   //Left Motor Speed
          analogWrite(motorPWM[1],rightPWM);
          gap=abs(((rightEnc.read()+leftEnc.read())/2)-newPos);
          readSensor();
          calculateError();
        }
        alignLine();
        previous_error=0;
}

/////////////////////////////////////////////////////////////////////////////////

///////////////////////// Line Alignmnet //////////////////////////////////////////////

void alignLine(){
  stopDrive();
  delay(20);
    readSensor();
    calculateError();
    while(activeSensor!=0 && activeSensor!=8 && (sensorRead[4]!=1 || sensorRead[3]!=1 || ((sensorRead[2]!=1 && sensorRead[5]!=1)))){
          if(error>0){
            leftforwardDrive();
          rightbackwardDrive();
          Serial3.write('align');
          analogWrite(motorPWM[0],120);   //Left Motor Speed
          analogWrite(motorPWM[1],120);
          }else{
          leftbackwardDrive();
          rightforwardDrive();
          analogWrite(motorPWM[0],120);   //Left Motor Speed
          analogWrite(motorPWM[1],120);
          }
     readSensor();
    calculateError();       
  }
  stopDrive();
  //delay(1000);
}

/////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////Calculate distance between node ////////////////////////////

void calibrateDistanceBetween2Node(){
  if(currentNode==1 && node1Enpos==0){
    node1Enpos=(rightEnc.read()+leftEnc.read())/2;
    Serial3.write("start  : ");
    Serial3.println(node1Enpos);
  }else if(currentNode==2 && node2Enpos==0){
    node2Enpos=(rightEnc.read()+leftEnc.read())/2;
    distanceBetween2node=node2Enpos- node1Enpos;
    Serial3.write("\nend : ");
    Serial3.println(node2Enpos);
    Serial3.write("\ndistance : ");
    Serial3.println(setNodeDistance(distanceBetween2node));
    Serial3.write("\nanalog sensor value : ");
    Serial3.println(analogIRMax);
    /*if(analogIRMax>200){ ///// 200 is a tempory value. we want analysis about sharp IR values
      ballPlace=3;
      analogIRMax=0;
    }*/
  }
}

int setNodeDistance(double encodeDistance){
  if(encodeDistance<=1125 && encodeDistance>850){
    return 1;    //20cm
  }else if(encodeDistance<=1350 && encodeDistance>1125){
    return 2;    //25cm
  }else if(encodeDistance<=1525 && encodeDistance>1350){
    return 3;    //30cm
  }else if(encodeDistance<=1750 && encodeDistance>1525){
    return 4;     //35cm
  }else if(encodeDistance<=1950 && encodeDistance>1750){
    return 5;    //40cm
  }else{
    return 0; 
  }
}

////////////////////////////////////////////////////////

///////////////Motor Direction Conntrol ///////////////////////////////

void forwardDrive(){
  digitalWrite(motor1[0],HIGH);
  digitalWrite(motor1[1],LOW);
  digitalWrite(motor2[0],HIGH);
  digitalWrite(motor2[1],LOW);
}

void leftforwardDrive(){
  digitalWrite(motor1[0],HIGH);
  digitalWrite(motor1[1],LOW);
}

void rightforwardDrive(){
  digitalWrite(motor2[0],HIGH);
  digitalWrite(motor2[1],LOW);
}

void backwardDrive(){
  digitalWrite(motor1[0],LOW);
  digitalWrite(motor1[1],HIGH);
  digitalWrite(motor2[0],LOW);
  digitalWrite(motor2[1],HIGH);
}

void leftbackwardDrive(){
  digitalWrite(motor1[0],LOW);
  digitalWrite(motor1[1],HIGH);
}

void rightbackwardDrive(){
  digitalWrite(motor2[0],LOW);
  digitalWrite(motor2[1],HIGH);
}

void stopDrive(){
  analogWrite(motorPWM[0],255);
  analogWrite(motorPWM[1],255);
  digitalWrite(motor1[0],HIGH);
  digitalWrite(motor1[1],HIGH);
  digitalWrite(motor2[0],HIGH);
  digitalWrite(motor2[1],HIGH);
}

//////////////////////////////////////////////////////////////////

/////////////// Arm Control  ////////////////////////////////////

void grabIn(int targetPos){
  for(grabPos;grabPos>=targetPos;grabPos--){
    grabServo.write(grabPos);
    delay(5);
  }
}

void grabOut(int targetPos){
  for(grabPos;grabPos<=targetPos;grabPos++){
    grabServo.write(grabPos);
    delay(5);
  }
}

void armUp(int targetPos){
  for(armPos;armPos<=targetPos;armPos++){
    armServo.write(armPos);
    delay(10);
  }
}

void armUpSpeed(int targetPos){
  for(armPos;armPos<=targetPos;armPos++){
    armServo.write(armPos);
    delay(5);
  }
}

void armDown(int targetPos){
  for(armPos;armPos>=targetPos;armPos--){
    armServo.write(armPos);
    delay(5);
  }
}

////////////////////////////////////////////////////////////////////


void buzz(int targetPin, long frequency, long length) {
  long delayValue = 1000000/frequency/2; // calculate the delay value between transitions
  //// 1 second's worth of microseconds, divided by the frequency, then split in half since
  //// there are two phases to each cycle
  long numCycles = frequency * length/ 1000; // calculate the number of cycles for proper timing
  //// multiply frequency, which is really cycles per second, by the number of seconds to 
  //// get the total number of cycles to produce
 for (long i=0; i < numCycles; i++){ // for the calculated length of time...
    digitalWrite(targetPin,HIGH); // write the buzzer pin high to push out the diaphram
    delayMicroseconds(delayValue); // wait for the calculated delay value
    digitalWrite(targetPin,LOW); // write the buzzer pin low to pull back the diaphram
    delayMicroseconds(delayValue); // wait againf or the calculated delay value
  }
}

