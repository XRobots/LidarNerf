#include <FastLED.h>
#include <Servo.h>
#include <PID_v1.h>  //PID loop from http://playground.arduino.cc/Code/PIDLibrary

#define NUM_LEDS 12    // APA102 LED setup
#define DATA_PIN 38
#define CLOCK_PIN 36

CRGB leds[NUM_LEDS];

double Pk1 = 0.5;  
double Ik1 = 0;
double Dk1 = 0.01;

double Setpoint1, Input1, Output1, Output1a;    // PID variables
PID PID1(&Input1, &Output1, &Setpoint1, Pk1, Ik1 , Dk1, DIRECT);    // PID Setup

double Pk2 = 0.5;  
double Ik2 = 0;
double Dk2 = 0.01;

double Setpoint2, Input2, Output2, Output2a;    // PID variables
PID PID2(&Input2, &Output2, &Setpoint2, Pk2, Ik2 , Dk2, DIRECT);    // PID Setup

Servo servo1;
Servo servo2;

float distance;
float angle;
bool  startBit;
byte  quality;

// variables for each segment - yes I know what arrays are!

int distance1;
int distanceFiltered1;
int distanceBookmarked1;

int distance2;
int distanceFiltered2;
int distanceBookmarked2;

int distance3;
int distanceFiltered3;
int distanceBookmarked3;

int distance4;
int distanceFiltered4;
int distanceBookmarked4;

int distance5;
int distanceFiltered5;
int distanceBookmarked5;

int distance6;
int distanceFiltered6;
int distanceBookmarked6;

int distance7;
int distanceFiltered7;
int distanceBookmarked7;

int distance8;
int distanceFiltered8;
int distanceBookmarked8;

int distance9;
int distanceFiltered9;
int distanceBookmarked9;

int distance10;
int distanceFiltered10;
int distanceBookmarked10;

int distance11;
int distanceFiltered11;
int distanceBookmarked11;

int distance12;
int distanceFiltered12;
int distanceBookmarked12;

int distance13;
int distanceFiltered13;
int distanceBookmarked13;

int distance14;
int distanceFiltered14;
int distanceBookmarked14;

int distance15;
int distanceFiltered15;
int distanceBookmarked15;

int distance16;
int distanceFiltered16;
int distanceBookmarked16;

int distance17;
int distanceFiltered17;
int distanceBookmarked17;

int distance18;
int distanceFiltered18;
int distanceBookmarked18;

int distance19;
int distanceFiltered19;
int distanceBookmarked19;

int distance20;
int distanceFiltered20;
int distanceBookmarked20;

int distance21;
int distanceFiltered21;
int distanceBookmarked21;

int distance22;
int distanceFiltered22;
int distanceBookmarked22;

int distance23;
int distanceFiltered23;
int distanceBookmarked23;

int distance24;
int distanceFiltered24;
int distanceBookmarked24;

int distance25;
int distanceFiltered25;
int distanceBookmarked25;

int distance26;
int distanceFiltered26;
int distanceBookmarked26;

int distance27;
int distanceFiltered27;
int distanceBookmarked27;

int distance28;
int distanceFiltered28;
int distanceBookmarked28;

int distance29;
int distanceFiltered29;
int distanceBookmarked29;

int distance30;
int distanceFiltered30;
int distanceBookmarked30;

int distance31;
int distanceFiltered31;
int distanceBookmarked31;

int distance32;
int distanceFiltered32;
int distanceBookmarked32;

int distance33;
int distanceFiltered33;
int distanceBookmarked33;

int distance34;
int distanceFiltered34;
int distanceBookmarked34;

int distance35;
int distanceFiltered35;
int distanceBookmarked35;

int distance36;
int distanceFiltered36;
int distanceBookmarked36;

// constraints and other variables

int minDistance = 150;
int maxDistance = 3000;
int threshold = 950;

int button;
int flag = 0;
int segmentDetected = 0;
int bulletCount = 7;

// timer variables

unsigned long currentMillis;
unsigned long previousArmMillis;
unsigned long previousMillis;

// wheel encoder interrupts

#define encoder0PinA 2      // encoder 1
#define encoder0PinB 3

#define encoder1PinA 18     // encoder 2
#define encoder1PinB 19

volatile long encoder0Pos = 0;    // encoder 1
volatile long encoder1Pos = 0;    // encoder 2

float demandRot;
float demandRotFiltered;

// This sketch code is based on the RPLIDAR driver library provided by RoboPeak
#include <RPLidar.h>

// You need to create an driver instance 
RPLidar lidar;

#define RPLIDAR_MOTOR 12 // The PWM pin for control the speed of RPLIDAR's motor.
                       
                        
void setup() {

  FastLED.addLeds<APA102, DATA_PIN, CLOCK_PIN, BGR>(leds, NUM_LEDS);    // LED setup

  setLEDs(0,0,100);  

  servo1.attach(53);    // servo pins
  servo2.attach(51);

  servo1.write(180);
  servo2.write(180);

  pinMode(4, OUTPUT);     // motor PWM pins
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);

  pinMode(encoder0PinA, INPUT_PULLUP);    // encoder pins
  pinMode(encoder0PinB, INPUT_PULLUP);

  pinMode(encoder1PinA, INPUT_PULLUP); 
  pinMode(encoder1PinB, INPUT_PULLUP);

  attachInterrupt(0, doEncoderA, CHANGE);
  attachInterrupt(1, doEncoderB, CHANGE); 

  attachInterrupt(4, doEncoderC, CHANGE);
  attachInterrupt(5, doEncoderD, CHANGE); 

  pinMode(41, OUTPUT);        // relay for blasters
  pinMode(49, INPUT_PULLUP);  // switch

  digitalWrite(41, HIGH);     // motors off to start with
  
  // bind the RPLIDAR driver to the arduino hardware serial
  lidar.begin(Serial2);
  Serial.begin(115200);
  
  // set pin modes
  pinMode(RPLIDAR_MOTOR, OUTPUT);

  PID1.SetMode(AUTOMATIC);              
  PID1.SetOutputLimits(-200, 200);  
  PID1.SetSampleTime(10);

  PID2.SetMode(AUTOMATIC);              
  PID2.SetOutputLimits(-200, 200);  
  PID2.SetSampleTime(10);
}

void loop() {

    currentMillis = millis();   // bookmark the time   
     
    if (IS_OK(lidar.waitPoint())) {
      distance = lidar.getCurrentPoint().distance; //distance value in mm unit
      angle    = lidar.getCurrentPoint().angle; //anglue value in degree
      startBit = lidar.getCurrentPoint().startBit; //whether this point is belong to a new scan
      quality  = lidar.getCurrentPoint().quality; //quality of the current measurement
      
      //perform data processing here... 
  
      if (angle >= 355 || angle <= 5 && quality >= 15) { 
        distance1 = constrain (distance, minDistance,maxDistance); 
        distanceFiltered1 = (0.8 * distanceFiltered1) + (0.2 * distance1);     
      }  
      if (angle > 5 && angle <= 15 && quality >= 15) {
        distance2 = constrain (distance, minDistance,maxDistance);  
        distanceFiltered2 = (0.8 * distanceFiltered2) + (0.2 * distance2);
      }   
      if (angle > 15 && angle <= 25 && quality >= 15) {
        distance3 = constrain (distance, minDistance,maxDistance);  
        distanceFiltered3 = (0.8 * distanceFiltered3) + (0.2 * distance3);
      }
      if (angle > 25 && angle <= 35 && quality >= 15) {
        distance4 = constrain (distance, minDistance,maxDistance);  
        distanceFiltered4 = (0.8 * distanceFiltered4) + (0.2 * distance4);
      }
      if (angle > 35 && angle <= 45 && quality >= 15) {
        distance5 = constrain (distance, minDistance,maxDistance);  
        distanceFiltered5 = (0.8 * distanceFiltered5) + (0.2 * distance5);
      }
      if (angle > 45 && angle <= 55 && quality >= 15) {
        distance6 = constrain (distance, minDistance,maxDistance);  
        distanceFiltered6 = (0.8 * distanceFiltered6) + (0.2 * distance6);
      }
      if (angle > 55 && angle <= 65 && quality >= 15) {
        distance7 = constrain (distance, minDistance,maxDistance);  
        distanceFiltered7 = (0.8 * distanceFiltered7) + (0.2 * distance7);
      }
      if (angle > 65 && angle <= 75 && quality >= 15) {
        distance8 = constrain (distance, minDistance,maxDistance);  
        distanceFiltered8 = (0.8 * distanceFiltered8) + (0.2 * distance8);
      }
      if (angle > 75 && angle <= 85 && quality >= 15) {
        distance9 = constrain (distance, minDistance,maxDistance);  
        distanceFiltered9 = (0.8 * distanceFiltered9) + (0.2 * distance9);
      }
      if (angle > 85 && angle <= 95 && quality >= 15) {
        distance10 = constrain (distance, minDistance,maxDistance);  
        distanceFiltered10 = (0.8 * distanceFiltered10) + (0.2 * distance10);
      }
      if (angle > 95 && angle <= 105 && quality >= 15) {
        distance11 = constrain (distance, minDistance,maxDistance);  
        distanceFiltered11 = (0.8 * distanceFiltered11) + (0.2 * distance11);
      }
      if (angle > 105 && angle <= 115 && quality >= 15) {
        distance12 = constrain (distance, minDistance,maxDistance);  
        distanceFiltered12 = (0.8 * distanceFiltered12) + (0.2 * distance12);
      }
      if (angle > 115 && angle <= 125 && quality >= 15) {
        distance13 = constrain (distance, minDistance,maxDistance);  
        distanceFiltered13 = (0.8 * distanceFiltered13) + (0.2 * distance13);
      }
      if (angle > 125 && angle <= 135 && quality >= 15) {
        distance14 = constrain (distance, minDistance,maxDistance);  
        distanceFiltered14 = (0.8 * distanceFiltered14) + (0.2 * distance14);
      }
      if (angle > 135 && angle <= 145 && quality >= 15) {
        distance15 = constrain (distance, minDistance,maxDistance);  
        distanceFiltered15 = (0.8 * distanceFiltered15) + (0.2 * distance15);
      }
      if (angle > 145 && angle <= 155 && quality >= 15) {
        distance16 = constrain (distance, minDistance,maxDistance);  
        distanceFiltered16 = (0.8 * distanceFiltered16) + (0.2 * distance16);
      }
      if (angle > 155 && angle <= 165 && quality >= 15) {
        distance17 = constrain (distance, minDistance,maxDistance);  
        distanceFiltered17 = (0.8 * distanceFiltered17) + (0.2 * distance17);
      }
      if (angle > 165 && angle <= 175 && quality >= 15) {
        distance18 = constrain (distance, minDistance,maxDistance);  
        distanceFiltered18 = (0.8 * distanceFiltered18) + (0.2 * distance18);
      }
      if (angle > 175 && angle <= 185 && quality >= 15) {
        distance19 = constrain (distance, minDistance,maxDistance);  
        distanceFiltered19 = (0.8 * distanceFiltered19) + (0.2 * distance19);
      }
      if (angle > 185 && angle <= 195 && quality >= 15) {
        distance20 = constrain (distance, minDistance,maxDistance);  
        distanceFiltered20 = (0.8 * distanceFiltered20) + (0.2 * distance20);
      }
      if (angle > 195 && angle <=205 && quality >= 15) {
        distance21 = constrain (distance, minDistance,maxDistance);  
        distanceFiltered21 = (0.8 * distanceFiltered21) + (0.2 * distance21);
      }
      if (angle > 205 && angle <=215 && quality >= 15) {
        distance22 = constrain (distance, minDistance,maxDistance);  
        distanceFiltered22 = (0.8 * distanceFiltered22) + (0.2 * distance22);
      }
      if (angle > 215 && angle <=225 && quality >= 15) {
        distance23 = constrain (distance, minDistance,maxDistance);  
        distanceFiltered23 = (0.8 * distanceFiltered23) + (0.2 * distance23);
      }
      if (angle > 225 && angle <=235 && quality >= 15) {
        distance24 = constrain (distance, minDistance,maxDistance);  
        distanceFiltered24 = (0.8 * distanceFiltered24) + (0.2 * distance24);
      }
      if (angle > 235 && angle <=245 && quality >= 15) {
        distance25 = constrain (distance, minDistance,maxDistance);  
        distanceFiltered25 = (0.8 * distanceFiltered25) + (0.2 * distance25);
      }
      if (angle > 245 && angle <=255 && quality >= 15) {
        distance26 = constrain (distance, minDistance,maxDistance);  
        distanceFiltered26 = (0.8 * distanceFiltered26) + (0.2 * distance26);
      }
      if (angle > 255 && angle <=265 && quality >= 15) {
        distance27 = constrain (distance, minDistance,maxDistance);  
        distanceFiltered27 = (0.8 * distanceFiltered27) + (0.2 * distance27);
      }
      if (angle > 265 && angle <=275 && quality >= 15) {
        distance28 = constrain (distance, minDistance,maxDistance);  
        distanceFiltered28 = (0.8 * distanceFiltered28) + (0.2 * distance28);
      }
      if (angle > 275 && angle <=285 && quality >= 15) {
        distance29 = constrain (distance, minDistance,maxDistance);  
        distanceFiltered29 = (0.8 * distanceFiltered29) + (0.2 * distance29);
      }
      if (angle > 285 && angle <=295 && quality >= 15) {
        distance30 = constrain (distance, minDistance,maxDistance);  
        distanceFiltered30 = (0.8 * distanceFiltered30) + (0.2 * distance30);
      }
      if (angle > 295 && angle <=305 && quality >= 15) {
        distance31 = constrain (distance, minDistance,maxDistance);  
        distanceFiltered31 = (0.8 * distanceFiltered31) + (0.2 * distance31);
      }
      if (angle > 305 && angle <=315 && quality >= 15) {
        distance32 = constrain (distance, minDistance,maxDistance);  
        distanceFiltered32 = (0.8 * distanceFiltered32) + (0.2 * distance32);
      }
      if (angle > 315 && angle <=325 && quality >= 15) {
        distance33 = constrain (distance, minDistance,maxDistance);  
        distanceFiltered33 = (0.8 * distanceFiltered33) + (0.2 * distance33);
      }
      if (angle > 325 && angle <=335 && quality >= 15) {
        distance34 = constrain (distance, minDistance,maxDistance);  
        distanceFiltered34 = (0.8 * distanceFiltered34) + (0.2 * distance34);
      }
      if (angle > 335 && angle <=345 && quality >= 15) {
        distance35 = constrain (distance, minDistance,maxDistance);  
        distanceFiltered35 = (0.8 * distanceFiltered35) + (0.2 * distance35);
      }
      if (angle > 345 && angle <=355 && quality >= 15) {
        distance36 = constrain (distance, minDistance,maxDistance);  
        distanceFiltered36 = (0.8 * distanceFiltered36) + (0.2 * distance36);
      }

    
    }   // end of lidar is good & processesing

    
    //  *** Rest of Lidar config ****
     
    else {
      analogWrite(RPLIDAR_MOTOR, 0); //stop the rplidar motor
      
      // try to detect RPLIDAR... 
      rplidar_response_device_info_t info;
      if (IS_OK(lidar.getDeviceInfo(info, 100))) {
         // detected...
         lidar.startScan();
         
         // start motor rotating at max allowed speed
         analogWrite(RPLIDAR_MOTOR, 255);
         delay(1000);
      }
    }

     if (currentMillis - previousMillis >= 10) {  // start timed event for motor driving only - PID works better if we slow the loop down
        previousMillis = currentMillis;
        
         if (Serial.available()>0) {       // manual control of wheesls via terminal
            char c = Serial.read();
  
            if (c == 'a') {            
              demandRot = 6300;
            }
            else if (c == 'b') {      
              demandRot = -6300;
            }
            else if (c == 'c') {       
              demandRot = 0;
            }

        }

        //Serial.println(distance1);

        demandRotFiltered = filter(demandRot, demandRotFiltered, 0);    // filter demand position so it moves nicely

        Setpoint1 = demandRotFiltered;
        Input1 = encoder0Pos;
        PID1.Compute();

        Setpoint2 = demandRotFiltered*-1;
        Input2 = encoder1Pos;
        PID2.Compute();
       
        // drive motor

        if (Output1 > 0) {
          Output1a = abs(Output1);
          analogWrite(6, Output1a);
          analogWrite(7, 0);
        }
        else if (Output1 < 0) {
          Output1a = abs(Output1);
          analogWrite(7, Output1a);
          analogWrite(6, 0);
        }
        else {
          analogWrite(7,0);
          analogWrite(6, 0);
        }

        // other motor

        if (Output2 > 0) {
          Output2a = abs(Output2);
          analogWrite(5, Output2a);
          analogWrite(4, 0);
        }
        else if (Output2 < 0) {
          Output2a = abs(Output2);
          analogWrite(4, Output2a);
          analogWrite(5, 0);
        }
        else {
          analogWrite(4,0);
          analogWrite(5, 0);
        }
    
    


        // *** button & fire processing ***
    
        button = digitalRead(49);     // look for arm switch
        if (button == 0) {
            setLEDs(50,50,0);         // set the LEDs to amber
            previousArmMillis = currentMillis;
            flag = 1;
        } 
    
        if (bulletCount <= 1 && flag == 1) {
            Serial.println("Bullets are out");
            setLEDs(0,100,0);     // set the LEDs to green
            bulletCount = 7;    // reset counts for thenext go
            flag = 0;
        }
      
        else if (currentMillis - previousArmMillis > 4000 && flag == 1) {
    
          bulletCount = bulletCount - 1;
          Serial.print("Bullets: ");
          Serial.println(bulletCount);
          
          // ** yes I understand what for loops are also, thanks! **
          
          distanceBookmarked1 = distanceFiltered1;
          distanceBookmarked2 = distanceFiltered2;
          distanceBookmarked3 = distanceFiltered3;
          distanceBookmarked4 = distanceFiltered4;
          distanceBookmarked5 = distanceFiltered5;
          distanceBookmarked6 = distanceFiltered6;
          distanceBookmarked7 = distanceFiltered7;
          distanceBookmarked8 = distanceFiltered8;
          distanceBookmarked9 = distanceFiltered9;
          distanceBookmarked10 = distanceFiltered10;
          distanceBookmarked11 = distanceFiltered11;
          distanceBookmarked12 = distanceFiltered12;
          distanceBookmarked13 = distanceFiltered13;
          distanceBookmarked14 = distanceFiltered14;
          distanceBookmarked15 = distanceFiltered15;
          distanceBookmarked16 = distanceFiltered16;
          distanceBookmarked17 = distanceFiltered17;
          distanceBookmarked18 = distanceFiltered18;
          distanceBookmarked19 = distanceFiltered19;
          distanceBookmarked20 = distanceFiltered20;
          distanceBookmarked21 = distanceFiltered21;
          distanceBookmarked22 = distanceFiltered22;
          distanceBookmarked23 = distanceFiltered23;
          distanceBookmarked24 = distanceFiltered24;
          distanceBookmarked25 = distanceFiltered25;
          distanceBookmarked26 = distanceFiltered26;
          distanceBookmarked27 = distanceFiltered27;
          distanceBookmarked28 = distanceFiltered28;
          distanceBookmarked29 = distanceFiltered29;
          distanceBookmarked30 = distanceFiltered30;
          distanceBookmarked31 = distanceFiltered31;
          distanceBookmarked32 = distanceFiltered32;
          distanceBookmarked33 = distanceFiltered33;
          distanceBookmarked34 = distanceFiltered34;
          distanceBookmarked35 = distanceFiltered35;
          distanceBookmarked36 = distanceFiltered36;
          Serial.print("READY, Bookmarked 1: ");
          Serial.println(distanceBookmarked1);
          Serial.print("READY, Bookmarked 2: ");
          Serial.println(distanceBookmarked2);
          Serial.print("READY, Bookmarked 3: ");
          Serial.println(distanceBookmarked2);
          Serial.print("READY, Bookmarked 4: ");
          Serial.println(distanceBookmarked4);
          Serial.print("READY, Bookmarked 5: ");
          Serial.println(distanceBookmarked5);
          Serial.print("READY, Bookmarked 6: ");
          Serial.println(distanceBookmarked6);
          Serial.print("READY, Bookmarked 7: ");
          Serial.println(distanceBookmarked7);
          Serial.print("READY, Bookmarked 8: ");
          Serial.println(distanceBookmarked8);
          Serial.print("READY, Bookmarked 9: ");
          Serial.println(distanceBookmarked9);
          Serial.print("READY, Bookmarked 10: ");
          Serial.println(distanceBookmarked10);
          Serial.print("READY, Bookmarked 11: ");
          Serial.println(distanceBookmarked11);
          Serial.print("READY, Bookmarked 12: ");
          Serial.println(distanceBookmarked12);
          Serial.print("READY, Bookmarked 13: ");
          Serial.println(distanceBookmarked13);
          Serial.print("READY, Bookmarked 14: ");
          Serial.println(distanceBookmarked14);
          Serial.print("READY, Bookmarked 15: ");
          Serial.println(distanceBookmarked15);
          Serial.print("READY, Bookmarked 16: ");
          Serial.println(distanceBookmarked16);
          Serial.print("READY, Bookmarked 17: ");
          Serial.println(distanceBookmarked17);
          Serial.print("READY, Bookmarked 18: ");
          Serial.println(distanceBookmarked18);
          Serial.print("READY, Bookmarked 19: ");
          Serial.println(distanceBookmarked19);
          Serial.print("READY, Bookmarked 20: ");
          Serial.println(distanceBookmarked20);
          Serial.print("READY, Bookmarked 21: ");
          Serial.println(distanceBookmarked21);
          Serial.print("READY, Bookmarked 22: ");
          Serial.println(distanceBookmarked22);
          Serial.print("READY, Bookmarked 23: ");
          Serial.println(distanceBookmarked23);
          Serial.print("READY, Bookmarked 24: ");
          Serial.println(distanceBookmarked24);
          Serial.print("READY, Bookmarked 25: ");
          Serial.println(distanceBookmarked25);
          Serial.print("READY, Bookmarked 26: ");
          Serial.println(distanceBookmarked26);
          Serial.print("READY, Bookmarked 27: ");
          Serial.println(distanceBookmarked27);
          Serial.print("READY, Bookmarked 28: ");
          Serial.println(distanceBookmarked28);
          Serial.print("READY, Bookmarked 29: ");
          Serial.println(distanceBookmarked29);
          Serial.print("READY, Bookmarked 30: ");
          Serial.println(distanceBookmarked30);
          Serial.print("READY, Bookmarked 30: ");
          Serial.println(distanceBookmarked30);
          Serial.print("READY, Bookmarked 31: ");
          Serial.println(distanceBookmarked31);
          Serial.print("READY, Bookmarked 32: ");
          Serial.println(distanceBookmarked32);
          Serial.print("READY, Bookmarked 33: ");
          Serial.println(distanceBookmarked33);
          Serial.print("READY, Bookmarked 34: ");
          Serial.println(distanceBookmarked34);
          Serial.print("READY, Bookmarked 35: ");
          Serial.println(distanceBookmarked35);
          Serial.print("READY, Bookmarked 36: ");
          Serial.println(distanceBookmarked36);
    
          setLEDs(100,0,0);     // set the LEDs to red
          
          previousArmMillis = currentMillis;
          flag = 2;
        }
    
        else if (flag == 2) {
          if (distanceFiltered1 <= distanceBookmarked1 - threshold) {
            segmentDetected = 1;
            Serial.println("TARGET DETECTED 1");
            Serial.println(distanceFiltered1);
            Serial.println("MOTORS ON");
            previousArmMillis = currentMillis;
            flag = 3;
          }
          else if (distanceFiltered2 <= distanceBookmarked2 - threshold) {
            segmentDetected = 2;
            Serial.println("TARGET DETECTED 2");
            Serial.println(distanceFiltered2);
            Serial.println("MOTORS ON");
            previousArmMillis = currentMillis;
            flag = 3;
          }
          else if (distanceFiltered3 <= distanceBookmarked3 - threshold) {
            segmentDetected = 3;
            Serial.println("TARGET DETECTED 3");
            Serial.println(distanceFiltered3);
            Serial.println("MOTORS ON");
            previousArmMillis = currentMillis;
            flag = 3;
          }
          else if (distanceFiltered4 <= distanceBookmarked4 - threshold) {
            segmentDetected = 4;
            Serial.println("TARGET DETECTED 4");
            Serial.println(distanceFiltered4);
            Serial.println("MOTORS ON");
            previousArmMillis = currentMillis;
            flag = 3;
          }
          else if (distanceFiltered5 <= distanceBookmarked5 - threshold) {
            segmentDetected = 5;
            Serial.println("TARGET DETECTED 5");
            Serial.println(distanceFiltered5);
            Serial.println("MOTORS ON");
            previousArmMillis = currentMillis;
            flag = 3;
          }
          else if (distanceFiltered6 <= distanceBookmarked6 - threshold) {
            segmentDetected = 6;
            Serial.println("TARGET DETECTED 6");
            Serial.println(distanceFiltered6);
            Serial.println("MOTORS ON");
            previousArmMillis = currentMillis;
            flag = 3;
          }
          else if (distanceFiltered7 <= distanceBookmarked7 - threshold) {
            segmentDetected = 7;
            Serial.println("TARGET DETECTED 7");
            Serial.println(distanceFiltered7);
            Serial.println("MOTORS ON");
            previousArmMillis = currentMillis;
            flag = 3;
          }
          else if (distanceFiltered8 <= distanceBookmarked8 - threshold) {
            segmentDetected = 8;
            Serial.println("TARGET DETECTED 8");
            Serial.println(distanceFiltered8);
            Serial.println("MOTORS ON");
            previousArmMillis = currentMillis;
            flag = 3;
          }
          else if (distanceFiltered9 <= distanceBookmarked9 - threshold) {
            segmentDetected = 9;
            Serial.println("TARGET DETECTED 9");
            Serial.println(distanceFiltered9);
            Serial.println("MOTORS ON");
            previousArmMillis = currentMillis;
            flag = 3;
          }
          else if (distanceFiltered10 <= distanceBookmarked10 - threshold) {
            segmentDetected = 10;
            Serial.println("TARGET DETECTED 10");
            Serial.println(distanceFiltered10);
            Serial.println("MOTORS ON");
            previousArmMillis = currentMillis;
            flag = 3;
          }
          else if (distanceFiltered11 <= distanceBookmarked11 - threshold) {
            segmentDetected = 11;
            Serial.println("TARGET DETECTED 11");
            Serial.println(distanceFiltered11);
            Serial.println("MOTORS ON");
            previousArmMillis = currentMillis;
            flag = 3;
          }
          else if (distanceFiltered12 <= distanceBookmarked12 - threshold) {
            segmentDetected = 12;
            Serial.println("TARGET DETECTED 12");
            Serial.println(distanceFiltered12);
            Serial.println("MOTORS ON");
            previousArmMillis = currentMillis;
            flag = 3;
          }
          else if (distanceFiltered13 <= distanceBookmarked13 - threshold) {
            segmentDetected = 13;
            Serial.println("TARGET DETECTED 13");
            Serial.println(distanceFiltered13);
            Serial.println("MOTORS ON");
            previousArmMillis = currentMillis;
            flag = 3;
          }
          else if (distanceFiltered14 <= distanceBookmarked14 - threshold) {
            segmentDetected = 14;
            Serial.println("TARGET DETECTED 14");
            Serial.println(distanceFiltered14);
            Serial.println("MOTORS ON");
            previousArmMillis = currentMillis;
            flag = 3;
          }
          else if (distanceFiltered15 <= distanceBookmarked15 - threshold) {
            segmentDetected = 15;
            Serial.println("TARGET DETECTED 15");
            Serial.println(distanceFiltered15);
            Serial.println("MOTORS ON");
            previousArmMillis = currentMillis;
            flag = 3;
          }
          else if (distanceFiltered16 <= distanceBookmarked16 - threshold) {
            segmentDetected = 16;
            Serial.println("TARGET DETECTED 16");
            Serial.println(distanceFiltered16);
            Serial.println("MOTORS ON");
            previousArmMillis = currentMillis;
            flag = 3;
          }
          else if (distanceFiltered17 <= distanceBookmarked17 - threshold) {
            segmentDetected = 17;
            Serial.println("TARGET DETECTED 17");
            Serial.println(distanceFiltered17);
            Serial.println("MOTORS ON");
            previousArmMillis = currentMillis;
            flag = 3;
          }
          else if (distanceFiltered18 <= distanceBookmarked18 - threshold) {
            segmentDetected = 18;
            Serial.println("TARGET DETECTED 18");
            Serial.println(distanceFiltered18);
            Serial.println("MOTORS ON");
            previousArmMillis = currentMillis;
            flag = 3;
          }
          else if (distanceFiltered19 <= distanceBookmarked19 - threshold) {
            segmentDetected = 19;
            Serial.println("TARGET DETECTED 19");
            Serial.println(distanceFiltered19);
            Serial.println("MOTORS ON");
            previousArmMillis = currentMillis;
            flag = 3;
          }
          else if (distanceFiltered20 <= distanceBookmarked20 - threshold) {
            segmentDetected = 20;
            Serial.println("TARGET DETECTED 20");
            Serial.println(distanceFiltered20);
            Serial.println("MOTORS ON");
            previousArmMillis = currentMillis;
            flag = 3;
          }
          else if (distanceFiltered21 <= distanceBookmarked21 - threshold) {
            segmentDetected = 21;
            Serial.println("TARGET DETECTED 21");
            Serial.println(distanceFiltered21);
            Serial.println("MOTORS ON");
            previousArmMillis = currentMillis;
            flag = 3;
          }
          else if (distanceFiltered22 <= distanceBookmarked22 - threshold) {
            segmentDetected = 22;
            Serial.println("TARGET DETECTED 22");
            Serial.println(distanceFiltered22);
            Serial.println("MOTORS ON");
            previousArmMillis = currentMillis;
            flag = 3;
          }
          else if (distanceFiltered23 <= distanceBookmarked23 - threshold) {
            segmentDetected = 23;
            Serial.println("TARGET DETECTED 23");
            Serial.println(distanceFiltered23);
            Serial.println("MOTORS ON");
            previousArmMillis = currentMillis;
            flag = 3;
          }
          else if (distanceFiltered24 <= distanceBookmarked24 - threshold) {
            segmentDetected = 24;
            Serial.println("TARGET DETECTED 24");
            Serial.println(distanceFiltered24);
            Serial.println("MOTORS ON");
            previousArmMillis = currentMillis;
            flag = 3;
          }
          else if (distanceFiltered25 <= distanceBookmarked25 - threshold) {
            segmentDetected = 25;
            Serial.println("TARGET DETECTED 25");
            Serial.println(distanceFiltered25);
            Serial.println("MOTORS ON");
            previousArmMillis = currentMillis;
            flag = 3;
          }
          else if (distanceFiltered26 <= distanceBookmarked26 - threshold) {
            segmentDetected = 26;
            Serial.println("TARGET DETECTED 26");
            Serial.println(distanceFiltered26);
            Serial.println("MOTORS ON");
            previousArmMillis = currentMillis;
            flag = 3;
          }
          else if (distanceFiltered27 <= distanceBookmarked27 - threshold) {
            segmentDetected = 27;
            Serial.println("TARGET DETECTED 27");
            Serial.println(distanceFiltered27);
            Serial.println("MOTORS ON");
            previousArmMillis = currentMillis;
            flag = 3;
          }
          else if (distanceFiltered28 <= distanceBookmarked28 - threshold) {
            segmentDetected = 28;
            Serial.println("TARGET DETECTED 28");
            Serial.println(distanceFiltered28);
            Serial.println("MOTORS ON");
            previousArmMillis = currentMillis;
            flag = 3;
          }
          else if (distanceFiltered29 <= distanceBookmarked29 - threshold) {
            segmentDetected = 29;
            Serial.println("TARGET DETECTED 29");
            Serial.println(distanceFiltered29);
            Serial.println("MOTORS ON");
            previousArmMillis = currentMillis;
            flag = 3;
          }
          else if (distanceFiltered30 <= distanceBookmarked30 - threshold) {
            segmentDetected = 30;
            Serial.println("TARGET DETECTED 30");
            Serial.println(distanceFiltered30);
            Serial.println("MOTORS ON");
            previousArmMillis = currentMillis;
            flag = 3;
          }
          else if (distanceFiltered31 <= distanceBookmarked31 - threshold) {
            segmentDetected = 31;
            Serial.println("TARGET DETECTED 31");
            Serial.println(distanceFiltered31);
            Serial.println("MOTORS ON");
            previousArmMillis = currentMillis;
            flag = 3;
          }
          else if (distanceFiltered32 <= distanceBookmarked32 - threshold) {
            segmentDetected = 32;
            Serial.println("TARGET DETECTED 32");
            Serial.println(distanceFiltered32);
            Serial.println("MOTORS ON");
            previousArmMillis = currentMillis;
            flag = 3;
          }
          else if (distanceFiltered33 <= distanceBookmarked33 - threshold) {
            segmentDetected = 33;
            Serial.println("TARGET DETECTED 33");
            Serial.println(distanceFiltered33);
            Serial.println("MOTORS ON");
            previousArmMillis = currentMillis;
            flag = 3;
          }
          else if (distanceFiltered34 <= distanceBookmarked34 - threshold) {
            segmentDetected = 34;
            Serial.println("TARGET DETECTED 34");
            Serial.println(distanceFiltered34);
            Serial.println("MOTORS ON");
            previousArmMillis = currentMillis;
            flag = 3;
          }
          else if (distanceFiltered35 <= distanceBookmarked35 - threshold) {
            segmentDetected = 35;
            Serial.println("TARGET DETECTED 35");
            Serial.println(distanceFiltered35);
            Serial.println("MOTORS ON");
            previousArmMillis = currentMillis;
            flag = 3;
          }
          else if (distanceFiltered36 <= distanceBookmarked36 - threshold) {
            segmentDetected = 36;
            Serial.println("TARGET DETECTED 36");
            Serial.println(distanceFiltered36);
            Serial.println("MOTORS ON");
            previousArmMillis = currentMillis;
            flag = 3;
          }     
         
        } // end of flag 2
    
        else if (flag == 3) {
            digitalWrite(41, LOW);    // turn on blasters
            if (segmentDetected < 18) {   // turn CW
                demandRot = (6300/18)*segmentDetected;
            }
            else if (segmentDetected > 18) {  // turn CCW
                demandRot = (6300/18)*(36-segmentDetected)*-1;
            }           
            flag = 4;
        }
    
        else if (currentMillis - previousArmMillis > 2000 && flag == 4) {
            encoder0Pos = 0;
            encoder1Pos = 1;
            demandRot = 0;
            Serial.println("FIRE 1");
            servo1.write(0);
            previousArmMillis = currentMillis;
            flag = 5;  
        }
    
        else if (currentMillis - previousArmMillis > 700 && flag == 5) {
            Serial.println("FIRE 2");
            servo1.write(180);
            servo2.write(0); 
            previousArmMillis = currentMillis;
            flag = 6;      
        }
    
        else if (currentMillis - previousArmMillis > 700 && flag == 6) {
            servo1.write(180);
            servo2.write(180); 
            previousArmMillis = currentMillis;
            flag = 7;      
        }
    
        else if (currentMillis - previousArmMillis > 500 && flag == 7) {
            Serial.println("MOTORS OFF");
            digitalWrite(41, HIGH);
            previousArmMillis = currentMillis;
            flag = 1;
            setLEDs(50,50,0);     // set the LEDs back to amber          
        } 

   }   // end of timed events for motor PID only.  

   
} // end of main loop




// ************** encoders interrupts **************

// ************** encoder 1 *********************


void doEncoderA(){  

  // look for a low-to-high on channel A
  if (digitalRead(encoder0PinA) == HIGH) { 
    // check channel B to see which way encoder is turning
    if (digitalRead(encoder0PinB) == LOW) {  
      encoder0Pos = encoder0Pos + 1;         // CW
    } 
    else {
      encoder0Pos = encoder0Pos - 1;         // CCW
    }
  }
  else   // must be a high-to-low edge on channel A                                       
  { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(encoder0PinB) == HIGH) {   
      encoder0Pos = encoder0Pos + 1;          // CW
    } 
    else {
      encoder0Pos = encoder0Pos - 1;          // CCW
    }
  }
 
}

void doEncoderB(){  

  // look for a low-to-high on channel B
  if (digitalRead(encoder0PinB) == HIGH) {   
   // check channel A to see which way encoder is turning
    if (digitalRead(encoder0PinA) == HIGH) {  
      encoder0Pos = encoder0Pos + 1;         // CW
    } 
    else {
      encoder0Pos = encoder0Pos - 1;         // CCW
    }
  }
  // Look for a high-to-low on channel B
  else { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(encoder0PinA) == LOW) {   
      encoder0Pos = encoder0Pos + 1;          // CW
    } 
    else {
      encoder0Pos = encoder0Pos - 1;          // CCW
    }
  }
  

}

// ************** encoder 2 *********************

void doEncoderC(){  

  // look for a low-to-high on channel A
  if (digitalRead(encoder1PinA) == HIGH) { 
    // check channel B to see which way encoder is turning
    if (digitalRead(encoder1PinB) == LOW) {  
      encoder1Pos = encoder1Pos - 1;         // CW
    } 
    else {
      encoder1Pos = encoder1Pos + 1;         // CCW
    }
  }
  else   // must be a high-to-low edge on channel A                                       
  { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(encoder1PinB) == HIGH) {   
      encoder1Pos = encoder1Pos - 1;          // CW
    } 
    else {
      encoder1Pos = encoder1Pos + 1;          // CCW
    }
  }
 
}

void doEncoderD(){  

  // look for a low-to-high on channel B
  if (digitalRead(encoder1PinB) == HIGH) {   
   // check channel A to see which way encoder is turning
    if (digitalRead(encoder1PinA) == HIGH) {  
      encoder1Pos = encoder1Pos - 1;         // CW
    } 
    else {
      encoder1Pos = encoder1Pos + 1;         // CCW
    }
  }
  // Look for a high-to-low on channel B
  else { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(encoder1PinA) == LOW) {   
      encoder1Pos = encoder1Pos - 1;          // CW
    } 
    else {
      encoder1Pos = encoder1Pos + 1;          // CCW
    }
  }
  

}

// filter funcition
float filter(float prevValue, float currentValue, int filter) {  
  float lengthFiltered =  (prevValue + (currentValue * filter)) / (filter + 1);  
  return lengthFiltered;  
}

void setLEDs(int red, int green, int blue) {
    for (int i = 0; i <= 11; i++) {
        leds[i] = CRGB(red, green, blue);
    }
    FastLED.show();
}

