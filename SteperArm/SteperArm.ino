#include <SPI.h>
#include <Servo.h>

//Pin connected to ST_CP of 74HC595
const int latchPin = 10;
//Pin connected to SH_CP of 74HC595
const int sclrPin = 8;
const int lightPin = 13;
const int penServoPin = 9;
unsigned long interval = 1000000;
unsigned long nextSerialCheck = 0;

const byte stepCount = 8;
byte steps[] = {B00000001, B00000011, B00000010, B00000110, B00000100, B00001100, B00001000, B00001001};
byte aStep = 255;
byte bStep = 255;
long aTarget = 0;
long bTarget = 0;
long aTotal = 0;
long bTotal = 0;
long aInterval = 0;
long bInterval = 0;
unsigned long aTime = 0;
unsigned long bTime = 0;
unsigned long steperMinInterval = 100000;
bool minus = false;
long strData = 0;
Servo penServo;

void setup() {
  SPI.begin();
  Serial.begin(115200);
  
  //set pins to output because they are addressed in the main loop
  pinMode(latchPin, OUTPUT);
  pinMode(sclrPin, OUTPUT);
  pinMode(lightPin, OUTPUT);

  digitalWrite(lightPin, HIGH);
  digitalWrite(sclrPin, LOW);
  delay(100);
  digitalWrite(lightPin, LOW);
  digitalWrite(sclrPin, HIGH);
  Output(0);
  
  nextSerialCheck = micros();
}

void loop() {
  CheckSerial();
  UpdateMotors();
} 

void CheckSerial(){
  unsigned long t = micros();
  long dif = nextSerialCheck - t;
  if (dif >= 0) return;
  nextSerialCheck = t;
  if (Serial.available() > 0){
    int b = Serial.read();
    if (b < 0) return;
    switch (b){
      case 'c':
        penServo.attach(penServoPin);
        Serial.println("Servo attached.");
        break;
      case 'u':
        penServo.detach();
        Serial.println("Servo detached.");
        break;
      case 'd':
        penServo.write(strData);
        Serial.print("Servo set:");
        Serial.println(strData);
        strData = 0;
        minus = false;
        break;
      case 'm':
        penServo.writeMicroseconds(strData);
        Serial.print("Servo set (milis):");
        Serial.println(strData);
        strData = 0;
        minus = false;
        break;
      case 'p':
        if (strData > 0){
          interval = strData;
        }
        nextSerialCheck = t + interval;
        return;
      case 'r':
        strData = 0;
        minus = false;        
        break;
      case '-':
        minus = true;
        break;
      case '0':
      case '1':
      case '2':
      case '3':
      case '4':
      case '5':
      case '6':
      case '7':
      case '8':
      case '9':
        strData = (strData * 10) + (b - '0');
        break;
      case 'A':
      case 'a':
        if (minus) strData = -strData;
        aTarget = strData;
        strData = 0;
        minus = false;
        Serial.print("A target:");
        Serial.println(aTarget);
        break;
      case 'B':
      case 'b':
        if (minus) strData = -strData;
        bTarget = strData;
        strData = 0;
        minus = false;
        Serial.print("B target:");
        Serial.println(bTarget);
        break;
      case 'S':
      case 's':
        steperMinInterval = 1000000 / strData;
        strData = 0;
        minus = false;
        Serial.print("Steper interval:");
        Serial.println(steperMinInterval);
        break;
      case 'O':
      case 'o':
        Output(strData);
        strData = 0;
        minus = false;
        aTarget = 0;
        bTarget = 0;
        aInterval = 0;
        bInterval = 0;
        aStep = 255;
        bStep = 255;
        Serial.println("Motors off:");
        break;
      case 'G':
      case 'g':
        strData = 0;
        minus = false;
        SetMotorData();
        dif = aTime - t;
        if (dif < 0) aTime = t;
        dif = bTime - t;
        if (dif < 0) bTime = t;
        break;
      case 'W':
      case 'w':
        while (IsRunning()){
          UpdateMotors();
        }
        break;
      case 'z':
      case 'Z':
        strData = 0;
        minus = false;
        aTarget = -aTotal;
        bTarget = -bTotal;
        SetMotorData();
        dif = aTime - t;
        if (dif < 0) aTime = t;
        dif = bTime - t;
        if (dif < 0) bTime = t;
        break;
        break;
      default:
        Serial.print("Not supported ");
        Serial.print(b);
        Serial.println(".");
    }    
  }
}

void SetMotorData(){
  long aAbs = abs(aTarget);
  long bAbs = abs(bTarget);
  long maxTarget = max(aAbs, bAbs);
  unsigned long totalTime = maxTarget * steperMinInterval;
  if (aTarget != 0){
    aInterval = totalTime / aAbs;
    if (aTarget<0){
      aInterval = -aInterval;
    }
  }
  else{
    aInterval = 0;
  }
  aTarget = abs(aTarget);
  Serial.print("aInterval:");
  Serial.println(aInterval);
    
  if (bTarget != 0){
    bInterval = totalTime / bAbs;
    if(bTarget<0){
      bInterval = -bInterval;
    }
  }
  else{
    bInterval = 0;
  }
  bTarget = abs(bTarget);
  Serial.print("bInterval:");
  Serial.println(bInterval);  
}

bool IsRunning(){
  return aTarget >= 0 || bTarget >= 0;
}

void UpdateMotors(){
   if (!IsRunning()) return;
   unsigned long t = micros();
   if (aTarget > 0){    
     long dif = aTime - t;
     if (dif < 0){
       aTime += abs(aInterval);
       aTarget--;
       if (aTarget >=0){
         if (aInterval > 0){
           aStep++;
           aTotal++;
           if (aStep >= stepCount){
             aStep = 0; 
           }
         }else{
           aStep--;
           aTotal--;
           if (aStep >= stepCount){
             aStep = stepCount - 1;  
           }
         }
       }
     }
   }
   if (bTarget>=0){
     long dif = bTime - t;
     if (dif < 0){
       bTime += abs(bInterval);
       bTarget--;
       if (bTarget>=0){
         if (bInterval > 0){
           bStep++;
           bTotal++;
           if (bStep >= stepCount){
             bStep = 0; 
           }
         }else{
           bStep--;
           bTotal--;
           if (bStep >= stepCount){
             bStep = stepCount - 1;  
           }
         }
       }
     }
   }
   int data = 0;
   if (aStep!=255){
     data = data | (steps[aStep]<<4);
   }
   if (bStep!=255){
     data = data | steps[bStep];
   }
   Output(data);
   if (!IsRunning()) {
     Serial.println("Done!");
   }
}

void Output(int data){
   digitalWrite(latchPin, LOW);
   SPI.transfer16(data);   
   digitalWrite(latchPin, HIGH);
}

