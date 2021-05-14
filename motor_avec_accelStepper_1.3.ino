#include <DS3231.h>

#include <AccelStepper.h>
#define HALFSTEP 8  //Half-step mode (8 step control signal sequence)

#include <Wire.h>

//horloge
RTCDateTime dt;
boolean isAlarm = false;
boolean alarmState = false;
byte Year ;
byte Month ;
byte Date ;
byte DoW ;
byte Hour ;
byte Minute ;
byte Second ;
bool Century  = false;
bool h12 ;
bool PM ;
//Objects
DS3231 clock;

// Bouton de calibrage
int triggerCalibragePin = 3;

// Motor pin definitions
#define mtrPin1  8     // IN1 on the ULN2003 driver 1
#define mtrPin2  9     // IN2 on the ULN2003 driver 1
#define mtrPin3  10     // IN3 on the ULN2003 driver 1
#define mtrPin4  11     // IN4 on the ULN2003 driver 1
#define speed -1000
#define stepPerRound 10178 // nombre de pas pour 1 tour
#define angleMini -5089 // angle d'une portion de tour
#define nbAngles 4

AccelStepper stepper(HALFSTEP, mtrPin1, mtrPin2, mtrPin3, mtrPin4);

int opticalSwitch;

void calibrate() {
  Serial.println("début calibrage position moteur"); 
  opticalSwitch = analogRead(A0);
  stepper.setSpeed(speed);
  while(opticalSwitch > 110)
  {
    stepper.runSpeed();
    opticalSwitch = analogRead(A0);
  }
  stepper.setCurrentPosition(0);
  Serial.println("fin calibrage position moteur"); 
  digitalWrite(mtrPin1,LOW);
  digitalWrite(mtrPin2,LOW);     
  digitalWrite(mtrPin3,LOW);   
  digitalWrite(mtrPin4,LOW); 
  return;
}

void distribute() {
  stepper.setCurrentPosition(0);
  Serial.println("début versement croquettes"); 
  stepper.setSpeed(speed);
  while(stepper.currentPosition() != angleMini) // positionnement 
  {
    stepper.runSpeed();
  }

  return;
}


void alarmFunction()
{
  Serial.println("*** INT 0 ***");
  isAlarm = true;
}

void setup() {
  pinMode(triggerCalibragePin, INPUT_PULLUP);
  Serial.begin(9600);
  Serial.println("------------------------------------");
  Serial.println("------  Croquette distributor   ----");
  stepper.setMaxSpeed(2000.0);
  stepper.setAcceleration(2000.0);  //Make the acc quick
  stepper.setSpeed(2000);

  Serial.println(F("Initialize System"));
  clock.armAlarm1(false);
  clock.armAlarm2(false);
  clock.clearAlarm1();
  clock.clearAlarm2();
  clock.setAlarm1(0, 0, 0, 10, DS3231_MATCH_S);
  attachInterrupt(0, alarmFunction, FALLING);
  Wire.begin();
}

void loop() {
  setDate();
  readRTC();
  // on regarde si il y a un clibrage
  if(digitalRead(triggerCalibragePin) == LOW){
    calibrate();
  }
  
  if (isAlarm) {
    Serial.println("Alarm detectee");
    isAlarm = false;
    distribute();
    //stop the motor
    digitalWrite(mtrPin1,LOW);
    digitalWrite(mtrPin2,LOW);     
    digitalWrite(mtrPin3,LOW);   
    digitalWrite(mtrPin4,LOW); 
    Serial.print("distibution à : ");Serial.println(clock.dateFormat("d-m-Y H:i:s", dt));
    clock.clearAlarm1();
  } 

  delay(1000); 
}

void readRTC( ) { /* function readRTC */
 ////Read Real Time clock
  dt = clock.getDateTime();
  Serial.print("heure : ");Serial.println(clock.dateFormat("d-m-Y H:i:s", dt));
}


void setDate( ) { /* function setDate */
 ////Set Real Time clock
 if (Serial.available()) {
   //int _start = millis();
   GetDateStuff(Year, Month, Date, DoW, Hour, Minute, Second);
    clock.setDateTime(Year, Month, Date, Hour, Minute, Second);

 }
}
void GetDateStuff(byte& Year, byte& Month, byte& Day, byte& DoW, byte& Hour, byte& Minute, byte& Second) { /* function GetDateStuff */
 ////Get date data
 // Call this if you notice something coming in on
 // the serial port. The stuff coming in should be in
 // the order YYMMDDwHHMMSS, with an 'x' at the end.
 boolean GotString = false;
 char InChar;
 byte Temp1, Temp2;
 char InString[20];
 byte j = 0;
 while (!GotString) {
   if (Serial.available()) {
     InChar = Serial.read();
     InString[j] = InChar;
     j += 1;
     if (InChar == 'x') {
       GotString = true;
     }
   }
 }
 Serial.println(InString);
 // Read Year first
 Temp1 = (byte)InString[0] - 48;
 Temp2 = (byte)InString[1] - 48;
 Year = Temp1 * 10 + Temp2;
 // now month
 Temp1 = (byte)InString[2] - 48;
 Temp2 = (byte)InString[3] - 48;
 Month = Temp1 * 10 + Temp2;
 // now date
 Temp1 = (byte)InString[4] - 48;
 Temp2 = (byte)InString[5] - 48;
 Day = Temp1 * 10 + Temp2;
 // now Day of Week
 DoW = (byte)InString[6] - 48;
 // now Hour
 Temp1 = (byte)InString[7] - 48;
 Temp2 = (byte)InString[8] - 48;
 Hour = Temp1 * 10 + Temp2;
 // now Minute
 Temp1 = (byte)InString[9] - 48;
 Temp2 = (byte)InString[10] - 48;
 Minute = Temp1 * 10 + Temp2;
 // now Second
 Temp1 = (byte)InString[11] - 48;
 Temp2 = (byte)InString[12] - 48;
 Second = Temp1 * 10 + Temp2;
}
