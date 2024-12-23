#include <DS3231.h> //https://github.com/jarzebski
#include <LiquidCrystal.h>
#include <AccelStepper.h>
#define HALFSTEP 8  //Half-step mode (8 step control signal sequence)

#include <Wire.h>
// affichage
LiquidCrystal lcd(13, 12, 7, 6, 5, 4);
//// affichage caratère spéciaux
byte degre[8] = {
  0b00000,
  0b01000,
  0b00100,
  0b11110,
  0b11111,
  0b11110,
  0b00100,
  0b01000,
};
//horloge
RTCDateTime dt;
boolean isAlarm = false;
boolean alarmState = false;
uint16_t Year ;
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
#define angleMini -2544 // angle d'une portion de tour
#define nbPortion 1 // nombre de portions distribuées


AccelStepper stepper(HALFSTEP, mtrPin1, mtrPin2, mtrPin3, mtrPin4);

int opticalSwitch;
// moments de distribution
int Heure1 = 8;
int Minute1 = 15;
int Seconde1 = 00;

int Heure2 = 13;
int Minute2 = 10;
int Seconde2 = 00;

int Heure3 = 18;
int Minute3 = 00;
int Seconde3 = 00;

int currentAlarm = 1;

void calibrate() {
  lcd.setCursor(0, 0);
  lcd.print("Calibrage");
  opticalSwitch = analogRead(A0);
  stepper.setSpeed(speed);
  while (opticalSwitch > 110 || digitalRead(triggerCalibragePin) == LOW)
  {
    stepper.runSpeed();
    opticalSwitch = analogRead(A0);
  }
  stepper.setCurrentPosition(0);
  digitalWrite(mtrPin1, LOW);
  digitalWrite(mtrPin2, LOW);
  digitalWrite(mtrPin3, LOW);
  digitalWrite(mtrPin4, LOW);
  lcd.clear();
  return;
}

void distribute() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Distribution");
  lcd.setCursor(0, 1);
  lcd.print("en cours");
  stepper.setCurrentPosition(0);
  stepper.setSpeed(speed);
  while (stepper.currentPosition() != (angleMini * nbPortion)) // positionnement
  {
    stepper.runSpeed();
  }
  lcd.clear();
  return;
}


void alarmFunction()
{
  Serial.println("*** INT 0 ***");
  isAlarm = true;
}

void setup() {
  lcd.begin(16, 2);
  lcd.createChar(0, degre);

  pinMode(triggerCalibragePin, INPUT_PULLUP);
  Serial.begin(9600);
  lcd.setCursor(0, 0);
  lcd.print("Distri Croquette");
  delay(2000);
  lcd.clear();
  stepper.setMaxSpeed(2000.0);
  stepper.setAcceleration(2000.0);  //Make the acc quick
  stepper.setSpeed(2000);
  Wire.begin();
  Serial.println(F("Initialize System"));
  clock.armAlarm1(false);
  clock.armAlarm2(false);
  clock.clearAlarm1();
  clock.clearAlarm2();
  clock.setAlarm1(0, Heure1, Minute1, Seconde1, DS3231_MATCH_H_M_S);
  //  clock.setAlarm1(0, 8, 30, 0, DS3231_MATCH_H_M_S);
  //clock.setAlarm2(0, 0, 45, DS3231_MATCH_M);
  //  clock.setAlarm2(0, 18, 30, DS3231_MATCH_H_M);
  attachInterrupt(0, alarmFunction, FALLING);

}

void loop() {
  RTCAlarmTime alarm;
  setDate();
  dt = clock.getDateTime();
  Serial.print("heure : "); Serial.println(clock.dateFormat("d-m-Y H:i:s", dt));
  // on regarde si il y a un clibrage
  if (digitalRead(triggerCalibragePin) == LOW) {
    calibrate();
  }

  if (isAlarm) {
    Serial.println("Alarm detectee");
    isAlarm = false;
    distribute();
    // set next alarm
    if (currentAlarm == 1) {
      currentAlarm = 2;
      clock.setAlarm1(0, Heure2, Minute2, Seconde2, DS3231_MATCH_H_M_S);
    } else if (currentAlarm == 2){
      currentAlarm = 3;
      clock.setAlarm1(0, Heure3, Minute3, Seconde3, DS3231_MATCH_H_M_S);
    } else {
      currentAlarm = 1;
      clock.setAlarm1(0, Heure1, Minute1, Seconde1, DS3231_MATCH_H_M_S);
      
    }

    //stop the motor
    digitalWrite(mtrPin1, LOW);
    digitalWrite(mtrPin2, LOW);
    digitalWrite(mtrPin3, LOW);
    digitalWrite(mtrPin4, LOW);
    Serial.print("distibution : "); Serial.println(clock.dateFormat("d/m H:i:s", dt));
  }
  alarm = clock.getAlarm1();
  Serial.print("distibution à: "); Serial.println(clock.dateFormat("H:i:s", alarm));
  lcd.setCursor(0, 0);
  lcd.print(clock.dateFormat("d/m H:i:s", dt));
  lcd.setCursor(6, 1);
  lcd.write((byte)0);
  lcd.setCursor(8, 1);
  lcd.print(clock.dateFormat("H:i:s", alarm));

  delay(250);
}

void setDate( ) { /* function setDate */
  ////Set Real Time clock
  if (Serial.available()) {
    //int _start = millis();
    GetDateStuff(Year, Month, Date, DoW, Hour, Minute, Second);
    clock.setDateTime(Year, Month, Date, Hour, Minute, Second);
    delay(250);

  }
}
void GetDateStuff(uint16_t& Year, byte& Month, byte& Day, byte& DoW, byte& Hour, byte& Minute, byte& Second) { /* function GetDateStuff */
  ////Get date data
  // Call this if you notice something coming in on
  // the serial port. The stuff coming in should be in
  // the order YYYYMMDDwHHMMSS, with an 'x' at the end. Ex. 20220714w081400x
  boolean GotString = false;
  char InChar;
  byte Temp1, Temp2, Temp3, Temp4;
  char InString[20];
  byte j = 0; Serial.println("test 5 ");
  while (!GotString) {
    if (Serial.available() > 0) {
      InChar = Serial.read();
      InString[j] = InChar;
      j += 1;
      if (InChar == 'x') {
        GotString = true;
        while (Serial.available() > 0) Serial.read(); // remove all remaining char
      }
    }
  }
  Serial.println(InString);
  // Read Year first
  Temp1 = (byte)InString[0] - 48;
  Serial.println(Temp1 * 1000);
  Temp2 = (byte)InString[1] - 48;
  Serial.println(Temp2 * 100);
  Temp3 = (byte)InString[2] - 48;
  Serial.println(Temp3 * 10);
  Temp4 = (byte)InString[3] - 48;
  Serial.println(Temp4);
  Year = Temp1 * 1000 + Temp2 * 100 + Temp3 * 10 + Temp4;
  Serial.print("Year ");
  Serial.println(Year);
  // now month
  Temp1 = (byte)InString[4] - 48;
  Temp2 = (byte)InString[5] - 48;
  Month = Temp1 * 10 + Temp2;
  // now date
  Temp1 = (byte)InString[6] - 48;
  Temp2 = (byte)InString[7] - 48;
  Day = Temp1 * 10 + Temp2;
  // now Day of Week
  DoW = (byte)InString[8] - 48;
  // now Hour
  Temp1 = (byte)InString[9] - 48;
  Temp2 = (byte)InString[10] - 48;
  Hour = Temp1 * 10 + Temp2;
  // now Minute
  Temp1 = (byte)InString[11] - 48;
  Temp2 = (byte)InString[12] - 48;
  Minute = Temp1 * 10 + Temp2;
  // now Second
  Temp1 = (byte)InString[13] - 48;
  Temp2 = (byte)InString[14] - 48;
  Second = Temp1 * 10 + Temp2;
}
