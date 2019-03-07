#define DEBUG_LEVEL 1

#if DEBUG_LEVEL >= 2
#define DEBUG_2_PRINT(x) Serial.print(x)
#define DEBUG_2_PRINTLN(x) Serial.println(x)
#define DEBUG_PRINT(x)  Serial.print(x)
#define DEBUG_PRINTLN(x)  Serial.println(x)
#elif DEBUG_LEVEL >= 1
#define DEBUG_2_PRINT(x)
#define DEBUG_2_PRINTLN(x)
#define DEBUG_PRINT(x)  Serial.print(x)
#define DEBUG_PRINTLN(x)  Serial.println(x)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#define DEBUG_2_PRINT(x)
#define DEBUG_2_PRINTLN(x)
#endif

#include <Wire.h>
#include <Adafruit_SSD1306.h>

#include <avr/io.h>
#include <util/delay.h>

#define OLED_RESET -1 // Arduino reset pin
Adafruit_SSD1306 display(OLED_RESET);

#include <Encoder.h>
#define CH_A A2 //
#define CH_B 3 //INT1
Encoder myEnc(CH_A, CH_B);
int oldPosition  = 0;
int newPosition = 0;

#include <max6675.h> //Die MAX6675 Bibliothek
int max6675SO = 4; // Serial Output
int max6675CS = 5; // Chip Select // changed from 3->6
int max6675CLK = 6; // Serial Clock
MAX6675 ktc(max6675CLK, max6675CS, max6675SO);

#include "Dimmer.h"
#define TRIAC_PIN 9
Dimmer dimmer( TRIAC_PIN,  DIMMER_COUNT, 1.5, 50);

byte incomingByte = 0;
unsigned long tnew = 0;
unsigned long told_serial = 0;
unsigned long told_display = 0;
unsigned long told_PID = 0;
byte set_temp = 0;

#include <PID_v1.h>
double Setpoint, Input, Output;
// AUTO PID WITH THIS FOLK
//https://github.com/br3ttb/Arduino-PID-AutoTune-Library/tree/f7ac48217c9f15f9a6d1ceb6281417045bc06a93
#define KU 4
#define TU 70
//double myKp = KU, myKi = 0.0, myKd = 0.000;
double myKp = 0.6 * KU , myKi = 0.8 * 1.2 * KU / TU, myKd = 1.0 * 3 * KU * TU / 40;
PID myPID(&Input, &Output, &Setpoint, myKp, myKi, myKd, P_ON_E , DIRECT); // P_ON_M Proportional on Measurement, helps prevent overshoots
#define MIN_T 0
#define MAX_T 170
#define SAMPLETIME 1000
/* KU ~ 8-9, T = 70s
   NZ method for PI controler
   KP = 0.45KU
   KI = 0.54/T*KU
*/

void UpdateEncoder() {
  newPosition = myEnc.read();
  if ( newPosition != oldPosition ) {
    if ( newPosition < MIN_T ) {
      newPosition = 0;
      myEnc.write(newPosition);
    }
    if ( newPosition > MAX_T ) {
      newPosition = 100;
      myEnc.write(newPosition);
    }
    oldPosition = newPosition;
    Setpoint = newPosition;
  }
}
void UpdateSerial() {
  DEBUG_PRINT( ((float)tnew) / 1000 );
  DEBUG_PRINT("; ");
  DEBUG_PRINT(Input);
  DEBUG_PRINT("; ");
  DEBUG_PRINT(Output);
  DEBUG_PRINT("; ");
  DEBUG_PRINT(Setpoint);
  DEBUG_PRINT("; ");
  DEBUG_PRINTLN("");
  told_serial = tnew;
}
void UpdateDisplay() {
  display.setCursor(0, 0);
  display.print("                    ");
  display.setCursor(0, 0);
  display.print("Target T:  ");
  display.print(Setpoint);
  display.print("C");

  display.setCursor(0, 10);
  display.print("                    ");
  display.setCursor(0, 10);
  display.print("Current T: ");
  display.print(ktc.readCelsius());
  display.println("C");
  display.display();
  told_display = tnew;

  display.setCursor(0, 20);
  display.print("                    ");
  display.setCursor(0, 20);
  display.print("Duty Cycle: ");
  display.print(Output);
  display.print("%");
}

long time_stable = 0;
long first_time_stable = -1;

boolean SetTemperature( int16_t temp, int holdTime = -1, byte TOL = 5) {
  Setpoint = temp;
  if (temp < 0) {
    Setpoint = 0;
    return true;
  }
  if ( abs(Input - temp) <= TOL ) {
    DEBUG_2_PRINT("stable");
    if (holdTime < 0) {
      time_stable = 0;
      first_time_stable = 0;
      DEBUG_2_PRINT(" holdtime forever ");
      DEBUG_2_PRINT(";");
      return false;// stay on infintily

    }
    else {
      time_stable = millis();
      if (first_time_stable < 0) {
        first_time_stable = time_stable;
        DEBUG_2_PRINT(" (first time at): ");
        DEBUG_2_PRINT(first_time_stable);
        DEBUG_2_PRINT(";");
        return false;
      }
      else {
        DEBUG_2_PRINT(" dt: ");
        DEBUG_2_PRINT(time_stable - first_time_stable);
        if ( (time_stable - first_time_stable) >= (holdTime * 1000.0) ) {
          DEBUG_2_PRINT(" done!");
          first_time_stable = -1;
          time_stable =  0;
          DEBUG_2_PRINT(";");
          return true;
        }
        else {
          DEBUG_2_PRINT(";");
          return false;
        }
      }
    }
  }
  else {
    DEBUG_2_PRINT("not stable");
    first_time_stable = -1;
    time_stable = 0;
    DEBUG_2_PRINT(";");
    return false;
  }

}

void setup() {
  pinMode(CH_A, INPUT_PULLUP);
  pinMode(CH_B, INPUT_PULLUP);
  unsigned long BAUDRATE = 9600;
  Serial.begin(BAUDRATE);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setTextColor(WHITE, BLACK);
  display.setTextSize(1);
  display.setCursor(0, 0);

  dimmer.begin();
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 100); // Set  Limits to PWM limits
  myPID.SetSampleTime( SAMPLETIME ); // Set PID sampling time to 10ms
  Setpoint = 0;
  oldPosition = Setpoint;
  myEnc.write(Setpoint);
}


int temps []      = { 150, 200, 260, -1 };
int hold_times [] = { 180, 60, 40, -1};
#define NUM_ELEM (sizeof( temps ) / sizeof( temps[0] ))
byte cnt = 0;

void loop() {
  //  if (Serial.available() > 0) {
  //    //set_temp = Serial.parseInt();
  //    Setpoint = Serial.parseInt();
  //    Serial.println(set_temp, DEC);
  //  }

  tnew = millis();

  if ( (tnew - told_PID) >= SAMPLETIME ) {
    told_PID = tnew;
    if (Setpoint >= MAX_T) {
      Setpoint = MAX_T;
    }
    if (Setpoint <= MIN_T) {
      Setpoint = MIN_T;
    }
    Input = ktc.readCelsius();
    myPID.Compute();

    if ( SetTemperature(temps[cnt], hold_times[cnt]) ) {
      Serial.print("Step ");
      Serial.print(cnt);
      Serial.println(" reached");
      if (cnt >= (NUM_ELEM - 1)) {
        Serial.println(" Profile Finished, turning off!");
        Setpoint = 0;
        cnt = (NUM_ELEM - 1);
      } else {
        cnt++;
      }
    }
  }

  UpdateEncoder();

  if ((tnew - told_serial) >= 1000) {
    UpdateSerial();
  }

  if ((tnew - told_display) >= 500) {
    UpdateDisplay();
  }


  dimmer.set(Output);
}


