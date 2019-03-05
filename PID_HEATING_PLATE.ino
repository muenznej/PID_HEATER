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
byte set_temp = 0;

#include <PID_v1.h>
double Setpoint, Input, Output;
double myKp = 0, myKi = 10, myKd = 0.001;
PID myPID(&Input, &Output, &Setpoint, myKp, myKi, myKd, P_ON_M , DIRECT); // Proportional on Measurement, helps prevent overshoots
#define MIN_T 0
#define MAX_T 150
#define SAMPLETIME 1000

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

}
void loop() {
  Input = constrain(ktc.readCelsius(), MIN_T, MAX_T);
  Input = map(Input, MIN_T, MAX_T, 0, 255);
  myPID.Compute();
  set_temp = Output;


  if (Serial.available() > 0) {
    set_temp = Serial.parseInt();
    Serial.println(set_temp, DEC);
  }

  newPosition = myEnc.read();
  if ( newPosition != oldPosition ) {
    if ( newPosition < 0 ) {
      newPosition = 0;
      myEnc.write(newPosition);
    }
    if ( newPosition > 100 ) {
      newPosition = 100;
      myEnc.write(newPosition);
    }
    oldPosition = newPosition;
    set_temp = newPosition;

  }
  tnew = millis();
  if ((tnew - told_serial) >= 1000) {

    Serial.print(tnew / 1000.0);
    Serial.print(";");
    Serial.print(set_temp);
    Serial.print(";");
    Serial.print(ktc.readCelsius());
    Serial.print(";");
    Serial.println("°C");
    told_serial = tnew;
  }

  if ((tnew - told_display) >= 500) {
    display.setCursor(0, 0);
    display.print("    ");
    display.setCursor(0, 0);
    display.print(dimmer.getValue());
    display.setCursor(0, 10);
    display.print("        ");
    display.setCursor(0, 10);
    display.print(ktc.readCelsius());
    display.println("C");
    display.display();
    told_display = tnew;
  }

  //_delay_ms(5);

  dimmer.set(set_temp);

}

