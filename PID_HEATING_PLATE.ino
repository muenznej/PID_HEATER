/*
   COUNTMODE:

   1% -> ~1Hz   ->44° steady state
   2% -> ~2Hz   ->__° steady state
   5% -> 5.3Hz  ->46 (nach 1 minute)
   8% -> 9 hz   -> 166 nach 10min

*/

#include <Wire.h>
#include <Adafruit_SSD1306.h>

#include <avr/io.h>
#include <util/delay.h>

#define OLED_RESET -1 // Arduino reset pin
Adafruit_SSD1306 display(OLED_RESET);

#include <Encoder.h>
#define CH_A 3 // A1->3 , aka INT1 should make encoder better (INT0 is for 
#define CH_B A2
Encoder myEnc(CH_A, CH_B);
long oldPosition  = 0;
long newPosition = 0;

#include <max6675.h> //Die MAX6675 Bibliothek

int max6675SO = 4; // Serial Output
int max6675CS = 5; // Chip Select // changed from 3->6
int max6675CLK = 6; // Serial Clock

// Initialisierung der MAX6675 Bibliothek mit
// den Werten der PINs
MAX6675 ktc(max6675CLK, max6675CS, max6675SO);

unsigned long t0 = 0;
unsigned long t1 = 0;
unsigned long t2 = 0;
unsigned long t3 = 0;
unsigned long t4 = 0;
unsigned long t5 = 0;

const unsigned long LOG_TIME_STEP = 1 * 1000; //  ms
const unsigned long TEMP_TIME_STEP = 500; // ms
const unsigned long  TEMP_INC_STEP = 600000; // ms
//const unsigned long  TEMP_INC_STEP = 1 * 1000; // ms

int set_temp = 0;
unsigned long n_log_item = 0;

#include "Dimmer.h"
#define TRIAC_PIN 9
Dimmer dimmer( TRIAC_PIN,  DIMMER_COUNT, 1.5, 50);
int value = 0;

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
//  t0 = millis();
//  t2 = millis();
//  t4 = millis();

  Serial.println("*******************************");
  Serial.println("Log-Time-Step [ms]: ");
  Serial.println(LOG_TIME_STEP);
  Serial.println("Temp-Increase-Time-Step [ms]: ");
  Serial.println(TEMP_INC_STEP);
  Serial.println("*******************************");
}
byte incomingByte = 0;

void loop() {
  Serial.print(t1);
  Serial.print("; ");
  Serial.print(set_temp);
  Serial.print("; ");
  Serial.print(ktc.readCelsius());
  Serial.println("");
    t1 = millis();
  // send data only when you receive data:
  if (Serial.available() > 0) {
    // read the incoming byte:
    set_temp = Serial.parseInt();
    Serial.println(set_temp, DEC);

  }
  
    dimmer.set(set_temp);

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
    delay(500);
  //
  //  long newPosition = myEnc.read();
  //  //Serial.println(newPosition);
  //  if ( newPosition != oldPosition ) {
  //
  //    if ( newPosition <= 0 ) {
  //      newPosition = 0;
  //    }
  //    if ( newPosition >= 100 ) {
  //      newPosition = 100;
  //    }
  //    oldPosition = newPosition;
  //    set_temp = newPosition;
  //    dimmer.set(set_temp);
  //    display.setCursor(0, 0);
  //    display.print("    ");
  //    display.setCursor(0, 0);
  //
  //    display.print(dimmer.getValue());
  //    display.print("(-> ");
  //    display.print(newPosition);
  //    display.print(" )");
  //    display.display();
  //  }
  //
  //  if ((t1 - t0) > TEMP_TIME_STEP) {
  //
  //    display.setCursor(0, 10);
  //    display.print("        ");
  //    display.setCursor(0, 10);
  //    display.print(ktc.readCelsius());
  //    display.println("C");
  //    display.display();
  //    t0 = t1;
  //  }
  //  if ((t1 - t2) > LOG_TIME_STEP) {
  //    // log_data_to_serial
  //
  //    Serial.print(n_log_item);
  //    Serial.print(": -> ");
  //    Serial.print(dimmer.getValue());
  //    Serial.print("% : ");
  //    Serial.print(ktc.readCelsius());
  //    Serial.println("");
  //    n_log_item++;
  //    t2 = t1;
  //  }
  //  if ((t1 - t4) > TEMP_INC_STEP) {
  //    set_temp += 5;
  //    dimmer.set(set_temp);
  //    t4 = t1;
  //  }
}

