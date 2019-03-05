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
//#define KU 8
//#define TU 70
//double myKp = 9, myKi = 0.0, myKd = 0.000;
//double myKp = 4.5, myKi = 0.6, myKd = 0.000;
//double myKp = 0.8*0.6*KU , myKi = 0.80*1.2*KU/TU, myKd = 2.0*3.0*KU*TU/40; // good values, overshoot

#define KU 4
#define TU 70
//double myKp = KU, myKi = 0.0, myKd = 0.000;
double myKp = 0.6*KU , myKi = 1.2*KU/TU, myKd = 3.0*KU*TU/40; 
PID myPID(&Input, &Output, &Setpoint, myKp, myKi, myKd, P_ON_E , DIRECT); // P_ON_M Proportional on Measurement, helps prevent overshoots
#define MIN_T 0
#define MAX_T 150
#define SAMPLETIME 2000

/* KU ~ 8-9, T = 70s
   NZ method for PI controler
   KP = 0.45KU
   KI = 0.54/T*KU
*/

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
  Setpoint = 60;
  oldPosition = Setpoint;
  myEnc.write(Setpoint);
}
void loop() {

  //  if (Serial.available() > 0) {
  //    //set_temp = Serial.parseInt();
  //    Setpoint = Serial.parseInt();
  //    Serial.println(set_temp, DEC);
  //  }

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
  tnew = millis();
  if ((tnew - told_serial) >= SAMPLETIME) {
    //    Input = constrain(ktc.readCelsius(), MIN_T, MAX_T);
    //    Input = map(Input, MIN_T, MAX_T, 0, 100);
    Input = ktc.readCelsius();
    myPID.Compute();
    set_temp = Output;
    Serial.print( ((float)tnew) / 1000 );
    Serial.print("; ");
    Serial.print(Input);
    Serial.print("; ");
    Serial.print(Output);
    Serial.print("; ");
    Serial.print(Setpoint);
    Serial.print("; ");
    Serial.println("");
    //   SerialReceive();
    //   SerialSend();
    told_serial = tnew;
  }

  if ((tnew - told_display) >= 500) {
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
    display.print(dimmer.getValue());
    display.print("%");


  }

  //_delay_ms(5);

  dimmer.set(set_temp);

}
//
//
//union {                // This Data structure lets
//  byte asBytes[24];    // us take the byte array
//  float asFloat[6];    // sent from processing and
//}                      // easily convert it to a
//foo;                   // float array
//
//// getting float values from processing into the arduino
//// was no small task.  the way this program does it is
//// as follows:
////  * a float takes up 4 bytes.  in processing, convert
////    the array of floats we want to send, into an array
////    of bytes.
////  * send the bytes to the arduino
////  * use a data structure known as a union to convert
////    the array of bytes back into an array of floats
//
////  the bytes coming from the arduino follow the following
////  format:
////  0: 0=Manual, 1=Auto, else = ? error ?
////  1: 0=Direct, 1=Reverse, else = ? error ?
////  2-5: float setpoint
////  6-9: float input
////  10-13: float output
////  14-17: float P_Param
////  18-21: float I_Param
////  22-245: float D_Param
//void SerialReceive()
//{
//
//  // read the bytes sent from Processing
//  int index = 0;
//  byte Auto_Man = -1;
//  byte Direct_Reverse = -1;
//  while (Serial.available() && index < 26)
//  {
//    if (index == 0) Auto_Man = Serial.read();
//    else if (index == 1) Direct_Reverse = Serial.read();
//    else foo.asBytes[index - 2] = Serial.read();
//    index++;
//  }
//
//  // if the information we got was in the correct format,
//  // read it into the system
//  if (index == 26  && (Auto_Man == 0 || Auto_Man == 1) && (Direct_Reverse == 0 || Direct_Reverse == 1))
//  {
//    Setpoint = double(foo.asFloat[0]);
//    Input = double(foo.asFloat[1]);     // * the user has the ability to send the
//    //   value of "Input"  in most cases (as
//    //   in this one) this is not needed.
//    if (Auto_Man == 0)                    // * only change the output if we are in
//    { //   manual mode.  otherwise we'll get an
//      Output = double(foo.asFloat[2]);    //   output blip, then the controller will
//    }                                     //   overwrite.
//
//    double p, i, d;                       // * read in and set the controller tunings
//    p = double(foo.asFloat[3]);           //
//    i = double(foo.asFloat[4]);           //
//    d = double(foo.asFloat[5]);           //
//    myPID.SetTunings(p, i, d);        //
//
//    if (Auto_Man == 0) myPID.SetMode(MANUAL); // * set the controller mode
//    else myPID.SetMode(AUTOMATIC);             //
//    //else myPID.SetMode(MANUAL);             //
//    if (Direct_Reverse == 0) myPID.SetControllerDirection(DIRECT); // * set the controller Direction
//    else myPID.SetControllerDirection(REVERSE);          //
//  }
//  Serial.flush();                         // * clear any random data from the serial buffer
//}
//
//// unlike our tiny microprocessor, the processing ap
//// has no problem converting strings into floats, so
//// we can just send strings.  much easier than getting
//// floats from processing to here no?
//void SerialSend()
//{
//  Serial.print("PID ");
//  Serial.print(Setpoint);
//  Serial.print(" ");
//  Serial.print(Input);
//  Serial.print(" ");
//  Serial.print(Output);
//  Serial.print(" ");
//  Serial.print(myPID.GetKp());
//  Serial.print(" ");
//  Serial.print(myPID.GetKi() );
//  Serial.print(" ");
//  Serial.print(myPID.GetKd());
//  Serial.print(" ");
//  if (myPID.GetMode() == AUTOMATIC) Serial.print("Automatic");
//  else Serial.print("Manual");
//  Serial.print(" ");
//  if (myPID.GetDirection() == DIRECT) Serial.println("Direct");
//  else Serial.println("Reverse");
//}

