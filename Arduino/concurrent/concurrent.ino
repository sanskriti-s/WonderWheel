#include <Arduino_FreeRTOS.h>
#include <odrive-arduino.h>
#include <HardwareSerial.h>
#include <SoftwareSerial.h>
#include <LiquidCrystal.h>

//wiring to lcd: https://create.arduino.cc/projecthub/najad/interfacing-lcd1602-with-arduino-764ec4

#define cellPin A8
// A8 - battery, GND-GND
// pin 19: RX - connect to ODrive TX
// pin 18: TX - connect to ODrive RX
HardwareSerial& odrive_serial = Serial1;
//HardwareSerial& hc06 = Serial3;

// Printing with stream operator helper functions
template<class T> inline Print& operator <<(Print &obj,     T arg) {
  obj.print(arg);
  return obj;
}
template<>        inline Print& operator <<(Print &obj, float arg) {
  obj.print(arg, 4);
  return obj;
}

// ODrive object
ODriveArduino odrive(odrive_serial);

//LCD setup
const int rs = 12, en = 13, d4 = 46, d5 = 48, d6 = 50, d7 = 52;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

//Battery charge math variables
const float mvpc = 4.55 ; //measured voltage of arduino through voltmeter
float counts = 0;  //battery volts in millivolts
float mv = 0;
float multiplier = 5.18;
float output = 0;
int charge = 0;

//Drive math variable
float current_theta = 0;

void Drive (void *pvParameters);
void Charge (void *pvParameters);

void setup() {
  xTaskCreate (
    Drive,
    (const char *)"Drive",//Human readable name
    128,//Stack size
    NULL,
    2,//Priority
    NULL );

  xTaskCreate (
    Charge,
    (const char *)"Charge",
    128,
    NULL,
    1,
    NULL );
}

void loop() {
  // Empty
}

void Drive (void *pvParameters)
{
  (void) pvParameters;

  //Serials
  Serial.begin(115200);
  odrive_serial.begin(115200);

  for (;;) {
    // read the input on analog pin 0:
    int adc_reading1 = analogRead(A0);
    // read the input on analog pin 1:
    int adc_reading2 = analogRead(A1);

    //Drive math
    float x = (float) adc_reading1 - 511;
    float y = (float) adc_reading2 - 511;
    float r = sqrt(x * x + y * y);    //distance formula
    float theta = atan(y / x);        //angle formula
    //Max r= 722 & max speed= 1
    float r_conv = 1.00*r/722.66;     //convert into fraction of max
    float theta_conv = 1.000 / theta; //convert into fraction of max
    //Serial.println(r_conv);
    //Serial.println(theta_conv);
    delay(5);                         // delay in between reads for stability

    //calculate how much the pivot motor has to moverelative to current position 
    float last_theta = current_theta; 
    float current_theta = theta_conv;
    float angle = current_theta - last_theta;

    odrive.SetPosition(0, angle);
    odrive.SetVelocity(1, r_conv);

    //Put in delay so other task is executed
    vTaskDelay( 1000 / portTICK_PERIOD_MS );
  }
}

void Charge (void *pvParameters)
{
  (void) pvParameters;
  //Initialize Bluetooth Serial Port
  //hc06.begin(9600);

  //Serial.begin(9600);     //  opens serial port, sets data rate to 9600 bps
  lcd.begin(16, 2);       // set up the LCD's number of columns and rows:

  pinMode(52, OUTPUT);
  pinMode(50, OUTPUT);
  pinMode(48, OUTPUT);
  pinMode(46, OUTPUT);
  pinMode(A8, INPUT);

  for (;;)
  {
    counts = analogRead(cellPin);
    mv = counts * mvpc;

    //Calculate  and print voltage to serial
    output = (mv * multiplier) / 1000 ;
    //Serial.print(output);
    //Serial.println("V");

    //Calculate and print charge to serial
    charge = (counts / 1023) * 100;
    //Serial.print(charge);
    //Serial.println("%");

    delay(1000);

    //Print charge to LCD
    lcd.setCursor(0, 1);
    lcd.print("Charge= ");
    lcd.print(charge);
    lcd.print("%");
    delay(100);

    //Write data from HC06 to Serial Monitor
    //if (hc06.available()){
    //  Serial.write(hc06.read());
    //}

    //Write from Serial Monitor to HC06
    //if (Serial.available()){
    //  hc06.write(Serial.read());
    //}

  }
}
