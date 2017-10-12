/*
  Blink
  The basic Energia example.
  Turns on an LED on for one second, then off for one second, repeatedly.
  Change the LED define to blink other LEDs.
  
  Hardware Required:
  * LaunchPad with an LED
  
  This example code is in the public domain.
*/

#include <Energia.h>

// most launchpads have a red LED
#define LED RED_LED

//see pins_energia.h for more LED definitions
//#define LED GREEN_LED
  
// the setup routine runs once when you press reset:
void setup6() {                
  // initialize the digital pin as an output.
  pinMode(LED, OUTPUT);     
}

// the loop routine runs over and over again forever:
void loop6() {
  digitalWrite(LED, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);               // wait for a second
  digitalWrite(LED, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);               // wait for a second
}

void setup6x()
{
    Serial.begin(115200);
    Serial.print("millis() at start: ");
    Serial.print(millis());
    Serial.print("micros() at start: ");
    Serial.println(micros());
}

uint32_t microsThen;
uint32_t millisTshen;

void loop6x()
{
// put your main code here, to run repeatedly:
    uint32_t microsNow = micros();
    uint32_t millisNow = millis();
    delay(1000);
    Serial.print("\nMicros after delay: ");
    Serial.println(micros() - microsNow);
    Serial.print("Millis after delay: ");
    Serial.println(millis() - millisNow);
    Serial.print("Micros after delay: ");
    Serial.println(micros() - microsNow);
}

