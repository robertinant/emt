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
void setup6x() {                
  // initialize the digital pin as an output.
  pinMode(LED, OUTPUT);     
}

// the loop routine runs over and over again forever:
void loop6x() {
  digitalWrite(LED, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);               // wait for a second
  digitalWrite(LED, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);               // wait for a second
}

void setup6()
{
    Serial.begin(115200);
    Serial.print("millis() at start: ");
    Serial.print(millis());
    Serial.print("micros() at start: ");
    Serial.println(micros());
}

uint32_t microsThen;
uint32_t millisTshen;

void loop6()
{
// put your main code here, to run repeatedly:
    uint32_t millisAfter;
    uint32_t microsAfter;

    uint32_t millisNow = millis();
    uint32_t microsNow = micros();

    delay(1000);

    millisAfter = millis();
    microsAfter = micros();

    Serial.print("\nMillis after delay: ");
    Serial.println(millisAfter - millisNow);

    Serial.print("Micros after delay: ");
    Serial.println(microsAfter - microsNow);

    Serial.print("Micros, Millis: ");
    Serial.print(microsAfter);
    Serial.print(" ");
    Serial.println(millisAfter);

    Serial.print("Micros, Millis: ");
    Serial.print(microsNow);
    Serial.print(" ");
    Serial.println(millisNow);
}

