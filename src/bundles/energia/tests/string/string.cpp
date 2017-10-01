#include <Energia.h> // Energia Wiring API
#include <WString.h> // Energia Wiring String API
#include <xdc/runtime/System.h>
#include <xdc/runtime/Memory.h>

int ledPin = RED_LED;

String stringOne = String("Hello: ");

__extern void stringSetup(void)
{
    System_printf("stringSetup() ...");
    System_flush();

    System_printf("len: %d\n", (stringOne + " String").length());

    pinMode(ledPin, OUTPUT);    // set ledPin pin as output
}

__extern void stringLoop(void)
{
    Memory_Stats stats;
    static int count = 1;
    static float fcount = 2.0f;
    const char *cstr;
    
    digitalWrite(ledPin, HIGH); // set the LED on

    cstr = (stringOne + count + '\n').c_str();
    System_printf("%s", cstr);
    count++;

    cstr = (stringOne + fcount + '\n').c_str();
    System_printf("%s", cstr);
    fcount *= 2.0;
    
    Memory_getStats(NULL, &stats);
    System_printf("heap size: 0x%x, free: 0x%x, max free block: 0x%x\n",
                  stats.totalSize, stats.totalFreeSize, stats.largestFreeSize);
    System_flush();

    delay(50);                  // wait for 1/20 second
    digitalWrite(ledPin, LOW);  // set the LED off
    delay(950);                 // wait for (1 - 1/20) second
}
