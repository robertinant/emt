#include <Energia.h>
#include <ti/sysbios/hal/Timer.h>

void timerIsr(UArg arg)
{
    Serial.print("A0 = ");
    Serial.println(analogRead(A0));
}

void setup5()
{
    Serial.begin(9600);
}

void loop5()
{
  Serial.println("hey");
}
