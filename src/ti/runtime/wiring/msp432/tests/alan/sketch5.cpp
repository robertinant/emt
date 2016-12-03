#include <Energia.h>
#include <ti/sysbios/hal/Timer.h>

void timerIsr(UArg arg)
{
    Serial.print("A0 = ");
    Serial.println(analogRead(A0));
}

void setup5()
{
    Timer_Params prms;

    Timer_Params_init(&prms);
    prms.period = 1000000;
    prms.arg = 1;

    Timer_create(Timer_ANY, timerIsr, &prms, NULL);
    Serial.begin(115200);
}

void loop5()
{
    Task_exit();
}
