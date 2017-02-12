#include <Energia.h>

extern void lp_setup();
extern void lp_loop();
extern void setupTelnetMon();
extern void loopTelnetMon();

void setup0()
{
//    lp_setup();
    setupTelnetMon();
}

void loop0()
{
//    lp_loop();
    loopTelnetMon();
}
