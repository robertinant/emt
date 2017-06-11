#include <PString.h>
#include <xdc/runtime/System.h>
#include <ti/sysbios/knl/Clock.h>

/* common buffer to assemble output line in */
char lineOutBuffer[256];
PString outLine(lineOutBuffer, sizeof(lineOutBuffer));

String ds(PI);
String is(32);

/*
 *  ======== pstringSetup ========
 */
void pstringSetup(void)
{
    Serial.begin(9600);

    System_printf("pstringSetup() ...");
    System_printf("len (capacity): %d (%d)\n",
                  outLine.length(), outLine.capacity());
    System_flush();
}

/*
 *  ======== pstringLoop ========
 */
void pstringLoop(void)
{
    static int logNum = 0;
    char c = '.';

    if (Serial.available()) {
        c = Serial.read();
    }

    /* reset the output line buffer */
    outLine = "";

    /* compose a line of output */
    outLine.print(logNum++);
    outLine.print(": ");
    outLine.print((float(1.467 * (float)logNum)) / 4096.0, 3);
    outLine.print(", at ");
    outLine.println(Clock_getTicks());

    /* output the line twice: as a Printable and as a string */
    Serial.print(outLine);
    System_printf((const char *)outLine);
}
