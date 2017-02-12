#include <Energia.h>
#include <Wire.h>

#include <xdc/std.h>

#include <xdc/runtime/System.h>
#include <xdc/runtime/Memory.h>
#include <xdc/runtime/Types.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/utils/Load.h>
#include <ti/sysbios/hal/Hwi.h>

#ifndef SERIAL
#define SERIAL Serial
#endif
#ifndef MON_SETUP
#define MON_SETUP mon_setup
#endif
#ifndef MON_LOOP
#define MON_LOOP mon_loop
#endif

/*
 * This is the sketch for the UART debug console. Nothing else should be using the same
 * serial connection.
 *
 * To add a command, simply create a prototype for the handler function following the
 * model of the handler prototypes in the `local functions' section below, add the
 * function itself in the `local function defs' section below, and add a line to the
 * _consoleCommandTable definition in the `command table' section. The name of your
 * command must have a length < MAX_COMMAND_LEN, and the handler's name should follow
 * the format `consoleHandler_<NAME>'. Also note that the line that is passed into the
 * handler starts right after the command's name; if arguments were supplied to the
 * command, *line will be a space character.
 */

#define DM_CMD 1     /* dump memory cmd */
#define WM_CMD 1     /* write to memory cmd */
#define DRW_CMDS 1   /* digital read/write cmds */
#define ARW_CMDS 1   /* analog read/write cmds */
#define WR_CMD 1     /* Wire read cmd */
#define WW_CMD 1     /* Wire write cmd */
#define SPI_CMD 1    /* SPI transfer cmd */
#define PRI_CMD 1    /* Set Task priority cmd */
#define STATS_CMD 1  /* CPU and task utilzation stats cmd */
#define ALOG_CMD 1   /* log analog value cmd */
#define PI_CMD 1     /* pulseIn() cmd */
#define ARTEST_CMD 1 /* analogRead() self test */
#define AWTEST_CMD 1 /* analogWrite() self test */

#if ARTEST_CMD == 1
#if defined(BOARD_CC3200LP) || defined(BOARD_CC3200_LAUNCHXL)
#define CC32XX_ARTEST_CMD 1
#define MSP432_ARTEST_CMD 0
#define CC26XX_ARTEST_CMD 0
#elif defined(BOARD_MSP432LP) || defined(BOARD_MSP_EXP432P401R)
#define CC32XX_ARTEST_CMD 0
#define MSP432_ARTEST_CMD 1
#define CC26XX_ARTEST_CMD 0
#elif defined(BOARD_CC2650_LAUNCHXL) || defined(BOARD_CC1310_LAUNCHXL) || defined(BOARD_CC1350_LAUNCHXL)
#define CC32XX_ARTEST_CMD 0
#define MSP432_ARTEST_CMD 0
#define CC26XX_ARTEST_CMD 1
#endif
#endif

#if AWTEST_CMD == 1
#if defined(BOARD_CC3200LP) || defined(BOARD_CC3200_LAUNCHXL)
#define CC32XX_AWTEST_CMD 1
#define MSP432_AWTEST_CMD 0
#define CC26XX_AWTEST_CMD 0
#elif defined(BOARD_MSP432LP) || defined(BOARD_MSP_EXP432P401R)
#define CC32XX_AWTEST_CMD 0
#define MSP432_AWTEST_CMD 1
#define CC26XX_AWTEST_CMD 0
#elif defined(BOARD_CC2650_LAUNCHXL) || defined(BOARD_CC1310_LAUNCHXL) || defined(BOARD_CC1350_LAUNCHXL)
#define CC32XX_AWTEST_CMD 0
#define MSP432_AWTEST_CMD 0
#define CC26XX_AWTEST_CMD 1
#endif
#endif

#define MAX_COMMAND_LEN 48
#define MAX_COMMAND_NAME_LEN 8
#define MAX_COMMAND_LINES 5


// Return codes for handlers
#define RETURN_SUCCESS          (0)
#define RETURN_FAIL             (-1)
#define RETURN_FAIL_PRINT_USAGE (-2)

typedef void (*RepeatFunc)(uint32_t arg0, uint32_t arg1, uint32_t arg2, uint32_t arg3);
static void doRepeat(RepeatFunc func, uint32_t arg0, uint32_t arg1, uint32_t arg2, uint32_t arg3);

static void sanitizeLine(char *line);

static int consoleHandler_help(const char *line);

#if DRW_CMDS == 1
static int consoleHandler_dr(const char *line);
static int consoleHandler_dw(const char *line);
#endif

#if ARW_CMDS == 1
static int consoleHandler_ar(const char *line);
static int consoleHandler_aw(const char *line);
#endif

#if WM_CMD == 1
static int consoleHandler_wm(const char *line);
static int consoleHandler_wm1(const char *line);
static int consoleHandler_wm2(const char *line);
#endif

#if DM_CMD == 1
static int consoleHandler_dm(const char *line);
#endif

#if STATS_CMD == 1
static int consoleHandler_stats(const char *line);
#endif

#if SPI_CMD == 1
static int consoleHandler_spi(const char *line);
#endif

#if WW_CMD == 1
static int consoleHandler_wwr(const char *line);
#endif

#if WR_CMD == 1
static int consoleHandler_wrd(const char *line);
#endif

#if (WR_CMD == 1) || (WW_CMD == 1) || (ARTEST_CMD == 1)
static bool wireBegun = false;
#endif

#if PRI_CMD == 1
static int consoleHandler_pri(const char *line);
#endif

#if PI_CMD == 1
static int consoleHandler_pi(const char *line);
#endif

#if ALOG_CMD == 1
static int consoleHandler_alog(const char *line);
static int consoleHandler_alogs(const char *line);
#endif

#if ARTEST_CMD == 1
static int consoleHandler_artest(const char *line);
#endif

#if AWTEST_CMD == 1
static int consoleHandler_awtest(const char *line);
#endif

static char home[] = "\e[H";
static char clear[] = "\e[2J";

/*
 *  ======== command table ========
 */
#define GEN_COMMTABLE_ENTRY(name, desc, detail) {   #name,  consoleHandler_##name,  desc,   detail  }

static const struct {
    const char*name;                    // The name of the command
    int (*handler)(const char*line);    // Pointer to the command's handler
    const char*description;             // A short description of the command (for use by help)
    const char*detailedUsage;           // A detailed description of the command's usage (for use by help)
} _consoleCommandTable[] = {
#if DM_CMD == 1
    GEN_COMMTABLE_ENTRY(dm,      "dump memory",                 "usage: dm <address (hex)> <num words> <word size (1/4)>"),
#endif
#if WM_CMD == 1
    GEN_COMMTABLE_ENTRY(wm,      "write to memory (32 bits)",   "usage: wm <address (hex)> <words..>"),
    GEN_COMMTABLE_ENTRY(wm2,     "write to memory (16 bits)",   "usage: wm <address (hex)> <half words..>"),
    GEN_COMMTABLE_ENTRY(wm1,     "write to memory (8 bits)",    "usage: wm <address (hex)> <bytes..>"),
#endif
#if DRW_CMDS == 1
    GEN_COMMTABLE_ENTRY(dw,      "digitalWrite to pin",         "usage: dw <pin> <value>"),
    GEN_COMMTABLE_ENTRY(dr,      "digitalRead from pin",        "usage: dr <pin>"),
#endif
#if ARW_CMDS == 1
    GEN_COMMTABLE_ENTRY(aw,      "analogWrite to pin",          "usage: aw <pin> <value>"),
    GEN_COMMTABLE_ENTRY(ar,      "analogRead from pin",         "usage: ar <pin>"),
#endif
#if ALOG_CMD == 1
    GEN_COMMTABLE_ENTRY(alog,    "log a pin's value",           "usage: alog <pin> <period>"),
    GEN_COMMTABLE_ENTRY(alogs,   "analog input scale factor",   "usage: alogs (float)scaleFactor"),
#endif
#if PRI_CMD == 1
    GEN_COMMTABLE_ENTRY(pri,     "Set task priority",           "usage: pri <task handle> <priority>"),
#endif
#if PI_CMD == 1
    GEN_COMMTABLE_ENTRY(pi,      "Pulse measurement",           "usage: pi <pin> <state> <timeout>"),
#endif
#if SPI_CMD == 1
    GEN_COMMTABLE_ENTRY(spi,     "SPI transfer",                "usage: spi <cs pin> <data>"),
#endif
#if WR_CMD == 1
    GEN_COMMTABLE_ENTRY(wrd,     "Wire read",                   "usage: wr <addr>"),
#endif
#if WW_CMD == 1
    GEN_COMMTABLE_ENTRY(wwr,     "Wire write",                  "usage: ww <addr> <data> <data> <...>"),
#endif
#if STATS_CMD == 1
    GEN_COMMTABLE_ENTRY(stats,   "Print CPU utlization info",   "usage: stats"),
#endif
#if ARTEST_CMD == 1
    GEN_COMMTABLE_ENTRY(artest,  "analogRead test",             "usage: artest"),
#endif
#if AWTEST_CMD == 1
    GEN_COMMTABLE_ENTRY(awtest,  "analogWrite test",            "usage: awtest"),
#endif
    GEN_COMMTABLE_ENTRY(help,    "Get information on commands. Usage: help [command]",  NULL),
    {NULL,NULL,NULL,NULL}   // Indicates end of table
};

void MON_SETUP(void)
{
    SERIAL.begin(115200, true);
    SERIAL.println("Welcome! This is the SERIAL debug console.");
}

#define UP_ARROW 0x0b    /* ctrl K */
#define DOWN_ARROW 0x0C  /* ctrl L */

void MON_LOOP()
{
    // each loop iteration is one command

    SERIAL.print("> ");

    // ------------------------------------- read line
    static char line[MAX_COMMAND_LINES][MAX_COMMAND_LEN];

    static int line_num = 0;
    int char_index = 0;
    bool line_end = false;
    int escape_index = 0;

    while (true) {

//        while (SERIAL.available() == 0) delay(10);

        char c = SERIAL.read();

        if (escape_index) {
            if (++escape_index == 3) {
                escape_index = 0;
                switch(c) {
                    case 'A':
                        c = UP_ARROW;
                        break;
                    case 'B':
                        c = DOWN_ARROW;
                        break;
                    default:
                        continue;
                }
            }
            else {
                continue;
            }
        }

        switch (c) {
            case 0x1b: /* escape */
                escape_index = 1;
                continue;
                
            case UP_ARROW:
                if (--line_num < 0) { 
                    line_num = MAX_COMMAND_LINES - 1;
                }
                SERIAL.print("\r                         \r> ");
                char_index = strlen(line[line_num]);
                SERIAL.print(line[line_num]);
                continue;
            case DOWN_ARROW:
                if (++line_num == MAX_COMMAND_LINES) { 
                    line_num = 0;
                }
                SERIAL.print("\r                         \r> ");
                char_index = strlen(line[line_num]);
                SERIAL.print(line[line_num]);
                continue;
            case '\r':
                SERIAL.println("");
                if (char_index == 0) {
                    SERIAL.print("> ");
                    continue;
                }
                else {
                    line_end = true;
                    break;
                }
            case 0x08:
            case 0x7f:   /* Backspace */
                if (char_index >= 1) {
                    char_index -= 1;
                    SERIAL.print("\b \b");
                }
                continue;
            case 3: /* control 'c' */
                SERIAL.println("^C");
                return;
        }

        if (line_end == false) {        
            line[line_num][char_index++] = c;

            if (char_index == MAX_COMMAND_LEN) {
                // The user typed something too long; abort so they don't get surprise commands running
                SERIAL.println("\r\nCommand too long.");
                return;
            }

            // default for SERIAL is ECHO_OFF, and there doesn't seem to be a way to change that
            SERIAL.print(c);
        }
        else {
            line[line_num][char_index] = 0;
            break;
        }
    }

    // ------------------------------------- process line
    sanitizeLine(line[line_num]);

    char cmdstr[MAX_COMMAND_NAME_LEN+1];
    
    memset(cmdstr, 0, sizeof(cmdstr));

    int i;

    for (i = 0; i < MAX_COMMAND_NAME_LEN && line[line_num][i] && line[line_num][i] != ' '; i++) {
        cmdstr[i] = line[line_num][i];
    }

    // ignore empty command
    if (!*cmdstr) {
        return;
    }

    int commandIndex = -1;
    for (i=0; _consoleCommandTable[i].name; i++) {
        if (!strncmp(cmdstr,_consoleCommandTable[i].name,MAX_COMMAND_NAME_LEN)) {
            commandIndex = i;
            break;
        }
    }

    if (commandIndex == -1) {
        SERIAL.print("The command `");
        SERIAL.print(cmdstr);
        SERIAL.println("' was not recognized.");
        return;
    }

    if (!_consoleCommandTable[commandIndex].handler) {
        SERIAL.println("That command has not yet been implemented.");
        return;
    }

    int returnVal = _consoleCommandTable[commandIndex].handler(line[line_num]
            +strlen(_consoleCommandTable[commandIndex].name));

    if (++line_num == MAX_COMMAND_LINES) {
        line_num = 0;
    }
    
    switch (returnVal) {
        case RETURN_SUCCESS:    break;
        case RETURN_FAIL:       break;

        case RETURN_FAIL_PRINT_USAGE: {
            const char*toPrint = _consoleCommandTable[commandIndex].detailedUsage
                     ? _consoleCommandTable[commandIndex].detailedUsage
                     : _consoleCommandTable[commandIndex].description;

            SERIAL.println(toPrint);
            break;
        }

        default:
            SERIAL.println("Warning: unknown return value!");
            break;
    }
}

static void sanitizeLine(char *line)
{
    int len = strlen(line);
    char buf[MAX_COMMAND_LEN];

    strncpy(buf, line, len);    // copy original line to buf (which is our source)
    memset(line, 0, len);       // zero out provided line (which is our destination)

    int bufIndex    = 0;
    int lineIndex   = 0;

    for (; bufIndex < len && lineIndex < len; bufIndex++) {
        char c = buf[bufIndex];
        bool copy = ('A'<=c && c<='Z') ||
                    ('a'<=c && c<='z') ||
                    ('0'<=c && c<='9') ||
                    c=='_' || c==' ' ||
                    c=='.' || c==',' ||
                    c==';' || c==':' ||
                    c=='*' || c=='-';
        if(copy) {
            line[lineIndex++] = c;
        }
    }
}

static int consoleHandler_help(const char *line){
    int i;

    if (!*line) {
        // No command in particular specified; just print them all
        SERIAL.println("Available commands:");
        for (i=0; _consoleCommandTable[i].name; i++) {
            if(!_consoleCommandTable[i].handler)    // if NYI, don't list
                continue;

            SERIAL.print("  ");
            SERIAL.print(_consoleCommandTable[i].name);
            SERIAL.print("\t  ");
            SERIAL.println(_consoleCommandTable[i].description);
        }
    } else {
        // get past the space so we can parse the command
        while (*line && *line==' ') {
            line++;
        }

        char cmdstr[MAX_COMMAND_NAME_LEN+1];
        memset(cmdstr,0,sizeof(cmdstr));
        for (i=0; i<MAX_COMMAND_NAME_LEN && line[i] && line[i] != ' '; i++) {
            cmdstr[i] = line[i];
        }

        int commandIndex = -1;
        for (i=0; _consoleCommandTable[i].name; i++) {
            if (!strncmp(cmdstr,_consoleCommandTable[i].name,MAX_COMMAND_NAME_LEN)) {
                commandIndex = i;
                break;
            }
        }

        if (commandIndex == -1) {
            SERIAL.print("The command `");
            SERIAL.print(cmdstr);
            SERIAL.println("' was not recognized.");
            return RETURN_FAIL;
        }

        const char*toPrint = _consoleCommandTable[i].detailedUsage
                     ? _consoleCommandTable[i].detailedUsage
                     : _consoleCommandTable[i].description;

        SERIAL.print(cmdstr);
        SERIAL.print(": ");
        SERIAL.println(toPrint);
    }

    return RETURN_SUCCESS;
}


static void doRepeat(RepeatFunc func, uint32_t arg0, uint32_t arg1, uint32_t arg2, uint32_t arg3)
{
    SERIAL.print(clear);
    while(!SERIAL.available()) {
        SERIAL.print(home);
        func(arg0, arg1, arg2, arg3);
    }
    SERIAL.read(); /* remove char from input buf */
}

#if DRW_CMDS == 1

static void doDr(uint32_t pin)
{
    SERIAL.print("digitalRead(");
    SERIAL.print(pin);
    SERIAL.print(") = ");
    SERIAL.println(digitalRead(pin));
}

static int consoleHandler_dr(const char *line)
{
    if (*line++ != ' ') {
        return RETURN_FAIL_PRINT_USAGE;
    }
    char *endptr = NULL;
    uint32_t pin = strtol(line, &endptr, 10);

    if (*endptr == ' ') {
        doRepeat((RepeatFunc)doDr, pin, 0, 0, 0);
    }
    else {
        doDr(pin);
    }
    return RETURN_SUCCESS;
}

static int consoleHandler_dw(const char *line)
{
    if (*line++ != ' ') {
        return RETURN_FAIL_PRINT_USAGE;
    }
    char *endptr = NULL;
    uint32_t pin = strtoul(line, &endptr, 10);
    int32_t val = strtol(endptr, NULL, 0);
    SERIAL.print("Calling digitalWrite(");
    SERIAL.print(pin);
    SERIAL.print(", ");
    SERIAL.print(val);
    SERIAL.println(").");
    digitalWrite(pin, val);
    return RETURN_SUCCESS;
}

#endif /* DRW_CMDS */

#if ARW_CMDS == 1

static void doAr(uint32_t pin)
{
    SERIAL.print("analogRead(");
    SERIAL.print(pin);
    SERIAL.print(") = ");
    SERIAL.println(analogRead(pin));
}

static int consoleHandler_ar(const char *line)
{
    if (*line++ != ' ') {
        return RETURN_FAIL_PRINT_USAGE;
    }
    char *endptr = NULL;
    uint32_t pin = strtol(line, &endptr, 10);
    
    if (*endptr == ' ') {
        doRepeat((RepeatFunc)doAr, pin, 0, 0, 0);
    }
    else {
        doAr(pin);
    }
    return RETURN_SUCCESS;
}

static int consoleHandler_aw(const char *line)
{
    if (*line++ != ' ') {
        return RETURN_FAIL_PRINT_USAGE;
    }
    char *endptr = NULL;
    uint32_t pin = strtol(line, &endptr, 10);
    int32_t val = strtol(endptr, NULL, 0);
    SERIAL.print("Calling analogWrite(");
    SERIAL.print(pin);
    SERIAL.print(", ");
    SERIAL.print(val);
    SERIAL.println(").");
    analogWrite(pin,val);
    return RETURN_SUCCESS;
}

#endif /* ARW_CMDS */

#if ALOG_CMD == 1

/* assume 3.3V analog reference voltage */
static float scaleFactor = 3.3;

static int consoleHandler_alogs(const char *line)
{
    if (*line++ != ' ') {
        return RETURN_FAIL_PRINT_USAGE;
    }

    char *endptr = NULL;

    scaleFactor = strtof(line, &endptr);

    SERIAL.print("Scale Factor = ");
    SERIAL.println(scaleFactor);

    return RETURN_SUCCESS;
}

static int consoleHandler_alog(const char *line)
{
    if (*line++ != ' ') {
        return RETURN_FAIL_PRINT_USAGE;
    }

    char *endptr = NULL;

    uint32_t period;
    uint32_t numPins = 0;
    uint32_t logNum = 0;
    unsigned int i, j;
    unsigned int aveNum = 10; /* number of readings to average */
    uint32_t pins[10];
    uint32_t aveTotal[10];

    i = 0;
    endptr = (char *)line;
    do {
        pins[i] = strtoul(endptr, &endptr, 10);
        if (++i == 10) break;
    }
    while (*endptr == ' ');

    if (i == 1) {
        numPins = 1;
        period = 1000;
    }
    else {
        numPins = i-1;
        period = pins[i-1];
    }

    analogReadResolution(12);

    SERIAL.print("Scale Factor = ");
    SERIAL.println(scaleFactor);

    while (true) {
        if (SERIAL.available()) {
            if (SERIAL.read() == 'q') break;
        }
        SERIAL.print(logNum++);
        SERIAL.print(", ");
        for (i = 0; i < numPins; i++) {
            aveTotal[i] = 0;
            for (j = 0; j < aveNum; j++) {
                aveTotal[i] += analogRead(pins[i]);
            }
            SERIAL.print((float(scaleFactor * (float)aveTotal[i]/(float)aveNum)) / 4096.0, 3);
            if (i < numPins - 1) {
                SERIAL.print(", ");
            }
        }

        SERIAL.print(" ");
        SERIAL.println(Clock_getTicks());
        delay(period);
    }

    return RETURN_SUCCESS;
}
#endif /* ALOG_CMD */

#if DM_CMD == 1
static void dumpMemory(uint32_t *address, uint32_t len, uint32_t size)
{
    static char response[80];

    uint8_t *base_addr = (uint8_t*)address;

    int i;
    if (size == 1) {
        for (i=0; i < len; i+=16) {
            uint8_t *addr = base_addr+i;

            // for printing literal values
            uint8_t asciiBytes[16];
            int j;
            for (j=0; j<16; j++) {
                asciiBytes[j] = (addr[j] > 31 && addr[j] < 127) ? addr[j] : '.';
            }

            System_snprintf(response,sizeof(response),
                "0x%x | %02x %02x %02x %02x  %02x %02x %02x %02x  %02x %02x %02x %02x  %02x %02x %02x %02x | %.16s",
                (int)addr,
                addr[0], addr[1], addr[2], addr[3], addr[4], addr[5], addr[6], addr[7],
                addr[8], addr[9], addr[10], addr[11], addr[12], addr[13], addr[14], addr[15],
                asciiBytes);
            SERIAL.println(response);
        }
    }

    if (size == 2) {
        for (i=0; i < len; i+=8) {
            uint16_t *addr = (uint16_t *)base_addr+i;
            uint8_t *baddr = (uint8_t *)addr;

            // for printing literal values
            uint8_t asciiBytes[16];
            int j;
            for (j=0; j<16; j++) {
                asciiBytes[j] = (baddr[j] > 31 && baddr[j] < 127) ? baddr[j] : '.';
            }

            System_snprintf(response,sizeof(response),
                "0x%x | %04x %04x  %04x %04x  %04x %04x  %04x %04x | %.16s",
                (int)addr,
                addr[0], addr[1], addr[2], addr[3], addr[4], addr[5], addr[6], addr[7],
                asciiBytes);
            SERIAL.println(response);
        }
    }

    if (size == 4) {
        for (i=0; i < len; i+=4) {
            uint32_t *addr = (uint32_t *)base_addr+i;
            uint8_t *baddr = (uint8_t *)addr;

            // for printing literal values
            uint8_t asciiBytes[16];
            int j;
            for (j=0; j<16; j++) {
                asciiBytes[j] = (baddr[j] > 31 && baddr[j] < 127) ? baddr[j] : '.';
            }

            System_snprintf(response,sizeof(response),
                "0x%x | %08x %08x %08x %08x | %.16s",
                (int)addr,
                addr[0], addr[1], addr[2], addr[3],
                asciiBytes);
            SERIAL.println(response);
        }
    }
}

static int consoleHandler_dm(const char *line)
{
    static uint32_t address = 0x00000000;
    static uint32_t len = 0x10;
    static uint32_t size = 4;
    static bool repeat = false;
    
    char *endptr;

    if (*line == ' ') {
        address = strtoul(line, &endptr, 16);
        repeat = false;
        if (*endptr == ' ') {
            len = strtoul(endptr, &endptr, 0);
            if (*endptr == ' ') {
                size = strtoul(endptr, &endptr, 10);
                if ((size !=1) && (size != 2) && (size != 4)) {
                    size = 4;
                }
                if (*endptr == ' ') {
                    repeat = true;
                }
            }
        }
    }

    if (len <= 0) {
        SERIAL.println("Bad number of bytes to dump.");
        return RETURN_FAIL_PRINT_USAGE;
    }

    if (repeat) {
         doRepeat((RepeatFunc)dumpMemory, address, len, size, 0);
    }
    else {
        dumpMemory((uint32_t *)address, len, size);
    }

    return RETURN_SUCCESS;
}

#endif /* DM_CMD */

#if WM_CMD == 1


static int consoleHandler_wm(const char *line)
{
    char *endptr;
    uint32_t word = 0;
    uint32_t *ptr;

    if (*line != ' ') {
        return RETURN_FAIL_PRINT_USAGE;
    }

    ptr = (uint32_t *)strtoul(line, &endptr, 16);

    if (*endptr != ' ') {
        return RETURN_FAIL_PRINT_USAGE;
    }

    while (*endptr)  {
        word = strtol(endptr , &endptr, 16);
        *ptr++ = word;
    }

    return RETURN_SUCCESS;
}

static int consoleHandler_wm1(const char *line)
{
    char *endptr;
    uint8_t word = 0;
    uint8_t *ptr;

    if (*line != ' ') {
        return RETURN_FAIL_PRINT_USAGE;
    }

    ptr = (uint8_t *)strtoul(line, &endptr, 16);

    if (*endptr != ' ') {
        return RETURN_FAIL_PRINT_USAGE;
    }

    while (*endptr)  {
        word = strtol(endptr , &endptr, 16);
        *ptr++ = word;
    }

    return RETURN_SUCCESS;
}

static int consoleHandler_wm2(const char *line)
{
    char *endptr;
    uint16_t word = 0;
    uint16_t *ptr;

    if (*line != ' ') {
        return RETURN_FAIL_PRINT_USAGE;
    }

    ptr = (uint16_t *)strtoul(line, &endptr, 16);

    if (*endptr != ' ') {
        return RETURN_FAIL_PRINT_USAGE;
    }

    while (*endptr)  {
        word = strtol(endptr , &endptr, 16);
        *ptr++ = word;
    }

    return RETURN_SUCCESS;
}

#endif /* WM_CMD */

#if WR_CMD == 1

/*
 * Wire read 'num' bytes from i2c at 'addr'
 */
static void doWrd(uint8_t addr, uint8_t num)
{
    unsigned int i, j;
    uint8_t bytes[32];

    Wire.requestFrom((int) addr, (int)num);
    
    i = 0;
    while (Wire.available()) 
    {
        bytes[i++] = Wire.read();
    }
    
    for (j = 0; j < i; j++) {
        SERIAL.print(bytes[j], 16); SERIAL.print(" ");
    }

    SERIAL.println("");
}

/*
 * Wire read 'num' bytes from 'cfg' register of i2c at 'addr'
 */
static void doWrdC(uint8_t addr, uint8_t cfg, uint8_t num)
{
    unsigned int i, j;
    uint8_t bytes[32];

    /* select internal address to read from */
    Wire.beginTransmission(addr);
    Wire.write(cfg);
    Wire.endTransmission();

    /* start read */
    Wire.requestFrom((int) addr, (int)num);

    i = 0;
    while (Wire.available())
    {
        bytes[i++] = Wire.read();
    }

    for (j = 0; j < i; j++) {
        SERIAL.print(bytes[j], 16); SERIAL.print(" ");
    }

    SERIAL.println("");
}

static int consoleHandler_wrd(const char *line)
{
    if (*line++ != ' ') {
        return RETURN_FAIL_PRINT_USAGE;
    }

    char *endptr = NULL;
    unsigned int i;
    uint8_t bytes[3];

    i = 0;
    endptr = (char *)line;
    do {
        bytes[i] = strtoul(endptr, &endptr, 16);
        if (++i == 3) break;
    }
    while ((*endptr == ' ') && (*(endptr+1) != 0));
    
    if (i < 2) {
        return RETURN_FAIL_PRINT_USAGE;
    }
    
    if (wireBegun == false) {
        Wire.begin();
        wireBegun = true;
    }

    if (i == 3) {  /* read from an internal register */
        if (*endptr == ' ') {
            doRepeat((RepeatFunc)doWrdC, bytes[0], bytes[1], bytes[2], 0);
        }
        else {
            doWrdC(bytes[0], bytes[1], bytes[2]);
        }
    }
    else if (i == 2) { /* read from base address */
        if (*endptr == ' ') {
            doRepeat((RepeatFunc)doWrd, bytes[0], bytes[1], 0, 0);
        }
        else {
            doWrd(bytes[0], bytes[1]);
        }
    }

    return RETURN_SUCCESS;
}

#endif /* WR_CMD */

#if WW_CMD == 1

static int consoleHandler_wwr(const char *line)
{

    if (*line++ != ' ') {
        return RETURN_FAIL_PRINT_USAGE;
    }

    char *endptr = NULL;

    unsigned int i, j;
    uint8_t bytes[33];

    i = 0;
    endptr = (char *)line;
    do {
        bytes[i] = strtoul(endptr, &endptr, 16);
        if (++i == 33) break;
    }
    while (*endptr == ' ');
    
    if (i < 2) {
        return RETURN_FAIL_PRINT_USAGE;
    }
    
    if (wireBegun == false) {
        Wire.begin();
        wireBegun = true;
    }

    Wire.beginTransmission(bytes[0]);

    for (j = 1; j < i; j++) {
        Wire.write(bytes[j]);
    }

     Wire.endTransmission();

    return RETURN_SUCCESS;
}

#endif /* WW_CMD */

#if SPI_CMD == 1

static int consoleHandler_spi(const char *line)
{
    static bool spiBegun = false;

    if (*line++ != ' ') {
        return RETURN_FAIL_PRINT_USAGE;
    }
    char *endptr = NULL;
    uint32_t data;
    uint32_t cs = 0;

    if (spiBegun == false) {
        SPI.begin();
        spiBegun = true;
    }

    data = strtol(line, &endptr, 10);

    if (*endptr == ' ') {
        cs = data;
        data = strtol(endptr, &endptr, 10);
    }

    SERIAL.print("Calling SPI.transfer(");
    if (cs) {
        SERIAL.print(cs);
        SERIAL.print(", ");
    }
    SERIAL.print(data);
    SERIAL.println(").");
    
    if (cs) {
        SPI.transfer(cs, data);
    }
    else {
        SPI.transfer(data);
    }
    return RETURN_SUCCESS;
}

#endif /* SPI_CMD */

#if STATS_CMD == 1

static char *getModeStr(UInt mode)
{
    switch (mode) {
        case Task_Mode_RUNNING:
            return((char *)"RUNNING");
        case Task_Mode_READY:
            return((char *)"READY");
        case Task_Mode_BLOCKED:
            return((char *)"BLOCKED");
        case Task_Mode_TERMINATED:
            return((char *)"TERMINATED");
        case Task_Mode_INACTIVE:
            return((char *)"INACTIVE");
    }
    return(NULL);
}

static void printTaskInfo(Task_Handle task)
{
    Task_Stat taskStat;
    Load_Stat loadStat;
    xdc_String name;
    static char buf[100];
    uint32_t loadInt, loadFrac;

    Task_stat(task, &taskStat);
    Load_getTaskLoad(task, &loadStat);

    if (taskStat.priority == 0) {
        name = (xdc_String)"Idle";
    }
    else {
        name = Task_Handle_name(task);
        if (name[0] == '{') {
            name = (xdc_String)"Unnamed";
        }
    }

    /* only show 1 decimal place of precision */
    /* System_snprintf() does not support %2.1f */ 
    loadInt = 100.0*(float)loadStat.threadTime/(float)loadStat.totalTime;
    loadFrac = 1000.0*(float)loadStat.threadTime/(float)loadStat.totalTime - 10.0*loadInt;
  
    /* use System_snprintf() because it uses 500 fewer bytes of stack than snprintf() */ 
    System_snprintf(buf, sizeof(buf),
          " task: %s/0x%x, pri: %d, stack usage: %d/%d, mode: %s load: %d.%1u",
          name, task, taskStat.priority, taskStat.used, taskStat.stackSize, getModeStr(taskStat.mode),
          loadInt, loadFrac);
    SERIAL.println(buf);
}

static void printUtilization()
{
    UInt i;
    Memory_Stats memStat;
    Hwi_StackInfo hwiStackStat;
    Load_Stat loadStat;
    Task_Handle tsk;
    float idleLoad;
    uint32_t idleLoadInt, idleLoadFrac;

    /* collect current stats */
    Load_update();   

    /* use time NOT spent in idle task for Total CPU Load */ 
    Load_getTaskLoad(Task_getIdleTask(), &loadStat);
    idleLoad = 100.0 - 100.0*(float)loadStat.threadTime/(float)loadStat.totalTime;
    idleLoadInt = idleLoad;
    idleLoadFrac = 10.0*idleLoad - 10.0*idleLoadInt;
 
    SERIAL.write("Total CPU Load: ");
    SERIAL.print(idleLoadInt);
    SERIAL.print(".");
    SERIAL.println(idleLoadFrac);
    SERIAL.println("");
    
    /* collect stats on all statically Created tasks */
    SERIAL.println("Task info:");
    for (i = 0; i < Task_Object_count(); i++) {
        tsk = Task_Object_get(NULL, i);
        printTaskInfo(tsk);
    }

    /* collect stats on all dynamically Created tasks */
    tsk = Task_Object_first();
    while (tsk) {
        printTaskInfo(tsk);
        tsk = Task_Object_next(tsk);
    }
    SERIAL.println("");

    Hwi_getStackInfo(&hwiStackStat, TRUE);
    SERIAL.print("Hwi stack usage: ");
    SERIAL.print(hwiStackStat.hwiStackPeak);
    SERIAL.print("/");
    SERIAL.println(hwiStackStat.hwiStackSize);
    SERIAL.println("");
    
    Memory_getStats(NULL, &memStat);
    SERIAL.print("Heap usage: ");
    SERIAL.print(memStat.totalSize - memStat.totalFreeSize);
    SERIAL.print("/");
    SERIAL.println(memStat.totalSize);
}

static int consoleHandler_stats(const char *line)
{
    if (*line == ' ') {
        doRepeat((RepeatFunc)printUtilization, 0, 0, 0, 0);
    }
    else {
        printUtilization();
    }
    return RETURN_SUCCESS;
}

#endif /* STATS_CMD */

#if PRI_CMD == 1

static int consoleHandler_pri(const char *line)
{
    Task_Handle tsk;
    char *endptr;
    int pri = 3;
    if (*line != ' ') {
        return RETURN_FAIL_PRINT_USAGE;
    }

    tsk = (Task_Handle)strtoul(line, &endptr, 16);
    pri = strtol(endptr, NULL, 10);

    if ((pri < -1) || (pri >= (int)Task_numPriorities)) {
        SERIAL.println("Invalid priority!");
        return RETURN_FAIL_PRINT_USAGE;
    }
    
    Task_setPri(tsk, pri);
    
    return RETURN_SUCCESS;
}

#endif /* PRI_CMD */

#if PI_CMD == 1

static int consoleHandler_pi(const char *line)
{
    uint8_t pin;
    uint8_t state = 1; /* default leading edge is low to high */
    uint32_t timeout = 1000000; /* default timeout is 1 ssecond */
    uint32_t width;
    char *endptr = NULL;
    unsigned int i;
    uint32_t args[4];

    if (*line++ != ' ') {
        return RETURN_FAIL_PRINT_USAGE;
    }

    i = 0;
    endptr = (char *)line;
    do {
        args[i] = strtoul(endptr, &endptr, 10);
        if (++i == 4) break;
    }
    while ((*endptr == ' ') && (*(endptr+1) != 0));

    if (i == 4) {
        return RETURN_FAIL_PRINT_USAGE;
    }

    pin = args[0];
    
    if (i > 1) state = args[1];
    if (i > 2) timeout = args[2]; 

    width = pulseIn(pin, state, timeout);
    SERIAL.print("pulseIn(");
    SERIAL.print(pin);
    SERIAL.print(") = ");
    SERIAL.print(width);
    SERIAL.println(" us");

    return RETURN_SUCCESS;
}

#endif

#if (ARTEST_CMD == 1) || (AWTEST_CMD == 1)

/*
 * The analog MUX selection bits are tied to the LS 4 bits
 * of a PCF8574A on the I2C bus.
 *
 * Two muxes share the channel selection bits.
 *
 * bit 4 = 0 enables lower 16 channels
 * bit 5 = 0 enables upper 16 channels
 */

/* pin to channel map */
uint8_t pin_to_channel[] = {
    255, /* 0 */
    255, /* 1 3.3V */
    0,   /* 2 */
    1,   /* 3 */
    2,   /* 4 */
    3,   /* 5 */
    4,   /* 6 */
    5,   /* 7 */
    6,   /* 8 */
    255, /* 9 SCL */
    255, /* 10 SDA */
    7,   /* 11 */
    8,   /* 12 */
    9,   /* 13 */
    10,  /* 14 */
    11,  /* 15 */
    255, /* 16 reset */
    12,  /* 17 */
    13,  /* 18 */
    14,  /* 19 */
    255, /* 20 GND */
    255, /* 21 5V */
    255, /* 22 GND */
    15,  /* 23 */
    255, /* 24 Common mux signal, DAC output */
    16,  /* 25 */
    17,  /* 26 */
    18,  /* 27 */
    19,  /* 28 */
    20,  /* 29 */
    21,  /* 30 */
    22,  /* 31 */
    23,  /* 32 */
    24,  /* 33 */
    25,  /* 34 */
    26,  /* 35 */
    27,  /* 36 */
    28,  /* 37 */
    29,  /* 38 */
    30,  /* 39 */
    31   /* 40 */
};

#define PCF8574A_I2C_ADDR  (0x38)

static void aMuxChannelEnable(unsigned int pin)
{
    uint8_t chan = pin_to_channel[pin];

    if (chan < 16) {
        chan = 0x20 | (chan & 0x0f); /* enable lower 16 channels */
    }
    else if (chan < 32) {
        chan = 0x10 | (chan & 0x0f); /* enable upper 16 channels */
    }
    else {
        chan = 0x30; /* disable b0th muxes */
    }

    Wire.beginTransmission(PCF8574A_I2C_ADDR);
    Wire.write(chan);
    Wire.endTransmission();
}

#endif

#if ARTEST_CMD == 1

#define MCP4726_I2C_ADDR     (0x62)
#define MCP4726_CMD_WRITEDAC (0x40)
#define MCP4726_CMD_DISABLEDAC (0x46)

static void dacWrite(unsigned int output)
{
    Wire.beginTransmission(MCP4726_I2C_ADDR);
    Wire.write(MCP4726_CMD_WRITEDAC);
    Wire.write(output / 16);         // Upper data bits   (D11.D10.D9.D8.D7.D6.D5.D4)
    Wire.write((output % 16) << 4);  // Lower data bits   (D3.D2.D1.D0.x.x.x.x)
    Wire.endTransmission();
}

/*
 * disable the DAC output
 */
static void disableDac()
{
    Wire.beginTransmission(MCP4726_I2C_ADDR);
    Wire.write(MCP4726_CMD_DISABLEDAC);
    Wire.write(0);
    Wire.write(0);
    Wire.endTransmission();
}

#if MSP432_ARTEST_CMD == 1

#define MAX_DAC_VALUE 2790  /* = 3.40V */

/* Supported Ax pins */
static uint8_t pinIds[] = {
    A0,  A1,  A2,  A3,
    A4,  A5,  A6,  A7,
    A8,  A9,  A10, A11,
    A12, A13, A14, A15
};

#endif  /* MSP432_ARTEST_CMD */

#if CC32XX_ARTEST_CMD == 1

#define MAX_DAC_VALUE 1192 /* = 1.455V */

/* Supported Ax pins */
static uint8_t pinIds[] = {
    A0,  A1,  A2,  A3,
};

#endif  /* CC32XX_ARTEST_CMD */

#if CC26XX_ARTEST_CMD == 1

#define MAX_DAC_VALUE 2708  /* = 3.30V */

/* Supported Ax pins */
static uint8_t pinIds[] = {
    A0,  A1,  A2,  A3,
    A4,  A5,  A6,  A7,
};

#endif  /* CC26XX_ARTEST_CMD */

static int consoleHandler_artest(const char * line)
{
    char *endptr;
    uint32_t dacValue, dacValues[5];
    uint8_t pinIdx, i, pin;
    uint16_t aval[4];
    static char response[80];

    Wire.begin();

    if (*line != ' ') {
        dacValue = MAX_DAC_VALUE;
    }
    else {
        dacValue = strtol(line , &endptr, 10);
        if (dacValue > MAX_DAC_VALUE) {
            dacValue = MAX_DAC_VALUE;
        }
    }

    dacValues[0] = 0;
    dacValues[1] = dacValue/4;
    dacValues[2] = dacValue/2;
    dacValues[3] = dacValue*3/4;
    dacValues[4] = dacValue;

    analogRead(24);

    aMuxChannelEnable(0);

    for (i = 0; i < 5; i++) {

        dacWrite(dacValues[i]);

        SERIAL.print("DAC Value = ");
        SERIAL.println(dacValues[i]);

        for (pinIdx = 0; pinIdx < sizeof(pinIds); pinIdx++ ) {
            pin = pinIds[pinIdx];

            aMuxChannelEnable(pin);

            analogReadResolution(8);
            aval[0] = analogRead(pin);

            analogReadResolution(10);
            aval[1] = analogRead(pin);

            analogReadResolution(12);
            aval[2] = analogRead(pin);

            analogReadResolution(14);
            aval[3] = analogRead(pin);

            analogReadResolution(10);
        
            System_snprintf(response, sizeof(response),
                " pin A%d = %8d  %8d  %8d  %8d", pinIdx, aval[0], aval[1], aval[2], aval[3]);

            SERIAL.println(response);
        }
    }

    return RETURN_SUCCESS;
}

#endif

#if AWTEST_CMD == 1

#if MSP432_AWTEST_CMD == 1

/* Supported analogWrite pins */
static uint8_t awPinIds[] = {
    11,  18,  19,  31,
    32,  34,  36,  39,
    40
};

#endif  /* MSP432_AWTEST_CMD */


#if CC32XX_AWTEST_CMD == 1

/* Supported analogWrite pins */
static uint8_t awPinIds[] = {
    13,  29,  
//  31,  /* GPIO_24 = JTAG TDO */
//  36,  /* GPIO_25 = same as pin 13 */
//  37,  /* GPIO_9 =  same as pin 29 */
//  38,  /* GPIO_24 = JTAG TDO */
//  39,  /* GPIO_10 = I2C */
//  40   /* GPIO_11 = I2C */
};

#endif  /* CC32XX_AWTEST_CMD */


#if CC26XX_AWTEST_CMD == 1
#if defined(BOARD_CC2650_LAUNCHXL)

/* Supported analogWrite pins */
static uint8_t awPinIds[] = {
    2,   5,
    6,   7,   8,   11,
    12,  13,  14,  15,
    18,  19,  23,  25,
    26,  27,  28,  29,
    30,  36,
    37,  39,  40
};

#elif defined(BOARD_CC1310_LAUNCHXL) || defined(BOARD_CC1350_LAUNCHXL)

/* Supported analogWrite pins */
static uint8_t awPinIds[] = {
    2,   5,
    6,   7,   8,   11,
    12,  13,  14,  15,
    18,  19,  23,  25,
    26,  27,  28,
    30,  36,
    37,  39,  40
};

#endif
#endif  /* CC26XX_AWTEST_CMD */

static int consoleHandler_awtest(const char * line)
{
    char *endptr;
    uint8_t pin, pinIdx;
    uint16_t aval[7];
    static char response[80];
    bool doLoop = true;

    if (*line == ' ') {
        pin = strtol(line , &endptr, 10);
        doLoop = false;
    }

    Wire.begin();

    /* turn off the DAC so that mux routes output pins to pin 24 */
    disableDac();

    pinMode(24, INPUT);

    aMuxChannelEnable(2);

    for (pinIdx = 0; pinIdx < sizeof(awPinIds); pinIdx++ ) {

        if (doLoop == true) {
            pin = awPinIds[pinIdx];
        }

        aMuxChannelEnable(pin);

        analogWrite(pin, 1);
        aval[0] = pulseIn(24, 1, 10000);

        analogWrite(pin, 128);
        aval[1] = pulseIn(24, 1, 10000);

        analogWrite(pin, 254);
        aval[2] = pulseIn(24, 1, 10000);

        analogWrite(pin, 0);
        aval[3] = pulseIn(24, 1, 10000);
        aval[4] = digitalRead(24);

        analogWrite(pin, 255);
        aval[5] = pulseIn(24, 1, 10000);
        aval[6] = digitalRead(24);

        /* release PWM resource */
        pinMode(pin, INPUT);

        System_snprintf(response, sizeof(response),
            " pin %2d = %5d  %5d  %5d  %5d  %5d  %5d  %5d", pin,
            aval[0], aval[1], aval[2], aval[3], aval[4], aval[5], aval[6]);

        SERIAL.println(response);

        if (doLoop == false) break;
    }

    return RETURN_SUCCESS;
}

#endif /* AWTEST_CMD */

