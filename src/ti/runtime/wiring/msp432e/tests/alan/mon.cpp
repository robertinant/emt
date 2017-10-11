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
#define AR_CMDS 0    /* analog read cmds */
#define AW_CMDS 1    /* analog write cmds */
#define WR_CMD 1     /* Wire read cmd */
#define WW_CMD 1     /* Wire write cmd */
#define SPI_CMD 1    /* SPI transfer cmd */
#define PRI_CMD 1    /* Set Task priority cmd */
#define STATS_CMD 1  /* CPU and task utilzation stats cmd */
#define ALOG_CMD 0   /* log analog value cmd */
#define PI_CMD 1     /* pulseIn() cmd */
#define ARTEST_CMD 0 /* analogRead() self test */
#define AWTEST_CMD 1 /* analogWrite() self test */
#define DRWTEST_CMD 1 /* digitalRead/Write self test */
#define NVSTEST_CMD 1 /* nvsTest self test */

#if ARTEST_CMD == 1
#if defined(BOARD_CC3200LP) || defined(BOARD_CC3200_LAUNCHXL) || defined(BOARD_CC3220S_LAUNCHXL) || defined(BOARD_CC3220SF_LAUNCHXL)
#define CC32XX_ARTEST_CMD 1
#define MSP432_ARTEST_CMD 0
#define CC26XX_ARTEST_CMD 0
#define MSP432E_ARTEST_CMD 0
#elif defined(BOARD_MSP432LP) || defined(BOARD_MSP_EXP432P401R)
#define CC32XX_ARTEST_CMD 0
#define MSP432_ARTEST_CMD 1
#define CC26XX_ARTEST_CMD 0
#define MSP432E_ARTEST_CMD 0
#elif defined(BOARD_CC2650_LAUNCHXL) || defined(BOARD_CC1310_LAUNCHXL) || defined(BOARD_CC1350_LAUNCHXL)
#define CC32XX_ARTEST_CMD 0
#define MSP432_ARTEST_CMD 0
#define CC26XX_ARTEST_CMD 1
#define MSP432E_ARTEST_CMD 0
#elif defined(BOARD_MSP_EXP432E401Y)
#define CC32XX_ARTEST_CMD 0
#define MSP432_ARTEST_CMD 0
#define CC26XX_ARTEST_CMD 0
#define MSP432E_ARTEST_CMD 0
#endif
#endif

#if AWTEST_CMD == 1
#if defined(BOARD_CC3200LP) || defined(BOARD_CC3200_LAUNCHXL) || defined(BOARD_CC3220S_LAUNCHXL) || defined(BOARD_CC3220SF_LAUNCHXL)
#define CC32XX_AWTEST_CMD 1
#define MSP432_AWTEST_CMD 0
#define CC26XX_AWTEST_CMD 0
#define MSP432E_AWTEST_CMD 0
#elif defined(BOARD_MSP432LP) || defined(BOARD_MSP_EXP432P401R)
#define CC32XX_AWTEST_CMD 0
#define MSP432_AWTEST_CMD 1
#define CC26XX_AWTEST_CMD 0
#define MSP432E_AWTEST_CMD 0
#elif defined(BOARD_CC2650_LAUNCHXL) || defined(BOARD_CC1310_LAUNCHXL) || defined(BOARD_CC1350_LAUNCHXL)
#define CC32XX_AWTEST_CMD 0
#define MSP432_AWTEST_CMD 0
#define CC26XX_AWTEST_CMD 1
#define MSP432E_AWTEST_CMD 0
#elif defined(BOARD_MSP_EXP432E401Y)
#define CC32XX_AWTEST_CMD 0
#define MSP432_AWTEST_CMD 0
#define CC26XX_AWTEST_CMD 0
#define MSP432E_AWTEST_CMD 1
#endif
#endif

#if DRWTEST_CMD == 1
#if defined(BOARD_CC3200LP) || defined(BOARD_CC3200_LAUNCHXL) || defined(BOARD_CC3220S_LAUNCHXL) || defined(BOARD_CC3220SF_LAUNCHXL)
#define CC32XX_DRWTEST_CMD 1
#define MSP432_DRWTEST_CMD 0
#define CC26XX_DRWTEST_CMD 0
#define MSP432E_DRWTEST_CMD 0
#elif defined(BOARD_MSP432LP) || defined(BOARD_MSP_EXP432P401R)
#define CC32XX_DRWTEST_CMD 0
#define MSP432_DRWTEST_CMD 1
#define CC26XX_DRWTEST_CMD 0
#define MSP432E_DRWTEST_CMD 0
#elif defined(BOARD_CC2650_LAUNCHXL) || defined(BOARD_CC1310_LAUNCHXL) || defined(BOARD_CC1350_LAUNCHXL)
#define CC32XX_DRWTEST_CMD 0
#define MSP432_DRWTEST_CMD 0
#define CC26XX_DRWTEST_CMD 1
#define MSP432E_DRWTEST_CMD 0
#elif defined(BOARD_MSP_EXP432E401Y)
#define CC32XX_DRWTEST_CMD 0
#define MSP432_DRWTEST_CMD 0
#define CC26XX_DRWTEST_CMD 0
#define MSP432E_DRWTEST_CMD 1
#endif
#endif

/* self test board's common pin */
#define COMMON_PIN 5

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

#if AR_CMDS == 1
static int consoleHandler_ar(const char *line);
static int consoleHandler_arr(const char *line);
static int consoleHandler_aref(const char *line);
#endif

#if AW_CMDS == 1
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

#if DRWTEST_CMD == 1
static int consoleHandler_drwtest(const char *line);
#endif

#if NVSTEST_CMD == 1
static int consoleHandler_nvstest(const char *line);
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
#if AW_CMDS == 1
    GEN_COMMTABLE_ENTRY(aw,      "analogWrite to pin",          "usage: aw <pin> <value>"),
#endif
#if AR_CMDS == 1
    GEN_COMMTABLE_ENTRY(ar,      "analogRead from pin",         "usage: ar <pin>"),
    GEN_COMMTABLE_ENTRY(aref,    "analogReference",             "usage: aref <1-6>"),
    GEN_COMMTABLE_ENTRY(arr,     "analogReadResolution",        "usage: arr <10,11,12,14>"),
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
#if DRWTEST_CMD == 1
    GEN_COMMTABLE_ENTRY(drwtest, "digitalReadWrite test",       "usage: drwtest"),
#endif
#if NVSTEST_CMD == 1
    GEN_COMMTABLE_ENTRY(nvstest, "NVS test",                    "usage: nvstest"),
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

#if AR_CMDS == 1

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

static int consoleHandler_arr(const char *line)
{
    if (*line++ != ' ') {
        return RETURN_FAIL_PRINT_USAGE;
    }
    char *endptr = NULL;
    uint32_t resolution = strtol(line, &endptr, 10);
    
    analogReadResolution(resolution);

    return RETURN_SUCCESS;
}

static int consoleHandler_aref(const char *line)
{
    if (*line++ != ' ') {
        return RETURN_FAIL_PRINT_USAGE;
    }
    char *endptr = NULL;
    uint32_t ref = strtol(line, &endptr, 10);
    
    analogReference(ref);

    return RETURN_SUCCESS;
}

#endif /* AR_CMDS */

#if AW_CMDS == 1

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

#endif /* AW_CMDS */

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

#if (ARTEST_CMD == 1) || (AWTEST_CMD == 1) || (DRWTEST_CMD == 1)

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
#define PCF8574T_I2C_ADDR  (0x20)

static uint8_t pcfAddress = 0;

static bool findPcf(void)
{
    uint8_t result;

    if (pcfAddress != 0) return (true);

    /* write 0x3f to the 'A' address */
    Wire.beginTransmission(PCF8574A_I2C_ADDR);
    Wire.write(0x3f);
    Wire.endTransmission();

    delay(1);

    /* write 0x1f to the 'T' address */
    Wire.beginTransmission(PCF8574T_I2C_ADDR);
    Wire.write(0x1f);
    Wire.endTransmission();

    delay(1);

    Wire.requestFrom((int)PCF8574A_I2C_ADDR, 1);

    /* read the 'A' address */
    while (Wire.available())
    {
        result = Wire.read();
    }
  
    /* if read = written, PCF device is 'A' type */ 
    if (result == 0x3f) {
        pcfAddress = PCF8574A_I2C_ADDR;
        return (true);
    }

    Wire.requestFrom((int)PCF8574T_I2C_ADDR, 1);

    /* read the 'T' address */
    while (Wire.available())
    {
        result = Wire.read();
    }


    /* if read = written, PCF device is 'T' type */ 
    if (result == 0x1f) {
        pcfAddress = PCF8574T_I2C_ADDR;
        return (true);
    }

    /* use default of A if search fails */
    pcfAddress = PCF8574A_I2C_ADDR;

    return (false);
}

static void aMuxChannelEnable(unsigned int pin)
{
    uint8_t chan = pin_to_channel[pin];

    findPcf();

    if (chan < 16) {
        chan = 0x20 | (chan & 0x0f); /* enable lower 16 channels */
    }
    else if (chan < 32) {
        chan = 0x10 | (chan & 0x0f); /* enable upper 16 channels */
    }
    else {
        chan = 0x30; /* disable b0th muxes */
    }

    Wire.beginTransmission(pcfAddress);
    Wire.write(chan);
    Wire.endTransmission();

    /* wait for the dust to settle */
    delay(1);
}

#endif

#define MCP4726_I2C_ADDR     (0x62)
#define MCP4726_CMD_WRITEDAC (0x40)
#define MCP4726_CMD_DISABLEDAC (0x46)

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

#if ARTEST_CMD == 1

static void dacWrite(unsigned int output)
{
    Wire.beginTransmission(MCP4726_I2C_ADDR);
    Wire.write(MCP4726_CMD_WRITEDAC);
    Wire.write(output / 16);         // Upper data bits   (D11.D10.D9.D8.D7.D6.D5.D4)
    Wire.write((output % 16) << 4);  // Lower data bits   (D3.D2.D1.D0.x.x.x.x)
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

#define MAX_DAC_VALUE 2790 /* = 3.4V */

#if SERIAL == Serial1

/* Supported Ax pins */
static uint8_t pinIds[] = {
    A1,  A2,  A3,
};

#else

/* Supported Ax pins */
static uint8_t pinIds[] = {
    A1,  A2,  A3,
};

#endif

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
    uint32_t maxDacValue, dacValues[5];
    uint8_t pinIdx, i, pin;
    uint16_t aval[4];
    static char response[80];

    Wire.begin();

    if (*line != ' ') {
        maxDacValue = MAX_DAC_VALUE;
    }
    else {
        maxDacValue = strtol(line , &endptr, 10);
        if (maxDacValue > MAX_DAC_VALUE) {
            maxDacValue = MAX_DAC_VALUE;
        }
    }

    dacValues[0] = 0;
    dacValues[1] = maxDacValue/4;
    dacValues[2] = maxDacValue/2;
    dacValues[3] = maxDacValue*3/4;
    dacValues[4] = maxDacValue;

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
                " pin %2d = %8d  %8d  %8d  %8d", pin, aval[0], aval[1], aval[2], aval[3]);

            SERIAL.println(response);
        }
    }

    /* turn off DAC */
    disableDac();

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
    18,  19,  23,  24,
    25,  26,  27,  28,
    29,
    30,  36,
    37,  39,  40
};

#elif defined(BOARD_CC1310_LAUNCHXL) || defined(BOARD_CC1350_LAUNCHXL)

/* Supported analogWrite pins */
static uint8_t awPinIds[] = {
    2,   5,
    6,   7,   8,   11,
    12,  13,  14,  15,
    18,  19,  23,  24,
    25,  26,  27,  28,
    30,  36,
    37,  39,  40
};

#endif
#endif  /* CC26XX_AWTEST_CMD */

#if MSP432E_AWTEST_CMD == 1

/* Supported analogWrite pins */
static uint8_t awPinIds[] = {
    37, 38, 39, 40
};

#endif  /* MSP432E_AWTEST_CMD */

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

    pinMode(COMMON_PIN, INPUT);

    aMuxChannelEnable(2);

    System_snprintf(response, sizeof(response),
        "          %5d  %5d  %5d  %5d  %5d  %5d  %5d", 
         8, 1024, 2034, 0, 0, 0, 1);

    SERIAL.println(response);

    for (pinIdx = 0; pinIdx < sizeof(awPinIds); pinIdx++ ) {

        if (doLoop == true) {
            pin = awPinIds[pinIdx];
        }

        /* can't perform analogWrite() and pulseIn() on the same pin */
        if (pin == COMMON_PIN) continue;

        aMuxChannelEnable(pin);

        analogWrite(pin, 1);
        aval[0] = pulseIn(COMMON_PIN, 1, 10000);

        analogWrite(pin, 128);
        aval[1] = pulseIn(COMMON_PIN, 1, 10000);

        analogWrite(pin, 254);
        aval[2] = pulseIn(COMMON_PIN, 1, 10000);

        analogWrite(pin, 0);
        aval[3] = pulseIn(COMMON_PIN, 1, 10000);
        aval[4] = digitalRead(COMMON_PIN);

        analogWrite(pin, 255);
        aval[5] = pulseIn(COMMON_PIN, 1, 10000);
        aval[6] = digitalRead(COMMON_PIN);

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


#if DRWTEST_CMD == 1

#if MSP432_DRWTEST_CMD == 1

/* Supported digital pins */
static uint8_t drwPinIds[] = {
    2, 5, 6, 7, 8,
   11, 12, 13, 14, 15, 17, 18, 19,
   23, 24, 25, 26, 27, 28, 29, 30,
   31, 32, 33, 34, 35, 36, 37, 38, 39, 40
};

#endif  /* MSP432_DRWTEST_CMD */


#if CC32XX_DRWTEST_CMD == 1

/* Supported digital pins */
static uint8_t drwPinIds[] = {
    3,  4,  5, 7, 8,
    11, /* 12, 13, */ 14, 15, /* 17, */18, 19,
    27, 28, 29, 30,
    /* 31, 32 */
};

#endif  /* CC32XX_DRWTEST_CMD */


#if CC26XX_DRWTEST_CMD == 1

#if defined(BOARD_CC2650_LAUNCHXL)

/* Supported digital pins */
static uint8_t drwPinIds[] = {
    2,   5,
    6,   7,   8,   11,
    12,  13,  14,  15,
    18,  19,  23,  24,
    25,  26,  27,  28,
    30, 
    36,  37,  38,  39,
    40
};

#elif defined(BOARD_CC1310_LAUNCHXL) || defined(BOARD_CC1350_LAUNCHXL)

/* Supported digital pins */
static uint8_t drwPinIds[] = {
    2,   5,
    6,   7,   8,   11,
    12,  13,  14,  15,
    18,  19,  23,  24,
    25,  26,  27,  28,
    30,
    36,  37,  38,  39,
    40
};

#endif
#endif  /* CC26XX_DRWTEST_CMD */

#if MSP432E_DRWTEST_CMD == 1

/* Supported digital pins */
static uint8_t drwPinIds[] = {
        2, 5, 6, 7, 8,
   11, 12, 13, 14, 15, 17, 18, 19,
   23, 24, 25, 26, 27, 28, 29, 30,
   31, 32, 33, 34, 35, 36, 37, 38, 39, 40
};

#endif  /* MSP432_DRWTEST_CMD */

static int consoleHandler_drwtest(const char * line)
{
    char *endptr;
    uint8_t pin, pinIdx;
    uint16_t dval[4];
    static char response[80];
    bool doLoop = true;

    if (*line == ' ') {
        pin = strtol(line , &endptr, 10);
        doLoop = false;
    }

    Wire.begin();

    /* turn off the DAC so that mux routes output pins to pin 23 */
    disableDac();

    pinMode(COMMON_PIN, INPUT);

    pinMode(24, INPUT);

    aMuxChannelEnable(2);

    System_snprintf(response, sizeof(response),
        "             w0     w1     r0     r1");
    SERIAL.println(response);

    for (pinIdx = 0; pinIdx < sizeof(drwPinIds); pinIdx++ ) {

        if (doLoop == true) {
            pin = drwPinIds[pinIdx];
        }

        aMuxChannelEnable(pin);

        /* digitalWrite test */
        pinMode(pin, OUTPUT);

        digitalWrite(pin, 0);
        dval[0] = digitalRead(COMMON_PIN);

        digitalWrite(pin, 1);
        dval[1] = digitalRead(COMMON_PIN);

        /* digitalRead test */
        pinMode(pin, INPUT);
        
        digitalWrite(COMMON_PIN, 0);
        dval[2] = digitalRead(pin);

        digitalWrite(COMMON_PIN, 1);
        dval[3] = digitalRead(pin);

        /* release PWM resource */
        pinMode(pin, INPUT);

        System_snprintf(response, sizeof(response),
            " pin %2d = %5d  %5d  %5d  %5d", pin,
            dval[0], dval[1], dval[2], dval[3]);

        SERIAL.println(response);

        if (doLoop == false) break;
    }

    return RETURN_SUCCESS;
}

#endif /* DRWTEST_CMD */

#if NVSTEST_CMD
#include  <ti/drivers/NVS.h>

static int consoleHandler_nvstest(const char *line)
{
    NVS_Handle handle;
    NVS_Params nvsParams;
    NVS_Attrs nvsAttrs;
    int_fast16_t status;
    uint32_t index = 0;
    char *endptr = NULL;
    char buffer[32];

    if (*line++ == ' ') {
        endptr = (char *)line;
        index = strtoul(endptr, &endptr, 10);
    }

    NVS_Params_init(&nvsParams);

    handle = NVS_open(index, &nvsParams);

    if (handle == NULL) {
        Serial.print("open failure: ");
        Serial.println(status, 16);
        return RETURN_SUCCESS;
    }

    NVS_getAttrs(handle, &nvsAttrs);

    Serial.print("region base = ");
    Serial.println((size_t)nvsAttrs.regionBase, 16);
    Serial.print("region size = ");
    Serial.println(nvsAttrs.regionSize, 16);

    status = NVS_erase(handle, 0, nvsAttrs.sectorSize);

    if (status != NVS_STATUS_SUCCESS) {
	Serial.print("Erase failure: ");
        Serial.println(status, 16);
        NVS_close(handle);
        return (RETURN_SUCCESS);
    }

    /* write a string */
    status = NVS_write(handle, 0, (void *)"Energia is easy to use!",
                       strlen("energia is easy to use!") + 1,
                       NVS_WRITE_PRE_VERIFY | NVS_WRITE_POST_VERIFY );

    if (status != NVS_STATUS_SUCCESS) {
        Serial.print("NVS_write failed: ");
        Serial.println(status, 16);
        NVS_close(handle);
        return (RETURN_SUCCESS);
    }

    status = NVS_read(handle, 0, buffer,
                      strlen("Energia is easy to use!") + 1);

    if (status != NVS_STATUS_SUCCESS) {
        Serial.print("NVS_read failed: ");
        Serial.println(status, 16);
        NVS_close(handle);
        return (RETURN_SUCCESS);
    }

    Serial.println(buffer); 

    NVS_close(handle);

    return (RETURN_SUCCESS);
}
#endif
