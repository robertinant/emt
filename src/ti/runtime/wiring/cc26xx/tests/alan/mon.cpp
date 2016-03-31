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
static int consoleHandler_ww(const char *line);
#endif

#if WR_CMD == 1
static int consoleHandler_wr(const char *line);
#endif

#if (WR_CMD == 1) || (WW_CMD == 1)
static bool wireBegun = false;
#endif

#if PRI_CMD == 1
static int consoleHandler_pri(const char *line);
#endif

#if ALOG_CMD == 1
static int consoleHandler_alog(const char *line);
static int consoleHandler_alogs(const char *line);
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
#if SPI_CMD == 1
    GEN_COMMTABLE_ENTRY(spi,     "SPI transfer",                "usage: spi <cs pin> <data>"),
#endif
#if WR_CMD == 1
    GEN_COMMTABLE_ENTRY(wr,      "Wire read",                   "usage: wr <addr>"),
#endif
#if WW_CMD == 1
    GEN_COMMTABLE_ENTRY(ww,      "Wire write",                  "usage: ww <addr> <data> <data> <...>"),
#endif
#if STATS_CMD == 1
    GEN_COMMTABLE_ENTRY(stats,   "Print CPU utlization info",   "usage: stats"),
#endif
    GEN_COMMTABLE_ENTRY(help,    "Get information on commands. Usage: help [command]",  NULL),
    {NULL,NULL,NULL,NULL}   // Indicates end of table
};

void MON_SETUP(void)
{
    SERIAL.begin(115200);
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

static void doWr(uint8_t addr, uint8_t cfg, uint8_t num)
{
    unsigned int i, j;
    uint8_t bytes[32];

    if (wireBegun == false) {
        Wire.begin();
        wireBegun = true;
    }

    Wire.beginTransmission(addr);
    Wire.write(cfg);
    Wire.endTransmission();
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

static int consoleHandler_wr(const char *line)
{
    static bool wireBegun = false;

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
    while (*endptr == ' ');
    
    if (i < 3) {
        return RETURN_FAIL_PRINT_USAGE;
    }
    
    if (wireBegun == false) {
        Wire.begin();
        wireBegun = true;
    }

    if (*endptr == ' ') {
        doRepeat((RepeatFunc)doWr, bytes[0], bytes[1], bytes[2], 0);
    }
    else {
        doWr(bytes[0], bytes[1], bytes[2]);
    }

    return RETURN_SUCCESS;
}

#endif /* WR_CMD */

#if WW_CMD == 1

static int consoleHandler_ww(const char *line)
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

void printUtilization()
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
