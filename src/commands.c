
#include "stm32f0xx.h"
#include "ff.h"
#include "lcd.h"
#include "tty.h"
#include "commands.h"
#include <string.h>
#include <stdio.h>

// Data structure for the mounted file system.
FATFS fs_storage;

typedef union {
    struct {
        unsigned int bisecond:5; // seconds divided by 2
        unsigned int minute:6;
        unsigned int hour:5;
        unsigned int day:5;
        unsigned int month:4;
        unsigned int year:7;
    };
} fattime_t;

// Current time in the FAT file system format.
static fattime_t fattime;

void set_fattime(int year, int month, int day, int hour, int minute, int second)
{
    fattime_t newtime;
    newtime.year = year - 1980;
    newtime.month = month;
    newtime.day = day;
    newtime.hour = hour;
    newtime.minute = minute;
    newtime.bisecond = second/2;
    int len = sizeof newtime;
    memcpy(&fattime, &newtime, len);
}

void advance_fattime(void)
{
    fattime_t newtime = fattime;
    newtime.bisecond += 1;
    if (newtime.bisecond == 30) {
        newtime.bisecond = 0;
        newtime.minute += 1;
    }
    if (newtime.minute == 60) {
        newtime.minute = 0;
        newtime.hour += 1;
    }
    if (newtime.hour == 24) {
        newtime.hour = 0;
        newtime.day += 1;
    }
    if (newtime.month == 2) {
        if (newtime.day >= 29) {
            int year = newtime.year + 1980;
            if ((year % 1000) == 0) { // we have a leap day in 2000
                if (newtime.day > 29) {
                    newtime.day -= 28;
                    newtime.month = 3;
                }
            } else if ((year % 100) == 0) { // no leap day in 2100
                if (newtime.day > 28)
                newtime.day -= 27;
                newtime.month = 3;
            } else if ((year % 4) == 0) { // leap day for other mod 4 years
                if (newtime.day > 29) {
                    newtime.day -= 28;
                    newtime.month = 3;
                }
            }
        }
    } else if (newtime.month == 9 || newtime.month == 4 || newtime.month == 6 || newtime.month == 10) {
        if (newtime.day == 31) {
            newtime.day -= 30;
            newtime.month += 1;
        }
    } else {
        if (newtime.day == 0) { // cannot advance to 32
            newtime.day = 1;
            newtime.month += 1;
        }
    }
    if (newtime.month == 13) {
        newtime.month = 1;
        newtime.year += 1;
    }

    fattime = newtime;
}

uint32_t get_fattime(void)
{
    union FattimeUnion {
        fattime_t time;
        uint32_t value;
    };

    union FattimeUnion u;
    u.time = fattime;
    return u.value;
}



void print_error(FRESULT fr, const char *msg)
{
    const char *errs[] = {
            [FR_OK] = "Success",
            [FR_DISK_ERR] = "Hard error in low-level disk I/O layer",
            [FR_INT_ERR] = "Assertion failed",
            [FR_NOT_READY] = "Physical drive cannot work",
            [FR_NO_FILE] = "File not found",
            [FR_NO_PATH] = "Path not found",
            [FR_INVALID_NAME] = "Path name format invalid",
            [FR_DENIED] = "Permision denied",
            [FR_EXIST] = "Prohibited access",
            [FR_INVALID_OBJECT] = "File or directory object invalid",
            [FR_WRITE_PROTECTED] = "Physical drive is write-protected",
            [FR_INVALID_DRIVE] = "Logical drive number is invalid",
            [FR_NOT_ENABLED] = "Volume has no work area",
            [FR_NO_FILESYSTEM] = "Not a valid FAT volume",
            [FR_MKFS_ABORTED] = "f_mkfs aborted",
            [FR_TIMEOUT] = "Unable to obtain grant for object",
            [FR_LOCKED] = "File locked",
            [FR_NOT_ENOUGH_CORE] = "File name is too large",
            [FR_TOO_MANY_OPEN_FILES] = "Too many open files",
            [FR_INVALID_PARAMETER] = "Invalid parameter",
    };
    if (fr < 0 || fr >= sizeof errs / sizeof errs[0])
        printf("%s: Invalid error\n", msg);
    else
        printf("%s: %s\n", msg, errs[fr]);
}

void append(int argc, char *argv[])
{
    if (argc != 2) {
        printf("Specify only one file name to append to.");
        return;
    }
    FIL fil;        /* File object */
    char line[100]; /* Line buffer */
    FRESULT fr;     /* FatFs return code */
    fr = f_open(&fil, argv[1], FA_WRITE|FA_OPEN_EXISTING|FA_OPEN_APPEND);
    if (fr) {
        print_error(fr, argv[1]);
        return;
    }
    printf("To end append, enter a line with a single '.'\n");
    for(;;) {
        fgets(line, sizeof(line)-1, stdin);
        if (line[0] == '.' && line[1] == '\n')
            break;
        int len = strlen(line);
        if (line[len-1] == '\004')
            len -= 1;
        UINT wlen;
        fr = f_write(&fil, (BYTE*)line, len, &wlen);
        if (fr)
            print_error(fr, argv[1]);
    }
    f_close(&fil);
}

void cat(int argc, char *argv[])
{
    for(int i=1; i<argc; i++) {
        FIL fil;        /* File object */
        char line[100]; /* Line buffer */
        FRESULT fr;     /* FatFs return code */

        /* Open a text file */
        fr = f_open(&fil, argv[i], FA_READ);
        if (fr) {
            print_error(fr, argv[i]);
            return;
        }

        /* Read every line and display it */
        while(f_gets(line, sizeof line, &fil))
            printf(line);
        /* Close the file */
        f_close(&fil);
    }
}

void cd(int argc, char *argv[])
{
    if (argc > 2) {
        printf("Too many arguments.");
        return;
    }
    FRESULT res;
    if (argc == 1) {
        res = f_chdir("/");
        if (res)
            print_error(res, "(default path)");
        return;
    }
    res = f_chdir(argv[1]);
    if (res)
        print_error(res, argv[1]);
}

int to_int(char *start, char *end, int base)
{
    int n = 0;
    for( ; start != end; start++)
        n = n * base + (*start - '0');
    return n;
}

static const char *month_name[] = {
        [1] = "Jan",
        [2] = "Feb",
        [3] = "Mar",
        [4] = "Apr",
        [5] = "May",
        [6] = "Jun",
        [7] = "Jul",
        [8] = "Aug",
        [9] = "Sep",
        [10] = "Oct",
        [11] = "Nov",
        [12] = "Dec",
};

void set_fattime(int,int,int,int,int,int);
void date(int argc, char *argv[])
{
    if (argc == 2) {
        char *d = argv[1];
        if (strlen(d) != 14) {
            printf("Date format: YYYYMMDDHHMMSS\n");
            return;
        }
        for(int i=0; i<14; i++)
            if (d[i] < '0' || d[i] > '9') {
                printf("Date format: YYYMMDDHHMMSS\n");
                return;
            }
        int year = to_int(d, &d[4], 10);
        int month = to_int(&d[4], &d[6], 10);
        int day   = to_int(&d[6], &d[8], 10);
        int hour  = to_int(&d[8], &d[10], 10);
        int minute = to_int(&d[10], &d[12], 10);
        int second = to_int(&d[12], &d[14], 10);
        set_fattime(year, month, day, hour, minute, second);
        return;
    }
    int integer = get_fattime();
    union {
        int integer;
        fattime_t ft;
    } u;
    u.integer = integer;
    fattime_t ft = u.ft;
    int year = ft.year + 1980;
    int month = ft.month;
    printf("%d-%s-%02d %02d:%02d:%02d\n",
            year, month_name[month], ft.day, ft.hour, ft.minute, ft.bisecond*2);
}

void dino(int argc, char *argv[])
{
    const char str[] =
    "   .-~~^-.\n"
    " .'  O    \\\n"
    "(_____,    \\\n"
    " `----.     \\\n"
    "       \\     \\\n"
    "        \\     \\\n"
    "         \\     `.             _ _\n"
    "          \\       ~- _ _ - ~       ~ - .\n"
    "           \\                              ~-.\n"
    "            \\                                `.\n"
    "             \\    /               /       \\    \\\n"
    "              `. |         }     |         }    \\\n"
    "                `|        /      |        /       \\\n"
    "                 |       /       |       /          \\\n"
    "                 |      /`- _ _ _|      /.- ~ ^-.     \\\n"
    "                 |     /         |     /          `.    \\\n"
    "                 |     |         |     |             -.   ` . _ _ _ _ _ _\n"
    "                 |_____|         |_____|                ~ . _ _ _ _ _ _ _ >\n";
    puts(str);
}

void input(int argc, char *argv[])
{
    if (argc != 2) {
        printf("Specify only one file name to create.");
        return;
    }
    FIL fil;        /* File object */
    char line[100]; /* Line buffer */
    FRESULT fr;     /* FatFs return code */
    fr = f_open(&fil, argv[1], FA_WRITE|FA_CREATE_NEW);
    if (fr) {
        print_error(fr, argv[1]);
        return;
    }
    printf("To end input, enter a line with a single '.'\n");
    for(;;) {
        fgets(line, sizeof(line)-1, stdin);
        if (line[0] == '.' && line[1] == '\n')
            break;
        int len = strlen(line);
        if (line[len-1] == '\004')
            len -= 1;
        UINT wlen;
        fr = f_write(&fil, (BYTE*)line, len, &wlen);
        if (fr)
            print_error(fr, argv[1]);
    }
    f_close(&fil);
}

void lcd_init(int argc, char *argv[])
{
    LCD_Setup();
}

void ls(int argc, char *argv[])
{
    FRESULT res;
    DIR dir;
    static FILINFO fno;
    const char *path = "";
    int info = 0;
    int i=1;
    do {
        if (argv[i][0] == '-') {
            for(char *c=&argv[i][1]; *c; c++)
                if (*c == 'l')
                    info=1;
            if (i+1 < argc) {
                i += 1;
                continue;
            }
        } else {
            path = argv[i];
        }

        res = f_opendir(&dir, path);                       /* Open the directory */
        if (res != FR_OK) {
            print_error(res, argv[1]);
            return;
        }
        for (;;) {
            res = f_readdir(&dir, &fno);                   /* Read a directory item */
            if (res != FR_OK || fno.fname[0] == 0) break;  /* Break on error or end of dir */
            if (info) {
                printf("%04d-%s-%02d %02d:%02d:%02d %6ld %c%c%c%c%c ",
                        (fno.fdate >> 9) + 1980,
                        month_name[fno.fdate >> 5 & 15],
                        fno.fdate & 31,
                        fno.ftime >> 11,
                        fno.ftime >> 5 & 63,
                        (fno.ftime & 31) * 2,
                        fno.fsize,
                        (fno.fattrib & AM_DIR) ? 'D' : '-',
                        (fno.fattrib & AM_RDO) ? 'R' : '-',
                        (fno.fattrib & AM_HID) ? 'H' : '-',
                        (fno.fattrib & AM_SYS) ? 'S' : '-',
                        (fno.fattrib & AM_ARC) ? 'A' : '-');
            }
            if (path[0] != '\0')
                printf("%s/%s\n", path, fno.fname);
            else
                printf("%s\n", fno.fname);
        }
        f_closedir(&dir);
        i += 1;
    } while(i<argc);
}

void mkdir(int argc, char *argv[])
{
    for(int i=1; i<argc; i++) {
        FRESULT res = f_mkdir(argv[i]);
        if (res != FR_OK) {
            print_error(res, argv[i]);
            return;
        }
    }
}

void mount(int argc, char *argv[])
{
    FATFS *fs = &fs_storage;
    if (fs->id != 0) {
        print_error(FR_DISK_ERR, "Already mounted.");
        return;
    }
    int res = f_mount(fs, "", 1);
    if (res != FR_OK)
        print_error(res, "Error occurred while mounting");
}

void pwd(int argc, char *argv[])
{
    char line[100];
    FRESULT res = f_getcwd(line, sizeof line);
    if (res != FR_OK)
        print_error(res, "pwd");
    else
        printf("%s\n", line);
}

void rm(int argc, char *argv[])
{
    FRESULT res;
    for(int i=1; i<argc; i++) {
        res = f_unlink(argv[i]);
        if (res != FR_OK)
            print_error(res, argv[i]);
    }
}

void shout(int argc, char *argv[])
{
    char arr[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789abcdefghijklmnopqrstuvwxyz0123456789\n";
    for(int i=0; i<1000; i++)
        for(int c=0; c<sizeof arr; c++)
            putchar(arr[c]);
}

void clear(int argc, char *argv[])
{
    int value = 0;
    if (argc == 2)
        value = strtoul(argv[1], 0, 16);
    LCD_Clear(value);
}

void drawline(int argc, char *argv[])
{
    if (argc != 6) {
        printf("Wrong number of arguments: line x1 y1 x2 y2 color");
        return;
    }
    int x1 = strtoul(argv[1], 0, 10);
    int y1 = strtoul(argv[2], 0, 10);
    int x2 = strtoul(argv[3], 0, 10);
    int y2 = strtoul(argv[4], 0, 10);
    int c = strtoul(argv[5], 0, 16);
    LCD_DrawLine(x1,y1,x2,y2,c);
}

void drawrect(int argc, char *argv[])
{
    if (argc != 6) {
        printf("Wrong number of arguments: drawrect x1 y1 x2 y2 color");
        return;
    }
    int x1 = strtoul(argv[1], 0, 10);
    int y1 = strtoul(argv[2], 0, 10);
    int x2 = strtoul(argv[3], 0, 10);
    int y2 = strtoul(argv[4], 0, 10);
    int c = strtoul(argv[5], 0, 16);
    LCD_DrawRectangle(x1,y1,x2,y2,c);
}

void drawfillrect(int argc, char *argv[])
{
    if (argc != 6) {
        printf("Wrong number of arguments: drawfillrect x1 y1 x2 y2 color");
        return;
    }
    int x1 = strtoul(argv[1], 0, 10);
    int y1 = strtoul(argv[2], 0, 10);
    int x2 = strtoul(argv[3], 0, 10);
    int y2 = strtoul(argv[4], 0, 10);
    int c = strtoul(argv[5], 0, 16);
    LCD_DrawFillRectangle(x1,y1,x2,y2,c);
}
// REPLACE -.-.-.--.-.-.-.--.-.-.-.--
void drawfillcircle(int argc, char *argv[]) {
    if (argc != 6) {
        printf("Wrong number of arguments: drawfillcircle x y radius fill color\n");
        return;
    }
    int x = strtoul(argv[1], 0, 10); // center x-coord
    int y = strtoul(argv[2], 0, 10); // center y-cord
    int radius = strtoul(argv[3], 0, 10); // radius
    int fill = strtoul(argv[4], 0, 10); // 1 for fill, 0 for outline
    int color = strtoul(argv[5], 0, 16); // color hexadecimal

    LCD_Circle(x, y, radius, fill, color);
}

// REPLACE -.-.-.--.-.-.-.--.-.-.-.--

void set_design() {

    
    // Outer Borders
    LCD_DrawFillRectangle(0, 0, 240, 2, 0xffff);
    LCD_DrawFillRectangle(0, 318, 240, 320, 0xffff);
    LCD_DrawFillRectangle(0, 0, 2, 320, 0xffff);
    LCD_DrawFillRectangle(238, 0, 240, 320, 0xffff);

    // Dividing Sections
    LCD_DrawFillRectangle(0, 79, 240, 80, 0xffff);
    LCD_DrawFillRectangle(0, 159, 240, 160, 0xffff);
    LCD_DrawFillRectangle(0, 239, 240, 240, 0xffff);

    // Symbols
    LCD_DrawFillRectangle(20, 20, 25, 60, 0x0f0f);
    LCD_Circle(23, 60, 8, 1, 0x0f0f);

    LCD_DrawFillTriangle(26, 120, 18, 135, 34, 135, 0xff);
    LCD_Circle(26, 140, 8, 1, 0xff);
    LCD_DrawFillTriangle(16, 110, 13, 118, 19, 118, 0xff);
    LCD_Circle(16, 120, 3, 1, 0xff);
    LCD_DrawFillTriangle(35, 103, 30, 111, 39, 111, 0xff);
    LCD_Circle(35, 113, 4, 1, 0xff);

    LCD_DrawFillTriangle(26, 175, 21, 196, 32, 196, 0xff00);
    LCD_DrawFillTriangle(21, 207, 21, 196, 32, 196, 0xff00);
    LCD_DrawFillTriangle(21, 207, 32, 207, 32, 196, 0xff00);
    LCD_DrawFillTriangle(26, 228, 21, 207, 32, 207, 0xff00);

    LCD_DrawFillTriangle(11, 201, 21, 207, 21, 196, 0xff00);
    LCD_DrawFillTriangle(42, 201, 32, 207, 32, 196, 0xff00);

    LCD_Circle(35, 270, 4, 1, 0x500f);
    LCD_Circle(20, 285, 3, 1, 0x500f);
    LCD_Circle(31, 295, 3, 1, 0x500f);
    LCD_DrawFillRectangle(14, 268, 40, 271, 0x500f);
    LCD_DrawFillRectangle(20, 275, 46, 278, 0x500f);
    LCD_DrawFillRectangle(10, 282, 38, 285, 0x500f);
    LCD_DrawFillRectangle(22, 289, 45, 292, 0x500f);
    LCD_DrawFillRectangle(12, 296, 39, 299, 0x500f);
}


void temp_print_values(int temp_val) {

    int temp_val_1 = temp_val / 100;
    int temp_val_2 = ((temp_val / 10) % 10);
    int temp_val_3 = (temp_val % 10);
    int temp_thresholds[] = {10, 15, 20, 25, 30, 35};
    int temp_x_coords[] = {120, 132, 144, 156, 168, 180};
    // uint16_t temp_on_color = 0xf000;
    uint16_t temp_on_color = 0x0f0f;
    uint16_t off_color = 0xffff;

    for (int i = 0; i < 6; i++) {
        uint16_t color = ((temp_val) > temp_thresholds[i]) ? temp_on_color : off_color;
        LCD_DrawFillRectangle(temp_x_coords[i], 20, temp_x_coords[i] + 7, 60, color);
    }

    if(temp_val >= 30) {
        //LCD_Circle(213, 40, 18, 1, 0x0000);
        LCD_Circle(213, 40, 18, 1, 0xf000);
        LCD_DrawFillRectangle(211, 28, 215, 43, 0xffff);
        LCD_Circle(213, 50, 2, 1, 0xffff);
    }
    else {
        LCD_Circle(213, 40, 18, 1, 0xffff);
    }
    
    LCD_DrawFillRectangle(55, 20, 108, 50, 0x0000);
    LCD_DrawChar(60, 35, 0xffff, 0x0000, (temp_val_1 + 48), 12, 1);
    LCD_DrawChar(70, 35, 0xffff, 0x0000, (temp_val_2 + 48), 12, 1);
    LCD_DrawChar(80, 35, 0xffff, 0x0000, (temp_val_3 + 48), 12, 1);
    LCD_DrawChar(100, 35, 0xffff, 0xffff, 67, 12, 1);
}


void hum_print_values(int hum_val) {

    int hum_val_1 = hum_val / 100;
    int hum_val_2 = ((hum_val / 10) % 10);
    int hum_val_3 = (hum_val % 10);
    int hum_thresholds[] = {20, 30, 40, 50, 60, 70};
    int hum_x_coords[] = {120, 132, 144, 156, 168, 180};
    uint16_t hum_on_color = 0xff;
    uint16_t off_color = 0xffff;

    for (int i = 0; i < 6; i++) {
        uint16_t color = ((hum_val) > hum_thresholds[i]) ? hum_on_color : off_color;
        LCD_DrawFillRectangle(hum_x_coords[i], 100, hum_x_coords[i] + 7, 140, color);
    }

    if(hum_val >= 50) {
        // LCD_Circle(213, 120, 18, 1, 0x0000);
        LCD_Circle(213, 120, 18, 1, 0xf000);
        LCD_DrawFillRectangle(211, 110, 215, 125, 0xffff);
        LCD_Circle(213, 130, 2, 1, 0xffff);
    }
    else {
        LCD_Circle(213, 120, 18, 1, 0xffff);
    }
    
    LCD_DrawFillRectangle(55, 100, 108, 130, 0x0000);
    LCD_DrawChar(60, 115, 0xffff, 0x0000, (hum_val_1 + 48), 12, 1);
    LCD_DrawChar(70, 115, 0xffff, 0x0000, (hum_val_2 + 48), 12, 1);
    LCD_DrawChar(80, 115, 0xffff, 0x0000, (hum_val_3 + 48), 12, 1);
    LCD_DrawChar(96, 115, 0xffff, 0xffff, 82, 12, 1);
    LCD_DrawChar(104, 115, 0xffff, 0xffff, 72, 12, 1);
}


void light_print_values(int light_val) {

    int light_val_1 = light_val / 100;
    int light_val_2 = ((light_val / 10) % 10);
    int light_val_3 = (light_val % 10);
    int light_thresholds[] = {1, 3, 5, 7, 9, 10};
    int light_x_coords[] = {120, 132, 144, 156, 168, 180};
    uint16_t light_on_color = 0xff00;
    uint16_t off_color = 0xffff;

    for (int i = 0; i < 6; i++) {
        uint16_t color = ((light_val_2+ 1) > light_thresholds[i]) ? light_on_color : off_color;
        LCD_DrawFillRectangle(light_x_coords[i], 180, light_x_coords[i] + 7, 220, color);
    }

    if(light_val_2 >= 6) {
        LCD_Circle(213, 200, 18, 1, 0xf000);
        LCD_DrawFillRectangle(211, 190, 215, 205, 0xffff);
        LCD_Circle(213, 210, 2, 1, 0xffff);
    }
    else {
        LCD_Circle(213, 200, 18, 1, 0xffff);
    }
    
    LCD_DrawFillRectangle(55, 180, 108, 210, 0x0000);
    LCD_DrawChar(60, 195, 0xffff, 0x0000, (light_val_1 + 48), 12, 1);
    LCD_DrawChar(70, 195, 0xffff, 0x0000, (light_val_2 + 48), 12, 1);
    LCD_DrawChar(80, 195, 0xffff, 0x0000, (light_val_3 + 48), 12, 1);
    LCD_DrawChar(95, 195, 0xffff, 0xffff, 76, 12, 1);
    LCD_DrawChar(104, 195, 0xffff, 0xffff, 73, 12, 1);
}


void AQI_print_values(int AQI_val) {

    int AQI_val_1 = 0;
    int AQI_val_2 = 9 - ((AQI_val / 10) % 10);
    int AQI_val_3 = 9 - (AQI_val % 10);
    int AQI_thresholds[] = {1, 3, 5, 7, 9, 10};
    int AQI_x_coords[] = {120, 132, 144, 156, 168, 180};
    uint16_t AQI_on_color = 0x500f;
    uint16_t off_color = 0xffff;

    for (int i = 0; i < 6; i++) {
        uint16_t color = ((AQI_val_2+ 1) > AQI_thresholds[i]) ? AQI_on_color : off_color;
        LCD_DrawFillRectangle(AQI_x_coords[i], 260, AQI_x_coords[i] + 7, 300, color);
    }

    if(AQI_val_2 >= 9) {
        // LCD_Circle(213, 280, 18, 1, 0xf000);
        LCD_DrawFillRectangle(211, 270, 215, 285, 0xffff);
        LCD_Circle(213, 290, 2, 1, 0xffff);
        LCD_Circle(213, 280, 18, 1, 0xffff);
    }
    else {
        LCD_Circle(213, 280, 18, 1, 0xffff);
    }
    
    LCD_DrawFillRectangle(55, 260, 108, 290, 0x0000);
    LCD_DrawChar(60, 275, 0xffff, 0x0000, (AQI_val_1 + 48), 12, 1);
    LCD_DrawChar(70, 275, 0xffff, 0x0000, (AQI_val_2 + 48), 12, 1);
    LCD_DrawChar(80, 275, 0xffff, 0x0000, (AQI_val_3 + 48), 12, 1);
    LCD_DrawChar(93, 275, 0xffff, 0x0000, 65, 12, 1);
    LCD_DrawChar(100, 275, 0xffff, 0x0000, 81, 12, 1);
    LCD_DrawChar(107, 275, 0xffff, 0x0000, 73, 12, 1);
}



void mines_to_command() {
    int AQI_val;
    int temp_val;
    int hum_val;
    int light_val;

    set_design();

    for(int i = 0; i < 1000; i++) {
        AQI_val = ADC_Read();
        AQI_print_values(AQI_val);


        temp_val = ret_temp();
        temp_print_values(temp_val);

        hum_val = ret_hum();
        hum_print_values(hum_val);

        light_val = Read_ADC();
        light_val = 999 - (light_val * 999 / 4095);
        light_print_values(light_val);

        nano_wait(100000000);
        
    }
    
    
}




void add(int argc, char *argv[])
{
  int sum = 0;
  for(int i=1; i < argc; i++) {
      sum += strtol(argv[i], 0, 0);
  }
  printf("The sum is %d\n", sum);
}

void mul(int argc, char *argv[])
{
  int prod = 1;
  for(int i=1; i < argc; i++) {
    prod *= strtol(argv[i], 0, 0);
  }
  printf("The product is %d\n", prod);
}

void birb(int argc, char *argv[])
{
    const char str[] =
    "   chyuu~ chyuu~ chyuu~"
    "       chyuu~ chyuu~ chyuu~";
    puts(str);
}

struct commands_t cmds[] = {
        { "append", append },
        { "cat", cat },
        { "cd", cd },
        { "date", date },
        { "dino", dino },
        { "input", input },
        { "lcd_init", lcd_init },
        { "ls", ls },
        { "mkdir", mkdir },
        { "mount", mount },
        { "pwd", pwd },
        { "rm", rm },
        { "shout", shout },
        { "clear",    clear },
        { "drawline", drawline },
        { "drawrect", drawrect },
        { "drawfillrect", drawfillrect },
        { "drawfillcircle", drawfillcircle}, // REPLACE
        //{ "drawstring", drawstring}, // REPLACE
        { "set_design", set_design},
        { "AQI_print_values", AQI_print_values},
        { "temp_print_values", temp_print_values},
        { "hum_print_values", hum_print_values},
        { "light_print_values", light_print_values},
        { "mines_to_command", mines_to_command},
        { "add", add},
        { "mul", mul},
        { "birb", birb},
};

// A weak definition that can be overridden by a better one.
// Replace this with your own usercmds entirely.
__attribute((weak)) struct commands_t usercmds[] = {
        { 0, 0 }
};

void exec(int argc, char *argv[])
{
    //for(int i=0; i<argc; i++)
    //    printf("%d: %s\n", i, argv[i]);
    for(int i=0; usercmds[i].cmd != 0; i++)
        if (strcmp(usercmds[i].cmd, argv[0]) == 0) {
            usercmds[i].fn(argc, argv);
            return;
        }
    for(int i=0; i<sizeof cmds/sizeof cmds[0]; i++)
        if (strcmp(cmds[i].cmd, argv[0]) == 0) {
            cmds[i].fn(argc, argv);
            return;
        }
    printf("%s: No such command.\n", argv[0]);
}

void parse_command(char *c)
{
    char *argv[20];
    int argc=0;
    int skipspace=1;
    for(; *c; c++) {
        if (skipspace) {
            if (*c != ' ' && *c != '\t') {
                argv[argc++] = c;
                skipspace = 0;
            }
        } else {
            if (*c == ' ' || *c == '\t') {
                *c = '\0';
                skipspace=1;
            }
        }
    }
    if (argc > 0) {
        argv[argc] = "";
        exec(argc, argv);
    }
}


static void insert_echo_string(const char *s)
{
    // This puts a string into the *input* stream, so it can be edited.
    while(*s)
        insert_echo_char(*s++);
}

void command_shell(void)
{
    printf("\nEnter current "); fflush(stdout);
    insert_echo_string("date 20240213000000");
    
    char line[100];
    fgets(line, 99, stdin);
    line[99] = '\0';
    int len = strlen(line);
    if (line[len-1] == '\n')
        line[len-1] = '\0';
    parse_command(line);

    puts("This is the STM32 command shell.");
    puts("Type 'mount' before trying any file system commands.");
    puts("Type 'lcd_init' before trying any draw commands.");
    for(;;) {
        printf("> ");
        fgets(line, 99, stdin);
        line[99] = '\0';
        len = strlen(line);
        if (line[len-1] == '\n')
            line[len-1] = '\0';
        parse_command(line);
    }
}

void my_command_shell(void)
{
  char line[100];
  int len = strlen(line);
  puts("This is the STM32 command shell.");
  for(;;) {
      printf("> ");
      fgets(line, 99, stdin);
      line[99] = '\0';
      len = strlen(line);
      if (line[len-1] == '\n')
          line[len-1] = '\0';
      parse_command(line);
  }
}
