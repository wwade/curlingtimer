#include <Arduino.h>
#include <LiquidCrystal.h>
#include <EEPROM.h>
#include <avr/wdt.h>
#include <stddef.h>

#define ENABLE_PROFILING
// #define EE_DEBUG
 #define DEBUG_MEASUREMENT


enum {
    lcd_button_NONE,
    lcd_button_RIGHT = 50,
    lcd_button_UP = 195,
    lcd_button_DOWN = 380,
    lcd_button_LEFT = 555,
    lcd_button_SELECT = 790,
};

enum {
    prog_state_main,
    prog_state_menu,
    prog_state_NUM,
};

enum {
    sensor_NEAR = 4,
    sensor_HOG = 3,
};


#define DBG(args...) ({ Serial.print(__LINE__); Serial.print(": "); Serial.println(args); })
#define DBG_D(x, args...) ({ Serial.print(__LINE__); Serial.print(" "); Serial.print(x); Serial.print(": "); Serial.println(args); })
enum { LCD_ROWS = 2, LCD_COLUMNS = 16 };
static LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

static  char lcd_clear_row[LCD_COLUMNS+1];
static void soft_clear(void)
{
    lcd.setCursor(0,1);
    lcd.print(lcd_clear_row);
    lcd.setCursor(0,1);
}

struct cfg {
    unsigned int ver_info;
    unsigned int threshold;
    unsigned int min_time;
    unsigned int max_time;
    unsigned char run_mode;
};

enum {
    exception_marker_stop = 0x55,
    exception_marker_more = 0xff,
};

struct exception {
    unsigned long ex_millis;
    int ex_line;
    unsigned long ex_a;
    unsigned long ex_b;
    unsigned char marker;
};

struct ee_data {
    struct cfg cfg;

    unsigned char exception_marker;
    char exceptions[0];
};

enum {
    ee_addr_max = 1024,
};
static struct cfg cfg = {
    /* version     */ 0x201,
    /* threshold   */ 150,
    /* min_time    */ 500,
    /* max_time    */ 6000,
    /* run_mode    */ 1,
};

static void EEWRITE(size_t addr, unsigned char val)
{
#ifdef EE_DEBUG
    Serial.print("EE WRITE ");
    Serial.print(addr);
    Serial.print(" = ");
    Serial.println(val, HEX);
#endif
    EEPROM.write(addr, val);
}

static unsigned char EEREAD(size_t addr)
{
#ifndef EE_DEBUG
    return EEPROM.read(addr);
#else
    unsigned char ret = EEPROM.read(addr);
    Serial.print("EE READ ");
    Serial.print(addr);
    Serial.print(" => ");
    Serial.println(ret, HEX);
    return ret;
#endif
}

static void print_config(const char *desc, struct cfg *ptr)
{
    Serial.print(desc);
    Serial.println(" config:");
    Serial.print("Config version: ");
    Serial.println(ptr->ver_info, HEX);
    Serial.print("Threshold: ");
    Serial.println(ptr->threshold);
    Serial.print("Min Time: ");
    Serial.println(ptr->min_time);
    Serial.print("Max Time: ");
    Serial.println(ptr->max_time);
    Serial.println("");
}

static void save_config(void)
{
    int i;
    for (i = 0; i < sizeof cfg; i++)
    {
        EEWRITE(i, ((unsigned char*)&cfg)[i]);
    }
}

static void load_config(void)
{
    int i;
    struct cfg tmp;
    for (i = 0; i < sizeof cfg; i++)
    {
        ((unsigned char*)(&tmp))[i] = EEREAD(i);
    }
    print_config("Default", &cfg);
    print_config("EEPROM", &tmp);
    if (tmp.ver_info == cfg.ver_info)
    {
        memcpy(&cfg, &tmp, sizeof cfg);
    }
    else
    {
        save_config();
        EEWRITE(offsetof(struct ee_data, exception_marker),
                exception_marker_stop);
    }
    print_config("Using", &cfg);
}


const char *e_div = "==============================================";
static size_t ex_ptr;

static void exception_print(struct exception *exc)
{
    Serial.print("    millis: "); Serial.println(exc->ex_millis);
    Serial.print("    line:   "); Serial.println(exc->ex_line);
    Serial.print("    A:      "); Serial.println(exc->ex_a);
    Serial.print("    B:      "); Serial.println(exc->ex_b);
}

static void exceptions_print(void)
{
    struct exception exc;
    unsigned int iter;
    size_t addr;

    ex_ptr = offsetof(struct ee_data, exception_marker);

    exc.marker = EEREAD(ex_ptr);
    if (exc.marker != exception_marker_more)
    {
        Serial.println("(no exceptions)");
        return;
    }

    Serial.println(e_div);
    Serial.println("Exceptions:");
    addr = offsetof(struct ee_data, exceptions);
    while (exc.marker == exception_marker_more && addr < ee_addr_max)
    {
        Serial.println(e_div);
        for (iter = 0; iter < sizeof exc; iter++)
        {
            ((unsigned char*)(&exc))[iter] = EEREAD(addr+iter);
        }
        ex_ptr = addr + offsetof(struct exception, marker);
        addr = addr + iter;
        exception_print(&exc);
    }
    Serial.println(e_div);
}

static int exceptions_clear(void)
{
    ex_ptr = offsetof(struct ee_data, exception_marker);
    if (EEREAD(ex_ptr) == exception_marker_stop)
        return 0;
    EEWRITE(ex_ptr, exception_marker_stop);
    return 1;
}

#define EXCEPT(args...) exception(__FUNCTION__, __LINE__, args)
static void exception(const char *func, int line, int a, int b)
{
    int now;
    int i;
    struct exception e_new = {
        millis(),
        line,
        a,
        b,
        exception_marker_stop,
    };

    now = millis();

    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(func);
    lcd.setCursor(0,1);
    lcd.print(line);
    lcd.print(" ");
    lcd.print(a);
    lcd.print(" ");
    lcd.print(b);

    if (EEREAD(ex_ptr) != exception_marker_stop)
    {
        DBG("CLEAR");
        exceptions_clear();
    }

    exception_print(&e_new);
    /* Store exception in EEPROM */
    for (i = 0; i < sizeof e_new; i++)
    {
        EEWRITE(i+1+ex_ptr, ((unsigned char*)&e_new)[i]);
    }
    EEWRITE(ex_ptr, exception_marker_more);

    /* Sleep a while */
    while (millis() - now < 10000)
    {
    }
    cli();
    wdt_enable(WDTO_15MS);
    while(1);
}



static int lcd_get_button_from_value(int adc_key_in)
{
    if (adc_key_in < lcd_button_RIGHT)
        return lcd_button_RIGHT; 
    else if (adc_key_in < lcd_button_UP)
        return lcd_button_UP;
    else if (adc_key_in < lcd_button_DOWN)
        return lcd_button_DOWN;
    else if (adc_key_in < lcd_button_LEFT)
        return lcd_button_LEFT;
    else if (adc_key_in < lcd_button_SELECT)
        return lcd_button_SELECT;  
    else
        return lcd_button_NONE;
}

static int lcd_get_button(void)
{
    return lcd_get_button_from_value(analogRead(0));
}


void main_setup(void)
{
    DBG(__FUNCTION__);
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Menu: Select");

    soft_clear();
}

void setup(void)
{
    memset(lcd_clear_row, ' ', sizeof(lcd_clear_row) - 1);
    Serial.begin(9600);
    lcd.begin(16, 2);

    load_config();
    exceptions_print();

    main_setup();
}

/*
 * Reading split times:
 * First, wait for sensor 1 to trigger.  After sensor 1, wait for sensor 2.
 * If sensor 2 is not detected within 5 seconds, reset and wait for sensor 1.
 */
/*
 * event states:
 * 0 - no rock
 * 1 - rock at back line
 * 2 - rock before hog
 * 3 - rock at hog
 */

enum { LIGHT_TO_DARK = 1, DARK_TO_LIGHT = 0, };
static int read_sensor_for_state(int cur_state, int *dir)
{
    const int cur_sensor[4] = {
        sensor_NEAR, sensor_NEAR,
        sensor_HOG, sensor_HOG,
    };
    const int state_dir[4] = {
        LIGHT_TO_DARK, DARK_TO_LIGHT,
        LIGHT_TO_DARK, DARK_TO_LIGHT,
    };
    if (cur_state < 0 || cur_state >= 4)
        EXCEPT(cur_state, 0);
    *dir = state_dir[cur_state];
    return analogRead(cur_sensor[cur_state]);
}

enum {
    EVT_TIMEOUT = 0,
    EVT_NOW = 1,
};

static unsigned long max_loop;
static unsigned long min_loop = (unsigned long)(-1);

static int next_event(int cur_state, unsigned long fail_time, unsigned long *etime, int *val)
{
    int adc_val;
    int cmp_val;
    int delta;
    int dir;
    unsigned long event_time;
#ifdef ENABLE_PROFILING
    unsigned long a;
    unsigned long b;
    unsigned long diff;
    float avg = 0;
    const int avg_num = 3000;
    int adc_idx = 0;
#endif

    cmp_val = read_sensor_for_state(cur_state, &dir);
    a = 0;
    do
    {
#ifdef ENABLE_PROFILING
        b = micros();
        if (a != 0)
        {
            diff = (b-a);
            if (diff < min_loop)
                min_loop = diff;
            if (diff > max_loop)
                max_loop = diff;
            avg += (float)(diff) / avg_num;
            adc_idx += 1;
            if (adc_idx == avg_num)
            {
                Serial.print("Average/min/max loop time (microseconds): ");
                Serial.print(avg, 3);
                Serial.print(" / ");
                Serial.print(min_loop);
                Serial.print(" / ");
                Serial.println(max_loop);
                adc_idx = 0;
                avg = 0;
            }
            a = b = micros();
        }
        else
        {
            a = b;
        }
#endif
        adc_val = read_sensor_for_state(cur_state, &dir);
        event_time = millis();
        if (dir == 1)
        {
            if (adc_val > cmp_val)
                delta = adc_val - cmp_val;
            else
                delta = 0;
        }
        else
        {
            if (adc_val < cmp_val)
                delta = cmp_val - adc_val;
            else
                delta = 0;
        }
        if (delta > cfg.threshold)
        {
            *etime = event_time;
            *val = adc_val;
            return EVT_NOW;
        }
    } while(millis() < fail_time);

    return EVT_TIMEOUT;
}

/*
 *
 */

static int main_loop_timeout(void)
{
    if (lcd_get_button() == lcd_button_SELECT)
        return prog_state_menu;
    else
        return prog_state_main;
}

static int main_loop(void)
{
    int evt;
    int iter;
    unsigned long events[4];
    int vals[4];
    float speed;
    const unsigned long timeout[4] = {
        500,
        5000,
        5000,
        5000,
    };
    const char msg[4] = {
        '1', '2', '3', '4',
    };

    for (iter = 0; iter < 4; iter++)
    {
        lcd.setCursor(15, 1);
        lcd.print(msg[iter]);
        evt = next_event(iter, millis() + timeout[iter], events + iter, vals + iter);
        if (evt == EVT_TIMEOUT)
        {
            if (iter > 0)
            {
                soft_clear();
                lcd.print("(TIMEOUT)");
            }
            return main_loop_timeout();
        }
    }

    Serial.println(e_div);

    soft_clear();
    for (iter = 0; iter < 4; iter++)
    {
        Serial.print("IDX ");
        Serial.print(iter);
        Serial.print(": ");
        Serial.print(vals[iter]);
        if (iter > 0)
        {
            Serial.print(", ");
            Serial.print(events[iter]-events[iter-1]);
            Serial.print(", ");
            Serial.println(events[iter]-events[0]);
        }
        else
        {
            Serial.println("");
        }
    }

    while (millis() - events[4] < cfg.min_time)
    {
        if (lcd_get_button() == lcd_button_SELECT)
        {
            return prog_state_menu;
        }
    }

    soft_clear();
    lcd.print((float)(events[2] - events[0])/1000.0, 2);
    lcd.print(" ");

    speed = 3048.0 / (float)(events[3]-events[2]);
    lcd.print(speed, 2);
    lcd.print("m/s");

    if (cfg.run_mode == 0)
    {
        while (millis() - events[3] < 2500)
        {
        }
        return prog_state_menu;
    }
    return prog_state_NUM;
}

/*
 *
 */

enum sb {
    state_start,
    state_error,

    state_s,
    state_sh,

    state_c,
    state_cl,
    state_clr,

    state_e,
    state_ex,

    state_d,
    state_du,
    state_dum,
    state_dump,

    state_m,
    state_mo,
    state_mod,
    state_mode,
} menu_state;


static void serial_menu_out(void)
{
    Serial.println("");
    Serial.println("Menu Mode:");
    Serial.println("  sh        show exceptions");
    Serial.println("  clr       clear exceptions");
    Serial.println("  ex        test exception");
    Serial.println("  dump      dump EEPROM data");
    Serial.print  ("  mode      change startup mode [");
    if (cfg.run_mode)
        Serial.print("run");
    else
        Serial.print("developer");
    Serial.println("]");
    Serial.println("");
    Serial.print("> ");
}

static void menu_enter(void)
{
    DBG(__FUNCTION__);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("MENU");
    serial_menu_out();
    menu_state = state_start;
    while (lcd_get_button() != lcd_button_NONE) { }
}

static void menu_leave(void)
{
    while (lcd_get_button() != lcd_button_NONE) { }
}

static inline int show_exceptions(void)
{
    Serial.println("> Show exceptions");
    exceptions_print();
}

static inline int clear_exceptions(void)
{
    Serial.println("> Clear exceptions");
    if (exceptions_clear())
        Serial.println("-> Cleared");
    else
        Serial.println("-> No exceptions to clear");
}

static inline int test_exception(void)
{
    Serial.println("> Test exception");
    EXCEPT(ex_ptr, millis());
}

static inline int dump_ee_data(void)
{
    int idx;
    char ad[9];
    unsigned char d;
    Serial.println("> Dump EEPROM data");

    for (idx = 0; idx < ee_addr_max; idx++)
    {
        if (idx % 16 == 0)
        {
            snprintf(ad, sizeof ad, "%07x:", idx);
            Serial.println("");
            Serial.print(ad);
        }
        if (idx % 2 == 0)
        {
            Serial.print(" ");
        }
        d = EEPROM.read(idx);
        snprintf(ad, sizeof ad, "%02hhx", d);
        Serial.print(ad);
    }
    Serial.println("");
}

static int change_run_mode(void)
{
    Serial.print("> Change startup mode to ");
    if (cfg.run_mode)
    {
        cfg.run_mode = 0;
        Serial.println("[developer]");
        save_config();
    }
    else
    {
        cfg.run_mode = 1;
        Serial.println("[run]");
        save_config();
    }
}

static int menu_loop(void)
{

    static unsigned long last_time;
    unsigned long now;
    int bytes;
    char sb;
    char extra;

    if (lcd_get_button() == lcd_button_SELECT)
        return prog_state_main;

    now = millis();
    if (now - last_time > 500)
    {
        last_time = now;
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Near: ");
        lcd.print(analogRead(sensor_NEAR));
        lcd.setCursor(0,1);
        lcd.print("Hog:  ");
        lcd.print(analogRead(sensor_HOG));
    }

    bytes = Serial.available();
    extra = 0;

    while (bytes-- > 0)
    {
        if (extra) {
            sb = extra;
            extra = 0;
        } else {
            sb = Serial.read();
        }
        Serial.print(sb);
        switch (menu_state) {
            case state_start:
                switch (sb) {
                    case 'c': case 'C':
                        menu_state = state_c;
                        break;
                    case 's': case 'S':
                        menu_state = state_s;
                        break;
                    case 'e': case 'E':
                        menu_state = state_e;
                        break;
                    case 'd': case 'D':
                        menu_state = state_d;
                        break;
                    case 'm': case 'M':
                        menu_state = state_m;
                        break;
                    case '\r': case '\n': case ' ':
                        break;
                    default:
                        menu_state = state_error;
                        break;
                }
                break;

            case state_error:
                switch (sb) {
                    case '\r': case '\n': case ' ':
                        Serial.println("\n>> Command error");
                        serial_menu_out();
                        menu_state = state_start;
                        break;
                }
                break;

            case state_s:
                switch (sb) {
                    case 'h': case 'H':
                        menu_state = state_sh;
                        break;
                    default:
                        bytes += 1;
                        extra = sb;
                        menu_state = state_error;
                        break;
                }
                break;

            case state_e:
                switch (sb) {
                    case 'x': case 'X':
                        menu_state = state_ex;
                        break;
                    default:
                        bytes += 1;
                        extra = sb;
                        menu_state = state_error;
                        break;
                }
                break;

            case state_c:
                switch (sb) {
                    case 'l': case 'L':
                        menu_state = state_cl;
                        break;
                    default:
                        bytes += 1;
                        extra = sb;
                        menu_state = state_error;
                        break;
                }
                break;

            case state_cl:
                switch (sb) {
                    case 'r': case 'R':
                        menu_state = state_clr;
                        break;
                    default:
                        bytes += 1;
                        extra = sb;
                        menu_state = state_error;
                        break;
                }
                break;

            case state_d:
                switch (sb) {
                    case 'u': case 'U':
                        menu_state = state_du;
                        break;
                    default:
                        bytes += 1;
                        extra = sb;
                        menu_state = state_error;
                        break;
                }
                break;
            case state_du:
                switch (sb) {
                    case 'm': case 'M':
                        menu_state = state_dum;
                        break;
                    default:
                        bytes += 1;
                        extra = sb;
                        menu_state = state_error;
                        break;
                }
                break;
            case state_dum:
                switch (sb) {
                    case 'p': case 'P':
                        menu_state = state_dump;
                        break;
                    default:
                        bytes += 1;
                        extra = sb;
                        menu_state = state_error;
                        break;
                }
                break;

            case state_m:
                switch (sb) {
                    case 'o': case 'O':
                        menu_state = state_mo;
                        break;
                    default:
                        bytes += 1;
                        extra = sb;
                        menu_state = state_error;
                        break;
                }
                break;
            case state_mo:
                switch (sb) {
                    case 'd': case 'D':
                        menu_state = state_mod;
                        break;
                    default:
                        bytes += 1;
                        extra = sb;
                        menu_state = state_error;
                        break;
                }
                break;
            case state_mod:
                switch (sb) {
                    case 'e': case 'E':
                        menu_state = state_mode;
                        break;
                    default:
                        bytes += 1;
                        extra = sb;
                        menu_state = state_error;
                        break;
                }
                break;

            case state_sh:
                switch (sb) {
                    case '\r': case '\n': case ' ':
                        show_exceptions();
                        serial_menu_out();
                        menu_state = state_start;
                        break;
                    default:
                        menu_state = state_error;
                        break;
                }
                break;

            case state_clr:
                switch (sb) {
                    case '\r': case '\n': case ' ':
                        clear_exceptions();
                        serial_menu_out();
                        menu_state = state_start;
                        break;
                    default:
                        menu_state = state_error;
                        break;
                }
                break;

            case state_ex:
                switch (sb) {
                    case '\r': case '\n': case ' ':
                        test_exception();
                        serial_menu_out();
                        menu_state = state_start;
                        break;
                    default:
                        menu_state = state_error;
                        break;
                }
                break;

            case state_dump:
                switch (sb) {
                    case '\r': case '\n': case ' ':
                        dump_ee_data();
                        serial_menu_out();
                        menu_state = state_start;
                        break;
                    default:
                        menu_state = state_error;
                        break;
                }
                break;

            case state_mode:
                switch (sb) {
                    case '\r': case '\n': case ' ':
                        change_run_mode();
                        serial_menu_out();
                        menu_state = state_start;
                        break;
                    default:
                        menu_state = state_error;
                        break;
                }
                break;

        }

    }

    return prog_state_NUM;
}

/*
 *
 */

struct func_trans {
    int (*exec)(void);
    void (*enter)(void);
    void (*leave)(void);
};
typedef int (*cb_t)(void);
void loop(void)
{
    static int state = -1;
    int ret;

    const struct func_trans cb[prog_state_NUM] = {
        { main_loop, main_setup, NULL, },
        { menu_loop, menu_enter, menu_leave, },

    };

    if (state < 0)
    {
        if (cfg.run_mode)
            state = prog_state_main;
        else
            state = prog_state_menu;
        cb[state].enter();
    }
    ret = cb[state].exec();
    if (ret < prog_state_NUM && state != ret)
    {
        if (cb[state].leave != NULL)
            cb[state].leave();
        state = ret;
        if (cb[state].enter != NULL)
            cb[state].enter();
    }
}
