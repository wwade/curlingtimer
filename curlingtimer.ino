#include <Arduino.h>
#include <LiquidCrystal.h>
#include <EEPROM.h>

//#define ENABLE_PROFILING
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


#define DBG(args...) ({ Serial.print(__LINE__); Serial.print(": "); Serial.println(args); delay(10); })

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
};
static struct cfg cfg = {
    /* version     */ 0x101,
    /* threshold   */ 100,
    /* min_time    */ 500,
    /* max_time    */ 6000,
};

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

static void load_config(void)
{
    struct cfg *tmp;
    unsigned char cfg_tmp[sizeof *tmp];
    int i;
    for (i = 0; i < sizeof cfg_tmp; i++)
    {
        cfg_tmp[i] = EEPROM.read(i);
    }
    tmp = (struct cfg*)((void*)cfg_tmp);
    print_config("Default", &cfg);
    print_config("EEPROM", tmp);
    if (tmp->ver_info == cfg.ver_info)
    {
        memcpy(&cfg, tmp, sizeof cfg);
    }
    else
    {
        memcpy(tmp, &cfg, sizeof cfg);
        for (i = 0; i < sizeof cfg_tmp; i++)
        {
            EEPROM.write(i, cfg_tmp[i]);
        }
    }
    print_config("Using", tmp);
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

    main_setup();
}


static int sensor_value;
static int check_sensor_for_event(int which)
{
    int now;
    int diff;

    switch (which)
    {
        case 0:
            now = analogRead(0);
            break;
        case 1:
            now = analogRead(0);
            break;
        default:
            now = 0;
            break;
    }

    if (now == lcd_button_SELECT)
        return 0;

    if (now > sensor_value)
        diff = now - sensor_value;
    else
        diff = sensor_value - now;
    sensor_value = now;
    return diff;
}

#ifdef ENABLE_PROFILING
unsigned long calc_average(unsigned long *values, int n)
{
    unsigned long ret;
    unsigned long avg;
    unsigned long was;
    int i;

    avg = 0;
    DBG(n);
    for (i = 0; i < n; i++)
    {
        was = avg;
        avg += values[i];
        if (avg < was)
        {
            DBG("WARNING: average() overflow");
            break;
        }
    }
    if (n == 0)
    {
        DBG("SIGFPE");
        return 0;
    }
    DBG(avg);
    ret = avg/n;
    DBG(ret);
    return ret;
}
#endif

/*
 * Reading split times:
 * First, wait for sensor 1 to trigger.  After sensor 1, wait for sensor 2.
 * If sensor 2 is not detected within 5 seconds, reset and wait for sensor 1.
 */

#ifdef ENABLE_PROFILING
const int adc_num = 100;
static unsigned long adc_reads[adc_num] = {};
#endif

static int main_loop(void)
{
    unsigned long timeout;
    unsigned long times[2];
    unsigned long now;
    unsigned long last;
    int iter;
    int diff;

    timeout = 0;
    last = millis();
    for (iter = 0; iter < 2; iter++)
    {
#ifdef ENABLE_PROFILING
        unsigned long avg;
        int adc_idx = 0;
        int adc_looped = 0;
#endif

        lcd.setCursor(15, 1);
        if (iter == 0)
            lcd.print("A");
        else
            lcd.print("B");

        check_sensor_for_event(iter);
        do
        {
#ifdef ENABLE_PROFILING
            unsigned long a;
            unsigned long b;
            a = micros();
#endif
            diff = check_sensor_for_event(iter) ;
            times[iter] = millis();
            if (iter > 0)
            {
                now = times[iter];
                if (now - times[iter-1] > cfg.max_time)
                {
                    timeout = 1;
                    break;
                }
            }
            else
            {
                if (times[iter] - last > 100)
                {
                    if (lcd_get_button() == lcd_button_SELECT)
                    {
                        Serial.println("got SELECT button");
                        return prog_state_menu;
                    }
                }
            }
#ifdef ENABLE_PROFILING
            b = micros();
            adc_reads[adc_idx] = (b-a);
            adc_idx += 1;
            if (adc_idx == adc_num)
            {
                adc_looped = 1;
                adc_idx = 0;
            }
#endif
        } while (diff < cfg.threshold);
        soft_clear();

        lcd.print(diff);
        lcd.print(" ");
        lcd.print(cfg.threshold);

#ifdef ENABLE_PROFILING
        if (adc_looped)
            adc_idx = adc_num;
        avg = calc_average(adc_reads, adc_idx);
        lcd.print(avg, DEC);
        Serial.print("AVERAGE: ");
        Serial.println(avg);
#endif
        while (millis() - times[iter] < cfg.min_time)
        {
        }
    }

    soft_clear();
    if (timeout)
    {
        lcd.print("(TIMEOUT)");
    }
    else
    {
        lcd.print((float)(times[1] - times[0])/1000.0, 2);
    }

    return prog_state_NUM;
}

static void menu_enter(void)
{
    DBG(__FUNCTION__);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("MENU");
    while (lcd_get_button() != lcd_button_NONE) { }
}

static void menu_leave(void)
{
    while (lcd_get_button() != lcd_button_NONE) { }
}

static int menu_loop(void)
{
    if (lcd_get_button() == lcd_button_SELECT)
        return prog_state_main;

    return prog_state_NUM;
}


struct func_trans {
    int (*exec)(void);
    void (*enter)(void);
    void (*leave)(void);
};
typedef int (*cb_t)(void);
void loop(void)
{
    static int state = prog_state_main;
    int ret;

    const struct func_trans cb[prog_state_NUM] = {
        { main_loop, main_setup, NULL, },
        { menu_loop, menu_enter, menu_leave, },

    };

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
