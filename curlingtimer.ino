#include <Arduino.h>
#include <LiquidCrystal.h>
#include <EEPROM.h>

//#define ENABLE_PROFILING

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

void setup(void)
{
    memset(lcd_clear_row, ' ', sizeof(lcd_clear_row) - 1);
    Serial.begin(9600);
    lcd.begin(16, 2);

    load_config();

    lcd.setCursor(0,0);
    lcd.print("Menu: Select");

    soft_clear();

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

void loop(void)
{
    unsigned long timeout;
    unsigned long times[2];
    unsigned long now;
    int iter;
    int diff;

    timeout = 0;
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
}
