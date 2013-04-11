#include <Arduino.h>
#include <LiquidCrystal.h>
#include <EEPROM.h>

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
    /* version     */ 0x100,
    /* threshold   */ 100,
    /* min_time    */ 500,
    /* max_time    */ 5000,
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

unsigned long adc_reads[1024];
int idx;

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

/*
 * Reading split times:
 * First, wait for sensor 1 to trigger.  After sensor 1, wait for sensor 2.
 * If sensor 2 is not detected within 5 seconds, reset and wait for sensor 1.
 */
void loop(void)
{
    unsigned long timeout;
    unsigned long times[2];
    unsigned long now;
    int iter;
    int diff;
    unsigned long a;
    unsigned long b;

    timeout = 0;
    for (iter = 0; iter < 2; iter++)
    {
        lcd.setCursor(15, 1);
        if (iter == 0)
            lcd.print("A");
        else
            lcd.print("B");

        check_sensor_for_event(iter);
        do
        {
            a = micros();
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
            b = micros();
        } while (diff < cfg.threshold);
        soft_clear();
        lcd.print(diff);
        lcd.print(" ");
        lcd.print(cfg.threshold);
        delay(cfg.min_time);
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
