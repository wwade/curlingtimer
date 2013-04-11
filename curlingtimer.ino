#include <Arduino.h>
#include <LiquidCrystal.h>

static LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

static void soft_clear(void)
{
    lcd.setCursor(0,1);
    lcd.print("                ");
    lcd.setCursor(0,1);
}

void setup(void)
{
    Serial.begin(9600);
    Serial.println("hello");
    lcd.begin(16, 2);
    lcd.setCursor(0,0);
    lcd.print("Menu: Select");
    soft_clear();
}

unsigned long last;
unsigned long count;

static void print_tag(const char *tag, unsigned long val)
{
    Serial.print(tag);
    Serial.println(val);
}

static void print_tag(const char *tag, float val)
{
    Serial.print(tag);
    Serial.println(val);
}

void loop(void)
{
    float xy;
    unsigned long a;
    unsigned long b;
    unsigned long c;
    int value;

    a = micros();
    value = analogRead(0);
    b = micros();
    xy = (float)value / 145.3;
    c = micros();

    print_tag("Analog Read (us): ", b-a);
    print_tag("Floating point math (us): ", c-b);
    print_tag("FP math result: ",  xy);

    Serial.println(a);
    Serial.println(b-a);
    Serial.println(c-b);
    while (millis() - last < 2000) {
    }
    last = millis();
    soft_clear();
}
