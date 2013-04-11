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
    lcd.begin(16, 2);
    lcd.setCursor(0,0);
    Serial.begin(9600);
    Serial.println("hello");
    lcd.print("Menu: Select");
    soft_clear();
}

unsigned long last;
unsigned long count;

void loop(void)
{
    unsigned long a;
    unsigned long b;
    unsigned long c;
    int value;

    a = micros();
    value = analogRead(0);
    b = micros();
    lcd.print(b-a);
    c = micros();

    lcd.print(" ");
    lcd.print(value);
    Serial.println(a);
    Serial.println(b-a);
    Serial.println(c-b);
    while (millis() - last < 2000) {
    }
    last = millis();
    soft_clear();
}
