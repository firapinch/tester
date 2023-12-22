#include <util/delay.h>
#include "U8glib.h"

#define CHARGE 12
#define DISCHARGE 10
#define CHARGEoff A3
#define CHARGEon A2
#define CHARGE_btn A6
#define TEST_btn A7
#define Buzzer_Pin 11
#define Bat_Pin A0
#define Res_Pin A1

U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_NONE);
float Capacity = 0.0;
float Res_Value = 5.35;
float Current = 0.0;
float mA = 0;
float Bat_Volt = 0.0;
float Res_Volt = 0.0;
float Bat_High = 4.2;
float Bat_Low = 2.9;
unsigned long previousMillis = 0;
unsigned long millisPassed = 0;
float sample1 = 0;
float sample2 = 0;

const int numReadings = 5;
const int numReadings1 = 5;
int readings[numReadings];
int readIndex = 0;
int total = 0;
int average = 0;
int readings1[numReadings1];
int readIndex1 = 0;
int total1 = 0;
int average1 = 0;

byte Start = false;
int TimeCount = 0;
byte tt = true;
byte Step = 0;

ISR(TIMER1_COMPA_vect) {
    if (Start) {
        TimeCount++;
        tt = true;
    }
}

void draw(void) {
    u8g.firstPage();
    do {
        u8g.setFont(u8g_font_fub14r);

        switch (Step) {
            case 0:
                if (Bat_Volt < 1){
                    u8g.setPrintPos(0,14);
                    u8g.print("Нет батареи");
                }
                else if (Bat_Volt > Bat_High){
                    u8g.setPrintPos(25,14);
                    u8g.print("Высокое напряжение");
                }
                else if(Bat_Volt < Bat_Low){
                    u8g.setPrintPos(25,14);
                    u8g.print("Низкое напряжение");
                }
                break;

            case 1:
                u8g.setPrintPos(0,14);
                u8g.print("Тест/Зарядка");
                break;

            case 2:
                u8g.setPrintPos(0,14);
                u8g.print("ЗАРЯД");
                u8g.drawStr(0, 32, "Напр: ");
                u8g.setPrintPos(58,32);
                u8g.print(Bat_Volt,2);
                u8g.print("V");
                break;

            case 3:
                u8g.drawStr(0, 14, "Заряд завершен");
                u8g.drawStr(0, 32, "Начать тест");
                break;

            case 4:
                u8g.drawStr(0, 14, "Напр:");
                u8g.drawStr(0, 32, "Ток:");
                u8g.drawStr(0, 52, "mAh:");

                u8g.setPrintPos(50,14);
                u8g.print(Bat_Volt,2);
                u8g.setPrintPos(50,32);
                u8g.print(mA,0);
                u8g.setPrintPos(50, 52);
                u8g.print(Capacity ,1);
                break;

            case 5:
                u8g.drawStr(0, 14, "ЗАРЯД");
                u8g.drawStr(0, 32, "Заряд батареи");
                u8g.drawStr(0, 52, "Напр: ");
                u8g.setPrintPos(58, 52);
                u8g.print(Bat_Volt,2);
                break;

            case 6:
                u8g.drawStr(0, 14, "Конец заряда");
                u8g.drawStr(0, 32, "Емкость батареи");
                u8g.drawStr(0, 52, "mAh:");
                u8g.setPrintPos(58, 52);
                u8g.print(Capacity ,1);
                break;
        }
    } while( u8g.nextPage() );
}

void beep(unsigned char delay_time) {
    analogWrite(Buzzer_Pin, 20);
    _delay_ms(delay_time);
    analogWrite(Buzzer_Pin, 0);
    _delay_ms(delay_time);
}

void V_meter() {
    total = total - readings[readIndex];
    total1 = total1 - readings1[readIndex1];
    readings[readIndex] = analogRead(Bat_Pin);
    readings1[readIndex1] = analogRead(Res_Pin);
    total = total + readings[readIndex];
    total1 = total1 + readings1[readIndex1];
    readIndex = (readIndex + 1) % numReadings;
    readIndex1 = (readIndex1 + 1) % numReadings1;

    average = total / numReadings;
    average1 = total1 / numReadings1;

    delay(1);

    sample1 = average;
    sample1 = (sample1 * 2.5) / 1024.0;
    Bat_Volt = sample1 / 0.505;

    sample2 = average1;
    sample2 = (sample2 * 2.5) / 1024.0;
    Res_Volt = sample2 / 0.505;
}

void ChargingBattery() {
    Serial.println("Идет зарядка батареи...");
    digitalWrite(CHARGE, HIGH);
    digitalWrite(DISCHARGE, LOW);
    V_meter();

    if (analogRead(CHARGEoff) > 800 && analogRead(CHARGEon) < 800) {
        Serial.println("Батарея заряжена!");
        Step++;
        beep(200);
    }

    if (millis() - chargeTimer >= 30000) {
        chargeTimer = millis();
    }
}

void setup() {
    analogReference(EXTERNAL);
    Serial.begin(9600);

    for (int thisReading = 0; thisReading < numReadings; thisReading++) {
        readings[thisReading] = 0;
    }

    for (int thisReading1 = 0; thisReading1 < numReadings1; thisReading1++) {
        readings1[thisReading1] = 0;
    }

    pinMode(CHARGE, OUTPUT);
    pinMode(DISCHARGE, OUTPUT);
    pinMode(CHARGEon, INPUT);
    pinMode(CHARGEoff, INPUT);
    pinMode(Buzzer_Pin, OUTPUT);

    digitalWrite(CHARGE, LOW);
    digitalWrite(DISCHARGE, LOW);

    Serial.println("CLEARDATA");
    Serial.println("LABEL,Time,Bat_Volt,capacity");

    cli();
    TCCR1A = 0;
    TCCR1B = (1 << WGM12) | (1 << CS12);
    TIMSK1 |= (1 << OCIE1A);
    TCNT1 = 0;
    OCR1A = 62499U;
    sei();
}

void loop() {
    switch (Step) {
        case 0:     // Проверка подключения батареи
            Serial.println("Проверка подключения батареи...");
            V_meter();
            if (Bat_Volt > Bat_High) {
                digitalWrite(CHARGE, LOW);
                digitalWrite(DISCHARGE, LOW);
                beep(200);
                Serial.println("Предупреждение: Высокое напряжение!");
                _delay_ms(1000);
            } else if (Bat_Volt < Bat_Low) {
                digitalWrite(CHARGE, LOW);
                digitalWrite(DISCHARGE, LOW);
                beep(200);
                Serial.println("Предупреждение: Низкое напряжение!");
                _delay_ms(1000);
            } else {
                Step = 1;
                Serial.println("Батарея подключена!");
                Serial.print("Напряжение: ");
                Serial.println(Bat_Volt);
                Serial.println("");
                Serial.println("Ожидание выбора режима...");
            }
            break;

        case 1:   // Выбор режима

            if (analogRead(CHARGE_btn) > 600) { // Только режим зарядки
                Step = 5;
                Serial.println("Выбран только режим зарядки");
                beep(200);
            }
            else if (analogRead(TEST_btn) > 600) { // Режим теста + зарядка
                Step = 2;
                Serial.println("Выбран режим теста");
                beep(200);
            }
            else {
                _delay_ms(100);
            }
            break;

        case 2:   // Режим теста + зарядка
            ChargingBattery();
            Serial.print(Bat_Volt); Serial.print("V,");
            break;

        case 3:
            draw();
            if (millis() - chargeTimer >= 10000) {
                Serial.println("Начало теста заряда батареи");
                digitalWrite(CHARGE, LOW);
                digitalWrite(DISCHARGE, HIGH);
                Step++;
                Start = true;
            }
            break;

        case 4:
            if (tt == true) {
                tt = false;
                V_meter();
                Current = (Bat_Volt - Res_Volt) / Res_Value;
                mA = Current * 1000.0;
                Capacity = Capacity + (mA / 3600.0);
                Serial.print("Напряжение,Емкость,"); Serial.print(Bat_Volt); Serial.print(","); Serial.println(Capacity);
            }

            if (Bat_Volt < Bat_Low) {
                Start = false;
                Step = 5;
                Serial.println("Проверка батареи завершена");
                Serial.print("Емкость батареи: ");
                Serial.println(Capacity);
                digitalWrite(CHARGE, LOW);
                digitalWrite(DISCHARGE, LOW);
                draw();
                beep(200);
                _delay_ms(10000);
            }
            break;

        case 5:
            ChargingBattery();
            Serial.print(Bat_Volt); Serial.print("V,");
            break;

        case 6:
            Serial.println("Задача выполнена!");
            Serial.print("Емкость батареи: ");
            Serial.println(Capacity);
            beep(200);
            _delay_ms(5000);
            break;
    }
    draw();
}
