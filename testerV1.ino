// Pro mini, индикатор SSD1306_128X64, из схемы удалены транзистор и плата зарядки на TP4056
// вычисление среднее значение за определённый промежуток времени из массива
#include <util/delay.h>
#include "U8glib.h"

#define CHARGE 12       // 8 нога TP4056 управление Мосфетом - включение зарядки аккумулятора
#define DISCHARGE 10    // управление Мосфетом - включение разряда аккумулятора
#define CHARGEoff A3    // 7 нога TP4056
#define CHARGEon A2     // 6 нога TP4056

#define CHARGE_btn A6   //кнопка тест
#define TEST_btn A7     //кнопка заряд

#define Buzzer_Pin 11

U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_NONE);  // I2C / TWI
//U8GLIB_SSD1306_128X32 u8g(U8G_I2C_OPT_NONE);    // I2C / TWI
float Capacity = 0.0;   // Capacity in mAh
float Res_Value = 5.35;  // ----- Resistor Value in Ohm
float Current = 0.0;    // Current in Amp
float mA=0;             // Current in mA
float Bat_Volt = 0.0;   // Battery Voltage
float Res_Volt = 0.0;   // Voltage at lower end of the Resistor
float Bat_High = 4.2;   // Battery High Voltage
float Bat_Low = 2.9;    // ------ Discharge Cut Off Voltage
unsigned long previousMillis = 0; // Previous time in ms
unsigned long millisPassed = 0;  // Current time in ms
float sample1 =0;
float sample2= 0;

// число показаний для получения среднего значения (подбирается индивидуально)
const int numReadings = 5;   //10
const int numReadings1 = 5;  //10

int readings[numReadings];      // массив данных от аналогового вывода
int readIndex = 0;              // 0 текущий индекс
int total = 0;                  // общее значение
int average = 0;                // среднее значение
int readings1[numReadings1];      // массив данных от аналогового вывода 1
int readIndex1 = 0;              // 0 текущий индекс 1
int total1 = 0;                  // общее значение 1
int average1 = 0;                // среднее значение 1


int Bat_Pin = A0; // Аналоговый вывод А0
int Res_Pin = A1; // Аналоговый вывод А1

byte Start = false;
int TimeCount = 0;
byte tt = true;
byte Step = 0;



ISR (TIMER1_COMPA_vect) // эта функция вызывается прерыванием по таймеру и используется для учета времени для подсчета емкости
{
    if (Start)            // Если "Start" = true
    {
        TimeCount++;      // считаем секунды
        tt = true;        // и разрешаем измерение силы тока (емкости) 1 раз в секунду
    }
}

//************************ OLED Display Draw Function *******************************************************
void draw(void) {
    u8g.firstPage();
    do {
        u8g.setFont(u8g_font_fub14r); // select font
        //u8g.setFont(u8g_font_unifont);

        switch (Step) {
            case 0:
                if ( Bat_Volt < 1){
                    u8g.setPrintPos(0,14);        // set position (10,20)
                    u8g.print("No Battery");
                }
                else if ( Bat_Volt > Bat_High){
                    u8g.setPrintPos(25,14);        // set position (25,20)
                    u8g.print("Hi-Volt");
                }
                else if(Bat_Volt < Bat_Low){
                    u8g.setPrintPos(25,14);        // set position (25,20)
                    u8g.print("Low-Volt");
                }
                break;

            case 1:
                u8g.setPrintPos(0,14);        // set position
                u8g.print("Test/Charge");
                break;

            case 2:
                u8g.setPrintPos(0,14);        // set position (25,14)
                u8g.print("CHARGE");
                u8g.drawStr(0, 32, "Volt: ");  //10, 28
                u8g.setPrintPos(58,32);        // set position 58,28
                u8g.print( Bat_Volt,2);  // display Battery Voltage in Volt
                u8g.print("V");
                break;

            case 3:
                u8g.drawStr(0, 14, "Charge is ended");   // put string of display at position X, Y
                u8g.drawStr(0, 32, "Start Test");
                break;

            case 4:
                u8g.drawStr(0, 14, "Volt:");   // put string of display at position X, Y
                u8g.drawStr(0, 32, "Curr:");
                u8g.drawStr(0, 52, "mAh:");

                u8g.setPrintPos(50,14);        // set position
                u8g.print( Bat_Volt,2);  // display Battery Voltage in Volt
                u8g.print("V");
                u8g.setPrintPos(50,32);        // set position
                u8g.print( mA,0);  // display current in mA
                u8g.print("mA");
                u8g.setPrintPos(50, 52);        // set position
                u8g.print( Capacity ,1);     // display capacity in mAh
                break;

            case 5:
                u8g.drawStr(0, 14, "CHARGE");   // put string of display at position X, Y
                u8g.drawStr(0, 32, "Battery charge");
                u8g.drawStr(0, 52, "Volt: ");
                u8g.setPrintPos(58, 52);        // set position
                u8g.print( Bat_Volt,2);     // display capacity in mAh
                break;

            case 6:
                u8g.drawStr(0, 14, "Charge End");   // put string of display at position X, Y
                u8g.drawStr(0, 32, "Batt capacity");
                u8g.drawStr(0, 52, "mAh:");
                u8g.setPrintPos(58, 52);        // set position
                u8g.print( Capacity ,1);     // display capacity in mAh
                break;
        }
    } while( u8g.nextPage() );
}


//******************************Buzzer Beep Function *********************************************************
void beep(unsigned char delay_time){
    analogWrite(Buzzer_Pin, 20);      // PWM signal to generate beep tone
    _delay_ms(delay_time);          // wait for a delayms ms
    analogWrite(Buzzer_Pin, 0);  // 0 turns it off
    _delay_ms(delay_time);          // wait for a delayms ms

}

void V_meter()
{
     // вычитаем значение
  total = total - readings[readIndex];
  total1 = total1 - readings1[readIndex1];
  // считываем с датчика
  readings[readIndex] = analogRead(Bat_Pin);
  readings1[readIndex1] = analogRead(Res_Pin);
  // добавляем полученное значение
  total = total + readings[readIndex];
  total1 = total1 + readings1[readIndex1];
  // переходим на следующую позицию в массиве
  readIndex = readIndex + 1;
  readIndex1 = readIndex1 + 1;

  // если достигли конца массива...
  if (readIndex >= numReadings) {
    // ...возвращаемся в начало:
    readIndex = 0;
  }
  // если достигли конца массива1...
  if (readIndex1 >= numReadings1) {
    // ...возвращаемся в начало:
    readIndex1 = 0;
  }

  // подсчитываем среднее значение
  average = total / numReadings;
  average1 = total1 / numReadings1;
//Serial.println(average); Serial.println(average1);
  delay(1);        // небольшая задержка для стабильности

    sample1 = average;
    sample1  = (sample1 * 2.5) / 1024.0;    // Преобразуем значение из АналогРид в десятичное с учетом опорного напряжения
    Bat_Volt = sample1 / 0.505;               // 0.5 - это коофициент делителя напряжения: R2/(R1+R2 ) // R1 =10K and R2 =10K

    sample2 = average1;
    sample2  = (sample2 * 2.5) / 1024.0;    // Преобразуем значение из АналогРид в десятичное с учетом опорного напряжения
    Res_Volt = sample2 / 0.505;               // 0.5 - это коофициент делителя напряжения: R4/(R3+R4 ) // R3 =10K and R4 =10K
}
void ChargingBattery()
{
    Serial.println("Зарядка батареи ...");
    digitalWrite(CHARGE, HIGH);  // MOSFET is on - включаем зарядку аккумулятора
    digitalWrite(DISCHARGE, LOW);  // MOSFET is off - перепроверяем что разряд откулючен (перестраховка)
    V_meter();
    if (analogRead(CHARGEoff) > 800){
        if (analogRead(CHARGEon) < 800){
            Serial.println("Батарея заряжена!");
            Step++;
            beep(200);
        }
    }
    _delay_ms(30000);
}

void setup() {
    analogReference(EXTERNAL); // устанавливаем откуда будет браться опорное напряжение
    Serial.begin(9600);
    // заполняем массив из 10 элементов нулями
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = 0;
  }
  // заполняем массив1 из 10 элементов нулями
  for (int thisReading1 = 0; thisReading1 < numReadings1; thisReading1++) {
    readings1[thisReading1] = 0;
  }
    pinMode(CHARGE, OUTPUT);
    pinMode(DISCHARGE, OUTPUT);
    pinMode(CHARGEon, INPUT);
    pinMode(CHARGEoff, INPUT);
    pinMode(Buzzer_Pin, OUTPUT);
    //--------------------------------------------------------------------------------------------------------------
    digitalWrite(CHARGE, LOW);  // MOSFET is off
    digitalWrite(DISCHARGE, LOW);  // MOSFET is off

    Serial.println("CLEARDATA");
    Serial.println("LABEL,Time,Bat_Volt,capacity");

    cli(); // запрещаем перывания
    TCCR1A = 0;                       // устанавливаем настройки таймеря для учета времени
    TCCR1B = (1<<WGM12) | (1<<CS12);  // mode 4 и делитель 256
    TIMSK1 |=(1<<OCIE1A);             // прерывание по совпадению
    TCNT1 = 0;                        //
    OCR1A = 62499U;                   //
    sei(); // разрешаем прерывания
}

void loop() {

    switch (Step) {
        case 0:     //проверка при включении
            Serial.println("Проверьте, подключен ли аккумулятор...");
            V_meter();
            if ( Bat_Volt > Bat_High){
                digitalWrite(CHARGE, LOW);  // MOSFET is off
                digitalWrite(DISCHARGE, LOW);  // MOSFET is off // Turned Off the MOSFET // No discharge
                beep(200);
                Serial.println( "Предупреждение о высоком напряжении! ");
                _delay_ms(1000);
            } else if(Bat_Volt < Bat_Low){
                digitalWrite(CHARGE, LOW);  // MOSFET is off
                digitalWrite(DISCHARGE, LOW);  // MOSFET is off
                beep(200);
                Serial.println( "Предупреждение о низком напряжении! ");
                _delay_ms(1000);
            } else {
                Step = 1;
                Serial.println("Аккумулятор подключен!");
                Serial.print("Volage: ");
                Serial.println(Bat_Volt);
                Serial.println("");
                Serial.println("Ожидание выбора режима...");
            }
            break;

        case 1:   // выбор режима

            if (analogRead(CHARGE_btn) > 600){ // режим только зарядки
                Step = 5;
                Serial.println("Выбран режим только для зарядки");
                beep(200);
            }
            else if (analogRead(TEST_btn) > 600){ // режим теста + зарядки
                Step = 2;
                Serial.println("Выбран тестовый режим");
                beep(200);
            }
            else {
                _delay_ms(100);
            }
            break;

        case 2:   // режим теста + зарядки
            ChargingBattery();
            Serial.print(Bat_Volt); Serial.print("V,");
            break;

        case 3:   // задержка для стабилизации батареи
            draw();
            _delay_ms(10000);
            Serial.println("Включаем тест заряда батареи");
            digitalWrite(CHARGE, LOW);  // MOSFET is off - выключаем зарядку аккумулятора (перестраховка)
            digitalWrite(DISCHARGE, HIGH);  // MOSFET is on - включаем тест аккумулятора
            Step++;
            Start = true;
            break;

        case 4:   // измерение емкости батареи
            if (tt == true){
                tt = false;
                V_meter();
                Current = (Bat_Volt - Res_Volt) / Res_Value;
                mA = Current * 1000.0;
                Capacity = Capacity + (mA / 3600.0); // 1 Hour = 3600 s
                Serial.print("Volt,Capacity,"); Serial.print(Bat_Volt); Serial.print(","); Serial.println(Capacity);
            }

            if (Bat_Volt < Bat_Low){
                Start = false;
                Step = 5;
                Serial.println("Проверка батареи завершена");
                Serial.print("Battery capacity: ");
                Serial.println(Capacity);
                digitalWrite(CHARGE, LOW);  // MOSFET is off
                digitalWrite(DISCHARGE, LOW);  // MOSFET is off
                draw();
                beep(200);
                _delay_ms(10000);
            }
            break;

        case 5:   // режим только зарядки
            ChargingBattery();
            Serial.print(Bat_Volt); Serial.print("V,");
            break;

        case 6:   // конец зарада батареи
            //--END--//
            Serial.println("Работа выполнена!");
            Serial.print("Battery capacity: ");
            Serial.println(Capacity);
            beep(200);
            _delay_ms(5000);
            break;
    }
    draw();
}
