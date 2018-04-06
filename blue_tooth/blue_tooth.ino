// 用于蓝牙AT模式设置

#include "Arduino.h"

#define LED 13
void setup()
{
    Serial.begin(9600);
    Serial1.begin(38400);
    delay(100);
}
void loop()
{
    if((millis() / 500) & 1)
       digitalWrite(LED, HIGH);
    else
       digitalWrite(LED, LOW);
    while(Serial.available()) {
        Serial1.write(Serial.read());
    }
    while(Serial1.available()) {
        Serial.write(Serial1.read());
    }
}
