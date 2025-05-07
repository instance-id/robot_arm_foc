#if defined(RUN_SERIAL_TEST) && RUN_SERIAL_TEST == 1 && defined(FOC_CONTROLLER) && FOC_CONTROLLER == 22

#include <SimpleFOC.h>

void setup()
{

    Serial.begin(115200);
    Serial.println("serial test");

    Serial.println("got this far");

    _delay(1000);
}

void loop()
{
    for (int i = 0; i < 2000; i++)
    {
    }
    Serial.println("TEST serial");
    delay(100);
}
#endif
