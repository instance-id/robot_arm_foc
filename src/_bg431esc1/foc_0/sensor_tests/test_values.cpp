

// #include <SimpleFOC.h>

// #define ST(A) #A
// #define STR(A) ST(A)

// int i;

// void setup()
// {
//     /* Serial.begin(UPLOAD_SPEED,SERIAL_8N1); */
//     Serial.begin(MONITOR_SPEED);
//     Wire.setClock(400000);

//     delay(1000);
//     while (!Serial)
//     {
//         delay(10);
//         Serial.println("================================");
//         Serial.println("= Error to initialize USB port =");
//     };
//     Serial.println("=========================================");
//     Serial.println("= Monitor speed was " + String(MONITOR_SPEED) + " baud.");
//     Serial.println("= Motor name is " + String(MOTOR) + ".");
//     Serial.println("= Pole pairs " + String(POLE_PAIRS) + ".");
//     Serial.println("= Voltage limit" + String(VOLTAGE_LIMIT) + ".");
//     Serial.println("=========================================");
//     Serial.println("======= work fine in setup section ================");
//     i = 1;

//     _delay(1000);
// }

// void loop()
// {
//     while (i <= 10)
//     {
//         Serial.println("======= work fine in loop section i = " + String(i));
//         i++;
//     }
// }