#if defined(RUN_MAIN) && RUN_MAIN == 2 && defined(FOC_CONTROLLER) && FOC_CONTROLLER == 24

#include <Arduino.h>
#include <climits>

#include "SimpleCAN.h" // <- this is the only include required, it should be smart enough to find the correct subclass
#include "CANProfile_V1.h"

byte txIdentifier = 0x21;

CANReceiver CANBroker;
CANHandler CANDevice(&CAN ,&CANBroker, txIdentifier);

uint32_t randomData = 0; // <- 32-bit unsigned is easy to use as can data (4 bytes)

void setup()
{

    Serial.begin(230400);

    CAN.logTo(&Serial);
    delay(2000);
    Serial.println("Starting CAN");
    // CAN.enableInternalLoopback();
    CanFilter filter = CanFilter(ACCEPT_ALL, 0x21, 0x22, FILTER_ANY_FRAME);
    CAN.filter(filter);
    CAN.begin(CAN_BITRATE);
    delay(10);
}

uint8_t *data = nullptr;

float floatData = 0.0f;
#define MAKE_CAN_ID(Device, Message) ((Device << 8) | Message)
#define GET_MESSAGE_ID(CanID) (CanID & 0xff)
#define GET_DEVICE_ID(CanID) (CanID >> 8)

uint32_t incrementedValue = 0;

// uint8_t *random_data() {
//     uint32_t randomNumber = incrementedValue++;
//     floatData = static_cast<float>(randomNumber + 0.5f);
//     uint32_t canMessage = floatData;
//
//     static uint8_t data[4];
//     data[0] = (canMessage >> 24) & 0xFF; // Extract the most significant byte
//     data[1] = (canMessage >> 16) & 0xFF; // Extract the next byte
//     data[2] = (canMessage >> 8) & 0xFF;  // Extract the second least significant byte
//     data[3] = canMessage & 0xFF;
//
//     return data;
// }

uint8_t *random_data()
{
    uint32_t randomNumber = incrementedValue++;
    floatToBytes.floatNumber = static_cast<float>(randomNumber + 0.5f);
    return floatToBytes.bytes;
}

uint8_t currentValue = 0;

const byte GetCANMessage()
{

    if (currentValue == 0)
    {
        currentValue = 1;
        return CAN_MESSAGE::SET_POSITION;
    }
    else
    {
        currentValue = 0;
        return CAN_MESSAGE::SET_TARGET;
    }
}

float lastTarget = 0.0f;

void loop()
{
    bool isExtendedFrame = false;

    byte txIdentifier = (uint32_t)0x21;
    bool isRtr = false;
    // delay(50);

    // An array to hold the bytes of the packed message
    // memcpy(packedMessage, &originalMessage, sizeof(originalMessage));

    // CanMsg txMsg = CanMsg(
    //         isExtendedFrame
    //         ? CanExtendedId(txIdentifier, isRtr)
    //         : CanStandardId(txIdentifier, isRtr),
    //         8,
    //         packedMessage);
    // delay(50);
    // CAN.write(txMsg);

    // delay(1);

    if (CAN.available() > 0)
    {
        uint8_t packedMessage[8];
        CanMsg const rxMsg = CAN.read();

        CANDevice.HandleCanMessage(rxMsg, rxMsg.data);

#pragma region Old Code
        // for (int i = 0; i < rxMsg.data_length; i++)
        // {
        //     packedMessage[i] = rxMsg.data[i];
        // }

        // CANMessageData receivedMessage{};
        // memcpy(&receivedMessage, packedMessage, sizeof(packedMessage));

        // uint16_t rxDeviceId = rxMsg.getStandardId();
        // uint8_t rxMsgType = receivedMessage.messageType;
        // float rxValue = receivedMessage.value;

        // String floatValAsString = String(rxValue, 3); // 2 is the number of decimals

        // Serial.printf("Device ID: 0x%x ", rxDeviceId);
        // Serial.printf("Message Type: %d ", rxMsgType);
        // Serial.println("Value: " + floatValAsString);

        // Serial.printf("Device ID: %x Message Type: %x Value: %0.2f\n", rxDeviceId, rxMsgType, rxValue);

        // if (rxMsg.data_length == 8)
        // {
        //     byteToFloat.bytes[0] = rxMsg.data[0];
        //     byteToFloat.bytes[1] = rxMsg.data[1];
        //     byteToFloat.bytes[2] = rxMsg.data[2];
        //     byteToFloat.bytes[3] = rxMsg.data[3];
        // }

        // switch (rxMsgType)
        // {
        // case CAN_MESSAGE::SET_POSITION:
        // {
        //     Serial.printf("Device ID: 0x%x Float: %f SET_POSITION: %0.2f\n", rxDeviceId, (double)rxValue, rxValue);
        //     break;
        // }

        // case CAN_MESSAGE::SET_TARGET:
        // {
        //     Serial.printf("Device ID: 0x%x Float: %f SET_TARGET: %f\n", rxDeviceId, (double)rxValue, rxValue);
        //     break;
        // }
        // }

        // Serial.print("polling read: ");

        // if (rxMsg.isExtendedId())

        // {
        //     Serial.print(rxMsg.getExtendedId(), HEX);
        //     Serial.println(" Extended ✅");
        // }
        // else
        // {
        //     Serial.print(rxMsg.getStandardId(), HEX);
        //     rxMsg.getStandardId();
        //     Serial.println(" Standard ✅");
        // }

#pragma endregion
    }

    if (CANBroker.motorTarget != 0.0f && lastTarget != CANBroker.motorTarget)
    {
        lastTarget = CANBroker.motorTarget;
        Serial.printf("Motor Value: %f\n", CANBroker.motorTarget);
    }
}

#endif
