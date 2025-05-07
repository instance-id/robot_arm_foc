#!/usr/bin/env python

import struct


class CANMessageData:
    def __init__(self, target, messageType, value_type, value, optional=None):

        # target: byte
        # If optional is not None, then it is a byte, combine target and option via:
        # target = target << 4 | optional

        if messageType == CAN_MESSAGE.SET_TARGET_EXT and optional is not None:
            self.target = target << 4 | optional
            print("target before :", target)
            print("optional: ", optional)
            print("target: ", self.target)

        else:
            self.target = target

        # messageType: byte
        self.messageType = messageType

        # value: dict
        self.value = {"floatValue": None, "intValue": None}

        # value_type: str
        self.value_type = value_type

        if value_type == "float":
            self.value["floatValue"] = value
        elif value_type == "int":
            self.value["intValue"] = value
        else:
            raise ValueError("Invalid value type. Must be 'float' or 'int'.")

    def to_bytearray(self):
        # Example serialization logic
        data = bytearray()

        data.extend(struct.pack("B", self.target))  # Assuming target is 1 byte

        data.extend(struct.pack("B", self.messageType))  # Assuming messageType is 4 bytes

        data.extend(struct.pack("BB", 0, 0))

        if self.value_type == "float":
            data.extend(struct.pack("f", self.value["floatValue"]))
        elif self.value_type == "int":
            data.extend(struct.pack("I", self.value["intValue"]))
        elif self.value_type == "byte":
            data.extend(struct.pack("B", self.value["byteValue"]))
        else:
            raise ValueError("Invalid value type. Must be 'float' or 'int' or 'byte'.")

        return data


class CAN_MESSAGE:
    NONE = 0x00
    SET_TARGET = 0x01
    SET_POSITION = 0x02
    SET_VELOCITY = 0x03
    SET_Q_PID_GAIN = 0x04
    SET_D_PID_GAIN = 0x05
    SET_V_PID_GAIN = 0x06
    SET_A_PID_GAIN = 0x07
    SET_CONTROL_MODE = 0x08
    SET_LIMIT = 0x09
    SET_TORQUE_TYPE = 0x0A
    SET_ENABLE = 0x0B
    GET_POSITION = 0x80
    GET_TARGET = 0x81
    GET_VELOCITY = 0x82
    GET_Q_PID_GAIN = 0x83
    GET_D_PID_GAIN = 0x84
    GET_V_PID_GAIN = 0x85
    GET_A_PID_GAIN = 0x86
    GET_CONTROL_MODE = 0x87
    GET_LIMIT = 0x88
    GET_TORQUE_TYPE = 0x89
    GET_ENABLE = 0x90
    BUTTON_PRESSED = 0x91
    SEND_FLOAT = 0x92
    SEND_TEMP = 0x93
    SEND_CURRENT = 0x94
    REQUEST_READY = 0x95
    SEND_READY = 0x96
    ENDSTOP_PRESSED = 0x97
    SET_RUN_MODE = 0x98
    REPORT_RUN_MODE = 0x99
    SET_TARGET_EXT = 0xA0

    class RUN_MODE:
        IDLE = 0x01
        INITIALIZE = 0x02
        CALIBRATE = 0x03
        RUN = 0x04

    class CURRENT_RUN_MODE:
        IDLE = 0x01
        CALIBRATING = 0x02
        CALIBRATED = 0x03
        READY = 0x04
        RUNNING = 0x05

    class CONTROLLER:
        P_GAIN = 0x01
        I_GAIN = 0x02
        D_GAIN = 0x03
        SATURATION_LIMIT = 0x04
        RAMP_PARAMETER = 0x05
        LOWPASS_TC = 0x06

    class CONTROL_MODE:
        TORQUE_CONTROL = 0
        VELOCITY_CONTROL = 1
        POSITION_CONTROL = 2
        VELOCITY_OPENLOOP_CONTROL = 3
        POSITION_OPENLOOP_CONTROL = 4

    class TORQUE_TYPE:
        VOLTAGE = 0x01
        DC_CURRENT = 0x02
        FOC_CURRENT = 0x03

    class LIMIT:
        CURRENT_LIMIT = 0x01
        VOLT_LIMIT = 0x02
        VELOCITY_LIMIT = 0x03

    class STATUS:
        ENABLE = 0x01
        DISABLE = 0x02

    class BUTTON:
        PRESSED = 0x01
        RELEASED = 0x02

    class TEST:
        FLOAT = 0x01
        INT = 0x02
        STRING = 0x03
        BOOL = 0x04
