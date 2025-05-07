#!/usr/bin/env python3

import os
import can
import struct
from typing import Dict
import argparse

# import datatypes from datatypes.py file
from datatypes import CANMessageData, CAN_MESSAGE


def prepare_data(is_rtr: bool, datatype: str, value: CANMessageData):
    if is_rtr:
        data = None
        dlc_val = 0
    elif datatype == "CANMessageData":
        data = value.to_bytearray()
        dlc_val = len(data)
    else:
        data = None
        dlc_val = 0
    return data, dlc_val


def send_calibrate():
    message = CANMessageData(
        target=0x001, messageType=CAN_MESSAGE.SET_RUN_MODE, value_type="int", value=CAN_MESSAGE.RUN_MODE.CALIBRATE
    )

    print(str(message))

    print(message.target)
    print(message.messageType)
    print(message.value)
    print(message.value["intValue"])

    (prepared_data, dlc) = prepare_data(False, "CANMessageData", message)
    print(dlc)
    print(prepared_data)

    msg = can.Message(arbitration_id=0x002, dlc=dlc, data=prepared_data)
    print(msg.data)
    return msg


def send_target(value):
    message = CANMessageData(
        target=0x001, messageType=CAN_MESSAGE.SET_TARGET_EXT, value_type="float", value=value, optional=0x021
    )

    print(str(message))

    print(message.target)
    print(message.messageType)
    print(message.value)
    print(message.value["floatValue"])

    (prepared_data, dlc) = prepare_data(False, "CANMessageData", message)
    print(dlc)
    print(prepared_data)

    msg = can.Message(arbitration_id=0x2, dlc=dlc, data=prepared_data, is_extended_id=False)
    print(msg.data)
    print(msg)
    return msg


def main():
    # Initialize the argument parser
    parser = argparse.ArgumentParser(description='Send CAN messages.')
    subparsers = parser.add_subparsers(title='commands', dest='command')

    # Parser for the "calibrate" command
    parser_calibrate = subparsers.add_parser('calibrate', help='Send the calibrate command.')

    # Parser for the "set_target" command
    parser_target = subparsers.add_parser('set_target', help='Send the set_target command.')
    parser_target.add_argument('value', type=float, help='Target value to set.')

    args = parser.parse_args()

    msg = None

    if args.command == 'calibrate':
        msg = send_calibrate()

    elif args.command == 'set_target':
        msg = send_target(args.value)

    print(msg)

    # Initialize the CAN interface
    # Get results of the command 'ip link show can9' and parse it to get the state of the can9 interface
    result = os.popen('ip link show can9').read()
    print(result)

    # Example Results
    # can9: <NOARP,ECHO> mtu 16 qdisc noop state DOWN mode DEFAULT group default qlen 10
    # link/can 

    # Parse the result to get the state of the can9 interface
    state = result.split(' ')[8]
    print(state)

    # If the state is 'DOWN', run the script to set up the can9 interface
    if state == 'DOWN':
        os.system('/home/mosthated/_dev/languages/pwsh/system/canbus/set_can.ps1')

    can9 = can.interface.Bus(channel="can9", interface="socketcan")

    if not can9:
        print("Error: Cannot find the CAN interface.")
        exit(1)

    can9.send(msg)
    can9.shutdown()


if __name__ == '__main__':
    main()
