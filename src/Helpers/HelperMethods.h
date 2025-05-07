//
// Created by mosthated on 9/28/24.
//

#ifndef HELPERMETHODS_H
#define HELPERMETHODS_H
#include <common/base_classes/FOCMotor.h>

#endif //HELPERMETHODS_H


const char* motorStatusToString(FOCMotorStatus status);

const char* motorStatusToString(FOCMotorStatus status) {
    switch (status) {
        case FOCMotorStatus::motor_uninitialized:
            return "Motor Uninitialized";
        case FOCMotorStatus::motor_initializing:
            return "Motor Initializing";
        case FOCMotorStatus::motor_uncalibrated:
            return "Motor Uncalibrated";
        case FOCMotorStatus::motor_calibrating:
            return "Motor Calibrating";
        case FOCMotorStatus::motor_ready:
            return "Motor Ready";
        case FOCMotorStatus::motor_error:
            return "Motor Error";
        case FOCMotorStatus::motor_calib_failed:
            return "Motor Calibration Failed";
        case FOCMotorStatus::motor_init_failed:
            return "Motor Initialization Failed";
        default:
            return "Unknown Status";
    }
}
