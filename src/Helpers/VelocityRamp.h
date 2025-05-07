//
// Created by mosthated on 8/11/24.
//

#pragma once

#ifndef VELOCITYRAMP_H
#define VELOCITYRAMP_H
#include <common/foc_utils.h>

#endif //VELOCITYRAMP_H
// inline float totalDelta = 0.0f;
// inline float currentDelta = 0.0f;
inline float newVelocityLimit = 0.0f;
inline float movementStartAngle = 0.0f;
inline float velocityLimitLow = 5.0f;
inline float velocityLimitHigh = 10.0f;
inline float velocityCalibrationLimit = 10.0f;
inline float velocityOutputRamp = 200;
inline float minimumAngleThreshold = (M_PI) / 12; // 15 degrees
inline float smallAngleThreshold = (M_PI) / 6; // 30 degrees
inline float largeAngleThreshold = (M_PI) / 3; // 60 degrees

inline float rampUpEnd = 0.25;
inline float rampDownStart = 0.75;
inline float rampRateLimit = 0.1f;
inline float previousVelocityLimit = velocityLimitLow;

inline float velocityRamp(
    const float startAngle,
    const float currentAngle,
    const float targetAngle,
    float velocityLimitLow,
    float velocityLimitHigh
)
{
    // Angle is in radian
    // Start movement with motor.velocity_limit = velocityLimitLow
    // increase motor.velocity_limit toward velocityLimitHigh
    // when currentAngle is 1/4 way to targetAngle,  motor.velocity_limit should be velocityLimitHigh
    // when currentAngle is 3/4 way to targetAngle begin to decrease motor.velocity_limit toward velocityLimitLow
    // when currentAngle is at targetAngle, motor.velocity_limit should be velocityLimitLow

    float velocityLimitH = velocityLimitHigh;
    const float velocityLimitL = velocityLimitLow;
    float newVelocityLimit = 0.0f;

    // Calculate total distance from start to target.
    const float totalDelta = abs(targetAngle - startAngle);

    // Calculate how far we traveled from the start.
    const float currentDelta = abs(currentAngle - startAngle);

    // Calculate our progress as a value from 0 to 1.
    const float progress = currentDelta / totalDelta;

    // if (totalDelta < minimumAngleThreshold) {
    if (totalDelta < 1) {
        // For tiny angles, spend more time in ramp up and ramp down stages
        rampUpEnd = 0.5;
        rampDownStart = 0.5;
        rampRateLimit = 0.05f;
        velocityLimitH = velocityLimitL;
    } else if (totalDelta <= 2) {
        // } else if (totalDelta < smallAngleThreshold) {
        // For small angles, spend more time in ramp up and ramp down stages
        rampUpEnd = 0.4;
        rampDownStart = 0.6;
        rampRateLimit = 0.1f;
        velocityLimitH = velocityLimitL + 2;
    } else if (totalDelta <= 8) {
        // } else if (totalDelta < smallAngleThreshold) {
        // For small angles, spend more time in ramp up and ramp down stages
        rampUpEnd = 0.4;
        rampDownStart = 0.6;
        rampRateLimit = 0.1f;
        velocityLimitH = velocityLimitHigh - 2;
    } else if (totalDelta < 20) {
        // } else if (totalDelta < largeAngleThreshold) {
        // For medium angles, spend less time in ramp up and ramp down stages
        rampUpEnd = 0.25;
        rampDownStart = 0.75;
        rampRateLimit = 0.5f;
    } else {
        // For large angles, use original fractions
        rampUpEnd = 0.20;
        rampDownStart = 0.75;
        rampRateLimit = 1.0f;
    }


    rampUpEnd = 0.25;
    rampDownStart = 0.75;
    rampRateLimit = 0.2f;


    if (progress < rampUpEnd) {
        // first stage: ramping up
        newVelocityLimit = velocityLimitL + ((velocityLimitH - velocityLimitL) * progress / rampUpEnd);
    } else if (progress < rampDownStart) {
        // second stage: constant speed
        newVelocityLimit = velocityLimitH;
    } else {
        // third stage: ramp down
        newVelocityLimit = velocityLimitH - ((velocityLimitH - velocityLimitL) * (progress - rampDownStart) / (1.0f - rampDownStart));
    }

    newVelocityLimit = std::min(std::max(newVelocityLimit, velocityLimitL), velocityLimitH);

    // --| Velocity Ramp Limiter ----------------
    {
        float velocityLimitChange = newVelocityLimit - previousVelocityLimit;

        if (abs(velocityLimitChange) > rampRateLimit) {
            newVelocityLimit = previousVelocityLimit + (rampRateLimit * _sign(velocityLimitChange));
        }

        previousVelocityLimit = newVelocityLimit;
    }

    newVelocityLimit = std::min(std::max(newVelocityLimit, velocityLimitL), velocityLimitH);

    return newVelocityLimit;
}
