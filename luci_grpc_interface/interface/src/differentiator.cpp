/**
 * @file differentiator.cpp
 * @brief Math class to differentiate values for speed calculations
 * @date 2023-08-10
 *
 * @copyright Copyright (c) 2023 LUCI Mobility, Inc. All Rights Reserved.
 *
 */

#include "../include/differentiator.h"

Differentiator::Differentiator() {}

Differentiator::~Differentiator() {}

float Differentiator::differentiate(float value, float timestamp)
{
    // Calculate the differential
    float derivative = 0.0;
    if (timestamp - this->previousTime == 0)
    {
        derivative = this->previousDerivative;
    }
    else if (this->previousTime == 0)
    {
        derivative = 0;
    }
    else
    {
        derivative = (value - this->previousValue) / (timestamp - this->previousTime);
    }

    this->previousTime = timestamp;
    this->previousValue = value;
    this->previousDerivative = derivative;
    return derivative;
}