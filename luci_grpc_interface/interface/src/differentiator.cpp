/**
 * @file differentiator.cpp
 * 
 * @brief Math class to differentiate values for speed calculations
 *
 * @copyright Copyright 2025 LUCI Mobility, Inc
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *    https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
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