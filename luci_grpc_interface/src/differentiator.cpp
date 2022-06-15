#include "../include/differentiator.h"

Differentiator::Differentiator() {}

Differentiator::~Differentiator() {}

float Differentiator::differentiate(float value, float timestamp)
{
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