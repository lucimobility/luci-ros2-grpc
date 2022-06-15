#pragma once

class Differentiator
{
  private:
    float previousTime = 0.0;
    float previousValue = 0.0;
    float previousDerivative = 0.0;

  public:
    float differentiate(float value, float timestamp);
    Differentiator();
    ~Differentiator();
};


