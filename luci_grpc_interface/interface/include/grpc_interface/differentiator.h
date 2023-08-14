/**
 * @file differentiator.h
 * @brief Math helper to differentiate values
 * @date 2023-08-10
 *
 * @copyright Copyright (c) 2023 LUCI Mobility, Inc. All Rights Reserved.
 */

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
