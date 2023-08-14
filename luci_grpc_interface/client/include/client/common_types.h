/**
 * @file luci_grpc_types.h
 * @brief Struct definitions of LUCI standard data types
 * @date 2023-08-10
 *
 * @copyright Copyright (c) 2023 LUCI Mobility, Inc. All Rights Reserved.
 *
 */
#pragma once
#include <string>

/**
 * @brief LUCI joystick
 *
 */
struct SystemJoystick
{
    int forward_back;
    int left_right;

    inline SystemJoystick(int forward_back, int left_right)
        : forward_back(forward_back), left_right(left_right)
    {
    }
};

struct LuciJoystickScaling
{
    int forward_back;
    int left_right;
    std::string joystick_zone;

    inline LuciJoystickScaling(int forward_back, int left_right, std::string joystick_zone)
        : forward_back(forward_back), left_right(left_right), joystick_zone(joystick_zone)
    {
    }
};

/**
 * @brief LUCI scaling zones
 *
 */
struct LuciZoneScaling
{
    float front_fb;
    float front_rl;
    float front_right_fb;
    float front_right_rl;
    float front_left_fb;
    float front_left_rl;
    float right_fb;
    float right_rl;
    float left_fb;
    float left_rl;
    float back_right_fb;
    float back_right_rl;
    float back_left_fb;
    float back_left_rl;
    float back_fb;
    float back_rl;

    inline LuciZoneScaling(float front_fb, float front_rl, float front_right_fb, float front_right_rl,
                    float front_left_fb, float front_left_rl, float right_fb, float right_rl,
                    float left_fb, float left_rl, float back_right_fb, float back_right_rl,
                    float back_left_fb, float back_left_rl, float back_fb, float back_rl)
        : front_fb(front_fb), front_rl(front_rl), front_right_fb(front_right_fb),
          front_right_rl(front_right_rl), front_left_fb(front_left_fb),
          front_left_rl(front_left_rl), right_fb(right_fb), right_rl(right_rl), left_fb(left_fb),
          left_rl(left_rl), back_right_fb(back_right_fb), back_right_rl(back_right_rl),
          back_left_fb(back_left_fb), back_left_rl(back_left_rl), back_fb(back_fb), back_rl(back_rl)
    {
    }
};

/**
 * @brief LUCI AHRS data
 *
 */
struct AhrsInfo
{
    float linear_velocity_x;
    float linear_velocity_y;
    float linear_velocity_z;
    float angular_velocity_x;
    float angular_velocity_y;
    float angular_velocity_z;

    inline AhrsInfo(float linear_velocity_x, float linear_velocity_y, float linear_velocity_z,
             float angular_velocity_x, float angular_velocity_y, float angular_velocity_z)
        : linear_velocity_x(linear_velocity_x), linear_velocity_y(linear_velocity_y),
          linear_velocity_z(linear_velocity_z), angular_velocity_x(angular_velocity_x),
          angular_velocity_y(angular_velocity_y), angular_velocity_z(angular_velocity_z)
    {
    }
};