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
#include <vector>

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

    inline LuciZoneScaling(float front_fb, float front_rl, float front_right_fb,
                           float front_right_rl, float front_left_fb, float front_left_rl,
                           float right_fb, float right_rl, float left_fb, float left_rl,
                           float back_right_fb, float back_right_rl, float back_left_fb,
                           float back_left_rl, float back_fb, float back_rl)
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

/**
 * @brief LUCI IMU Data
 *
 */
struct ImuData
{
    float quaternion_x;
    float quaternion_y;
    float quaternion_z;
    float quaternion_w;

    float acceleration_x;
    float acceleration_y;
    float acceleration_z;

    float gyro_x;
    float gyro_y;
    float gyro_z;

    float euler_x;
    float euler_y;
    float euler_z;

    float accelerometer_x;
    float accelerometer_y;
    float accelerometer_z;

    float magnetometer_x;
    float magnetometer_y;
    float magnetometer_z;

    float gravity_x;
    float gravity_y;
    float gravity_z;

    int cal_system;
    int cal_gyroscope;
    int cal_accelerometer;
    int cal_magnetometer;

    int source;

    ImuData(float quaternion_x, float quaternion_y, float quaternion_z, float quaternion_w,
            float acceleration_x, float acceleration_y, float acceleration_z, float gyro_x,
            float gyro_y, float gyro_z, float euler_x, float euler_y, float euler_z,
            float accelerometer_x, float accelerometer_y, float accelerometer_z,
            float magnetometer_x, float magnetometer_y, float magnetometer_z, float gravity_x,
            float gravity_y, float gravity_z, int cal_system, int cal_gyroscope,
            int cal_accelerometer, int cal_magnetometer, int source)
        : quaternion_x(quaternion_x), quaternion_y(quaternion_y), quaternion_z(quaternion_z),
          quaternion_w(quaternion_w), acceleration_x(acceleration_x),
          acceleration_y(acceleration_y), acceleration_z(acceleration_z), gyro_x(gyro_x),
          euler_x(euler_x), euler_y(euler_y), euler_z(euler_z), accelerometer_x(accelerometer_x),
          accelerometer_y(accelerometer_y), accelerometer_z(accelerometer_z),
          magnetometer_x(magnetometer_x), magnetometer_y(magnetometer_y),
          magnetometer_z(magnetometer_z), gravity_x(gravity_x), gravity_y(gravity_y),
          gravity_z(gravity_z), source(source)
    {
    }
};

/**
 * @brief LUCI Encoder data
 *
 */
struct EncoderData
{
    float left_angle;
    float right_angle;
    float fl_caster_degrees;
    float bl_caster_degrees;
    float fr_caster_degrees;
    float br_caster_degrees;

    inline EncoderData(float left_angle, float right_angle, float fl_caster_degrees,
                       float bl_caster_degrees, float fr_caster_degrees, float br_caster_degrees)
        : left_angle(left_angle), right_angle(right_angle), fl_caster_degrees(fl_caster_degrees),
          bl_caster_degrees(bl_caster_degrees), fr_caster_degrees(fr_caster_degrees),
          br_caster_degrees(br_caster_degrees)
    {
    }
};

/**
 * @brief Luci Ir Camera Data
 *
 * @tparam Width
 * @tparam Height
 */
template <int32_t Width, int32_t Height> struct CameraIrData
{
    int width;
    int height;
    std::vector<uint8_t> data;

    inline CameraIrData(int width, int height, std::vector<uint8_t> data)
        : width(width), height(height), data(data)
    {
    }
};