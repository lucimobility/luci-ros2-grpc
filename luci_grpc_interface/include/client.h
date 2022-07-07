/**
 * @file client.h
 * @author Luci Ramp Assist team
 * @brief
 * @version 0.1
 * @date 2021-06-01
 *
 * @copyright Copyright (c) 2021
 *
 */

#ifndef LUCI_PTOLEMY_CLIENT_H_
#define LUCI_PTOLEMY_CLIENT_H_

#include <grpc/grpc.h>
#include <grpcpp/grpcpp.h>
#include <grpcpp/security/server_credentials.h>
#include <grpcpp/server.h>
#include <grpcpp/server_builder.h>
#include <grpcpp/server_context.h>

#include "../generated_code/client_library/ptolemy.grpc.pb.h"
#include <opencv2/opencv.hpp>

// #include <pcl/common/transforms.h>
// #include <pcl/filters/passthrough.h>
// #include <pcl/filters/statistical_outlier_removal.h>
// #include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
// #include <pcl/visualization/cloud_viewer.h>

// #include <//spdlog///spdlog.h>

#include <chrono>
#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

using grpc::Channel;
using grpc::ClientContext;
using grpc::ClientReader;
using grpc::Status;
using sensors::AhrsData;
using sensors::CameraPoints;
using sensors::ChairSpeed;
using sensors::EncoderData;
using sensors::ImuCtrl;
using sensors::ImuData;
using sensors::IrFrame;
using sensors::Joystick;
using sensors::ModeCtrl;
using sensors::RadarPoints;
using sensors::RemoteJsValues;
using sensors::Response;
using sensors::UltrasonicDistances;

// Number of frames in a row, per camera, without a ramp detected before the the camera is switched
#define NO_RAMP_IN_FRAME_THRESHOLD_RAMP_MODE_ON 3
#define NO_RAMP_IN_FRAME_THRESHOLD_RAMP_MODE_OFF 1

// NUmber of frames in a row without a detected ramp before ramp assist is deactivated
#define DISABLE_THRESHOLD 8

#define OVERRIDE_PRESS_COUNT 1

#define UNIQUE_IDS 100

// Depending on ramp mode status, process only a certain amount of frames per second
#define FRAME_RATE_RAMP_MODE_ON 3   // 5 fps
#define FRAME_RATE_RAMP_MODE_OFF 15 // 1 fps
#define NANO_SEC 1000000000

enum CameraSource
{
    LEFT_CAMERA,
    RIGHT_CAMERA,
    BACK_CAMERA
};

// Using OpenCV matrix
using namespace cv;

class ClientGuide
{
  public:
    ClientGuide(std::shared_ptr<Channel> channel) : stub_(sensors::Sensors::NewStub(channel)) {}

    std::unique_ptr<sensors::Sensors::Stub> stub_;
    CameraSource defaultCamera = RIGHT_CAMERA;

    struct RemoteCamera
    {
        // Camera
        int frameSize;
        int frameWidth;
        int frameHeight;
        CameraSource sourceCamera;
        Mat recievedMatrix;
        long long timestamp;
        int uniqueId = 0;
    };

    struct IMU
    {
        double euler_z = 0.0;
        double euler_x = 0.0;
        double acceleration_y = 0.0;
        double accelerometer_y = 0.0;
        double deltaTimeMs = 0.0;
        double lastTimeSec = 0.0;
        double lastTimeNano = 0.0;
    };

    struct Encoder
    {
        float leftAngle = 0.0;
        float rightAngle = 0.0;
        float timestamp = 0.0;
    };

    struct Vec3
    {
        float x = 0.0;
        float y = 0.0;
        float z = 0.0;
    };

    struct AHRS
    {
        Vec3 linear_velocity;
        Vec3 angular_velocity;
    };

    AHRS ahrsData;
    Encoder encoderData;
    double speedLastTimeSec = 0.0;
    double speedLastTimeNano = 0.0;
    double speedDeltaTimeMs = 0.0;

    int luciForwardBack;
    int luciLeftRight;

    RemoteCamera remoteCamera;
    IMU imu;

    pcl::PointCloud<pcl::PointXYZ> cameraPointCloud;
    pcl::PointCloud<pcl::PointXYZ> ultrasonicPointCloud;
    pcl::PointCloud<pcl::PointXYZ> radarPointCloud;

    float chairSpeed = 0.0;

    // Mutexs
    mutable std::mutex joystickMutex;
    mutable std::mutex irMutex;
    mutable std::mutex imuMutex;
    mutable std::mutex speedMutex;
    mutable std::mutex ahrsMutex;
    mutable std::mutex pclMutex;
    mutable std::mutex encoderMutex;
    mutable std::mutex ultrasonicMutex;
    mutable std::mutex radarMutex;

    std::condition_variable irConditionVariable;

    std::chrono::time_point<std::chrono::high_resolution_clock> lastJSTime = std::chrono::high_resolution_clock::now();
    std::chrono::time_point<std::chrono::high_resolution_clock> lastFrameTime = std::chrono::high_resolution_clock::now();

    // Single calls over gRPC

    /**
     * @brief Interface to turn on engaged mode at the msp level
     *
     * @return bool success code (true-success, false-failure)
     */
    bool activateEngagedMode()
    {
        ClientContext context;
        Response response;
        ModeCtrl request;
        request.set_mode(sensors::DriveMode::ENGAGED);
        Status status = stub_->SetDriveMode(&context, request, &response);
        if (status.ok())
        {
            // //spdlog::warn(response.reply());
        }
        else
        {
            return false;
        }
        return true;
    }

    /**
     * @brief Interface to turn on user mode at the msp level
     *
     * @return bool success code (true-success, false-failure)
     */
    bool activateUserMode()
    {
        ClientContext context;
        Response response;
        ModeCtrl request;
        request.set_mode(sensors::DriveMode::USER);
        Status status = stub_->SetDriveMode(&context, request, &response);
        if (status.ok())
        {
            // spdlog::warn(response.reply());
        }
        else
        {
            return false;
        }
        return true;
    }

    /**
     * @brief Interface to turn on auto mode at the msp level
     *
     * @return bool success code (true-success, false-failure)
     */
    bool activateAutoMode()
    {
        ClientContext context;
        Response response;
        ModeCtrl request;
        request.set_mode(sensors::DriveMode::AUTONOMOUS);
        Status status = stub_->SetDriveMode(&context, request, &response);
        if (status.ok())
        {
            // spdlog::warn(response.reply());
        }
        else
        {
            return false;
        }
        return true;
    }

    /**
     * @brief Send JS values to luci sensors
     *
     * @param forwardBack
     * @param leftRight
     * @return int
     */
    int sendJS(int forwardBack, int leftRight)
    {
        // The forwardBack and the leftRight should be between 0-200 (100 being zero)
        ClientContext context;
        Response response;
        RemoteJsValues request;

        request.set_left_right(leftRight);
        request.set_forward_back(forwardBack);

        auto currentTime = std::chrono::high_resolution_clock::now();

        Status status = stub_->JsOverride(&context, request, &response);
        if (status.ok())
        {
            // spdlog::info("Sending remote call... values ({} {}) status: {}", std::to_string(forwardBack), std::to_string(leftRight),
            //  response.reply());
        }
        else
        {
            // spdlog::error("Error communicating with server... pass in correct ip and port of chair");
            return 1;
        }
        // spdlog::debug("SEND JS: {}", std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - lastJSTime).count());
        lastJSTime = currentTime;
        return 0;
    }

    // Readers

    /**
     * @brief Read the joystick position
     *
     * @todo CLP Timestamp this stuff real good
     *
     */
    void readJoystickPosition()
    {
        ClientContext context;
        const google::protobuf::Empty request;
        Joystick response;
        std::unique_ptr<ClientReader<Joystick>> reader(stub_->JoystickStream(&context, request));
        reader->Read(&response);
        while (reader->Read(&response))
        {
            std::lock_guard<std::mutex> jsLock(joystickMutex);

            this->luciForwardBack = response.forward_back();
            this->luciLeftRight = response.left_right();
        }
    }

    void readAhrsData()
    {
        ClientContext context;
        const google::protobuf::Empty request;
        AhrsData response;
        std::unique_ptr<ClientReader<AhrsData>> reader(stub_->AhrsStream(&context, request));
        reader->Read(&response);
        while (reader->Read(&response))
        {
            std::lock_guard<std::mutex> ahrsLock(ahrsMutex);
            // ROS has x where our y axis is
            this->ahrsData.linear_velocity.x = response.linear_velocity().x();
            this->ahrsData.linear_velocity.y = response.linear_velocity().y();
            this->ahrsData.angular_velocity.z = response.angular_velocity().z();
        }
    }

    void readCameraPointData()
    {
        ClientContext context;
        const google::protobuf::Empty request;
        CameraPoints response;
        std::unique_ptr<ClientReader<CameraPoints>> reader(stub_->CameraStream(&context, request));
        reader->Read(&response);
        while (reader->Read(&response))
        {
            std::cout << "RECIEVED" << std::endl;
            std::lock_guard<std::mutex> cameraPointLock(pclMutex);
            this->cameraPointCloud.clear();
            // Pack into pcl object
            // transformedPointClouds->points.push_back(transformedPoint);
            for (auto point : response.points())
            // for (int i = 0; i < response.points().size() / 5; i++)
            {
                // auto point = response.points().at(i);
                // std::cout << "POITN" << std::endl;
                pcl::PointXYZ pclPoint;
                pclPoint.x = point.x();
                pclPoint.y = point.y();
                pclPoint.z = point.z();
                this->cameraPointCloud.points.push_back(pclPoint);
            }
        }
    }

    void readUltrasonicPointData()
    {
        ClientContext context;
        const google::protobuf::Empty request;
        UltrasonicDistances response;
        std::unique_ptr<ClientReader<UltrasonicDistances>> reader(stub_->UltrasonicStream(&context, request));
        reader->Read(&response);
        while (reader->Read(&response))
        {
            // std::cout << "RECIEVED" << std::endl;
            std::lock_guard<std::mutex> ultrasonicPointLock(ultrasonicMutex);
            this->ultrasonicPointCloud.clear();
            // Pack into pcl object
            for (auto distance : response.distances())
            {
                for (auto point : distance.arc_points())
                {
                    pcl::PointXYZ pclPoint;
                    pclPoint.x = point.x();
                    pclPoint.y = point.y();
                    pclPoint.z = point.z();
                    this->ultrasonicPointCloud.points.push_back(pclPoint);
                }
            }
        }
    }
    void readRadarPointData()
    {
        ClientContext context;
        const google::protobuf::Empty request;
        RadarPoints response;
        std::unique_ptr<ClientReader<RadarPoints>> reader(stub_->RadarStream(&context, request));
        reader->Read(&response);
        while (reader->Read(&response))
        {
            // std::cout << "RECIEVED" << std::endl;
            std::lock_guard<std::mutex> radarPointLock(radarMutex);
            this->radarPointCloud.clear();

            for (auto point : response.points())
            {
                pcl::PointXYZ pclPoint;
                pclPoint.x = point.x();
                pclPoint.y = point.y();
                pclPoint.z = point.z();
                this->radarPointCloud.points.push_back(pclPoint);
            }
        }
    }

    void readEncoderData()
    {
        ClientContext context;
        const google::protobuf::Empty request;
        EncoderData response;
        std::unique_ptr<ClientReader<EncoderData>> reader(stub_->EncoderStream(&context, request));

        reader->Read(&response);
        while (reader->Read(&response))
        {

            std::lock_guard<std::mutex> encoderLock(encoderMutex);
            this->encoderData.leftAngle = response.left_angle();
            this->encoderData.rightAngle = response.right_angle();
            this->encoderData.timestamp = response.timestamp().seconds() + response.timestamp().nanos() / 1000000000.0;
        }
    }

    void readImuData()
    {
        ClientContext context;
        ImuCtrl request;
        request.set_imu(sensors::Imu::MPU);
        ImuData response;

        std::unique_ptr<ClientReader<ImuData>> reader(stub_->ImuStream(&context, request));
        reader->Read(&response);
        while (reader->Read(&response))
        {

            std::lock_guard<std::mutex> imuLock(imuMutex);
            this->imu.euler_z = response.euler_z();
            this->imu.euler_x = response.euler_x();
            this->imu.acceleration_y = response.acceleration_y();
            this->imu.accelerometer_y = response.accelerometer_y();

            auto secDelta = this->imu.lastTimeSec - response.timestamp().seconds();

            if (secDelta != 0)
            {
                // Handle new second
                auto delta = (NANO_SEC - this->imu.lastTimeNano);
                delta += (NANO_SEC * (secDelta - 1));
                delta += response.timestamp().nanos();
                this->imu.deltaTimeMs = delta / 1000000;
            }
            else
            {
                this->imu.deltaTimeMs = (response.timestamp().nanos() - this->imu.lastTimeNano) / 1000000;
            }

            this->imu.lastTimeSec = response.timestamp().seconds();
            this->imu.lastTimeNano = response.timestamp().nanos();
        }
    }

    void readChairSpeed()
    {
        ClientContext context;
        const google::protobuf::Empty request;
        ChairSpeed response;

        std::unique_ptr<ClientReader<ChairSpeed>> reader(stub_->ChairSpeedStream(&context, request));
        reader->Read(&response);
        while (reader->Read(&response))
        {
            std::lock_guard<std::mutex> speedLock(speedMutex);
            this->chairSpeed = response.speed_m_p_s();
            auto secDelta = this->speedLastTimeSec - response.timestamp().seconds();

            if (secDelta != 0)
            {
                // Handle new second
                auto delta = (NANO_SEC - this->speedLastTimeNano);
                delta += (NANO_SEC * (secDelta - 1));
                delta += response.timestamp().nanos();
                this->speedDeltaTimeMs = delta / 1000000;
            }
            else
            {
                this->speedDeltaTimeMs = (response.timestamp().nanos() - this->speedLastTimeNano) / 1000000;
            }
            this->speedLastTimeSec = response.timestamp().seconds();
            this->speedLastTimeNano = response.timestamp().nanos();
        }
    }

    /**
     * @brief Read the IR frame
     *
     */
    // void readIrFrame()
    // {
    //     ClientContext context;
    //     const google::protobuf::Empty request;
    //     IrFrame response;
    //     std::unique_ptr<ClientReader<IrFrame>> reader(stub_->IrStream(&context, request));
    //     reader->Read(&response);
    //     int id = 0;

    //     std::map<std::string, int> counters;
    //     counters["left"] = FRAME_RATE_RAMP_MODE_OFF;
    //     counters["right"] = FRAME_RATE_RAMP_MODE_OFF;

    //     while (reader->Read(&response))
    //     {
    //         if (response.size() > 0)
    //         {

    //             counters[response.camera()]--;

    //             if ((((response.camera() == "left") && (defaultCamera == LEFT_CAMERA)) ||
    //                  ((response.camera() == "right") && (defaultCamera == RIGHT_CAMERA))) &&
    //                 counters[response.camera()] <= 0)
    //             {
    //                 if (rampMode)
    //                 {
    //                     counters[response.camera()] = FRAME_RATE_RAMP_MODE_ON;
    //                 }
    //                 else
    //                 {
    //                     counters[response.camera()] = FRAME_RATE_RAMP_MODE_OFF;
    //                 }

    //                 std::lock_guard<std::mutex> irLock(irMutex);

    //                 if (response.camera() == "left")
    //                 {
    //                     remoteCamera.sourceCamera = LEFT_CAMERA;
    //                 }
    //                 else if (response.camera() == "right")
    //                 {
    //                     remoteCamera.sourceCamera = RIGHT_CAMERA;
    //                 }

    //                 // Make a new pointer to an array of bytes of image size
    //                 auto& frame = response.frame();

    //                 std::vector<uint8_t> bytes(frame.begin(), frame.end());

    //                 remoteCamera.frameWidth = response.width();
    //                 remoteCamera.frameHeight = response.height();

    //                 remoteCamera.recievedMatrix = Mat(remoteCamera.frameHeight, remoteCamera.frameWidth, CV_8UC1, bytes.data()).clone();
    //                 remoteCamera.uniqueId = id;
    //                 id++;
    //                 if (id > UNIQUE_IDS)
    //                 {
    //                     id = 0;
    //                 }
    //                 irConditionVariable.notify_all();
    //             }
    //         }
    //         else
    //         {
    //             // spdlog::warn("Skipping frame from client ");
    //         }
    //     }
    // }

    // Getters

    /**
     * @brief Get the Joystick Position object
     *
     * @return int
     */
    int getJoystickPosition() const
    {
        std::lock_guard<std::mutex> jsLock(joystickMutex);
        return luciForwardBack;
    }

    const IMU getImuData() const
    {
        std::lock_guard<std::mutex> imuLock(imuMutex);
        return this->imu;
    }

    const Encoder getEncoderData() const
    {
        std::lock_guard<std::mutex> encoderLock(encoderMutex);
        return this->encoderData;
    }

    const AHRS getAhrsData() const
    {
        std::lock_guard<std::mutex> ahrsLock(ahrsMutex);
        return this->ahrsData;
    }

    const pcl::PointCloud<pcl::PointXYZ> getCameraPointCloud() const
    {
        std::lock_guard<std::mutex> cameraPointLock(pclMutex);
        return this->cameraPointCloud;
    }

    const pcl::PointCloud<pcl::PointXYZ> getUltrasonicPointCloud() const
    {
        std::lock_guard<std::mutex> ultrasonicPointLock(ultrasonicMutex);
        return this->ultrasonicPointCloud;
    }
    const pcl::PointCloud<pcl::PointXYZ> getRadarPointCloud() const
    {
        std::lock_guard<std::mutex> radarPointLock(radarMutex);
        return this->radarPointCloud;
    }

    float getChairSpeed() const
    {
        std::lock_guard<std::mutex> speedLock(speedMutex);
        return this->chairSpeed;
    }

    /**
     * @brief Get the Ir Frame object
     *
     * @return const RemoteCamera&
     */
    const RemoteCamera& getIrFrame() const { return remoteCamera; }

    /**
     * @brief Get the Raw JS Position object
     *
     * @return std::vector<int>
     */
    std::vector<int> getRawJSPosition() const
    {
        std::lock_guard<std::mutex> jsLock(joystickMutex);
        std::vector<int> values = {luciForwardBack, luciLeftRight};
        return values;
    }
};

#endif