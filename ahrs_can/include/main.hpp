#include <rclcpp/logging.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <std_msgs/msg/float64.hpp>
#include "std_msgs/msg/string.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <unistd.h>
#include <memory>
#include <string>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time_source.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "can_msgs/msg/frame.hpp"

#define ACC 0x33
#define GYO 0x34
#define DEG 0x35
#define MAG 0x36

#define convertor_g2a 9.80665        // linear_acceleration (g to m/s^2)
#define convertor_d2r (M_PI / 180.0) // angular_velocity (degree to radian)
#define convertor_ut2t 1000000       // magnetic_field (uT to Tesla)
#define convertor_c 1.0              // temperature (celsius)

#define AHRS_ID 18                  // AHRS_DEVICE ID 임의지정

enum axis
{
  x = 0,
  y = 1,
  z = 2
};

static float acc_value[3] = {0,},gyr_value[3] = {0,},deg_value[3] = {0,},mag_value[3] = {0,};
static double linear_acceleration_stddev_, angular_velocity_stddev_, magnetic_field_stddev_, orientation_stddev_;

