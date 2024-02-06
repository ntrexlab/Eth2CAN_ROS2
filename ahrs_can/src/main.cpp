#include "main.hpp"

class AHRSNode : public rclcpp::Node
{
public:
  AHRSNode(): Node("ahrs_can_node")
  {
    declare_parameter<double>("linear_acceleration_stddev", 0.02);
    get_parameter("linear_acceleration_stddev", linear_acceleration_stddev_);

    declare_parameter<double>("angular_velocity_stddev", 0.01);
    get_parameter("angular_velocity_stddev", angular_velocity_stddev_);

    declare_parameter<double>("magnetic_field_stddev", 0.00000327486);
    get_parameter("magnetic_field_stddev", magnetic_field_stddev_);

    declare_parameter<double>("orientation_stddev", 0.00125);
    get_parameter("orientation_stddev", orientation_stddev_);

    rclcpp::QoS qos(10);
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE); // 신뢰성 활성화
    qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);   // 내구성 활성화

    cmd_subscription_ = create_subscription<std_msgs::msg::String>("/ahrs_can_cmd",10,[this](const std_msgs::msg::String::SharedPtr msg) {handleCmdMessage(msg);});

    subscriber_ = create_subscription<can_msgs::msg::Frame>("/eth2can_recv",  qos,  std::bind(&AHRSNode::ahrsDataCallback, this, std::placeholders::_1));
    publisher_ = create_publisher<can_msgs::msg::Frame>("/eth2can_send", qos);

    imu_data_raw_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", qos);
    imu_data_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", qos);
    imu_mag_pub_ = this->create_publisher<sensor_msgs::msg::MagneticField>("imu/mag", qos);
    imu_yaw_pub_ = this->create_publisher<std_msgs::msg::Float64>("imu/yaw", qos);
  }

private:
  tf2::Quaternion Euler2Quaternion(float roll, float pitch, float yaw)
  {
    float qx = (sin(roll / 2) * cos(pitch / 2) * cos(yaw / 2)) -
               (cos(roll / 2) * sin(pitch / 2) * sin(yaw / 2));
    float qy = (cos(roll / 2) * sin(pitch / 2) * cos(yaw / 2)) +
               (sin(roll / 2) * cos(pitch / 2) * sin(yaw / 2));
    float qz = (cos(roll / 2) * cos(pitch / 2) * sin(yaw / 2)) -
               (sin(roll / 2) * sin(pitch / 2) * cos(yaw / 2));
    float qw = (cos(roll / 2) * cos(pitch / 2) * cos(yaw / 2)) +
               (sin(roll / 2) * sin(pitch / 2) * sin(yaw / 2));

    tf2::Quaternion q(qx, qy, qz, qw);
    return q;
  }

   void handleCmdMessage(const std_msgs::msg::String::SharedPtr msg) {
    //RCLCPP_INFO(get_logger(), "Received command: %s", msg->data.c_str());
    if (msg->data == "reset") {
      can_msgs::msg::Frame can_msg;

      can_msg.id = AHRS_ID; 
      can_msg.dlc = 8; 
      can_msg.extended = false; 
      
      can_msg.data[0] = 0x18;  
      can_msg.data[1] = 0x07;  
      can_msg.data[2] = 0x00; 
      can_msg.data[3] = 0x00;
      can_msg.data[4] = 0x05;
      can_msg.data[5] = 0x00;
      can_msg.data[6] = 0x00;
      can_msg.data[7] = 0x00;

    /*

      [0] - Command
      [1] - Index
      [2] - Index
      [3] - Sub-index
      [4] ~ [7] - value
      
    */
      
     publisher_->publish(can_msg);  //eth2can_send 로 보내기
    }
  }

  void ahrsDataCallback(const can_msgs::msg::Frame::SharedPtr msg)
  {
    if (msg->id == AHRS_ID) 
    {
      switch ((int)(unsigned char)msg->data[1])
      {
      case ACC:
        acc_value[axis::x] = (int16_t)(((int)(unsigned char)msg->data[2] | (int)(unsigned char)msg->data[3] << 8)) / 1000.0;
        acc_value[axis::y] = (int16_t)(((int)(unsigned char)msg->data[4] | (int)(unsigned char)msg->data[5] << 8)) / 1000.0;
        acc_value[axis::z] = (int16_t)(((int)(unsigned char)msg->data[6] | (int)(unsigned char)msg->data[7] << 8)) / 1000.0;
        break;

      case GYO:
        gyr_value[axis::x] = (int16_t)(((int)(unsigned char)msg->data[2] | (int)(unsigned char)msg->data[3] << 8)) / 10.0;
        gyr_value[axis::y] = (int16_t)(((int)(unsigned char)msg->data[4] | (int)(unsigned char)msg->data[5] << 8)) / 10.0;
        gyr_value[axis::z] = (int16_t)(((int)(unsigned char)msg->data[6] | (int)(unsigned char)msg->data[7] << 8)) / 10.0;
        break;

      case DEG:
        deg_value[axis::x] = (int16_t)(((int)(unsigned char)msg->data[2] | (int)(unsigned char)msg->data[3] << 8)) / 100.0;
        deg_value[axis::y] = (int16_t)(((int)(unsigned char)msg->data[4] | (int)(unsigned char)msg->data[5] << 8)) / 100.0;
        deg_value[axis::z] = (int16_t)(((int)(unsigned char)msg->data[6] | (int)(unsigned char)msg->data[7] << 8)) / 100.0;
        break;

      case MAG:
        mag_value[axis::x] = (int16_t)(((int)(unsigned char)msg->data[2] | (int)(unsigned char)msg->data[3] << 8)) / 10.0;
        mag_value[axis::y] = (int16_t)(((int)(unsigned char)msg->data[4] | (int)(unsigned char)msg->data[5] << 8)) / 10.0;
        mag_value[axis::z] = (int16_t)(((int)(unsigned char)msg->data[6] | (int)(unsigned char)msg->data[7] << 8)) / 10.0;
        break;
      }

      publish_AHRS_Topic();
    }

    /*
     RCLCPP_INFO(this->get_logger(), "Received AHRS Data - ID: %d, DLC: %d, Data: %02X %02X %02X %02X %02X %02X %02X %02X",
       msg->id, msg->dlc,
       msg->data[0], msg->data[1], msg->data[2], msg->data[3],
       msg->data[4], msg->data[5], msg->data[6], msg->data[7]);
   */
  }

  void publish_AHRS_Topic()
  {
    auto imu_data_raw_msg = sensor_msgs::msg::Imu();
    auto imu_data_msg = sensor_msgs::msg::Imu();
    auto imu_magnetic_msg = sensor_msgs::msg::MagneticField();
    auto imu_yaw_msg = std_msgs::msg::Float64();

    double linear_acceleration_cov = linear_acceleration_stddev_ * linear_acceleration_stddev_;
    double angular_velocity_cov = angular_velocity_stddev_ * angular_velocity_stddev_;
    double magnetic_field_cov = magnetic_field_stddev_ * magnetic_field_stddev_;
    double orientation_cov = orientation_stddev_ * orientation_stddev_;

    imu_data_raw_msg.linear_acceleration_covariance[0] =
    imu_data_raw_msg.linear_acceleration_covariance[4] =
    imu_data_raw_msg.linear_acceleration_covariance[8] =
    imu_data_msg.linear_acceleration_covariance[0] =
    imu_data_msg.linear_acceleration_covariance[4] =
    imu_data_msg.linear_acceleration_covariance[8] =
    linear_acceleration_cov;

    imu_data_raw_msg.angular_velocity_covariance[0] =
    imu_data_raw_msg.angular_velocity_covariance[4] =
    imu_data_raw_msg.angular_velocity_covariance[8] =
    imu_data_msg.angular_velocity_covariance[0] =
    imu_data_msg.angular_velocity_covariance[4] =
    imu_data_msg.angular_velocity_covariance[8] =
    angular_velocity_cov;

    imu_data_msg.orientation_covariance[0] =
    imu_data_msg.orientation_covariance[4] =
    imu_data_msg.orientation_covariance[8] = 
    orientation_cov;

    imu_magnetic_msg.magnetic_field_covariance[0] =
    imu_magnetic_msg.magnetic_field_covariance[4] =
    imu_magnetic_msg.magnetic_field_covariance[8] = 
    magnetic_field_cov;

    double roll, pitch, yaw;

    roll = deg_value[axis::x] * convertor_d2r;
    pitch = mag_value[axis::y] * convertor_d2r;
    yaw = deg_value[axis::z] * convertor_d2r;

    tf2::Quaternion tf_orientation = Euler2Quaternion(roll, pitch, yaw);

    rclcpp::Time now = this->now();

    imu_data_raw_msg.header.stamp = imu_data_msg.header.stamp =
        imu_magnetic_msg.header.stamp = now;

    imu_data_raw_msg.header.frame_id = imu_data_msg.header.frame_id =
        imu_magnetic_msg.header.frame_id = frame_id_;

    // orientation
    imu_data_msg.orientation.x = tf_orientation.x();
    imu_data_msg.orientation.y = tf_orientation.y();
    imu_data_msg.orientation.z = tf_orientation.z();
    imu_data_msg.orientation.w = tf_orientation.w();

    // original data used the g unit, convert to m/s^2
    imu_data_raw_msg.linear_acceleration.x = imu_data_msg.linear_acceleration.x =
        acc_value[axis::x] * convertor_g2a;
    imu_data_raw_msg.linear_acceleration.y = imu_data_msg.linear_acceleration.y =
        acc_value[axis::y] * convertor_g2a;
    imu_data_raw_msg.linear_acceleration.z = imu_data_msg.linear_acceleration.z =
        acc_value[axis::z] * convertor_g2a;

    // original data used the degree/s unit, convert to radian/s
    imu_data_raw_msg.angular_velocity.x = imu_data_msg.angular_velocity.x =
        gyr_value[axis::x] * convertor_d2r;
    imu_data_raw_msg.angular_velocity.y = imu_data_msg.angular_velocity.y =
        gyr_value[axis::y] * convertor_d2r;
    imu_data_raw_msg.angular_velocity.z = imu_data_msg.angular_velocity.z =
        gyr_value[axis::z] * convertor_d2r;

    // original data used the uTesla unit, convert to Tesla
    imu_magnetic_msg.magnetic_field.x = mag_value[axis::x] / convertor_ut2t;
    imu_magnetic_msg.magnetic_field.y = mag_value[axis::y] / convertor_ut2t;
    imu_magnetic_msg.magnetic_field.z = mag_value[axis::z] / convertor_ut2t;

    // original data used the celsius unit
    imu_yaw_msg.data = deg_value[axis::z]; 

    // publish the IMU data
    imu_data_raw_pub_->publish(std::move(imu_data_raw_msg));
    imu_data_pub_->publish(std::move(imu_data_msg));
    imu_mag_pub_->publish(std::move(imu_magnetic_msg));
    imu_yaw_pub_->publish(std::move(imu_yaw_msg));

    // publish tf
    if (publish_tf_)
    {
      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp = now;
      tf.header.frame_id = parent_frame_id_;
      tf.child_frame_id = frame_id_;
      tf.transform.translation.x = 0.0;
      tf.transform.translation.y = 0.0;
      tf.transform.translation.z = 0.0;
      tf.transform.rotation = imu_data_msg.orientation;

      broadcaster_->sendTransform(tf);
    }
  }

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_data_raw_pub_, imu_data_pub_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr imu_mag_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr imu_yaw_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr subscriber_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_subscription_;
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr publisher_;
  
  bool publish_tf_ = false;
  std::string parent_frame_id_;
  std::string frame_id_;

  double linear_acceleration_stddev_, angular_velocity_stddev_, magnetic_field_stddev_, orientation_stddev_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AHRSNode>());
  rclcpp::shutdown();
  return 0;
}
