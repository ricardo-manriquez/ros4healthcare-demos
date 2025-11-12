#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/u_int16.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include "tf2_ros/transform_broadcaster.h"
#define _USE_MATH_DEFINES
#include <math.h>

#define SAMPLE_RATE (100)

#include <sys/ioctl.h>
extern "C" {
	#include <hidapi/hidapi.h>
}

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>

#include <sys/types.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <fcntl.h>

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

class TreadmillController : public rclcpp::Node {
    public:
        TreadmillController(hid_device* m_hid) : Node("treadmill") {
            mload_publisher = this->create_publisher<std_msgs::msg::UInt16>("mload", 10);

            jointstate_publisher = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 50);
            tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
            odom_publisher = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

            maxspeed_ = this->create_subscription<geometry_msgs::msg::Twist>(
                "max_speed", 10, std::bind(&TreadmillController::speed_callback, this, _1)
            );
            cmdvel_ = this->create_publisher<geometry_msgs::msg::Twist>(
                "cmd_vel", 10);
            timer_ = this->create_wall_timer(
                8ms, std::bind(&TreadmillController::timer_callback, this)
            );

            reset_odom_service = this->create_service<std_srvs::srv::Trigger>("reset_odom", std::bind(&TreadmillController::reset_odom, this, _1, _2));

            hid = m_hid;
            //Set auto mode
            uint8_t data0[10];
            data0[0] = 0;
            data0[1] = 0;
            data0[2] = 0;
            data0[3] = 0;
            data0[4] = 0;
            data0[5] = 0;
            data0[6] = 0;
            data0[7] = 0;
            data0[8] = 2;

            hid_write(hid, data0, 9);

        }
        ~TreadmillController() {
            //TURN Off Everything
            uint8_t data0[10];
            data0[0] = 0;
            data0[1] = 0;
            data0[2] = 0;
            data0[3] = 0;
            data0[4] = 0;
            data0[5] = 0;
            data0[6] = 0;
            data0[7] = 0;
            data0[8] = 2;

            hid_write(hid, data0, 9);
            rclcpp::Node::~Node();
        }
        void publish_value() {
            auto message = std_msgs::msg::String();
            message.data = "Recv CAN frame";
            RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
            //publisher_->publish(message);
        }
        void reset_odom(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                        const std::shared_ptr<std_srvs::srv::Trigger::Response> response
                ) {
            uint8_t data0[10];
            data0[0] = 0;
            data0[1] = 0;
            data0[2] = 0;
            data0[3] = 0;
            data0[4] = 0;
            data0[5] = 0;
            data0[6] = 0;
            data0[7] = 0;
            data0[8] = 3;

            hid_write(hid, data0, 9);

            odom_pose[0] = 0.0f;
            odom_pose[1] = 0.0f;
            odom_pose[2] = 0.0f;

            last_enc = 0;

            response->success = true;
            response->message = "Odom reset";
        }
        enum {
            MOTOR = 0,
            RELAY = 1,
            RESET_ODOM = 2,
        };
    private:
        void timer_callback() {
            auto stamp = this->now();
            auto odom = nav_msgs::msg::Odometry();
            auto m_tf = geometry_msgs::msg::TransformStamped();
            auto jointstate = sensor_msgs::msg::JointState();

            odom.header.stamp = stamp;
            odom.header.frame_id = "odom";
            odom.child_frame_id = "base_link";

            m_tf.header.stamp = stamp;
            m_tf.header.frame_id = "odom";
            m_tf.child_frame_id = "base_link";

            jointstate.header.stamp = stamp;
            jointstate.header.frame_id = "joint_state";

            jointstate.name.push_back("base_to_right_wheel");
            jointstate.name.push_back("base_to_left_wheel");
            //jointstate.name.push_back("base_angle");

            auto mload = std_msgs::msg::UInt16();
            auto m_twist = geometry_msgs::msg::Twist();
            unsigned char buf[10];
            uint8_t res = hid_read(hid, buf, 9);
            if (res != 0) {
                int16_t m_enc = (buf[0] + (buf[1] << 8));
                mload.data = buf[2] + (buf[3] << 8);
                int16_t m_speed = (buf[4] + (buf[5] << 8));

                int32_t diff=0;
                diff = m_enc - last_enc;

                const int32_t MLIM = INT16_MAX + 1;

                if (diff > INT16_MAX) {
                    diff = ((m_enc - MLIM) - ( last_enc + MLIM));
                } else if (diff < INT16_MIN) {
                    diff = ((m_enc + MLIM) - ( last_enc - MLIM));
                }

                double drot_w = diff * 1.0f/(6.0f*M_PI*39.5); // 1200.0 counts/turn 20:1 14PPR Encoder

                change_w = diff / 6000.0f;  // Distance in meters

                rotation_w += drot_w; // Accumulated wheel position

                last_enc = m_enc;

                odom_pose[0] += change_w;
                odom_pose[1] = 0;
                odom_pose[2] = 0;

                double step_time = (stamp - last_time).seconds();
                last_time = stamp;
//                RCLCPP_INFO(this->get_logger(), "D: %i C: %f T: %f", diff, change_w, step_time);
                jointstate.position.push_back(rotation_w); // Right
                //jointstate.position.push_back(odom_pose[2]); // base angle

                jointstate.velocity.push_back(drot_w/step_time); // Left
                //jointstate.velocity.push_back(change_yaw / step_time);

                odom.twist.twist.linear.x  = (m_speed / 600.0f);
                odom.twist.twist.linear.y  = 0.0f;
                odom.twist.twist.linear.z  = 0.0f;
                odom.twist.twist.angular.x = 0.0f;
                odom.twist.twist.angular.y = 0.0f;
                odom.twist.twist.angular.z = 0.0f;

                m_twist.linear.x = (m_speed / 600.0f);
                cmdvel_->publish(m_twist);

                odom.pose.pose.position.x = odom_pose[0];
                odom.pose.pose.position.y = odom_pose[1];
                odom.pose.pose.position.z = 0.0f;
                tf2::Quaternion q;
                q.setRPY(0, 0, odom_pose[2]);
                odom.pose.pose.orientation.x = q.x();
                odom.pose.pose.orientation.y = q.y();
                odom.pose.pose.orientation.z = q.z();
                odom.pose.pose.orientation.w = q.w();

                m_tf.transform.translation.x = odom_pose[0];
                m_tf.transform.translation.y = odom_pose[1];
                m_tf.transform.translation.z = 0.0f;

                m_tf.transform.rotation.x = q.x();
                m_tf.transform.rotation.y = q.y();
                m_tf.transform.rotation.z = q.z();
                m_tf.transform.rotation.w = q.w();

                mload_publisher->publish(mload);

                jointstate_publisher->publish(jointstate);
                odom_publisher->publish(odom);
                tf_broadcaster->sendTransform(m_tf);

            } else {

            }

        }

        void speed_callback(const geometry_msgs::msg::Twist & message) {
            const int speed = (int) abs(600*message.linear.x); // Treadmill in m/min multiplied by 10

            uint8_t data0[10];
            data0[0] = 0;
            data0[1] = speed & 0xff;
            data0[2] = (speed & 0xff00) >> 8;
            data0[3] = 0;
            data0[4] = 0;
            data0[5] = 0;
            data0[6] = 0;
            data0[7] = 0;
            data0[8] = 0;
            hid_write(hid, data0, 9);
            RCLCPP_INFO(this->get_logger(), "Wrote speed: '%i'", speed);
        }

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_odom_service;
        rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr mload_publisher;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmdvel_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr maxspeed_;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr jointstate_publisher;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
        hid_device *hid;

        float last_x_vel;

        int16_t last_enc;

        rclcpp::Time last_time = this->now();

        const float w_rad  = 74.3/1000.0; //Wheel radius
        const float base_w = 518.0/1000.0; //Wheel distance
        const float rs_rpm = 60.0/(2.0*M_PI); //rad/s -> RPM

        float odom_pose[3];
        float debug = 0.0f;

        double change_w = 0.0f;

        double rotation_w = 0.0f;
};

int main(int argc, char **argv) {
    hid_device *handle;

    hid_init();
    handle = hid_open(0x3eb, 0x204f, NULL);
    if (!handle) {
        printf("Unable to open USB device, check connections\n");
        printf("%ls\n", hid_error(NULL));
        hid_exit();
        return 1;
    }
    hid_set_nonblocking(handle, 1);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TreadmillController>(handle));
    hid_close(handle);
    hid_exit();
    rclcpp::shutdown();
    return 0;
}
