#pragma once
/* includes //{ */

/* each ROS nodelet must have these */
#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/CameraInfo.h>
#include <rosgraph_msgs/Log.h>

/* some STL includes */
#include <cstdlib>
#include <cstdio>

/* custom helper functions from our library */
#include <mrs_lib/param_loader.h>
#include <mrs_lib/transformer.h>
#include <mrs_msgs/GimbalPRY.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include <pluginlib/class_list_macros.h>

#include <apriltag_ros/AprilTagDetectionArray.h>

template<typename T>
T deg2rad(const T x) { return x * M_PI / 180; }

template<typename T>
T rad2deg(const T x) { return x / M_PI * 180; }

namespace gimbal_camera {

/* class GimbalCameraNodelet //{ */
    class GimbalCameraNodelet : public nodelet::Nodelet {

    public:
        /* onInit() is called when nodelet is launched (similar to main() in regular node) */
        virtual void onInit();

    private:
        /* flags */
        bool m_recv_camera_info = false;
        /* ros parameters */

        float m_max_angle;
        float m_time_before_centering;
        float m_max_x_error;
        float m_max_y_error;
        std::string m_uav_name;

        ros::Timer m_timer_centering;
        sensor_msgs::CameraInfo m_camera_info;

        // | --------------------- MRS transformer -------------------- |
        mrs_lib::Transformer m_transformer;

        // | ---------------------- msg callbacks --------------------- |

        void m_cbk_camera_info(const sensor_msgs::CameraInfo::ConstPtr &msg);
        void m_cbk_tag_detection(const apriltag_ros::AprilTagDetectionArray msg);

        // | ----------------------- publishers ----------------------- |
        ros::Publisher m_pub_transform2gimbal_pry;
        ros::Publisher m_pub_transform2gimbal_quat;

        // | ----------------------- subscribers ---------------------- |
        ros::Subscriber m_sub_camera_info;
        ros::Subscriber m_sub_tag_detection;

        // | --------------------- other functions -------------------- |

        void center_camera(const ros::TimerEvent &ev);

    };
//}

}  // namespace gimbal_camera
