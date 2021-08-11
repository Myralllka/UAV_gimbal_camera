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

//}

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
        sensor_msgs::CameraInfo m_camera_info;

        float m_yaw_movement{0.0};
        float m_pitch_movement{0.0};
        const float m_step{0.001};
        const float m_max_angle{0.28};
        const float m_time_before_centering{1};

        const float m_max_x_error{0.005};
        const float m_max_y_error{0.005};
        ros::Timer m_timer_centering;

        std::mutex m_movement_mutex;
        // | --------------------- MRS transformer -------------------- |
        mrs_lib::Transformer m_transformer;

        // | ---------------------- msg callbacks --------------------- |

        void m_cbk_camera_info(const sensor_msgs::CameraInfo::ConstPtr &msg);

        void m_cbk_tag_detection(const apriltag_ros::AprilTagDetectionArray msg);
        // | --------------------- timer callbacks -------------------- |

        // | --------- variables, related to message checking --------- |


        // | ----------------------- publishers ----------------------- |

        ros::Publisher m_pub_transform2gimbal_pry;
        ros::Publisher m_pub_transform2gimbal_quat;

        // | ----------------------- subscribers ---------------------- |

        ros::Subscriber m_sub_camera_info;
        ros::Subscriber m_sub_tag_detection;
//        ros::Subscriber m_sub_gimbal_;
        // | --------------------- other functions -------------------- |

        void follow_apriltag_incremental(const ros::TimerEvent &ev);

        void follow_apriltag_from_two_vectors();

        void center_camera(const ros::TimerEvent &ev);

        [[maybe_unused]] inline float calculate_step(const float current_error, const float max_error) const {
            float step = std::abs(current_error) / max_error * m_step;
            return step;
        }
    };
//}

}  // namespace gimbal_camera
