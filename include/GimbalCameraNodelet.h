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


#include <pluginlib/class_list_macros.h>

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
        bool m_centering_flag = false;
        /* ros parameters */
        sensor_msgs::CameraInfo m_camera_info;

        float m_yaw_movement{0.0};
        float m_pitch_movement{0.0};
        const float m_step{0.001};
        const float m_max_angle{0.28};
        const float m_time_before_centering{1};

        const float m_max_x_error{0.01};
        const float m_max_y_error{0.01};
        ros::Timer m_timer_following;
        ros::Timer m_timer_centering;

        size_t m_missed_images = 0;
        std::mutex m_movement_mutex;
        std::mutex m_centering_mutex;
        // | --------------------- MRS transformer -------------------- |
        mrs_lib::Transformer m_transformer;
        // | ---------------------- msg callbacks --------------------- |

        void callback_camera_info(const sensor_msgs::CameraInfo::ConstPtr &msg);

        void callback_missing_images(const rosgraph_msgs::Log::ConstPtr &msg);
        // | --------------------- timer callbacks -------------------- |

        // | --------- variables, related to message checking --------- |


        // | ----------------------- publishers ----------------------- |

        ros::Publisher m_pub_transform2gimbal_pry;
        ros::Publisher m_pub_transform2gimbal_quat;

        // | ----------------------- subscribers ---------------------- |

        ros::Subscriber m_sub_gimbal_camera_info;
        ros::Subscriber m_sub_rosout;
//        ros::Subscriber m_sub_gimbal_;
        // | --------------------- other functions -------------------- |

        void follow_apriltag_using_z_coordinate(const ros::TimerEvent &ev);

        void follow_apriltag_incremental(const ros::TimerEvent &ev);

        void follow_apriltag_from_two_vectors(const ros::TimerEvent &ev);

        void center_camera(const ros::TimerEvent &ev);

        [[maybe_unused]] inline float calculate_step(const float current_error, const float max_error) const {
            float step = std::abs(current_error) / max_error * m_step;
            return step;
        }
    };
//}

}  // namespace gimbal_camera
