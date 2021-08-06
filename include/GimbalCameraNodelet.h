#pragma once
/* includes //{ */

/* each ROS nodelet must have these */
#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/CameraInfo.h>

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
        /* ros parameters */
        sensor_msgs::CameraInfo m_camera_info;
        float m_yaw_movement{0.0};
        float m_pitch_movement{0.0};
        const float m_step{0.001};
        const float m_max_angle{0.3};
        ros::Timer m_timer;
        // | --------------------- MRS transformer -------------------- |
        mrs_lib::Transformer m_transformer;
        // | ---------------------- msg callbacks --------------------- |

        void callback_camera_info(const sensor_msgs::CameraInfo::ConstPtr &msg);
        // | --------------------- timer callbacks -------------------- |

        // | --------- variables, related to message checking --------- |


        // | ----------------------- publishers ----------------------- |

        ros::Publisher m_pub_transform2gimbal;

        // | ----------------------- subscribers ---------------------- |

        ros::Subscriber m_sub_gimbal_camera_info;
//        ros::Subscriber m_sub_gimbal_;
        // | --------------------- other functions -------------------- |

        void follow_apriltag_using_z_coordinate();

        void follow_apriltag_steps(const ros::TimerEvent &ev);

    };
//}

}  // namespace gimbal_camera
