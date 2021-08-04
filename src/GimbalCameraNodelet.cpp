#include <GimbalCameraNodelet.h>

/* every nodelet must include macros which export the class as a nodelet plugin */

namespace gimbal_camera {

/* onInit() method //{ */
    void GimbalCameraNodelet::onInit() {

        // | ---------------- set my booleans to false ---------------- |

        /* obtain node handle */
        ros::NodeHandle nh = nodelet::Nodelet::getMTPrivateNodeHandle();

        /* waits for the ROS to publish clock */
        ros::Time::waitForValid();

        // | ------------------- load ros parameters ------------------ |

        // | --------------------- tf transformer --------------------- |
        m_transformer = mrs_lib::Transformer("gimbal_camera");
        // | -------------------- initialize timers ------------------- |

        m_pub_transform2gimbal = nh.advertise<mrs_msgs::GimbalPRY>("/uav1/gimbal_driver/cmd_pry", 2);

        m_sub_gimbal_camera_info = nh.subscribe("/camera/camera_info", 2,
                                                &GimbalCameraNodelet::callback_camera_info,
                                                this);

        ROS_INFO_ONCE("[GimbalCameraNodelet]: initialized");
        while (true) {
            follow_apriltag();
        }
    }
//}

// | ---------------------- msg callbacks --------------------- |

    void GimbalCameraNodelet::callback_camera_info(const sensor_msgs::CameraInfo::ConstPtr &msg) {
        if (not m_recv_camera_info) {
            m_camera_info = *msg;
            m_recv_camera_info = true;
        }
    }
// | --------------------- timer callbacks -------------------- |

// | -------------------- other functions ------------------- |

    void GimbalCameraNodelet::follow_apriltag() {
        if (not m_recv_camera_info) return;

        const auto tf_tag_cam = m_transformer.getTransform("mbundle", "camera");
        if (!tf_tag_cam.has_value()) {
            ROS_ERROR_THROTTLE(1.0,
                               "[GimbalCameraNodelet]: Could not transform commanded orientation from frame mbundle to camera, ignoring.");
            return;
        }
        ROS_INFO_THROTTLE(1.0, "[GimbalCamera]: start following april tag");
        const float horizontal_pixels_per_angle = m_camera_info.K[2] / m_camera_info.K[0];
        const float vertical_pixels_per_angle = m_camera_info.K[5] / m_camera_info.K[4];

        //std::cout << horizontal_pixels_per_angle << std::endl;
        //std::cout << vertical_pixels_per_angle << std::endl;

        auto msg_pry = boost::make_shared<mrs_msgs::GimbalPRY>();

        float x_error = static_cast<float>(tf_tag_cam->getTransform().transform.translation.x);
        float y_error = static_cast<float>(tf_tag_cam->getTransform().transform.translation.y);
        ROS_INFO_THROTTLE(1.0, "[GimbalCamera]: %f - x_error, %f - y_error", x_error, y_error);
        ROS_INFO_THROTTLE(1.0, "[GimbalCamera]: %f - ver    , %f - hor    ", vertical_pixels_per_angle, horizontal_pixels_per_angle);


        bool x_change = std::abs(x_error) > 0.1;
        bool y_change = std::abs(y_error) > 0.1;

        msg_pry->yaw = x_change
                       ? - deg2rad(x_error /  0.24) : 0;
        msg_pry->pitch = y_change
                         ? deg2rad(y_error / 0.24) : 0;
        msg_pry->roll = 0;
        if (x_change or y_change) {
            m_pub_transform2gimbal.publish(msg_pry);
            ROS_INFO_THROTTLE(1.0, "[GimbalCamera]: msg sended");
        } else {
            ROS_INFO_THROTTLE(1.0, "[GimbalCamera]: no changes - msg is not sended");
        }
    }


}  // namespace gimbal_camera  

/* every nodelet must export its class as nodelet plugin */
PLUGINLIB_EXPORT_CLASS(gimbal_camera::GimbalCameraNodelet, nodelet::Nodelet)
