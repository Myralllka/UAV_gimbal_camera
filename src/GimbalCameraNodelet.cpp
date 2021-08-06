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
        m_timer = nh.createTimer(ros::Duration(0.02), &GimbalCameraNodelet::follow_apriltag_steps, this);
        // | --------------------- tf transformer --------------------- |
        m_transformer = mrs_lib::Transformer("gimbal_camera");
        // | -------------------- initialize timers ------------------- |

        m_pub_transform2gimbal = nh.advertise<mrs_msgs::GimbalPRY>("/uav1/gimbal_driver/cmd_pry", 2);

        m_sub_gimbal_camera_info = nh.subscribe("/camera/camera_info", 2,
                                                &GimbalCameraNodelet::callback_camera_info,
                                                this);

        ROS_INFO_ONCE("[GimbalCameraNodelet]: initialized");
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

    void GimbalCameraNodelet::follow_apriltag_steps([[maybe_unused]] const ros::TimerEvent &ev) {
        if (not m_recv_camera_info) return;
        const auto tf_tag_cam = m_transformer.getTransform("mbundle", "camera", ros::Time::now());

        if (!tf_tag_cam.has_value() or (ros::Time::now().sec - tf_tag_cam->stamp().sec > 1)) {
            ROS_ERROR("[GimbalCameraNodelet]: Could not transform commanded orientation from frame mbundle to camera, ignoring.");
            m_yaw_movement = 0.0;
            m_pitch_movement = 0.0;
            auto m = boost::make_shared<mrs_msgs::GimbalPRY>();
            m->yaw = 0;
            m->pitch = 0;
            m->yaw = 0;
            m_pub_transform2gimbal.publish(m);
            return;
        }

//        ROS_INFO_THROTTLE(1.0, "[GimbalCamera]: start following april tag");

        const auto translation = tf_tag_cam->getTransform().transform.translation;

        const auto x_error = static_cast<float>(translation.x);
        const auto y_error = static_cast<float>(translation.y);

        ROS_INFO("[GimbalCamera]: %f - x_error, %f - y_error", x_error, y_error);

        const bool x_change_flag = std::abs(x_error) > 0.01;
        const bool y_change_flag = std::abs(y_error) > 0.01;


        ROS_INFO("[GimbalCamera]: %f - x_movement, %f - y_movement", rad2deg(m_yaw_movement),
                          rad2deg(m_pitch_movement));
        if (x_change_flag or y_change_flag) {
            if (std::abs(m_yaw_movement) < m_max_angle)
                x_error < 0 ? (m_yaw_movement += m_step) : (m_yaw_movement -= m_step);
            if (std::abs(m_pitch_movement) < m_max_angle)
                y_error < 0 ? (m_pitch_movement -= m_step) : (m_pitch_movement += m_step);

            auto msg_pry = boost::make_shared<mrs_msgs::GimbalPRY>();
            msg_pry->roll = 0;
            msg_pry->yaw = m_yaw_movement;
            msg_pry->pitch = m_pitch_movement;

            m_pub_transform2gimbal.publish(msg_pry);

            std::cout << "[GimbalCamera]: rotate on deg: " << rad2deg(msg_pry->yaw) << std::endl;

            //ROS_INFO_THROTTLE(1.0, "[GimbalCamera]: msg sent");
        } else {
            ROS_INFO_THROTTLE(1.0, "[GimbalCamera]: no changes - msg is not sent");
        }
    }

    [[maybe_unused]] void GimbalCameraNodelet::follow_apriltag_using_z_coordinate() {
        if (not m_recv_camera_info) return;
        const auto tf_tag_cam = m_transformer.getTransform("camera", "mbundle", ros::Time::now());
        //ROS_ERROR_STREAM_THROTTLE(1.0, "[GimbalCameraNodelet]: stamp sec: " << tf_tag_cam->stamp().sec << "; now: "<< ros::Time::now().sec<< "; duration: " << ros::Time::now().sec - tf_tag_cam->stamp().sec);

        if (!tf_tag_cam.has_value() or (ros::Time::now().sec - tf_tag_cam->stamp().sec > 1)) {
            ROS_ERROR_THROTTLE(1.0,
                               "[GimbalCameraNodelet]: Could not transform commanded orientation from frame mbundle to camera, ignoring.");
            return;
        }

        ROS_INFO_THROTTLE(1.0, "[GimbalCamera]: start following april tag");

        const auto translation = tf_tag_cam->getTransform().transform.translation;

        const auto x_error = static_cast<float>(translation.x);
        const auto y_error = static_cast<float>(translation.y);

        ROS_INFO_THROTTLE(1.0, "[GimbalCamera]: %f - x_error, %f - y_error", x_error, y_error);

        const bool x_change_flag = std::abs(x_error) > 0.01;
        const bool y_change_flag = std::abs(y_error) > 2;

        const auto yaw_angle_rotation = static_cast<float>(atan2(translation.x, translation.z));
        //const auto pitch_angle_rotation = static_cast<float>(atan2(translation.y, translation.z));

        if (x_change_flag or y_change_flag) {
            auto msg_pry = boost::make_shared<mrs_msgs::GimbalPRY>();
            msg_pry->roll = 0;
            msg_pry->yaw = yaw_angle_rotation;
            msg_pry->pitch = 0;
//            msg_pry->pitch = std::copysign(y_error, pitch_angle_rotation);
            m_pub_transform2gimbal.publish(msg_pry);

            std::cout << "[GimbalCamera]: rotate on deg: " << rad2deg(msg_pry->yaw) << std::endl;

            ROS_INFO_THROTTLE(1.0, "[GimbalCamera]: msg sent");
            //ros::Duration(0.1).sleep();
        } else {
            ROS_INFO_THROTTLE(1.0, "[GimbalCamera]: no changes - msg is not sent");
        }
    }


}  // namespace gimbal_camera  

/* every nodelet must export its class as nodelet plugin */
PLUGINLIB_EXPORT_CLASS(gimbal_camera::GimbalCameraNodelet, nodelet::Nodelet)
