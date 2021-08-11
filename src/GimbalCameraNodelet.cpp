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

        m_timer_following = nh.createTimer(ros::Duration(1), &GimbalCameraNodelet::follow_apriltag_from_two_vectors,
                                           this);

        m_timer_centering = nh.createTimer(ros::Duration(m_time_before_centering), &GimbalCameraNodelet::center_camera,
                                           this);

        m_pub_transform2gimbal_pry = nh.advertise<mrs_msgs::GimbalPRY>("/uav1/gimbal_driver/cmd_pry", 8);

        m_pub_transform2gimbal_quat = nh.advertise<mrs_msgs::GimbalPRY>("/uav1/gimbal_driver/cmd_orientation", 8);

        m_sub_gimbal_camera_info = nh.subscribe("/camera/camera_info", 8,
                                                &GimbalCameraNodelet::callback_camera_info,
                                                this);
        m_sub_rosout = nh.subscribe("/rosout", 2, &GimbalCameraNodelet::callback_missing_images, this);
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

    void GimbalCameraNodelet::callback_missing_images(const rosgraph_msgs::Log::ConstPtr &msg) {
        if (msg->name != "/camera/camera_nodelet_manager") return;
        if (msg->msg.find("Image received from camera") != std::string::npos) {
            m_missed_images += 1;
        }
    }
// | --------------------- timer callbacks -------------------- |


    [[maybe_unused]] void
    GimbalCameraNodelet::follow_apriltag_from_two_vectors([[maybe_unused]] const ros::TimerEvent &ev) {
        {
            std::lock_guard<std::mutex> l{m_centering_mutex};
            m_centering_flag = true;
        }
        if (not m_recv_camera_info) return;
        const auto tf_tag_cam = m_transformer.getTransform("mbundle", "camera", ros::Time::now());

        if (!tf_tag_cam.has_value() or (ros::Time::now().sec - tf_tag_cam->stamp().sec > 1)) return;

        const auto transformation = tf_tag_cam->getTransform().transform;

        const auto vec2apriltag = Eigen::Vector3d{transformation.translation.x,
                                                  transformation.translation.y,
                                                  transformation.translation.z};

        Eigen::Quaterniond orientation = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitX(), vec2apriltag);

        const auto x_error = static_cast<float>(transformation.translation.x);
        const auto y_error = static_cast<float>(transformation.translation.y);

        const bool x_change_flag = std::abs(x_error) > m_max_x_error;
        const bool y_change_flag = std::abs(y_error) > m_max_y_error;

        if (x_change_flag or y_change_flag) {

            auto msg_quat = boost::make_shared<geometry_msgs::QuaternionStamped>();

            msg_quat->header.stamp = ros::Time::now();
            msg_quat->header.frame_id = "uav1/gimbal/base_link";
            msg_quat->quaternion.x = orientation.x();
            msg_quat->quaternion.y = orientation.y();
            msg_quat->quaternion.z = orientation.z();
            msg_quat->quaternion.w = orientation.w();

            m_pub_transform2gimbal_quat.publish(msg_quat);

            ROS_INFO_THROTTLE(1.0, "[GimbalCamera]: msg sent");
        } else {
            ROS_INFO_THROTTLE(1.0, "[GimbalCamera]: no changes - msg is not sent");
        }

    }

    [[maybe_unused]] void GimbalCameraNodelet::follow_apriltag_incremental([[maybe_unused]] const ros::TimerEvent &ev) {
        {
            std::lock_guard<std::mutex> l{m_centering_mutex};
            m_centering_flag = false;
        }
        if (not m_recv_camera_info) return;
        const auto tf_tag_cam = m_transformer.getTransform("mbundle", "camera", ros::Time::now());

        if (!tf_tag_cam.has_value() or (ros::Time::now().sec - tf_tag_cam->stamp().sec > 0.5)) {
            return;
        }

        ROS_INFO_THROTTLE(1.0, "[GimbalCamera]: Incremental apriltag following");

        const auto translation = tf_tag_cam->getTransform().transform.translation;

        const auto x_error = static_cast<float>(translation.x);
        const auto y_error = static_cast<float>(translation.y);

        const bool x_change_flag = std::abs(x_error) > m_max_x_error;
        const bool y_change_flag = std::abs(y_error) > m_max_y_error;

        const std::lock_guard<std::mutex> lock(m_movement_mutex);
        if (x_change_flag) {
            const auto step = calculate_step(x_error, m_max_x_error);
            if (x_error < 0) {
                if (m_yaw_movement < m_max_angle) m_yaw_movement += step;
            } else {
                if (m_yaw_movement > -m_max_angle) m_yaw_movement -= step;
            }
        }
        if (y_change_flag) {
            const auto step = calculate_step(y_error, m_max_y_error);
            if (y_error < 0) {
                if (m_pitch_movement > -m_max_angle) m_pitch_movement -= step;
            } else {
                if (m_pitch_movement < m_max_angle) m_pitch_movement += step;
            }
        }

        if (x_change_flag or y_change_flag) {
            auto msg_pry = boost::make_shared<mrs_msgs::GimbalPRY>();
            msg_pry->roll = 0;
            msg_pry->yaw = m_yaw_movement;
            msg_pry->pitch = m_pitch_movement;
            m_pub_transform2gimbal_pry.publish(msg_pry);

            ROS_INFO_THROTTLE(1.0, "[GimbalCamera]: msg sent");
            //ROS_INFO("[GimbalCamera]: %f - x_movement deg, %f - y_movement deg", rad2deg(m_yaw_movement),
            //         rad2deg(m_pitch_movement));
            //ROS_INFO("[GimbalCamera]: %f - x_err, %f - y_err", x_error, y_error);
        } else {
            ROS_INFO_THROTTLE(1.0, "[GimbalCamera]: no changes - msg is not sent");
        }
    }

    [[maybe_unused]] void
    GimbalCameraNodelet::follow_apriltag_using_z_coordinate([[maybe_unused]] const ros::TimerEvent &ev) {
        if (not m_recv_camera_info) return;
        const auto tf_tag_cam = m_transformer.getTransform("camera", "mbundle", ros::Time::now());

        if (!tf_tag_cam.has_value() or (ros::Time::now().sec - tf_tag_cam->stamp().sec > 1)) {
            return;
        }

        ROS_INFO_THROTTLE(1.0, "[GimbalCamera]: start following april tag");

        const auto translation = tf_tag_cam->getTransform().transform.translation;

        const auto x_error = static_cast<float>(translation.x);
        const auto y_error = static_cast<float>(translation.y);

        ROS_INFO_THROTTLE(1.0, "[GimbalCamera]: %f - x_error, %f - y_error", x_error, y_error);

        const bool x_change_flag = std::abs(x_error) > m_max_x_error;
        const bool y_change_flag = std::abs(y_error) > m_max_y_error;

        const auto yaw_angle_rotation = static_cast<float>(atan2(translation.x, translation.z));
        //const auto pitch_angle_rotation = static_cast<float>(atan2(translation.y, translation.z));

        if (x_change_flag or y_change_flag) {
            auto msg_pry = boost::make_shared<mrs_msgs::GimbalPRY>();
            msg_pry->roll = 0;
            msg_pry->yaw = yaw_angle_rotation;
            msg_pry->pitch = 0;
//            msg_pry->pitch = std::copysign(y_error, pitch_angle_rotation);
            m_pub_transform2gimbal_pry.publish(msg_pry);

            ROS_INFO_THROTTLE(1.0, "[GimbalCamera]: msg sent");
            //ros::Duration(0.1).sleep();
        } else {
            ROS_INFO_THROTTLE(1.0, "[GimbalCamera]: no changes - msg is not sent");
        }
    }

    void GimbalCameraNodelet::center_camera([[maybe_unused]] const ros::TimerEvent &ev) {

        if (m_centering_flag)
            return;
        else {
            std::lock_guard<std::mutex> l{m_centering_mutex};
            m_centering_flag = true;
        }
        if (not m_recv_camera_info) return;

        const auto tf_tag_cam = m_transformer.getTransform("mbundle", "camera", ros::Time::now());
        const std::lock_guard<std::mutex> lock(m_movement_mutex);
        if (!tf_tag_cam.has_value() or (ros::Time::now().sec - tf_tag_cam->stamp().sec > m_time_before_centering)) {
            ROS_ERROR(
                    "[GimbalCameraNodelet]: Could not transform commanded orientation from frame mbundle to camera, ignoring.");
            m_yaw_movement = 0.0;
            m_pitch_movement = 0.0;
            auto m = boost::make_shared<mrs_msgs::GimbalPRY>();
            m->yaw = 0;
            m->pitch = 0;
            m->yaw = 0;
            m_pub_transform2gimbal_pry.publish(m);
        }
    }

    // | -------------------- other functions ------------------- |

}  // namespace gimbal_camera

/* every nodelet must export its class as nodelet plugin */
PLUGINLIB_EXPORT_CLASS(gimbal_camera::GimbalCameraNodelet, nodelet::Nodelet)
