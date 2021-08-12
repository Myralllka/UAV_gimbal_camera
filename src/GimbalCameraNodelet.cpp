#include <GimbalCameraNodelet.h>

/* every nodelet must include macros which export the class as a nodelet plugin */

namespace gimbal_camera {

/* onInit() method //{ */
    void GimbalCameraNodelet::onInit() {

        /* obtain node handle */
        ros::NodeHandle nh = nodelet::Nodelet::getMTPrivateNodeHandle();

        /* waits for the ROS to publish clock */
        ros::Time::waitForValid();

        // | ------------------- load ros parameters ------------------ |
        mrs_lib::ParamLoader pl(nh);
        pl.loadParam2("uav_name", m_uav_name);
        pl.loadParam("max_angle", m_max_angle);
        pl.loadParam("time_before_centering", m_time_before_centering);
        pl.loadParam("x_deadband", m_max_x_error);
        pl.loadParam("y_deadband", m_max_y_error);

        if (not pl.loadedSuccessfully()) {
            ROS_ERROR("[GimbalCameraNodelet]: Some compulsory parameters could not be loaded! Ending.");
            ros::shutdown();
            return;
        }


        // | --------------------- tf transformer --------------------- |
        m_transformer = mrs_lib::Transformer("gimbal_camera");

        // | -------------------- initialize timers ------------------- |

        m_timer_centering = nh.createTimer(ros::Duration(m_time_before_centering),
                                           &GimbalCameraNodelet::center_camera, this);
        // | ------------ initialize publishers subscribers ------------|
        m_pub_transform2gimbal_pry = nh.advertise<mrs_msgs::GimbalPRY>("cmd_pry", 1);

        m_pub_transform2gimbal_quat = nh.advertise<geometry_msgs::QuaternionStamped>("cmd_quat", 1);

        m_sub_tag_detection = nh.subscribe("/tag_detections", 8,
                                           &GimbalCameraNodelet::m_cbk_tag_detection,
                                           this);

        m_sub_camera_info = nh.subscribe("/camera/camera_info", 8,
                                         &GimbalCameraNodelet::m_cbk_camera_info,
                                         this);
        ROS_INFO_ONCE("[GimbalCameraNodelet]: Nodelet initialized");
    }
//}

// | ---------------------- msg callbacks --------------------- |

    void GimbalCameraNodelet::m_cbk_camera_info(const sensor_msgs::CameraInfo::ConstPtr &msg) {
        if (not m_recv_camera_info) {
            m_camera_info = *msg;
            m_recv_camera_info = true;
        } else {
            m_sub_camera_info.shutdown();
        }
    }

    void GimbalCameraNodelet::m_cbk_tag_detection(const apriltag_ros::AprilTagDetectionArray msg) {
        if (msg.detections.size() == 0) return;
        auto pose = msg.detections.back().pose.pose.pose;

        const auto vec2apriltag = Eigen::Vector3d{pose.position.x,
                                                  pose.position.y,
                                                  pose.position.z};

        Eigen::Quaterniond qt = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitX(), vec2apriltag);

        const auto x_error = static_cast<float>(pose.position.x);
        const auto y_error = static_cast<float>(pose.position.y);

        if (std::abs(y_error) > m_max_y_error or std::abs(x_error) > m_max_x_error) {
            auto msg_quat = boost::make_shared<geometry_msgs::QuaternionStamped>();
            msg_quat->header.stamp = ros::Time::now();
            msg_quat->header.frame_id = "uav1/gimbal/camera_optical";
            msg_quat->quaternion.x = qt.x();
            msg_quat->quaternion.y = qt.y();
            msg_quat->quaternion.z = qt.z();
            msg_quat->quaternion.w = qt.w();

            m_pub_transform2gimbal_quat.publish(msg_quat);
            ROS_INFO_THROTTLE(1.0, "[GimbalCamera]: msg sent");
        } else {
            ROS_INFO_THROTTLE(1.0, "[GimbalCamera]: no changes - msg is not sent");
        }

    }
// | --------------------- timer callbacks -------------------- |

    void GimbalCameraNodelet::center_camera([[maybe_unused]] const ros::TimerEvent &ev) {
        if (not m_recv_camera_info) return;

        const auto tf_tag_cam = m_transformer.getTransform("mbundle", "camera", ros::Time::now());
        if (!tf_tag_cam.has_value() or (ros::Time::now().sec - tf_tag_cam->stamp().sec > m_time_before_centering)) {
            ROS_ERROR_THROTTLE(1.0,
                    "[GimbalCameraNodelet]: Could not transform commanded orientation from frame mbundle to camera, ignoring.");
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
