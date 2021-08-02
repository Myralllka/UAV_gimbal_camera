#include <GimbalCameraNodelet.h>

/* every nodelet must include macros which export the class as a nodelet plugin */
#include <pluginlib/class_list_macros.h>

namespace gimbal_camera {

/* onInit() method //{ */
    void GimbalCameraNodelet::onInit() {

        // | ---------------- set my booleans to false ---------------- |

        /* obtain node handle */
        ros::NodeHandle nh = nodelet::Nodelet::getMTPrivateNodeHandle();

        /* waits for the ROS to publish clock */
        ros::Time::waitForValid();

        // | ------------------- load ros parameters ------------------ |
        /* (mrs_lib implementation checks whether the parameter was loaded or not) */


        // | --------------------- tf transformer --------------------- |

        // | -------------------- initialize timers ------------------- |

        ROS_INFO_ONCE("[GimbalCameraNodelet]: initialized");

        is_initialized = true;
    }
//}

// | ---------------------- msg callbacks --------------------- |


// | --------------------- timer callbacks -------------------- |

// | -------------------- other functions ------------------- |

}  // namespace gimbal_camera  

/* every nodelet must export its class as nodelet plugin */
PLUGINLIB_EXPORT_CLASS(gimbal_camera::GimbalCameraNodelet, nodelet::Nodelet)