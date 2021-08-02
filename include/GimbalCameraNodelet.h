#pragma once
/* includes //{ */

/* each ROS nodelet must have these */
#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>

/* some STL includes */
#include <cstdlib>
#include <cstdio>

/* custom helper functions from our library */
#include <mrs_lib/param_loader.h>
#include <mrs_lib/transformer.h>

//}

namespace gimbal_camera {

/* class GimbalCameraNodelet //{ */
    class GimbalCameraNodelet : public nodelet::Nodelet {

    public:
        /* onInit() is called when nodelet is launched (similar to main() in regular node) */
        virtual void onInit();

    private:
        /* flags */
        /* ros parameters */
        // | --------------------- MRS transformer -------------------- |

        // | ---------------------- msg callbacks --------------------- |

        void callback_gimbal_image(const sensor_msgs::ImageConstPtr &msg);
        // | --------------------- timer callbacks -------------------- |

        // | --------- variables, related to message checking --------- |


        // | ----------------------- publishers ----------------------- |

        ros::Publisher m_pub_im2apriltag;

        // | ----------------------- subscribers ---------------------- |

        ros::Subscriber m_sub_gimbal_camera_image;
        // | --------------------- other functions -------------------- |

    };
//}

}  // namespace gimbal_camera
