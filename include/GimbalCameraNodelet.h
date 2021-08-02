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
        bool is_initialized = false;
        /* ros parameters */

        std::string _uav_name_;

        // | --------------------- MRS transformer -------------------- |

        mrs_lib::Transformer transformer_;

        // | ---------------------- msg callbacks --------------------- |

        // | --------------------- timer callbacks -------------------- |

        // | --------- variables, related to message checking --------- |


        // | ----------------------- publishers ----------------------- |

        ros::Publisher pub_example;

        // | --------------------- other functions -------------------- |

    };
//}

}  // namespace gimbal_camera
