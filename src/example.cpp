#include "ros/ros.h"
#include "servo_interpolator/ServoInterpolator.hpp"
#include "std_msgs/UInt16.h"
#include "std_msgs/Int8.h"

using AngleMsg_t = std_msgs::Int8;
using PwmMsg_t = std_msgs::UInt16;
int main(int argc, char *argv[])
{

    ros::init(argc,argv,"servo_controller");

    using ServoInterpolatorRos_t = ServoInterpolatorRos<PwmMsg_t,AngleMsg_t>;
    using ServoInterpolatorConf_t = ServoInterpolatorRos_t::ServoInterpolatorConf_t;
    ServoInterpolatorConf_t interpolator_conf = {
        .pwmExtremeRight=2000,
        .pwmExtremeLeft=1000,
        .pwmCenter=1500,
        .angleExtremeRight=45,
        .angleExtremeLeft=-45,
        .angleCenter=0,
    };

    ServoInterpolatorRosConf ros_conf = {
        .pwm_topic="/servo_pwm",
        .angle_topic="/servo_angle",
        .pwm_qs=100,
        .angle_qs=100,
        .pwm_param="/dummy"
    };

    ros::NodeHandle nh;

    ServoInterpolatorRos_t interpolator_node(nh,ros_conf,interpolator_conf);


    ros::spin();

    return 0; 
}
