#ifndef LANDINGCONTROL_H
#define LANDINGCONTROL_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/PoseStamped.h>
#include <control_toolbox/pid.h>
#include <dji_sdk/SDKControlAuthority.h>
#include <dji_sdk/Gimbal.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Float32.h>

enum LANDING_STAGE
{
    CENTERING,      // centering under the marker using camera
    DESCENDING,     // descending after centered
    LANDING         // last landing stage
};

#endif // LANDINGCONTROL_H
