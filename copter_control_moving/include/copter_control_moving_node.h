/** @file demo_local_position_control.h
 *  @version 3.3
 *  @date September, 2017
 *
 *  @brief
 *  demo sample of how to use local position control APIs
 *
 *  @copyright 2017 DJI. All rights reserved.
 *
 */

#ifndef PROJECT_DEMO_LOCAL_POSITION_CONTROL_H
#define PROJECT_DEMO_LOCAL_POSITION_CONTROL_H

#endif //PROJECT_DEMO_LOCAL_POSITION_CONTROL_H

#include <dji_sdk/SetLocalPosRef.h>
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/UInt8.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/NavSatFix.h>
#include <control_toolbox/pid.h>

//DJI SDK includes
#include <dji_sdk/DroneTaskControl.h>
#include <dji_sdk/SDKControlAuthority.h>
#include <dji_sdk/QueryDroneVersion.h>

enum LANDING_STAGE
{
    ON_THE_GROUND,
    TAKING_OFF,
    APPROACHING,    // go to marker area by its gps coordinates, received with telemetry
    CENTERING,      // centering under the marker using camera
    DESCENDING,     // descending after centered
    LAND_PLACE_FOUND,
    LANDING,         // last landing stage
    FINISHED
};

bool set_local_position();

float target_offset_x;
float target_offset_y;
float target_offset_z;
float target_yaw;
int target_set_state = 0;

void setTarget(float x, float y, float z, float yaw)
{
  target_offset_x = x;
  target_offset_y = y;
  target_offset_z = z;
  target_yaw      = yaw;
}

void local_position_callback(const geometry_msgs::PointStamped::ConstPtr& msg);

void display_mode_callback(const std_msgs::UInt8::ConstPtr& msg);

void flight_status_callback(const std_msgs::UInt8::ConstPtr& msg);

void gps_position_callback(const sensor_msgs::NavSatFix::ConstPtr& msg);

void gps_health_callback(const std_msgs::UInt8::ConstPtr& msg);

bool takeoff_land(int task);

bool obtain_control();

bool is_M100();

bool monitoredTakeoff();

bool M100monitoredTakeoff();

void local_position_ctrl(double &xCmd, double &yCmd, double &zCmd);


typedef struct ServiceAck
{
  bool         result;
  int          cmd_set;
  int          cmd_id;
  unsigned int ack_data;
  ServiceAck(bool res, int set, int id, unsigned int ack)
    : result(res)
    , cmd_set(set)
    , cmd_id(id)
    , ack_data(ack)
  {
  }
  ServiceAck()
  {
  }
} ServiceAck;

ServiceAck land();

void wait_until_control();

bool rc_sdk_enable_ = false;
bool prev_rc_sdk_enable_= false;
bool sdk_control_enabled_ = false;

LANDING_STAGE landing_stage_ = ON_THE_GROUND;

ros::Timer control_timer_;
ros::Time last_time_;

void timerCallback(const ros::TimerEvent& event);

void controlByPosErrYawBody(double dx, double dy, double vert_vel, double yaw);

control_toolbox::Pid pid_roll_;
control_toolbox::Pid pid_pitch_;
