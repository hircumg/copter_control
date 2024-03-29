/** @file demo_flight_control.cpp
 *  @version 3.3
 *  @date September, 2017
 *
 *  @brief
 *  demo sample of how to use Local position control
 *
 *  @copyright 2017 DJI. All rights reserved.
 *
 */

#include "dji_sdk.h"
#include "copter_control_moving_node.h"

ros::ServiceClient set_local_pos_reference;
ros::ServiceClient sdk_ctrl_authority_service;
ros::ServiceClient drone_task_service;
ros::ServiceClient query_version_service;

ros::Publisher ctrlPosYawPub;
ros::Publisher ctrl_pub_;

// global variables for subscribed topics
uint8_t flight_status = 255;
uint8_t display_mode  = 255;
uint8_t current_gps_health = 0;
int num_targets = 0;
geometry_msgs::PointStamped local_position;
sensor_msgs::NavSatFix current_gps_position;


int main(int argc, char** argv)
{
  ros::init(argc, argv, "demo_local_position_control_node");
  ros::NodeHandle nh;
  // Subscribe to messages from dji_sdk_node
  ros::Subscriber flightStatusSub = nh.subscribe("dji_sdk/flight_status", 10, &flight_status_callback);
  ros::Subscriber displayModeSub = nh.subscribe("dji_sdk/display_mode", 10, &display_mode_callback);
  ros::Subscriber localPosition = nh.subscribe("dji_sdk/local_position", 10, &local_position_callback);
  ros::Subscriber gpsSub      = nh.subscribe("dji_sdk/gps_position", 10, &gps_position_callback);
  ros::Subscriber gpsHealth      = nh.subscribe("dji_sdk/gps_health", 10, &gps_health_callback);

  // Publish the control signal
  //ctrlPosYawPub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_ENUposition_yaw", 10);
  ctrl_pub_ = nh.advertise<sensor_msgs::Joy>("/dji_sdk/flight_control_setpoint_generic", 10);
  // Basic services
  sdk_ctrl_authority_service = nh.serviceClient<dji_sdk::SDKControlAuthority> ("dji_sdk/sdk_control_authority");
  drone_task_service         = nh.serviceClient<dji_sdk::DroneTaskControl>("dji_sdk/drone_task_control");
  query_version_service      = nh.serviceClient<dji_sdk::QueryDroneVersion>("dji_sdk/query_drone_version");
  set_local_pos_reference    = nh.serviceClient<dji_sdk::SetLocalPosRef> ("dji_sdk/set_local_pos_ref");

  pid_roll_.initPid(0.75, 0.051, 0.051, 5, -5);
  pid_pitch_.initPid(0.75, 0.051, 0.051, 5, -5);

  if (!set_local_position())
  {
    ROS_ERROR("GPS health insufficient - No local frame reference for height. Exiting.");
    return 1;
  }
    
  control_timer_ = nh.createTimer(ros::Duration(1.0/30.0), &timerCallback);

  ros::spin();
  return 0;
}

/*!
 * This function calculates the difference between target and current local position
 * and sends the commands to the Position and Yaw control topic.
 *
 */
void local_position_ctrl(double &xCmd, double &yCmd, double &zCmd)
{
  xCmd = target_offset_x - local_position.point.x;
  yCmd = target_offset_y - local_position.point.y;
  zCmd = target_offset_z;

  sensor_msgs::Joy controlPosYaw;
  controlPosYaw.axes.push_back(xCmd);
  controlPosYaw.axes.push_back(yCmd);
  controlPosYaw.axes.push_back(zCmd);
  controlPosYaw.axes.push_back(target_yaw);
  ctrlPosYawPub.publish(controlPosYaw);

  // 0.1m or 10cms is the minimum error to reach target in x y and z axes.
  // This error threshold will have to change depending on aircraft/payload/wind conditions.
  if (((std::abs(xCmd)) < 0.1) && ((std::abs(yCmd)) < 0.1) &&
      (local_position.point.z > (target_offset_z - 0.1)) && (local_position.point.z < (target_offset_z + 0.1)))
  {
      landing_stage_ = LAND_PLACE_FOUND;
  }
}

bool takeoff_land(int task)
{
  dji_sdk::DroneTaskControl droneTaskControl;

  droneTaskControl.request.task = task;

  drone_task_service.call(droneTaskControl);

  if(!droneTaskControl.response.result)
  {
    ROS_ERROR("takeoff_land fail");
    return false;
  }

  return true;
}


void wait_until_control()
{
//    if(!rc_sdk_enable_)
//    {
//        prev_rc_sdk_enable_ = false;
//        ROS_INFO_THROTTLE(2, "Idle, SDK control disabled");
//        //pid_roll_.reset();
//        //pid_pitch_.reset();
//        sdk_control_enabled_ = false;
//        //landing_stage_ = APPROACHING;
//    }
//    else if(!prev_rc_sdk_enable_)
//    {
    int tries = 0;
    while(tries < 5) {
        tries++;
        if(obtain_control()) {
            sdk_control_enabled_ = true;
            break;
        }
        else
            sdk_control_enabled_ = false;
        ros::Duration(0.1).sleep();
    }
}

bool obtain_control()
{
  dji_sdk::SDKControlAuthority authority;
  authority.request.control_enable=1;
  sdk_ctrl_authority_service.call(authority);

  if(!authority.response.result)
  {
    ROS_ERROR("obtain control failed!");
    return false;
  }

  return true;
}

bool is_M100()
{
  dji_sdk::QueryDroneVersion query;
  query_version_service.call(query);

  if(query.response.version == DJISDK::DroneFirmwareVersion::M100_31)
  {
    return true;
  }

  return false;
}


void gps_position_callback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
  current_gps_position = *msg;
}

void gps_health_callback(const std_msgs::UInt8::ConstPtr& msg) {
  current_gps_health = msg->data;
}

void flight_status_callback(const std_msgs::UInt8::ConstPtr& msg)
{
  flight_status = msg->data;
}

void display_mode_callback(const std_msgs::UInt8::ConstPtr& msg)
{
  display_mode = msg->data;
}

/*!
 * This function demos how to use M100 flight_status
 * to monitor the take off process with some error
 * handling. Note M100 flight status is different
 * from A3/N3 flight status.
 */
bool
M100monitoredTakeoff()
{
  ros::Time start_time = ros::Time::now();

  float home_altitude = current_gps_position.altitude;
  if(!takeoff_land(dji_sdk::DroneTaskControl::Request::TASK_TAKEOFF))
  {
    return false;
  }

  ros::Duration(0.01).sleep();
  ros::spinOnce();

  // Step 1: If M100 is not in the air after 10 seconds, fail.
  while (ros::Time::now() - start_time < ros::Duration(10))
  {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  if(flight_status != DJISDK::M100FlightStatus::M100_STATUS_IN_AIR ||
     current_gps_position.altitude - home_altitude < 1.0)
  {
    ROS_ERROR("Takeoff failed.");
    return false;
  }
  else
  {
    start_time = ros::Time::now();
    ROS_INFO("Successful takeoff!");
    ros::spinOnce();
  }
  return true;
}

bool set_local_position()
{
  dji_sdk::SetLocalPosRef localPosReferenceSetter;
  set_local_pos_reference.call(localPosReferenceSetter);

  return (bool)localPosReferenceSetter.response.result;
}

ServiceAck
land()
{
  dji_sdk::DroneTaskControl droneTaskControl;
  droneTaskControl.request.task = 6;
  drone_task_service.call(droneTaskControl);
  if (!droneTaskControl.response.result)
  {
    ROS_WARN("ack.info: set = %i id = %i", droneTaskControl.response.cmd_set,
             droneTaskControl.response.cmd_id);
    ROS_WARN("ack.data: %i", droneTaskControl.response.ack_data);
  }
  return ServiceAck(
    droneTaskControl.response.result, droneTaskControl.response.cmd_set,
    droneTaskControl.response.cmd_id, droneTaskControl.response.ack_data);
}

void timerCallback(const ros::TimerEvent& event) {
    wait_until_control();
    
    double xCmd, yCmd, zCmd;
    
    switch (landing_stage_) {
        case ON_THE_GROUND:
            landing_stage_ = TAKING_OFF;
            ROS_INFO("TAKE OFF");
            
            if(M100monitoredTakeoff())
            {
                landing_stage_ = APPROACHING;
                last_time_ = ros::Time::now();
            }
            break;
        case APPROACHING:
        {
            ROS_INFO_THROTTLE(1, "APPROACHING");
            //setTarget(10, 5, 1, 0);
            //local_position_ctrl(xCmd, yCmd, zCmd);
            int dx = 10 - local_position.point.x;
            int dy = 5 - local_position.point.y;
            controlByPosErrYawBody(dx, dy, 0, 0);
            if (std::abs(dx) < 0.1 && std::abs(dy) < 0.1) {
                landing_stage_ = LAND_PLACE_FOUND;
            }
            break;
        }
        case LAND_PLACE_FOUND:
            landing_stage_ = LANDING;
            ROS_INFO("LANDING");
            ros::Duration(1.0).sleep();
            ROS_INFO("position x: %.1f, y: %.1f", local_position.point.x,
                     local_position.point.y);
            if (land().result)
            {
                ROS_INFO("Land command sent successfully");
                landing_stage_ = FINISHED;
            }
            else
            {
                ROS_WARN("Failed sending land command");
                landing_stage_ = APPROACHING;
            }
            break;
        default:
            break;
    }
}

void local_position_callback(const geometry_msgs::PointStamped::ConstPtr& msg) {
    local_position = *msg;
}

void controlByPosErrYawBody(double dx, double dy, double vert_vel, double yaw)
{
    ros::Duration dt = ros::Time::now() - last_time_;
    if(dt.toSec() > 1.0/30.0)
        dt = ros::Duration(1.0/30.0);
    last_time_ = ros::Time::now();
    
    double ctrl_roll = pid_roll_.computeCommand(dy, dt);
    double ctrl_pitch = pid_pitch_.computeCommand(dx, dt);
    
    sensor_msgs::Joy ctrl_msg;
    ctrl_msg.axes.push_back(ctrl_pitch);    // Roll Channel
    ctrl_msg.axes.push_back(ctrl_roll);     // Pitch Channel
    ctrl_msg.axes.push_back(vert_vel);      // Throttle Channel
    ctrl_msg.axes.push_back(yaw);    // Yaw Channel
    
    unsigned int flag =
    0x40 | /*Command horizontal velocities*/
    0x00 | /*Command yaw angle*/
    0x02 | /*Horizontal command is body_FLU frame*/
    0x01; /*Actively break to hold position after stop sending setpoint*/
    ctrl_msg.axes.push_back(flag);
    
    ctrl_pub_.publish(ctrl_msg);
}




