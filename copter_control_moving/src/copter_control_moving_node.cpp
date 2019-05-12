#include "copter_control_moving_node.h"

bool requestSDKControl()
{
    ros::NodeHandle node;

    ros::ServiceClient sdk_client
            = node.serviceClient<dji_sdk::SDKControlAuthority>("/dji_sdk/sdk_control_authority");

    dji_sdk::SDKControlAuthority srv;
    srv.request.control_enable = dji_sdk::SDKControlAuthority::Request::REQUEST_CONTROL;
    if (sdk_client.call(srv)){
        ROS_INFO("SDK control request result: %d", srv.response.ack_data);
        return (srv.response.ack_data == 2);
    }
    else {
        ROS_ERROR("Failed to call SDKControlAuthority service");
        return false;
    }
}


int main(int argc, char** argv){
    ros::init(argc, argv, "copter_control_moving_node");
    ros::NodeHandle nh_ = ros::NodeHandle("copter_control");

    requestSDKControl();

    ros::spin();
    return 0;
};

