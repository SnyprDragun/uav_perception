#include "quadcopter_perception/takeoff_node.hpp"
#include "quadcopter_perception/offboard_node.hpp"
#include "quadcopter_perception/landing_node.hpp"

#include <ros/ros.h>
#include <string.h>
#include <cmath>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <iostream>

#define KEY_UP 72
#define KEY_DOWN 80
#define KEY_LEFT 75
#define KEY_RIGHT 77

using namespace std;
using namespace ros;

mavros_msgs::State current_state_offboard;

void state_cb(const mavros_msgs::State& msg){
    current_state_offboard = msg;
}

int main(int argc, char **argv){
    
    init(argc, argv, "manual_controller", init_options::AnonymousName);
    NodeHandle nh;

    string state_sub_topic = "/mavros/state";
    Subscriber state_sub = nh.subscribe(state_sub_topic, 10, &state_cb);
    
    string position_pub_topic = "/mavros/setpoint_position/local";
    Publisher position_pub = nh.advertise<geometry_msgs::PoseStamped>(position_pub_topic, 10);

    string vel_pub_topic = "/mavros/setpoint_velocity/cmd_vel";
    Publisher velocity_pub = nh.advertise<geometry_msgs::TwistStamped>(vel_pub_topic, 10);
    
    string set_mode_client_topic = "/mavros/set_mode";
    ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>(set_mode_client_topic);

    mavros_msgs::State current_state;
    mavros_msgs::SetMode set_mode;
    geometry_msgs::PoseStamped linear_pose;
    geometry_msgs::TwistStamped angular_pose;

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    
    ROS_INFO("Initializing Manual Controller...");

    Takeoff* takeoff = new Takeoff();
    takeoff->init_connection();
    takeoff->arm();
    takeoff->takeoff(1);
    Time last_request = Time::now();

    int c = 0;
    while(1)
    {
        c = 0;

        switch((c=getchar())) {
        case KEY_UP:
            cout << endl << "Up" << endl;//key up
            while(ok()){
                if( current_state_offboard.mode != "OFFBOARD" && (Time::now() - last_request > Duration(5.0))){
                    if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){
                        ROS_INFO("Preparing to move...");
                    }
                    last_request = Time::now();
                }
                linear_pose.pose.position.z += 1;
                position_pub.publish(linear_pose);
                spinOnce();
            }
            break;
        case KEY_DOWN:
            cout << endl << "Down" << endl;   // key down
            linear_pose.pose.position.z -= 1;
            position_pub.publish(linear_pose);
            break;
        case KEY_LEFT:
            cout << endl << "Left" << endl;  // key left
            angular_pose.twist.angular.z += 0.2;
            velocity_pub.publish(angular_pose);
            break;
        case KEY_RIGHT:
            cout << endl << "Right" << endl;  // key right
            angular_pose.twist.angular.z -= 0.2;
            velocity_pub.publish(angular_pose);
            break;
        default:
            cout << endl << "null" << endl;  // not arrow
            break;
        }

    }

    spin();
    return 0;
}
