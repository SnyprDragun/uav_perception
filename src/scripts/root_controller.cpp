#include "quadcopter_perception/takeoff_node.hpp"
#include "quadcopter_perception/offboard_node.hpp"
#include "quadcopter_perception/landing_node.hpp"

mavros_msgs::State current_state;
mavros_msgs::SetMode set_mode;

int main(int argc, char **argv){
    init(argc, argv, "root_controller", init_options::AnonymousName);
    
    ROS_INFO("Initializing Root Controller...");

    float altitude = 5.0;

    Takeoff* takeoff = new Takeoff();
    takeoff->init_connection();
    takeoff->arm();
    takeoff->takeoff(altitude);
    Time last_request = Time::now();

    bool flag = true;
    while (flag){
        if (current_state.mode != "AUTO.LOITER" && (Time::now() - last_request > Duration(5.0))){
            ROS_INFO("Waiting to complete takeoff...");
        }
        else{
            ROS_INFO("Takeoff Completed Successfully!");
            Duration(10).sleep();

            Offboard* offboard = new Offboard();
            offboard->init_connection();
            offboard->offboard(2, 2, 2);
            offboard->panorama();

            flag = false;
        }
    }
    Duration(10).sleep();
    Land* land = new Land();
    land->init_connection();
    land->land();

    spin();
    return 0;
}
