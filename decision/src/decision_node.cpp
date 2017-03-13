#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include <cmath>
#include <tf/transform_datatypes.h>

class decision {
private:

    ros::NodeHandle n;

    // communication with person_detector or person_tracker
    ros::Publisher pub_goal_reached;
    ros::Subscriber sub_goal_to_reach;

    // communication with rotation_action
    ros::Publisher pub_rotation_to_do;
    ros::Subscriber sub_rotation_done;

    // communication with translation_action
    ros::Publisher pub_translation_to_do;
    ros::Subscriber sub_translation_done;

    int rotation;//boolean to check if there is a rotation to do
    int translation;//boolean to check if there is a translation to do

    float rotation_to_do;
    float rotation_done;
    float translation_to_do;
    float translation_done;

public:

decision() {

    // communication with person_detector or person_tracker
    pub_goal_reached = n.advertise<geometry_msgs::Point>("goal_reached", 1);
    sub_goal_to_reach = n.subscribe("goal_to_reach", 1, &decision::goal_to_reachCallback, this);

    // communication with rotation_action
    pub_rotation_to_do = n.advertise<std_msgs::Float32>("rotation_to_do", 0);
    sub_rotation_done = n.subscribe("rotation_done", 1, &decision::rotation_doneCallback, this);
    rotation = 0;

    // communication with translation_action
    pub_translation_to_do = n.advertise<std_msgs::Float32>("translation_to_do", 0);
    sub_translation_done = n.subscribe("translation_done", 1, &decision::translation_doneCallback, this);
    translation = 0;

}

void goal_to_reachCallback(const geometry_msgs::Point::ConstPtr& g) {
// process the goal received from the person tracker
// we decompose the goal_received in one rotation and one translation
// and we perform the rotation first

    if ( !rotation && !translation ) {//we do not accept new goal if the current goal is not reached
        geometry_msgs::Point goal_to_reach;

        goal_to_reach.x = g->x;
        goal_to_reach.y = g->y;

        ROS_INFO("(decision_node) goal_to_reach (%f, %f)", goal_to_reach.x, goal_to_reach.y);
        getchar();

        //we compute the translation_to_do
        translation = 1;
        translation_to_do = sqrt( ( goal_to_reach.x * goal_to_reach.x ) + ( goal_to_reach.y * goal_to_reach.y ) );

        //we compute the rotation_to_do
        rotation = 1;
        rotation_to_do = acos( goal_to_reach.x / translation_to_do );

        if ( goal_to_reach.y < 0 )
            rotation_to_do *=-1;

        ROS_INFO("(decision_node) rotation_to_do: %f", rotation_to_do*180/M_PI);

        //we first perform the rotation_to_do and secondly the translation_to_do
        std_msgs::Float32 msg_rotation_to_do;

        //todo

    }

}

void rotation_doneCallback(const std_msgs::Float32::ConstPtr& a) {
// process the angle received from the rotation node

    rotation_done = a->data;
    ROS_INFO("(decision_node) rotation is done : %f", rotation_done*180/M_PI);
    rotation = 0;

    //the rotation_to_do is done so we perform the translation_to_do
    std_msgs::Float32 msg_translation_to_do;

    ROS_INFO("(decision_node) translation_to_do: %f", translation_to_do);
    //todo

}

void translation_doneCallback(const std_msgs::Float32::ConstPtr& r) {
// process the range received from the translation node

    translation_done = r->data;
    ROS_INFO("(decision_node) translation is done : %f\n", translation_done);
    translation = 0;

    //the translation is done so we send the goal_reached to the detector/tracker node
    geometry_msgs::Point msg_goal_reached;

    //todo
}

};

int main(int argc, char **argv){

    ros::init(argc, argv, "decision");

    decision bsObject;

    ros::spin();

    return 0;
}
