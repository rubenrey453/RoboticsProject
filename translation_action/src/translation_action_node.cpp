#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/ColorRGBA.h"
#include "std_msgs/Float32.h"
#include <cmath>
#include "nav_msgs/Odometry.h"
#include <tf/transform_datatypes.h>

using namespace std;

class translation_action {
private:

    ros::NodeHandle n;

    // communication with cmd_vel to send command to the mobile robot
    ros::Publisher pub_cmd_vel;

    // communication with odometry
    ros::Publisher pub_change_odometry;
    ros::Subscriber sub_odometry;

    // communication with decision
    ros::Publisher pub_translation_done;
    ros::Subscriber sub_translation_to_do;

    float translation_to_do, translation_done;
    int cond_translation;

public:

translation_action() {

    // communication with cmd_vel
    pub_cmd_vel = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    // communication with odometry
    pub_change_odometry = n.advertise<geometry_msgs::Point>("change_odometry", 1);
    sub_odometry = n.subscribe("odom", 1, &translation_action::odomCallback, this);
    cond_translation = 0;

    // communication with decision
    pub_translation_done = n.advertise<std_msgs::Float32>("translation_done", 1);
    sub_translation_to_do = n.subscribe("translation_to_do", 1, &translation_action::translation_to_doCallback, this);//this is the translation that has to be performed

}

void odomCallback(const nav_msgs::Odometry::ConstPtr& o) {

    translation_done = o->pose.pose.position.x;
    //ROS_INFO("(translation_node) translation_done %f", translation_done);

    if ( cond_translation ) {

        geometry_msgs::Twist twist;
        twist.linear.x = 0;
        twist.linear.y = 0;
        twist.linear.z = 0;

        twist.angular.x = 0;
        twist.angular.y = 0;
        twist.angular.z = 0;

        ROS_INFO("(translation_node) translation_done: %f, translation_to_do: %f", translation_done, translation_to_do);

        /*if ( rotation_to_do has not been reached  ) {
                 *
                 * float kp, ep, ki, ei, kd, ed;
                 *
                 * implementation of a PID controller
                 *
                    ep = (translation_to_do - translation_done);//the current error is the difference between the translation_to_do (ie, the postion to reach) and the translation_done (ie, the current position)

                    twist.linear.x *= kp * ep + ki * ei + kd * ed;;
            pub_cmd_vel.publish(twist);
        }
        else {
            cond_translation = 0;
            ROS_INFO("(translation_node) translation_to_do: %f", translation_to_do);
            ROS_INFO("(translation_node) translation_done: %f", translation_done);

            std_msgs::Float32 msg_translation_done;
            msg_translation_done.data = translation_done;
            pub_translation_done.publish(msg_translation_done);//we sent the translation_done to decision_node;
        }*/
            //getchar();
    }
    cond_translation = 0;
            ROS_INFO("(translation_node) translation_to_do: %f", translation_to_do);
            ROS_INFO("(translation_node) translation_done: %f", translation_done);

            std_msgs::Float32 msg_translation_done;
            msg_translation_done.data = translation_done;
            pub_translation_done.publish(msg_translation_done);//we sent the translation_done to decision_node;

}

void translation_to_doCallback(const std_msgs::Float32::ConstPtr & r) {
// process the translation to do received from the decision node

    ROS_INFO("\n(translation_node) processing the translation_to_do received from the decision node");
    //getchar();
    translation_to_do = r->data;

    ROS_INFO("(translation_node) translation_to_do : %f", translation_to_do);
    //getchar();

    cond_translation = 1;
    translation_done = 0;

    //reset odometry
    geometry_msgs::Point p;
    p.x = 0;
    p.y = 0;
    p.z = 0;
    pub_change_odometry.publish(p);
    ros::Duration(0.5).sleep();

}

};


int main(int argc, char **argv){

    ros::init(argc, argv, "translation_action");

    translation_action bsObject;

    ros::spin();

    return 0;
}


