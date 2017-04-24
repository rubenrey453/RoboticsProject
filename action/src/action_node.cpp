#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include <cmath>
#include <tf/transform_datatypes.h>
#include "geometry_msgs/Point.h"

class action {
private:

    ros::NodeHandle n;

    // communication with cmd_vel to send command to the mobile robot
    ros::Publisher pub_cmd_vel;

    // communication with odometry
    ros::Publisher pub_change_odometry;
    ros::Subscriber sub_odometry;

    // communication with decision
    ros::Publisher pub_action_done;
    ros::Subscriber sub_action_to_do;

    // communication with person_detector or person_tracker
    ros::Publisher pub_goal_reached;
    ros::Subscriber sub_goal_to_reach;

    int cond_action = 0;
    int action_done;

    float translation_to_do, translation_done;
    float rotation_to_do, rotation_done;

public:

    action() {

        // communication with cmd_vel to command the mobile robot
        pub_cmd_vel = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);

        // communication with odometry
        pub_change_odometry = n.advertise<geometry_msgs::Point>("change_odometry", 1);
        sub_odometry = n.subscribe("odom", 1, &rotation_action::odomCallback, this);
        cond_rotation = 0;

        // communication with decision
        pub_goal_reached = n.advertise<geometry_msgs::Point>("goal_reached", 1);
        sub_goal_to_reach = n.subscribe("goal_to_reach", 1, &action::goal_to_reachCallback,
                                        this);//this is the rotation that has to be performed

    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr &o) {

        rotation_done = tf::getYaw(o->pose.pose.orientation);
        float tx = o->pose.pose.position.x;
        float ty = o->pose.pose.position.y;
        translation_done = sqrt((tx * tx) + (ty * ty));
        ROS_INFO("(action_node) action_done %f", rotation_done * 180 / M_PI);

        if (cond_action) {
            geometry_msgs::Twist twist;
            twist.linear.x = 0;
            twist.linear.y = 0;
            twist.linear.z = 0;

            twist.angular.x = 0;
            twist.angular.y = 0;
            twist.angular.z = 0;

            ROS_INFO("(rotation_node) rotation_done: %f, rotation_to_do: %f", rotation_done * 180 / M_PI,
                     rotation_to_do * 180 / M_PI);
            float threshold = 0.1;

            //boolean to say if the twist changed
            int twistToPublish = 0;

            if (rotation_to_do - threshold >= rotation_done || rotation_to_do + threshold <= rotation_done) {

                float kp, ep, ki, ei, kd, ed;

                //implementation of a PID controller

                ep = (rotation_to_do -
                      rotation_done);//the current error is the difference between the rotation_to_do (ie, the angle to reach) and the rotation_done (ie, the current angle)

                kp = 1.0;
                ki = 0.0;
                ei = 0.0;
                kd = 0.0;
                ed = 0.0;
                //twist.angular.z = kp * ep;
                twist.angular.z *= kp * ep + ki * ei + kd * ed;
                twistToPublish = 1;

            }
            threshold = 0.2;

            if (translation_to_do - threshold >= translation_done ||
                translation_to_do + threshold <= translation_done) {

                float kp, ep, ki, ei, kd, ed;

                //implementation of a PID controller
                ep = (translation_to_do -
                      translation_done);//the current error is the difference between the translation_to_do (ie, the postion to reach) and the translation_done (ie, the current position)
                kp = 1.0;
                ki = 0.0;
                ei = 0.0;
                kd = 0.0;
                ed = 0.0;

                //twist.linear.x = kp * ep;
                twist.linear.x *= kp * ep + ki * ei + kd * ed;
                twistToPublish = 1;
            }

            if (twistToPublish) {
                pub_cmd_vel.publish(twist);
                twistToPublish = 0;
            } else {
                cond_rotation = 0;
                ROS_INFO("(rotation_node) rotation_to_do: %f", rotation_to_do * 180 / M_PI);
                ROS_INFO("(rotation_node) rotation_done: %f", rotation_done * 180 / M_PI);

                std_msgs::Float32 msg_rotation_done;
                msg_rotation_done.data = rotation_done;
                ROS_INFO("(rotation_node) rotation_done : %f", msg_rotation_done.data * 180 / M_PI);

                pub_rotation_done.publish(msg_rotation_done);//we sent the rotation_done to decision_node;
            }
            //getchar();
        }

    }

    void goal_to_reachCallback(const std_msgs::Float32::ConstPtr &a) {
// process the rotation to do received from the decision node

        ROS_INFO("\n(action_node) processing the action received from the decision node");
        //getchar();
        rotation_to_do = a->data;

        ROS_INFO("(action_node) goal_to_reach (%f, %f)", goal_to_reach.x, goal_to_reach.y);
        //getchar();
        cond_action = 1;//we will perform a rotation

        action_done = 0;

        //reset odometry
        geometry_msgs::Point p;
        p.x = 0;
        p.y = 0;
        p.z = 0;
        pub_change_odometry.publish(p);

    }

};

int main(int argc, char **argv) {

    ros::init(argc, argv, "action");

    action bsObject;

    ros::spin();

    return 0;
}
