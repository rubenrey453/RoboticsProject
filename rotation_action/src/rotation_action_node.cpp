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

class rotation_action {
private:

    ros::NodeHandle n;

    // communication with cmd_vel to send command to the mobile robot
    ros::Publisher pub_cmd_vel;

    // communication with odometry
    ros::Publisher pub_change_odometry;
    ros::Subscriber sub_odometry;

    // communication with decision
    ros::Publisher pub_rotation_done;
    ros::Subscriber sub_rotation_to_do;

    float rotation_to_do, rotation_done;
    int cond_rotation;// boolean to check if we still have to rotate or not

public:

rotation_action() {

    // communication with cmd_vel to command the mobile robot
    pub_cmd_vel = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    // communication with odometry
    pub_change_odometry = n.advertise<geometry_msgs::Point>("change_odometry", 1);
    sub_odometry = n.subscribe("odom", 1, &rotation_action::odomCallback, this);
    cond_rotation = 0;

    // communication with decision
    pub_rotation_done = n.advertise<std_msgs::Float32>("rotation_done", 1);
    sub_rotation_to_do = n.subscribe("rotation_to_do", 1, &rotation_action::rotation_to_doCallback, this);//this is the rotation that has to be performed

}

void odomCallback(const nav_msgs::Odometry::ConstPtr& o) {

    rotation_done = tf::getYaw(o->pose.pose.orientation);
    ROS_INFO("(rotation_node) rotation_done %f", rotation_done*180/M_PI);

    if ( cond_rotation ) {
        geometry_msgs::Twist twist;
        twist.linear.x = 0;
        twist.linear.y = 0;
        twist.linear.z = 0;

        twist.angular.x = 0;
        twist.angular.y = 0;
        twist.angular.z = 0;

        ROS_INFO("(rotation_node) rotation_done: %f, rotation_to_do: %f", rotation_done*180/M_PI, rotation_to_do*180/M_PI);
        float threshold=0.01;
        if ( rotation_to_do-threshold >= rotation_done || rotation_to_do+threshold <= rotation_done ) {
        
          float kp, ep, ki, ei, kd, ed;
         
          //implementation of a PID controller
         
            ep = (rotation_to_do - rotation_done);//the current error is the difference between the rotation_to_do (ie, the angle to reach) and the rotation_done (ie, the current angle)

            kp=1.0;
            ki=0.0;
            ei=0.0;
            kd=0.0;
            ed=0.0;
            twist.angular.z = kp * ep;
           // twist.angular.z *= kp * ep + ki * ei + kd * ed;;

            pub_cmd_vel.publish(twist);
        }
        else {
            cond_rotation = 0;
            ROS_INFO("(rotation_node) rotation_to_do: %f", rotation_to_do*180/M_PI);
            ROS_INFO("(rotation_node) rotation_done: %f", rotation_done*180/M_PI);

            std_msgs::Float32 msg_rotation_done;
            msg_rotation_done.data = rotation_done;
            ROS_INFO("(rotation_node) rotation_done : %f", msg_rotation_done.data*180/M_PI);

            pub_rotation_done.publish(msg_rotation_done);//we sent the rotation_done to decision_node;
        }
            //getchar();
    }
    
 }

void rotation_to_doCallback(const std_msgs::Float32::ConstPtr & a) {
// process the rotation to do received from the decision node

    ROS_INFO("\n(rotation_node) processing the rotation_to_do received from the decision node");
    //getchar();
    rotation_to_do = a->data;

    ROS_INFO("(rotation_node) rotation_to_do : %f", rotation_to_do*180/M_PI);
    //getchar();
    cond_rotation = 1;//we will perform a rotation

    rotation_done = 0;

    //reset odometry
    geometry_msgs::Point p;
    p.x = 0;
    p.y = 0;
    p.z = 0;
    pub_change_odometry.publish(p);

}

};

int main(int argc, char **argv){

    ros::init(argc, argv, "rotation_action");

    rotation_action bsObject;

    ros::spin();

    return 0;
}
