#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/ColorRGBA.h"
#include "std_msgs/Float32.h"
#include <cmath>
#include "nav_msgs/Odometry.h"
#include <tf/transform_datatypes.h>

// We use the already built messages types
#include <geometry_msgs/Twist.h>

using namespace std;

#define detection_range 0.05

class translation_action {
private:

    ros::Subscriber sub_scan;

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

     // receive data of laserscanner
    sub_scan = n.subscribe("scan", 1, &translation_action::scanCallback, this);

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
    ROS_INFO("(translation_node) translation_done %f", translation_done);

    if ( cond_translation ) {

        geometry_msgs::Twist twist;
        twist.linear.x = 0;
        twist.linear.y = 0;
        twist.linear.z = 0;

        twist.angular.x = 0;
        twist.angular.y = 0;
        twist.angular.z = 0;

        ROS_INFO("(translation_node) translation_done: %f, translation_to_do: %f", translation_done, translation_to_do);
        float threshold=0.2;
        if ( translation_to_do-threshold >= translation_done || translation_to_do+threshold <= translation_done  ) {
                 
                  float kp, ep, ki, ei, kd, ed;
                 
                  //implementation of a PID controller
                 
                    ep = (translation_to_do - translation_done);//the current error is the difference between the translation_to_do (ie, the postion to reach) and the translation_done (ie, the current position)
                    kp=1.0;
                    ki=0.0;
                    ei=0.0;
                    kd=0.0;
                    ed=0.0;

                   twist.linear.x = kp * ep;
                 //  twist.linear.x *= kp * ep + ki * ei + kd * ed;;
            pub_cmd_vel.publish(twist);
        }
        else {
            cond_translation = 0;
            ROS_INFO("(translation_node) translation_to_do: %f", translation_to_do);
            ROS_INFO("(translation_node) translation_done: %f", translation_done);

            std_msgs::Float32 msg_translation_done;
            msg_translation_done.data = translation_done;
            pub_translation_done.publish(msg_translation_done);//we sent the translation_done to decision_node;
        }
            //getchar();
    }
   
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

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
//the robot moves in translation at 1meter per second and stops if it perceives an obstacle at less than 1meter in angle between -20degrees and +20degrees
    ROS_INFO("--Listening to scan:");

    int obstacle = 0;
    int loop = 342;//starting angle at about -20 degrees

    float beam_angle = scan->angle_min + scan->angle_increment*loop;

    while ( ( loop <= 465 ) && not ( obstacle ) ) {//we stop if there an obstacle at less than 1 meter between -20 degres and +20degres
        obstacle = ( scan->ranges[loop] < 1 ) && ( scan->ranges[loop] > scan->range_min );
        if ( obstacle ){
            ROS_INFO("obstacle detected at %f where the distance is %f", beam_angle*180/M_PI, scan->ranges[loop]);
            ROS_WARN("------------------obstacle detected---------------------------");
            ROS_INFO("(translation_node) translation_done: %f", translation_done);
            
            cond_translation = 0;
            std_msgs::Float32 msg_translation_done;
            msg_translation_done.data = translation_done;
            pub_translation_done.publish(msg_translation_done);//we sent the translation_done to decision_node;
        }
        loop++;
        beam_angle += scan->angle_increment;
    } 

}

};



int main(int argc, char **argv){

    ros::init(argc, argv, "translation_action");

    translation_action bsObject;

    ros::spin();

    return 0;
}


