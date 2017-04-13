// person detector using lidar data
// written by O. Aycard

#include "ros/ros.h"
#include "ros/time.h"
// We use the already built messages types
#include "sensor_msgs/LaserScan.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/ColorRGBA.h"
#include <cmath>
#include "nav_msgs/Odometry.h"
#include <tf/transform_datatypes.h>
#include "std_msgs/Float32.h"

#define detection_range 0.1

using namespace std;

class person_detector {

private:
    ros::NodeHandle n;
    ros::Subscriber sub_scan;
    ros::Publisher pub_person_detector;
    ros::Publisher pub_goal_to_reach;
    ros::Subscriber sub_goal_reached;

    // to store, process and display the laserdata
    int nb_beams;
    float range_min, range_max;
    float angle_min, angle_max, angle_inc;
    float range[1000];
    geometry_msgs::Point current_scan[1000];
    std_msgs::ColorRGBA current_colors[1000];

    //to perform detection
    float background[1000];// to store the background
    int detection[1000];// to store if the current hit of the laser is static or dynamic
    int first_scan;// to know if this is the first scan or not

    //to perform clustering
    int nb_cluster;// number of cluster
    int cluster[1000]; //to store for each hit, the cluster it belongs to
    float size_cluster[1000];// to store the size of each cluster
    geometry_msgs::Point center_cluster[1000];// to store the center of gravity of each cluster
    float dynamic_cluster[1000];// to store the percentage of the cluster that is dynamic

    //to perform search of moving legs and to store them
    int nb_moving_leg;
    int moving_leg[1000];//we store the cluster corresponding to a moving leg

    //to perform search of moving persons and to store them
    int nb_moving_person;
    geometry_msgs::Point position_moving_person[1000];

    int published;
    int goal_reached;

public:

    person_detector() {
        published=0;
        goal_reached=0;
        pub_person_detector = n.advertise<visualization_msgs::Marker>("person_detector",
                                                    1); // Preparing a topic to publish our results. This will be used by the visualization tool rviz

        sub_scan = n.subscribe("scan", 1, &person_detector::scanCallback, this);

        // communication with control_node
        pub_goal_to_reach = n.advertise<geometry_msgs::Point>("goal_to_reach",
                                                              1);     // Preparing a topic to publish the goal to reach.
        sub_goal_reached = n.subscribe("goal_reached", 1, &person_detector::goal_reachedCallback, this);

        goal_reached = 1;
        published=0;
        first_scan = 1;

    }

// Distance between two points
    float distancePoints(geometry_msgs::Point pa, geometry_msgs::Point pb) {

        return sqrt(pow((pa.x - pb.x), 2.0) + pow((pa.y - pb.y), 2.0));

    }

   void averagePoints(int point1, int point2,geometry_msgs::Point *p) {
        float sumX = 0, sumY = 0; int nbPoint = 0;
        for (int i = point1; i <= point2; i++) {
            sumX += current_scan[i].x;
            sumY += current_scan[i].y;
            nbPoint++;
        }

        p->x = sumX / (float)nbPoint;
        p->y = sumY / (float)nbPoint;
    }

    void averagePoints2(int point1, int point2,geometry_msgs::Point *p) {
        float sumX = 0, sumY = 0; int nbPoint = 0;
        
            sumX += center_cluster[point1].x;
            sumY += center_cluster[point1].y;

            sumX += center_cluster[point2].x;
            sumY += center_cluster[point2].y;

        p->x = sumX / 2.0;
        p->y = sumY / 2.0;
    }


// Draw the field of view and other references
    void populateMarkerReference() {

        visualization_msgs::Marker references;

        references.header.frame_id = "base_laser";
        references.header.stamp = ros::Time::now();
        references.ns = "person_detector";
        references.id = 1;
        references.type = visualization_msgs::Marker::LINE_STRIP;
        references.action = visualization_msgs::Marker::ADD;
        references.pose.orientation.w = 1;

        references.scale.x = 0.02;

        references.color.r = 1.0f;
        references.color.g = 1.0f;
        references.color.b = 1.0f;
        references.color.a = 1.0;
        geometry_msgs::Point v;

        v.x = 0.02 * cos(-2.356194);
        v.y = 0.02 * sin(-2.356194);
        v.z = 0.0;
        references.points.push_back(v);

        v.x = 5.6 * cos(-2.356194);
        v.y = 5.6 * sin(-2.356194);
        v.z = 0.0;
        references.points.push_back(v);

        float beam_angle = -2.356194 + 0.006136;
        // first and last beam are already included
        for (int i = 0; i < 723; i++, beam_angle += 0.006136) {
            v.x = 5.6 * cos(beam_angle);
            v.y = 5.6 * sin(beam_angle);
            v.z = 0.0;
            references.points.push_back(v);
        }

        v.x = 5.6 * cos(2.092350);
        v.y = 5.6 * sin(2.092350);
        v.z = 0.0;
        references.points.push_back(v);

        v.x = 0.02 * cos(2.092350);
        v.y = 0.02 * sin(2.092350);
        v.z = 0.0;
        references.points.push_back(v);

        pub_person_detector.publish(references);

    }

    void populateMarkerTopic(int nb_beams, geometry_msgs::Point *current_scan, std_msgs::ColorRGBA *current_colors) {

        visualization_msgs::Marker marker;

        //ROS_INFO("entree dans marker topic");
        marker.header.frame_id = "base_laser";
        marker.header.stamp = ros::Time::now();
        marker.ns = "person_detector";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::POINTS;
        marker.action = visualization_msgs::Marker::ADD;

//    marker.pose.position.x = 1;
//    marker.pose.position.y = 1;
//    marker.pose.position.z = 1;
//    marker.pose.orientation.x = 0;
//    marker.pose.orientation.y = 0;
//    marker.pose.orientation.z = 0;
        marker.pose.orientation.w = 1;

        marker.scale.x = 0.05;
        marker.scale.y = 0.05;
//    marker.scale.z = 2;

//    marker.color.g = 1.0f;
        marker.color.a = 1.0;

        //ROS_INFO("ok");
        for (int loop = 0; loop < nb_beams; loop++) {
            geometry_msgs::Point p;
            std_msgs::ColorRGBA c;

            p.x = current_scan[loop].x;
            p.y = current_scan[loop].y;
            p.z = current_scan[loop].z;

            c.r = current_colors[loop].r;
            c.g = current_colors[loop].g;
            c.b = current_colors[loop].b;
            c.a = current_colors[loop].a;

            marker.points.push_back(p);
            marker.colors.push_back(c);

        }

        pub_person_detector.publish(marker);
        populateMarkerReference();

    }

    void store_background() {
// store all the hits of the laser in the background table

        ROS_INFO("store_background");
        for (int i = 0; i <= 1000; i++) {
            background[i] = range[i];
        }
        float beam_angle = angle_min;

    }

    void detect_motion() {
// for each hit, compare the current range with the background to detect motion
// if the difference between the background and the current range is over than a given threshold (0.2 meters for instance)
// then detection is equal to 1
// else it is equal to 0

        ROS_INFO("detect_motion");
        for (int loop = 0; loop < 1000; loop++) {
            if (background[loop] - range[loop] < 0.1) {
                detection[loop] = 0;
            } else {
                detection[loop] = 1;
            }
            current_colors[loop].r = 0.0;
            current_colors[loop].g = 1.0f;
            current_colors[loop].b = 0.0;
            current_colors[loop].a = 1.0;
        }


    }

    void clustering() {
//store in the table cluster, the cluster of each hit of the laser
//if the distance between the previous hit of the laser and the current one is lower than a threshold
//then the two hits r in the same cluster
//else we start a new cluster with the current hit
//we use the table "cluster" to store the result
        float threshold = 0.05f;

        int start_cluster, end_cluster;// to store the start and end of each cluster
        ROS_INFO("clustering");
        int nb_dynamic = 0;//to count the number of hits that r dynamic for the current cluster
        int i;// variable used to perform a loop over all the hits

        nb_cluster = 0;//use to count the number of clusters
        cluster[0] = 0;
        start_cluster = 0;
        for (i = 0; i < 1000; i++) {
            if (distancePoints(current_scan[i], current_scan[i - 1]) > threshold) {
                //create a new Cluster
                end_cluster = i - 1;
                averagePoints(start_cluster, end_cluster,&center_cluster[nb_cluster]);
                size_cluster[nb_cluster] = distancePoints(current_scan[end_cluster], current_scan[start_cluster]);
                dynamic_cluster[nb_cluster] = ((float) nb_dynamic) / ((float) (i - start_cluster));

                nb_cluster++;
                nb_dynamic = 0;
                start_cluster = i;
            } else {
                //same cluster
            }
            cluster[i] = nb_cluster;
            if (detection[i] == 1) {
                nb_dynamic++;
            }
        }
        end_cluster = i;
        averagePoints(start_cluster, end_cluster,&center_cluster[nb_cluster]);
        size_cluster[nb_cluster] = distancePoints(current_scan[end_cluster], current_scan[start_cluster]);
        dynamic_cluster[nb_cluster] = ((float) nb_dynamic) / ((float) (i - start_cluster));

        float beam_angle = angle_min;

        //ROS_INFO("cluster = %d, size = %f, center_cluster(%f, %f), percentage_dynamic = %f", nb_cluster, size_cluster[nb_cluster], center_cluster[nb_cluster].x, center_cluster[nb_cluster].y, dynamic_cluster[nb_cluster]);

        //getchar();

    }

    void search_moving_leg() {
//for each cluster, we search if its a moving leg. A moving leg is cluster with a size between 0.05meter and 0.25meter and with at least 75% of its hits that r dynamic
// we store the cluster corresponding to a moving leg in the table moving_leg

        ROS_INFO("search_moving_leg");
        nb_moving_leg = 0;// to count the number of moving legs
        int loop, loop2;// variables used to perform a loop over all the clusters and over all the hits

        float beam_angle = angle_min;

        for (loop = 0; loop <= nb_cluster; loop++) {
            if (size_cluster[loop] >= 0.05 && size_cluster[loop] <= 0.25 && dynamic_cluster[loop] >= 0.75f) {
                moving_leg[loop] = 1;
                nb_moving_leg++;
                for (loop2 = 0; loop2 < nb_beams; ++loop2) {
                    if(cluster[loop2]==loop) {
                        current_colors[loop2].r = 0.0;
                        current_colors[loop2].g = 0.0f;
                        current_colors[loop2].b = 1.0;
                        current_colors[loop2].a = 1.0;
                    }
                }
                printf("%f %f\n", center_cluster[loop].x,center_cluster[loop].y);
            } else {
                moving_leg[loop] = 0;
            }

        }

        //populateMarkerTopic(nb_beams, current_scan, current_colors);
        //    getchar();

    }

    void search_moving_person() {
// a moving person has two moving legs that r located within 0.5 meters of each other

        ROS_INFO("search_moving_person");
        nb_moving_person = 0;
        int loop, loop2, loop3;//variables used to perform loop over the two legs

        for (loop = 0; loop < nb_cluster; loop++) {
            if(moving_leg[loop]==1) {
                for (loop2 = loop + 1; loop2 < nb_cluster; loop2++) {
                    if(moving_leg[loop2]==1) {
                        if (distancePoints(center_cluster[loop], center_cluster[loop2]) <= 0.5) {
                            averagePoints2(loop, loop2,&position_moving_person[nb_moving_person]);
                            //position_moving_person[nb_moving_person]=center_cluster[loop];
                            nb_moving_person++;
                            for (loop3 = 0; loop3 < nb_beams; loop3++) {
                                if (cluster[loop3] == loop || cluster[loop3] == loop2) {
                                    current_colors[loop3].r = 1.0f;
                                    current_colors[loop3].g = 1.0;
                                    current_colors[loop3].b = 0.0f;
                                    current_colors[loop3].a = 1.0;
                                }
                            }
                        }
                    }
                }
            }
        }

        //populateMarkerTopic(nb_beams, current_scan, current_colors);
        //getchar();

    }

    void choose_goal() {
//within all the moving person in the environment, we have to choose one that we will follow

        ROS_INFO("choose_goal");
        printf("%d %d", nb_moving_person,published);
        if(nb_moving_person>0 && published==0) {
            published=1;
            goal_reached=0;
            pub_goal_to_reach.publish(position_moving_person[0]);
            printf("%f %f \n",position_moving_person[0].x,position_moving_person[0].y);
            ROS_INFO("publishing point %f ",position_moving_person[0].x);
        }

        populateMarkerTopic(nb_beams, current_scan, current_colors);
        //getchar();

    }

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan) {

        ROS_INFO("--Listening to scan:");
        ROS_INFO("goal_reached = %d", goal_reached);
        ROS_INFO("first_scan = %d", first_scan);
        printf("%d", goal_reached);
        if (goal_reached) {

            ROS_INFO("Timestamp: %d", scan->header.seq);

            // store the important data related to laserscanner
            range_min = scan->range_min;
            range_max = 5;//scan->range_max; we decide that the range is limited to 5 meters
            angle_min = scan->angle_min;
            angle_max = scan->angle_max;
            angle_inc = scan->angle_increment;
            nb_beams = ((-1 * angle_min) + angle_max) / angle_inc;

            ROS_INFO("range_min, range_max: %f, %f", range_min, range_max);
            ROS_INFO("angle_min: %f", angle_min * 180 / M_PI);
            ROS_INFO("angle_max: %f", angle_max * 180 / M_PI);
            ROS_INFO("angle_increment: %f", angle_inc * 180 / M_PI);
            ROS_INFO("number_of_beams: %d", nb_beams);

            // store the range and the coordinates in cartesian framework of each hit
            float beam_angle = angle_min;
            for (int loop = 0; loop < nb_beams; loop++, beam_angle += angle_inc) {
                if (scan->ranges[loop] < range_max)
                    range[loop] = scan->ranges[loop];
                else
                    range[loop] = range_max;

                //transform the scan in cartesian framewrok
                current_scan[loop].x = range[loop] * cos(beam_angle);
                current_scan[loop].y = range[loop] * sin(beam_angle);
                current_scan[loop].z = 0.0;

            }

            if (first_scan) {
                store_background();
                first_scan = 0;

            } else {

                detect_motion();
                clustering();
                search_moving_leg();
                search_moving_person();
               choose_goal();

            }

        }
    }

    void goal_reachedCallback(const geometry_msgs::Point::ConstPtr &g) {

    }

};


int main(int argc, char **argv) {

    ros::init(argc, argv, "person_detector");

    person_detector bsObject;

    ros::spin();

    return 0;
}


