#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include "classbox.h"

struct tempClassbox { // a struct for making initalizing classbox message easy
    float xmin;
    float xmax;
    float ymin;
    float ymax;
    char* fruit_name;
    char* disease;
    int disease_score;
};

int main(int argc, char **argv) {

//  dummy classbox objects
    tempClassbox arr[] = {
            { xmin: 0.2, xmax: 0.6, ymin: -0.2, ymax: 0, fruit_name: "apple", disease: "mold",  disease_score: 4},
            {xmin: 0.1, xmax: 0.5, ymin: 0.1, ymax: 0.3, fruit_name: "orange", disease: "undefined", disease_score: 0},
            {xmin: 0.2, xmax: 0.4, ymin: 0.5, ymax: 0.8, fruit_name: "banana", disease: "mold", disease_score: 2}
    };

    robot_move::classbox classboxes [3];

//  initalize classbox message of classbox.h
    for (int i = 0; i < 3; i++) {
        classboxes[i].xmin = arr[i].xmin;
        classboxes[i].xmax = arr[i].xmax;
        classboxes[i].ymin = arr[i].ymin;
        classboxes[i].ymin = arr[i].ymin;
        classboxes[i].ymax = arr[i].ymax;
        classboxes[i].fruit_name = arr[i].fruit_name;
        classboxes[i].disease = arr[i].disease;
        classboxes[i].disease_score = arr[i].disease_score;
    }

    ros::init(argc, argv, "talker");
    ros::NodeHandle n;
//  publish to topic chatter
    ros::Publisher chatter_pub = n.advertise<robot_move::classbox>("chatter", 1000);

//  Publish each message with a delay of 5 seconds before each publish
    for (int i = 0; i < 3; i++) {
        ros::Duration d(5);
        d.sleep();
        ROS_INFO("sending about fruit with disease %s", classboxes[i].disease.c_str());
        ROS_INFO("sending about fruit with score %i", classboxes[i].disease_score);
        chatter_pub.publish(classboxes[i]);
        ros::spinOnce();
    }
    return 0;
}
