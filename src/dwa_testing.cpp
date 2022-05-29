#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

void chatterCallback(const nav_msgs::OdometryConstPtr &msg)
{
    // ROS_INFO("I heard: [%f]", msg->twist.twist.linear.x);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "testing");

    ROS_INFO("HOLA");

    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/odom", 1000, chatterCallback);

    ros::spin();

    return 0;
}
