#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/PoseStamped.h"
#include "MahonyAHRS.h"

Mahony ahrs_;

ros::Publisher pub_pose_;

void imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
    ROS_DEBUG("%lf,%lf,%lf,%lf,%lf,%lf", msg->angular_velocity.x,
              msg->angular_velocity.y,
              msg->angular_velocity.z,
              msg->linear_acceleration.x,
              msg->linear_acceleration.y,
              msg->linear_acceleration.z);

    float gx = msg->angular_velocity.x;
    float gy = msg->angular_velocity.y;
    float gz = msg->angular_velocity.z;
    float ax = msg->linear_acceleration.x;
    float ay = msg->linear_acceleration.y;
    float az = msg->linear_acceleration.z;
    float mx = 0.0;
    float my = 0.0;
    float mz = 0.0;

    ahrs_.update(gx, gy, gz, ax, ay, az, mx, my, mz);

    float q[4] = {0};
    ahrs_.getQuaternion(q);

    geometry_msgs::PoseStamped posestamped;
    posestamped.header.frame_id = "imu";
    posestamped.header.stamp = msg->header.stamp;
    posestamped.pose.position.x = 0.;
    posestamped.pose.position.y = 0.;
    posestamped.pose.position.z = 0.;
    posestamped.pose.orientation.w = q[0];
    posestamped.pose.orientation.x = q[1];
    posestamped.pose.orientation.y = q[2];
    posestamped.pose.orientation.z = q[3];

    pub_pose_.publish(posestamped);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mahony");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("imu0", 1, imuCallback);
    pub_pose_ = n.advertise<geometry_msgs::PoseStamped>("ahrs_mahony", 1);

    ros::spin();
    return 0;
}