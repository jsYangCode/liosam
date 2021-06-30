//
// Created by gc on 19-1-21.
//
#include "utility.h"

//todo gc
#include <lego_loam/SaveMap.h>
#include <sensor_msgs/NavSatFix.h>
#include <fstream>

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>

#include <gtsam/nonlinear/ISAM2.h>

#include <cmath>


#include <nav_msgs/Odometry.h>
#include <opencv/cv.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

ros::Publisher imu_pub;
double yaw;
std::mutex yaw_mutex;

void gpsHandler(const sensor_msgs::NavSatFixConstPtr& gpsIn)
{
    if(gpsIn->altitude == 0)
        return;
    std::lock_guard<std::mutex> lock(yaw_mutex);
    yaw = 2*M_PI - gpsIn->altitude * 2*M_PI /360.0;//2*M_PI - gpsIn->altitude * 2*M_PI /360.0;
    std::cout << gpsIn->altitude << std::endl;
}


void imuTransHandler(const sensor_msgs::Imu::ConstPtr& imuIn)//imu的数据,只是以pointcloud形式存储
{
    sensor_msgs::Imu imu_data;
    tf::Quaternion q_init;
    q_init.setX(imuIn->orientation.x);
    q_init.setY(imuIn->orientation.y);
    q_init.setZ(imuIn->orientation.z);
    q_init.setW(imuIn->orientation.w);
    double roll_init, pitch_init, yaw_init;

    tf::Matrix3x3(q_init).getRPY(roll_init, pitch_init, yaw_init);
    std::lock_guard<std::mutex> lock(yaw_mutex);
    double roll_after, pitch_after, yaw_after;
    roll_after = roll_init;
    pitch_after = pitch_init;
    yaw = yaw + imuIn->angular_velocity.z * 0.01;

    geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw(roll_init, pitch_init, yaw);

    imu_data.orientation.x = geoQuat.x;//imuIn->orientation.x;
    imu_data.orientation.y = geoQuat.y;//imuIn->orientation.y;
    imu_data.orientation.z = geoQuat.z;//imuIn->orientation.z;
    imu_data.orientation.w = geoQuat.w;//imuIn->orientation.w;

    imu_data.angular_velocity = imuIn->angular_velocity;
    imu_data.linear_acceleration = imuIn->linear_acceleration;

    imu_data.header = imuIn->header;
    imu_pub.publish(imu_data);

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "imu_gps");
    ros::NodeHandle nh;
    yaw = 2 * M_PI - 337.857940674 * 2 * M_PI /360.0;//2*M_PI - 337.857940674 * 2 * M_PI /360.0 ;


    imu_pub = nh.advertise<sensor_msgs::Imu>("/imu_yaw_gps", 2);
    ROS_INFO("\033[1;32m---->\033[0m imu_gps Started.");



    ros::Subscriber subImuTrans = nh.subscribe<sensor_msgs::Imu>
            ("/imu_data", 5, imuTransHandler);
    ros::Subscriber subGps = nh.subscribe<sensor_msgs::NavSatFix>("/GNSS_data",10,gpsHandler);

    ros::Rate rate(200);
    while (ros::ok())
    {
        ros::spinOnce();



        rate.sleep();
    }


}
