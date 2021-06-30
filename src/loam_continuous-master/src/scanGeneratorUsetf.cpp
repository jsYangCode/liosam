#include <cmath>
#include <vector>
#include <mutex>

#include <pcl/point_types.h>
#include <opencv/cv.h>
#include <nav_msgs/Odometry.h>
#include <opencv/cv.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/LaserScan.h>

#include <tf/transform_listener.h>


//
// Created by bb on 19-3-4.
//
using namespace std;
using std::sin;
using std::cos;
using std::atan2;

typedef pcl::PointXYZI PointType;



class scanGenerator{
private:
    ros::NodeHandle nh;

    ros::Subscriber subLaserCloud;
    ros::Subscriber subOdom;

    ros::Publisher pubLaserCloud;

    pcl::PointCloud<PointType>::Ptr fullCloud;


    std_msgs::Header scanHead;

    PointType temp;
    const float pi = 3.141592653;

    float motorspeed = 3.141592653;
    float solution = 0.4*(3.141592653/180);

    tf::TransformListener motorstate;
    bool tfstate = false;
    bool systeminit = false;

    double initTime;
    double timeLaserCloudCur;
    double timeLasted;
public:

    scanGenerator():
            nh("~")
    {
        subLaserCloud = nh.subscribe("/noise_lidar_point_result", 2, &scanGenerator::laserScanHandler, this);

        pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2>
               ("/velodyne_points", 2);
    }

    ~scanGenerator(){};


    void laserScanHandler(const sensor_msgs::LaserScan::ConstPtr& laserScanMsg){

        if(!systeminit){
            initTime = laserScanMsg->header.stamp.toSec();
            systeminit = true;
        };

        timeLaserCloudCur = laserScanMsg->header.stamp.toSec();
        timeLasted = timeLaserCloudCur - initTime;

        if(!tfstate){
            if((!motorstate.frameExists(laserScanMsg->header.frame_id))||(timeLasted<1)){
                return;
            };
            tfstate = true;
            cout<<"tf is ready!"<<endl;
        }

        sensor_msgs::LaserScan currentScan = *laserScanMsg;
        scanHead = currentScan.header;

        fullCloud.reset(new pcl::PointCloud<PointType>());

        float timeIncrement = currentScan.time_increment;
        float startAngle = currentScan.angle_min;

        Eigen::Vector3d point_scanPose;//点在scan中的表达
        Eigen::Vector3d point_odomPose ;//点在odom坐标系下的表达
        Eigen::Matrix3d tranform = Eigen::Matrix3d::Identity() ;//scan到odom的变换

        //cout<<"currentScan.range_min = "<<currentScan.range_min<<endl;
        //cout<<"currentScan.range_max = "<<currentScan.range_max<<endl;
        for (int i =0 ;i< (currentScan.ranges).size();i++){
            if(currentScan.range_min<currentScan.ranges[i]&&currentScan.range_max>currentScan.ranges[i]){

                float pointAngle = startAngle + (currentScan.angle_increment) * i;//转角

                float range = currentScan.ranges[i];//模长
                //cout<<"currentScan.ranges["<<i<<"] = "<<currentScan.ranges[i]<<endl;

                ros::Time currenttime=ros::Time::now();

                currenttime = scanHead.stamp+ros::Duration(currentScan.time_increment * i);
                //cout<<"currenttime"<<currenttime<<endl;
                motorstate.waitForTransform("/motor","/lidar",currenttime,ros::Duration(1));

                tf::StampedTransform transform1;

                motorstate.lookupTransform("/motor","/lidar",currenttime,transform1);

                double motor_roll,motor_pitch,motor_yaw;
                tf::Matrix3x3(transform1.getRotation()).getRPY(motor_roll,motor_pitch,motor_yaw);

               // if(motor_roll>=pi){
                 //   motor_roll-=pi;
                   // pointAngle= -pointAngle;
               // }

                Eigen::AngleAxisd rotationVector(motor_roll, Eigen::Vector3d(1,0,0));
                tranform = rotationVector.toRotationMatrix();//tf::StampedTransform到Eigen::Matrix3d

                point_scanPose(0) = range * cos(pointAngle);
                point_scanPose(1) = range * sin(pointAngle);
                point_scanPose(2) = 0;
                //cout<<"tf::StampedTransform到Eigen::Matrix3d succeed"<<endl;
                point_odomPose = tranform * point_scanPose;

                temp.x = point_odomPose(0);temp.y = point_odomPose(1);temp.z = point_odomPose(2);
                temp.intensity = currentScan.intensities[i];

                fullCloud->push_back(temp);
            };
        }

        if(fullCloud->size()<10){
            cout<<"fullcloud size = "<<fullCloud->size() <<endl;
        }

        sensor_msgs::PointCloud2 laserCloudTemp;
        pcl::toROSMsg(*fullCloud, laserCloudTemp);
        laserCloudTemp.header.stamp = scanHead.stamp;
        laserCloudTemp.header.frame_id = "motor";

        pubLaserCloud.publish(laserCloudTemp);

    };

};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "lego_loam");

    scanGenerator SG;
    cout<<"scanGenerator"<<endl;


    ros::spin();

    return 0;
}

