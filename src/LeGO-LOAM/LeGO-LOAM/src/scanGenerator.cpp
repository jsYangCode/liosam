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

    std::vector<sensor_msgs::Imu::ConstPtr> odomDB;

    pcl::PointCloud<PointType>::Ptr fullCloud;


    std_msgs::Header scanHead;

    PointType temp;
    const float pi = 3.141592653;

    float motorspeed = 3.141592653;
    float solution = 0.4*(3.141592653/180);

    std::mutex odom_data_mutex;
public:



    scanGenerator():
            nh("~")
    {
        subLaserCloud = nh.subscribe("/noise_lidar_point_result", 2, &scanGenerator::laserScanHandler, this);
        subOdom = nh.subscribe("/prius/revo_hd1_lidar/state", 5, &scanGenerator::motorstateHandler, this);

        pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2>
               ("/velodyne_points", 2);


    }

    ~scanGenerator(){};

    void motorstateHandler(const sensor_msgs::Imu::ConstPtr& odom_msg){
        std::lock_guard<std::mutex> lock(odom_data_mutex);
        odomDB.push_back(odom_msg);

    };

    void laserScanHandler(const sensor_msgs::LaserScan::ConstPtr& laserScanMsg){


        std::lock_guard<std::mutex> lock(odom_data_mutex);

        if(!odomDB.empty()){
            sensor_msgs::LaserScan currentScan = *laserScanMsg;
            scanHead = currentScan.header;

            auto odomcurrent = odomDB.begin();

            for(odomcurrent;odomcurrent!=odomDB.end();odomcurrent++){
                if(scanHead.stamp.toSec()<(*odomcurrent)->header.stamp.toSec()){
                    break;
                }

            };

            auto odom_iter_begin = odomDB.begin();
            auto odom_iter_end = odomcurrent-1;

            fullCloud.reset(new pcl::PointCloud<PointType>());

            float timeIncrement = currentScan.time_increment;
            float startAngle = currentScan.angle_min;

            Eigen::Vector3d point_scanPose;//点在scan中的表达
            Eigen::Vector3d point_odomPose ;//点在odom坐标系下的表达
            Eigen::Matrix3d tranform = Eigen::Matrix3d::Identity() ;//scan到odom的变换

            tf::Quaternion orientation;
            tf::quaternionMsgToTF((*odom_iter_end)->orientation, orientation);

            cout<<"currentScan.range_min = "<<currentScan.range_min<<endl;
            cout<<"currentScan.range_max = "<<currentScan.range_max<<endl;
            for (int i =0 ;i< (currentScan.ranges).size();i++){
                if(currentScan.range_min<currentScan.ranges[i]&&currentScan.range_max>currentScan.ranges[i]){

                    float pointAngle = startAngle + (currentScan.angle_increment) * i;//转角

                    float range = currentScan.ranges[i];//模长
                    //cout<<"currentScan.ranges["<<i<<"] = "<<currentScan.ranges[i]<<endl;

                    point_scanPose(0) = range * cos(pointAngle);
                    point_scanPose(1) = range * sin(pointAngle);
                    point_scanPose(2) = 0;

                    double scanAngle = orientation.getAngle() + timeIncrement * motorspeed;//scan的位置

                    Eigen::AngleAxisd rotationVector(scanAngle,static_cast<Eigen::Vector3d>(orientation.getAxis()));//绕轴旋转scanAngle角度
                    tranform = rotationVector.toRotationMatrix();


                    point_odomPose = tranform * point_scanPose;

                    temp.x = point_odomPose(0);temp.y = point_odomPose(1);temp.z = point_odomPose(2);
                    temp.intensity = currentScan.intensities[i];

                    fullCloud->push_back(temp);
                };
            }

            cout<<"fullcloud size = "<<fullCloud->size() <<endl;
            sensor_msgs::PointCloud2 laserCloudTemp;
            pcl::toROSMsg(*fullCloud, laserCloudTemp);
            laserCloudTemp.header.stamp.sec = scanHead.stamp.toSec();
            laserCloudTemp.header.frame_id = "motor";

            pubLaserCloud.publish(laserCloudTemp);
            odomDB.erase(odom_iter_begin,odom_iter_end);
        };

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

