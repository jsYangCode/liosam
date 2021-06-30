#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/conversions.h>
// pcl fromROSMsg() has changed, need to include <pcl_conversions/pcl_conversions.h> header
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

const double PI = 3.1415926;
const double rad2deg = 180 / PI;
const double deg2rad = PI / 180;

double initTime;
double timeStart;
double timeLasted;
bool systemInited = false;

double timeScanCur = 0;
double timeScanLast = 0;

int laserRotDir = 1;

int skipFrameNum ;//多少线为一帧
int skipFrameCount = 0;

float scanDuration;

pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudExtreCur(new pcl::PointCloud<pcl::PointXYZHSV>());
pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudLessExtreCur(new pcl::PointCloud<pcl::PointXYZHSV>());

sensor_msgs::PointCloud2 laserCloudExtreCur2;
sensor_msgs::PointCloud2 laserCloudLast2;

ros::Publisher* pubLaserCloudExtreCurPointer;
ros::Publisher* pubLaserCloudLastPointer;

int cloudSortInd[900];//曲率点对应的序号
int cloudNeighborPicked[900];//点是否筛选过标志：0-未筛选过，1-筛选过

int imuPointerFront = 0;//imu时间戳大于当前点云时间戳的位置
int imuPointerLast = -1;//imu最新收到的点在数组中的位置
const int imuQueLength = 50;//imu循环队列长度
bool imuInited = false;

//点云数据开始第一个点的位移/速度/欧拉角
//当前点的速度，位移信息
float imuRollStart, imuPitchStart, imuYawStart;
float imuRollCur, imuPitchCur, imuYawCur;

float imuVeloXStart, imuVeloYStart, imuVeloZStart;
float imuShiftXStart, imuShiftYStart, imuShiftZStart;
float imuVeloXCur, imuVeloYCur, imuVeloZCur;
float imuShiftXCur, imuShiftYCur, imuShiftZCur;

//每次点云数据当前点相对于开始第一个点的畸变位移，速度
float imuShiftFromStartXCur, imuShiftFromStartYCur, imuShiftFromStartZCur;
float imuVeloFromStartXCur, imuVeloFromStartYCur, imuVeloFromStartZCur;

//IMU信息
double imuTime[imuQueLength] = {0};
float imuRoll[imuQueLength] = {0};
float imuPitch[imuQueLength] = {0};
float imuYaw[imuQueLength] = {0};

float imuAccX[imuQueLength] = {0};
float imuAccY[imuQueLength] = {0};
float imuAccZ[imuQueLength] = {0};

float imuVeloX[imuQueLength] = {0};
float imuVeloY[imuQueLength] = {0};
float imuVeloZ[imuQueLength] = {0};

float imuShiftX[imuQueLength] = {0};
float imuShiftY[imuQueLength] = {0};
float imuShiftZ[imuQueLength] = {0};

//double imuAccuRoll = 0;
//double imuAccuPitch = 0;
double imuAccuYaw = 0;

//计算局部坐标系下点云中的点相对第一个开始点的由于加减速运动产生的位移畸变
void ShiftToStartIMU()
{
    //绕y轴旋转(-imuYawStart)，即Ry(yaw).inverse
  float x1 = cos(imuYawStart) * imuShiftFromStartXCur - sin(imuYawStart) * imuShiftFromStartZCur;
  float y1 = imuShiftFromStartYCur;
  float z1 = sin(imuYawStart) * imuShiftFromStartXCur + cos(imuYawStart) * imuShiftFromStartZCur;

    //绕x轴旋转(-imuPitchStart)，即Rx(pitch).inverse
  float x2 = x1;
  float y2 = cos(imuPitchStart) * y1 + sin(imuPitchStart) * z1;
  float z2 = -sin(imuPitchStart) * y1 + cos(imuPitchStart) * z1;

    //绕z轴旋转(-imuRollStart)，即Rz(pitch).inverse
  imuShiftFromStartXCur = cos(imuRollStart) * x2 + sin(imuRollStart) * y2;
  imuShiftFromStartYCur = -sin(imuRollStart) * x2 + cos(imuRollStart) * y2;
  imuShiftFromStartZCur = z2;
}

//计算局部坐标系下点云中的点相对第一个开始点由于加减速产生的的速度畸变（增量）
void VeloToStartIMU()
{
    //绕y轴旋转(-imuYawStart)，即Ry(yaw).inverse
  float x1 = cos(imuYawStart) * imuVeloFromStartXCur - sin(imuYawStart) * imuVeloFromStartZCur;
  float y1 = imuVeloFromStartYCur;
  float z1 = sin(imuYawStart) * imuVeloFromStartXCur + cos(imuYawStart) * imuVeloFromStartZCur;

    //绕x轴旋转(-imuPitchStart)，即Rx(pitch).inverse
  float x2 = x1;
  float y2 = cos(imuPitchStart) * y1 + sin(imuPitchStart) * z1;
  float z2 = -sin(imuPitchStart) * y1 + cos(imuPitchStart) * z1;

    //绕z轴旋转(-imuRollStart)，即Rz(pitch).inverse
  imuVeloFromStartXCur = cos(imuRollStart) * x2 + sin(imuRollStart) * y2;
  imuVeloFromStartYCur = -sin(imuRollStart) * x2 + cos(imuRollStart) * y2;
  imuVeloFromStartZCur = z2;
}

//去除点云加减速产生的位移畸变
void TransformToStartIMU(pcl::PointXYZHSV *p)
{
    /********************************************************************************
  Ry*Rx*Rz*Pl, transform point to the global frame
*********************************************************************************/
    //绕z轴旋转(imuRollCur)
  float x1 = cos(imuRollCur) * p->x - sin(imuRollCur) * p->y;
  float y1 = sin(imuRollCur) * p->x + cos(imuRollCur) * p->y;
  float z1 = p->z;

    //绕x轴旋转(imuPitchCur)
  float x2 = x1;
  float y2 = cos(imuPitchCur) * y1 - sin(imuPitchCur) * z1;
  float z2 = sin(imuPitchCur) * y1 + cos(imuPitchCur) * z1;

    //绕y轴旋转(imuYawCur)
  float x3 = cos(imuYawCur) * x2 + sin(imuYawCur) * z2;
  float y3 = y2;
  float z3 = -sin(imuYawCur) * x2 + cos(imuYawCur) * z2;

    /********************************************************************************
  Rz(pitch).inverse * Rx(pitch).inverse * Ry(yaw).inverse * Pg
  transfrom global points to the local frame
*********************************************************************************/

    //绕y轴旋转(-imuYawStart)
  float x4 = cos(imuYawStart) * x3 - sin(imuYawStart) * z3;
  float y4 = y3;
  float z4 = sin(imuYawStart) * x3 + cos(imuYawStart) * z3;

    //绕x轴旋转(-imuPitchStart)
  float x5 = x4;
  float y5 = cos(imuPitchStart) * y4 + sin(imuPitchStart) * z4;
  float z5 = -sin(imuPitchStart) * y4 + cos(imuPitchStart) * z4;

    //绕z轴旋转(-imuRollStart)，然后叠加平移量
  p->x = cos(imuRollStart) * x5 + sin(imuRollStart) * y5 + imuShiftFromStartXCur;
  p->y = -sin(imuRollStart) * x5 + cos(imuRollStart) * y5 + imuShiftFromStartYCur;
  p->z = z5 + imuShiftFromStartZCur;
}

//积分速度与位移
void AccumulateIMUShift()
{
  float roll = imuRoll[imuPointerLast];
  float pitch = imuPitch[imuPointerLast];
  float yaw = imuYaw[imuPointerLast];
  float accX = imuAccX[imuPointerLast];
  float accY = imuAccY[imuPointerLast];
  float accZ = imuAccZ[imuPointerLast];

    //将当前时刻的加速度值绕交换过的ZXY固定轴（原XYZ）分别旋转(roll, pitch, yaw)角，转换得到世界坐标系下的加速度值(right hand rule)
    //绕z轴旋转(roll)
  float x1 = cos(roll) * accX - sin(roll) * accY;
  float y1 = sin(roll) * accX + cos(roll) * accY;
  float z1 = accZ;
    //绕x轴旋转(pitch)
  float x2 = x1;
  float y2 = cos(pitch) * y1 - sin(pitch) * z1;
  float z2 = sin(pitch) * y1 + cos(pitch) * z1;
    //绕y轴旋转(yaw)
  accX = cos(yaw) * x2 + sin(yaw) * z2;
  accY = y2;
  accZ = -sin(yaw) * x2 + cos(yaw) * z2;

    //上一个imu点
  int imuPointerBack = (imuPointerLast + imuQueLength - 1) % imuQueLength;
    //上一个点到当前点所经历的时间，即计算imu测量周期
  double timeDiff = imuTime[imuPointerLast] - imuTime[imuPointerBack];
    //要求imu的频率至少比lidar高，这样的imu信息才使用，后面校正也才有意义
  if (timeDiff < 0.1) {//（隐含从静止开始运动）
     //求每个imu时间点的位移与速度,两点之间视为匀加速直线运动
    imuShiftX[imuPointerLast] = imuShiftX[imuPointerBack] + imuVeloX[imuPointerBack] * timeDiff 
                              + accX * timeDiff * timeDiff / 2;
    imuShiftY[imuPointerLast] = imuShiftY[imuPointerBack] + imuVeloY[imuPointerBack] * timeDiff 
                              + accY * timeDiff * timeDiff / 2;
    imuShiftZ[imuPointerLast] = imuShiftZ[imuPointerBack] + imuVeloZ[imuPointerBack] * timeDiff 
                              + accZ * timeDiff * timeDiff / 2;

    imuVeloX[imuPointerLast] = imuVeloX[imuPointerBack] + accX * timeDiff;
    imuVeloY[imuPointerLast] = imuVeloY[imuPointerBack] + accY * timeDiff;
    imuVeloZ[imuPointerLast] = imuVeloZ[imuPointerBack] + accZ * timeDiff;
  }
}

//接收点云数据,输入点云的坐标系位x轴向前，y轴向左，z轴向上的右手坐标系
void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudIn2)
{
  if (!systemInited) {//初始化，
    initTime = laserCloudIn2->header.stamp.toSec();
    imuPointerFront = (imuPointerLast + 1) % imuQueLength;//当前imu,与下一时刻imu
    systemInited = true;
  }


  timeScanLast = timeScanCur;//todo 上一帧时间？
  timeScanCur = laserCloudIn2->header.stamp.toSec();//当前帧时间
  timeLasted = timeScanCur - initTime;//当前帧相对于初始化时间点的时间差
  pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudIn(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*laserCloudIn2, *laserCloudIn);//点云容器
  int cloudInSize = laserCloudIn->points.size();//点云大小

  int cloudSize = 0;
  pcl::PointXYZHSV laserPointIn;//（空间位置 + HSV颜色空间表示的颜色）
  pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloud(new pcl::PointCloud<pcl::PointXYZHSV>());
  for (int i = 0; i < cloudInSize; i++) {
    laserPointIn.x = laserCloudIn->points[i].x;//坐标系变换，这里无变换
    laserPointIn.y = laserCloudIn->points[i].y;
    laserPointIn.z = laserCloudIn->points[i].z;
    laserPointIn.h = timeLasted;//用时间区分每一帧
    laserPointIn.v = 0;

    if (!(fabs(laserPointIn.x) < 0.5 && fabs(laserPointIn.y) < 0.5 && fabs(laserPointIn.z) < 0.5)) {//去除无效点
      laserCloud->push_back(laserPointIn);
      cloudSortInd[cloudSize] = cloudSize;
      cloudNeighborPicked[cloudSize] = 0;
      cloudSize++;
    }
  }

  pcl::PointXYZ laserPointFirst = laserCloudIn->points[5];//起始点
  pcl::PointXYZ laserPointLast = laserCloudIn->points[cloudInSize - 5];//终止点

  float rangeFirst = sqrt(laserPointFirst.x * laserPointFirst.x + laserPointFirst.y * laserPointFirst.y
                 + laserPointFirst.z * laserPointFirst.z);
  laserPointFirst.x /= rangeFirst;//归一化
  laserPointFirst.y /= rangeFirst;
  laserPointFirst.z /= rangeFirst;

  float rangeLast = sqrt(laserPointLast.x * laserPointLast.x + laserPointLast.y * laserPointLast.y
                 + laserPointLast.z * laserPointLast.z);
  laserPointLast.x /= rangeLast;//归一化
  laserPointLast.y /= rangeLast;
  laserPointLast.z /= rangeLast;

  float laserAngle = atan2(laserPointLast.x - laserPointFirst.x, laserPointLast.y - laserPointFirst.y);//laser扫过的角度

  bool newSweep = false;
  if (laserAngle * laserRotDir < 0 && timeLasted - timeStart > scanDuration) {//todo ywc 新加入帧的判断依据，当前帧与上一帧的时间差是否大于0.7
    laserRotDir *= -1;
    newSweep = true;
    std::cout<<"timeLasted - timeStart = "<<timeLasted - timeStart<<std::endl;
  }

  if (newSweep) {
    timeStart = timeScanLast - initTime;//上一帧结束时间，即当前帧开始时间

      //publich IMU消息,由于循环到了最后，因此是Cur都是代表最后一个点，即最后一个点的欧拉角，畸变位移及一个点云周期增加的速度
    pcl::PointCloud<pcl::PointXYZHSV>::Ptr imuTrans(new pcl::PointCloud<pcl::PointXYZHSV>(4, 1));

      //起始点欧拉角
    imuTrans->points[0].x = imuPitchStart;
    imuTrans->points[0].y = imuYawStart;
    imuTrans->points[0].z = imuRollStart;
    imuTrans->points[0].v = 10;

      //最后一个点的欧拉角
    imuTrans->points[1].x = imuPitchCur;
    imuTrans->points[1].y = imuYawCur;
    imuTrans->points[1].z = imuRollCur;
    imuTrans->points[1].v = 11;

      //最后一个点相对于第一个点的畸变位移和速度
    imuTrans->points[2].x = imuShiftFromStartXCur;
    imuTrans->points[2].y = imuShiftFromStartYCur;
    imuTrans->points[2].z = imuShiftFromStartZCur;
    imuTrans->points[2].v = 12;

    imuTrans->points[3].x = imuVeloFromStartXCur;
    imuTrans->points[3].y = imuVeloFromStartYCur;
    imuTrans->points[3].z = imuVeloFromStartZCur;
    imuTrans->points[3].v = 13;

    *laserCloudExtreCur += *laserCloudLessExtreCur;
    pcl::toROSMsg(*laserCloudExtreCur + *imuTrans, laserCloudLast2);
    laserCloudLast2.header.stamp = ros::Time().fromSec(timeScanLast);
    laserCloudLast2.header.frame_id = "/camera";
    laserCloudExtreCur->clear();
    laserCloudLessExtreCur->clear();
    imuTrans->clear();

      //记住点云起始位置的速度，位移，欧拉角
    imuRollStart = imuRollCur;
    imuPitchStart = imuPitchCur;
    imuYawStart = imuYawCur;

    imuVeloXStart = imuVeloXCur;
    imuVeloYStart = imuVeloYCur;
    imuVeloZStart = imuVeloZCur;

    imuShiftXStart = imuShiftXCur;
    imuShiftYStart = imuShiftYCur;
    imuShiftZStart = imuShiftZCur;
  }

  imuRollCur = 0; imuPitchCur = 0; imuYawCur = 0;
  imuVeloXCur = 0; imuVeloYCur = 0; imuVeloZCur = 0;
  imuShiftXCur = 0; imuShiftYCur = 0; imuShiftZCur = 0;
  if (imuPointerLast >= 0) {//如果收到IMU数据,使用IMU矫正点云畸变
      //寻找是否有点云的时间戳小于IMU的时间戳的IMU位置:imuPointerFront
    while (imuPointerFront != imuPointerLast) {
      if (timeScanCur < imuTime[imuPointerFront]) {
        break;
      }
      imuPointerFront = (imuPointerFront + 1) % imuQueLength;
    }

    if (timeScanCur > imuTime[imuPointerFront]) {//没找到,此时imuPointerFront==imtPointerLast,只能以当前收到的最新的IMU的速度，位移，欧拉角作为当前点的速度，位移，欧拉角使用
      imuRollCur = imuRoll[imuPointerFront];
      imuPitchCur = imuPitch[imuPointerFront];
      imuYawCur = imuYaw[imuPointerFront];

      imuVeloXCur = imuVeloX[imuPointerFront];
      imuVeloYCur = imuVeloY[imuPointerFront];
      imuVeloZCur = imuVeloZ[imuPointerFront];

      imuShiftXCur = imuShiftX[imuPointerFront];
      imuShiftYCur = imuShiftY[imuPointerFront];
      imuShiftZCur = imuShiftZ[imuPointerFront];
    } else {//找到了点云时间戳小于IMU时间戳的IMU位置,则该点必处于imuPointerBack和imuPointerFront之间，据此线性插值，计算点云点的速度，位移和欧拉角
      int imuPointerBack = (imuPointerFront + imuQueLength - 1) % imuQueLength;
        //按时间距离计算权重分配比率,也即线性插值
      float ratioFront = (timeScanCur - imuTime[imuPointerBack]) 
                       / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
      float ratioBack = (imuTime[imuPointerFront] - timeScanCur) 
                      / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);

      imuRollCur = imuRoll[imuPointerFront] * ratioFront + imuRoll[imuPointerBack] * ratioBack;
      imuPitchCur = imuPitch[imuPointerFront] * ratioFront + imuPitch[imuPointerBack] * ratioBack;
      if (imuYaw[imuPointerFront] - imuYaw[imuPointerBack] > PI) {
        imuYawCur = imuYaw[imuPointerFront] * ratioFront + (imuYaw[imuPointerBack] + 2 * PI) * ratioBack;
      } else if (imuYaw[imuPointerFront] - imuYaw[imuPointerBack] < -PI) {
        imuYawCur = imuYaw[imuPointerFront] * ratioFront + (imuYaw[imuPointerBack] - 2 * PI) * ratioBack;
      } else {
        imuYawCur = imuYaw[imuPointerFront] * ratioFront + imuYaw[imuPointerBack] * ratioBack;
      }
        //本质:imuVeloXCur = imuVeloX[imuPointerback] + (imuVelX[imuPointerFront]-imuVelX[imuPoniterBack])*ratioFront
      imuVeloXCur = imuVeloX[imuPointerFront] * ratioFront + imuVeloX[imuPointerBack] * ratioBack;
      imuVeloYCur = imuVeloY[imuPointerFront] * ratioFront + imuVeloY[imuPointerBack] * ratioBack;
      imuVeloZCur = imuVeloZ[imuPointerFront] * ratioFront + imuVeloZ[imuPointerBack] * ratioBack;

      imuShiftXCur = imuShiftX[imuPointerFront] * ratioFront + imuShiftX[imuPointerBack] * ratioBack;
      imuShiftYCur = imuShiftY[imuPointerFront] * ratioFront + imuShiftY[imuPointerBack] * ratioBack;
      imuShiftZCur = imuShiftZ[imuPointerFront] * ratioFront + imuShiftZ[imuPointerBack] * ratioBack;
    }
  }

  if (!imuInited) {//初始化
    imuRollStart = imuRollCur;
    imuPitchStart = imuPitchCur;
    imuYawStart = imuYawCur;

    imuVeloXStart = imuVeloXCur;
    imuVeloYStart = imuVeloYCur;
    imuVeloZStart = imuVeloZCur;

    imuShiftXStart = imuShiftXCur;
    imuShiftYStart = imuShiftYCur;
    imuShiftZStart = imuShiftZCur;

    imuInited = true;
  }

//计算之后每个点相对于第一个点的由于加减速非匀速运动产生的位移速度畸变，并对点云中的每个点位置信息重新补偿矫正
  imuShiftFromStartXCur = imuShiftXCur - imuShiftXStart - imuVeloXStart * (timeLasted - timeStart);
  imuShiftFromStartYCur = imuShiftYCur - imuShiftYStart - imuVeloYStart * (timeLasted - timeStart);
  imuShiftFromStartZCur = imuShiftZCur - imuShiftZStart - imuVeloZStart * (timeLasted - timeStart);

  ShiftToStartIMU();

  imuVeloFromStartXCur = imuVeloXCur - imuVeloXStart;
  imuVeloFromStartYCur = imuVeloYCur - imuVeloYStart;
  imuVeloFromStartZCur = imuVeloZCur - imuVeloZStart;

  VeloToStartIMU();

  for (int i = 0; i < cloudSize; i++) {
    TransformToStartIMU(&laserCloud->points[i]);
  }

  for (int i = 5; i < cloudSize - 5; i++) {////使用每个点的前后五个点计算曲率，因此前五个与最后五个点跳过
    float diffX = laserCloud->points[i - 5].x + laserCloud->points[i - 4].x 
                + laserCloud->points[i - 3].x + laserCloud->points[i - 2].x 
                + laserCloud->points[i - 1].x - 10 * laserCloud->points[i].x 
                + laserCloud->points[i + 1].x + laserCloud->points[i + 2].x
                + laserCloud->points[i + 3].x + laserCloud->points[i + 4].x
                + laserCloud->points[i + 5].x;
    float diffY = laserCloud->points[i - 5].y + laserCloud->points[i - 4].y 
                + laserCloud->points[i - 3].y + laserCloud->points[i - 2].y 
                + laserCloud->points[i - 1].y - 10 * laserCloud->points[i].y 
                + laserCloud->points[i + 1].y + laserCloud->points[i + 2].y
                + laserCloud->points[i + 3].y + laserCloud->points[i + 4].y
                + laserCloud->points[i + 5].y;
    float diffZ = laserCloud->points[i - 5].z + laserCloud->points[i - 4].z 
                + laserCloud->points[i - 3].z + laserCloud->points[i - 2].z 
                + laserCloud->points[i - 1].z - 10 * laserCloud->points[i].z 
                + laserCloud->points[i + 1].z + laserCloud->points[i + 2].z
                + laserCloud->points[i + 3].z + laserCloud->points[i + 4].z
                + laserCloud->points[i + 5].z;
      //曲率计算
    laserCloud->points[i].s = diffX * diffX + diffY * diffY + diffZ * diffZ;
  }
    //挑选点，排除容易被斜面挡住的点以及离群点，有些点容易被斜面挡住，而离群点可能出现带有偶然性，这些情况都可能导致前后两次扫描不能被同时看到
  for (int i = 5; i < cloudSize - 6; i++) {//与后一个点差值，所以减6
    float diffX = laserCloud->points[i + 1].x - laserCloud->points[i].x;
    float diffY = laserCloud->points[i + 1].y - laserCloud->points[i].y;
    float diffZ = laserCloud->points[i + 1].z - laserCloud->points[i].z;
      //计算有效曲率点与后一个点之间的距离平方和
    float diff = diffX * diffX + diffY * diffY + diffZ * diffZ;
    //前提:两个点之间距离要大于0.05
    if (diff > 0.05) {
        //点的深度
      float depth1 = sqrt(laserCloud->points[i].x * laserCloud->points[i].x + 
                     laserCloud->points[i].y * laserCloud->points[i].y +
                     laserCloud->points[i].z * laserCloud->points[i].z);
        //后一个点的深度
      float depth2 = sqrt(laserCloud->points[i + 1].x * laserCloud->points[i + 1].x + 
                     laserCloud->points[i + 1].y * laserCloud->points[i + 1].y +
                     laserCloud->points[i + 1].z * laserCloud->points[i + 1].z);
        //按照两点的深度的比例，将深度较大的点拉回后计算距离
      if (depth1 > depth2) {
        diffX = laserCloud->points[i + 1].x - laserCloud->points[i].x * depth2 / depth1;
        diffY = laserCloud->points[i + 1].y - laserCloud->points[i].y * depth2 / depth1;
        diffZ = laserCloud->points[i + 1].z - laserCloud->points[i].z * depth2 / depth1;
          //边长比也即是弧度值，若小于0.1，说明夹角比较小，斜面比较陡峭,点深度变化比较剧烈,点处在近似与激光束平行的斜面上
        if (sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ) / depth2 < 0.1) {//排除容易被斜面挡住的点
            //该点及前面五个点（大致都在斜面上）全部置为筛选过
          cloudNeighborPicked[i - 5] = 1;
          cloudNeighborPicked[i - 4] = 1;
          cloudNeighborPicked[i - 3] = 1;
          cloudNeighborPicked[i - 2] = 1;
          cloudNeighborPicked[i - 1] = 1;
          cloudNeighborPicked[i] = 1;
        }
      } else {
        diffX = laserCloud->points[i + 1].x * depth1 / depth2 - laserCloud->points[i].x;
        diffY = laserCloud->points[i + 1].y * depth1 / depth2 - laserCloud->points[i].y;
        diffZ = laserCloud->points[i + 1].z * depth1 / depth2 - laserCloud->points[i].z;

        if (sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ) / depth1 < 0.1) {
          cloudNeighborPicked[i + 1] = 1;
          cloudNeighborPicked[i + 2] = 1;
          cloudNeighborPicked[i + 3] = 1;
          cloudNeighborPicked[i + 4] = 1;
          cloudNeighborPicked[i + 5] = 1;
          cloudNeighborPicked[i + 6] = 1;
        }
      }
    }


    float diffX2 = laserCloud->points[i].x - laserCloud->points[i - 1].x;
    float diffY2 = laserCloud->points[i].y - laserCloud->points[i - 1].y;
    float diffZ2 = laserCloud->points[i].z - laserCloud->points[i - 1].z;
    float diff2 = diffX2 * diffX2 + diffY2 * diffY2 + diffZ2 * diffZ2;//与前一个点的距离平方和

    float dis = laserCloud->points[i].x * laserCloud->points[i].x//点深度的平方和
              + laserCloud->points[i].y * laserCloud->points[i].y
              + laserCloud->points[i].z * laserCloud->points[i].z;
    //与前后点的平方和都大于深度平方和的万分之二，这些点视为离群点，包括陡斜面上的点，强烈凸凹点和空旷区域中的某些点，置为筛选过，弃用
    if (diff > (0.25 * 0.25) / (20 * 20) * dis && diff2 > (0.25 * 0.25) / (20 * 20) * dis) {
      cloudNeighborPicked[i] = 1;
    }
  }

  pcl::PointCloud<pcl::PointXYZHSV>::Ptr cornerPointsSharp(new pcl::PointCloud<pcl::PointXYZHSV>());
  pcl::PointCloud<pcl::PointXYZHSV>::Ptr cornerPointsLessSharp(new pcl::PointCloud<pcl::PointXYZHSV>());
  pcl::PointCloud<pcl::PointXYZHSV>::Ptr surfPointsFlat(new pcl::PointCloud<pcl::PointXYZHSV>());
  pcl::PointCloud<pcl::PointXYZHSV>::Ptr surfPointsLessFlat(new pcl::PointCloud<pcl::PointXYZHSV>());

  int startPoints[4] = {5, 6 + int((cloudSize - 10) / 4.0), 
                        6 + int((cloudSize - 10) / 2.0), 6 + int(3 * (cloudSize - 10) / 4.0)};
  int endPoints[4] = {5 + int((cloudSize - 10) / 4.0), 5 + int((cloudSize - 10) / 2.0), 
                      5 + int(3 * (cloudSize - 10) / 4.0), cloudSize - 6};

    //将每条线上的点分入相应的类别：边沿点和平面点
  for (int i = 0; i < 4; i++) {//将每个scan的曲率点分成4等份处理,确保周围都有点被选作特征点
    int sp = startPoints[i];//4等份起点
    int ep = endPoints[i];//4等份终点

    for (int j = sp + 1; j <= ep; j++) {//按曲率从小到大冒泡排序
      for (int k = j; k >= sp + 1; k--) {
        if (laserCloud->points[cloudSortInd[k]].s < laserCloud->points[cloudSortInd[k - 1]].s) {//如果后面曲率点大于前面，则交换
          int temp = cloudSortInd[k - 1];
          cloudSortInd[k - 1] = cloudSortInd[k];
          cloudSortInd[k] = temp;
        }
      }
    }

      //挑选每个分段的曲率很大和比较大的点
    int largestPickedNum = 0;
    for (int j = ep; j >= sp; j--) {
      if (cloudNeighborPicked[cloudSortInd[j]] == 0 &&
          laserCloud->points[cloudSortInd[j]].s > 0.1 &&//如果曲率大的点，曲率的确比较大，并且未被筛选过滤掉
          (fabs(laserCloud->points[cloudSortInd[j]].x) > 0.3 || 
          fabs(laserCloud->points[cloudSortInd[j]].y) > 0.3 || 
          fabs(laserCloud->points[cloudSortInd[j]].z) > 0.3) && 
          fabs(laserCloud->points[cloudSortInd[j]].x) < 30 && //雷达有效距离
          fabs(laserCloud->points[cloudSortInd[j]].y) < 30 && 
          fabs(laserCloud->points[cloudSortInd[j]].z) < 30) {
        
        largestPickedNum++;
        if (largestPickedNum <= 2) {//挑选曲率最大的前2个点放入sharp点集合
          laserCloud->points[cloudSortInd[j]].v = 2;//2代表点曲率很大
          cornerPointsSharp->push_back(laserCloud->points[cloudSortInd[j]]);
        } else if (largestPickedNum <= 20) {//挑选曲率最大的前20个点放入less sharp点集合
          laserCloud->points[cloudSortInd[j]].v = 1;//1代表点曲率比较尖锐
          cornerPointsLessSharp->push_back(laserCloud->points[cloudSortInd[j]]);
        } else {
          break;
        }

        cloudNeighborPicked[cloudSortInd[j]] = 1;//筛选标志置位
        for (int k = 1; k <= 5; k++) {//将曲率比较大的点的前后各5个连续距离比较近的点筛选出去，防止特征点聚集，使得特征点在每个方向上尽量分布均匀
          float diffX = laserCloud->points[cloudSortInd[j] + k].x 
                      - laserCloud->points[cloudSortInd[j] + k - 1].x;
          float diffY = laserCloud->points[cloudSortInd[j] + k].y 
                      - laserCloud->points[cloudSortInd[j] + k - 1].y;
          float diffZ = laserCloud->points[cloudSortInd[j] + k].z 
                      - laserCloud->points[cloudSortInd[j] + k - 1].z;
          if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
            break;
          }

          cloudNeighborPicked[cloudSortInd[j] + k] = 1;
        }
        for (int k = -1; k >= -5; k--) {
          float diffX = laserCloud->points[cloudSortInd[j] + k].x 
                      - laserCloud->points[cloudSortInd[j] + k + 1].x;
          float diffY = laserCloud->points[cloudSortInd[j] + k].y 
                      - laserCloud->points[cloudSortInd[j] + k + 1].y;
          float diffZ = laserCloud->points[cloudSortInd[j] + k].z 
                      - laserCloud->points[cloudSortInd[j] + k + 1].z;
          if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
            break;
          }

          cloudNeighborPicked[cloudSortInd[j] + k] = 1;
        }
      }
    }

      //挑选每个分段的曲率很小比较小的点
    int smallestPickedNum = 0;
    for (int j = sp; j <= ep; j++) {
      if (cloudNeighborPicked[cloudSortInd[j]] == 0 &&
          laserCloud->points[cloudSortInd[j]].s < 0.1 &&//如果曲率的确比较小，并且未被筛选出
          (fabs(laserCloud->points[cloudSortInd[j]].x) > 0.3 || 
          fabs(laserCloud->points[cloudSortInd[j]].y) > 0.3 || 
          fabs(laserCloud->points[cloudSortInd[j]].z) > 0.3) && 
          fabs(laserCloud->points[cloudSortInd[j]].x) < 30 && 
          fabs(laserCloud->points[cloudSortInd[j]].y) < 30 && 
          fabs(laserCloud->points[cloudSortInd[j]].z) < 30) {

        laserCloud->points[cloudSortInd[j]].v = -1;//-1代表曲率很小的点
        surfPointsFlat->push_back(laserCloud->points[cloudSortInd[j]]);

        smallestPickedNum++;
        if (smallestPickedNum >= 4) {//只选最小的四个，剩下的Label==0,就都是曲率比较小的
          break;
        }

        cloudNeighborPicked[cloudSortInd[j]] = 1;
        for (int k = 1; k <= 5; k++) {//同样防止特征点聚集
          float diffX = laserCloud->points[cloudSortInd[j] + k].x 
                      - laserCloud->points[cloudSortInd[j] + k - 1].x;
          float diffY = laserCloud->points[cloudSortInd[j] + k].y 
                      - laserCloud->points[cloudSortInd[j] + k - 1].y;
          float diffZ = laserCloud->points[cloudSortInd[j] + k].z 
                      - laserCloud->points[cloudSortInd[j] + k - 1].z;
          if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
            break;
          }

          cloudNeighborPicked[cloudSortInd[j] + k] = 1;
        }
        for (int k = -1; k >= -5; k--) {
          float diffX = laserCloud->points[cloudSortInd[j] + k].x 
                      - laserCloud->points[cloudSortInd[j] + k + 1].x;
          float diffY = laserCloud->points[cloudSortInd[j] + k].y 
                      - laserCloud->points[cloudSortInd[j] + k + 1].y;
          float diffZ = laserCloud->points[cloudSortInd[j] + k].z 
                      - laserCloud->points[cloudSortInd[j] + k + 1].z;
          if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
            break;
          }

          cloudNeighborPicked[cloudSortInd[j] + k] = 1;
        }
      }
    }
  }

  for (int i = 0; i < cloudSize; i++) {//将剩余的点（包括之前被排除的点）全部归入平面点中less flat类别中
    if (laserCloud->points[i].v == 0) {
      surfPointsLessFlat->push_back(laserCloud->points[i]);
    }
  }

  pcl::PointCloud<pcl::PointXYZHSV>::Ptr surfPointsLessFlatDS(new pcl::PointCloud<pcl::PointXYZHSV>());
  pcl::VoxelGrid<pcl::PointXYZHSV> downSizeFilter;//由于less flat点最多，对每个分段less flat的点进行体素栅格滤波
  downSizeFilter.setInputCloud(surfPointsLessFlat);
  downSizeFilter.setLeafSize(0.1, 0.1, 0.1);
  downSizeFilter.filter(*surfPointsLessFlatDS);

    //点汇总
  *laserCloudExtreCur += *cornerPointsSharp;
  *laserCloudExtreCur += *surfPointsFlat;
  *laserCloudLessExtreCur += *cornerPointsLessSharp;
  *laserCloudLessExtreCur += *surfPointsLessFlatDS;

  laserCloudIn->clear();
  laserCloud->clear();
  cornerPointsSharp->clear();
  cornerPointsLessSharp->clear();
  surfPointsFlat->clear();
  surfPointsLessFlat->clear();
  surfPointsLessFlatDS->clear();

  if (skipFrameCount >= skipFrameNum) {
    skipFrameCount = 0;
//publich IMU消息,由于循环到了最后，因此是Cur都是代表最后一个点，即最后一个点的欧拉角，畸变位移及一个点云周期增加的速度
    pcl::PointCloud<pcl::PointXYZHSV>::Ptr imuTrans(new pcl::PointCloud<pcl::PointXYZHSV>(4, 1));
    imuTrans->points[0].x = imuPitchStart;
    imuTrans->points[0].y = imuYawStart;
    imuTrans->points[0].z = imuRollStart;
    imuTrans->points[0].v = 10;

    imuTrans->points[1].x = imuPitchCur;
    imuTrans->points[1].y = imuYawCur;
    imuTrans->points[1].z = imuRollCur;
    imuTrans->points[1].v = 11;

    imuTrans->points[2].x = imuShiftFromStartXCur;
    imuTrans->points[2].y = imuShiftFromStartYCur;
    imuTrans->points[2].z = imuShiftFromStartZCur;
    imuTrans->points[2].v = 12;

    imuTrans->points[3].x = imuVeloFromStartXCur;
    imuTrans->points[3].y = imuVeloFromStartYCur;
    imuTrans->points[3].z = imuVeloFromStartZCur;
    imuTrans->points[3].v = 13;

    sensor_msgs::PointCloud2 laserCloudExtreCur2;
    pcl::toROSMsg(*laserCloudExtreCur + *imuTrans, laserCloudExtreCur2);
    laserCloudExtreCur2.header.stamp = ros::Time().fromSec(timeScanCur);
    laserCloudExtreCur2.header.frame_id = "/camera";
    pubLaserCloudExtreCurPointer->publish(laserCloudExtreCur2);//发布特征点
    imuTrans->clear();
    //std::cout<<"the size of laserCloudLast2 = "<<laserCloudExtreCur2.data.size()<<std::endl;
    pubLaserCloudLastPointer->publish(laserCloudLast2);//发布所有点

    //ROS_INFO ("%d %d", laserCloudLast2.width, laserCloudExtreCur2.width);
  }
  skipFrameCount++;

}

void imuHandler(const sensor_msgs::Imu::ConstPtr& imuIn)
{
  double roll, pitch, yaw;
  tf::Quaternion orientation;
  tf::quaternionMsgToTF(imuIn->orientation, orientation);
  tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

  int imuPointerBack = imuPointerLast;
  imuPointerLast = (imuPointerLast + 1) % imuQueLength;
  imuTime[imuPointerLast] = imuIn->header.stamp.toSec();
  double timeDiff = imuTime[imuPointerLast] - imuTime[imuPointerBack];

  if (timeDiff < 0.1) {

    //imuAccuRoll += timeDiff * imuIn->angular_velocity.x;
    //imuAccuPitch += timeDiff * imuIn->angular_velocity.y;
    imuAccuYaw += timeDiff * imuIn->angular_velocity.z;

    imuRoll[imuPointerLast] = roll;
    imuPitch[imuPointerLast] = -pitch;
    //imuYaw[imuPointerLast] = -yaw;
    //imuRoll[imuPointerLast] = imuAccuRoll;
    //imuPitch[imuPointerLast] = -imuAccuPitch;
    imuYaw[imuPointerLast] = -imuAccuYaw;

    //imuAccX[imuPointerLast] = -imuIn->linear_acceleration.y;
    //imuAccY[imuPointerLast] = -imuIn->linear_acceleration.z - 9.81;
    //imuAccZ[imuPointerLast] = imuIn->linear_acceleration.x;

    AccumulateIMUShift();
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "scanRegistration");
  ros::NodeHandle nh;

  if(!nh.getParam("scanDuration",scanDuration))
    scanDuration = 0.9;
  if(!nh.getParam("skipFrameNum",skipFrameNum))
    skipFrameNum = 2;

  std::cout<<"scanDuration = "<<scanDuration<<std::endl;
  std::cout<<"skipFrameNum = "<<skipFrameNum<<std::endl;
  std::cout<<"imuQueLength = "<<imuQueLength<<std::endl;

  ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2> 
                                  ("/velodyne_points", 2, laserCloudHandler);

  ros::Subscriber subImu = nh.subscribe<sensor_msgs::Imu> 
                           ("/microstrain/imu", 5, imuHandler);

  ros::Publisher pubLaserCloudExtreCur = nh.advertise<sensor_msgs::PointCloud2> 
                                         ("/laser_cloud_extre_cur", 2);

  ros::Publisher pubLaserCloudLast = nh.advertise<sensor_msgs::PointCloud2> 
                                     ("/laser_cloud_last", 2);

  pubLaserCloudExtreCurPointer = &pubLaserCloudExtreCur;
  pubLaserCloudLastPointer = &pubLaserCloudLast;

  ros::spin();

  return 0;
}
