#ifndef SRC_PURE_PURSUIT_H
#define SRC_PURE_PURSUIT_H

#include <iostream>
#include "ros/ros.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

class mycontrol {
public:
    // 构造函数与析构函数
    mycontrol();
    ~mycontrol();
    //模糊PID
    float KP_Fuzzy(float E,float EC);
    float Kd_Fuzzy(float EC);

    void odomCallback(const nav_msgs::Odometry::ConstPtr &odomMsg);//里程计回调
    void pathCallback(const nav_msgs::Path::ConstPtr &pathMsg);//TEB更新回调
    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& goalMsg);//导航目标点更新回调
    void controlLoopCallback(const ros::TimerEvent&);//定时中断回调

    double ERR_GET(const geometry_msgs::Pose& carPose);//获取角度ERR
 
    double Dis2(geometry_msgs::PoseStamped& wayPt,const geometry_msgs::Point& car_pos);//计算目标与当前小车位置的距离平方
    int minIndex(const geometry_msgs::Point& carPose);

    geometry_msgs::Point get_odom_car2WayPtVec(const geometry_msgs::Pose& carPose);

    bool ifSeeClose(const geometry_msgs::Point& wayPt, const geometry_msgs::Point& car_pos);//筛选路径上的小目标点，太靠近就换下一个目标点

    bool ifPointForward(const geometry_msgs::Point& wayPt, const geometry_msgs::Pose& carPose);//判断路径点是否在车头正方向

    float kp_max=1.85;//p最大值1.65
    float kd_max=1.55;//d最大值
    float eff[7]={-0.5,-0.3,-0.1,0,0.1,0.3,0.5};
    float err,last_err;//误差特征点


    ros::NodeHandle nh_;    // 创建公共的节点句柄
private:
    ros::Subscriber odom_sub_, path_sub_, goal_sub_;
    ros::Publisher ackermann_pub_, marker_pub_, plan_pub_;
    ros::Timer timer1, timer2;
    tf::TransformListener tf_listener_;
    tf::StampedTransform transform_;
    visualization_msgs::Marker points_, line_strip_, goal_circle_;
    geometry_msgs::Point odom_goal_pos_, goal_pos_;
    ackermann_msgs::AckermannDriveStamped ackermann_cmd_;
    nav_msgs::Odometry odom_;
    nav_msgs::Path map_path_, odom_path_;

    double base_shape_L;//轴距
    double oil;//油门
    double speed,turn,base_angle_;//最后计算输出时使用的速度，角度数据
    double  fsee;//前视距离
    double fgoal;//接近发布的导航目标点时，提前判断到达(连续导航)
    int Freq;//定时器平率
    bool SeeClose, goal_received, goal_reached, debug_mode;//一些状态量和标志
};

#endif //SRC_PURE_PURSUIT_H
