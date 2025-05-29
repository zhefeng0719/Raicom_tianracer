#include "mycontrol.h"
/*
第一套：
oil 4.7 fsee 1.05 fgoal 1.70 kd 6.5 减速系数0.68
*/
using namespace std;
mycontrol::mycontrol() {
    ros::NodeHandle pravite_nh("~");// 创建私有的节点句柄
    //参数设置
    pravite_nh.param("base_shape_L", base_shape_L, 0.265); // 机器人轴距
    pravite_nh.param("oil", oil, 4.3);// 目标速度
    pravite_nh.param("fsee", fsee, 1.05); // 前视距离
    pravite_nh.param("Freq", Freq, 100);   // 控制频率
    pravite_nh.param("fgoal", fgoal, 1.75); // 导航目标点前瞻距离
    pravite_nh.param("debug_mode", debug_mode, false); // debug mode
    //话题通信
    odom_sub_ = nh_.subscribe("/tianracer/odom", 1, &mycontrol::odomCallback, this);//订阅里程计
    path_sub_ = nh_.subscribe("/tianracer/move_base/TebLocalPlannerROS/global_plan", 1, &mycontrol::pathCallback, this);//订阅TEB规划器的局部目标
    goal_sub_ = nh_.subscribe("/tianracer/move_base_simple/goal", 1, &mycontrol::goalCallback, this);//订阅导航目标
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/tianracer/path_marker", 50);
    ackermann_pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("/tianracer/ackermann_cmd_stamped", 1);    //发布控制话题
    timer1 = nh_.createTimer(ros::Duration((1.0) / Freq), &mycontrol::controlLoopCallback, this); //定时中断
    //变量初始化
    SeeClose = false;
    goal_received = false;
    goal_reached = false;
    speed = 0.0;
    turn = 0.0;
    ackermann_cmd_ = ackermann_msgs::AckermannDriveStamped();//阿克曼小车控制赋值  
}
mycontrol::~mycontrol(){};//析构函数
double mycontrol::Dis2(geometry_msgs::PoseStamped& wayPt,const geometry_msgs::Point& car_pos){
    double dx = wayPt.pose.position.x - car_pos.x;
    double dy = wayPt.pose.position.y - car_pos.y;
    return dx * dx + dy * dy;
}
void mycontrol::odomCallback(const nav_msgs::Odometry::ConstPtr &odomMsg) {
    this->odom_ = *odomMsg;
    if(this->goal_received)
    {
        double car2goal_x = goal_pos_.x - odomMsg->pose.pose.position.x;
        double car2goal_y = goal_pos_.y - odomMsg->pose.pose.position.y;
        double dist2goal = sqrt(car2goal_x*car2goal_x + car2goal_y*car2goal_y);
        /// 和move_base一起使用时这段注释解除
       if(dist2goal < this->fgoal)
       {
           this->goal_reached = true;
           this->goal_received = false;
           ROS_INFO("Goal Reached !");
       }
    }
}
void mycontrol::pathCallback(const nav_msgs::Path::ConstPtr &pathMsg) {
    this->map_path_ = *pathMsg;
}
void mycontrol::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& goalMsg)
{
    this->goal_pos_ = goalMsg->pose.position;
    try
    {
        ROS_INFO("get goal");
        geometry_msgs::PoseStamped odom_goal;
        tf_listener_.transformPose("/tianracer/odom", ros::Time(0) , *goalMsg, "map" ,odom_goal);
        odom_goal_pos_ = odom_goal.pose.position;
        goal_received = true;
        goal_reached = false;
        /*Draw Goal on RVIZ*/
        goal_circle_.pose = odom_goal.pose;
        marker_pub_.publish(goal_circle_);
    }
    catch(tf::TransformException &ex)
    {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }
}
double mycontrol::ERR_GET(const geometry_msgs::Pose& carPose)
{
    geometry_msgs::Point odom_car2WayPtVec = get_odom_car2WayPtVec(carPose);
    return atan2(odom_car2WayPtVec.y,odom_car2WayPtVec.x);
}
bool mycontrol::ifSeeClose(const geometry_msgs::Point& wayPt, const geometry_msgs::Point& car_pos)
{
    double dx = wayPt.x - car_pos.x;
    double dy = wayPt.y - car_pos.y;
    double dist = sqrt(dx*dx + dy*dy);   
    return (dist >= fsee);  
}

int mycontrol::minIndex(const geometry_msgs::Point& carPose){
    int index_min = 0;
    int d_min = INT16_MAX;
    for(int i =0; i< map_path_.poses.size(); i++)
    {
        geometry_msgs::PoseStamped map_path_pose = map_path_.poses[i];
        double d_temp = Dis2(map_path_pose,carPose);
        if (d_temp < d_min) {
            d_min = d_temp;
            index_min = i;
        }
    }
    return index_min;
}
bool mycontrol::ifPointForward(const geometry_msgs::Point& wayPt, const geometry_msgs::Pose& carPose)
{
    float car2wayPt_x = wayPt.x - carPose.position.x;
    float car2wayPt_y = wayPt.y - carPose.position.y;
    double car_theta =  tf::getYaw(carPose.orientation);
    float car_car2wayPt_x = cos(car_theta)*car2wayPt_x + sin(car_theta)*car2wayPt_y;
    if(car_car2wayPt_x >0)
        return true;
    else
        return false;
}
geometry_msgs::Point mycontrol::get_odom_car2WayPtVec(const geometry_msgs::Pose& carPose)
{
    geometry_msgs::Point carPose_pos = carPose.position;
    double carPose_yaw = tf::getYaw(carPose.orientation);
    geometry_msgs::Point forwardPt;
    geometry_msgs::Point odom_car2WayPtVec;
    SeeClose = false;
    if(!goal_reached){
        //寻找离机器人最近的路径点
        int index = minIndex(carPose_pos);
        for(int i =index; i< map_path_.poses.size(); i++)
        {
            geometry_msgs::PoseStamped map_path_pose = map_path_.poses[i];
            geometry_msgs::PoseStamped odom_path_pose;
            try
            {
                tf_listener_.transformPose("/tianracer/odom", ros::Time(0) , map_path_pose, "map" ,odom_path_pose);
                geometry_msgs::Point odom_path_wayPt = odom_path_pose.pose.position;
                if( ifPointForward(odom_path_wayPt,carPose) )
                {
                    bool _isWayPtAwayFromLfwDist = ifSeeClose(odom_path_wayPt,carPose_pos);
                    if(_isWayPtAwayFromLfwDist)
                    {
                        forwardPt = odom_path_wayPt;
                        SeeClose = true;
                        break;
                    }
                }
            }
            catch(tf::TransformException &ex)
            {
                ROS_ERROR("%s",ex.what());
                ros::Duration(1.0).sleep();
            }
        }
    }
    else if(goal_reached)
    {
        forwardPt = odom_goal_pos_;
        SeeClose = false;
    }
    points_.points.clear();
    line_strip_.points.clear();
    if(SeeClose && !goal_reached)
    {
        points_.points.push_back(carPose_pos);
        points_.points.push_back(forwardPt);
        line_strip_.points.push_back(carPose_pos);
        line_strip_.points.push_back(forwardPt);
    }
    marker_pub_.publish(points_);
    marker_pub_.publish(line_strip_);
    //坐标变换：将小车坐标系转到地图坐标系
    odom_car2WayPtVec.x = cos(carPose_yaw)*(forwardPt.x - carPose_pos.x) + sin(carPose_yaw)*(forwardPt.y - carPose_pos.y);
    odom_car2WayPtVec.y = -sin(carPose_yaw)*(forwardPt.x - carPose_pos.x) + cos(carPose_yaw)*(forwardPt.y - carPose_pos.y);
    return odom_car2WayPtVec;
}
//pid
void mycontrol::controlLoopCallback(const ros::TimerEvent&)
{
    geometry_msgs::Pose carPose = this->odom_.pose.pose;
    geometry_msgs::Twist carVel = this->odom_.twist.twist;
    float E=0,EC=0;
    if(this->goal_received)
    {
        float Kp,Kd;
        err = ERR_GET(carPose);
        float angle=0;
        if(SeeClose)
        {
            if(!this->goal_reached)
            {
                E=err;
                EC=err-last_err;
                Kp=KP_Fuzzy(E,EC); 
                Kd=Kd_Fuzzy(EC);
                angle=Kp*E+Kd*EC;
                last_err=err;
                this->turn = angle;
                
// 计算转角对应的速度系数-----------------------
double speed_factor = exp(-0.645*fabs(turn));
speed = oil * speed_factor;    
//----------------------------
}
        }
    }
    if(goal_reached)
    {
        speed = 0.0;
        turn = 0.0; 
    }
    this->ackermann_cmd_.drive.steering_angle = this->turn;
    this->ackermann_cmd_.drive.speed =  this->speed;
    this->ackermann_pub_.publish(this->ackermann_cmd_);
}

float mycontrol::KP_Fuzzy(float E,float EC)
{
 
    int rule_p[7][7]=
    {
        { 6 , 5 , 4 , 4 , 3 , 0 , 0},//-36
        { 6 , 4 , 3 , 3 , 2 , 0 , 0},//-24
        { 4 , 3 , 2 , 1 , 0 , 1 , 2},//-12
        { 2 , 1 , 1 , 0 , 1 , 1 , 2},//0
        { 2 , 1 , 0 , 1 , 2 , 3 , 4},//12
        { 0 , 0 , 2 , 3 , 3 , 4 , 6},//24
        { 0 , 1 , 3 , 4 , 4 , 5 , 6},//36
    };//模糊规则表 P
 
    int i2;
    /*输入量P语言值特征点*/
    float EFF[7];
    for(int i=0;i<=6;i++)
    {
        EFF[i]=eff[i];
        // printf("eff[%d]=%f\n",i,EFF[i]);   
    }
    /*输入量D语言值特征点*/
    float DFF[7];
    
    for(int i=0;i<=6;i++)
        DFF[i]=eff[i]/2;
    /*输出量U语言值特征点(根据赛道类型选择不同的输出值)*/
    float UFF[7];
 
    for(i2=0;i2<=6;i2++)
    {
        UFF[i2]=kp_max/6*i2;
        // printf("kpm=%f,uff[%d]=%f\n",kp_m,i2,UFF[i2]);
    }
 
 
    float U=0;  /*偏差,偏差微分以及输出值的精确量*/
    float PF[2]={0},DF[2]={0},UF[4]={0};
    /*偏差,偏差微分以及输出值的隶属度*/
    int Pn=0,Dn=0,Un[4]={0};
    float t1=0,t2=0,t3=0,t4=0,temp1=0,temp2=0;
    /*隶属度的确定*/
    /*根据PD的指定语言值获得有效隶属度*/
    if(E>EFF[0] && E<EFF[6])
    {
        if(E<=EFF[1])
        {
            Pn=-2;
            PF[0]=(EFF[1]-E)/(EFF[1]-EFF[0]);
        }
        else if(E<=EFF[2])
        {
            Pn=-1;
            PF[0]=(EFF[2]-E)/(EFF[2]-EFF[1]);
        }
        else if(E<=EFF[3])
        {
            Pn=0;
            PF[0]=(EFF[3]-E)/(EFF[3]-EFF[2]);
        }
        else if(E<=EFF[4])
        {
            Pn=1;
            PF[0]=(EFF[4]-E)/(EFF[4]-EFF[3]);
        }
        else if(E<=EFF[5])
        {
            Pn=2;
            PF[0]=(EFF[5]-E)/(EFF[5]-EFF[4]);
        }
        else if(E<=EFF[6])
        {
            Pn=3;
            PF[0]=(EFF[6]-E)/(EFF[6]-EFF[5]);
        }
    }
 
    else if(E<=EFF[0])
    {
        Pn=-2;/*  ??? */
        PF[0]=1;
    }
    else if(E>=EFF[6])
    {
        Pn=3;
        PF[0]=0;
    }
 
    PF[1]=1-PF[0];
 
 
    //判断D的隶属度
    if(EC>DFF[0]&&EC<DFF[6])
    {
        if(EC<=DFF[1])
        {
            Dn=-2;
            DF[0]=(DFF[1]-EC)/(DFF[1]-DFF[0]);
        }
        else if(EC<=DFF[2])
        {
            Dn=-1;
            DF[0]=(DFF[2]-EC)/(DFF[2]-DFF[1]);
        }
        else if(EC<=DFF[3])
        {
            Dn=0;
            DF[0]=(DFF[3]-EC)/(DFF[3]-DFF[2]);
        }
        else if(EC<=DFF[4])
        {
            Dn=1;
            DF[0]=(DFF[4]-EC)/(DFF[4]-DFF[3]);
        }
        else if(EC<=DFF[5])
        {
            Dn=2;
            DF[0]=(DFF[5]-EC)/(DFF[5]-DFF[4]);
        }
        else if(EC<=DFF[6])
        {
            Dn=3;
            DF[0]=(DFF[6]-EC)/(DFF[6]-DFF[5]);
        }
    }
    //不在给定的区间内
    else if (EC<=DFF[0])
    {
        Dn=-2;
        DF[0]=1;
    }
    else if(EC>=DFF[6])
    {
        Dn=3;
        DF[0]=0;
    }
 
    DF[1]=1-DF[0];
 
    /*使用误差范围优化后的规则表rule[7][7]*/
    /*输出值使用13个隶属函数,中心值由UFF[7]指定*/
    /*一般都是四个规则有效*/
    Un[0]=rule_p[Pn+2][Dn+2];
    Un[1]=rule_p[Pn+3][Dn+2];
    Un[2]=rule_p[Pn+2][Dn+3];
    Un[3]=rule_p[Pn+3][Dn+3];
 
    if(PF[0]<=DF[0])    //求小
        UF[0]=PF[0];
    else
        UF[0]=DF[0];
    if(PF[1]<=DF[0])
        UF[1]=PF[1];
    else
        UF[1]=DF[0];
    if(PF[0]<=DF[1])
        UF[2]=PF[0];
    else
        UF[2]=DF[1];
    if(PF[1]<=DF[1])
        UF[3]=PF[1];
    else
        UF[3]=DF[1];
    /*同隶属函数输出语言值求大*/
    if(Un[0]==Un[1])
    {
        if(UF[0]>UF[1])
            UF[1]=0;
        else
            UF[0]=0;
    }
    if(Un[0]==Un[2])
    {
        if(UF[0]>UF[2])
            UF[2]=0;
        else
            UF[0]=0;
    }
    if(Un[0]==Un[3])
    {
        if(UF[0]>UF[3])
            UF[3]=0;
        else
            UF[0]=0;
    }
    if(Un[1]==Un[2])
    {
        if(UF[1]>UF[2])
            UF[2]=0;
        else
            UF[1]=0;
    }
    if(Un[1]==Un[3])
    {
        if(UF[1]>UF[3])
            UF[3]=0;
        else
            UF[1]=0;
    }
    if(Un[2]==Un[3])
    {
        if(UF[2]>UF[3])
            UF[3]=0;
        else
            UF[2]=0;
    }
    t1=UF[0]*UFF[Un[0]];
    t2=UF[1]*UFF[Un[1]];
    t3=UF[2]*UFF[Un[2]];
    t4=UF[3]*UFF[Un[3]];
    temp1=t1+t2+t3+t4;
    temp2=UF[0]+UF[1]+UF[2]+UF[3];//模糊量输出
    if(temp2!=0)
        U=temp1/temp2;
    else {
        U=0;
    }

    return U;
}
float mycontrol::Kd_Fuzzy(float EC)
{
    int rule_d[7] = { 6 , 5 , 3 , 2 , 3 , 5 , 6};//模糊规则表 D
    float out=0;
    int i=0;
    float degree_left = 0,degree_right = 0;
    int degree_left_index = 0,degree_right_index = 0;
    float DFF[7];
    for(int i=0;i<=6;i++)
        DFF[i]=eff[i]/2;
    float UFF[7];
 
    for(i=0;i<7;i++)
            UFF[i]=kd_max/6*i;
 
    if(EC<DFF[0])
    {
        degree_left = 1;
        degree_right = 0;
        degree_left_index = 0;
    }
    else if (EC>DFF[6]) {
        degree_left = 1;
        degree_right = 0;
        degree_left_index = 6;
    }
    else {
        for(i=0;i<6;i++)
        {
            if(EC>=DFF[i]&&EC<DFF[i+1])
            {
                degree_left = (float)(DFF[i+1] - EC)/(DFF[i+1] - DFF[i]);
                degree_right = 1 - degree_left;
                degree_left_index = i;
                degree_right_index = i+1;
                break;
            }
        }
    }
 
    out = UFF[rule_d[degree_left_index]]*degree_left+UFF[rule_d[degree_right_index]]*degree_right;
 
    return out;
}

int main(int argc, char **argv) {   
    ros::init(argc, argv, "mycontrol");// 创建ROS节点   
    mycontrol ppc;// 调用类   
    ros::AsyncSpinner spinner(3);// 多线程工作
    spinner.start();
    ros::waitForShutdown();
    return 0;
}
