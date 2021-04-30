#include <signal.h>
#include <locale.h>
#include "ros/ros.h"
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf/transform_broadcaster.h>
#include <webots_ros/set_float.h>
#include <webots_ros/set_int.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Imu.h>
#include <math.h>
#define TIME_STEP 32
#define NMOTORS 4
#define NWHEELS 4
#define MAX_SPEED 6
#define PI 3.14159265358
#define TWO_PI (PI*2)
#define HALF_PI (PI/2)
#define QUARTER_PI (PI/4)

ros::NodeHandle *n;

static int controllerCount;
static std::vector<std::string> controllerList;

ros::ServiceClient timeStepClient;
webots_ros::set_int timeStepSrv;

static const char *motorNames[NMOTORS] = {"Motor1", "Motor2", "Motor3", "Motor4"};
static const char *wheelNames[NWHEELS] = {"Wheel1","Wheel2","Wheel3","Wheel4"};
double inertialUnitValues[4] = {0, 0, 0, 0};
typedef struct Motor_info{
   double Position;
   double Angle;
   int8_t Circle_num;
} Motor_info;


Motor_info Motor[NMOTORS];
double Motor_Tar[NMOTORS]={0};
double Wheel_Tar[NWHEELS]={0};
// 更新速度
void updateSpeed() {
  for (int i = 0; i < NMOTORS; ++i) {
    ros::ServiceClient set_position_client;
    webots_ros::set_float set_position_srv;
    set_position_client = n->serviceClient<webots_ros::set_float>(std::string("my_robot/") + std::string(motorNames[i]) +
                                                                  std::string("/set_position"));
    set_position_srv.request.value = Motor_Tar[i];
    Motor[i].Angle=Motor_Tar[i];
    Motor[i].Circle_num=(int8_t)(Motor_Tar[i])/TWO_PI;
    Motor[i].Position=Motor_Tar[i]- Motor[i].Circle_num*TWO_PI;
    set_position_client.call(set_position_srv);
  }for (int i = 0; i < NWHEELS; ++i) {
    ros::ServiceClient set_velocity_client;
    webots_ros::set_float set_velocity_srv;
    set_velocity_client = n->serviceClient<webots_ros::set_float>(std::string("my_robot/") + std::string(wheelNames[i]) +
                                                                  std::string("/set_velocity"));
    set_velocity_srv.request.value = Wheel_Tar[i];
    set_velocity_client.call(set_velocity_srv);
  }
}

void Speed_analysis(float speed_x,float speed_y)
{
  static double Xbox_Angle=0;
  static double Xbox_Speed=0;
  double dA_Now=0;
  double dA_Tar=0;
  double Error=0;
  double Angle_Temp=0;

  Xbox_Angle=atan2(-speed_x,speed_y);//xbox手柄的x轴向左为正, 所以加符号
  Xbox_Speed=sqrt((speed_x*speed_x)+(speed_y*speed_y))*MAX_SPEED;
  if(Xbox_Angle>0) dA_Tar=Xbox_Angle;
  else dA_Tar=Xbox_Angle+TWO_PI;

  for(int i=0;i<NMOTORS;i++)
  {
    Angle_Temp=abs(Motor[i].Position);
    if(Motor[i].Position>=0){
      if(Motor[i].Position>HALF_PI&&Motor[i].Position<HALF_PI+PI) //当前角度在第三,第四象限
        if((dA_Tar>Motor[i].Position-HALF_PI)&&(dA_Tar<Motor[i].Angle+HALF_PI)){
          Motor_Tar[i]=dA_Tar-Motor[i].Position+Motor[i].Angle,Wheel_Tar[i]=Xbox_Speed;           //在左右九十度内
        }else if(dA_Tar<Motor[i].Position){
           Motor_Tar[i]=dA_Tar-Motor[i].Position+Motor[i].Angle+PI,Wheel_Tar[i]=-Xbox_Speed;      //目标值加180
        }else Motor_Tar[i]=dA_Tar-Motor[i].Position+Motor[i].Angle-PI,Wheel_Tar[i]=-Xbox_Speed;   //目标值减180
      else if(Motor[i].Position<HALF_PI){                         //当前角度在第一象限
        if((dA_Tar>Motor[i].Position+HALF_PI)&&(dA_Tar<Motor[i].Position+HALF_PI+PI)){
          Motor_Tar[i]=dA_Tar-Motor[i].Position+Motor[i].Angle-PI,Wheel_Tar[i]=-Xbox_Speed;       //目标值减180
        }else if(dA_Tar<Motor[i].Position+HALF_PI){
          Motor_Tar[i]=dA_Tar-Motor[i].Position+Motor[i].Angle,Wheel_Tar[i]=Xbox_Speed;           //在左右九十度内, 不需要越界
        }else Motor_Tar[i]=dA_Tar-TWO_PI-Motor[i].Position+Motor[i].Angle,Wheel_Tar[i]=Xbox_Speed;//在左右九十度内, 需要越界
      }else{                                                      //当前角度在第二象限
        if((dA_Tar<Motor[i].Position-HALF_PI)&&(dA_Tar>Motor[i].Position-HALF_PI-PI)){
          Motor_Tar[i]=dA_Tar-Motor[i].Position+Motor[i].Angle+PI,Wheel_Tar[i]=-Xbox_Speed;       //目标值减180
        }else if(dA_Tar>Motor[i].Position-HALF_PI){
          Motor_Tar[i]=dA_Tar-Motor[i].Position+Motor[i].Angle,Wheel_Tar[i]=Xbox_Speed;           //在左右九十度内, 不需要越界
        }else Motor_Tar[i]=TWO_PI+dA_Tar-Motor[i].Position+Motor[i].Angle,Wheel_Tar[i]=Xbox_Speed;//在左右九十度内, 需要越界
      }
    }else{
      Motor[i].Position+=TWO_PI;
      if(Motor[i].Position>HALF_PI&&Motor[i].Position<HALF_PI+PI) //当前角度在第三,第四象限
        if((dA_Tar>Motor[i].Position-HALF_PI)&&(dA_Tar<Motor[i].Angle+HALF_PI)){
          Motor_Tar[i]=dA_Tar-Motor[i].Position+Motor[i].Angle,Wheel_Tar[i]=Xbox_Speed;           //在左右九十度内
        }else if(dA_Tar<Motor[i].Position){
           Motor_Tar[i]=dA_Tar-Motor[i].Position+Motor[i].Angle+PI,Wheel_Tar[i]=-Xbox_Speed;      //目标值加180
        }else Motor_Tar[i]=dA_Tar-Motor[i].Position+Motor[i].Angle-PI,Wheel_Tar[i]=-Xbox_Speed;   //目标值减180
      else if(Motor[i].Position<HALF_PI){                         //当前角度在第一象限
        if((dA_Tar>Motor[i].Position+HALF_PI)&&(dA_Tar<Motor[i].Position+HALF_PI+PI)){
          Motor_Tar[i]=dA_Tar-Motor[i].Position+Motor[i].Angle-PI,Wheel_Tar[i]=-Xbox_Speed;       //目标值减180
        }else if(dA_Tar<Motor[i].Position+HALF_PI){
          Motor_Tar[i]=dA_Tar-Motor[i].Position+Motor[i].Angle,Wheel_Tar[i]=Xbox_Speed;           //在左右九十度内, 不需要越界
        }else Motor_Tar[i]=dA_Tar-TWO_PI-Motor[i].Position+Motor[i].Angle,Wheel_Tar[i]=Xbox_Speed;//在左右九十度内, 需要越界
      }else{                                                      //当前角度在第二象限
        if((dA_Tar<Motor[i].Position-HALF_PI)&&(dA_Tar>Motor[i].Position-HALF_PI-PI)){
          Motor_Tar[i]=dA_Tar-Motor[i].Position+Motor[i].Angle+PI,Wheel_Tar[i]=-Xbox_Speed;       //目标值减180
        }else if(dA_Tar>Motor[i].Position-HALF_PI){
          Motor_Tar[i]=dA_Tar-Motor[i].Position+Motor[i].Angle,Wheel_Tar[i]=Xbox_Speed;           //在左右九十度内, 不需要越界
        }else Motor_Tar[i]=TWO_PI+dA_Tar-Motor[i].Position+Motor[i].Angle,Wheel_Tar[i]=Xbox_Speed;//在左右九十度内, 需要越界
      }
    }
  }
  ROS_INFO("当前角度为:%.4f,%.2f,圈数为:%d,目标速度为:%.2f,轮子实际速度:%.2f",Motor[0].Position,Motor[0].Position*180/PI,Motor[0].Circle_num,Xbox_Speed,Wheel_Tar[0]);
}


/*    分类讨论算法思路, 待定废弃
 *    先判断同异侧, 然后判断相差角度类型, 然后判断当前值和目标值大小关系.
 *
 *   dA_Tar=abs(Xbox_Angle);
 *
    dA_Now=abs(Motor[i].Position);
    if(Xbox_Angle*Motor[i].Position>=0)
    {
      Error=Xbox_Angle-Motor[i].Position;
      if(abs(dA_Tar-dA_Now)<HALF_PI)
        if(Xbox_Angle<Motor[i].Position) Motor_Tar[i]=Plus_Tar(Error,Motor[i].Angle);
        else Motor_Tar[i]=Minus_Tar(Error,Motor[i].Angle);
      else
        if(Xbox_Angle<Motor[i].Position) Motor_Tar[i]=Minus_Tar(Error,Motor[i].Angle);
        else Motor_Tar[i]=Plus_Tar(Error,Motor[i].Angle);
    }
    if(Xbox_Angle*Motor[i].Position<0)
    {
      Error=TWO_PI-dA_Tar-dA_Now;
      if(dA_Tar+dA_Now<HALF_PI||dA_Tar+dA_Now>HALF_PI+QUARTER_PI)
        if(Xbox_Angle<Motor[i].Position) Motor_Tar[i]=Minus_Tar(Error,Motor[i].Angle);
        else Motor_Tar[i]=Plus_Tar(Error,Motor[i].Angle);
      else
        if(Xbox_Angle<Motor[i].Position) Motor_Tar[i]=Plus_Tar(Error,Motor[i].Angle);
        else Motor_Tar[i]=Minus_Tar(Error,Motor[i].Angle);
    }
*/

void JoyDataCallback(const sensor_msgs::Joy::ConstPtr &value)
{
  Speed_analysis(value->axes[0],value->axes[1]);
  updateSpeed();
}

// 获取可用控制器的名称
void controllerNameCallback(const std_msgs::String::ConstPtr &name) {
  controllerCount++;
  controllerList.push_back(name->data);
  ROS_INFO("Controller #%d: %s.", controllerCount, controllerList.back().c_str());
}

void inertialUnitCallback(const sensor_msgs::Imu::ConstPtr &values) {
  inertialUnitValues[0] = values->orientation.x;
  inertialUnitValues[1] = values->orientation.y;
  inertialUnitValues[2] = values->orientation.z;
  inertialUnitValues[3] = values->orientation.w;

//  ROS_INFO("Inertial unit values (quaternions) are x=%f y=%f z=%f w=%f (time: %d:%d).", inertialUnitValues[0],
 //          inertialUnitValues[1], inertialUnitValues[2], inertialUnitValues[2], values->header.stamp.sec,
 //          values->header.stamp.nsec);
}

//退出函数
void quit(int sig) {
  ROS_INFO("终止节点运行.");
  timeStepSrv.request.value = 0;
  timeStepClient.call(timeStepSrv);
  ros::shutdown();
  exit(0);
}

int main(int argc, char **argv) {
  setlocale(LC_CTYPE,"zh_CN.utf8");

  ros::init(argc, argv, "my_robot", ros::init_options::AnonymousName);
  n = new ros::NodeHandle;

  signal(SIGINT, quit);

  std::string controllerName;
  ros::Subscriber joySub;
  joySub = n->subscribe("/joy",1,JoyDataCallback);
/**********************************************   当作模板即可   ****************************************************/
  // 订阅主题model_name以获得可用控制器列表
  ros::Subscriber nameSub = n->subscribe("model_name", 100, controllerNameCallback);
  while (controllerCount == 0 || controllerCount < nameSub.getNumPublishers()) {
    ros::spinOnce();
    ros::spinOnce();
    ros::spinOnce();
  }
  ros::spinOnce();

// 设置基本仿真步长
  timeStepClient = n->serviceClient<webots_ros::set_int>("my_robot/robot/time_step");
  timeStepSrv.request.value = TIME_STEP;

  // 多控制器时可选
  if (controllerCount == 1){
  controllerName = controllerList[0];
  }
  else {
    int wantedController = 0;
    std::cout << "选择要使用的控制器的编号 :\n";
    std::cin >> wantedController;
    if (1 <= wantedController && wantedController <= controllerCount)
      controllerName = controllerList[wantedController - 1];
    else {
      ROS_ERROR("无效的控制器编号.");
      return 1;
    }
  }
  ROS_INFO("Using controller: '%s'", controllerName.c_str());
  nameSub.shutdown();

  //使能激光雷达
  ros::ServiceClient lidar_Client;
  webots_ros::set_int lidar_Srv;
  lidar_Client = n->serviceClient<webots_ros::set_int>("/my_robot/Sick_LMS_291/enable"); // 订阅lidar使能服务
  lidar_Srv.request.value = TIME_STEP;
  // 判断是否使能成功
  if (lidar_Client.call(lidar_Srv) && lidar_Srv.response.success) {
      ROS_INFO("gps enabled.");
  } else {
      if (!lidar_Srv.response.success)
      ROS_ERROR("Failed to enable lidar.");
      return 1;
  }
/**********************************************  电机模式设置   ****************************************************/
  for (int i = 0; i < NMOTORS; ++i) {
    ros::ServiceClient set_position_client;
    webots_ros::set_float set_position_srv;
    set_position_client = n->serviceClient<webots_ros::set_float>(std::string("my_robot/") + std::string(motorNames[i]) +
                                                                  std::string("/set_position"));
    //设置位置模式
    set_position_srv.request.value = 0;
    Motor[i].Circle_num=0;
    Motor[i].Position=0;
    if (set_position_client.call(set_position_srv) && set_position_srv.response.success)
      ROS_INFO("motor %s 工作模式为位置模式.", motorNames[i]);
    else
      ROS_ERROR("无法调用motor %s的set_position服务 .", motorNames[i]);
  }

  for (int i = 0; i < NWHEELS; ++i) {
    ros::ServiceClient set_position_client;
    webots_ros::set_float set_position_srv;
    set_position_client = n->serviceClient<webots_ros::set_float>(std::string("my_robot/") + std::string(wheelNames[i]) +
                                                                  std::string("/set_position"));
    //设置速度模式
    set_position_srv.request.value = INFINITY;
    if (set_position_client.call(set_position_srv) && set_position_srv.response.success)
      ROS_INFO("motor %s 工作模式为速度模式.", wheelNames[i]);
    else
      ROS_ERROR("无法调用motor %s的set_position服务 .", wheelNames[i]);

    // 创建速度client
    ros::ServiceClient set_velocity_client;
    set_velocity_client = n->serviceClient<webots_ros::set_float>(std::string("my_robot/") + std::string(wheelNames[i]) +
                                                                  std::string("/set_velocity"));
    // 创建服务
    webots_ros::set_float set_velocity_srv;
    set_velocity_srv.request.value = 0.0;

    if (set_velocity_client.call(set_velocity_srv) && set_velocity_srv.response.success == 1)
      ROS_INFO("Velocity set to 0.0 for motor %s.", wheelNames[i]);
    else
      ROS_ERROR("Failed to call service set_velocity on motor %s.", wheelNames[i]);
  }
  /**********************************************   传感器使能   ****************************************************/
  // enable inertial unit
    ros::ServiceClient set_inertial_unit_client;
    webots_ros::set_int inertial_unit_srv;
    ros::Subscriber sub_inertial_unit;
    set_inertial_unit_client = n->serviceClient<webots_ros::set_int>("my_robot/inertial_unit/enable");
    inertial_unit_srv.request.value = 32;
    if (set_inertial_unit_client.call(inertial_unit_srv) && inertial_unit_srv.response.success) {
      sub_inertial_unit = n->subscribe("my_robot/inertial_unit/roll_pitch_yaw", 1, inertialUnitCallback);
      while (sub_inertial_unit.getNumPublishers() == 0) {
      }
      ROS_INFO("Inertial unit enabled.");
    } else {
      if (!inertial_unit_srv.response.success)
        ROS_ERROR("Sampling period is not valid.");
      ROS_ERROR("Failed to enable inertial unit.");
      return 1;
    }

/******************************************** 主循环 ****************************************************** */
  while (ros::ok()) {
    updateSpeed();
    if (!timeStepClient.call(timeStepSrv) || !timeStepSrv.response.success) {
      ROS_ERROR("Failed to call service time_step for next step.");
      break;
    }
    ros::spinOnce();
  }
  timeStepSrv.request.value = 0;
  timeStepClient.call(timeStepSrv);

  ros::shutdown();
  return 0;
}
