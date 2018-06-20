#include <ros/ros.h>
#include <dji_sdk/dji_drone.h>
#include "gimbal_pid.h"
#include <robot_kcftracker/KCFTracker.h>
#include <dji_sdk/Gimbal.h>

using namespace std;

class GimbalControl
{
public:
  //node
  ros::NodeHandle nh_;
  ros::NodeHandle nh_param;
  DJIDrone* drone;
  ros::Subscriber sub_kcf;
  ros::Subscriber sub_gim;
  //kcf track
  int kcf_x, kcf_y, kcf_width, kcf_height, kcf_center_x, kcf_center_y;
  double image_width, image_height;
  float kcf_center_normal_x, kcf_center_normal_y;
  //time
  ros::Time ros_time;
  double time, last_time, dt;
  //pid
  double p_pitch, i_pitch, d_pitch, constrain_pitch,start_ditstance_pitch , p_yaw, i_yaw, d_yaw, constrain_yaw, start_ditstance_yaw;
  GimbalPID gp;
  float pid_output_pitch;
  float pid_output_yaw;
  //gimbal
  double body_min_pitch, body_max_pitch, body_min_yaw, body_max_yaw, gim_min_yaw, gim_max_yaw;
  bool gim_init_flag;
  double gim_init_yaw, gim_pitch, gim_yaw;
  
  GimbalControl(){}
  
  GimbalControl(bool pid_control_flag):
  nh_param("~")
  {
    drone = new DJIDrone(nh_);
    drone->request_sdk_permission_control();
    if(pid_control_flag)
    {
      //node
      sub_kcf = nh_.subscribe("/kcf_track", 10, &GimbalControl::kcfCallback, this);
      sub_gim = nh_.subscribe("/dji_sdk/gimbal", 10, &GimbalControl::gimbalCallback, this);
      //time
      ros_time = ros::Time::now();
      time = ros_time.toSec();
      last_time = ros_time.toSec();
      dt = 0.1;
      //pid
      if(!nh_param.getParam("p_pitch", p_pitch))p_pitch = 300.0;
      if(!nh_param.getParam("i_pitch", i_pitch))i_pitch = 0.0;
      if(!nh_param.getParam("d_pitch", d_pitch))d_pitch = 0.0;
      if(!nh_param.getParam("constrain_pitch", constrain_pitch))constrain_pitch = 300.0;
      if(!nh_param.getParam("start_ditstance_pitch", start_ditstance_pitch))start_ditstance_pitch = 1.0;
      if(!nh_param.getParam("p_yaw", p_yaw))p_yaw = 300.0;
      if(!nh_param.getParam("i_yaw", i_yaw))i_yaw = 0.0;
      if(!nh_param.getParam("d_yaw", d_yaw))d_yaw = 0.0;
      if(!nh_param.getParam("constrain_yaw", constrain_yaw))constrain_yaw = 300.0;
      if(!nh_param.getParam("start_ditstance_yaw", start_ditstance_yaw))start_ditstance_yaw = 1.0;
      //gimbal
      if(!nh_param.getParam("body_min_pitch", body_min_pitch))body_min_pitch = -80.0;
      if(!nh_param.getParam("body_max_pitch", body_max_pitch))body_max_pitch = -10.0;
      if(!nh_param.getParam("body_min_yaw", body_min_yaw))body_min_yaw = -45.0;
      if(!nh_param.getParam("body_max_yaw", body_max_yaw))body_max_yaw = 45.0;
      gim_min_yaw = -180.0;
      gim_max_yaw = 180.0;
      gim_init_flag = false;
    }
  }
  
  ~GimbalControl()
  {
    drone->release_sdk_permission_control();
  }
  
  void kcfCallback(const robot_kcftracker::KCFTracker::ConstPtr& msg)
  {
    //kcf track
    kcf_x = msg->kcf_x;
    kcf_y = msg->kcf_y;
    kcf_width = msg->kcf_width;
    kcf_height = msg->kcf_height;
    image_width = msg->image_width;
    image_height = msg->image_height;
    kcf_center_x = kcf_x + kcf_width / 2;
    kcf_center_y = kcf_y + kcf_height / 2;
    kcf_center_normal_x = kcf_center_x / image_width;
    kcf_center_normal_y = kcf_center_y / image_height;
    //time
    ros_time = ros::Time::now();
    time = ros_time.toSec();
    dt = time - last_time;
    
    //initializing GimbalPID
    if(gp.pid_init_end_flag == false)
    {
      gp.initCurrent(kcf_center_normal_y, kcf_center_normal_x);
      gp.initTarget(1.0/2.0, 1.0/2.0);
      gp.initPID(p_pitch, i_pitch, d_pitch, p_yaw, i_yaw, d_yaw);
      gp.initDt(dt);
      gp.initOutConstrain(-constrain_pitch, constrain_pitch, -constrain_yaw, constrain_yaw);
      gp.initStartDistance(start_ditstance_pitch, start_ditstance_yaw);
      gp.pid_init_end_flag = true;
    }
    else
    {
      //update control output
      pid_output_pitch = gp.updatePitch(kcf_center_normal_y, 1.0/2.0, dt, true);
      if((gim_pitch<body_min_pitch && pid_output_pitch<0) || (gim_pitch>body_max_pitch && pid_output_pitch>0))
	pid_output_pitch = 0;
      pid_output_yaw = -1.0 * gp.updateYaw(kcf_center_normal_x, 1.0/2.0, dt, true);
      if((this->angleNormal(gim_yaw - gim_min_yaw)<0 && pid_output_yaw<0) || (this->angleNormal(gim_yaw - gim_max_yaw)>0 && pid_output_yaw>0))
	pid_output_yaw = 0;
      cout<<"kcf_center_normal_y:"<<kcf_center_normal_y<<endl;
      cout<<"kcf_center_normal_x:"<<kcf_center_normal_x<<endl;
      cout<<"value_pitch:"<<pid_output_pitch<<endl;
      cout<<"value_yaw:"<<pid_output_yaw<<endl;
      //publish control output
      drone->gimbal_speed_control(0, pid_output_pitch, pid_output_yaw);
    }
    cout<<"dt:"<<dt<<endl;
    last_time = time;
  }
  
  void gimbalCallback(const dji_sdk::Gimbal::ConstPtr& msg)
  {
    if(gim_init_flag == false)
    {
      gim_init_yaw = msg->yaw;
      gim_min_yaw = this->angleNormal(gim_init_yaw + body_min_yaw);
      gim_max_yaw = this->angleNormal(gim_init_yaw + body_max_yaw);
      gim_init_flag = true;
    }
    gim_pitch = msg->pitch;
    gim_yaw = msg->yaw;
  }
  
  void keyboardControl()
  {
    ros::Rate rate(10);//frequency: n Hz
    
    cout<<"press A or D to change Roll"<<endl;
    cout<<"press W or S to change Pitch"<<endl;
    drone->request_sdk_permission_control();
    char c;
    while(ros::ok())
    {	
      c = getchar();
      //cout<<c<<endl;
      ros::spinOnce();
      
      if(c=='a'||c=='A')
      {
	for(int i = 0;i<20;i++)
	{
	  drone->gimbal_speed_control(0, 0, -100);
	  rate.sleep();
	}
      }
      else if(c=='d'||c=='D')
      {
	for(int i = 0;i<20;i++)
	{
	  drone->gimbal_speed_control(0, 0, 100);
	  rate.sleep();
	}
      }
      else if(c=='w'||c=='W')
      {
	for(int i = 0;i<20;i++)
	{
	  drone->gimbal_speed_control(0, 100, 0);
	  rate.sleep();
	}
      }
      else if(c=='s'||c=='S')
      {
	for(int i = 0;i<20;i++)
	{
	  drone->gimbal_speed_control(0, -100, 0);
	  rate.sleep();
	}
      }
    }
    drone->release_sdk_permission_control();
    ros::spin();
  }
  
  float angleNormal(float angle)
  {
    if(angle>180.0)
      return angle - 360.0;
    if(angle<-180.0)
      return angle + 360.0;
    return angle;
  }
};

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "gimbal_control_node");
  ros::NodeHandle nh_mode_param("~");
  bool pid_control_flag;
  if(!nh_mode_param.getParam("pid_control_flag", pid_control_flag))pid_control_flag = true;
  
  GimbalControl gc(pid_control_flag);
  
  if(pid_control_flag)
    ros::spin();
  else
    gc.keyboardControl();
  
  return 0;
}
