#include "ros/ros.h"
#include <robot_position_estimate/RobotPositionEstimate.h>
#include <dji_sdk/Gimbal.h>
#include <dji_sdk/LocalPosition.h>
#include <robot_kcftracker/KCFTracker.h>
#include <math.h>

using namespace std;

class PositionEstimate
{
public:
  //node
  ros::NodeHandle nh;
  ros::NodeHandle nh_param;
  ros::Subscriber sub_kcf;
  ros::Subscriber sub_gim;
  ros::Subscriber sub_loc;
  ros::Publisher pub_pos;
  robot_position_estimate::RobotPositionEstimate msg_pos;
  //kcf track
  int kcf_x, kcf_y, kcf_width, kcf_height, kcf_center_x, kcf_center_y;
  double image_width, image_height;
  //gimbal
  float gim_pitch;
  //LocalPosition
  bool listen_h_flag;
  double loc_h;
  //camera
  double fu,fv;
  //estimate
  float c2r_x;//camera 2 robot, camera frame
  float c2r_y;
  float gu, gv, pu, pv, lu, lv, alpha, yo, h;
  
  PositionEstimate():
  nh_param("~")
  {
    //node
    sub_kcf = nh.subscribe("/kcf_track", 10, &PositionEstimate::kcfCallback,this);
    sub_gim = nh.subscribe("/dji_sdk/gimbal", 10, &PositionEstimate::gimbalCallback,this);
    sub_loc = nh.subscribe("/dji_sdk/local_position", 10, &PositionEstimate::localPositionCallback,this);
    pub_pos  = nh.advertise<robot_position_estimate::RobotPositionEstimate>("/robot_position", 10);
    //camera
    if(!nh_param.getParam("fu", fu))fu = 376.629954;
    if(!nh_param.getParam("fv", fv))fv = 494.151786;
    //LocalPosition
    if(!nh_param.getParam("listen_h_flag", listen_h_flag))listen_h_flag = true;
    if(!nh_param.getParam("loc_h", loc_h))loc_h = 1.0;
  }
  
  ~PositionEstimate(){}
  
  void kcfCallback(const robot_kcftracker::KCFTracker::ConstPtr& msg)
  {
    kcf_x = msg->kcf_x;
    kcf_y = msg->kcf_y;
    kcf_width = msg->kcf_width;
    kcf_height = msg->kcf_height;
    image_width = msg->image_width;
    image_height = msg->image_height;
    kcf_center_x = kcf_x + kcf_width / 2;
    kcf_center_y = kcf_y + kcf_height / 2;
    //runEstimate
    if(gim_pitch>-90.0 && gim_pitch<30.0 && loc_h>0.5 && loc_h<4)
    {
      this->runEstimate();
    }
    else
    {
      cout<<"gim_pitch or loc_h is error"<<endl;
      cout<<"gim_pitch: "<<gim_pitch<<endl;
      cout<<"loc_h: "<<loc_h<<endl;
    }
  }
  
  void gimbalCallback(const dji_sdk::Gimbal::ConstPtr& msg)
  {
    gim_pitch = msg->pitch;
  }
  
  void localPositionCallback(const dji_sdk::LocalPosition::ConstPtr& msg)
  {
    if(listen_h_flag)
      loc_h = msg->z;
  }
  
  void runEstimate()
  {
    //c2r_x
    gv = image_height / 2.0;
    pv = kcf_center_y;
    yo = gim_pitch / 360.0 * (2.0*M_PI);
    h = loc_h;
    
    alpha = atan((gv - pv) /fv);
    c2r_x = tan(alpha + yo + M_PI/2.0) * h;
    
    //c2r_y
    lu = kcf_center_x;
    pu = image_width / 2.0;
    
    c2r_y = (lu-pu)*cos(alpha)*h / (fu*cos(alpha+yo+M_PI/2.0));
    /*
    cout<<"gv: "<<gv<<endl;
    cout<<"pv: "<<pv<<endl;
    cout<<"yo: "<<yo<<endl;
    cout<<"h: "<<h<<endl;
    cout<<"(gv - pv) /fv: "<<(gv - pv) /fv<<endl;
    cout<<"alpha: "<<alpha<<endl;
    cout<<"c2r_x: "<<c2r_x<<endl;
    cout<<"lu: "<<lu<<endl;
    cout<<"pu: "<<pu<<endl;
    cout<<"(lu-pu)*cos(alpha)*h: "<<(lu-pu)*cos(alpha)*h<<endl;
    cout<<"(fu*cos(alpha+yo)): "<<(fu*cos(alpha+yo))<<endl;
    cout<<"c2r_y: "<<c2r_y<<endl;
    */
    //publish
    msg_pos.camera_coordinate_x = c2r_x;
    msg_pos.camera_coordinate_y = c2r_y;
    pub_pos.publish(msg_pos);
  }
};

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "robot_position_estimate_node");
  PositionEstimate pe;
  ros::spin();
  return 0;
}
