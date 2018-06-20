#ifndef __PIDCONTROLLER_H
#define __PIDCONTROLLER_H

#include <math.h>

struct MyPID
{
  float current;
  float target;
  float p;
  float i;
  float d;
  float integrator;
  float last_value;
  float last_target;
  float out;
  float out_max;
  float out_min;
  float dt;
  float outbias;
};

class GimbalPID
{
public:
  MyPID pid_pitch, pid_yaw;
  bool pid_init_end_flag;
  float start_distance_pitch, start_distance_yaw;
  
  GimbalPID()
  {
    pid_pitch.integrator = 0;
    pid_yaw.integrator = 0;
    pid_pitch.out = 0;
    pid_yaw.out = 0;
    pid_pitch.outbias = 0;
    pid_yaw.outbias = 0;
    pid_init_end_flag = false;
    start_distance_pitch = 1.0;
    start_distance_yaw = 1.0;
  }
  
  void initCurrent(float current_pitch, float current_yaw)
  {
    pid_pitch.current = current_pitch;
    pid_yaw.current = current_yaw;
    pid_pitch.last_value = current_pitch;
    pid_yaw.last_value = current_yaw;
  }
  void initTarget(float target_pitch, float target_yaw)
  {
    pid_pitch.target = target_pitch;
    pid_yaw.target = target_yaw;
    pid_pitch.last_target = target_pitch;
    pid_yaw.last_target = target_yaw;
  }
  void initPID(float p_pitch, float i_pitch, float d_pitch, float p_yaw, float i_yaw, float d_yaw)
  {
    pid_pitch.p = p_pitch;
    pid_pitch.i = i_pitch;
    pid_pitch.d = d_pitch;
    pid_yaw.p = p_yaw;
    pid_yaw.i = i_yaw;
    pid_yaw.d = d_yaw;
  }
  void initDt(float dt)
  {
    pid_pitch.dt = dt;
    pid_yaw.dt = dt;
  }
  void initOutConstrain(float min_pitch, float max_pitch, float min_yaw, float max_yaw)
  {
    pid_pitch.out_min = min_pitch;
    pid_pitch.out_max = max_pitch;
    pid_yaw.out_min = min_yaw;
    pid_yaw.out_max = max_yaw;
  }
  
  void initStartDistance(float start_distance_pitch0, float start_distance_yaw0)
  {
    start_distance_pitch = start_distance_pitch0;
    start_distance_yaw = start_distance_yaw0;
  }
  
  float updatePitch(float current_pitch, float target_pitch, float dt, bool use_start_distance)
  {
    pid_pitch.current = current_pitch;
    if(use_start_distance)
      pid_pitch.target = updateTarget(current_pitch, target_pitch, start_distance_pitch);
    else
      pid_pitch.target = target_pitch;
    pid_pitch.dt = dt;
    return updatePID(pid_pitch);
  }
  
  float updateYaw(float current_yaw, float target_yaw, float dt, bool use_start_distance)
  {
    pid_yaw.current = current_yaw;
    if(use_start_distance)
      pid_yaw.target = updateTarget(current_yaw, target_yaw, start_distance_yaw);
    else
      pid_yaw.target = target_yaw;
    pid_yaw.dt = dt;
    return updatePID(pid_yaw);
  }
  
private:
  float updateTarget(float current, float target, float max_distance)
  {
    return current + constrainValue(target - current, -max_distance, max_distance);
  }
    
  float updatePID(MyPID &PID)
  {
    float error,last_error,PTerm,ITerm,DTerm,PIDout;
    error = PID.target - PID.current;
    last_error = PID.last_target - PID.last_value;
    //P
    PTerm = error * PID.p;
    //I
    PID.integrator += error * PID.i * PID.dt;
    PID.integrator = constrainValue(PID.integrator,PID.out_min,PID.out_max);
    ITerm = PID.integrator;
    //D
    DTerm = (error - last_error) * PID.d / PID.dt;
    //out
    PIDout = PTerm + ITerm + DTerm;//OUT=P+I+D
    PIDout = constrainValue(PIDout,PID.out_min,PID.out_max);
    PID.out = PIDout;
    PID.last_value = PID.current;
    PID.last_target = PID.target;
    return PID.out + PID.outbias;
  }
  
  float constrainValue(float value, float min, float max)
  {
    if(value > max)value = max;
    else if(value < min)value = min;
    return value;
  }
};

#endif