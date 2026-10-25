#include "extra.h"
#include "sim_api.h"
#include <cmath>
#include <algorithm>
// 可运行的真实状态 PD 参考控制器。使用真值便于隔离控制律与姿态估计问题。
// 接入实机算法时，在 sim_sample 中使用 gyro_dps / accel_g 更新自己的估计器。
static SimConfig config;
SIM_API int sim_abi_version(){return 1;}
SIM_API void sim_reset(const SimConfig* c){config=*c;config.kp[0]=extra_gain(config.kp[0]);}
SIM_API int sim_sample(const SimInput*){return 0;}
SIM_API int sim_step(const SimInput* i,SimOutput* o){
  constexpr double rad=57.29577951308232;
  double x=std::atan2(-i->truth_rotation[6],i->truth_rotation[8])*rad;
  double y=std::atan2(-i->truth_rotation[7],i->truth_rotation[8])*rad;
  double vx=-config.kp[0]*x-config.kd[0]*i->gyro_dps[1];
  double vy=-config.kp[1]*y+config.kd[1]*i->gyro_dps[0];
  double amount=std::min(config.correction_limit,std::hypot(vx,vy));
  double az=amount<.001?180:std::atan2(vy,vx)*rad+180;
  o->pwm_us[0]=500+std::clamp(az,0.,360.)/360*2000;
  o->pwm_us[1]=1500+amount/270*2000;
  o->throttle_pct=config.throttle_pct;
  o->estimated_lean_deg[0]=x;o->estimated_lean_deg[1]=y;
  o->pid[0]=-config.kp[0]*x;o->pid[3]=-config.kp[1]*y;
  return 0;
}
