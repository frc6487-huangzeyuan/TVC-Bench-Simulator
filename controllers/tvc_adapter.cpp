#include "sim_api.h"
#include "GimbalControl.h"
#include "HistoryAttitude.h"
#include <memory>
// 直接调用工程原始头文件，不在 Python/JavaScript 中重写 PID。
static std::unique_ptr<GimbalPid> pid;
static Attitude attitude;
static SimConfig settings;
static GimbalConfig pidConfig;
static int sent[2] = {1500,1500};
static float previous_gyro[3]={0,0,0};
SIM_API int sim_abi_version(){return 1;}
SIM_API void sim_reset(const SimConfig* c){
  settings=*c; attitude=Attitude(); sent[0]=sent[1]=1500;
  for(int k=0;k<3;++k)previous_gyro[k]=0;
  GimbalConfig p{{float(c->kp[0]),float(c->ki[0]),float(c->kd[0])},
                 {float(c->kp[1]),float(c->ki[1]),float(c->kd[1])},
                 float(c->correction_limit),float(c->integral_limit),float(c->derivative_filter),
                 float(c->azimuth_zero),float(c->sign[0]),float(c->sign[1]),
                 int(c->slew_us),uint32_t(c->max_gap_us)};
  pidConfig=p;pid.reset(new GimbalPid(p));
}
#ifdef TVC_MODEL_COMPENSATION_API
// 可选 sim_api v1 扩展。参数只来自本次仿真模型，不自动写入真实飞控。
SIM_API int sim_set_model(const double *values){
  auto &m=pidConfig.model;m.enabled=true;
  for(int k=0;k<2;++k)m.bias[k]=float(values[k]);
  for(int k=0;k<4;++k){m.gravity[k]=float(values[2+k]);m.inverse[k]=float(values[6+k]);}
  m.accelerationLimit=float(values[10]);
  pid.reset(new GimbalPid(pidConfig));return pid->configurationValid()?0:-1;
}
#endif
SIM_API int sim_preposition(double x,double y,double current_azimuth,double *pwm){
  float vx,vy,azimuth,elevation;
#ifdef TVC_MODEL_COMPENSATION_API
  GimbalPid::initialVector(pidConfig,float(x),float(y),vx,vy);
  current_azimuth=GimbalPid::initialAzimuthReference(pidConfig,float(current_azimuth),vx,vy);
#else
  vx=pidConfig.signX*std::max(-pidConfig.limit,std::min(pidConfig.limit,-pidConfig.x.kp*float(x)));
  vy=pidConfig.signY*std::max(-pidConfig.limit,std::min(pidConfig.limit,-pidConfig.y.kp*float(y)));
  const float magnitude=std::hypot(vx,vy);
  if(magnitude>pidConfig.limit){vx*=pidConfig.limit/magnitude;vy*=pidConfig.limit/magnitude;}
#endif
  if(!std::isfinite(vx)||!std::isfinite(vy))return -1;
  GimbalPid::vectorToAngles(vx,vy,float(current_azimuth),pidConfig.azimuthZero,azimuth,elevation);
  pwm[0]=limitServoUs(0,GimbalPid::angleToUs(azimuth,0));
  pwm[1]=limitServoUs(1,GimbalPid::angleToUs(elevation,1));return 0;
}
// sim_api v1 可选扩展，仅设置仿真释放前的执行器状态，不修改原飞控源码。
SIM_API void sim_set_initial_pwm(double azimuth_us,double elevation_us){
  sent[0]=int(std::lround(azimuth_us));sent[1]=int(std::lround(elevation_us));
}
SIM_API int sim_sample(const SimInput* i){
  float g[3],a[3];for(int k=0;k<3;++k){g[k]=float(i->gyro_dps[k]);a[k]=float(i->accel_g[k]);}
  bool ok;
  if(!attitude.initialized())ok=attitude.initializeFromAccel(a[0],a[1],a[2]);
  else ok=acceptHistorySample(attitude,previous_gyro,g,a,uint32_t(i->dt*1e6+.5),true);
  for(int k=0;k<3;++k)previous_gyro[k]=g[k];return ok?0:-2;
}
SIM_API int sim_step(const SimInput* i,SimOutput* o){
  if(!pid || !pid->configurationValid() || !attitude.initialized())return -3;
  Attitude predicted;
  if(!projectHistory(attitude,previous_gyro,uint64_t(i->sample_age_s*1e6),150000,predicted))return -6;
  AttitudeMatrix r=predicted.rotation();
  if(settings.use_truth>.5)for(int a=0;a<3;++a)for(int b=0;b<3;++b)r.m[a][b]=float(i->truth_rotation[a*3+b]);
  float x,y;if(!gimbalLean(r,x,y))return -4;
  int target[2]={sent[0],sent[1]};
  auto result=pid->update(uint64_t(i->time_us),x,y,sent,target,i->sample_age_s<=.05);
  if(result==GimbalPid::INVALID)return -5;
  sent[0]=target[0];sent[1]=target[1];
  o->pwm_us[0]=sent[0];o->pwm_us[1]=sent[1];o->throttle_pct=settings.throttle_pct;
  o->estimated_lean_deg[0]=x;o->estimated_lean_deg[1]=y;
  auto tx=pid->x().terms(),ty=pid->y().terms();
  o->pid[0]=tx.p;o->pid[1]=tx.i;o->pid[2]=tx.d;
  o->pid[3]=ty.p;o->pid[4]=ty.i;o->pid[5]=ty.d;
  o->limited=pid->limited()?1:0;o->status=int(result);return 0;
}
