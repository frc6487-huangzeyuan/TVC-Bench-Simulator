#pragma once
// 仿真接入接口 v1。统一秒、度/秒、g、微秒；矩阵为行优先机体到世界旋转。
#ifdef _WIN32
#define SIM_API extern "C" __declspec(dllexport)
#else
#define SIM_API extern "C"
#endif
struct SimConfig {
  double kp[2], ki[2], kd[2];
  double integral_limit, derivative_filter, correction_limit;
  double sign[2], azimuth_zero, slew_us, max_gap_us, throttle_pct, use_truth;
};
struct SimInput {
  double time_us, dt, gyro_dps[3], accel_g[3], truth_rotation[9], sample_age_s;
};
struct SimOutput {
  double pwm_us[2], throttle_pct, estimated_lean_deg[2], pid[6], limited, status;
};
SIM_API int sim_abi_version();
SIM_API void sim_reset(const SimConfig* config);
SIM_API int sim_sample(const SimInput* input);
SIM_API int sim_step(const SimInput* input, SimOutput* output);
