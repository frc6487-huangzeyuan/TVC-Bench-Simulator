/*
 * ESP32-S3-N16R8 + MPU6050：X 轴倾斜补偿与舵机回中
 * 接线：舵机信号 GPIO1；MPU6050 SCL GPIO11、SDA GPIO12。
 * MPU6050 模块使用 3.3V，所有设备共地；舵机使用符合型号要求的独立电源。
 * 依赖：ESP32Servo（库管理器安装），Wire 由 ESP32 开发板包提供。
 *
 * 上电后保持静止约 4 秒，当前姿态作为零位。
 * 默认倾角 +20° -> 舵机目标 70°；恢复零位 -> 舵机目标 90°。
 * 这是单轴、近水平的姿态补偿示例；角度来自陀螺仪和加速度计融合。
 */

#include <Arduino.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <math.h>

#if !defined(CONFIG_IDF_TARGET_ESP32S3)
#error "Select ESP32S3 Dev Module. These pins are for the confirmed ESP32-S3 board."
#endif

// ==================== 接线与可调参数 ====================
constexpr int SERVO_PIN = 1;
constexpr int MPU_SCL_PIN = 11;
constexpr int MPU_SDA_PIN = 12;

constexpr float SERVO_CENTER_DEG = 90.0f; // 机械中位需要偏移时修改这里
constexpr float SERVO_MIN_DEG = 45.0f;    // 初始只允许中位两侧各 45°
constexpr float SERVO_MAX_DEG = 135.0f;
constexpr float SERVO_DIRECTION = -1.0f; // 补偿方向反了就改为 +1.0f
constexpr float CORRECTION_GAIN = 1.0f;  // 每倾斜 1°，舵机补偿 1°
constexpr float CENTER_DEADBAND_DEG = 0.5f; // 零位附近直接回中，减少抖动

// 常见位置舵机的示例脉宽范围，实际行程以舵机规格为准。
constexpr int SERVO_MIN_US = 500;
constexpr int SERVO_MAX_US = 2500;
constexpr uint32_t SAMPLE_PERIOD_US = 5000; // 200Hz 读取和计算
constexpr uint32_t PRINT_PERIOD_MS = 100;  // 10Hz 串口输出，115200 波特率
constexpr float FILTER_TIME_CONSTANT_S = 0.25f;
constexpr uint16_t CALIBRATION_SAMPLES = 400; // 约 2 秒静止采样

Servo servo;
uint8_t mpuAddress = 0;
float ax = 0, ay = 0, az = 0; // 加速度，单位 g
float gx = 0, gy = 0, gz = 0; // 原始换算后的角速度，单位 °/s
float gyroBias[3] = {0, 0, 0};
float rollDeg = 0;
float zeroRollDeg = 0;
float servoCommandDeg = SERVO_CENTER_DEG;
uint32_t previousSampleUs = 0;
uint32_t previousPrintMs = 0;

// 只包回 [-180, 180)，避免跨过 ±180°时突然跳变。
float wrapDegrees(float angle) {
  while (angle >= 180.0f) angle -= 360.0f;
  while (angle < -180.0f) angle += 360.0f;
  return angle;
}

void stopWithError(const char *message) {
  if (servo.attached()) servo.write(SERVO_CENTER_DEG);
  Serial.print("# 错误：");
  Serial.println(message);
  Serial.println("# 已停止补偿；检查后按开发板 RESET/EN 重启。");
  while (true) delay(1000);
}

bool readRegisters(uint8_t address, uint8_t reg, uint8_t *data, size_t length) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(address, length, true) != length) return false;
  for (size_t i = 0; i < length; ++i) data[i] = Wire.read();
  return true;
}

bool writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(mpuAddress);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

int16_t signedWord(const uint8_t *bytes) {
  return static_cast<int16_t>((static_cast<uint16_t>(bytes[0]) << 8) | bytes[1]);
}

bool readMotion() {
  uint8_t data[14];
  // 从 0x3B 连读：加速度 XYZ、温度、陀螺仪 XYZ；本例不使用温度。
  if (!readRegisters(mpuAddress, 0x3B, data, sizeof(data))) return false;
  // 与下面设置的 ±2g、±500°/s 量程相对应。
  ax = signedWord(data + 0) / 16384.0f;
  ay = signedWord(data + 2) / 16384.0f;
  az = signedWord(data + 4) / 16384.0f;
  gx = signedWord(data + 8) / 65.5f;
  gy = signedWord(data + 10) / 65.5f;
  gz = signedWord(data + 12) / 65.5f;
  return true;
}

void initializeMpu() {
  // AD0 接低时地址 0x68，接高时地址 0x69；都实际检查一次。
  for (uint8_t address = 0x68; address <= 0x69; ++address) {
    uint8_t identity = 0;
    if (!readRegisters(address, 0x75, &identity, 1)) continue;
    Serial.printf("# I2C=0x%02X, WHO_AM_I=0x%02X\n", address, identity);
    // 通信地址和芯片身份是不同概念，MPU6050 的身份应为 0x68。
    if (identity == 0x68) {
      mpuAddress = address;
      break;
    }
  }
  if (mpuAddress == 0) stopWithError("未找到 MPU6050，请检查接线、供电和芯片型号。");

  if (!writeRegister(0x6B, 0x80)) stopWithError("MPU6050 复位失败。");
  delay(100);
  if (!writeRegister(0x6B, 0x01) || // 唤醒，使用 X 轴陀螺仪 PLL 时钟
      !writeRegister(0x6C, 0x00) || // 启用所有测量轴
      !writeRegister(0x1A, 0x03) || // 数字低通：陀螺仪约 42Hz
      !writeRegister(0x19, 0x04) || // 1kHz / (1 + 4) = 200Hz
      !writeRegister(0x1B, 0x08) || // 陀螺仪 ±500°/s
      !writeRegister(0x1C, 0x00))  // 加速度计 ±2g
    stopWithError("MPU6050 参数写入失败。");
  delay(100);
}

void calibrateGyroAndZero() {
  Serial.println("# 开始静止校准：保持传感器不动，当前姿态将作为零位。");
  float mean[6] = {0}; // 前三项角速度，后三项加速度
  float m2[6] = {0};   // 在线统计方差，用于提示校准期间的明显运动
  for (uint16_t n = 1; n <= CALIBRATION_SAMPLES; ++n) {
    if (!readMotion()) stopWithError("校准期间 I2C 读取失败。");
    const float values[6] = {gx, gy, gz, ax, ay, az};
    for (int i = 0; i < 6; ++i) {
      const float difference = values[i] - mean[i];
      mean[i] += difference / n;
      m2[i] += difference * (values[i] - mean[i]);
    }
    delay(5);
  }
  bool moved = false;
  for (int i = 0; i < 6; ++i) {
    const float deviation = sqrtf(m2[i] / (CALIBRATION_SAMPLES - 1));
    if (i < 3) {
      Serial.printf("# gyro%c: bias=%.3f deg/s, std=%.3f deg/s\n", 'X' + i, mean[i], deviation);
      if (deviation > 1.0f) moved = true;
    } else if (deviation > 0.025f) {
      moved = true;
    }
  }
  const float gravity = sqrtf(mean[3] * mean[3] + mean[4] * mean[4] + mean[5] * mean[5]);
  if (moved || gravity < 0.85f || gravity > 1.15f)
    stopWithError("校准数据波动过大或重力异常；请静置后重新启动校准。");
  for (int i = 0; i < 3; ++i) gyroBias[i] = mean[i];
  // 初始姿态建议芯片正面朝上、接近水平。绕板上 X 箭头方向转动为本例控制轴。
  zeroRollDeg = atan2f(mean[4], mean[5]) * RAD_TO_DEG;
  rollDeg = zeroRollDeg;
  Serial.printf("# 校准完成，X 轴零位=%.2f deg\n", zeroRollDeg);
}

void setup() {
  Serial.begin(115200);
  // 原生 USB 串口最多等待 1.5 秒，未打开串口也能继续运行。
  const uint32_t serialStartMs = millis();
  while (!Serial && millis() - serialStartMs < 1500) delay(10);

  servo.setPeriodHertz(50);
  servo.attach(SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);
  if (!servo.attached()) stopWithError("舵机 PWM 初始化失败。");
  servo.write(SERVO_CENTER_DEG);
  Serial.println("# ESP32-S3：舵机已命令回中，准备读取 MPU6050。");
  // Wire.begin 的参数顺序是 SDA、SCL，不是 SCL、SDA。
  if (!Wire.begin(MPU_SDA_PIN, MPU_SCL_PIN, 400000))
    stopWithError("I2C 初始化失败。");
  Wire.setTimeOut(20);
  initializeMpu();
  delay(1000); // 等待舵机回中、传感器和支架静止
  calibrateGyroAndZero();
  Serial.println("# 角速度单位 deg/s；角度单位 deg；servo_cmd 是指令，不是实测位置。");
  Serial.println("ms,gx_dps,gy_dps,gz_dps,roll_deg,relative_x_deg,servo_cmd_deg");
  previousSampleUs = micros();
}

void loop() {
  const uint32_t nowUs = micros();
  const uint32_t elapsedUs = nowUs - previousSampleUs;
  if (elapsedUs < SAMPLE_PERIOD_US) {
    delay(1);
    return;
  }
  previousSampleUs = nowUs;
  if (!readMotion()) stopWithError("MPU6050 读取失败，舵机已命令回中。");
  const float dt = elapsedUs * 1.0e-6f;
  const float gyroX = gx - gyroBias[0];
  const float gyroY = gy - gyroBias[1];
  const float gyroZ = gz - gyroBias[2];

  // 陀螺仪负责快速变化，加速度计提供重力方向，修正长期积分漂移。
  const float accelRollDeg = atan2f(ay, az) * RAD_TO_DEG;
  const float alpha = FILTER_TIME_CONSTANT_S / (FILTER_TIME_CONSTANT_S + dt);
  const float predictedRoll = wrapDegrees(rollDeg + gyroX * dt);
  rollDeg = wrapDegrees(predictedRoll + (1.0f - alpha) * wrapDegrees(accelRollDeg - predictedRoll));
  const float relativeRoll = wrapDegrees(rollDeg - zeroRollDeg);

  // 反向比例补偿：回到零位时输出中心角；限幅避免直接打到舵机两端。
  servoCommandDeg = SERVO_CENTER_DEG;
  if (fabsf(relativeRoll) > CENTER_DEADBAND_DEG)
    servoCommandDeg += SERVO_DIRECTION * CORRECTION_GAIN * relativeRoll;
  servoCommandDeg = constrain(servoCommandDeg, SERVO_MIN_DEG, SERVO_MAX_DEG);
  servo.write(static_cast<int>(lroundf(servoCommandDeg)));

  const uint32_t nowMs = millis();
  if (nowMs - previousPrintMs >= PRINT_PERIOD_MS) {
    previousPrintMs = nowMs;
    Serial.printf("%lu,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f\n",
                  static_cast<unsigned long>(nowMs), gyroX, gyroY, gyroZ,
                  rollDeg, relativeRoll, servoCommandDeg);
  }
}
