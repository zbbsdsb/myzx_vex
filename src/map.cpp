/#include <cmath>
#include<vex.h>
#include <algorithm>

// 卡尔曼滤波器类
class KalmanFilter {
private:
  double Q_angle;   // 过程噪声协方差 - 角度
  double Q_bias;    // 过程噪声协方差 - 偏差
  double R_measure; // 测量噪声协方差
  
  double angle;     // 角度估计
  double bias;      // 偏差估计
  double rate;      // 角速率
  
  double P[2][2];  // 误差协方差矩阵
  
public:
  KalmanFilter() {
    Q_angle = 0.001;
    Q_bias = 0.003;
    R_measure = 0.03;
    
    angle = 0.0;
    bias = 0.0;
    rate = 0.0;
    
    // 初始化误差协方差矩阵
    P[0][0] = 0.0;
    P[0][1] = 0.0;
    P[1][0] = 0.0;
    P[1][1] = 0.0;
  }
  
  // 设置噪声参数
  void setNoise(double q_angle, double q_bias, double r_measure) {
    Q_angle = q_angle;
    Q_bias = q_bias;
    R_measure = r_measure;
  }
  
  // 卡尔曼滤波更新
  double update(double newAngle, double newRate, double dt) {
    // 预测步骤
    rate = newRate - bias;
    angle += dt * rate;
    
    // 更新误差协方差矩阵
    P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;
    
    // 计算卡尔曼增益
    double S = P[0][0] + R_measure;
    double K[2];
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;
    
    // 更新估计
    double y = newAngle - angle;
    angle += K[0] * y;
    bias += K[1] * y;
    
    // 更新误差协方差
    double P00_temp = P[0][0];
    double P01_temp = P[0][1];
    
    P[0][0] -= K[0] * P00_temp;
    P[0][1] -= K[0] * P01_temp;
    P[1][0] -= K[1] * P00_temp;
    P[1][1] -= K[1] * P01_temp;
    
    return angle;
  }
  
  double getAngle() { return angle; }
  double getRate() { return rate; }
};


// 扩展机器人状态结构
struct RobotState {
  double x;        // 全局X坐标 (mm)
  double y;        // 全局Y坐标 (mm)  
  double heading;  // 全局航向 (度)
  double vx;       // X方向速度 (mm/s)
  double vy;       // Y方向速度 (mm/s)
  double omega;    // 角速度 (度/s)
  
  // 卡尔曼滤波状态
  double x_kalman;     // 卡尔曼滤波后的X坐标
  double y_kalman;     // 卡尔曼滤波后的Y坐标
  double heading_kalman; // 卡尔曼滤波后的航向
};

// 卡尔曼滤波器实例
KalmanFilter headingFilter;    // 航向卡尔曼滤波
KalmanFilter xPositionFilter;  // X位置卡尔曼滤波  
KalmanFilter yPositionFilter;  // Y位置卡尔曼滤波

// 传感器噪声参数 (需要根据实际测量调整)
const double HEADING_PROCESS_NOISE = 0.001;
const double HEADING_BIAS_NOISE = 0.003;  
const double HEADING_MEASURE_NOISE = 0.03;

const double POSITION_PROCESS_NOISE = 0.1;
const double POSITION_BIAS_NOISE = 0.01;
const double POSITION_MEASURE_NOISE = 5.0;


// 机器人状态结构
struct RobotState {
  double x;        // 全局X坐标 (mm)
  double y;        // 全局Y坐标 (mm)  
  double heading;  // 全局航向 (度, 0=东, 90=北, 180=西, 270=南)
  double vx;       // X方向速度 (mm/s)
  double vy;       // Y方向速度 (mm/s)
  double omega;    // 角速度 (度/s)
};

// 传感器和电机定义 (根据实际端口修改)
rotation odomWheel1 = rotation(PORT19);  // 第一个万向轮旋转传感器
rotation odomWheel2 = rotation(PORT20);  // 第二个万向轮旋转传感器
inertial imu = inertial(PORT16);         // 惯性传感器

// 驱动电机定义 - 左右各3个电机
motor leftMotor1 = motor(PORT1, ratio18_1, false);
motor leftMotor2 = motor(PORT2, ratio18_1, false);
motor leftMotor3 = motor(PORT3, ratidouble kP = 0.23;  // 比例系数
double kI = 0.12;  // 积分系数  
double kD = 0.05;  // 微分系数double kP = 0.23;  // 比例系数
double kI = 0.12;  // 积分系数  
double kD = 0.05;  // 微分系数double kP = 0.23;  // 比例系数
double kI = 0.12;  // 积分系数  
double kD = 0.05;  // 微分系数o18_1, false);

motor rightMotor1 = motor(PORT10, ratio18_1, true);
motor rightMotor2 = motor(PORT9, ratio18_1, true);
motor rightMotor3 = motor(PORT8, ratio18_1, true);

// 电机组定义
motor_group leftMotors = motor_group(leftMotor1, leftMotor2, leftMotor3);
motor_group rightMotors = motor_group(rightMotor1, rightMotor2, rightMotor3);

// 全局状态变量
RobotState currentState = {0, 0, 0, 0, 0, 0};
RobotState targetState = {0, 0, 0, 0, 0, 0};

// 传感器历史数据
double lastOdom1 = 0;
double lastOdom2 = 0;
double lastIMUHeading = 0;
double lastUpdateTime = 0;

// 系统参数 (需要根据实际机器人校准)
const double ODOM_WHEEL_DIAMETER = 3.25 * 25.4; // 3.25英寸转毫米
const double ODOM_WHEEL_CIRCUMFERENCE = ODOM_WHEEL_DIAMETER * M_PI;
const double ODOM_TICKS_PER_REV = 360.0; // V5旋转传感器每转360个单位
const double MM_PER_TICK = ODOM_WHEEL_CIRCUMFERENCE / ODOM_TICKS_PER_REV;

// 万向轮安装位置和方向参数
// 假设万向轮1指向机器人前方(0度)，万向轮2指向机器人左侧(90度)
const double ODOM1_ANGLE = 0.0;    // 相对于机器人前方角度(度)
const double ODOM2_ANGLE = 90.0;  // 相对于机器人前方角度(度)

// 万向轮相对于机器人中心的安装位置 (需要测量)
//const double ODOM1_X_OFFSET = 0.0; // 在机器人坐标系中的X偏移(mm)
//const double ODOM1_Y_OFFSET = 100.0; // 在机器人坐标系中的Y偏移(mm)
//const double ODOM2_X_OFFSET = -100.0;// 在机器人坐标系中的X偏移(mm)  
//const double ODOM2_Y_OFFSET = 0.0;   // 在机器人坐标系中的Y偏移(mm)

// 控制参数
const double KP_POSITION = 0.4;     // 位置控制比例系数
const double KI_POSITION = 0.01;    // 位置控制积分系数
const double KD_POSITION = 0.2;     // 位置控制微分系数

const double KP_HEADING = 1.2;      // 航向控制比例系数  
const double KI_HEADING = 0.02;     // 航向控制积分系数
const double KD_HEADING = 0.3;      // 航向控制微分系数

const double MAX_LINEAR_SPEED = 600.0;  // 最大线速度 mm/s
const double MAX_ANGULAR_SPEED = 180.0; // 最大角速度 度/s
const double POSITION_TOLERANCE = 15.0; // 位置容差 mm
const double HEADING_TOLERANCE = 2.0;   // 航向容差 度

// PID控制器结构
struct PIDController {
  double kp, ki, kd;
  double integral;
  double previous_error;
  double output_limit;
};

// 初始化PID控制器
void initPID(PIDController& pid, double kp, double ki, double kd, double limit) {
  pid.kp = kp;
  pid.ki = ki;
  pid.kd = kd;
  pid.integral = 0;
  pid.previous_error = 0;
  pid.output_limit = limit;
}

// 计算PID输出
double computePID(PIDController& pid, double error, double dt) {
  pid.integral += error * dt;
  double derivative = (error - pid.previous_error) / dt;
  
  double output = pid.kp * error + pid.ki * pid.integral + pid.kd * derivative;
  
  // 限制输出
  output = std::max(std::min(output, pid.output_limit), -pid.output_limit);
  
  pid.previous_error = error;
  return output;
}

// 初始化定位系统
// 初始化定位系统
void initLocalization() {
  // 重置传感器
  odomWheel1.resetPosition();
  odomWheel2.resetPosition();
  imu.calibrate();
  
  // 等待IMU校准完成
  while (imu.isCalibrating()) {
    wait(50, msec);
  }
  
  // 初始化状态
  lastOdom1 = odomWheel1.position(degrees);
  lastOdom2 = odomWheel2.position(degrees);
  lastIMUHeading = imu.heading();
  lastUpdateTime = Brain.Timer.value();
  
  currentState.heading = lastIMUHeading;
  currentState.x_kalman = currentState.x;
  currentState.y_kalman = currentState.y;
  currentState.heading_kalman = currentState.heading;
  
  // 初始化卡尔曼滤波器
  headingFilter.setNoise(HEADING_PROCESS_NOISE, HEADING_BIAS_NOISE, HEADING_MEASURE_NOISE);
  xPositionFilter.setNoise(POSITION_PROCESS_NOISE, POSITION_BIAS_NOISE, POSITION_MEASURE_NOISE);
  yPositionFilter.setNoise(POSITION_PROCESS_NOISE, POSITION_BIAS_NOISE, POSITION_MEASURE_NOISE);
  
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1, 1);
  Brain.Screen.print("Localization System Ready");
  Brain.Screen.setCursor(2, 1);
  Brain.Screen.print("Kalman Filter Initialized");
}

// 更新机器人位置估计 (基于万向轮编码器和卡尔曼滤波)
void updatePosition() {
  double currentTime = Brain.Timer.value();
  double dt = currentTime - lastUpdateTime;
  if (dt <= 0) dt = 0.01; // 避免除零
  
  // 读取当前传感器值
  double currentOdom1 = odomWheel1.position(degrees);
  double currentOdom2 = odomWheel2.position(degrees);
  double currentIMUHeading = imu.heading();
  double currentIMURate = imu.gyroRate(zaxis, dps); // 获取角速度
  
  // 计算传感器变化量
  double deltaOdom1 = currentOdom1 - lastOdom1;
  double deltaOdom2 = currentOdom2 - lastOdom2;
  
  // 处理航向角度跳变
  double deltaHeading = currentIMUHeading - lastIMUHeading;
  if (deltaHeading > 180) deltaHeading -= 360;
  if (deltaHeading < -180) deltaHeading += 360;
  
  // 将编码器变化转换为位移(mm)
  double deltaDistance1 = deltaOdom1 * MM_PER_TICK;
  double deltaDistance2 = deltaOdom2 * MM_PER_TICK;
  
  // 使用平均航向进行坐标转换
  double avgHeadingRad = (currentState.heading + currentIMUHeading) * 0.5 * M_PI / 180.0;
  
  // 计算每个万向轮在机器人坐标系中的位移
  double wheel1DeltaX = deltaDistance1 * cos(ODOM1_ANGLE * M_PI / 180.0);
  double wheel1DeltaY = deltaDistance1 * sin(ODOM1_ANGLE * M_PI / 180.0);
  
  double wheel2DeltaX = deltaDistance2 * cos(ODOM2_ANGLE * M_PI / 180.0);
  double wheel2DeltaY = deltaDistance2 * sin(ODOM2_ANGLE * M_PI / 180.0);
  
  // 将轮子位移转换到机器人中心
  double robotDeltaX = (wheel1DeltaX + wheel2DeltaX) / 2.0;
  double robotDeltaY = (wheel1DeltaY + wheel2DeltaY) / 2.0;
  
  // 将机器人坐标系位移转换到全局坐标系
  double globalDeltaX = robotDeltaX * cos(avgHeadingRad) - robotDeltaY * sin(avgHeadingRad);
  double globalDeltaY = robotDeltaX * sin(avgHeadingRad) + robotDeltaY * cos(avgHeadingRad);
  
  // 更新原始状态估计
  currentState.x += globalDeltaX;
  currentState.y += globalDeltaY;
  currentState.heading = currentIMUHeading;
  
  // 应用卡尔曼滤波
  currentState.heading_kalman = headingFilter.update(currentIMUHeading, currentIMURate, dt);
  currentState.x_kalman = xPositionFilter.update(currentState.x, currentState.vx, dt);
  currentState.y_kalman = yPositionFilter.update(currentState.y, currentState.vy, dt);
  
  // 计算速度 (用于控制)
  if (dt > 0) {
    currentState.vx = globalDeltaX / dt;
    currentState.vy = globalDeltaY / dt; 
    currentState.omega = deltaHeading / dt;
  }
  
  // 保存当前值用于下一次计算
  lastOdom1 = currentOdom1;
  lastOdom2 = currentOdom2;
  lastIMUHeading = currentIMUHeading;
  lastUpdateTime = currentTime;
  
  // 显示位置信息 (包括卡尔曼滤波结果)
  Brain.Screen.setCursor(3, 1);
  Brain.Screen.print("Raw: X:%6.1f Y:%6.1f H:%5.1f", 
                     currentState.x, currentState.y, currentState.heading);
  Brain.Screen.setCursor(4, 1);
  Brain.Screen.print("Kalman: X:%6.1f Y:%6.1f H:%5.1f", 
                     currentState.x_kalman, currentState.y_kalman, currentState.heading_kalman);
}

// 设置目标位置
void setTarget(double targetX, double targetY, double targetHeading = -1) {
  targetState.x = targetX;
  targetState.y = targetY;
  if (targetHeading >= 0) {
    targetState.heading = targetHeading;
  }
}

// 计算到目标的距离
double getDistanceToTarget() {
  double dx = targetState.x - currentState.x;
  double dy = targetState.y - currentState.y;
  return sqrt(dx*dx + dy*dy);
}

// 计算到目标的方向角(度)
double getHeadingToTarget() {
  double dx = targetState.x - currentState.x;
  double dy = targetState.y - currentState.y;
  double targetHeading = atan2(dy, dx) * 180.0 / M_PI;
  
  // 标准化到0-360范围
  if (targetHeading < 0) targetHeading += 360;
  return targetHeading;
}

// 计算航向误差(处理360度跳变)
double getHeadingError() {
  double targetHeading = getHeadingToTarget();
  double error = targetHeading - currentState.heading;
  
  // 处理角度跳变
  if (error > 180) error -= 360;
  if (error < -180) error += 360;
  
  return error;
}

// 设置底盘速度 (线速度mm/s, 角速度度/s)
void setChassisSpeed(double linearSpeed, double angularSpeed) {
  // 将线速度和角速度转换为左右轮速度
  // 假设机器人的轮距为WHEELBASE (需要测量实际值)
  const double WHEELBASE = 300.0; // mm, 需要根据实际机器人调整
  
  // 差速驱动运动学模型
  double leftSpeed = linearSpeed - (angularSpeed * WHEELBASE / 2.0);
  double rightSpeed = linearSpeed + (angularSpeed * WHEELBASE / 2.0);
  
  // 将速度转换为电机百分比 (假设最大线速度对应100%电机速度)
  double leftPercent = (leftSpeed / MAX_LINEAR_SPEED) * 100.0;
  double rightPercent = (rightSpeed / MAX_LINEAR_SPEED) * 100.0;
  
  // 限制速度范围
  leftPercent = std::max(std::min(leftPercent, 100.0), -100.0);
  rightPercent = std::max(std::min(rightPercent, 100.0), -100.0);
  
  // 设置电机速度
  leftMotors.spin(forward, leftPercent, percent);
  rightMotors.spin(forward, rightPercent, percent);
}

// 停止底盘
void stopChassis() {
  leftMotors.stop();
  rightMotors.stop();
}

// 移动到目标位置的主函数
// 计算到目标的距离 (使用卡尔曼滤波后的位置)
double getDistanceToTarget() {
  double dx = targetState.x - currentState.x_kalman;
  double dy = targetState.y - currentState.y_kalman;
  return sqrt(dx*dx + dy*dy);
}

// 计算到目标的方向角(度) (使用卡尔曼滤波后的位置和航向)
double getHeadingToTarget() {
  double dx = targetState.x - currentState.x_kalman;
  double dy = targetState.y - currentState.y_kalman;
  double targetHeading = atan2(dy, dx) * 180.0 / M_PI;
  
  // 标准化到0-360范围
  if (targetHeading < 0) targetHeading += 360;
  return targetHeading;
}

// 计算航向误差(处理360度跳变) (使用卡尔曼滤波后的航向)
double getHeadingError() {
  double targetHeading = getHeadingToTarget();
  double error = targetHeading - currentState.heading_kalman;
  
  // 处理角度跳变
  if (error > 180) error -= 360;
  if (error < -180) error += 360;
  
  return error;
}
