#include "conctrl.h"

Motor_parameter MOTOR_A, MOTOR_B;
Encoder OriginalEncoder;
Smooth_Control smooth_control;

void Drive_Motor(float Vx, float Vz) {
  float amplitude = 1.0f; // Wheel target speed limit //车轮目标速度限幅

  // Inverse kinematics //运动学逆解
  MOTOR_A.Target = Vx - Vz * Diff_wheelSpacing / 2.0f; // 计算出左轮的目标速度
  MOTOR_B.Target = Vx + Vz * Diff_wheelSpacing / 2.0f; // 计算出右轮的目标速度

  // Wheel (motor) target speed limit //车轮(电机)目标速度限幅
  MOTOR_A.Target = target_limit_float(MOTOR_A.Target, -amplitude, amplitude);
  MOTOR_B.Target = target_limit_float(MOTOR_B.Target, -amplitude, amplitude);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

  if (htim->Instance == TIM2) {
    Count++;
    if (Count >= 20) {
      Count = 0;
    }
    Get_Angle();
    Get_Velocity_Form_Encoder();
    // 仅保留 APP 按键控制电机
    Get_RC();
    MOTOR_A.Motor_Pwm = Incremental_PI_A(MOTOR_A.Encoder, MOTOR_A.Target);
    MOTOR_B.Motor_Pwm = Incremental_PI_B(MOTOR_B.Encoder, MOTOR_B.Target);
    Set_Pwm(-MOTOR_A.Motor_Pwm, MOTOR_B.Motor_Pwm);
  }
}

void Set_Pwm(int motor_a, int motor_b) {
  // Forward and reverse control of motor
  // 电机正反转控制
  if (motor_a < 0)
    AIN1 = 1, AIN2 = 0, PWMA = -motor_a;
  else
    AIN1 = 0, AIN2 = 1, PWMA = motor_a;

  // Forward and reverse control of motor
  // 电机正反转控制
  if (motor_b < 0)
    BIN1 = 1, BIN2 = 0, PWMB = -motor_b;
  else
    BIN1 = 0, BIN2 = 1, PWMB = motor_b;
}

void Get_RC(void) {
  if (Flag_Direction == 0) {
    Move_X = 0, Move_Z = 0; // 停止
  } else if (Flag_Direction == 1) {
    Move_X = RC_Velocity, Move_Z = 0; // 前进
  } else if (Flag_Direction == 2) {
    Move_X = RC_Velocity, Move_Z = Pi / 2; // 右前
  } else if (Flag_Direction == 3) {
    Move_X = 0, Move_Z = Pi / 2; // 向右
  } else if (Flag_Direction == 4) {
    Move_X = -RC_Velocity, Move_Z = Pi / 2; // 右后
  } else if (Flag_Direction == 5) {
    Move_X = -RC_Velocity, Move_Z = 0; // 后退
  } else if (Flag_Direction == 6) {
    Move_X = -RC_Velocity, Move_Z = -Pi / 2; // 左后
  } else if (Flag_Direction == 7) {
    Move_X = 0, Move_Z = -Pi / 2; // 向左
  } else if (Flag_Direction == 8) {
    Move_X = RC_Velocity, Move_Z = -Pi / 2; // 左前
  } else {
    Move_X = 0, Move_Z = 0;
  };

  if (Move_X < 0)
    Move_Z = -Move_Z; // The differential control principle series requires this
                      // treatment //差速控制原理系列需要此处理
  Move_Z = Move_Z * RC_Velocity / 500;

  Move_X = Move_X / 1000;
  Move_Z = -Move_Z;
  Drive_Motor(Move_X, Move_Z);
}

void Lidar_Avoid_RC(void) {
  int i = 0;
  u8 calculation_angle_cnt = 0; // 用于判断100个点中需要做避障的点
  float angle_sum = 0;          // 粗略计算障碍物位于左或者右
  u8 distance_count = 0;        // 距离小于某值的计数
  int distance = 400;           // 设定避障距离,默认是300
  for (i = 0; i < 450; i++) {
    if ((Dataprocess[i].angle > 300) || (Dataprocess[i].angle < 60)) {
      if ((0 < Dataprocess[i].distance) &&
          (Dataprocess[i].distance <
           distance)) // 距离小于350mm需要避障,只需要100度范围内点
      {
        calculation_angle_cnt++; // 计算距离小于避障距离的点个数
        if (Dataprocess[i].angle < 60)
          angle_sum += Dataprocess[i].angle;
        else if (Dataprocess[i].angle > 300)
          angle_sum +=
              (Dataprocess[i].angle - 360); // 310度到50度转化为-50度到50度
        if (Dataprocess[i].distance < 300)  // 记录小于200mm的点的计数
          distance_count++;
      }
    }
  }
  if (calculation_angle_cnt < 8) // 小于8点不需要避障，去除一些噪点
  {
    if ((Move_X += 0.1) >=
        Aovid_Speed) // 避障的速度设定为260，逐渐增加到260可稍微平滑一些
      Move_X = Aovid_Speed;
    Move_Z = 0; // 不避障时不需要转弯
  } else        // 需要避障，简单地判断障碍物方位
  {
    if (distance_count > 8)              // 距离小于避战距离
      Move_X = -Aovid_Speed, Move_Z = 0; // 往后退
    else {
      if ((Move_X -= 0.1) <= (Aovid_Speed * 0.5)) // 避障时速度降到低速80
        Move_X = Aovid_Speed * 0.5;
      if (angle_sum > 0)  // 障碍物偏右
        Move_Z = -Pi / 5; // 每次转弯角度为PI/5，直到100度范围内无障碍物就停止
      else                // 偏左
        Move_Z = Pi / 5;
    }
  }
  Move_Z = -Move_Z;
  Drive_Motor(Move_X, Move_Z);
}

void Lidar_Along_RC(void) {
  static u32 target_distance = 0;
  static int n = 0;

  int i;
  u32 distance;

  for (i = 0; i < 450; i++) {
    if (Dataprocess[i].angle > Along_Angle &&
        Dataprocess[i].angle < (Along_Angle + 2)) {
      if (n == 0) {
        target_distance = Dataprocess[i].distance; // 获取的第一个点作为目标距离
        n++;
      }
      if (Dataprocess[i].distance <
          target_distance + 100) //+100限制获取距离的范围值
      {
        distance = Dataprocess[i].distance; // 获取实时距离
      }
    }
  }
  Move_X = forward_velocity; // 初始速度
  Move_Z = -Along_Adjust_PID(distance, target_distance);
  Move_Z = target_limit_float(Move_Z, -Pi / 4, Pi / 4); // 限幅

  Drive_Motor(Move_X, Move_Z);
}

void Lidar_Follow_RC(void) {
  static u16 cnt = 0;
  int i;
  int calculation_angle_cnt = 0;
  static float angle = 0;      // 跟随的角度
  static float last_angle = 0; //
  u16 mini_distance = 65535;
  static u8 data_count = 0; // 用于滤除一写噪点的计数变量
  // 需要找出跟随的那个点的角度
  for (i = 0; i < 450; i++) {
    if (100 < Dataprocess[i].distance &&
        Dataprocess[i].distance < Follow_Distance) // 1200范围内就需要跟随
    {
      calculation_angle_cnt++;
      if (Dataprocess[i].distance < mini_distance) // 找出距离最小的点
      {
        mini_distance = Dataprocess[i].distance;
        angle = Dataprocess[i].angle;
      }
    }
  }
  if (angle > 180)
    angle -= 360; // 0--360度转换成0--180；-180--0（顺时针）
  if (angle - last_angle > 10 ||
      angle - last_angle < -10) // 做一定消抖，波动大于10度的需要做判断
  {
    if (++data_count ==
        60) // 连续60次采集到的值(300ms后)和上次的比大于10度，此时才是认为是有效值
    {
      data_count = 0;
      last_angle = angle;
    }
  } else // 波动小于10度的可以直接认为是有效值
  {
    data_count = 0;
    last_angle = angle;
  }
  if (calculation_angle_cnt < 6) // 在跟随范围内的点少于6个
  {

    if (cnt < 40) // 连续计数超40次没有要跟随的点，此时才是不用跟随
      cnt++;
    if (cnt >= 40) {
      Move_X = 0; // 速度为0
      Move_Z = 0;
    }
  } else {
    cnt = 0;
    if ((((angle > 15) && (angle < 180)) || ((angle > -180) && angle < -15)) &&
        (mini_distance <
         500)) // 阿卡曼车型处理车头不对着跟随物，相当于倒车一样，一次不对准，那后退再来对准
    {
      Move_X = -0.20;
      Move_Z = -Follow_Turn_PID(last_angle, 0);
    } else {
      Move_X = Distance_Adjust_PID(mini_distance,
                                   Keep_Follow_Distance); // 保持距离保持在400mm
      Move_Z = Follow_Turn_PID(last_angle, 0);
    }
  }
  Move_Z = target_limit_float(Move_Z, -Pi / 6, Pi / 6); // 限幅
  Move_X = target_limit_float(Move_X, -0.4, 0.4);

  Drive_Motor(Move_X, Move_Z);
}

void Get_Velocity_Form_Encoder(void) {
  // Read raw encoder counts
  OriginalEncoder.A = Read_Encoder(5);
  OriginalEncoder.B = Read_Encoder(3);

  // Convert counts -> m/s
  // Use Encoder_precision macro (defined in sys.h) to keep consistency
  const float counts_to_mps =
      (float)CONTROL_FREQUENCY * Wheel_perimeter / Encoder_precision;
  float raw_a = -((float)OriginalEncoder.A) * counts_to_mps;
  float raw_b = ((float)OriginalEncoder.B) * counts_to_mps;

  // Clamp raw speeds to a reasonable physical maximum to avoid encoder glitches
  if (raw_a > ENCODER_MAX_SPEED_MPS)
    raw_a = ENCODER_MAX_SPEED_MPS;
  if (raw_a < -ENCODER_MAX_SPEED_MPS)
    raw_a = -ENCODER_MAX_SPEED_MPS;
  if (raw_b > ENCODER_MAX_SPEED_MPS)
    raw_b = ENCODER_MAX_SPEED_MPS;
  if (raw_b < -ENCODER_MAX_SPEED_MPS)
    raw_b = -ENCODER_MAX_SPEED_MPS;

  // Exponential moving average filter to smooth spikes
  const float alpha =
      ENCODER_FILTER_ALPHA; // smoothing factor (0..1). higher = less smoothing
  static float last_a = 0.0f;
  static float last_b = 0.0f;
  float filt_a = alpha * raw_a + (1.0f - alpha) * last_a;
  float filt_b = alpha * raw_b + (1.0f - alpha) * last_b;
  last_a = filt_a;
  last_b = filt_b;

  MOTOR_A.Encoder = filt_a;
  MOTOR_B.Encoder = filt_b;
}

/**************************************************************************
Function: Incremental PI controller
Input   : Encoder measured value (actual speed), target speed
Output  : Motor PWM
According to the incremental discrete PID formula
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k) represents the current deviation
e(k-1) is the last deviation and so on
PWM stands for incremental output
In our speed control closed loop system, only PI control is used
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)

函数功能：增量式PI控制器
入口参数：编码器测量值(实际速度)，目标速度
返回  值：电机PWM
根据增量式离散PID公式
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)代表本次偏差
e(k-1)代表上一次的偏差  以此类推
pwm代表增量输出
在我们的速度控制闭环系统里面，只使用PI控制
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)
**************************************************************************/
int Incremental_PI_A(float Encoder, float Target) {
  static float Bias, Pwm, Last_bias;
  if (Target == 0.0f && fabs(Encoder) < 1.0f) {
    Pwm = 0;
    Last_bias = 0;
    return 0;
  }
  Bias = Target - Encoder; // Calculate the deviation //计算偏差
  Pwm += Velocity_KP * (Bias - Last_bias) + Velocity_KI * Bias;
  if (Pwm > 7200)
    Pwm = 7200;
  if (Pwm < -7200)
    Pwm = -7200;
  Last_bias = Bias; // Save the last deviation //保存上一次偏差
  return Pwm;
}
int Incremental_PI_B(float Encoder, float Target) {
  static float Bias, Pwm, Last_bias;
  if (Target == 0.0f && fabs(Encoder) < 1.0f) {
    Pwm = 0;
    Last_bias = 0;
    return 0;
  }
  Bias = Target - Encoder;
  Pwm += Velocity_KP * (Bias - Last_bias) + Velocity_KI * Bias;
  // Pwm += Velocity_KP * (Bias - Last_bias);
  if (Pwm > 7200)
    Pwm = 7200;
  if (Pwm < -7200)
    Pwm = -7200;
  Last_bias = Bias; // Save the last deviation //保存上一次偏差
  return Pwm;
}

/**************************************************************************
Function: Distance_Adjust_PID
Input   : Current_Distance;Target_Distance
Output  : OutPut
函数功能：走直线雷达距离pid
入口参数: 当前距离和目标距离
返回  值：电机目标速度
**************************************************************************/
// 走直线雷达距离调整pid

float Along_Adjust_PID(float Current_Distance,
                       float Target_Distance) // 距离调整PID
{
  static float Bias, OutPut, Integral_bias, Last_Bias;
  Bias = Target_Distance - Current_Distance; // 计算偏差
  Integral_bias += Bias;                     // 求出偏差的积分
  if (Integral_bias > 1000)
    Integral_bias = 1000;
  else if (Integral_bias < -1000)
    Integral_bias = -1000;
  OutPut = -Akm_Along_Distance_KP * Bias -
           Akm_Along_Distance_KI * Integral_bias -
           Akm_Along_Distance_KD * (Bias - Last_Bias); // 位置式PID控制器
  Last_Bias = Bias;                                    // 保存上一次偏差
  if (MOTOR_A.Motor_Pwm == 0 &&
      MOTOR_B.Motor_Pwm == 0) // 电机关闭，此时积分清零
    Integral_bias = 0;
  return OutPut; // 输出
}

/**************************************************************************
Function: Follow_Turn_PID
Input   : Current_Angle;Target_Angle
Output  : OutPut
函数功能：雷达转向pid
入口参数: 当前角度和目标角度
返回  值：电机转向速度
**************************************************************************/
// 雷达转向pid
float Follow_Turn_PID(float Current_Angle, float Target_Angle) {
  static float Bias, OutPut, Integral_bias, Last_Bias;
  Bias = Target_Angle - Current_Angle; // 计算偏差
  Integral_bias += Bias;               // 求出偏差的积分
  if (Integral_bias > 1000)
    Integral_bias = 1000;
  else if (Integral_bias < -1000)
    Integral_bias = -1000;
  OutPut = Follow_KP_Akm * Bias + Follow_KI_Akm * Integral_bias +
           Follow_KD_Akm * (Bias - Last_Bias); // 位置式PID控制器
  Last_Bias = Bias;                            // 保存上一次偏差
  if (MOTOR_A.Motor_Pwm == 0 &&
      MOTOR_B.Motor_Pwm == 0) // 电机关闭，此时积分清零
    Integral_bias = 0;
  return OutPut; // 输出
}

/**************************************************************************
Function: Distance_Adjust_PID
Input   : Current_Distance;Target_Distance
Output  : OutPut
函数功能：雷达转向pid
入口参数: 当前距离和目标距离
返回  值：电机目标速度
**************************************************************************/
// 雷达距离调整pid
float Distance_Adjust_PID(float Current_Distance,
                          float Target_Distance) // 距离调整PID
{
  static float Bias, OutPut, Integral_bias, Last_Bias;
  Bias = Target_Distance - Current_Distance; // 计算偏差
  Integral_bias += Bias;                     // 求出偏差的积分
  if (Integral_bias > 1000)
    Integral_bias = 1000.0;
  else if (Integral_bias < -1000)
    Integral_bias = -1000.0;
  OutPut = -Distance_KP * Bias - Distance_KI * Integral_bias -
           Distance_KD * (Bias - Last_Bias); // 位置式PID控制器
  Last_Bias = Bias;                          // 保存上一次偏差
  if (MOTOR_A.Motor_Pwm == 0 &&
      MOTOR_B.Motor_Pwm == 0) // 电机关闭，此时积分清零
    Integral_bias = 0;
  return OutPut;
}

/**************************************************************************
Function: Check the battery voltage, enable switch status, software failure flag
status Input   : Voltage Output  : Whether control is allowed, 1: not allowed, 0
allowed 函数功能：检查电池电压、使能开关状态、软件失能标志位状态 入口参数：电压
返回  值：是否允许控制，1：不允许，0允许
**************************************************************************/
u8 Turn_Off(int voltage) {
  u8 temp;
  if (voltage < 1000)
    temp = 1;
  else
    temp = 0;
  return temp;
}

/**************************************************************************
Function: Limiting function
Input   : Value
Output  : none
函数功能：限幅函数
入口参数：幅值
返回  值：无
**************************************************************************/
float target_limit_float(float insert, float low, float high) {
  if (insert < low)
    return low;
  else if (insert > high)
    return high;
  else
    return insert;
}
int target_limit_int(int insert, int low, int high) {
  if (insert < low)
    return low;
  else if (insert > high)
    return high;
  else
    return insert;
}

/**************************************************************************
Function: Calculates the check bits of data to be sent/received
Input   : Count_Number: The first few digits of a check; Mode: 0-Verify the
received data, 1-Validate the sent data Output  : Check result
函数功能：计算要发送/接收的数据校验结果
入口参数：Count_Number：校验的前几位数；Mode：0-对接收数据进行校验，1-对发送数据进行校验
返回  值：校验结果
**************************************************************************/
u8 Check_Sum(unsigned char Count_Number, unsigned char Mode) {
  unsigned char check_sum = 0, k;

  // Validate the data to be sent
  // 对要发送的数据进行校验
  if (Mode == 1)
    for (k = 0; k < Count_Number; k++) {
      check_sum = check_sum ^ Send_Data.buffer[k];
    }

  // Verify the data received
  // 对接收到的数据进行校验
  if (Mode == 0)
    for (k = 0; k < Count_Number; k++) {
      check_sum = check_sum ^ Receive_Data[k];
    }
  return check_sum;
}

/**************************************************************************
Function: The data sent by the serial port is assigned
Input   : none
Output  : none
函数功能：串口发送的数据进行赋值
入口参数：无
返回  值：无
**************************************************************************/
void data_transition(void) {
  Send_Data.Sensor_Str.Frame_Header = FRAME_HEADER; // Frame_header //帧头
  Send_Data.Sensor_Str.Frame_Tail = FRAME_TAIL;     // Frame_tail //帧尾

  Send_Data.Sensor_Str.X_speed =
      ((MOTOR_A.Encoder + MOTOR_B.Encoder) / 4) * 1000;
  Send_Data.Sensor_Str.Y_speed =
      ((MOTOR_A.Encoder - MOTOR_B.Encoder) / 4) * 1000;
  //	Send_Data.Sensor_Str.Z_speed =
  //((-MOTOR_A.Encoder+MOTOR_B.Encoder)/4/(Wheelspacing))*1000;

  // The acceleration of the triaxial acceleration //加速度计三轴加速度
  Send_Data.Sensor_Str.Accelerometer.X_data =
      0; // The accelerometer Y-axis is converted to the ros coordinate X axis

  Send_Data.Sensor_Str.Accelerometer.Y_data =
      0; // The accelerometer X-axis is converted to the ros coordinate y axis
  Send_Data.Sensor_Str.Accelerometer.Z_data =
      0; // The accelerometer Z-axis is converted to the ros coordinate Z axis

  // The Angle velocity of the triaxial velocity //角速度计三轴角速度
  Send_Data.Sensor_Str.Gyroscope.X_data =
      0; // The Y-axis is converted to the ros coordinate X axis
  Send_Data.Sensor_Str.Gyroscope.Y_data =
      0; // The Y-axis is converted to the ros coordinate Y axis
  Send_Data.Sensor_Str.Gyroscope.Z_data =
      0; // The Z-axis is converted to the ros coordinate Z axis

  // Battery voltage (this is a thousand times larger floating point number,
  // which will be reduced by a thousand times as well as receiving the data).
  // 电池电压(这里将浮点数放大一千倍传输，相应的在接收端在接收到数据后也会缩小一千倍)
  Send_Data.Sensor_Str.Power_Voltage = voltage * 10;

  Send_Data.buffer[0] = Send_Data.Sensor_Str.Frame_Header; // Frame_heade //帧头
  Send_Data.buffer[1] =
      0; // This is reserved, can be expanded  //该为做保留，可自行拓展

  // The three-axis speed of / / car is split into two eight digit Numbers
  // 小车三轴速度,各轴都拆分为两个8位数据再发送
  Send_Data.buffer[2] = Send_Data.Sensor_Str.X_speed >> 8;
  Send_Data.buffer[3] = Send_Data.Sensor_Str.X_speed;
  Send_Data.buffer[4] = Send_Data.Sensor_Str.Y_speed >> 8;
  Send_Data.buffer[5] = Send_Data.Sensor_Str.Y_speed;
  Send_Data.buffer[6] = Send_Data.Sensor_Str.Z_speed >> 8;
  Send_Data.buffer[7] = Send_Data.Sensor_Str.Z_speed;

  // The acceleration of the triaxial axis of / / imu accelerometer is divided
  // into two eight digit reams
  // IMU加速度计三轴加速度,各轴都拆分为两个8位数据再发送
  Send_Data.buffer[8] = Send_Data.Sensor_Str.Accelerometer.X_data >> 8;
  Send_Data.buffer[9] = Send_Data.Sensor_Str.Accelerometer.X_data;
  Send_Data.buffer[10] = Send_Data.Sensor_Str.Accelerometer.Y_data >> 8;
  Send_Data.buffer[11] = Send_Data.Sensor_Str.Accelerometer.Y_data;
  Send_Data.buffer[12] = Send_Data.Sensor_Str.Accelerometer.Z_data >> 8;
  Send_Data.buffer[13] = Send_Data.Sensor_Str.Accelerometer.Z_data;

  // The axis of the triaxial velocity of the / /imu is divided into two eight
  // digits IMU角速度计三轴角速度,各轴都拆分为两个8位数据再发送
  Send_Data.buffer[14] = Send_Data.Sensor_Str.Gyroscope.X_data >> 8;
  Send_Data.buffer[15] = Send_Data.Sensor_Str.Gyroscope.X_data;
  Send_Data.buffer[16] = Send_Data.Sensor_Str.Gyroscope.Y_data >> 8;
  Send_Data.buffer[17] = Send_Data.Sensor_Str.Gyroscope.Y_data;
  Send_Data.buffer[18] = Send_Data.Sensor_Str.Gyroscope.Z_data >> 8;
  Send_Data.buffer[19] = Send_Data.Sensor_Str.Gyroscope.Z_data;

  // Battery voltage, split into two 8 digit Numbers
  // 电池电压,拆分为两个8位数据发送
  Send_Data.buffer[20] = Send_Data.Sensor_Str.Power_Voltage >> 8;
  Send_Data.buffer[21] = Send_Data.Sensor_Str.Power_Voltage;

  // Data check digit calculation, Pattern 1 is a data check
  // 数据校验位计算，模式1是发送数据校验
  Send_Data.buffer[22] = Check_Sum(22, 1);

  Send_Data.buffer[23] = Send_Data.Sensor_Str.Frame_Tail; // Frame_tail //帧尾
}

/**************************************************************************
Function: Get Angle function
Input   : none
Output  : none
函数功能：获取角度信息
入口参数：无
返回  值：无
**************************************************************************/
void Get_Angle(void) {
  imu->Update_9axisVal(&axis_9Val);             // 陀螺仪数据更新
  imu->UpdateAttitude(axis_9Val, &AttitudeVal); // 更新姿态角
  //*57.2958是将弧度转化为角度
  Roll = (AttitudeVal.roll * 57.2958);
  Pitch = (AttitudeVal.pitch * 57.2958);
  Yaw = AttitudeVal.yaw * 57.2958;
}
