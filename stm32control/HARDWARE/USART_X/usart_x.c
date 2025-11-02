#include "usart_x.h"

SEND_DATA Send_Data;

PointDataProcessDef Dataprocess[1000]; // 更新50个数据
LiDARFrameTypeDef Pack_Data;

void Data_Send(void) {
  unsigned char i = 0;

  for (i = 0; i < 24; i++) {
    usart1_send(Send_Data.buffer[i]);
  }
}

/**************************************************************************
Function: Serial port 1 sends data
Input   : Data to be sent
Output  : none
函数功能：串口1发送数据
入口参数：要发送的数据
返回  值：无
**************************************************************************/
void usart1_send(u8 data) {
  USART3->DR = data;
  while ((USART3->SR & 0x40) == 0)
    ;
}

/**************************************************************************
Function: Serial port 2 sends data
Input   : Data to be sent
Output  : none
函数功能：串口1发送数据
入口参数：要发送的数据
返回  值：无
**************************************************************************/
void usart2_send(u8 data) {
  USART2->DR = data;
  while ((USART2->SR & 0x40) == 0)
    ;
}

/**************************************************************************
Function: data_process
Input   : none
Output  : none
函数功能：数据处理函数
入口参数：无
返回  值：无
**************************************************************************/
void data_process(void) // 数据处理函数
{
  static u16 data_cnt = 0;
  u8 i;
  //	u32 distance_sum = 0;
  u16 average_distance = 0;
  float start_angle =
      (((u16)Pack_Data.start_angle_h << 8) + Pack_Data.start_angle_l) /
      100.0; // 计算16个点的开始角度，数据传输时放大100倍
  float end_angel =
      (((u16)Pack_Data.end_angle_h << 8) + Pack_Data.end_angle_l) / 100.0;
  float area_angel; // 暂存每个点的角度
  if (start_angle > end_angel)
    end_angel += 360; // 开始与结束的角度被0度分开的情况

  for (i = 0; i < 16; i++) // 处理当前帧数据的16个点
  {
    area_angel =
        start_angle + (end_angel - start_angle) / 16 * i; // 获取当前点的角度
    if (area_angel >= 360)
      area_angel -= 360; // 当前角度超过360度

    average_distance = ((u16)Pack_Data.point[i].distance_h << 8) +
                       Pack_Data.point[i].distance_l; // 当前点离周边物体的距离
    // 将数据保存到结构体中
    Dataprocess[data_cnt].angle = area_angel;
    Dataprocess[data_cnt++].distance = average_distance;

    if (data_cnt == 450)
      data_cnt = 0;
  }
  // 成功接收雷达数据标志
  Lidar_Success_Receive_flag = 1;
}

///**************************************************************************
// Function: Calculates the check bits of data to be sent/received
// Input   : Count_Number: The first few digits of a check; Mode: 0-Verify the
// received data, 1-Validate the sent data Output  : Check result
// 函数功能：计算要发送/接收的数据校验结果
// 入口参数：Count_Number：校验的前几位数；Mode：0-对接收数据进行校验，1-对发送数据进行校验
// 返回  值：校验结果
//**************************************************************************/
// u8 Check_Sum(unsigned char Count_Number,unsigned char Mode)
//{
//	unsigned char check_sum=0,k;
//
//	//Validate the data to be sent
//	//对要发送的数据进行校验
//	if(Mode==1)
//	for(k=0;k<Count_Number;k++)
//	{
//		check_sum=check_sum^Send_Data.buffer[k];
//	}
//
//	//Verify the data received
//	//对接收到的数据进行校验
//	if(Mode==0)
//	for(k=0;k<Count_Number;k++)
//	{
//	check_sum=check_sum^Receive_Data[k];
//	}
//	return check_sum;
// }

/**************************************************************************
Function: After the top 8 and low 8 figures are integrated into a short type
data, the unit reduction is converted Input   : 8 bits high, 8 bits low Output
: The target velocity of the robot on the X/Y/Z axis
函数功能：将上位机发过来的高8位和低8位数据整合成一个short型数据后，再做单位还原换算
入口参数：高8位，低8位
返回  值：机器人X/Y/Z轴的目标速度
**************************************************************************/
float XYZ_Target_Speed_transition(u8 High, u8 Low) {
  // Data conversion intermediate variable
  // 数据转换的中间变量
  short transition;

  // 将高8位和低8位整合成一个16位的short型数据
  // The high 8 and low 8 bits are integrated into a 16-bit short data
  transition = ((High << 8) + Low);
  return transition / 1000 +
         (transition % 1000) *
             0.001; // Unit conversion, mm/s->m/s //单位转换, mm/s->m/s
}
