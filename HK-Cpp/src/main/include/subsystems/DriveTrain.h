#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DigitalInput.h>
#include <frc/DigitalOutput.h>
#include "Constants.h"

#include "studica/TitanQuad.h"
#include "studica/Lidar.h"
#include "AHRS.h"
#include <math.h>

#include "functions/DataContainer.h"

/**
 *  DriveBase
 * 
 *  X-Drive          0
 *  TwoWheel_Drive   1
 *  TheStack_Drive   2
 *  Mecanum_Drive    3
 *  SixWHeel_Drive   4
 */
#define DRIVEBASE 2

namespace WSHK
{

namespace _2024
{

class DriveTrain : public frc2::SubsystemBase
{
public:
   DriveTrain();
   void Periodic() override;
   double GetYaw(void);
   double GetAngle(void);
   bool GetStartButton(void);
   bool GetEStopButton(void);
   void SetRunningLED(bool on);
   void SetStoppedLED(bool on);
   void ResetYaw(void);
   void LidarStop(void);
   void LidarStart(void);
   void StackMotorControl(double x, double y);
   void TwoWheelMotorControl(double x, double y);
   void SixWheelMotorControl(double x, double y);
   void MecanumMotorControl(double x, double y, double z);
   void XBotMotorControl(double x, double y, double z);
   // Lidar
   bool lidarRunning = true;

   /*******新添加的代码********/
   struct Acoord
   {
      float x[360];
      float y[360];
   };
   Acoord mcoord;

   frc2::Timer sytemTime; //调用系统时间

   double mtimeraw = 0.0;  //初始时间
   double mtimedata = 0.0; //传递时间
   double mtimeNow = 0.0;  //实时时间

   //机器人底盘类型
   enum RobotDrive
   {
      X_Drive = 0,
      TwoWheel_Drive = 1,
      TheStack_Drive = 2,
      Mecanum_Drive = 3,
      SixWHeel_Drive = 4
   };

   /** 雷达角度調整
 * 
 *              :            :            :            :      
 *     225°     :   247.5°   :    270°    :   292.5°   :    315°     
 *              :            :            :            :  
 *              :            :            :            :      
 *  ................................................................
 *              :            :            :            :      
 *    202.5°    :            :            :            :   337.5°    
 *              :            :            :            :  
 *              :            :            :            :      
 *  ................................................................
 *              :            :            :            :       
 *     180°     :            :            :            :     0°      
 *      銀色    :            :            :            :    銀色   
 *    相反方向   :            :            :            :      
 *  ................................................................
 *              :            :            :            :      
 *    157.5°    :            :            :            :    22.5°    
 *              :            :            :            :    
 *              :            :            :            :      
 *  ................................................................
 *              :            :            :            :      
 *     135°     :   112.5°   :    90°     :    67.5°   :     45°     
 *              :            :            :            :  
 *              :            :            :            :      
 * 
 * 
 *  liDAR_right_ang
 *  雷达右方角度, 銀色突起部分為0度
 */
   int liDAR_right_ang = 5; //雷达右方角度, 如果銀色在正右方, 此處為 int liDAR_right_ang = 0;

#if DRIVEBASE == 0

   /********设置机器人类型*******/
   RobotDrive mrobot = X_Drive;

   /********设置避障参数*********/

   // X_Drive
   static constexpr int Lamend_times = 15;   //左边障碍，向右贴墙避障修正次数
   static constexpr int Ramend_times = 12;   //右边障碍，向左贴墙避障修正次数
   static constexpr int WallHigh = 290;      //贴边距离范围，最大值；单位毫米（mm）
   static constexpr int WallLow = 220;       //贴边距离范围，最小值；单位毫米（mm）
   static constexpr double Rate_left = 90.0; //机器人旋转角度
   static constexpr double walktimes = 2.5;  //机器人行进时间

   //   設置速度

   //   forwardSpeed
   //   前進速度
   double forwardSpeed = 0.18; // X_Drive

   //   turnLeftSpeed
   //   左转速度
   double turnLeftSpeed = -0.23; // X_Drive
   double turnLeftSpeedY = 0;    // X_Drive

   //   turnRightSpeed
   //   右转速度
   double turnRightSpeed = 0.23; // X_Drive
   double turnRightSpeedY = 0;   // X_Drive

   //   backSpeed
   //   后退速度
   double bacekSped = -0.15; // X_Drive
#elif DRIVEBASE == 1

   /********设置机器人类型*******/
   RobotDrive mrobot = TwoWheel_Drive;

   /********设置避障参数*********/

   // TwoWheel_Drive
   static constexpr int Lamend_times = 20;   //左边障碍，向右贴墙避障修正次数
   static constexpr int Ramend_times = 10;   //右边障碍，向左贴墙避障修正次数
   static constexpr int WallHigh = 270;      //贴边距离范围，最大值；单位毫米（mm）
   static constexpr int WallLow = 180;       //贴边距离范围，最小值；单位毫米（mm）
   static constexpr double Rate_left = 90.0; //机器人旋转角度
   static constexpr double walktimes = 1.75; //机器人行进时间

   //   設置速度

   //   forwardSpeed
   //   前進速度
   double forwardSpeed = 0.26; // TwoWheel_Drive

   //   turnLeftSpeed
   //   左转速度
   double turnLeftSpeed = -0.15; // TwoWheel_Drive
   double turnLeftSpeedY = 0;    // TwoWheel_Drive

   //   turnRightSpeed
   //   右转速度
   double turnRightSpeed = 0.15; // TwoWheel_Drive
   double turnRightSpeedY = 0;   // TwoWheel_Drive

   //   backSpeed
   //   后退速度
   double backSpeed = -0.23;            // TwoWheel_Drive
#elif DRIVEBASE == 2

   /********设置机器人类型*******/
   RobotDrive mrobot = TheStack_Drive;

   /********设置避障参数*********/

   // TheStack_Drive
   static constexpr int Lamend_times = 5;    //左边障碍，向右贴墙避障修正次数
   static constexpr int Ramend_times = 8;    //右边障碍，向左贴墙避障修正次数
   static constexpr int WallHigh = 290;       //贴边距离范围，最大值；单位毫米（mm）
   static constexpr int WallLow = 230;        //贴边距离范围，最小值；单位毫米（mm）
   static constexpr double Rate_left = 105.0; //机器人旋转角度
   static constexpr double walktimes = 2.5;   //机器人行进时间

   //   設置速度

   //   forwardSpeed
   //   前進速度
   double forwardSpeed = 0.25; // TheStack_Drive

   //   turnLeftSpeed
   //   左转速度
   double turnLeftSpeed = -0.22; // TheStack_Drive
   double turnLeftSpeedY = 0;    // TheStack_Drive

   //   turnRightSpeed
   //   右转速度
   double turnRightSpeed = 0.22; // TheStack_Drive
   double turnRightSpeedY = 0;   // TheStack_Drive

   //   backSpeed
   //   后退速度
   double backSpeed = -0.15;            // TheStack_Drive

#elif DRIVEBASE == 3

   /********设置机器人类型*******/
   RobotDrive mrobot = Mecanum_Drive;

   /********设置避障参数*********/

   // Mecanum_Drive
   static constexpr int Lamend_times = 15;   //左边障碍，向右贴墙避障修正次数
   static constexpr int Ramend_times = 10;   //右边障碍，向左贴墙避障修正次数
   static constexpr int WallHigh = 300;      //贴边距离范围，最大值；单位毫米（mm）
   static constexpr int WallLow = 210;       //贴边距离范围，最小值；单位毫米（mm）
   static constexpr double Rate_left = 90.0; //机器人旋转角度
   static constexpr double walktimes = 1.5;  //机器人行进时间

   //   設置速度

   //   forwardSpeed
   //   前進速度
   double forwardSpeed = 0.20; // Mecanum_Drive

   //   turnLeftSpeed
   //   左转速度
   double turnLeftSpeed = -0.25; // Mecanum_Drive
   double turnLeftSpeedY = 0;    // Mecanum_Drive

   //   turnRightSpeed
   //   右转速度
   double turnRightSpeed = 0.25; // Mecanum_Drive
   double turnRightSpeedY = 0;   // Mecanum_Drive

   //   backSpeed
   //   后退速度
   double backSpeed = -0.17;            // Mecanum_Drive

#elif DRIVEBASE == 4

   /********设置机器人类型*******/
   RobotDrive mrobot = SixWHeel_Drive;

   /********设置避障参数*********/

   // SixWHeel_Drive
   static constexpr int Lamend_times = 15;   //左边障碍，向右贴墙避障修正次数
   static constexpr int Ramend_times = 12;   //右边障碍，向左贴墙避障修正次数
   static constexpr int WallHigh = 300;      //贴边距离范围，最大值；单位毫米（mm）
   static constexpr int WallLow = 230;       //贴边距离范围，最小值；单位毫米（mm）
   static constexpr double Rate_left = 90.0; //机器人旋转角度
   static constexpr double walktimes = 0.92; //机器人行进时间

   //   設置速度

   //   forwardSpeed
   //   前進速度
   double forwardSpeed = 0.25; // SixWHeel_Drive

   //   turnLeftSpeed
   //   左转速度
   double turnLeftSpeed = -0.20; // SixWHeel_Drive
   double turnLeftSpeedY = 0;    // SixWHeel_Drive

   //   turnRightSpeed
   //   右转速度
   double turnRightSpeed = 0.20;   // SixWHeel_Drive
   double turnRightSpeedY = -0.05; // SixWHeel_Drive

   //   backSpeed
   //   后退速度
   double backSpeed = -0.22;            // SixWHeel_Drive

#else

   /********设置机器人类型*******/
   RobotDrive mrobot = X_Drive;

   /********设置避障参数*********/

   // X_Drive
   static constexpr int Lamend_times = 15;   //左边障碍，向右贴墙避障修正次数
   static constexpr int Ramend_times = 12;   //右边障碍，向左贴墙避障修正次数
   static constexpr int WallHigh = 290;      //贴边距离范围，最大值；单位毫米（mm）
   static constexpr int WallLow = 220;       //贴边距离范围，最小值；单位毫米（mm）
   static constexpr double Rate_left = 90.0; //机器人旋转角度
   static constexpr double walktimes = 2.5;  //机器人行进时间

   //   設置速度

   //   forwardSpeed
   //   前進速度
   double forwardSpeed = 0.18; // X_Drive

   //   turnLeftSpeed
   //   左转速度
   double turnLeftSpeed = -0.23; // X_Drive
   double turnLeftSpeedY = 0;    // X_Drive

   //   turnRightSpeed
   //   右转速度
   double turnRightSpeed = 0.23; // X_Drive
   double turnRightSpeedY = 0;   // X_Drive

   //   backSpeed
   //   后退速度
   double backSpeed = -0.15;            // X_Drive

#endif

   /**
 * 雷达检测角度&距離調整
 * 
 * 
 *              :            :            :            :      
 *     225°     :   247.5°   :    270°    :   292.5°   :    315°     
 *      NW      :    NW-N    :     N      :    N-NE    :     NE     
 *              :            :            :            :      
 *  ................................................................
 *              :            :            :            :      
 *    202.5°    :            :            :            :   337.5°    
 *     W-NW     :            :            :            :    NE-E   
 *              :            :            :            :      
 *  ................................................................
 *              :            :            :            :       
 *     180°     :            :            :            :     0°      
 *      W       :            :            :            :     E   
 *              :            :            :            :      
 *  ................................................................
 *              :            :            :            :      
 *    157.5°    :            :            :            :    22.5°    
 *     SW-W     :            :            :            :    E-SE    
 *              :            :            :            :      
 *  ................................................................
 *              :            :            :            :      
 *     135°     :   112.5°   :    90°     :    67.5°   :     45°     
 *      SW      :    S-SW    :     S      :    SE-S    :     SE    
 *              :            :            :            :      
 * 
 * 检测左前方及右前方障碍, 透過在startAngle 和 endAngle 之間的雷達數據
 * obs_dis < distance = 有障礙
 * 
 * obs_left_front  左上角上段
 * obs_left_middle 左上角中段
 * obs_left_back   左上角下段
 * 
 * int obs_left_front[2]  = {startAngle, endAngle};
 * int obs_left_middle[2] = {startAngle, endAngle};
 * int obs_left_back[2]   = {startAngle, endAngle};
 * 
 * obs_right_front  右上角上段
 * obs_right_middle 右上角中段
 * obs_right_back   右上角下段
 * 
 * int obs_right_front[2]  = {startAngle, endAngle};
 * int obs_right_middle[2] = {startAngle, endAngle};
 * int obs_right_back[2]   = {startAngle, endAngle};
 * 
 * int obs_dis = 200; // 左前方/右前方障碍 有障礙的雷達數據
 * 
 * 检测前方障碍
 * 
 * obs_dis_NW_NE
 * 检测前方障碍, 透過在startAngle 和 endAngle 之間的雷達數據
 * 雷達數據 < distance = 有障礙
 * 
 * int obs_dis_NW_NE[3] = {startAngle, endAngle, distance};
 * 
 * 检测左上方附近及右上方附近障碍(远端) 
 * 
 * obs_dis_NW
 * 檢測左上方附近障礙物, 透過在startAngle 和 endAngle 之間的雷達數據
 * 如果有 count 數量的雷達數據 < distance = 有障礙
 * int obs_dis_NW[4] = {startAngle, endAngle, distance, count};
 * 
 * obs_dis_NE
 * 檢測右上方附近障礙物, 透過在startAngle 和 endAngle 之間的雷達數據
 * 如果有 count 數量的雷達數據 < distance = 有障礙
 * int obs_dis_NE[4] = {startAngle, endAngle, distance, count};
 * 
 * 检测正前方範圍距离及正左方範圍距离
 * 
 * left_range
 * 正左方範圍
 * 
 * float left_range[2] = {startAngle, endAngle};
 * 
 * front_range
 * 检测正前方區域是否能移動, 透過在startAngle 和 endAngle 之間的雷達數據
 * 雷達數據 > distance = 可移動
 * 
 * float front_range[3] = {startAngle, endAngle, distance};
 * 
 * obs_upper_left
 * 检测左前方區域是否有障碍
 * 
 * int obs_upper_left[4] = {startAngle, endAngle, distance, count};
 * 
 * obs_upper
 * 检测左前方至前方區域是否有障碍
 * 
 * int obs_upper[3] = {startAngle, endAngle, distance};
 * 
 * b_proportion
 * 有障礙的前方空間雷達數據 對於 前方空間雷達數據 的比例
 * 
 * 检测左方及右方障碍(远端) 
 * 
 * obs_dis_NW_W
 * 檢測左方至左上方障礙物, 透過在startAngle 和 endAngle 之間的雷達數據
 * 如果有 count 數量的雷達數據 < distance = 有障礙
 * int obs_dis_NW_W[4] = {startAngle, endAngle, distance, count};
 * 
 * obs_dis_NE_E
 * 檢測右方至右上方障礙物, 透過在startAngle 和 endAngle 之間的雷達數據
 * 如果有 count 數量的雷達數據 < distance = 有障礙
 * int obs_dis_NE_E[4] = {startAngle, endAngle, distance, count};
 * 
 * 检测近距离障碍
 * 
 * obs_dis_SW_W
 * 檢測左方至左下方障礙物, 透過在startAngle 和 endAngle 之間的雷達數據
 * 每個雷達數據 < distance, 計數器 +1
 * 如果計數器 > count 數量 = 有障礙
 * int obs_dis_SW_W[4] = {startAngle, endAngle, distance, count};
 * 
  * obs_dis_SE_E
 * 檢測左方至左上方障礙物, 透過在startAngle 和 endAngle 之間的雷達數據
 * 每個雷達數據 < distance, 計數器 +1
 * 如果計數器 > count 數量 = 有障礙
 * int obs_dis_SE_E[4] = {startAngle, endAngle, distance, count};
 * 
 * obs_dis_NE_E_2
 * 檢測右方至右上方障礙物, 透過在startAngle 和 endAngle 之間的雷達數據
 * 每個雷達數據 < distance, 計數器 +1
 * 如果計數器 > count 數量 = 有障礙
 * int obs_dis_NE_E_2[4] = {startAngle, endAngle, distance, count};
 * 
 * 左侧区域检测通道口
 * 定義是否能向左侧旋轉90度
 * 
 * wide_left_no_wall
 * 檢測較大範圍左前方空間, 透過在startAngle 和 endAngle 之間的雷達數據
 * 每個雷達數據 > distance = 該角度前方沒障礙
 * 
 * int wide_left_no_wall[3] = {startAngle, endAngle, distance};
 * 
 * m_proportion
 * 前方沒障礙的左前方空間雷達數據 對於 左前方空間雷達數據 的比例
 * 
 * upper_left_range
 * 左前方範圍
 * 每個雷達數據 > distance = 該角度前方沒障礙
 * 
 * int upper_left_range[3] = {startAngle, endAngle, distance};
 * 
 * narrow_left_no_wall
 * 檢測較小範圍左前方空間, 透過在startAngle 和 endAngle 之間的雷達數據
 * 每個雷達數據 > distance = 該角度前方沒障礙
 * 
 * int narrow_left_no_wall[3] = {startAngle, endAngle, distance};
 */

#if DRIVEBASE == 0
   int obs_left_front[2] = {225, 230};  // Others
   int obs_left_middle[2] = {219, 224}; // Others
   int obs_left_back[2] = {213, 218};   // Others

   int obs_right_front[2] = {310, 315};  // All
   int obs_right_middle[2] = {316, 321}; // All
   int obs_right_back[2] = {322, 327};   // All

   int obs_dis = 200; // 左前方/右前方障碍 有障礙的雷達數據

   int obs_dis_NW_NE[3] = {235, 315, 280}; // X_Drive

   int obs_dis_NW[4] = {205, 240, 230, 3}; // X_Drive

   int obs_dis_NE[4] = {300, 340, 220, 3}; // All

   float left_range[2] = {185, 192};       // X_Drive
   float front_range[3] = {267, 273, 500}; // X_Drive

   int obs_upper_left[4] = {205, 228, 230, 5};
   int obs_upper[3] = {210, 280, 300};
   double b_proportion = 0.5;

   int obs_dis_NW_W[4] = {185, 225, 220, 5};     // X_Drive
   int obs_dis_NE_E[4] = {315, 335, 235, 5};     // X_Drive
   int obs_dis_SW_W[4] = {160, 180, 220, 2};     // X_Drive
   int obs_dis_SE_E[4] = {181, 215, 220, 2};     // X_Drive
   int obs_dis_NE_E_2[4] = {300, 335, 200, 3};   // X_Drive
   int wide_left_no_wall[3] = {190, 235, 550};   // X_Drive
   double m_proportion = 0.85;                   // X_Drive
   int upper_left_range[3] = {190, 225, 500};    // X_Drive
   int narrow_left_no_wall[3] = {190, 225, 550}; // X_Drive

#elif DRIVEBASE == 1
   int obs_left_front[2] = {225, 230};  // Others
   int obs_left_middle[2] = {219, 224}; // Others
   int obs_left_back[2] = {213, 218};   // Others

   int obs_right_front[2] = {310, 315};  // All
   int obs_right_middle[2] = {316, 321}; // All
   int obs_right_back[2] = {322, 327};   // All

   int obs_dis = 200; // 左前方/右前方障碍 有障礙的雷達數據

   int obs_dis_NW_NE[3] = {240, 305, 320}; // TwoWheel_Drive

   int obs_dis_NW[4] = {205, 240, 230, 3}; // TwoWheel_Drive

   int obs_dis_NE[4] = {300, 340, 220, 3}; // All

   float left_range[2] = {185, 192};       // TwoWheel_Drive
   float front_range[3] = {267, 273, 500}; // TwoWheel_Drive

   int obs_upper_left[4] = {205, 228, 230, 5};
   int obs_upper[3] = {210, 280, 300};
   double b_proportion = 0.5;

   int obs_dis_NW_W[4] = {200, 235, 220, 5}; // TwoWheel_Drive

   int obs_dis_NE_E[4] = {315, 335, 235, 5}; // TwoWheel_Drive
   int obs_dis_SW_W[4] = {170, 180, 220, 2}; // TwoWheel_Drive

   int obs_dis_SE_E[4] = {181, 235, 220, 2};     // TwoWheel_Drive
   int obs_dis_NE_E_2[4] = {300, 335, 220, 3};   // TwoWheel_Drive
   int wide_left_no_wall[3] = {190, 245, 550};   // TwoWheel_Drive
   double m_proportion = 0.80;                   // TwoWheel_Drive
   int upper_left_range[3] = {210, 250, 500};    // TwoWheel_Drive
   int narrow_left_no_wall[3] = {190, 225, 500}; // TwoWheel_Drive

#elif DRIVEBASE == 2
   int obs_left_front[2] = {225, 230};  // Others
   int obs_left_middle[2] = {219, 224}; // Others
   int obs_left_back[2] = {213, 218};   // Others

   int obs_right_front[2] = {310, 315};  // All
   int obs_right_middle[2] = {316, 321}; // All
   int obs_right_back[2] = {322, 327};   // All

   int obs_dis = 200; // 左前方/右前方障碍 有障礙的雷達數據

   int obs_dis_NW_NE[3] = {235, 290, 260}; // TheStack_Drive

   int obs_dis_NW[4] = {220, 245, 220, 3}; // TheStack_Drive

   int obs_dis_NE[4] = {300, 340, 220, 3}; // All

   float left_range[2] = {208, 212};       // TheStack_Drive
   float front_range[3] = {267, 273, 400}; // TheStack_Drive

   int obs_upper_left[4] = {205, 228, 230, 5};
   int obs_upper[3] = {210, 280, 300};
   double b_proportion = 0.5;

   int obs_dis_NW_W[4] = {210, 242, 220, 5}; // TheStack_Drive
   int obs_dis_NE_E[4] = {315, 335, 235, 5}; // TheStack_Drive
   int obs_dis_SW_W[4] = {205, 220, 220, 2}; // TheStack_Drive

   int obs_dis_SE_E[4] = {221, 225, 220, 2};     // TheStack_Drive
   int obs_dis_NE_E_2[4] = {300, 335, 200, 3};   // TheStack_Drive
   int wide_left_no_wall[3] = {210, 245, 550};   // TheStack_Drive
   double m_proportion = 0.70;                   // TheStack_Drive
   int upper_left_range[3] = {210, 245, 500};    // TheStack_Drive
   int narrow_left_no_wall[3] = {210, 240, 400}; // TheStack_Drive

#elif DRIVEBASE == 3
   int obs_left_front[2] = {225, 230};  // Others
   int obs_left_middle[2] = {219, 224}; // Others
   int obs_left_back[2] = {213, 218};   // Others

   int obs_right_front[2] = {310, 315};  // All
   int obs_right_middle[2] = {316, 321}; // All
   int obs_right_back[2] = {322, 327};   // All

   int obs_dis = 200; // 左前方/右前方障碍 有障礙的雷達數據

   int obs_dis_NW_NE[3] = {235, 315, 350}; // Mecanum_Drive

   int obs_dis_NW[4] = {205, 250, 230, 3}; // Mecanum_Drive

   int obs_dis_NE[4] = {300, 340, 220, 3}; // All

   float left_range[2] = {185, 192};       // Mecanum_Drive
   float front_range[3] = {267, 273, 700}; // Mecanum_Drive

   int obs_upper_left[4] = {205, 228, 230, 5};
   int obs_upper[3] = {210, 280, 300};
   double b_proportion = 0.5;

   int obs_dis_NW_W[4] = {185, 240, 220, 5};     // Mecanum_Drive
   int obs_dis_NE_E[4] = {315, 335, 235, 5};     // Mecanum_Drive
   int obs_dis_SW_W[4] = {150, 180, 230, 2};     // Mecanum_Drive
   int obs_dis_SE_E[4] = {181, 235, 230, 2};     // Mecanum_Drive
   int obs_dis_NE_E_2[4] = {300, 335, 220, 3};   // Mecanum_Drive
   int wide_left_no_wall[3] = {190, 235, 550};   // Mecanum_Drive
   double m_proportion = 0.85;                   // Mecanum_Drive
   int upper_left_range[3] = {210, 250, 600};    // Mecanum_Drive
   int narrow_left_no_wall[3] = {190, 225, 500}; // Mecanum_Drive

#elif DRIVEBASE == 4
   int obs_left_front[2] = {236, 240};  // SixWHeel_Drive
   int obs_left_middle[2] = {231, 235}; // SixWHeel_Drive
   int obs_left_back[2] = {225, 230};   // SixWHeel_Drive

   int obs_right_front[2] = {310, 315};  // All
   int obs_right_middle[2] = {316, 321}; // All
   int obs_right_back[2] = {322, 327};   // All

   int obs_dis = 200; // 左前方/右前方障碍 有障礙的雷達數據

   int obs_dis_NW_NE[3] = {235, 315, 350}; // SixWHeel_Drive

   int obs_dis_NW[4] = {205, 250, 230, 3}; // SixWHeel_Drive

   int obs_dis_NE[4] = {300, 340, 220, 3}; // All

   float left_range[2] = {185, 192};       // SixWHeel_Drive
   float front_range[3] = {267, 273, 700}; // SixWHeel_Drive

   int obs_upper_left[4] = {205, 228, 230, 5};
   int obs_upper[3] = {210, 280, 300};
   double b_proportion = 0.5;

   int obs_dis_NW_W[4] = {185, 240, 230, 5};   // SixWHeel_Drive
   int obs_dis_NE_E[4] = {315, 335, 235, 5};   // SixWHeel_Drive
   int obs_dis_SW_W[4] = {150, 180, 230, 2};   // SixWHeel_Drive
   int obs_dis_SE_E[4] = {181, 235, 230, 2};   // SixWHeel_Drive
   int obs_dis_NE_E_2[4] = {300, 335, 220, 3}; // SixWHeel_Drive

   int wide_left_no_wall[3] = {210, 240, 550}; // SixWHeel_Drive
   double m_proportion = 0.85;                 // SixWHeel_Drive

   int upper_left_range[3] = {210, 250, 600};    // SixWHeel_Drive
   int narrow_left_no_wall[3] = {190, 225, 550}; // SixWHeel_Drive

#else
   int obs_left_front[2] = {225, 230};  // Others
   int obs_left_middle[2] = {219, 224}; // Others
   int obs_left_back[2] = {213, 218};   // Others

   int obs_right_front[2] = {310, 315};  // All
   int obs_right_middle[2] = {316, 321}; // All
   int obs_right_back[2] = {322, 327};   // All

   int obs_dis = 200; // 左前方/右前方障碍 有障礙的雷達數據

   int obs_dis_NW_NE[3] = {235, 315, 280}; // X_Drive

   int obs_dis_NW[4] = {205, 240, 230, 3}; // X_Drive

   int obs_dis_NE[4] = {300, 340, 220, 3}; // All

   float left_range[2] = {185, 192};       // X_Drive
   float front_range[3] = {267, 273, 500}; // X_Drive

   int obs_upper_left[4] = {205, 228, 230, 5};
   int obs_upper[3] = {210, 280, 300};
   double b_proportion = 0.5;

   int obs_dis_NW_W[4] = {185, 225, 220, 5};     // X_Drive
   int obs_dis_NE_E[4] = {315, 335, 235, 5};     // X_Drive
   int obs_dis_SW_W[4] = {160, 180, 220, 2};     // X_Drive
   int obs_dis_SE_E[4] = {181, 215, 220, 2};     // X_Drive
   int obs_dis_NE_E_2[4] = {300, 335, 200, 3};   // X_Drive
   int wide_left_no_wall[3] = {190, 235, 550};   // X_Drive
   double m_proportion = 0.85;                   // X_Drive
   int upper_left_range[3] = {190, 225, 500};    // X_Drive
   int narrow_left_no_wall[3] = {190, 225, 550}; // X_Drive

#endif

   static constexpr int mlidarminDis = 100;  //雷达检测的最小距离
   static constexpr int mlidarmaxDis = 5500; //雷达检测的最大距离
   static constexpr int mlidarbeyond = 1000; //超出设定得距离

   static constexpr int indexMax = 360; //雷达最大索引角度
   static constexpr int indexMin = 0;   //雷达最小索引角度

   /******************************************************************************************/
   /************避障参数*************/
   int mRobot_l = 0;        //累加
   int front_L = 0;         //大于前方值——左累加
   int front_R = 0;         //大于前方值——右累加
   int motor_direction = 0; //运动方向

   int Drobot = 0;           //机器人避障状态
   double mcoordangle = 0.0; //机器人角度
   int mangle = 0;           //传递数据（角度）
   int mchannel = 0;         //检测通道口次数
   int RobotObsnow = 0;      //避障状态

   //速度
   double Yspeeed = 0.0;
   double Zspeeed = 0.0;

   //设置机器人底盘类型
   void SetRobot(RobotDrive robot);

   //雷达本地数据
   void GetLidarDataM();

   //角度转换弧度；输入角度
   float AngleRadian(float inputangle);

   //雷达数据转坐标系；开始角度、结束角度
   void RadarCoordinateTransformation(int RCstrat, int RCend);

   //设定范围内返回雷达原始距离的均值；开始角度、结束角度
   float CalculateAverageDistance(int CMstart, int CMend);

   //设定范围内雷达数据转换X轴均值；开始角度、结束角度
   float GetXAxisMeanValue(int XMstart, int XMend);

   //X轴最小值；开始角度、结束角度
   double GetMinXAxisDistance(int XMinst, int XMined);

   //返回设定距离范围内符合的个数，两种模式；一是大于设定值（1），二是小于设定值（-1）;
   //开始角度、结束角度、设定范围、模式（1：大于设定值；-1：小于设定值）
   int GetTargetCountInRange(int Numst, int Numed, double Numrange, int Greaterthanorless);

   //上升沿信号,布尔信号输入、设定超过次数
   bool DetectRisingEdge(bool intput, int Rtimes);

   //返回X轴设定范围内符合的个数，两种模式；一是大于设定值（1），二是小于设定值（-1）；
   //开始角度、结束角度、设定范围、模式（1：大于设定值；-1：小于设定值）
   int GetXAxisTargetCount(int NCin, int NCed, double setpointXdistance, int Greaterthanorless);

   //返回设定距离范围内的占比率，两种模式；一是大于设定值（1），二是小于设定值（-1）；
   //开始角度、结束角度、设定范围、模式（1：大于设定值；-1：小于设定值）
   float CalculateTargetProportion(int Cst, int Ced, double Crange, int CGreaterthanorless);

   void rotateScanData(studica::Lidar::ScanData &scanData, int frontAngle);

   double takeAvg(int startAng, int endAng, const studica::Lidar::ScanData &_scanData);

private:
   studica::TitanQuad frontLeftMotor{constant::TITAN_ID, constant::FRONT_LEFT_MOTOR};
   studica::TitanQuad backLeftMotor{constant::TITAN_ID, constant::BACK_LEFT_MOTOR};
   studica::TitanQuad frontRightMotor{constant::TITAN_ID, constant::FRONT_RIGHT_MOTOR};
   studica::TitanQuad backRightMotor{constant::TITAN_ID, constant::BACK_RIGHT_MOTOR};
   AHRS navX{frc::SPI::Port::kMXP};
   studica::Lidar lidar{studica::Lidar::Port::kUSB1};
   studica::Lidar::ScanData scanData;
   frc::DigitalInput startButton{constant::START_BUTTON};
   frc::DigitalInput eStopButton{constant::E_STOP_BUTTON};
   frc::DigitalOutput runningLED{constant::RUNNING_LED};
   frc::DigitalOutput stoppedLED{constant::STOPPED_LED};

   // Holonomic Variables
   double denomonator = 0;
};
} // namespace _2024
} // namespace WSHK