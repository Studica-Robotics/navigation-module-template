#include "subsystems/DriveTrain.h"

#define DEBUG true
#define Obs true      //避障主体程序
#define function true //新添加的功能函数
#define LookData true //查看SmartDashboard数据

WSHK::_2024::DriveTrain::DriveTrain()
{
    ResetYaw();

    //Motor Invert flags
    frontLeftMotor.SetInverted(false);
    backLeftMotor.SetInverted(false);
    frontRightMotor.SetInverted(true);
    backRightMotor.SetInverted(true);
    //New code
    sytemTime.Start();
}

double WSHK::_2024::DriveTrain::GetYaw()
{
    return navX.GetYaw();
}

double WSHK::_2024::DriveTrain::GetAngle()
{
    return navX.GetAngle();
}

void WSHK::_2024::DriveTrain::ResetYaw()
{
    navX.ZeroYaw();
}

bool WSHK::_2024::DriveTrain::GetStartButton()
{
    return startButton.Get();
}

bool WSHK::_2024::DriveTrain::GetEStopButton()
{
    return eStopButton.Get();
}

void WSHK::_2024::DriveTrain::SetRunningLED(bool on)
{
    runningLED.Set(on);
}

void WSHK::_2024::DriveTrain::SetStoppedLED(bool on)
{
    stoppedLED.Set(on);
}

void WSHK::_2024::DriveTrain::LidarStop()
{
    if (!lidarRunning)
    {
        lidar.Start();
        lidarRunning = true;
    }
}

void WSHK::_2024::DriveTrain::LidarStart()
{
    if (lidarRunning)
    {
        lidar.Stop();
        lidarRunning = false;
    }
}

void WSHK::_2024::DriveTrain::StackMotorControl(double x, double y)
{
    frontLeftMotor.Set(y + x);
    backLeftMotor.Set(y + x);
    frontRightMotor.Set(y - x);
    backRightMotor.Set(y - x);
}

void WSHK::_2024::DriveTrain::TwoWheelMotorControl(double x, double y)
{
    frontLeftMotor.Set(y + x);
    frontRightMotor.Set(y - x);
}

void WSHK::_2024::DriveTrain::SixWheelMotorControl(double x, double y)
{
    frontLeftMotor.Set(y + x);
    backLeftMotor.Set(y + x);
    frontRightMotor.Set(y - x);
    backRightMotor.Set(y - x);
}

void WSHK::_2024::DriveTrain::MecanumMotorControl(double x, double y, double z)
{
    denomonator = fmax(fabs(y) + fabs(x) + fabs(z), 1.0);
    frontLeftMotor.Set(y + (x) + z / denomonator);
    backLeftMotor.Set(y - (x) + z / denomonator);
    frontRightMotor.Set(y - (x)-z / denomonator);
    backRightMotor.Set(y + (x)-z / denomonator);
}

void WSHK::_2024::DriveTrain::XBotMotorControl(double x, double y, double z)
{
    denomonator = fmax(fabs(y) + fabs(x) + fabs(z), 1.0);
    frontLeftMotor.Set(y + (x) + z / denomonator);
    backLeftMotor.Set(y - (x) + z / denomonator);
    frontRightMotor.Set(y - (x)-z / denomonator);
    backRightMotor.Set(y + (x)-z / denomonator);
}

void WSHK::_2024::DriveTrain::Periodic()
{
    if (lidarRunning)
    {
        GetLidarDataM(); //获取雷达数据
    }

#if DEBUG
    frc::SmartDashboard::PutNumber("Yaw", GetYaw());
    frc::SmartDashboard::PutNumber("Angle", GetAngle());
#endif

#if Obs
    mtimeNow = static_cast<double>(sytemTime.Get());
    //X轴检测
    double NT_left_front = GetXAxisMeanValue(obs_left_front[0], obs_left_front[1]); //左侧通道的x轴数据点
    double NT_left_middle = GetXAxisMeanValue(obs_left_middle[0], obs_left_middle[1]);
    double NT_left_back = GetXAxisMeanValue(obs_left_back[0], obs_left_back[1]);

    double NT_right_front = GetXAxisMeanValue(obs_right_front[0], obs_right_front[1]); //右侧通道的x轴数据点
    double NT_right_middle = GetXAxisMeanValue(obs_right_middle[0], obs_right_middle[1]);
    double NT_right_back = GetXAxisMeanValue(obs_right_back[0], obs_right_back[1]);

    //通道可通过的条件,true有障碍
    bool NT_all = (NT_left_back < obs_dis && NT_left_middle < obs_dis && NT_left_front < obs_dis) && (NT_right_front < obs_dis || NT_right_middle < obs_dis || NT_right_back < obs_dis);

    //检测竖立在场地的障碍,左前侧区域
    double front_sector = GetTargetCountInRange(obs_dis_NW_NE[0], obs_dis_NW_NE[1], obs_dis_NW_NE[2], -1);
    bool sector_bool = DetectRisingEdge(front_sector > 0, 2);

    //检测左右侧障碍
    double Left_are = GetTargetCountInRange(obs_dis_NW[0], obs_dis_NW[1], obs_dis_NW[2], -1);
    double Right_are = GetTargetCountInRange(obs_dis_NE[0], obs_dis_NE[1], obs_dis_NE[2], -1);
    bool Left_are_B = Left_are > obs_dis_NW[2];
    bool Right_are_B = Right_are > obs_dis_NE[2];

    float mleft_xdis = GetXAxisMeanValue(left_range[0], left_range[1]);       //用于贴墙左侧X轴的距离
    float DisMean = CalculateAverageDistance(front_range[0], front_range[1]); //正前方距离

    //用于近距离退后避障的数据点
    int Bleftnear = GetTargetCountInRange(obs_upper_left[0], obs_upper_left[1], obs_upper_left[2], -1);
    bool Bleftnear_B = Bleftnear > obs_upper_left[3];
    double Nearroportion = CalculateTargetProportion(obs_upper[0], obs_upper[1], obs_upper[2], -1); //左侧较密集的点
    bool NearleftP = Nearroportion < b_proportion;

    //X轴检测障碍(远端)
    int leftNumber = GetXAxisTargetCount(obs_dis_NW_W[0], obs_dis_NW_W[1], obs_dis_NW_W[2], -1);
    int rightNumber = GetXAxisTargetCount(obs_dis_NE_E[0], obs_dis_NE_E[1], obs_dis_NE_E[2], -1);
    bool left_farB = leftNumber > obs_dis_NW_W[3];
    bool right_farB = rightNumber > obs_dis_NE_E[3];

    // 近距离障碍
    int leftNearback = GetXAxisTargetCount(obs_dis_SW_W[0], obs_dis_SW_W[1], obs_dis_SW_W[2], -1);
    int leftNearfront = GetXAxisTargetCount(obs_dis_SE_E[0], obs_dis_SE_E[1], obs_dis_SE_E[2], -1);
    int rightNearnumber = GetXAxisTargetCount(obs_dis_NE_E_2[0], obs_dis_NE_E_2[1], obs_dis_NE_E_2[2], -1);
    bool leftNearB = leftNearback > obs_dis_SW_W[3] || leftNearfront > obs_dis_SE_E[3];
    bool rightNear_B = rightNearnumber > obs_dis_NE_E_2[3];

    // 左侧区域检测通道口
    double proportion = CalculateTargetProportion(wide_left_no_wall[0], wide_left_no_wall[1], wide_left_no_wall[2], 1); //用于计算左侧区域满足距离的占比率
    double leftXmindis = GetMinXAxisDistance(upper_left_range[0], upper_left_range[1]);
    double leftone = GetTargetCountInRange(narrow_left_no_wall[0], narrow_left_no_wall[1], narrow_left_no_wall[2], -1);
    bool aisleLeft = (leftXmindis > upper_left_range[2] || proportion > m_proportion) && DisMean > front_range[2];
    bool leftone_bool = leftone < 1;
    bool Aisle = (aisleLeft || leftone_bool) && (!leftNearB);

    //检测贴墙
    bool wall_target = mleft_xdis < WallHigh && mleft_xdis > WallLow;
    //检测左边障碍信号
    bool Obs_left = mleft_xdis < WallLow || NT_all || Left_are_B || sector_bool || left_farB || leftNearB;
    //检测右边障碍信号
    bool Obs_right = mleft_xdis > WallHigh || Right_are_B || right_farB || rightNear_B;

    /********主逻辑*******/
    if (mrobot == TheStack_Drive)
    {
        switch (Drobot)
        {
            static int Neardata = 0; //后退循环次数
            static int Obschannel = 0;
        case 0:
        {
            if (Aisle)
            {
                mchannel++;
                if (mchannel > 1)
                {
                    Obschannel = 0;
                    Drobot = 1;
                }
                else
                {
                    Drobot = 2;
                }
            }
            else
            {
                mchannel = 0;
                Drobot = 2;
            }
            mRobot_l = 0;
        }
        break;
        /* 左边可通行（用执行通道口功能）*/
        case 1:
        {
            if (mRobot_l == 0)
            {
                mtimeraw = mtimeNow;
                mcoordangle = navX.GetAngle();
                RobotObsnow = 103;
            }
            mRobot_l++;
            mtimedata = std::abs(mtimeNow - mtimeraw);
            mangle = std::abs(navX.GetAngle() - mcoordangle);
            switch (Obschannel)
            {
            case 0:
            {
                RobotObsnow = 102;
                if (mtimedata > walktimes || rightNear_B)
                {
                    Obschannel = 1;
                }
                else
                {
                    motor_direction = 0;
                    RobotObsnow = 109; /*前进*/
                }
            }
            break;
            case 1:
            {
                if (mangle >= Rate_left || leftNearB)
                {
                    Drobot = 0;
                }
                else
                {
                    motor_direction = 1; /*左转*/
                }
            }
            break;
            default:
                break;
            }
        }
        break;
        /***贴边与避障***/
        case 2:
        {
            if (wall_target && !NT_all && !sector_bool)
            {
                RobotObsnow = 29;
                motor_direction = 0; /*前进 */
            }
            else if (Obs_left)
            {
                front_L = 0;
                front_R++;
                if (Bleftnear_B && NearleftP)
                {
                    Neardata = 25;
                    Drobot = 3;
                }
                else if (front_R > Lamend_times || NT_all || sector_bool || leftNearB)
                {
                    RobotObsnow = 221;
                    motor_direction = 2; /*右转*/
                }
                else
                {
                    RobotObsnow = 229;
                    motor_direction = 0; /*前进*/
                }
            }
            else if (Obs_right)
            {
                front_R = 0;
                front_L++;
                if (front_L > Ramend_times)
                {
                    RobotObsnow = 231;
                    motor_direction = 1; /*左转*/
                }
                else
                {
                    RobotObsnow = 239;
                    motor_direction = 0; /*前进*/
                    Drobot = 0;
                }
            }
        }
        break;
        case 3:
        {
            RobotObsnow = 300;
            if (Neardata <= 0)
            {
                Drobot = 2;
            }
            else
            {
                Neardata--;
                motor_direction = 4; /*后退*/
            }
        }
        break;
        case 4:
        {
            if (left_farB || leftNearB)
            {
                motor_direction = 2; /*右转*/
            }
            else
            {
                Drobot = 0;
            }
        }
        break;
        default:
            break;
        }
    }
    else
    {
        switch (Drobot)
        {
            static int Obschannel = 0;
        case 0:
        {
            if (Aisle)
            {
                mchannel++;
                if (mchannel > 1)
                {
                    Obschannel = 0;
                    Drobot = 1;
                }
                else
                {
                    Drobot = 2;
                }
            }
            else
            {
                mchannel = 0;
                Drobot = 2;
            }
            mRobot_l = 0;
        }
        break;
        /* 左边可通行（用执行通道口功能）*/
        case 1:
        {
            if (mRobot_l == 0)
            {
                mtimeraw = mtimeNow;
                mcoordangle = navX.GetAngle();
                RobotObsnow = 103;
            }
            mRobot_l++;
            mtimedata = std::abs(mtimeNow - mtimeraw);
            mangle = std::abs(navX.GetAngle() - mcoordangle);
            switch (Obschannel)
            {
            case 0:
            {
                RobotObsnow = 102;
                if (mtimedata > walktimes || rightNear_B)
                {
                    Obschannel = 1;
                }
                else
                {
                    motor_direction = 0;
                    RobotObsnow = 109; /*前进*/
                }
            }
            break;
            case 1:
            {
                if (mangle >= Rate_left || leftNearB)
                {
                    Drobot = 0;
                }
                else
                {
                    motor_direction = 1; /*左转*/
                }
            }
            break;
            default:
                break;
            }
        }
        break;
        /***贴边与避障***/
        case 2:
        {
            if (wall_target && !NT_all && !sector_bool)
            {
                RobotObsnow = 29;
                motor_direction = 0; /*前进 */
            }
            else if (Obs_left)
            {
                front_L = 0;
                front_R++;
                if (front_R > Lamend_times || NT_all || sector_bool || leftNearB)
                {
                    RobotObsnow = 221;
                    motor_direction = 2; /*右转*/
                }
                else
                {
                    RobotObsnow = 229;
                    motor_direction = 0; /*前进*/
                }
            }
            else if (Obs_right)
            {
                front_R = 0;
                front_L++;
                if (front_L > Ramend_times)
                {
                    RobotObsnow = 231;
                    motor_direction = 1; /*左转*/
                }
                else
                {
                    RobotObsnow = 239;
                    motor_direction = 0; /*前进*/
                }
            }
            Drobot = 0;
        }
        break;
        default:
            break;
        }
    }

    /***********速度分配************/
    switch (motor_direction)
    {
    case 0: /*前进*/
        Yspeeed = forwardSpeed;
        Zspeeed = 0.0;
        break;
    case 1: /*左转*/
        Yspeeed = turnLeftSpeedY;
        Zspeeed = turnLeftSpeed;
        break;
    case 2: /*右转*/
        Yspeeed = turnRightSpeedY;
        Zspeeed = turnRightSpeed;
        break;
    case 3: /*停止*/
        Yspeeed = 0.0;
        Zspeeed = 0.0;
        break;
    case 4: /*后退*/
        Yspeeed = backSpeed;
        Zspeeed = 0.0;
        break;
    default:
        break;
    }
    /***管理机器人类型***/
    switch (mrobot)
    {
    case RobotDrive::X_Drive:
        XBotMotorControl(0.0, Yspeeed, Zspeeed);
        break;
    case RobotDrive::TwoWheel_Drive:
        TwoWheelMotorControl(Zspeeed, Yspeeed);
        break;
    case RobotDrive::TheStack_Drive:
        StackMotorControl(Zspeeed, Yspeeed);
        break;
    case RobotDrive::Mecanum_Drive:
        MecanumMotorControl(0.0, Yspeeed, Zspeeed);
        break;
    case RobotDrive::SixWHeel_Drive:
        SixWheelMotorControl(Zspeeed, Yspeeed);
        break;
    default:
        break;
    }
#endif

#if LookData
    frc::SmartDashboard::PutNumber("dis_NW_NE", takeAvg(235, 315, scanData));
    frc::SmartDashboard::PutNumber("dis_NW", takeAvg(205, 240, scanData));
    frc::SmartDashboard::PutNumber("dis_NE", takeAvg(300, 340, scanData));
    frc::SmartDashboard::PutNumber("dis_NW_W", takeAvg(185, 225, scanData));
    frc::SmartDashboard::PutNumber("dis_NE_E", takeAvg(315, 335, scanData));
    frc::SmartDashboard::PutNumber("dis_SW_W", takeAvg(160, 180, scanData));
    frc::SmartDashboard::PutNumber("dis_SE_E", takeAvg(181, 215, scanData));

    /******避障状态******/
    frc::SmartDashboard::PutNumber("BDrobot", Drobot);
    frc::SmartDashboard::PutNumber("BmRobot_l", mRobot_l);
    frc::SmartDashboard::PutBoolean("BObs_left", Obs_left);
    frc::SmartDashboard::PutBoolean("BObs_right", Obs_right);
    frc::SmartDashboard::PutNumber("BRobotObsnow", RobotObsnow);
    /****竖障碍检测*******/
    frc::SmartDashboard::PutBoolean("sector_bool", sector_bool);
    frc::SmartDashboard::PutNumber("front_sector", front_sector);
    /***数据****/
    frc::SmartDashboard::PutNumber("proportion", proportion);
    /*********通道检测数据查看************/ //C
    frc::SmartDashboard::PutNumber("CleftNearback", leftNearback);
    frc::SmartDashboard::PutNumber("CleftNearfront", leftNearfront);
    frc::SmartDashboard::PutBoolean("CaisleLeft", aisleLeft);
    frc::SmartDashboard::PutBoolean("Cleftone_bool", leftone_bool);
    frc::SmartDashboard::PutBoolean("CAisle", Aisle);
    frc::SmartDashboard::PutNumber("CDisMean5", DisMean);
    frc::SmartDashboard::PutNumber("CleftXmindis", leftXmindis);
    frc::SmartDashboard::PutNumber("Cleftone", leftone);
    frc::SmartDashboard::PutBoolean("CLeft_are_B", Left_are_B);
    /***********贴边**************/
    frc::SmartDashboard::PutNumber("Tmleft_xdis", mleft_xdis);
    frc::SmartDashboard::PutBoolean("TNT_all", NT_all);
    frc::SmartDashboard::PutNumber("TleftNumber", leftNumber);
    frc::SmartDashboard::PutNumber("TrightNumber", rightNumber);
    frc::SmartDashboard::PutBoolean("TRight_are_B", Right_are_B);
    frc::SmartDashboard::PutBoolean("rightNear_B", rightNear_B);
    frc::SmartDashboard::PutNumber("TrightNear-1", rightNearnumber);
    frc::SmartDashboard::PutBoolean("Tleft_farB", left_farB);
    frc::SmartDashboard::PutBoolean("TleftNearB", leftNearB);
#endif
}

float WSHK::_2024::DriveTrain::AngleRadian(float inputangle)
{ //角度转换弧度
    return (inputangle * M_PI) / 180 + M_PI;
}

void WSHK::_2024::DriveTrain::RadarCoordinateTransformation(int RCstrat, int RCend)
{ //转换角度区间数据（坐标）
    if (RCstrat > indexMin && RCend < indexMax)
    {
        for (int i = RCstrat; i < RCend; i++)
        {
            double m_radian = AngleRadian(scanData.angle[i]);
            mcoord.x[i] = std::cos(m_radian) * scanData.distance[i];
            mcoord.y[i] = std::sin(m_radian) * scanData.distance[i];
        }
    }
}

#if function
/***************新添加的代码**************/

void WSHK::_2024::DriveTrain::SetRobot(RobotDrive robot)
{
    mrobot = robot;
}

void WSHK::_2024::DriveTrain::GetLidarDataM()
{
    if (mrobot == TheStack_Drive)
    {
        lidar.JitterConfig(50);
        lidar.EnableFilter(studica::Lidar::Filter::kJITTER, true);
    }
    scanData = lidar.GetData();
    WSHK::_2024::DataContainer::set_LiDAR_data(scanData, liDAR_right_ang - 90);
    rotateScanData(scanData, liDAR_right_ang);
}

float WSHK::_2024::DriveTrain::CalculateAverageDistance(int CMstart, int CMend)
{
    if (CMstart > indexMin && CMend < indexMax)
    {
        double D_value = 0;
        int D_times = 0;
        int Drange_T = 0;
        int Drange_F = 0;
        double DvauleF = 0;
        for (int i = CMstart; i < CMend; i++)
        {
            if (scanData.distance[i] > mlidarminDis && scanData.distance[i] < mlidarmaxDis)
            {
                if (scanData.distance[i] > mlidarbeyond)
                {
                    Drange_T++;
                }
                else
                {
                    Drange_F++;
                    DvauleF += scanData.distance[i];
                }
                D_value += scanData.distance[i];
                D_times++;
            }
        }
        double DdataF = (Drange_F > 0) ? DvauleF / Drange_F : 0;
        double Ddatam = (D_times > 0) ? D_value / D_times : 0;
        if (Drange_T > (D_times / 2))
        {
            return Ddatam;
        }
        else
        {
            return DdataF;
        }
    }
    else
    {
        return -1;
    }
}

float WSHK::_2024::DriveTrain::GetXAxisMeanValue(int XMstart, int XMend)
{
    if (XMstart > indexMin && XMend < indexMax)
    {
        double m_allvalue = 0;
        int m_LM = 0;
        int m_beyT = 0;
        int m_beyF = 0;
        double m_bey_value = 0;
        RadarCoordinateTransformation(XMstart, XMend);
        for (int i = XMstart; i < XMend; i++)
        {
            double Lmean = std::abs(mcoord.x[i]);
            if (Lmean > mlidarminDis && Lmean < mlidarmaxDis)
            {
                if (Lmean > mlidarbeyond)
                {
                    m_beyT++;
                }
                else
                {
                    m_beyF++;
                    m_bey_value += Lmean;
                }
                m_allvalue += Lmean;
                m_LM++;
            }
        }
        double mdata = m_LM > 0 ? m_allvalue / m_LM : 0;
        double mbeydata = m_beyF > 0 ? m_bey_value / m_beyF : 0;
        if (m_beyT > (m_LM / 2))
        {
            return mdata;
        }
        else
        {
            return mbeydata;
        }
    }
    else
    {
        return -1;
    }
}

double WSHK::_2024::DriveTrain::GetMinXAxisDistance(int XMinst, int XMined)
{
    if (XMinst > indexMin && XMined < indexMax)
    {
        double xmin = mlidarmaxDis;
        RadarCoordinateTransformation(XMinst, XMined);
        for (int i = XMinst; i < XMined; i++)
        {
            double mdata = std::abs(mcoord.x[i]);
            if (mdata < xmin && mdata > mlidarminDis)
            {
                xmin = mdata;
            }
        }
        return xmin;
    }
    else
    {
        return -1;
    }
}

int WSHK::_2024::DriveTrain::GetTargetCountInRange(int Numst, int Numed, double Numrange, int Greaterthanorless = 1)
{
    if (Numst > indexMin && Numed < indexMax)
    {
        int Ntimes = 0;
        for (int i = Numst; i < Numed; i++)
        {
            float mdistance = scanData.distance[i];
            switch (Greaterthanorless)
            {
            case 1:
                if (mdistance > Numrange && mdistance > mlidarminDis)
                {
                    Ntimes++;
                }
                break;
            case -1:
                if (mdistance < Numrange && mdistance > mlidarminDis)
                {
                    Ntimes++;
                }
                break;
            default:
                break;
            }
        }
        return Ntimes;
    }
    else
    {
        return -1;
    }
}

bool WSHK::_2024::DriveTrain::DetectRisingEdge(bool input, int Rtimes)
{
    static int up_times = 0;
    if (input)
    {
        up_times++;
    }
    else
    {
        up_times = 0;
    }
    return up_times > Rtimes;
}

int WSHK::_2024::DriveTrain::GetXAxisTargetCount(int NCin, int NCed, double setpointXdistance, int Greaterthanorless)
{
    if (NCin > indexMin && NCed < indexMax)
    {
        int NCtimes = 0;
        RadarCoordinateTransformation(NCin, NCed);
        for (int i = NCin; i < NCed; i++)
        {
            float mlidar_x = std::abs(mcoord.x[i]);
            switch (Greaterthanorless)
            {
            case 1:
                if (mlidar_x > setpointXdistance && mlidar_x > mlidarminDis)
                {
                    NCtimes++;
                }
                break;
            case -1:
                if (mlidar_x < setpointXdistance && mlidar_x > mlidarminDis)
                {
                    NCtimes++;
                }
                break;
            default:
                break;
            }
        }
        return NCtimes;
    }
    else
    {
        return -1;
    }
}

float WSHK::_2024::DriveTrain::CalculateTargetProportion(int Cst, int Ced, double Crange, int CGreaterthanorless)
{
    if (Cst > indexMin && Ced < indexMax)
    {
        float Ntimes = 0;
        float alltimes = 0;
        for (int i = Cst; i < Ced; i++)
        {
            bool mbool = scanData.distance[i] > mlidarminDis;
            switch (CGreaterthanorless)
            {
            case 1:
                if (mbool)
                {
                    if (scanData.distance[i] > Crange)
                    {
                        Ntimes++;
                    }
                    alltimes++;
                }
                break;
            case -1:
                if (mbool)
                {
                    if (scanData.distance[i] < Crange)
                    {
                        Ntimes++;
                    }
                    alltimes++;
                }
                break;
            default:
                break;
            }
        }
        float data = (alltimes == 0) ? (-1.0) : (Ntimes / alltimes);
        return data;
    }
    else
    {
        return -1.0;
    }
}
#endif

void WSHK::_2024::DriveTrain::rotateScanData(studica::Lidar::ScanData &_scanData, int frontAngle)
{
    // Calculate the rotation index
    int index = (frontAngle + 360) % 360;

    // Rotate the angle array
    // std::rotate(_scanData.angle, _scanData.angle + index, _scanData.angle + 360);

    // Rotate the distance array
    std::rotate(_scanData.distance, _scanData.distance + index, _scanData.distance + 360);
}

double WSHK::_2024::DriveTrain::takeAvg(int startAng, int endAng, const studica::Lidar::ScanData &_scanData)
{
    int _count = 0;
    double _dis_temp = 0;
    for (int k = startAng; k < endAng; k++)
    {
        int i = k % 360;
        if ((std::fabs(_scanData.distance[i]) > 0) && (std::fabs(_scanData.distance[i]) < 10000))
        {
            _dis_temp += std::ceil(std::fabs(_scanData.distance[i]));
            _count += 1;
        }
    }
    if (_count != 0)
    {
        _dis_temp /= _count;
    }
    return _dis_temp;
}
