#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

#include "PathPlanner.h"
#include "Util.h"
#include "Move.h"

#include "Navigation/DynamicWindowApproach.h"
#include "Navigation/Astar.h"

PathPlanner::PathPlanner()
{
    map_ = new unsigned char*[Constants::MAP_SIZE];
    costmap_ = new unsigned char*[Constants::MAP_SIZE];
    for (int i = 0; i < Constants::MAP_SIZE; i++)
    {
        map_[i] = new unsigned char[Constants::MAP_SIZE];
        costmap_[i] = new unsigned char[Constants::MAP_SIZE];
    }

    map_view_ = frc::CameraServer::GetInstance()->PutVideo("map_view", Constants::MAP_SIZE, Constants::MAP_SIZE);
    map_mat_ = cv::Mat(Constants::MAP_SIZE, Constants::MAP_SIZE, CV_8UC3, cv::Scalar(128, 128, 128));

    costmap_view_ = frc::CameraServer::GetInstance()->PutVideo("costmap_view", Constants::MAP_SIZE, Constants::MAP_SIZE);
    costmap_mat_ = cv::Mat(Constants::MAP_SIZE, Constants::MAP_SIZE, CV_8UC3, cv::Scalar(128, 128, 128));

    kdtree_ = new KDTree<Vector2i>();
    occupancy_grid_map_ = new OcuupancyGridMap(map_);
    dwa_ = new DynamicWindowApproach();
    mcl_ = new MCL(map_);

    mcl_running_ = occupancy_running_ = astar_running_ = false;

    dwa_thread_ = std::thread(&PathPlanner::DWAThread, this);
    astar_thread_ = std::thread(&PathPlanner::AstarThread, this);
    mcl_thread_ = std::thread(&PathPlanner::MCLThread, this);
    map_view_thread_ = std::thread(&PathPlanner::MapViewThread, this);

    turn_flg_ = false;
    stop_cnt_ = 0;
}
PathPlanner::~PathPlanner()
{
    dwa_thread_.join();
    astar_thread_.join();
    mcl_thread_.join();
    map_view_thread_.join();

    delete kdtree_;
    delete occupancy_grid_map_;
    delete dwa_;
    delete mcl_;

    for (int i = 0; i < Constants::MAP_SIZE; i++)
    {
        delete[] map_[i];
        delete[] costmap_[i];
    }
    delete[] map_;
    delete[] costmap_;
}
void PathPlanner::SetPlannerProperties()
{
    if (Interface::SW1_Pressed)
    {
        Interface::GyroReset = true;

        mcl_->Reset();
        dwa_->Init();

        local_goal_[0] = local_goal_[1] = current_goal_[0] = current_goal_[1] = -1;
        mcl_running_ = occupancy_running_ = astar_running_ = false;

        occupancy_grid_map_->ResetMap();
        path_.clear();
        traj_.clear();
        control_[0] = control_[1] = control_[2] = control_[3] = control_[4] = 0;

        turn_flg_ = false;
        stop_cnt_ = 0;
        Interface::RobotPose[0] = Interface::RobotPose[1] = Interface::RobotPose[2] = 0;

        delay(1000);

        Interface::PathPlanning = true;
        mcl_running_ = true;
        occupancy_running_ = true;
        astar_running_ = true;

        Interface::SW1_Pressed = false;
    }
}
void PathPlanner::DWAUpdate(Vector5d &control)
{
    if(!Interface::PathPlanning) return;

    current_goal_[0] = Constants::Goal[0];
    current_goal_[1] = Constants::Goal[1];

    control[0] = Interface::RobotPose[0];
    control[1] = Interface::RobotPose[1];
    control[2] = Interface::RobotPose[2];
    control[3] = Interface::Velocity[0];
    control[4] = ToRadian(Interface::Velocity[2]);

    for(int i = 0; i < int(path_.size()); i++)
    {
        if(GetDistance(ConvertToGridCoordinate(Interface::RobotPose), path_[i]) <= Constants::LOOK_AHEAD_DISTANCE / double(Constants::MM_PER_PIXEL))
        {
            local_goal_[0] = path_[i][0];
            local_goal_[1] = path_[i][1];
            break;
        }
    }
    if(local_goal_[0] == -1 || local_goal_[1] == -1) return;

    Vector3d world_local_goal = ConvertToWorldCoordinate(local_goal_);

    std::vector<Vector2d> obstacle;
    double r, th;

    for (int i = 0; i < 450; i++)
    {
        if(Interface::laser_scan[i].range)
        {
            r = Interface::laser_scan[i].range;
            th = NormalizeRadian(Interface::laser_scan[i].angle + Interface::RobotPose[2]);

            Vector2d laser_point;
            laser_point[0] = r * std::cos(th) + Interface::RobotPose[0];
            laser_point[1] = r * std::sin(th) + Interface::RobotPose[1];
            obstacle.push_back(laser_point);
        }
    }

    Vector3d center_pose;
    GetRobotCenterPosition({control[0], control[1], control[2]}, center_pose);
    double dist = GetDistance(Vector2d{center_pose[0], center_pose[1]}, Vector2d{current_goal_[0], current_goal_[1]});
    // double dist = GetDistance(Vector2d{control[0], control[1]}, Vector2d{current_goal_[0], current_goal_[1]});
    dwa_->Update(obstacle, {world_local_goal[0], world_local_goal[1]}, control, traj_);

    if(dist < Constants::GOAL_TORLERANCE || Interface::RobotPose[0] > 4650)
    {
        Move::filter(0, 0, 0);
        if(std::abs(Interface::Velocity[0]) < 1 && std::abs(Interface::Velocity[2]) < 1)
        {
            Move::Stop();

            current_goal_[0] = current_goal_[1] = -1;
            path_.clear();
            Interface::PathPlanning = false;
            astar_running_ = false;
        }
    }
    else
    {
        if(dist < Constants::DECELERATION_DISTANCE + Constants::GOAL_TORLERANCE)
        {
            double curvature = control[4] / control[3];
            double linear_vel = Trans(Constants::GOAL_TORLERANCE, Constants::DECELERATION_DISTANCE + Constants::GOAL_TORLERANCE,
                                      Constants::MIN_VELOCITY[0], 500, dist) * sign(control[3]);
            if (std::abs(linear_vel) < std::abs(control[3]))
            {
                control[3] = linear_vel;
                control[4] = std::abs(control[3] * curvature) * sign(control[4]);
            }
        }
        Move::filter(control[3], 0, ToDegree(control[4]));
    }
}
void PathPlanner::TireUpdate()
{
    current_goal_[0] = Constants::Goal[0];
    current_goal_[1] = Constants::Goal[1];

    if(path_.empty()) return;

    int look_dist = 230;

    for(int i = 0; i < int(path_.size()); i++)
    {
        if(GetDistance(ConvertToGridCoordinate(Interface::RobotPose), path_[i]) <= look_dist / double(Constants::MM_PER_PIXEL))
        {
            local_goal_[0] = path_[i][0];
            local_goal_[1] = path_[i][1];
            break;
        }
    }
    Vector3d world_local_goal = ConvertToWorldCoordinate(local_goal_);

    Vector3d center_pose;
    GetRobotCenterPosition({Interface::RobotPose[0], Interface::RobotPose[1], Interface::RobotPose[2]}, center_pose);
    double dist = GetDistance(Vector2d{center_pose[0], center_pose[1]}, Vector2d{current_goal_[0], current_goal_[1]});
    if(dist < Constants::GOAL_TORLERANCE)
    {
        Move::filter(0, 0, 0);
        if(std::abs(Interface::Velocity[0]) < 1 && std::abs(Interface::Velocity[2]) < 1)
        {
            Move::Stop();

            current_goal_[0] = current_goal_[1] = -1;
            path_.clear();
            Interface::PathPlanning = false;
            astar_running_ = false;
        }
    }
    else
    {
        double theta = NormalizeRadian(std::atan2(world_local_goal[1] - Interface::RobotPose[1], world_local_goal[0] - Interface::RobotPose[0]) - Interface::RobotPose[2]);
        double err = ToDegree(theta);
        double curvature = 2 * std::sin(theta) / look_dist;

        if(std::abs(err) > 15) turn_flg_ = true;
        else if(std::abs(err) < 5) turn_flg_ = false;

        double vel[2] = {150, 50};
        if(DRIVE_TYPE == Mecanum_Drive) vel[0] = 250, vel[1] = 80;

        if(turn_flg_) Move::filter(0, 0, vel[1] * sign(err));
        else Move::filter(vel[0], 0, vel[0] * ToDegree(curvature));
    }
}
void PathPlanner::DWAThread()
{
    int past_time;

    delay(300, 0);
    while (true)
    {
        while(Interface::EMS) delay(50, 0);

        Interface::SW1_Pressed = false;
        Interface::SW2_Pressed = false;

        occupancy_grid_map_->ResetMap();

        Interface::PathPlanning = false;
        mcl_running_ = occupancy_running_ = astar_running_ = false;

        current_goal_[0] = current_goal_[1] = -1;
        local_goal_[0] = local_goal_[1] = -1;
        path_.clear();
        traj_.clear();

        control_[0] = control_[1] = control_[2] = control_[3] = control_[4] = 0;

        delay(500, 0);

        while(!Interface::EMS)
        {
            past_time = GetTime();

            SetPlannerProperties();
            Interface::LED_G = Interface::PathPlanning;

            if(Interface::PathPlanning)
            {
                if(DRIVE_TYPE == Tire_Drive) TireUpdate(); //  || DRIVE_TYPE == Mecanum_Drive
                else
                {
                    if(Interface::Velocity[0])
                    {
                        int obstacle_cnt = 0;
                        
                        for (int i = -32 * 1.25; i < 32 * 1.25; i++)
                        {
                            int deg = i + 0;
                            if(deg < 0) deg += 450;
                            // 수정
                            if(Interface::laser_scan_storage[deg].range && Interface::laser_scan_storage[deg].range - Constants::ROBOT_CENTER_OFFSET < Constants::ROBOT_RADIUS * Constants::MM_PER_PIXEL + 130 + 50) obstacle_cnt++;
                        }
                        if(stop_cnt_ == 0 && obstacle_cnt > 5) stop_cnt_ = 31;
                    }

                    if(stop_cnt_ > 30)
                    {
                        Move::filter(0, 0, 0);
                        if(std::abs(Interface::Velocity[0]) < 1 && std::abs(Interface::Velocity[2]) < 1) stop_cnt_--;
                    }
                    else if(Interface::laser_scan_flg2 && stop_cnt_ <= 30)
                    {
                        DWAUpdate(control_);
                        Interface::laser_scan_flg2 = false;

                        if(control_[3] || control_[4])
                        {
                            stop_cnt_--;
                            if(stop_cnt_ < 0) stop_cnt_ = 0;
                        }
                    }
                }
            }
            delay(20, past_time);
        }
        Interface::LED_G = false;
    }
}
void PathPlanner::AstarThread()
{
    int past_time;
    Astar astar;

    delay(400, 0);
    while (true)
    {
        while(Interface::EMS || !astar_running_ || current_goal_[0] == -1 || current_goal_[1] == -1) delay(50, 0);

        past_time = GetTime();

        BuildCostmap();
        bool status = astar.Update(ConvertToGridCoordinate(Interface::RobotPose), ConvertToGridCoordinate(current_goal_), costmap_, path_);
        if(!status)
        {
            occupancy_grid_map_->ResetMap();
            for(int i = 0; i < Constants::MAP_SIZE; i++) for(int j = 0; j < Constants::MAP_SIZE; j++) costmap_[i][j] = 255;
        }
        delay(status ? 400 : 50, past_time);
    }
}
void PathPlanner::MCLThread()
{
    int past_time, mcl_prev_time = 0;
    double prev_yaw = 999;

    delay(500, 0);
    while (true)
    {
        while(Interface::EMS) delay(50, 0);
        
        mcl_->Reset();
        mcl_prev_time = 0;
        prev_yaw = 999;

        while(!Interface::EMS)
        {
            past_time = GetTime();
                
            if(Interface::PathPlanning && Interface::laser_scan_flg)
            {
                if(Interface::RobotPose[0] > 3900 + Constants::ROBOT_CENTER_OFFSET)
                {
                    mcl_running_ = false;
                    occupancy_running_ = false;
                }

                if(mcl_running_) mcl_->Update();
                else
                {
                    if(prev_yaw != 999) Interface::RobotPose[2] = NormalizeRadian(Interface::RobotPose[2] + NormalizeRadian(Interface::GyroYaw - prev_yaw));

                    if(mcl_prev_time)
                    {
                        double predict_vel = Interface::Velocity[0];

                        if(DRIVE_TYPE == Mecanum_Drive) predict_vel *= 0.74;

                        double r = predict_vel * (GetTime() - mcl_prev_time) / 1000.0;
                        Interface::RobotPose[0] += r * std::cos(Interface::RobotPose[2]);
                        Interface::RobotPose[1] += r * std::sin(Interface::RobotPose[2]);
                    }
                }
                prev_yaw = Interface::GyroYaw;
                mcl_prev_time = GetTime();
                
                if(occupancy_running_) occupancy_grid_map_->Update();
                Interface::laser_scan_flg = false;
            }
            
            delay(5, past_time);
        }
    }
}
void PathPlanner::MapViewThread()
{
    Vector2i pose;
    int past_time;

    delay(600, 0);
    while (true)
    {
        while (Interface::EMS || !Constants::DEBUG) delay(100, 0);
        past_time = GetTime();

        for (int i = 0; i < Constants::MAP_SIZE; i++)
        {
            for (int j = 0; j < Constants::MAP_SIZE; j++)
            {
                map_mat_.data[i * map_mat_.cols * 3 + j * 3 + 0] = 128;
                map_mat_.data[i * map_mat_.cols * 3 + j * 3 + 1] = 128;
                map_mat_.data[i * map_mat_.cols * 3 + j * 3 + 2] = 128;
            }
        }

        if(!mcl_running_) Interface::RobotPose[2] = Interface::GyroYaw;

        occupancy_grid_map_->Show(map_mat_);
        double r, th;
        for (int i = 0; i < 450; i++)
        {
            if(Interface::laser_scan[i].range)
            {
                r = Interface::laser_scan[i].range;
                th = NormalizeRadian(Interface::laser_scan[i].angle + Interface::RobotPose[2]);

                Vector3d laser_point;
                laser_point[0] = r * std::cos(th) + Interface::RobotPose[0];
                laser_point[1] = r * std::sin(th) + Interface::RobotPose[1];
                laser_point[2] = th;

                if(laser_point[0] <= 100 + Constants::ROBOT_CENTER_OFFSET) continue;
                if(laser_point[0] >= Constants::MapBorder[0] + Constants::ROBOT_CENTER_OFFSET + Constants::LIDAR_DATA_CUT_SIZE) continue;
                if(laser_point[1] <= -Constants::MapBorder[1] / 2.0 - Constants::LIDAR_DATA_CUT_SIZE) continue;
                if(laser_point[1] >= Constants::MapBorder[1] / 2.0 + Constants::LIDAR_DATA_CUT_SIZE) continue;

                Vector2i grid_pose = ConvertToGridCoordinate(laser_point);
                if (IsSafe(grid_pose, Constants::MAP_SIZE))
                    cv::circle(map_mat_, cv::Point(grid_pose[1], grid_pose[0]), 1, cv::Scalar(0, 0, 255), -1);
            }
        }
        
        for (unsigned int i = 0; i < path_.size(); i += 2)
            cv::circle(map_mat_, cv::Point(path_[i][1], path_[i][0]), 1, cv::Scalar(0, 0, 255), -1);

        mcl_->Show(map_mat_);
        
        // dwa_->Show(map_mat_);
        for (unsigned int i = 0; i < traj_.size(); i++)
        {
            pose = ConvertToGridCoordinate(Vector3d{traj_[i][0], traj_[i][1], 0});
			cv::circle(map_mat_, cv::Point(pose[1], pose[0]), 1, cv::Scalar(0, 0, 255), -1);
        }

        if(local_goal_[0] != -1 && local_goal_[1] != -1)
            cv::circle(map_mat_, cv::Point(std::round(local_goal_[1]), std::round(local_goal_[0])), 3, cv::Scalar(255, 0, 0), -1);

        pose = ConvertToGridCoordinate(Vector3d{current_goal_[0], current_goal_[1], 0});
        cv::circle(map_mat_, cv::Point(pose[1], pose[0]), 2, cv::Scalar(255, 0, 255), -1);
        
        ShowRobotPosition(Interface::RobotPose);

        for (int i = 0; i < Constants::MAP_SIZE; i++)
        {
            for (int j = 0; j < Constants::MAP_SIZE; j++)
            {
                if(costmap_[i][j] == 254)
                {
                    costmap_mat_.data[i * costmap_mat_.cols * 3 + j * 3 + 0] = 255;
                    costmap_mat_.data[i * costmap_mat_.cols * 3 + j * 3 + 1] = 0;
                    costmap_mat_.data[i * costmap_mat_.cols * 3 + j * 3 + 2] = 0;
                }
                else if(costmap_[i][j] == 253 || costmap_[i][j] == 252)
                {
                    costmap_mat_.data[i * costmap_mat_.cols * 3 + j * 3 + 0] = 255;
                    costmap_mat_.data[i * costmap_mat_.cols * 3 + j * 3 + 1] = 128;
                    costmap_mat_.data[i * costmap_mat_.cols * 3 + j * 3 + 2] = 128;
                }
                else if(costmap_[i][j] <= 127)
                {
                    costmap_mat_.data[i * costmap_mat_.cols * 3 + j * 3 + 0] = 0;
                    costmap_mat_.data[i * costmap_mat_.cols * 3 + j * 3 + 1] = 255;
                    costmap_mat_.data[i * costmap_mat_.cols * 3 + j * 3 + 2] = 0;
                }
                else
                {
                    costmap_mat_.data[i * costmap_mat_.cols * 3 + j * 3 + 0] = 128;
                    costmap_mat_.data[i * costmap_mat_.cols * 3 + j * 3 + 1] = 128;
                    costmap_mat_.data[i * costmap_mat_.cols * 3 + j * 3 + 2] = 128;
                }
                
            }
        }

        map_view_.PutFrame(map_mat_);
        costmap_view_.PutFrame(costmap_mat_);

        delay(41, past_time);
    }
}

void PathPlanner::BuildCostmap()
{
	std::vector<Vector2i> occupied_list;

    for (int i = 0; i < Constants::MAP_SIZE; i++)
    {
        for (int j = 0; j < Constants::MAP_SIZE; j++)
        {
            if (map_[i][j] < 64)
            {
                costmap_[i][j] = 254;
                occupied_list.push_back({i, j});
            }
            else costmap_[i][j] = 100;
        }
    }

    for (int i = 0; i < 320; i++) // 50
    {
        for (int j = 0; j < Constants::MAP_SIZE; j++)
        {
            if(j > 120 && j < Constants::MAP_SIZE - 120) continue;
            costmap_[i][j] = 254;
        }
    }
    for (int i = 320 - 45 + Constants::ROBOT_CENTER_OFFSET / Constants::MM_PER_PIXEL; i < Constants::MAP_SIZE; i++)
    {
        for (int j = 0; j < Constants::MAP_SIZE; j++)
        {
            costmap_[i][j] = 254;
        }
    }    

	kdtree_->build(occupied_list);
	double min_dist;
	for (int i = 0; i < Constants::MAP_SIZE; i++)
	{
		for (int j = 0; j < Constants::MAP_SIZE; j++)
		{
            if(costmap_[i][j] != 254)
			{
				kdtree_->nnSearch({ i, j }, &min_dist);

                if (min_dist <= Constants::ROBOT_RADIUS / 2.0)
					costmap_[i][j] = 253;
				else if (min_dist <= Constants::ROBOT_RADIUS * 0.8)
					costmap_[i][j] = 252;
				else if (min_dist <= Constants::INFLATION_RADIUS)
					costmap_[i][j] = std::round(Trans(Constants::ROBOT_RADIUS * 0.8, Constants::INFLATION_RADIUS, 251, 128, min_dist));
				else
                    costmap_[i][j] = Trans(Constants::INFLATION_RADIUS, Constants::INFLATION_RADIUS + 250 / double(Constants::MM_PER_PIXEL), 127, 0, min_dist);
			}
		}
	}
}

void PathPlanner::ShowRobotPosition(const Vector3d &pose)
{
    std::array<std::array<double, 2>, 4> vertices;

    GetRobotVertices(pose, vertices);
    for(int i = 0; i < 4; i++)
    {
        vertices[i][0] = std::round(Constants::ROBOT_X_OFFSET - vertices[i][0] / Constants::MM_PER_PIXEL);
        vertices[i][1] = std::round(Constants::ROBOT_Y_OFFSET + vertices[i][1] / Constants::MM_PER_PIXEL);
    }

    for (int i = 0; i < 4; i++)
        cv::line(
            map_mat_,
            cv::Point(vertices[i][1], vertices[i][0]), cv::Point(vertices[(i + 1) % 4][1], vertices[(i + 1) % 4][0]),
            cv::Scalar(0, 255, 0), 1);

    int x1, y1, x2, y2;
    x1 = std::round((vertices[0][0] + vertices[1][0] + vertices[2][0] + vertices[3][0]) / 4.0);
    y1 = std::round((vertices[0][1] + vertices[1][1] + vertices[2][1] + vertices[3][1]) / 4.0);
    x2 = std::round((vertices[0][0] + vertices[1][0]) / 2.0);
    y2 = std::round((vertices[0][1] + vertices[1][1]) / 2.0);
    cv::arrowedLine(map_mat_, cv::Point(y1, x1), cv::Point(y2, x2), cv::Scalar(0, 0, 255), 1, cv::LINE_AA, 0, 0.5);
}