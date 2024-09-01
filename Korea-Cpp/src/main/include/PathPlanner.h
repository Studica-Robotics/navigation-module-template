#pragma once

#include <thread>
#include <vector>
#include <cameraserver/CameraServer.h>
#include <opencv2/core/core.hpp>

#include "Interface.h"
#include "KDTree.h"
#include "SLAM/OcuupancyGridMap.h"
#include "Navigation/DynamicWindowApproach.h"
#include "Navigation/MCL.h"

class PathPlanner
{
public:
    PathPlanner();
    ~PathPlanner();
    
private:
    const double kAlmostZero = 1e-6;

    std::thread dwa_thread_, astar_thread_, mcl_thread_, map_view_thread_, occupancy_grid_map_thread_;

    cs::CvSource map_view_, costmap_view_;
    cv::Mat map_mat_, costmap_mat_;

    unsigned char **map_, **costmap_;
    KDTree<Vector2i> *kdtree_;
    OcuupancyGridMap *occupancy_grid_map_;
    DynamicWindowApproach *dwa_;
    MCL *mcl_;

    bool mcl_running_, occupancy_running_, astar_running_;

    bool turn_flg_;
    int stop_cnt_;

    Vector3d current_goal_;
    Vector2i local_goal_;
    std::vector<Vector2i> path_;
    Traj traj_;

    Vector5d control_;
    
    void SetPlannerProperties();
    void DWAUpdate(Vector5d &control);
    void TireUpdate();
    void DWAThread();
    void AstarThread();
    void MCLThread();

    void MapViewThread();

    void BuildCostmap();

    void ShowRobotPosition(const Vector3d &pose);
};
