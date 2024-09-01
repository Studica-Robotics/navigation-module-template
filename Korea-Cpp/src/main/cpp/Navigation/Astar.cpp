#include <cmath>
#include <queue>
#include <limits>

#include "Navigation/Astar.h"
#include "Util.h"
#include "Constants.h"

struct AstarNode
{
    Vector2i pose;
    int cost;
    AstarNode *parent_node;

    AstarNode(const Vector2i& pose, int cost = 0, AstarNode *parent_node = nullptr) :pose(pose), cost(cost), parent_node(parent_node) {};
};

Astar::Astar()
{
    visit_ = new bool*[Constants::MAP_SIZE];
    path_cost_ = new int*[Constants::MAP_SIZE];

    for (int i = 0; i < Constants::MAP_SIZE; i++)
    {
        visit_[i] = new bool[Constants::MAP_SIZE];
        path_cost_[i] = new int[Constants::MAP_SIZE];
    }
}

Astar::~Astar()
{
    for (int i = 0; i < Constants::MAP_SIZE; i++)
    {
        delete[] visit_[i];
        delete[] path_cost_[i];
    }
    delete[] visit_;
    delete[] path_cost_;
}

bool Astar::Update(const Vector2i& grid_start, const Vector2i& grid_goal, unsigned char** costmap_, std::vector<Vector2i>& path)
{
    if(!IsSafe(grid_start, Constants::MAP_SIZE) || !IsSafe(grid_goal, Constants::MAP_SIZE)) return false;

    for (int i = 0; i < Constants::MAP_SIZE; i++)
    {
        for (int j = 0; j < Constants::MAP_SIZE; j++)
        {
            visit_[i][j] = false;
            path_cost_[i][j] = std::numeric_limits<int>::max();
        }
    }

    path_cost_[grid_start[0]][grid_start[1]] = 0;

    std::queue<AstarNode*> astar_node_pointer;
    AstarNode *start_node = new AstarNode(grid_start);
    AstarNode *goal_node = new AstarNode(grid_goal);

    auto cmp = [](const AstarNode *left, const AstarNode *right) {return left->cost > right->cost; };
    std::priority_queue<AstarNode*, std::vector<AstarNode*>, decltype(cmp)> pq(cmp);
    pq.push(start_node);

    static const int move[8][3] = {
        {-1, -1, 14}, {-1, 0, 10}, {-1, 1, 14},
        { 0, -1, 10},              { 0, 1, 10},
        { 1, -1, 14}, { 1, 0, 10}, { 1, 1, 14},
    };

    while (true)
    {
        if(pq.empty()) return false;

        AstarNode *node = pq.top();

        if (visit_[node->pose[0]][node->pose[1]])
        {
            pq.pop();
            continue;
        }
        else
        {
            pq.pop();
            visit_[node->pose[0]][node->pose[1]] = true;
        }

        if (node->pose[0] == goal_node->pose[0] && node->pose[1] == goal_node->pose[1])
        {
            goal_node->cost = node->cost;
            goal_node->parent_node = node;
            break;
        }

        for (int i = 0; i < 8; i++)
        {
            Vector2i new_pose{ node->pose[0] + move[i][0], node->pose[1] + move[i][1] };
            if (
                    !IsSafe(new_pose, Constants::MAP_SIZE) || 
                    (GetDistance(grid_start, new_pose) > Constants::ROBOT_RADIUS && 252 <= costmap_[new_pose[0]][new_pose[1]] && costmap_[new_pose[0]][new_pose[1]] <= 254) ||
                    visit_[new_pose[0]][new_pose[1]]
                )
                continue;

            if (path_cost_[node->pose[0]][node->pose[1]] + move[i][2] < path_cost_[new_pose[0]][new_pose[1]])
            {
                int cost = path_cost_[node->pose[0]][node->pose[1]] + move[i][2] + std::round(GetDistance(goal_node->pose, new_pose)) + costmap_[new_pose[0]][new_pose[1]] * 3;

                AstarNode *new_node = new AstarNode(new_pose, cost, node);
                astar_node_pointer.push(new_node);

                path_cost_[new_node->pose[0]][new_node->pose[1]] = path_cost_[node->pose[0]][node->pose[1]] + move[i][2];
                pq.push(new_node);
            }
        }
    }
    AstarNode *node = goal_node;
    std::vector<Vector2i> tmp_path;
    while (node->parent_node != nullptr)
    {
        node = node->parent_node;
        tmp_path.push_back(node->pose);
    }
    path = tmp_path;

    delete start_node;
    delete goal_node;
    while(!astar_node_pointer.empty())
    {
        delete astar_node_pointer.front();
        astar_node_pointer.pop();
    }

    return true;
}