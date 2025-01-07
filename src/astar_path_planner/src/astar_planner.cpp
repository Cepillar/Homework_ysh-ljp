#include <ros/ros.h>
#include <utility>
#include <vector>
#include <queue>
#include <cmath>
#include <Eigen/Dense>
#include "visualization_msgs/MarkerArray.h"
#include <geometry_msgs/Point.h>
#include "nav_msgs/Path.h"
#include <iostream>
#include <unordered_set>
#include <chrono>

#define OBS_RADIUS 0.2
struct Node {
    int x, y;        // 节点所在的网格坐标
    double g_cost;   // 从起点到当前节点的代价
    double h_cost;   // 从当前节点到终点的估计代价
    std::shared_ptr<Node> parent;    // 父节点，用于回溯路径

    Node(int x, int y, double g_cost, double h_cost, std::shared_ptr<Node> parent = nullptr)
            : x(x), y(y), g_cost(g_cost), h_cost(h_cost), parent(std::move(parent)) {}

    double f() const { return g_cost + h_cost; } // 总代价值

};
// 比较器，用于优先队列
struct cmp{
    bool operator()(std::shared_ptr<Node> a, std::shared_ptr<Node> b){
        return a->f() > b->f();
    }

};
struct GridMap {
    int width;
    int height;
    double map_max;
    double map_min;
    double grid_resolution;
    std::vector<std::vector<int>> grid; // 0: 空闲, 1: 占用

    GridMap(int w, int h, double map_min_, double map_max_, double res) : width(w), height(h), map_min(map_min_), map_max(map_max_), grid_resolution(res), grid(w, std::vector<int>(h, 0)) {}

    void markObstacle(double cx, double cy, double radius) {
        // radius = 0.2;
        // std::cout << width << height << map_max << map_min << grid_resolution << std::endl;
        int grid_cx = std::round((cx - map_min) / grid_resolution);
        int grid_cy = std::round((cy - map_min) / grid_resolution);
        int grid_radius = std::round(radius / grid_resolution);

        for (int i = -grid_radius; i <= grid_radius; ++i) {
            for (int j = -grid_radius; j <= grid_radius; ++j) {
                int nx = grid_cx + i;
                int ny = grid_cy + j;
                if (nx >= 0 && nx < width && ny >= 0 && ny < height &&
                    std::sqrt(i * i + j * j) <= grid_radius) {
                    grid[nx][ny] = 1;  // 标记为占用
                }
            }
        }
    }
};
class AStarPlanner {
public:
    AStarPlanner(int width, int height, double m_min, double m_max, double res) : width_(width), height_(height), map_min_(m_min), map_max_(m_max), grid_resolution_(res), grid_map_(width, height, map_min_, map_max_, grid_resolution_), num_of_obs_(0) {

    }

    void setObstacle(double cx, double cy, double radius) {
        num_of_obs_++;
        grid_map_.markObstacle(cx, cy, radius+OBS_RADIUS);
    }

    void printGridMap(){
        for(int i = 0; i < width_; i++){
            for(int j = 0; j < height_; j++){
                std::cout<<grid_map_.grid[i][j]<<" ";
            }
            std::cout<<std::endl;
        }
        std::cout<<"num of obstacles: "<<num_of_obs_<<std::endl;
    }

    // Helper function to find the nearest free grid
    std::pair<int, int> findNearestFreeGrid(int x, int y) {
        std::queue<std::pair<int, int>> q;
        std::vector<std::vector<bool>> visited(width_, std::vector<bool>(height_, false));
        q.push({x, y});
        visited[x][y] = true;

        // 八连通方向
        std::vector<std::pair<int, int>> directions = {
            {1, 0}, {0, 1}, {-1, 0}, {0, -1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}
        };

        while (!q.empty()) {
            auto [cx, cy] = q.front();
            q.pop();

            if (grid_map_.grid[cx][cy] == 0) { // 找到空闲栅格
                return {cx, cy};
            }

            for (const auto& dir : directions) {
                int nx = cx + dir.first;
                int ny = cy + dir.second;
                if (nx >= 0 && nx < width_ && ny >= 0 && ny < height_ && !visited[nx][ny]) {
                    visited[nx][ny] = true;
                    q.push({nx, ny});
                }
            }
        }

        throw std::runtime_error("No free grid found near the point");
    }
    // 计算路径转向代价
    double calculateTurningCost(const Node& current, const Node& previous) {
        // 确保 previous 节点有父节点
        if (previous.parent == nullptr) {
            return 0.0;  // 起始节点没有父节点，返回零代价
        }

        // 计算当前节点和上一个节点的方向向量
        Eigen::Vector2d current_direction(current.x - previous.x, current.y - previous.y);
        Eigen::Vector2d previous_direction(previous.x - previous.parent->x, previous.y - previous.parent->y);

        // 如果方向向量接近零，直接返回零代价（避免除零错误）
        if (current_direction.norm() < 1e-6 || previous_direction.norm() < 1e-6) {
            return 0.0;
        }

        // 计算两个方向向量的夹角
        double angle = std::atan2(current_direction.y(), current_direction.x()) -
                    std::atan2(previous_direction.y(), previous_direction.x());
        
        // 确保角度在 -PI 到 PI 之间
        angle = std::fmod(angle + M_PI, 2 * M_PI);
        if (angle < 0) {
            angle += 2 * M_PI;
        }
        angle -= M_PI;

        // 计算转向代价：角度差异越大，代价越高
        double turning_cost = std::abs(angle) > 0.1 ? std::abs(angle) : 0.0; // 可调节角度阈值
        return turning_cost;
    }

    // 修改 `getNeighbors` 函数，增加路径转向代价
    std::vector<Node> getNeighbors(const Node& current, std::shared_ptr<Node> parent) {
        std::vector<Node> neighbors;

        std::vector<std::pair<int, int>> directions = {
            {1, 0}, {0, 1}, {-1, 0}, {0, -1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}
        };

        for (const auto& dir : directions) {
            int nx = current.x + dir.first;
            int ny = current.y + dir.second;

            if (nx >= 0 && nx < width_ && ny >= 0 && ny < height_ && grid_map_.grid[nx][ny] == 0) {
                double g_cost = current.g_cost + std::sqrt(dir.first * dir.first + dir.second * dir.second);

                // 如果有父节点，则计算路径转向代价
                double turning_cost = 0.0;
                if (parent) {
                    turning_cost = calculateTurningCost(current, *parent);
                }

                neighbors.emplace_back(nx, ny, g_cost + turning_cost, 0.0); // h_cost 暂时为 0
            }
        }
        return neighbors;
    }
    // 在 `findPath` 中调用 `getNeighbors` 时修改传递的参数
    std::vector<Eigen::Vector2d> findPath(Eigen::Vector2d start, Eigen::Vector2d goal) {
        if (num_of_obs_ == 0) {
            return {};
        }

        auto gridStart = worldToGrid(start);
        auto gridGoal = worldToGrid(goal);

        // 检查起点或终点是否被障碍物占据
        if (grid_map_.grid[gridStart.first][gridStart.second] == 1) {
            std::cout << "Start point is occupied. Searching for the nearest free grid..." << std::endl;
            gridStart = findNearestFreeGrid(gridStart.first, gridStart.second);
            std::cout << "New start point: (" << gridStart.first << ", " << gridStart.second << ")" << std::endl;
        }

        if (grid_map_.grid[gridGoal.first][gridGoal.second] == 1) {
            std::cout << "Goal point is occupied. Searching for the nearest free grid..." << std::endl;
            gridGoal = findNearestFreeGrid(gridGoal.first, gridGoal.second);
            std::cout << "New goal point: (" << gridGoal.first << ", " << gridGoal.second << ")" << std::endl;
        }

        // 开放列表和关闭列表
        std::priority_queue<std::shared_ptr<Node>, std::vector<std::shared_ptr<Node>>, cmp> open_list;
        std::unordered_set<std::string> closed_list; // 用哈希表存储关闭节点

        open_list.push(std::make_shared<Node>(gridStart.first, gridStart.second, 0.0, heuristic(gridStart, gridGoal)));

        while (!open_list.empty()) {
            auto current_node = open_list.top();
            open_list.pop();

            std::string current_key = std::to_string(current_node->x) + "_" + std::to_string(current_node->y);
            if (closed_list.count(current_key)) {
                continue;
            }
            closed_list.insert(current_key);

            if (current_node->x == gridGoal.first && current_node->y == gridGoal.second) {
                std::cout<<"find path"<<std::endl;
                return reconstructPath(current_node);  // 找到路径
            }

            // 获取邻居节点时传递当前节点作为父节点
            for (auto& neighbor : getNeighbors(*current_node, current_node->parent)) {
                std::string neighbor_key = std::to_string(neighbor.x) + "_" + std::to_string(neighbor.y);
                if (closed_list.count(neighbor_key)) {
                    continue;
                }

                neighbor.h_cost = heuristic({neighbor.x, neighbor.y}, gridGoal);
                neighbor.parent = current_node;
                open_list.push(std::make_shared<Node>(neighbor));
            }
        }
        std::cout<<"failed to find path"<<std::endl;
        return {};  // 未找到路径
    }

    void reset(){
        num_of_obs_ = 0;
        grid_map_.grid = std::vector<std::vector<int>>(width_, std::vector<int>(height_, 0));
    }
private:

    // 计算启发式代价（使用欧几里得距离）
    double heuristic(const std::pair<int, int>& from, const std::pair<int, int>& to) {
        return std::sqrt(std::pow(from.first - to.first, 2) + std::pow(from.second - to.second, 2));
    }

    // 计算两节点之间的距离（用于邻居代价计算）
    double distance(const Node& a, const Node& b) {
        return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
    }

    // 从世界坐标转换到栅格坐标
    std::pair<int, int> worldToGrid(const Eigen::Vector2d& position) {
        int x = std::round((position.x() - map_min_) / grid_resolution_);
        int y = std::round((position.y() - map_min_) / grid_resolution_);
        return {x, y};
    }

    // 从栅格坐标转换到世界坐标（主要用于路径结果显示）
    Eigen::Vector2d gridToWorld(int x, int y) {
        double wx = x * grid_resolution_ + map_min_;
        double wy = y * grid_resolution_ + map_min_;
        return Eigen::Vector2d(wx, wy);
    }

    std::vector<Node> getNeighbors(const Node& current) {
        std::vector<Node> neighbors;
        std::vector<std::pair<int, int>> directions = {
            {1, 0}, {0, 1}, {-1, 0}, {0, -1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}
        };

        for (const auto& dir : directions) {
            int nx = current.x + dir.first;
            int ny = current.y + dir.second;

            if (nx >= 0 && nx < width_ && ny >= 0 && ny < height_ &&
                grid_map_.grid[nx][ny] == 0) { // 检查边界和占用状态
                double g_cost = current.g_cost + std::sqrt(dir.first * dir.first + dir.second * dir.second);
                neighbors.emplace_back(nx, ny, g_cost, 0.0);  // h_cost 暂时为 0
            }
        }
        return neighbors;
    }

    // 回溯路径
    std::vector<Eigen::Vector2d> reconstructPath(std::shared_ptr<Node> node) {
        std::vector<Eigen::Vector2d> path;
        while (node) {
            path.push_back(gridToWorld(node->x, node->y));
            node = node->parent;
        }
        std::reverse(path.begin(), path.end());
        reset();
        return path;
    }

    // 地图数据
    int width_, height_;
    double map_min_, map_max_, grid_resolution_;
    GridMap grid_map_; // 栅格地图，0: 空闲，1: 障碍物
    int num_of_obs_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "astar_planner");
    ros::NodeHandle nh;
    double map_min_, map_max_, grid_resolution_;
    double start_x_, start_y_, goal_x_, goal_y_;
    nh.param("astar_planner/map_min", map_min_, -5.0);
    nh.param("astar_planner/map_max", map_max_, 5.0);
    nh.param("astar_planner/grid_resolution", grid_resolution_, 0.1);
    nh.param("astar_planner/start_x", start_x_, -4.5);
    nh.param("astar_planner/start_y", start_y_, -4.5);
    nh.param("astar_planner/goal_x", goal_x_, 4.5);
    nh.param("astar_planner/goal_y", goal_y_, 4.5);

    // 地图参数
    int grid_width = std::round((map_max_ - map_min_) / grid_resolution_);
    int grid_height = grid_width;

    AStarPlanner planner(grid_width, grid_height, map_min_, map_max_, grid_resolution_);
    // 障碍物订阅
    ros::Subscriber obstacle_sub = nh.subscribe<visualization_msgs::MarkerArray>("obstacles", 1,
                                                                                 [&planner, &grid_resolution_, &map_min_](const visualization_msgs::MarkerArray::ConstPtr& msg) {
                                                                                     for (const auto& marker : msg->markers) {
                                                                                         planner.setObstacle(marker.pose.position.x, marker.pose.position.y, marker.scale.x / 2.0);
                                                                                     }
                                                                                 });



    // 发布路径
    ros::Rate rate(10);
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("path", 1);
    // 起点和终点参数
    Eigen::Vector2d start(start_x_, start_y_);
    Eigen::Vector2d goal(goal_x_, goal_y_);
    while (ros::ok()) {
        planner.reset();
//        // 等待障碍物加载
//        ros::Duration(1.0).sleep();
        ros::spinOnce();
        // 执行路径搜索
        auto start_time = std::chrono::high_resolution_clock::now();
        std::vector<Eigen::Vector2d> path = planner.findPath(start, goal);
        auto finish_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> time_cost = finish_time - start_time;
        
        // 路径可视化
        if (path.empty()){
            continue;
        }
        
        std::cout<<"find path"<<std::endl;
        std::cout << "A* time cost: " << time_cost.count() << "s\n";
        nav_msgs::Path path_msg;
        path_msg.header.frame_id = "map";
        path_msg.header.stamp = ros::Time::now();
        for (const auto& point : path) {
            geometry_msgs::PoseStamped pose;
            pose.pose.position.x = point.x();
            pose.pose.position.y = point.y();
            pose.pose.position.z = 0.0; // 平面路径，z 设置为 0
            path_msg.poses.push_back(pose);
        }
        path_pub.publish(path_msg);
        rate.sleep();
    }

    return 0;
}