#include "backward.hpp"
#include "PUTN_planner.h"
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <rover_msgs/roverGoalStatus.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <string>

using namespace std;
using namespace std_msgs;
using namespace Eigen;
using namespace PUTN;
using namespace PUTN::visualization;
using namespace PUTN::planner;

namespace backward
{
backward::SignalHandling sh;
}

// ros related
ros::Subscriber wp_sub;
ros::Subscriber goal_sub;
ros::Subscriber local_goal_status_sub;

ros::Publisher grid_map_vis_pub;
ros::Publisher path_vis_pub;
ros::Publisher goal_vis_pub;
ros::Publisher surf_vis_pub;
ros::Publisher tree_vis_pub;
ros::Publisher path_interpolation_pub;
ros::Publisher tree_tra_pub;
ros::Publisher global_goal_status_pub;

// indicate whether the robot has a moving goal
bool has_goal = false;
bool is_planning = false;
bool waiting_for_local_complete = false;

// simulation param from launch file
double resolution;
double goal_thre;
double step_size;
double h_surf_car;
double max_initial_time;
double radius_fit_plane;
FitPlaneArg fit_plane_arg;
double neighbor_radius;
string map_file_path;

// 路径平滑参数
double min_turn_radius;
double fillet_clearance;
double fillet_ds;
bool use_clothoid;
double clothoid_length_factor;  // 新增：Clothoid长度因子
double max_curvature_rate;      // 新增：最大曲率变化率
double smoothing_fallback_threshold; // 新增：平滑失败回退阈值

// useful global variables
Vector3d start_pt;
Vector3d target_pt;
Vector3d last_target_pt;
World* world = NULL;
PFRRTStar* pf_rrt_star = NULL;

// 时间管理
ros::Time last_goal_time;
ros::Time last_planning_time;
const double MIN_REPLAN_INTERVAL = 0.5;
const double GOAL_STABILIZE_TIME = 0.3;

// function declaration
void rcvWaypointsCallback(const nav_msgs::Path& wp);
void pubInterpolatedPath(const vector<Node*>& solution, ros::Publisher* _path_interpolation_pub);
void pubSmoothedPath(const vector<Node*>& solution, ros::Publisher* _path_interpolation_pub);
void findSolution();
void callPlanner();
void rcvLocalStatusCallback(const rover_msgs::roverGoalStatus& status);
bool isNewGoal(const Vector3d& new_goal);
void resetPlanner();
bool loadMapFromFile(const string& pcd_file_path);
bool validateSmoothingParams(); // 新增：参数验证

/**
 * @brief 验证平滑参数的合理性
 */
bool validateSmoothingParams()
{
    if (min_turn_radius <= 0.0)
    {
        ROS_ERROR("Invalid min_turn_radius: %.3f (must be > 0)", min_turn_radius);
        return false;
    }
    
    if (fillet_clearance < 0.0)
    {
        ROS_ERROR("Invalid fillet_clearance: %.3f (must be >= 0)", fillet_clearance);
        return false;
    }
    
    if (fillet_ds <= 0.0 || fillet_ds > min_turn_radius)
    {
        ROS_ERROR("Invalid fillet_ds: %.3f (must be > 0 and <= min_turn_radius)", fillet_ds);
        return false;
    }
    
    if (use_clothoid)
    {
        if (clothoid_length_factor <= 0.0 || clothoid_length_factor > 2.0)
        {
            ROS_WARN("Clothoid length factor %.3f out of recommended range [0.1, 2.0], adjusting to 0.4", 
                     clothoid_length_factor);
            clothoid_length_factor = 0.4;
        }
        
        if (max_curvature_rate <= 0.0)
        {
            ROS_ERROR("Invalid max_curvature_rate: %.3f (must be > 0)", max_curvature_rate);
            return false;
        }
    }
    
    return true;
}

template <typename T>
void loadParamCompat(ros::NodeHandle& nh,
                     const std::string& primary_key,
                     const std::string& legacy_key,
                     T& out_value,
                     const T& default_value)
{
    if (nh.hasParam(primary_key))
    {
        nh.param(primary_key, out_value, default_value);
        return;
    }

    if (nh.hasParam(legacy_key))
    {
        nh.param(legacy_key, out_value, default_value);
        ROS_WARN("Using legacy parameter key '%s'. Please migrate to '%s'.",
                 legacy_key.c_str(), primary_key.c_str());
        return;
    }

    out_value = default_value;
}

/**
 *@brief 从PCD文件加载地图
 */
bool loadMapFromFile(const string& pcd_file_path)
{
    ROS_INFO("Loading map from PCD file: %s", pcd_file_path.c_str());
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    // 加载PCD文件
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file_path, *cloud) == -1)
    {
        ROS_ERROR("Failed to load PCD file: %s", pcd_file_path.c_str());
        return false;
    }
    
    if (cloud->points.empty())
    {
        ROS_ERROR("Loaded empty point cloud from: %s", pcd_file_path.c_str());
        return false;
    }
    
    ROS_INFO("Successfully loaded %zu points from PCD file", cloud->points.size());
    
    // 初始化网格地图
    world->initGridMap(*cloud);
    
    // 设置障碍物
    int obstacle_count = 0;
    for (const auto& pt : cloud->points)
    {
        Vector3d obstacle(pt.x, pt.y, pt.z);
        world->setObs(obstacle);
        obstacle_count++;
    }
    
    ROS_INFO("Map initialized successfully with %d obstacle points", obstacle_count);
    ROS_INFO("Map bounds: [%.2f, %.2f, %.2f] to [%.2f, %.2f, %.2f]",
             world->getLowerBound()(0), world->getLowerBound()(1), world->getLowerBound()(2),
             world->getUpperBound()(0), world->getUpperBound()(1), world->getUpperBound()(2));
    
    // 发布地图可视化
    sensor_msgs::PointCloud2 pc2;
    pcl::toROSMsg(*cloud, pc2);
    pc2.header.frame_id = "camera_init";         
    pc2.header.stamp = ros::Time::now();
    grid_map_vis_pub.publish(pc2);
    visWorld(world, &grid_map_vis_pub);
    
    return true;
}

/**
 *@brief 检查是否是新目标点
 */
bool isNewGoal(const Vector3d& new_goal)
{
    double dist = (new_goal - last_target_pt).norm();
    return dist > 0.1;
}

/**
 *@brief 重置规划器状态
 */
void resetPlanner()
{
    ROS_INFO("Resetting global planner state");
    is_planning = false;
    waiting_for_local_complete = false;
    
    // 清除可视化
    visPath({}, &path_vis_pub);
    visSurf({}, &surf_vis_pub);
}

/**
 *@brief receive goal from rviz
 */
void rcvWaypointsCallback(const nav_msgs::Path& wp)
{
    if (wp.poses.empty())
    {
        ROS_WARN("Received empty waypoint message");
        return;
    }
    
    Vector3d new_target = Vector3d(wp.poses[0].pose.position.x, 
                                    wp.poses[0].pose.position.y, 
                                    wp.poses[0].pose.position.z);
    
    if (!isNewGoal(new_target))
    {
        ROS_INFO("Same goal received, ignoring");
        return;
    }
    
    has_goal = true;
    target_pt = new_target;
    last_target_pt = target_pt;
    last_goal_time = ros::Time::now();
    
    resetPlanner();
    
    ROS_INFO("Receive new planning target from rviz: [%.2f, %.2f, %.2f]",
             target_pt(0), target_pt(1), target_pt(2));
    
    // 发布目标状态
    rover_msgs::roverGoalStatus status;
    status.status = rover_msgs::roverGoalStatus::ACTIVE;
    global_goal_status_pub.publish(status);
}

/**
 *@brief 接收来自多点导航插件的目标点
 */
void rcvGoalCallback(const geometry_msgs::PoseStamped& goal)
{
    Vector3d new_target = Vector3d(goal.pose.position.x, 
                                    goal.pose.position.y, 
                                    goal.pose.position.z);
    
    if (!isNewGoal(new_target))
    {
        ROS_INFO("Same goal received, ignoring");
        return;
    }
    
    ros::Time current_time = ros::Time::now();
    if ((current_time - last_goal_time).toSec() < GOAL_STABILIZE_TIME)
    {
        ROS_WARN("New goal received too quickly, waiting for stabilization");
        return;
    }
    
    has_goal = true;
    target_pt = new_target;
    last_target_pt = target_pt;
    last_goal_time = current_time;
    
    resetPlanner();
    
    ROS_INFO("Receive new planning target from multi-goal plugin: [%.2f, %.2f, %.2f]",
             target_pt(0), target_pt(1), target_pt(2));
    
    rover_msgs::roverGoalStatus status;
    status.status = rover_msgs::roverGoalStatus::ACTIVE;
    global_goal_status_pub.publish(status);
}

/**
 *@brief 接收局部规划器状态
 */
void rcvLocalStatusCallback(const rover_msgs::roverGoalStatus& status)
{
    if (status.status == rover_msgs::roverGoalStatus::SUCCEEDED)
    {
        ROS_INFO("Local planner reported success, ready for next goal");
        waiting_for_local_complete = false;
        
        visPath({}, &path_vis_pub);
        visSurf({}, &surf_vis_pub);
        
        rover_msgs::roverGoalStatus global_status;
        global_status.status = rover_msgs::roverGoalStatus::SUCCEEDED;
        global_goal_status_pub.publish(global_status);
        
        has_goal = false;
    }
    else if (status.status == rover_msgs::roverGoalStatus::ABORTED)
    {
        ROS_WARN("Local planner aborted, resetting global planner");
        waiting_for_local_complete = false;
        has_goal = false;
        resetPlanner();
    }
}

/**
 *@brief 发布经过平滑处理的路径
 */
void pubSmoothedPath(const vector<Node*>& solution, ros::Publisher* path_interpolation_pub)
{
    if (path_interpolation_pub == NULL || solution.empty())
    {
        ROS_WARN("Cannot publish smoothed path: null publisher or empty solution");
        return;
    }
    
    if (solution.size() < 3)
    {
        ROS_INFO("Path too short for smoothing (%lu nodes), using linear interpolation", 
                 solution.size());
        pubInterpolatedPath(solution, path_interpolation_pub);
        return;
    }
    
    ROS_INFO("Applying %s smoothing to path with %lu nodes", 
             use_clothoid ? "Clothoid" : "circular fillet", solution.size());
    
    // 配置平滑参数，使用实际读取的参数
    FilletParams fillet_params;
    fillet_params.R_min = min_turn_radius;
    fillet_params.clearance = fillet_clearance;
    fillet_params.ds = fillet_ds;
    fillet_params.use_clothoid = use_clothoid;
    
    try 
    {
        auto smoothed_traj = pf_rrt_star->smoothWithFillets(solution, world, fillet_params);
        
        // 修正判断条件：只要不为空就是成功
        if (!smoothed_traj.empty())
        {
            Float32MultiArray msg;
            msg.data.clear();
            msg.data.reserve(smoothed_traj.size() * 5);
            
            for (const auto& point : smoothed_traj)
            {
                msg.data.push_back(point(0)); // x
                msg.data.push_back(point(1)); // y
                msg.data.push_back(point(2)); // z
                msg.data.push_back(point(3)); // yaw
                msg.data.push_back(point(4)); // kappa
            }
            
            path_interpolation_pub->publish(msg);
            
            // 计算路径统计信息
            double max_kappa = 0.0;
            for (const auto& point : smoothed_traj)
            {
                max_kappa = std::max(max_kappa, std::abs(point(4)));
            }
            
            ROS_INFO("Published smoothed path: %lu points, max curvature: %.4f (1/m)", 
                     smoothed_traj.size(), max_kappa);
        }
        else
        {
            ROS_WARN("Smoothing failed due to obstacles, using linear interpolation");
            pubInterpolatedPath(solution, path_interpolation_pub);
        }
    }
    catch (const std::exception& e)
    {
        ROS_ERROR("Exception during path smoothing: %s. Using linear interpolation.", e.what());
        pubInterpolatedPath(solution, path_interpolation_pub);
    }
}

/**
 *@brief Linearly interpolate the generated path (保持原有功能作为备用)
 */
void pubInterpolatedPath(const vector<Node*>& solution, ros::Publisher* path_interpolation_pub)
{
    if (path_interpolation_pub == NULL || solution.empty())
        return;
        
    Float32MultiArray msg;
    msg.data.clear();
    
    for (size_t i = 0; i < solution.size(); i++)
    {
        if (i == solution.size() - 1)
        {
            msg.data.push_back(solution[i]->position_(0));
            msg.data.push_back(solution[i]->position_(1));
            msg.data.push_back(solution[i]->position_(2));
            msg.data.push_back(0.0); // yaw
            msg.data.push_back(0.0); // kappa
        }
        else
        {
            size_t interpolation_num = std::max(1, 
                (int)(EuclideanDistance(solution[i + 1], solution[i]) / 0.1));
            Vector3d diff_pt = solution[i + 1]->position_ - solution[i]->position_;
            
            for (size_t j = 0; j < interpolation_num; j++)
            {
                Vector3d interpt = solution[i]->position_ + diff_pt * (double)j / interpolation_num;
                msg.data.push_back(interpt(0));
                msg.data.push_back(interpt(1));
                msg.data.push_back(interpt(2));
                msg.data.push_back(0.0); // yaw
                msg.data.push_back(0.0); // kappa
            }
        }
    }
    
    path_interpolation_pub->publish(msg);
    ROS_INFO("Published linear interpolated path with %lu points", msg.data.size() / 5);
}

/**
 *@brief PF-RRT* planning
 */
void findSolution()
{
    if (is_planning)
    {
        ROS_WARN("Already planning, skipping this cycle");
        return;
    }
    
    ros::Time current_time = ros::Time::now();
    if ((current_time - last_planning_time).toSec() < MIN_REPLAN_INTERVAL)
    {
        return;
    }
    
    is_planning = true;
    last_planning_time = current_time;
    
    printf("=========================================================================\n");
    ROS_INFO("Start calling PF-RRT* for target [%.2f, %.2f, %.2f]", 
             target_pt(0), target_pt(1), target_pt(2));
    
    Path solution = Path();
    
    pf_rrt_star->initWithGoal(start_pt, target_pt);

    if (pf_rrt_star->state() == Invalid)
    {
        ROS_WARN("The start point can't be projected. Unable to start PF-RRT*!");
        is_planning = false;
        
        rover_msgs::roverGoalStatus status;
        status.status = rover_msgs::roverGoalStatus::ABORTED;
        global_goal_status_pub.publish(status);
        return;
    }
    else if (pf_rrt_star->state() == Global)
    {
        ROS_INFO("Starting PF-RRT* algorithm at the state of global planning");
        int max_iter = 5000;
        double max_time = 100.0;
        
        if (waiting_for_local_complete)
        {
            max_time = 50.0;
        }

        while (solution.type_ == Path::Empty && max_time < max_initial_time)
        {
            solution = pf_rrt_star->planner(max_iter, max_time);
            if (solution.type_ == Path::Empty)
            {
                max_time += 50.0;
                ROS_INFO("No solution found, increasing max_time to %.1f", max_time);
            }
        }

        if (!solution.nodes_.empty())
        {
            ROS_INFO("Got a global path with %lu nodes!", solution.nodes_.size());
            waiting_for_local_complete = true;
        }
        else
        {
            ROS_WARN("No solution found after maximum attempts!");
            
            rover_msgs::roverGoalStatus status;
            status.status = rover_msgs::roverGoalStatus::ABORTED;
            global_goal_status_pub.publish(status);
        }
    }
    else
    {
        ROS_INFO("Starting PF-RRT* algorithm at the state of rolling planning");
        int max_iter = 1500;
        double max_time = 50.0;

        solution = pf_rrt_star->planner(max_iter, max_time);

        if (!solution.nodes_.empty())
        {
            ROS_INFO("Got a sub path with %lu nodes!", solution.nodes_.size());
        }
        else
        {
            ROS_WARN("No solution found!");
        }
    }
    
    ROS_INFO("End calling PF-RRT*");
    printf("=========================================================================\n");

    // 发布平滑路径
    if (!solution.nodes_.empty())
    {
        pubSmoothedPath(solution.nodes_, &path_interpolation_pub);
        
        // 保持可视化
        visPath(solution.nodes_, &path_vis_pub);
        visSurf(solution.nodes_, &surf_vis_pub);
    }
    else
    {
        visPath({}, &path_vis_pub);
    }

    if (solution.type_ == Path::Global && 
        EuclideanDistance(pf_rrt_star->origin(), pf_rrt_star->target()) < goal_thre)
    {
        ROS_INFO("Path found to goal region, waiting for local planner to complete");
    }
    
    is_planning = false;
}

/**
 *@brief Main planning loop
 */
void callPlanner()
{
    static double init_time_cost = 0.0;
    
    if (!has_goal && !waiting_for_local_complete && init_time_cost < 1000)
    {
        timeval start;
        gettimeofday(&start, NULL);
        pf_rrt_star->initWithoutGoal(start_pt);
        timeval end;
        gettimeofday(&end, NULL);
        init_time_cost = 1000 * (end.tv_sec - start.tv_sec) + 0.001 * (end.tv_usec - start.tv_usec);
        
        if (pf_rrt_star->state() == WithoutGoal)
        {
            int max_iter = 550;
            double max_time = 100.0;
            pf_rrt_star->planner(max_iter, max_time);
            ROS_INFO_THROTTLE(5.0, "Expanding tree, current size: %d", (int)(pf_rrt_star->tree().size()));
        }
        else
        {
            ROS_WARN_THROTTLE(5.0, "Cannot expand tree, start point can't be projected");
        }
    }
    else if (has_goal)
    {
        findSolution();
        init_time_cost = 0.0;
    }
    else if (waiting_for_local_complete)
    {
        ROS_INFO_THROTTLE(2.0, "Waiting for local planner to complete current goal");
    }
    else
    {
        ROS_INFO_THROTTLE(10.0, "Tree expansion stopped. Current size: %d", 
                          (int)(pf_rrt_star->tree().size()));
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "global_planning_node");
    ros::NodeHandle nh("~");

    // 订阅话题
    wp_sub = nh.subscribe("waypoints", 1, rcvWaypointsCallback);
    goal_sub = nh.subscribe("/cur_goal", 1, rcvGoalCallback);
    local_goal_status_sub = nh.subscribe("/cur_local_goal_status", 1, rcvLocalStatusCallback);

    // 发布话题
    grid_map_vis_pub = nh.advertise<sensor_msgs::PointCloud2>("grid_map_vis", 1, true);
    path_vis_pub = nh.advertise<visualization_msgs::Marker>("path_vis", 20);
    goal_vis_pub = nh.advertise<visualization_msgs::Marker>("goal_vis", 1);
    surf_vis_pub = nh.advertise<sensor_msgs::PointCloud2>("surf_vis", 100);
    tree_vis_pub = nh.advertise<visualization_msgs::Marker>("tree_vis", 1);
    tree_tra_pub = nh.advertise<std_msgs::Float32MultiArray>("tree_tra", 1);
    path_interpolation_pub = nh.advertise<std_msgs::Float32MultiArray>("/surf_predict_pub", 1000);
    global_goal_status_pub = nh.advertise<rover_msgs::roverGoalStatus>("/cur_global_goal_status", 1);

    // 读取原有参数
    nh.param("map/resolution", resolution, 0.1);
    nh.param("planning/goal_thre", goal_thre, 1.0);
    nh.param("planning/step_size", step_size, 0.2);
    nh.param("planning/h_surf_car", h_surf_car, 0.4);
    nh.param("planning/neighbor_radius", neighbor_radius, 1.0);
    nh.param("planning/w_fit_plane", fit_plane_arg.w_total_, 0.4);
    nh.param("planning/w_flatness", fit_plane_arg.w_flatness_, 4000.0);
    nh.param("planning/w_slope", fit_plane_arg.w_slope_, 0.4);
    nh.param("planning/w_sparsity", fit_plane_arg.w_sparsity_, 0.4);
    nh.param("planning/ratio_min", fit_plane_arg.ratio_min_, 0.25);
    nh.param("planning/ratio_max", fit_plane_arg.ratio_max_, 0.4);
    nh.param("planning/conv_thre", fit_plane_arg.conv_thre_, 0.1152);
    nh.param("planning/radius_fit_plane", radius_fit_plane, 1.0);
    nh.param("planning/max_initial_time", max_initial_time, 1000.0);
    
    // 读取路径平滑相关参数（兼容旧命名：planning/*）
    loadParamCompat<double>(nh, "smoothing/min_turn_radius", "planning/min_turn_radius", min_turn_radius, 0.50);
    loadParamCompat<double>(nh, "smoothing/fillet_clearance", "planning/fillet_clearance", fillet_clearance, 0.20);
    loadParamCompat<double>(nh, "smoothing/fillet_ds", "planning/fillet_ds", fillet_ds, 0.05);
    loadParamCompat<bool>(nh, "smoothing/use_clothoid", "planning/use_clothoid", use_clothoid, false);
    loadParamCompat<double>(nh, "smoothing/clothoid_length_factor", "planning/clothoid_length_factor", clothoid_length_factor, 0.4);
    loadParamCompat<double>(nh, "smoothing/max_curvature_rate", "planning/max_curvature_rate", max_curvature_rate, 2.0);
    loadParamCompat<double>(nh, "smoothing/fallback_threshold", "planning/fallback_threshold", smoothing_fallback_threshold, 0.5);
    
    // 验证平滑参数
    if (!validateSmoothingParams())
    {
        ROS_ERROR("Invalid smoothing parameters, exiting...");
        return -1;
    }
    
    // 读取地图文件路径参数
    if (!nh.getParam("map/pcd_file_path", map_file_path))
    {
        ROS_ERROR("Missing required parameter: map/pcd_file_path");
        return -1;
    }

    // 初始化
    world = new World(resolution);
    pf_rrt_star = new PFRRTStar(h_surf_car, world);

    // 设置PF-RRT*参数
    pf_rrt_star->setGoalThre(goal_thre);
    pf_rrt_star->setStepSize(step_size);
    pf_rrt_star->setFitPlaneArg(fit_plane_arg);
    pf_rrt_star->setFitPlaneRadius(radius_fit_plane);
    pf_rrt_star->setNeighborRadius(neighbor_radius);

    pf_rrt_star->goal_vis_pub_ = &goal_vis_pub;
    pf_rrt_star->tree_vis_pub_ = &tree_vis_pub;
    pf_rrt_star->tree_tra_pub_ = &tree_tra_pub;

    // 初始化时间戳
    last_goal_time = ros::Time::now();
    last_planning_time = ros::Time::now();

    // 输出参数信息
    ROS_INFO("=== Path Smoothing Parameters ===");
    ROS_INFO("  Smoothing method: %s", use_clothoid ? "Clothoid curves" : "Circular fillets");
    ROS_INFO("  Min turn radius: %.3f m", min_turn_radius);
    ROS_INFO("  Safety clearance: %.3f m", fillet_clearance);
    ROS_INFO("  Effective radius: %.3f m", min_turn_radius + fillet_clearance);
    ROS_INFO("  Sampling distance: %.3f m", fillet_ds);
    if (use_clothoid)
    {
        ROS_INFO("  Clothoid length factor: %.3f", clothoid_length_factor);
        ROS_INFO("  Max curvature rate: %.3f rad/m²", max_curvature_rate);
    }
    ROS_INFO("=================================");

    // 加载地图文件
    ROS_INFO("Attempting to load map from: %s", map_file_path.c_str());
    if (!loadMapFromFile(map_file_path))
    {
        ROS_ERROR("Failed to load map file, exiting...");
        return -1;
    }

    // 创建tf2的Buffer和TransformListener
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    ros::Rate rate(10);
    bool tf_ready = false;
    
    while (ros::ok())
    {
        // 更新机器人位置
        geometry_msgs::TransformStamped transformStamped;
        try
        {
            transformStamped = tfBuffer.lookupTransform("camera_init", "aft_mapped", 
                                                       ros::Time(0), ros::Duration(0.1));
            start_pt << transformStamped.transform.translation.x, 
                        transformStamped.transform.translation.y, 
                        transformStamped.transform.translation.z;
            
            if (!tf_ready)
            {
                ROS_INFO("TF ready, robot position: [%.2f, %.2f, %.2f]", 
                         start_pt(0), start_pt(1), start_pt(2));
                tf_ready = true;
            }
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN_THROTTLE(1.0, "TF lookup failed: %s", ex.what());
            ros::Duration(0.1).sleep();
            continue;
        }

        ros::spinOnce();
        
        if (tf_ready)
        {
            callPlanner();
        }
        
        rate.sleep();
    }
    
    delete pf_rrt_star;
    delete world;
    
    return 0;
}
