#ifndef PUTN_PLANNER_H
#define PUTN_PLANNER_H

#include <utility>
#include "PUTN_vis.h"
#include <cmath>

namespace PUTN
{
namespace planner
{

/**
 * @brief represents four different working states of the planner.
 */
enum PlanningState{Global,Roll,WithoutGoal,Invalid}; 

// ========== 路径平滑相关结构体 ==========

/**
 * @brief 路径平滑参数结构体
 */
struct FilletParams {
    double R_min;        // 最小转弯半径（必需）
    double clearance;    // 与障碍的安全余量
    double ds;           // 采样间距
    bool   use_clothoid; // 是否使用 Clothoid 曲线
    double clothoid_length_factor;  // Clothoid长度因子
    double max_curvature_rate;      // 最大曲率变化率
};

/**
 * @brief 包含曲率信息的2D位姿结构体
 */
struct KappaPose2D {
    Eigen::Vector2d p;   // 2D位置
    double yaw;          // 航向角
    double kappa;        // 曲率（圆弧=±1/R，直线=0）
};

/**
 * @brief Clothoid 曲线参数结构体
 */
struct ClothoidParams {
    double x0, y0;        // 起点坐标
    double theta0;        // 起点切线角度
    double kappa0;        // 起点曲率
    double dk;            // 曲率变化率
    double length;        // 曲线长度
};

/**
 * @brief Fresnel 积分结果结构体
 */
struct FresnelIntegrals {
    double C, S;  // Fresnel 余弦积分和正弦积分
};

/**
 * @brief Clothoid 曲线计算类
 */
class ClothoidCurve {
private:
    /**
     * @brief Fresnel 积分的级数展开近似计算
     * @param t 积分上限参数
     * @return Fresnel 积分结果
     */
    FresnelIntegrals computeFresnel(double t) {
        FresnelIntegrals result;
        
        // 使用级数展开计算 Fresnel 积分
        double t2 = t * t;
        double t4 = t2 * t2;
        double t8 = t4 * t4;
        
        // Fresnel 余弦积分 C(t)
        result.C = t - t4*t/10.0 + t8*t/216.0 - t4*t8*t/9360.0;
        
        // Fresnel 正弦积分 S(t)  
        result.S = t2*t/6.0 - t4*t2*t/120.0 + t8*t2*t/5040.0 - t4*t8*t2*t/362880.0;
        
        return result;
    }

public:
    /**
     * @brief 计算 Clothoid 曲线上的点
     * @param params Clothoid 参数
     * @param ds 采样间距
     * @return 曲线上的采样点列表
     */
    std::vector<KappaPose2D> generateClothoidPoints(const ClothoidParams& params, double ds) {
        std::vector<KappaPose2D> points;
        
        int num_points = std::max(1, static_cast<int>(params.length / ds));
        
        for (int i = 0; i <= num_points; i++) {
            double s = (double)i / num_points * params.length;
            
            // Clothoid 参数
            double tau = std::sqrt(std::abs(params.dk) * 0.5 / M_PI) * s;
            
            FresnelIntegrals fresnel = computeFresnel(tau);
            
            // 根据曲率变化率的符号调整
            double sign = (params.dk >= 0) ? 1.0 : -1.0;
            double scale = std::sqrt(2.0 * M_PI / std::abs(params.dk + 1e-10));
            
            // 计算相对于起点的位移
            double dx = scale * fresnel.C;
            double dy = sign * scale * fresnel.S;
            
            // 旋转到起点切线方向
            double cos_theta = std::cos(params.theta0);
            double sin_theta = std::sin(params.theta0);
            
            KappaPose2D point;
            point.p.x() = params.x0 + dx * cos_theta - dy * sin_theta;
            point.p.y() = params.y0 + dx * sin_theta + dy * cos_theta;
            point.yaw = params.theta0 + params.kappa0 * s + 0.5 * params.dk * s * s;
            point.kappa = params.kappa0 + params.dk * s;
            
            points.push_back(point);
        }
        
        return points;
    }
};

class PFRRTStar
{
public:
    //ros related,which will not work unless the user assigns.
    ros::Publisher* tree_vis_pub_=NULL;
    ros::Publisher* goal_vis_pub_=NULL;
    ros::Publisher* tree_tra_pub_=NULL;

    PFRRTStar();
    PFRRTStar(const double &height,World* world);//Input the height of the robot center,and the array of grid map.
    ~PFRRTStar();

    /** 
     * @brief Set the origin and target for the planner.According to whether they are successfully projected to the
     *        surface,the planner will convert to 3 different working states:Global,Roll,Invalid.
     * @param Vector3d start_point
     * @param Vector3d end_point
     * @return void
     * @note In fact,only the x and y dimensions of the input point are used
     */
    void initWithGoal(const Eigen::Vector3d &start_pos,const Eigen::Vector3d &end_pos);

     /** 
     * @brief Only set the origin for the planner.According to whether it's successfully projected to the
     *        surface,the planner will convert to 2 different working states:WithoutGoal,Invalid.
     * @param Vector3d start_point
     * @return void
     */
    void initWithoutGoal(const Eigen::Vector3d &start_pos);

    /**
     * @brief Expand the tree to search the solution,and stop after reaching the max iterations or max time.
     * @param int max_iter
     * @param float max_time
     * @return Path(The solution to the goal)
     */
    Path planner(const int &max_iter,const double &max_time);

    int getCurrentIterations(){return curr_iter_;}

    double getCurrentTime(){return curr_time_;}

    void setFitPlaneRadius(const float &radius){radius_fit_plane_=radius;}
    
    void setFitPlaneArg(const FitPlaneArg &fit_plane_arg){fit_plane_arg_=fit_plane_arg;}

    void setStepSize(const double &step_size){step_size_=step_size;}

    void setGoalThre(const double &threshold){goal_threshold_=threshold;}

    void setGoalBiased(const double &goal_biased){goal_biased_=goal_biased;}

    void setNeighborRadius(const double &neighbor_radius){neighbor_radius_=neighbor_radius;}

    Node* origin(){return node_origin_;}
    Node* target(){return node_target_;}

    std::vector<Node*> tree(){return tree_;}
    Path path(){return path_;}
    PlanningState state(){return planning_state_;}

    /**
     * @brief Project a point to the surface and then fit a local plane on it.Generate a new node based on the plane.
     * @param Vector2d 
     * @return Node*
     */
    Node* fitPlane(const Eigen::Vector2d &p_original);
    Node* fitPlane(const Eigen::Vector3d &p_original){return fitPlane(project2plane(p_original));}

    /**
     * @brief According to the init coordinates stored by the node,update its information about plane and position.
     * @param Node*
     * @return void
     * @note Unlike the above function,it doesn't create new nodes,but updates the existing node
     */
    void fitPlane(Node* node);

    // ========== 路径平滑功能声明 ==========
    
    /**
     * @brief 角度规范化到 [-π, π]
     * @param a 输入角度
     * @return 规范化后的角度
     */
    double wrapYaw(double a);

    /**
     * @brief 为单个拐角生成圆弧倒角轨迹
     * @param v1 前一个路径点
     * @param v2 拐角顶点
     * @param v3 后一个路径点
     * @param R 倒角半径
     * @param ds 采样间距
     * @return 倒角轨迹点列表（包含位置、航向角、曲率）
     */
    std::vector<KappaPose2D> makeCornerFillet(const Eigen::Vector2d& v1,
                                             const Eigen::Vector2d& v2,
                                             const Eigen::Vector2d& v3,
                                             double R, double ds);

    /**
     * @brief 为单个拐角生成 Clothoid 曲线平滑轨迹
     * @param v1 前一个路径点
     * @param v2 拐角顶点
     * @param v3 后一个路径点
     * @param R 参考转弯半径
     * @param ds 采样间距
     * @param length_factor Clothoid长度因子
     * @param max_curvature_rate 最大曲率变化率
     * @return Clothoid 轨迹点列表（包含位置、航向角、曲率）
     */
    std::vector<KappaPose2D> makeClothoidFillet(const Eigen::Vector2d& v1,
                                               const Eigen::Vector2d& v2,
                                               const Eigen::Vector2d& v3,
                                               double R, double ds,
                                               double length_factor = 0.4,
                                               double max_curvature_rate = 2.0);

    /**
     * @brief 对整条路径进行平滑处理（支持圆弧倒角和 Clothoid 曲线）
     * @param nodes 原始路径节点
     * @param world 环境指针（用于地面投影）
     * @param prm 平滑参数
     * @return 平滑后的轨迹 [x,y,z,yaw,kappa]
     */
    std::vector<Eigen::Matrix<double,5,1>> smoothWithFillets(const std::vector<Node*>& nodes,
                                                            World* world,
                                                            const FilletParams& prm);

protected:
//Data Members
    Node* node_origin_=NULL;
    Node* node_target_=NULL;

    int curr_iter_;
    double curr_time_;//(in ms)

    //To accelerate the speed of generating the initial solution,the tree will grow toward the target with it,a centain probability 
    double goal_biased_=0.15;

    double goal_threshold_=1.0;
    double sub_goal_threshold_=1.0;

    double inherit_threshold_=1.25;

    //step size used when generating new nodes
    double step_size_=0.2;

    //parameters related to function fitPlane
    float h_surf_;
    FitPlaneArg fit_plane_arg_={1.0,2000.0,0.0014,0.4,0.25,0.4,0.1152};
    double radius_fit_plane_=1.0;

    //radius used in function FindNeighbors
    float neighbor_radius_=1.0f;

    PlanningState planning_state_;

    World* world_;

    //used in function generatePath
    std::vector<std::pair<Node*,float>> close_check_record_;

    std::vector<Node*> tree_;

    Path path_;

    //record 2D information of target,when the target can't be projected to surface,it will be used in rolling-planning
    Eigen::Vector2d end_pos_2D_;

    // ========== 路径平滑相关参数 ==========
    FilletParams fillet_params_;

//Function Members

//----------funtions for inherit
    void updateNode(Node *node_input);

    bool inheritPath(Node* new_root,Path::Type type);

    void addInvalidNodes(Node* &node_input,const bool &ifdelete,std::vector<Node*> &invalid_nodes);

    void trimTree();

    bool inheritTree(Node* new_root);
//----------

//----------funtions for sample    

    float getRandomNum();

    Eigen::Vector2d getRandom2DPoint();

    Eigen::Vector3d sampleInEllipsoid();

    Eigen::Vector2d sampleInSector();

    /**
     * @brief Sample to get a random 2D point in various ways.It integrates all the above sampling functions
     * @param void
     * @return Vector2d
     */
    Eigen::Vector2d sample();
//----------

    Node* findNearest(const Eigen::Vector2d &point);

    Eigen::Vector2d steer(const Eigen::Vector2d &point_rand_projection, const Eigen::Vector2d &point_nearest);

    void findNearNeighbors(Node* node_new,std::vector<std::pair<Node*,float>> &record);

    void findParent(Node* node_new,const std::vector<std::pair<Node*,float>> &record);

    void reWire(Node* node_new,const std::vector<std::pair<Node*,float>> &record);

    void deleteChildren(Node* node_parent,Node* node_children);

    void updateChildrenCost(Node* &node_root, const float &costdifference);

    /**
     * @brief Check the node.If it has met the conditions set in advance(i.e.,it's close enough to the target),
     *        add it to the data member "close_check_record_".   
     * @param Node*
     * @return void
     */
    void closeCheck(Node* node);

    /**
     * @brief Read information from "close_check_record_".According to the node information stored in,the function
     *        will select the node with the smallest valuation funtion,and generate a path through the node   
     * @param void
     * @return void
     */
    void generatePath();

    float calPathDis(const std::vector<Node*> &nodes);

    void pubTraversabilityOfTree(ros::Publisher* tree_tra_pub);
};

}
}

#endif