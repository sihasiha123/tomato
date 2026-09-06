#ifndef MULTI_NAVI_GOAL_PANEL_H
#define MULTI_NAVI_GOAL_PANEL_H

#include <string>
#include <ros/ros.h>
#include <ros/console.h>
#include <rviz/panel.h>
#include <QPushButton>
#include <QTableWidget>
#include <QCheckBox>
#include <QLineEdit>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>  // 新增：用于Bool消息类型
#include <actionlib_msgs/GoalStatus.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <tf/transform_datatypes.h>
#include <rover_msgs/roverGoalStatus.h>
#include <std_msgs/Float32MultiArray.h>

namespace rviz_multi_goals_manager_plugins
{
    class MultiNaviGoalsPanel : public rviz::Panel
    {
        Q_OBJECT
        
    public:
        explicit MultiNaviGoalsPanel(QWidget *parent = 0);
        
        // 普通公有成员函数
        void writePose(geometry_msgs::Pose pose);
        void markPose(const geometry_msgs::PoseStamped &pose);
        void deleteMark();
        void publishNavigationProgress(bool is_cycle_mode = false);
        bool appendGoalPose(const geometry_msgs::PoseStamped &pose);
        
    public Q_SLOTS:
        void setMaxNumGoal(const QString &maxNumGoal);
        
    protected Q_SLOTS:
        void changeEvent();     // update max number of goal
        void initPoseTable();   // initialize the pose table
        void updatePoseTable(); // update the pose table
        void startNavi();       // start navigate for the first pose
        void cancelNavi();
        void addManualGoal();
        void checkCycle();
        void startSpin();       // spin for sub
        
    protected:
        // 普通保护成员函数
        void goalCntCB(const geometry_msgs::PoseStamped::ConstPtr &pose); // goal count sub callback function
        void statusGlobalPoseCB(const rover_msgs::roverGoalStatus::ConstPtr &poseStatus);
        void statusLocalPoseCB(const rover_msgs::roverGoalStatus::ConstPtr &poseStatus);
        void completeNavi();    // after the first pose, continue to navigate the rest of poses
        void cycleNavi();
        bool checkGoal(std::vector<actionlib_msgs::GoalStatus> status_list); // check whether arrived the goal
        bool checkGoalStatus(rover_msgs::roverGoalStatus globalStatus, rover_msgs::roverGoalStatus localStatus);
        void publishFinalGoalFlag(int goal_index);  // 新增方法
        
        // Qt界面组件
        QLineEdit *output_maxNumGoal_editor_;
        QPushButton *output_maxNumGoal_button_, *output_reset_button_, *output_startNavi_button_, *output_cancel_button_;
        QLineEdit *manual_goal_x_editor_, *manual_goal_y_editor_, *manual_goal_yaw_editor_;
        QPushButton *output_add_goal_button_;
        QTableWidget *poseArray_table_;
        QCheckBox *cycle_checkbox_;
        QString output_maxNumGoal_;
        
        // ROS相关成员
        ros::NodeHandle nh_;
        ros::Publisher goal_pub_, cancel_pub_, marker_pub_;
        ros::Publisher final_goal_flag_pub_;      // 发布最终目标标志
        ros::Publisher navigation_progress_pub_;  // 发布导航进度
        ros::Subscriber goal_sub_, status_sub_, global_goal_status_sub_, local_goal_status_sub_;
        
        // 状态变量
        int maxNumGoal_;
        int curGoalIdx_ = 0, cycleCnt_ = 0;
        bool permit_ = false, cycle_ = false, arrived_ = false;
        geometry_msgs::PoseArray pose_array_;
        std::string manual_goal_frame_id_;
        actionlib_msgs::GoalID cur_goalid_;
        int cur_goal_id;
        rover_msgs::roverGoalStatus globalGoalStatus, localGoalStatus;
    };
}

#endif // MULTI_NAVI_GOAL_PANEL_H
