#include "PUTN_planner.h"
#include <std_msgs/Float32MultiArray.h>
#include <random>

using namespace std;
using namespace std_msgs;
using namespace Eigen;
using namespace ros;
using namespace PUTN;
using namespace PUTN::visualization;
using namespace PUTN::planner;

PFRRTStar::PFRRTStar(){}
PFRRTStar::PFRRTStar(const double &height,World* world):h_surf_(height),world_(world)
{
    // 初始化圆弧倒角参数的默认值
    fillet_params_.R_min = 0.50;
    fillet_params_.clearance = 0.20;
    fillet_params_.ds = 0.05;
    fillet_params_.use_clothoid = false;
    fillet_params_.clothoid_length_factor = 0.4;
    fillet_params_.max_curvature_rate = 2.0;
}
PFRRTStar::~PFRRTStar()
{
    clean_vector(tree_);
    delete node_target_;
    node_target_=NULL;
}

void PFRRTStar::initWithGoal(const Vector3d &start_pos,const Vector3d &end_pos)
{
    Vector2d last_end_pos_2D=end_pos_2D_;
    PlanningState last_planning_state=planning_state_;
    curr_iter_=0;
    curr_time_=0.0;
    end_pos_2D_=project2plane(end_pos);

    Node* node_origin=fitPlane(start_pos);

    if(node_origin==NULL)
    {
        planning_state_=Invalid;
        return;
    }
    
    Node* node_target=fitPlane(end_pos);

    planning_state_=(node_target==NULL)?Roll:Global;

    close_check_record_.clear();

    bool inherit_flag=false;

    switch(planning_state_)
    {
        case Global:
        {
            switch(last_planning_state)
            {
                case Global:
                    if(last_end_pos_2D==end_pos_2D_ && inheritPath(node_origin,Path::Global))
                    {
                        inherit_flag=true;
                        delete node_target;node_target=NULL;
                    }
                    else{delete node_target_;node_target_=node_target;}
                    break;
                case WithoutGoal:
                    delete node_target_;node_target_=node_target;
                    inherit_flag=inheritTree(node_origin);
                    break;
                default:
                    delete node_target_;node_target_=node_target;
                    break;
            }
        }
        break;
        case Roll:
        {
            switch(last_planning_state)
            {
                case Roll:
                    inherit_flag=(last_end_pos_2D==end_pos_2D_ && inheritPath(node_origin,Path::Sub));
                    break;
                case WithoutGoal:
                    inherit_flag=inheritTree(node_origin);
                    break;
                default:
                    break;
            }
        }
        break;
        default:
        break;
    }

    if(!inherit_flag)
    {
        path_=Path();
        clean_vector(tree_);
        node_origin_=node_origin;
        tree_.push_back(node_origin_);
    }

    if(planning_state_==Roll && last_end_pos_2D!=end_pos_2D_) sub_goal_threshold_=1.0f;

    vector<Node*> origin_and_goal{node_origin_};
    if(planning_state_==Global) origin_and_goal.push_back(node_target_);
    visOriginAndGoal(origin_and_goal,goal_vis_pub_);
}

void PFRRTStar::initWithoutGoal(const Vector3d &start_pos)
{
    curr_iter_=0;
    curr_time_=0.0;

    close_check_record_.clear();
    path_=Path();

    PlanningState last_planning_state=planning_state_;
  
    Node* node_origin=fitPlane(start_pos);

    if(node_origin==NULL)
    {
        planning_state_=Invalid;
        return;
    }

    planning_state_=WithoutGoal;

    delete node_target_;
    node_target_=NULL;

    if(last_planning_state!=WithoutGoal || !inheritTree(node_origin))
    {
        clean_vector(tree_);
        node_origin_=node_origin;
        tree_.push_back(node_origin_);
    }
}

void PFRRTStar::updateNode(Node* node_input)
{
    if(node_input->parent_!=NULL)//Skip the root node
        node_input->cost_=node_input->parent_->cost_+calCostBetweenTwoNode(node_input,node_input->parent_);

    closeCheck(node_input);

    //Update by recursion
    for(auto &node:node_input->children_) updateNode(node);
}

void PFRRTStar::addInvalidNodes(Node* &node_input,const bool &ifdelete,vector<Node*> &invalid_nodes)
{
    if(node_input==NULL) return;
    bool delete_flag=false;
    if(ifdelete || node_input->plane_==NULL) delete_flag=true;
    else
    {
        if(node_input->parent_!=NULL)
            delete_flag=!world_->collisionFree(node_input,node_input->parent_);
        else//If the root node is input 
            delete_flag=!world_->isFree(node_input->position_);
    }
    if(delete_flag) invalid_nodes.push_back(node_input);
    for(auto &node:node_input->children_) addInvalidNodes(node,delete_flag,invalid_nodes);
}

void PFRRTStar::trimTree()
{
    vector<Node*> invalid_nodes;
    addInvalidNodes(node_origin_,false,invalid_nodes);
    for(auto &node:invalid_nodes)
    {
        if(node->parent_!=NULL) deleteChildren(node->parent_,node);
        for(vector<Node*>::iterator it=tree_.begin();it!=tree_.end();++it)
        {
            if(*it==node)
            {
                tree_.erase(it);
                break;
            }
        }
    }
    clean_vector(invalid_nodes);
}

bool PFRRTStar::inheritTree(Node* new_root)
{ 
    bool result=false;

    //considering that the update of grid map may affect the result of plane-fitting
    for(auto&node:tree_) fitPlane(node);
        
    trimTree();

    float min_dis = INF;
    Node* node_insert = NULL;
  
    for(const auto&node:tree_) 
    { 
        float tmp_dis=EuclideanDistance(node,new_root);
        if(tmp_dis < min_dis && world_->collisionFree(node,new_root)) 
        {
            min_dis = tmp_dis;
            node_insert = node;
        }
    }

    if(node_insert!=NULL)
    {
        result=true;
        Node* node=node_insert;
        vector<Node*> node_record;
        while(node!=NULL)
        {
            node_record.push_back(node);
            node=node->parent_;
        }
        for(size_t i=node_record.size()-1;i>0;i--)
        {
            deleteChildren(node_record[i],node_record[i-1]);
            node_record[i]->parent_=node_record[i-1];
            node_record[i-1]->children_.push_back(node_record[i]);
        }
        new_root->children_.push_back(node_insert);
        node_insert->parent_=new_root;
        tree_.push_back(new_root);
        updateNode(new_root);
        node_origin_=new_root;

        vector<pair<Node*,float>> neighbor_record;
        findNearNeighbors(node_origin_,neighbor_record);
        reWire(node_origin_,neighbor_record);
    }
    return result;
}

bool PFRRTStar::inheritPath(Node* new_root,Path::Type type)
{
    bool result=false;
    if(path_.type_==type)
    {
        //copy the path
        vector<Node*> tmp_nodes;
        for(size_t i = 0;i < path_.nodes_.size();i++)
        {
            Node* node_now=path_.nodes_[i];
            tmp_nodes.push_back(fitPlane(node_now->plane_->init_coord));
            if(tmp_nodes[i]==NULL || (tmp_nodes.size()>1 && !world_->collisionFree(tmp_nodes[i],tmp_nodes[i-1])))
                return false;
            //if the distance between the current node and the new root is less
            //than the threshold and there is no obstacle between them.
            if(EuclideanDistance(tmp_nodes[i],new_root) < inherit_threshold_ && world_->collisionFree(tmp_nodes[i],new_root))
            {
                result=true;
                break;
            }
        }
        if(result)
        {
            tmp_nodes.push_back(new_root);
            size_t start_index=(type==Path::Global?1:0);
            for(size_t i=start_index;i<tmp_nodes.size()-1;i++)
            {
                tmp_nodes[i]->parent_=tmp_nodes[i+1];
                tmp_nodes[i+1]->children_.push_back(tmp_nodes[i]);
            }
            path_=Path();
            clean_vector(tree_);
            tree_.assign(tmp_nodes.begin()+start_index,tmp_nodes.end());          
            node_origin_=new_root;
            updateNode(node_origin_);
            generatePath();
        }
    }
    return result;
}

float PFRRTStar::getRandomNum()
{
    random_device rand_rd;
    mt19937 rand_gen(rand_rd());
    uniform_real_distribution<> rand_unif(0, 1.0);
    return rand_unif(rand_gen);
}

Vector2d PFRRTStar::getRandom2DPoint()
{
    Vector3d lb = world_->getLowerBound();
    Vector3d ub = world_->getUpperBound();

    Vector2d rand_point=Vector2d( (ub(0)-lb(0))*getRandomNum()+lb(0),(ub(1)-lb(1))*getRandomNum()+lb(1));
    return rand_point;           
}

Vector3d PFRRTStar::sampleInEllipsoid()
{
    float cmin=EuclideanDistance(node_target_,node_origin_);
    Vector3d a_1=(node_target_->position_-node_origin_->position_)/cmin;  
    RowVector3d id_t(1,0,0);
    Matrix3d M=a_1*id_t;
    JacobiSVD<MatrixXd> SVD(M,ComputeFullU|ComputeFullV);
    Matrix3d U=SVD.matrixU();   
    Matrix3d V=SVD.matrixV();
    Matrix3d A=Matrix3d::Zero();
    A(0,0)=A(1,1)=1,A(2,2)=U.determinant()*V.determinant();
    Matrix3d C=U*A*V;

    float cbest=path_.dis_+1.0f;

    Matrix3d L=Matrix3d::Zero();
    L(0,0)=cbest*0.5,L(1,1)=L(2,2)=sqrt(powf(cbest, 2)-powf(cmin,2))*0.5;

    float theta1=acos(2*getRandomNum()-1);
    float theta2=2*PI*getRandomNum();
    float radius=powf(getRandomNum(),1.0/3);

    Vector3d random_ball;
    random_ball << radius*sin(theta1)*cos(theta2),
                   radius*sin(theta1)*sin(theta2),
                   radius*cos(theta1);

    Vector3d random_ellipsoid=C*L*random_ball;

    Vector3d center= (node_origin_->position_+node_target_->position_)*0.5;
    Vector3d point=random_ellipsoid+center;
    return point;
}

Vector2d PFRRTStar::sampleInSector()
{
    float sample_sector_lb_=2.0f;

    vector<float> theta_record;

    Vector2d start_2D=project2plane(node_origin_->position_);
    for(size_t i=0;i<path_.nodes_.size()-1;i++)
    {
        Vector2d pt_2D=project2plane(path_.nodes_[i]->position_);
        Vector2d diff=pt_2D-start_2D;
        if(diff.norm() < sample_sector_lb_) continue;
        float theta=atan2f(diff(1),diff(0));
        theta_record.push_back(theta);       
    }

    if(theta_record.empty()) return getRandom2DPoint(); 

    default_random_engine engine;

    uniform_int_distribution<unsigned> rand_int(0,theta_record.size()-1);

    float theta_rate = getRandomNum();

    float theta=theta_record[rand_int(engine)]+(theta_rate-0.5)*20.0*PI/180.0;

    Vector2d sub_goal_2D=project2plane(path_.nodes_.front()->position_);

    float sample_sector_ub=(EuclideanDistance(start_2D,sub_goal_2D)+EuclideanDistance(start_2D,end_pos_2D_))*0.5+1.0f;

    float rand_num=getRandomNum();

    float R=sqrt(rand_num)*(sample_sector_ub-sample_sector_lb_)+sample_sector_lb_;

    Vector2d rand_point(R*cos(theta) , R*sin(theta));
    rand_point+=project2plane(node_origin_->position_);

    return rand_point;
}

Vector2d PFRRTStar::sample()
{
    Vector2d point_sample;
    switch(planning_state_)
    {
        case Global:
        {
            if(!path_.nodes_.empty()) point_sample=project2plane(sampleInEllipsoid());
            else
                point_sample=(getRandomNum() < goal_biased_ )?project2plane(node_target_->position_):getRandom2DPoint();
        }
        break;
        case Roll:
            point_sample=path_.nodes_.empty()?getRandom2DPoint():sampleInSector();
        break;
        case WithoutGoal:
            point_sample=getRandom2DPoint();
        break;
        default:
        break;
    }
    return point_sample;
}

Node* PFRRTStar::findNearest(const Vector2d &point)
{
    float min_dis = INF;
    Node* node_closest = NULL;
  
    for(const auto&node:tree_) 
    { 
        //Here use Manhattan distance instead of Euclidean distance to improve the calculate speed.
        float tmp_dis=fabs(point(0)-node->position_(0))+fabs(point(1)-node->position_(1));
        if(tmp_dis < min_dis) 
        {
            min_dis = tmp_dis;
            node_closest = node;
        }
    }
    return node_closest;
}

Vector2d PFRRTStar::steer(const Vector2d &point_rand_projection, const Vector2d &point_nearest_projection)
{
    Vector2d point_new_projection;
    Vector2d steer_p = point_rand_projection - point_nearest_projection;
    float steer_norm=steer_p.norm();
    if (steer_norm > step_size_) //check if the EuclideanDistance between two nodes is larger than the maximum travel step size
        point_new_projection = point_nearest_projection +  steer_p * step_size_/steer_norm;
    else 
        point_new_projection=point_rand_projection;
    return point_new_projection;
}

Node* PFRRTStar::fitPlane(const Vector2d &p_original)
{
    Node* node = NULL;
    Vector3d p_surface;
    
    //make sure that p_original can be projected to the surface,otherwise the nullptr will be returned
    if(world_->project2surface(p_original(0),p_original(1),&p_surface))
    {
        node=new Node;
        node->plane_=new Plane(p_surface,world_,radius_fit_plane_,fit_plane_arg_);
        node->position_ = p_surface + h_surf_ * node->plane_->normal_vector;
    }
    return node;
}

void PFRRTStar::fitPlane(Node* node)
{
    Vector2d init_coord=node->plane_->init_coord;
    //cout << RowVector3d(node->plane_->normal_vector) << endl;
    //cout << RowVector3d(node->position_) << endl;
    delete node->plane_;
    node->plane_=NULL;
    Vector3d p_surface;
    if(world_->project2surface(init_coord(0),init_coord(1),&p_surface))
    {
        node->plane_=new Plane(p_surface,world_,radius_fit_plane_,fit_plane_arg_);
        //cout << RowVector3d(node->plane_->normal_vector) << endl;
        //cout << RowVector3d(node->position_) << endl;
        node->position_=p_surface + h_surf_ * node->plane_->normal_vector;
    }
}

void PFRRTStar::findNearNeighbors(Node* node_new,vector<pair<Node*,float>> &record) 
{ 
    for (const auto&node:tree_) 
    {
        if(EuclideanDistance(node_new,node) < neighbor_radius_ && world_->collisionFree(node_new,node) )
            record.push_back( pair<Node*,float>(node,calCostBetweenTwoNode(node_new,node)) );
    }
}

void PFRRTStar::findParent(Node* node_new,const vector<pair<Node*,float>> &record) 
{    
    Node* node_parent=NULL;
    float min_cost = INF;
    for(const auto&rec:record) 
    { 
        Node* node=rec.first;
        float tmp_cost=node->cost_+rec.second;
        if(tmp_cost < min_cost)
        {
            node_parent=node;
            min_cost=tmp_cost;
        }
    }
    node_new->parent_=node_parent;
    node_new->cost_=min_cost;
    node_parent->children_.push_back(node_new);
}

void PFRRTStar::reWire(Node* node_new,const vector<pair<Node*,float>> &record) 
{ 
    for (const auto&rec:record) 
    { 
        Node* node=rec.first;
        float tmp_cost=node_new->cost_+rec.second;//cost value if the new node is the parent
        float costdifference=node->cost_-tmp_cost;//compare the two and update if the latter is smaller,change new node to the parent node
        if(costdifference > 0) 
        {
            deleteChildren(node->parent_,node);
            node->parent_=node_new;
            node->cost_=tmp_cost;
            node_new->children_.push_back(node);
            updateChildrenCost(node,costdifference);
        }
    }
}

void PFRRTStar::deleteChildren(Node* node_parent,Node* node_children)
{
    for(vector<Node*>::iterator it=node_parent->children_.begin();it!=node_parent->children_.end();++it)
    {
        if(*it==node_children)
        {
            node_parent->children_.erase(it);
            break;
        }
    }
}

void PFRRTStar::updateChildrenCost(Node* &node_root, const float &costdifference) 
{
    for(auto&node:node_root->children_)
    {
        node->cost_ -= costdifference;
        updateChildrenCost(node,costdifference); 
    }
}

void PFRRTStar::closeCheck(Node* node)
{
    switch(planning_state_)
    {
        case Global:
        {
            if(EuclideanDistance(node,node_target_) < goal_threshold_ && world_->collisionFree(node,node_target_))
                close_check_record_.push_back(pair<Node*,float>(node,calCostBetweenTwoNode(node,node_target_)));
        }
        break;
        case Roll: 
        {
            if(EuclideanDistance(project2plane(node->position_),end_pos_2D_) < sub_goal_threshold_)
                close_check_record_.push_back(pair<Node*,float>(node,powf(EuclideanDistance(end_pos_2D_,project2plane(node->position_)),3)));
        }
        break;
        default:
        break;
    }
}

float PFRRTStar::calPathDis(const vector<Node*> &nodes)
{
    float dis=0.0f;
    for(size_t i=0;i < nodes.size()-1; i++)
        dis+=EuclideanDistance(nodes[i],nodes[i+1]);
    return dis;
}

void PFRRTStar::generatePath()
{
    switch(planning_state_)
    {
        case Global:
        {
            Node* node_choosed=NULL;
            float min_cost=path_.cost_;
            for (const auto&rec:close_check_record_) 
            {
                Node* node=rec.first;
                float tmp_cost=node->cost_+rec.second;
                if(tmp_cost < min_cost)
                {
                    min_cost=tmp_cost;
                    node_choosed=node;
                }
            }
            if(min_cost!=path_.cost_)
            {
                path_.nodes_.clear();
                path_.nodes_.push_back(node_target_);
                while(node_choosed!=NULL)
                {
                    path_.nodes_.push_back(node_choosed);
                    node_choosed=node_choosed->parent_;
                }
                path_.cost_=min_cost;
                path_.dis_=calPathDis(path_.nodes_);
                path_.type_=Path::Global;
            }
        }
        break;
        case Roll:
        {
            path_=Path();
            Node* sub_goal = NULL;
            float min_cost = INF;  
            for(const auto&rec:close_check_record_) 
            { 
                Node* node=rec.first;
                float tmp_cost=node->cost_+rec.second;
                if (tmp_cost < min_cost) 
                {
                    min_cost = tmp_cost; 
                    sub_goal = node;
                }
            }
            if(sub_goal!=NULL)
            {
                while(sub_goal!=NULL)
                {
                    path_.nodes_.push_back(sub_goal);
                    sub_goal=sub_goal->parent_;
                }
                path_.cost_=path_.nodes_.front()->cost_;
                path_.dis_=calPathDis(path_.nodes_);
                path_.type_=Path::Sub;
            }
            else
            {
                //If close_check_record is empty,adjust the threshold according to the current information of distance
                //so that next time it will more likely to get a solution
                float min_dis=INF;
                for(const auto&node:tree_)
                {
                    float tmp_dis=EuclideanDistance(project2plane(node->position_),end_pos_2D_);
                    if(tmp_dis<min_dis) min_dis=tmp_dis;
                }
                sub_goal_threshold_=min_dis+1.0f;
            }
        }
        break;
        default:
        break;
    }
}

Path PFRRTStar::planner(const int &max_iter,const double &max_time)
{
    if(planning_state_==Invalid)
    {
        ROS_ERROR("Illegal operation:the planner is at an invalid working state!!");
        return {};
    }
    double time_now=curr_time_;
    timeval start;gettimeofday(&start,NULL);
    while (curr_iter_ < max_iter && curr_time_ < max_time)
    {
        //Update current iteration
        curr_iter_++;

        //Update current time consuming
        timeval end;gettimeofday(&end,NULL);
        float ms=1000*(end.tv_sec-start.tv_sec)+0.001*(end.tv_usec-start.tv_usec);
        curr_time_=ms+time_now;

        //Sample to get a random 2D point
        Vector2d rand_point_2D=sample();

        //Find the nearest node to the random point
        Node* nearest_node= findNearest(rand_point_2D);
        Vector2d nearest_point_2D = project2plane(nearest_node->position_);

        //Expand from the nearest point to the random point
        Vector2d new_point_2D = steer(rand_point_2D,nearest_point_2D);

        //Based on the new 2D point,
        Node* new_node = fitPlane(new_point_2D); 

        if( new_node!=NULL//1.Fail to fit the plane,it will return a null pointer
            &&world_->isInsideBorder(new_node->position_)//2.The position is out of the range of the grid map.
          ) 
        {
            //Get the set of the neighbors of the new node in the tree
            vector<pair<Node*,float>> neighbor_record;
            findNearNeighbors(new_node,neighbor_record);

            //Select an appropriate parent node for the new node from the set.
            if(!neighbor_record.empty()) findParent(new_node,neighbor_record);
            //Different from other RRT algorithm,it is posible that the new node is too far away from the whole tree.If
            //so,discard the new node.
            else
            {
                delete new_node;
                continue;
            }

            //Add the new node to the tree
            tree_.push_back(new_node);

            //Rewire the tree to optimize it
            reWire(new_node,neighbor_record);

            //Check if the new node is close enough to the goal
            closeCheck(new_node);

            if(planning_state_==Global) generatePath();
        }
        else  
            delete new_node;
    }

    if(planning_state_==Roll) generatePath();

    visTree(tree_,tree_vis_pub_);
    pubTraversabilityOfTree(tree_tra_pub_);
    
    return path_;
}

void PFRRTStar::pubTraversabilityOfTree(Publisher* tree_tra_pub)
{
    if(tree_tra_pub==NULL) return;
    Float32MultiArray msg;
    for(const auto&node:tree_)
    {
        msg.data.push_back(node->position_(0));
        msg.data.push_back(node->position_(1));
        msg.data.push_back(node->position_(2));
        msg.data.push_back(node->plane_->traversability);
    }
    tree_tra_pub->publish(msg);
}

// ========== 路径平滑功能函数 ==========

double PFRRTStar::wrapYaw(double a) {
    while(a > M_PI) a -= 2*M_PI;
    while(a < -M_PI) a += 2*M_PI;
    return a;
}

std::vector<KappaPose2D> PFRRTStar::makeCornerFillet(const Eigen::Vector2d& v1,
                                                    const Eigen::Vector2d& v2,
                                                    const Eigen::Vector2d& v3,
                                                    double R, double ds)
{
    // 入/出方向
    Eigen::Vector2d t1 = (v2 - v1); 
    double L1 = t1.norm(); 
    t1 /= std::max(L1, 1e-9);
    
    Eigen::Vector2d t2 = (v3 - v2); 
    double L2 = t2.norm(); 
    t2 /= std::max(L2, 1e-9);

    // 夹角（逆时针为正）
    double ang = std::atan2(t1.x()*t2.y() - t1.y()*t2.x(), t1.dot(t2));
    double phi = std::fabs(ang);
    if (phi < 1e-3) return {};   // 基本共线，无需倒角

    // 每条直线向内退距，得到切点距离：d = R * tan(phi/2)
    double d = R * std::tan(phi/2.0);
    if (L1 < d + 1e-3 || L2 < d + 1e-3) {
        return {};  // 直线段太短，无法放下半径R的圆弧
    }

    // 切点
    Eigen::Vector2d p_t1 = v2 - d * t1;
    Eigen::Vector2d p_t2 = v2 + d * t2;

    // 圆心方向计算
    Eigen::Vector2d n1(-t1.y(), t1.x());
    Eigen::Vector2d n2( t2.y(), -t2.x());
    Eigen::Vector2d bis = (n1 + n2);
    if (bis.norm() < 1e-6) {
        bis = n1;
    }
    bis.normalize();
    
    // 圆心
    Eigen::Vector2d center = v2 + (R / std::sin(phi/2.0)) * bis;

    // 起始/终止弧角
    double a1 = std::atan2(p_t1.y() - center.y(), p_t1.x() - center.x());
    double a2 = std::atan2(p_t2.y() - center.y(), p_t2.x() - center.x());

    // 确定转向并规范插值方向
    double da = (ang > 0 ? +1 : -1) * ds / R;
    
    auto angDiff = [&](double a, double b){
        double d = b - a; 
        while(d > M_PI) d -= 2*M_PI; 
        while(d < -M_PI) d += 2*M_PI; 
        return d;
    };
    
    double need = angDiff(a1, a2);
    if ((need > 0 && da < 0) || (need < 0 && da > 0)) da = -da;

    std::vector<KappaPose2D> out;
    
    // 直线段1（从 v1 到 p_t1）
    double Lpre = (p_t1 - v1).norm();
    int npre = std::max(1, int(std::floor(Lpre / ds)));
    for (int i=1; i<=npre; i++){
        double s = (double)i/npre;
        Eigen::Vector2d p = v1*(1-s) + p_t1*s;
        double yaw = std::atan2(t1.y(), t1.x());
        out.push_back({p, yaw, 0.0});
    }

    // 圆弧段
    int narc = std::max(1, int(std::ceil(std::fabs(need) / std::fabs(da))));
    for (int i=0; i<=narc; i++){
        double ai = a1 + i*da;
        if ((da>0 && ai>a2) || (da<0 && ai<a2)) ai = a2;
        
        Eigen::Vector2d p(center.x() + R*std::cos(ai),
                         center.y() + R*std::sin(ai));
        double yaw = ai + (ang>0 ? +M_PI/2.0 : -M_PI/2.0);
        out.push_back({p, wrapYaw(yaw), (ang>0 ? +1.0 : -1.0)/R});
        
        if (ai==a2) break;
    }

    // 直线段2（从 p_t2 到 v3）
    double Lpost = (v3 - p_t2).norm();
    int npost = std::max(1, int(std::floor(Lpost / ds)));
    for (int i=1; i<=npost; i++){
        double s = (double)i/npost;
        Eigen::Vector2d p = p_t2*(1-s) + v3*s;
        double yaw = std::atan2(t2.y(), t2.x());
        out.push_back({p, yaw, 0.0});
    }

    return out;
}

std::vector<KappaPose2D> PFRRTStar::makeClothoidFillet(const Eigen::Vector2d& v1,
                                                       const Eigen::Vector2d& v2,
                                                       const Eigen::Vector2d& v3,
                                                       double R_eff, double ds,
                                                       double length_factor,
                                                       double max_curvature_rate) {
    // 计算入射和出射方向
    Eigen::Vector2d t1 = (v2 - v1);
    double L1 = t1.norm();
    if (L1 < 1e-6) return {};
    t1 /= L1;
    
    Eigen::Vector2d t2 = (v3 - v2);
    double L2 = t2.norm();
    if (L2 < 1e-6) return {};
    t2 /= L2;
    
    // 计算转向角度
    double cross_product = t1.x() * t2.y() - t1.y() * t2.x();
    double dot_product = t1.dot(t2);
    double turn_angle = atan2(cross_product, dot_product);
    
    if (std::abs(turn_angle) < 1e-3) {
        return {}; // 基本直线，无需平滑
    }
    
    // 使用length_factor计算Clothoid长度
    double clothoid_length = std::min(L1, L2) * length_factor;
    clothoid_length = std::min(clothoid_length, std::abs(turn_angle) * R_eff * 1.2);
    clothoid_length = std::max(clothoid_length, R_eff * 0.3);  // 最小长度
    
    // 计算所需的曲率变化率
    double dk_desired = 2.0 * turn_angle / (clothoid_length * clothoid_length);
    
    // 双重钳位：1) 最大曲率约束 2) 最大曲率变化率约束
    double max_kappa = 1.0 / R_eff;  // 最大曲率 = 1/最小半径
    double dk_max_from_curvature = 2.0 * max_kappa / clothoid_length;
    
    // 应用约束
    double dk_clamped = std::min(std::abs(dk_desired), 
                                std::min(dk_max_from_curvature, max_curvature_rate));
    if (turn_angle < 0) dk_clamped = -dk_clamped;
    
    // 如果约束后的dk与期望相差太大，调整长度
    if (std::abs(dk_clamped) < std::abs(dk_desired) * 0.5) {
        clothoid_length = std::sqrt(2.0 * std::abs(turn_angle) / std::abs(dk_clamped));
        clothoid_length = std::min(clothoid_length, std::min(L1, L2) * 0.8);
    }
    
    // 检查线段是否足够长
    double d1 = clothoid_length * 0.5;
    double d2 = clothoid_length * 0.5;
    
    if (L1 < d1 + 1e-3 || L2 < d2 + 1e-3) {
        return {}; // 线段太短
    }
    
    Eigen::Vector2d p_start = v2 - d1 * t1;
    Eigen::Vector2d p_end = v2 + d2 * t2;
    
    std::vector<KappaPose2D> result;
    
    // 第一段直线
    double line1_length = (p_start - v1).norm();
    int n1 = std::max(1, static_cast<int>(line1_length / ds));
    for (int i = 1; i <= n1; i++) {
        double s = static_cast<double>(i) / n1;
        Eigen::Vector2d p = v1 * (1 - s) + p_start * s;
        double yaw = atan2(t1.y(), t1.x());
        result.push_back({p, yaw, 0.0});
    }
    
    // Clothoid 段
    ClothoidCurve clothoid;
    ClothoidParams params;
    params.x0 = p_start.x();
    params.y0 = p_start.y();
    params.theta0 = atan2(t1.y(), t1.x());
    params.kappa0 = 0.0;
    params.length = clothoid_length;
    params.dk = dk_clamped;  // 使用约束后的曲率变化率
    
    auto clothoid_points = clothoid.generateClothoidPoints(params, ds);
    
    // 添加 Clothoid 点（跳过第一个点避免重复）
    if (clothoid_points.size() > 1) {
        result.insert(result.end(), clothoid_points.begin() + 1, clothoid_points.end());
    }
    
    // 第二段直线
    double line2_length = (v3 - p_end).norm();
    int n2 = std::max(1, static_cast<int>(line2_length / ds));
    for (int i = 1; i <= n2; i++) {
        double s = static_cast<double>(i) / n2;
        Eigen::Vector2d p = p_end * (1 - s) + v3 * s;
        double yaw = atan2(t2.y(), t2.x());
        result.push_back({p, yaw, 0.0});
    }
    
    return result;
}

std::vector<Eigen::Matrix<double,5,1>> PFRRTStar::smoothWithFillets(const std::vector<Node*>& nodes,
                                                                   World* world,
                                                                   const FilletParams& prm)
{
    std::vector<Eigen::Matrix<double,5,1>> traj; // [x,y,z,yaw,kappa]
    if (nodes.size() < 2) return traj;

    // 抽取 2D 顶点
    std::vector<Eigen::Vector2d> V; 
    V.reserve(nodes.size());
    for (auto* nd : nodes) V.push_back(project2plane(nd->position_));

    // 计算有效半径
    const double R_eff = prm.R_min + prm.clearance;

    // 逐段处理
    auto appendSample = [&](const KappaPose2D& kp) -> bool {
        // 地面投影取 z
        Eigen::Vector3d p_surf;
        if (!world->project2surface(kp.p.x(), kp.p.y(), &p_surf)) return false;
        
        // 障碍物检查
        if (!world->isFree(p_surf)) return false;
        
        Eigen::Matrix<double,5,1> rec;
        rec << p_surf.x(), p_surf.y(), p_surf.z(), kp.yaw, kp.kappa;
        traj.push_back(rec);
        return true;
    };

    // 第一段先直线到第二点
    if (V.size() > 1) {
        Eigen::Vector2d p0 = V[0], p1 = V[1];
        Eigen::Vector2d t = (p1-p0); 
        double L=t.norm(); 
        if (L>1e-6) t/=L;
        int n = std::max(1, int(std::floor(L/prm.ds)));
        for(int i=0;i<=n;i++){
            double s=(double)i/n;
            Eigen::Vector2d p=p0*(1-s)+p1*s;
            double yaw=std::atan2(t.y(),t.x());
            if (!appendSample({p,yaw,0.0})) {
                // 障碍物冲突，回退到线性插值
                return {};
            }
        }
    }

    // 处理所有拐角
    for (size_t i=1; i+1<V.size(); ++i){
        std::vector<KappaPose2D> block;
        
        if (prm.use_clothoid) {
            // 使用 Clothoid 曲线，传入额外参数
            block = makeClothoidFillet(V[i-1], V[i], V[i+1], R_eff, prm.ds,
                                     prm.clothoid_length_factor,
                                     prm.max_curvature_rate);
        } else {
            // 使用圆弧倒角
            block = makeCornerFillet(V[i-1], V[i], V[i+1], R_eff, prm.ds);
        }
        
        if (block.empty()){
            // 不能放下半径 → 退化为直线段
            Eigen::Vector2d a=V[i], b=V[i+1];
            Eigen::Vector2d t=(b-a); 
            double L=t.norm(); 
            if(L>1e-6) t/=L;
            int n=std::max(1,int(std::floor(L/prm.ds)));
            for(int k=1;k<=n;k++){
                double s=(double)k/n;
                Eigen::Vector2d p=a*(1-s)+b*s;
                double yaw=std::atan2(t.y(),t.x());
                if (!appendSample({p,yaw,0.0})) {
                    return {}; // 障碍物冲突
                }
            }
        }else{
            // 检查 fillet 段是否无碰撞
            bool collision_free = true;
            size_t original_size = traj.size();
            
            for (auto& kp : block) {
                if (!appendSample(kp)) {
                    collision_free = false;
                    break;
                }
            }
            
            if (!collision_free) {
                // 回滚到fillet前的状态
                traj.resize(original_size);
                return {}; // 整体失败，让外层回退
            }
        }
    }
    return traj;
}