#include "route_generator/route_generator_node.hpp"

namespace planning_component
{

RouteGenerator::RouteGenerator(const rclcpp::NodeOptions & node_options)
: Node("route_publisher", node_options)
{
    generate_route_executed_ = false;
    current_lane_ = 2; //[PARAM] start_lane_id
    forward_lane_ = 5; //[PARAM] forward_start_lane_id
    checkpoint_ = 0;
    pose_msg_ptr_ = nullptr;

    goalpose_vec_ = generateGoalpoint();
    checkpoint_vec_ = generateCheckpoint();

    dist_threshold_ = this->declare_parameter("dist_threshold", 15.0);
    
    sub_pose_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/localization/kinematic_state", rclcpp::QoS(1),
            std::bind(&RouteGenerator::poseCallback, this, std::placeholders::_1));

    pub_route_ = this->create_publisher<LaneletRoute>(
        "/planning/mission_planning/route", rclcpp::QoS(1).transient_local());

    client_ = this->create_client<SetRoutePoints>(
        "/api/routing/set_route_points");  

    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&RouteGenerator::run, this));
}

void RouteGenerator::sendRequest()
{
    if (!client_->wait_for_service()) 
    {
        RCLCPP_WARN(this->get_logger(), "Service not available");
        return;
    }

    auto request = std::make_shared<SetRoutePoints::Request>();
    request->header.stamp = this->now();
    request->header.frame_id = "map";
    request->option.allow_goal_modification = false;
    request->goal.position.x = 0.0;
    request->goal.position.y = 0.0;
    request->goal.position.z = 0.0;
    request->goal.orientation.x = 0.0;
    request->goal.orientation.y = 0.0;
    request->goal.orientation.z = 0.0;
    request->goal.orientation.w = 1.0;
    request->waypoints.clear();

    client_->async_send_request(request);
}

void RouteGenerator::run()
{
    if(!checkSubscription())
    {
        return;
    }

    int checkpoint_ex = getCheckpointEx();
    int checkpoint = calcClosestCheckpoint(pose_msg_ptr_->pose.pose.position);
    LaneletRoute route = createRoute(*pose_msg_ptr_, checkpoint, checkpoint_ex);
    pub_route_->publish(route);
    if(!generate_route_executed_) //[FIX] doesn't work
    {
        sendRequest();
        generate_route_executed_ = true;
    }
}

bool RouteGenerator::checkSubscription()
{
    if(pose_msg_ptr_ == nullptr)
    {
        return false;
    }
    return true;
}

void RouteGenerator::poseCallback(const nav_msgs::msg::Odometry::SharedPtr pose_msg)
{
    pose_msg_ptr_ = pose_msg;
}

int RouteGenerator::getCheckpointEx()
{
    return checkpoint_;
}

int RouteGenerator::calcClosestCheckpoint(const geometry_msgs::msg::Point & position)
{
    for(int i = 0; i < checkpoint_vec_.size(); i++)
    {
        float x_diff = checkpoint_vec_[i].x - position.x;
        float y_diff = checkpoint_vec_[i].y - position.y;

        if(std::abs(x_diff) + std::abs(y_diff) < dist_threshold_)
        {
            if(i+1 == 5) //[PARAM] 5 checkpoint_num
            {
                checkpoint_ = 0;
                return checkpoint_;
            }
            else
            {
                checkpoint_ = i+1;
                return checkpoint_;
            }
        }
    }
    return checkpoint_;
}

LaneletRoute RouteGenerator::createRoute(const nav_msgs::msg::Odometry & ego_pose, 
    const int & checkpoint, const int & checkpoint_ex)
{
    LaneletRoute route_msg;
    LaneletSegment segment;
    LaneletPrimitive primitive;

    if(checkpoint != checkpoint_ex) //return 0; in calc_closest_checkpoint() makes trigger
    { 
        current_lane_ += 3;
        forward_lane_ += 3;
        if(forward_lane_ > 15) //[PARAM] lane_end_id
        {
            forward_lane_ = 2; 
        } 
        else if(current_lane_ > 15)
        {
            current_lane_ = 2;
        }
    }

    route_msg.header.stamp = this->get_clock()->now();
    route_msg.header.frame_id = "map";
    
    route_msg.start_pose.position.x = ego_pose.pose.pose.position.x;
    route_msg.start_pose.position.y = ego_pose.pose.pose.position.y;
    route_msg.start_pose.position.z = ego_pose.pose.pose.position.z;

    route_msg.start_pose.orientation.x = ego_pose.pose.pose.orientation.x;
    route_msg.start_pose.orientation.y = ego_pose.pose.pose.orientation.y;
    route_msg.start_pose.orientation.z = ego_pose.pose.pose.orientation.z;
    route_msg.start_pose.orientation.w = ego_pose.pose.pose.orientation.w;
    
    route_msg.goal_pose = goalpose_vec_[checkpoint]; 

    segment.preferred_primitive.id = current_lane_;
    primitive.id = current_lane_;
    primitive.primitive_type = "lane";
    segment.primitives.push_back(primitive);
    primitive.id = current_lane_ - 1;
    primitive.primitive_type = "lane";
    segment.primitives.push_back(primitive);
    primitive.id = current_lane_ + 1;
    primitive.primitive_type = "lane";
    segment.primitives.push_back(primitive);
    route_msg.segments.push_back(segment);

    segment.primitives.clear();
    segment.preferred_primitive.id = forward_lane_;
    primitive.id = forward_lane_;
    primitive.primitive_type = "lane";
    segment.primitives.push_back(primitive);
    primitive.id = forward_lane_ - 1;
    primitive.primitive_type = "lane";
    segment.primitives.push_back(primitive);
    primitive.id = forward_lane_ + 1;
    primitive.primitive_type = "lane";
    segment.primitives.push_back(primitive);
    route_msg.segments.push_back(segment);

    return route_msg;
}

std::vector<Pose> RouteGenerator::generateGoalpoint()
{
    std::vector<Pose> goalpoint_vec;

    std::ifstream file("/home/sws/my_ws/src/racing_project/planning/route_generator/data/goalpose.csv");//[PARAM] goalpose_directory
    
    if (!file.is_open()) 
    {
        RCLCPP_ERROR(rclcpp::get_logger("route_generator"), "goalpose file not found");
        return goalpoint_vec;
    }

    std::string line;
    while (std::getline(file, line)) 
    {
        std::stringstream ss(line);
        std::string item;
        std::vector<double> row;

        while (std::getline(ss, item, ',')) 
        {
            try 
            {
                double value = std::stod(item);
                row.push_back(value);
            } 
            catch (const std::invalid_argument& e) 
            {
                std::cerr << "invalid type " << item << std::endl;
            } 
            catch (const std::out_of_range& e) 
            {
                std::cerr << "too big data " << item << std::endl;
            }
        }

        Pose pose;
        pose.position.x = row[0];
        pose.position.y = row[1];
        pose.position.z = row[2];
        pose.orientation.x = row[3];
        pose.orientation.y = row[4];
        pose.orientation.z = row[5];
        pose.orientation.w = row[6];
        goalpoint_vec.push_back(pose);
    }

    file.close();
    return goalpoint_vec;
}

std::vector<Point> RouteGenerator::generateCheckpoint()
{
    std::vector<Point> checkpoint_vec;

    std::ifstream file("/home/sws/my_ws/src/racing_project/planning/route_generator/data/checkpoint.csv");//[PARAM] goalpose_directory
    if (!file.is_open()) 
    {
        RCLCPP_ERROR(rclcpp::get_logger("route_generator"), "checkpoint file not found");
        return checkpoint_vec;
    }

    std::string line;
    while (std::getline(file, line)) 
    {
        std::stringstream ss(line);
        std::string item;
        std::vector<double> row;

        while (std::getline(ss, item, ',')) 
        {
            try 
            {
              double value = std::stod(item);
              row.push_back(value);
            } 
            catch (const std::invalid_argument& e) 
            {
              std::cerr << "invalid type " << item << std::endl;
            } 
            catch (const std::out_of_range& e) 
            {
              std::cerr << "too big data " << item << std::endl;
            }
        }
        Point p;
        p.x = row[0];
        p.y = row[1];
        p.z = 0.0;
        checkpoint_vec.push_back(p);
      
    }
    file.close();
    return checkpoint_vec;
}
} // namespace planning_component
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(planning_component::RouteGenerator)


