#include "route_generator/route_generator_node.hpp"

namespace planning_component
{

RouteGenerator::RouteGenerator(const rclcpp::NodeOptions & node_options)
: Node("route_publisher", node_options)
{
    ego_status_.current_lane_id = 2; //[PARAM] start_lane_id
    ego_status_.forward_lane_id = 5; //[PARAM] forward_start_lane_id
    ego_status_.checkpoint = 0;
    ego_status_.checkpoint_ex = 0;
    ego_status_.is_lanechanging = false;
    ego_status_.current_lane = CurrentLane::CENTER;
    reserved_checkpoint_ = 0;
    checkpoint_reserved_ = false;

    pose_msg_ptr_ = nullptr;
    lanechange_status_msg_ptr_ = nullptr;

    goalpose_vec_ = generateGoalpose();
    checkpoint_vec_ = generateCheckpoint();

    dist_threshold_ = this->declare_parameter("dist_threshold", 15.0);
    
    sub_pose_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/localization/kinematic_state", rclcpp::QoS(1),
            std::bind(&RouteGenerator::poseCallback, this, std::placeholders::_1));
    sub_lanechange_status_ = this->create_subscription<LanechangeStatus>(
        "/racing/lanechange_status", rclcpp::QoS(1),
            std::bind(&RouteGenerator::lanechangestatusCallback, this, std::placeholders::_1));
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

bool RouteGenerator::checkSubscription()
{
    if(pose_msg_ptr_ == nullptr)
    {
        return false;
    }
    else if(lanechange_status_msg_ptr_ == nullptr)
    {
        createFirstRoute(*pose_msg_ptr_);
        return false;
    }
    return true;
}

void RouteGenerator::poseCallback(const nav_msgs::msg::Odometry::SharedPtr pose_msg)
{
    pose_msg_ptr_ = pose_msg;
}

void RouteGenerator::lanechangestatusCallback(const LanechangeStatus::SharedPtr lanechange_status_msg)
{
    lanechange_status_msg_ptr_ = lanechange_status_msg;
}

void RouteGenerator::run()
{
    if(!checkSubscription())
    {
        return;
    }

    updateCurrentLane(*lanechange_status_msg_ptr_);
    calcClosestCheckpoint(pose_msg_ptr_->pose.pose.position);
    createRoute(*pose_msg_ptr_);
}

void RouteGenerator::calcClosestCheckpoint(const geometry_msgs::msg::Point & position)
{
    for(int i = 0; i < checkpoint_vec_.size(); i++)
    {
        const float x_diff = checkpoint_vec_[i].x - position.x;
        const float y_diff = checkpoint_vec_[i].y - position.y;

        if(std::abs(x_diff) + std::abs(y_diff) < dist_threshold_)
        {
            if(!ego_status_.is_lanechanging)
            {
                if(i+1 == 5) //[PARAM] 5 checkpoint_num
                {
                    ego_status_.checkpoint = 0;
                }
                else
                {
                    ego_status_.checkpoint = i+1;
                }
            }
            else
            {
                RCLCPP_INFO(rclcpp::get_logger("route_generator"),"checkpoint reserved");
                if(i+1 == 5) //[PARAM] 5 checkpoint_num
                {
                    reserved_checkpoint_ = 0;
                }
                else
                {
                    reserved_checkpoint_ = i+1;
                }
                checkpoint_reserved_ = true;
            }
        }
        if(checkpoint_reserved_ && !ego_status_.is_lanechanging)
        {
            RCLCPP_INFO(rclcpp::get_logger("route_generator"),"reserved checkpoint initialized");
            ego_status_.checkpoint = reserved_checkpoint_;
            checkpoint_reserved_ = false;
        }
    }

}

void RouteGenerator::updateCurrentLane(const LanechangeStatus & lanechange_status_msg)
{
    /*
        [Update Current Lane]
        update current lane
    */
    if(lanechange_status_msg.type == LanechangeStatus::CHANGEDLEFT)
    {
        ego_status_.current_lane = static_cast<CurrentLane>(static_cast<uint8_t>(ego_status_.current_lane) - 1);
        ego_status_.current_lane_id -= 1; 
        ego_status_.forward_lane_id -= 1; 
        ego_status_.is_lanechanging = false;
        RCLCPP_INFO(rclcpp::get_logger("route_generator"),"[Update Current Lane] Ego changed left");
    }
    else if(lanechange_status_msg.type == LanechangeStatus::CHANGEDRIGHT)
    {
        ego_status_.current_lane = static_cast<CurrentLane>(static_cast<uint8_t>(ego_status_.current_lane) + 1);
        ego_status_.current_lane_id += 1; 
        ego_status_.forward_lane_id += 1; 
        ego_status_.is_lanechanging = false;
        RCLCPP_INFO(rclcpp::get_logger("route_generator"),"[Update Current Lane] Ego changed right");
    }

    if(lanechange_status_msg.type == LanechangeStatus::CHANGINGLEFT || lanechange_status_msg.type == LanechangeStatus::CHANGINGRIGHT)
    {
        ego_status_.is_lanechanging = true; //[FIX] consider aborting situation
    }
    else if(lanechange_status_msg.type == LanechangeStatus::NONE)
    {
        ego_status_.is_lanechanging = false;
    }
}

void RouteGenerator::createFirstRoute(const nav_msgs::msg::Odometry & ego_pose)
{
    LaneletRoute route_msg;
    LaneletSegment segment;
    LaneletPrimitive primitive;

    route_msg.header.stamp = this->get_clock()->now();
    route_msg.header.frame_id = "map";

    createStartpose(route_msg, ego_pose);
    
    /*
        [Create currentlane segment]
        Create segment by the value current_lane_.
    */
    segment.preferred_primitive.id = ego_status_.current_lane_id;

    for(int i = 1; i < 4; i++)
    {
        primitive.id = ego_status_.checkpoint * 3 + i;
        primitive.primitive_type = "lane";
        segment.primitives.push_back(primitive);
    }

    route_msg.segments.push_back(segment);
    segment.primitives.clear();



    /*
        [Create forwardlane segment]
        Create segment by the value forward_lane_.
        If checkpoint >= 4, consider as ego has reached the last lane.
    */
    segment.preferred_primitive.id = ego_status_.forward_lane_id;

    if(ego_status_.checkpoint >= 4)
    {
        for(int i = 1; i < 4; i++)
        {
            primitive.id = i;
            primitive.primitive_type = "lane";
            segment.primitives.push_back(primitive);
        }
    }
    else
    {
        for(int i = 1; i < 4; i++)
        {
            primitive.id = (ego_status_.checkpoint + 1) * 3 + i;
            primitive.primitive_type = "lane";
            segment.primitives.push_back(primitive);
        }
    }

    route_msg.goal_pose = goalpose_vec_[ego_status_.checkpoint]; 

    route_msg.segments.push_back(segment);

    pub_route_->publish(route_msg);
}

void RouteGenerator::createRoute(const nav_msgs::msg::Odometry & ego_pose)
{
    LaneletRoute route_msg;

    route_msg.header.stamp = this->get_clock()->now();
    route_msg.header.frame_id = "map";
    
    createStartpose(route_msg, ego_pose);

    createSegment(route_msg);

    // if(!validateSegment(route_msg))
    // {
    //     RCLCPP_INFO(rclcpp::get_logger("route_generator"),"[Update Current Lane] something wrong");
    // }

    createGoalpose(route_msg);

    pub_route_->publish(route_msg);
}

LaneletRoute RouteGenerator::createStartpose(LaneletRoute & route_msg, const nav_msgs::msg::Odometry & ego_pose)
{
    /*
        [Create Startpose]
        Create startpose which is same as ego pose
    */
    route_msg.start_pose.position.x = ego_pose.pose.pose.position.x;
    route_msg.start_pose.position.y = ego_pose.pose.pose.position.y;
    route_msg.start_pose.position.z = ego_pose.pose.pose.position.z;

    route_msg.start_pose.orientation.x = ego_pose.pose.pose.orientation.x;
    route_msg.start_pose.orientation.y = ego_pose.pose.pose.orientation.y;
    route_msg.start_pose.orientation.z = ego_pose.pose.pose.orientation.z;
    route_msg.start_pose.orientation.w = ego_pose.pose.pose.orientation.w;

    return route_msg;
}

LaneletRoute RouteGenerator::createSegment(LaneletRoute & route_msg)
{
    LaneletSegment segment;
    LaneletPrimitive primitive;

    /*
        [Create currentlane segment]
        Create segment by the value current_lane_.
    */
    segment.preferred_primitive.id = ego_status_.current_lane_id;

    for(int i = 1; i < 4; i++)
    {
        primitive.id = ego_status_.checkpoint * 3 + i;
        primitive.primitive_type = "lane";
        segment.primitives.push_back(primitive);
    }

    route_msg.segments.push_back(segment);
    segment.primitives.clear();



    /*
        [Create forwardlane segment]
        Create segment by the value forward_lane_.
        If checkpoint >= 4, consider as ego has reached the last lane.
    */
    segment.preferred_primitive.id = ego_status_.forward_lane_id;

    if(ego_status_.checkpoint >= 4)
    {
        for(int i = 1; i < 4; i++)
        {
            primitive.id = i;
            primitive.primitive_type = "lane";
            segment.primitives.push_back(primitive);
        }
    }
    else
    {
        for(int i = 1; i < 4; i++)
        {
            primitive.id = (ego_status_.checkpoint + 1) * 3 + i;
            primitive.primitive_type = "lane";
            segment.primitives.push_back(primitive);
        }
    }

    route_msg.segments.push_back(segment);
    checkCheckpointTrigger(route_msg);

    ego_status_.checkpoint_ex = ego_status_.checkpoint;

    return route_msg;
}

LaneletRoute RouteGenerator::checkCheckpointTrigger(LaneletRoute & route_msg)
{
    /*
        [Checkpoint Triggered]
        If "checkpoint != checkpoint_ex" consider as ego has left current lane.
        If += 3 has reached the max lane id, reset to start lane id(- 12).
    */

    if(ego_status_.checkpoint != ego_status_.checkpoint_ex)
    {   
        ego_status_.current_lane_id += 3;
        ego_status_.forward_lane_id += 3;

        if(ego_status_.forward_lane_id > 15) //[PARAM] lane_end_id
        {
            ego_status_.forward_lane_id = ego_status_.forward_lane_id - 15; 
        } 
        else if(ego_status_.current_lane_id > 15)
        {
            ego_status_.current_lane_id = ego_status_.current_lane_id - 15;
        }

        route_msg.segments[0].preferred_primitive.id = ego_status_.current_lane_id;
        route_msg.segments[1].preferred_primitive.id = ego_status_.forward_lane_id;

        RCLCPP_INFO(rclcpp::get_logger("route_generator_node"), "[Checkpoint Triggered] checkpoint_: %d, current_lane_: %d, forward_lane_: %d", 
            ego_status_.checkpoint, ego_status_.current_lane_id, ego_status_.forward_lane_id);

        return route_msg;
    }
    else
    {
        return route_msg;
    }
    return route_msg;
}

// bool RouteGenerator::validateSegment(const LaneletRoute & route_msg)
// {
//     const int current_lane_max = (checkpoint_ + 1) * 3;
//     const int current_lane_min = current_lane_max - 2;

//     int forward_lane_max;
//     int forward_lane_min;

//     if(checkpoint_ >= 4)
//     {
//         forward_lane_max = 3;
//         forward_lane_min = forward_lane_max - 2;
//     }
//     else
//     {
//         forward_lane_max = (checkpoint_ + 2) * 3;
//         forward_lane_min = forward_lane_max - 2;
//     }

//     /*
//         [Validate current&forwardlane]
//         Create segment by the value forward_lane_.
//         If checkpoint < 4, consider as ego has reached the last lane.
//     */
//     if(std::abs(current_lane_ - current_lane_max) > 2 || std::abs(current_lane_ - current_lane_min) > 2)
//     {
//         return false;
//     }



//     /*
//         [Validate current&forwardlane's primitives]
//         Create segment by the value forward_lane_.
//         If checkpoint < 4, consider as ego has reached the last lane.
//     */
//     for(int i = 0; i < route_msg.segments.size(); i++)
//     {
//         for(int j = 0; j < route_msg.segments[i].primitives.size(); j++)
//         {
//             if(i == 0)
//             {
//                 if(std::abs(current_lane_max - route_msg.segments[i].primitives[j].id) > 2 || 
//                     std::abs(current_lane_min - route_msg.segments[i].primitives[j].id) > 2)
//                 {
//                     RCLCPP_INFO(rclcpp::get_logger("route_generator_node"), "[Validate current&forwardlane] currentlane primitives out of index");
//                     return false;
//                 }
//             }
//             else if(i == 1)
//             {
//                 if(std::abs(forward_lane_max - route_msg.segments[i].primitives[j].id) > 2 || 
//                     std::abs(forward_lane_min - route_msg.segments[i].primitives[j].id) > 2)
//                 {
//                     RCLCPP_INFO(rclcpp::get_logger("route_generator_node"), "[Validate current&forwardlane] forwardlane primitives out of index");
//                     return false;
//                 }
//             }
//         }
//     }
//     return true;
// }

LaneletRoute RouteGenerator::createGoalpose(LaneletRoute & route_msg)
{
    if(ego_status_.current_lane == CurrentLane::LEFT)
    {
        route_msg.goal_pose = goalpose_vec_[ego_status_.checkpoint + 5]; 
        return route_msg;
    }
    else if(ego_status_.current_lane == CurrentLane::RIGHT)
    {
        route_msg.goal_pose = goalpose_vec_[ego_status_.checkpoint + 10]; 
        return route_msg;
    }
    else
    {
        route_msg.goal_pose = goalpose_vec_[ego_status_.checkpoint]; 
        return route_msg;
    }
    return route_msg;
}

std::vector<Pose> RouteGenerator::generateGoalpose()
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


