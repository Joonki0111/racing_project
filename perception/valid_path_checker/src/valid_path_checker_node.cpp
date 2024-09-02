#include "valid_path_checker/valid_path_checker_node.hpp"

namespace perception_component
{
  
ValidPathChecker::ValidPathChecker(const rclcpp::NodeOptions & node_options) 
  : Node("path_object_checker_node", node_options)
{
    ego_status_.is_ego_cruising = false;
    lane_status_.left_occupied = false;
    lane_status_.right_occupied = false;
    lane_status_.furthest_lane.type = ValidPath::NONE;

    sub_main_path_ = this->create_subscription<Path>("/planning/scenario_planning/lane_driving/behavior_planning/path", 
        rclcpp::QoS(1), std::bind(&ValidPathChecker::mainpathCallback, this, std::placeholders::_1));
    sub_right_path_ = this->create_subscription<Path>("/planning/path_candidate/external_request_lane_change_right", 
        rclcpp::QoS(1), std::bind(&ValidPathChecker::rightpathCallback, this, std::placeholders::_1));
    sub_left_path_ = this->create_subscription<Path>("/planning/path_candidate/external_request_lane_change_left", 
        rclcpp::QoS(1), std::bind(&ValidPathChecker::leftpathCallback, this, std::placeholders::_1));
    sub_objects_ = this->create_subscription<DetectedObjects>("/perception/object_recognition/detection/objects", 
        rclcpp::QoS(1), std::bind(&ValidPathChecker::objectsCallback, this, std::placeholders::_1));
    sub_obstacle_dist = this->create_subscription<DistToObject>("/racing/dist_to_obstacle", 
        rclcpp::QoS(1), std::bind(&ValidPathChecker::obstacledistCallback, this, std::placeholders::_1));
    sub_pose_ = this->create_subscription<Odometry>("/localization/kinematic_state", rclcpp::QoS(1),
        std::bind(&ValidPathChecker::poseCallback, this, std::placeholders::_1));

    pub_valid_path_ = this->create_publisher<ValidPath>("/racing/valid_path", rclcpp::QoS(1));
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/racing/points_within_blindspot/debug", rclcpp::QoS(1)); //debug

    main_path_msg_ptr_ = nullptr;
    right_path_msg_ptr_ = nullptr;
    left_path_msg_ptr_ = nullptr;
    objects_msg_ptr_ = nullptr;
    obstacle_dist_msg_ptr_ = nullptr;
    pose_msg_ptr_ = nullptr;

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), std::bind(&ValidPathChecker::run, this));
}

void ValidPathChecker::mainpathCallback(const Path::SharedPtr main_path_msg)
{
    main_path_msg_ptr_ = main_path_msg;
}

void ValidPathChecker::rightpathCallback(const Path::SharedPtr right_path_msg)
{
    right_path_msg_ptr_ = right_path_msg;
}

void ValidPathChecker::leftpathCallback(const Path::SharedPtr left_path_msg)
{
    left_path_msg_ptr_ = left_path_msg;
}

void ValidPathChecker::objectsCallback(const DetectedObjects::SharedPtr objects_msg)
{
    objects_msg_ptr_ = objects_msg;
}

void ValidPathChecker::obstacledistCallback(const DistToObject::SharedPtr obstacle_dist_msg)
{
    obstacle_dist_msg_ptr_ = obstacle_dist_msg;
}

void ValidPathChecker::poseCallback(const Odometry::SharedPtr pose_msg)
{
    pose_msg_ptr_ = pose_msg;
}

void ValidPathChecker::run()
{
    if(!checkSubscription())
    {
        return;
    }

    ValidPath valid_path;
    
    ego_status_.is_ego_cruising = checkEgoCruiseState(*obstacle_dist_msg_ptr_);
    getLeftRightPathExistence(valid_path, *right_path_msg_ptr_, *left_path_msg_ptr_);
    filterPathWithBlindspot(valid_path, *objects_msg_ptr_);
    pub_valid_path_->publish(valid_path);
}

bool ValidPathChecker::checkSubscription()
{
    if(main_path_msg_ptr_ == nullptr)
    {
        return false;
    }
    if(right_path_msg_ptr_ == nullptr)
    {
        return false;
    }
    if(left_path_msg_ptr_ == nullptr)
    {
        return false;
    }
    if(objects_msg_ptr_ == nullptr)
    {
        return false;
    }    
    if(obstacle_dist_msg_ptr_ == nullptr)
    {
        return false;
    }
    return true;
}

bool ValidPathChecker::checkEgoCruiseState(const DistToObject & dist_to_object_msg)
{
    if(dist_to_object_msg.dist_to_object == 0.0)
    {
        return false;
    }
    else if(dist_to_object_msg.dist_to_object > 12.0 && dist_to_object_msg.dist_to_object < 30.0) //PARAM
    {
        return true;
    }

    return false;
}

void ValidPathChecker::checkBlindSpot(const DetectedObjects & objects_msg)
{    
    lane_status_.left_occupied = false;
    lane_status_.right_occupied = false;

    for(DetectedObject object : objects_msg.objects)
    {
        const float object_x = object.kinematics.pose_with_covariance.pose.position.x;
        const float object_y = object.kinematics.pose_with_covariance.pose.position.y;

        const float object_diff = std::sqrt(std::pow(object_x, 2) + std::pow(object_y, 2));
    
        if(object_diff < 20.0f)
        {
            if(object_y > 1.5f && object_y < 4.5f) //PARAM
            {
                lane_status_.left_occupied = true;
                
            }
            else if(object_y < -1.5f && object_y > -4.5f)
            {
                lane_status_.right_occupied = true;
            }
        }
    }
}

ValidPath ValidPathChecker::getLeftRightPathExistence(
  ValidPath & valid_path, const Path & right_path, const Path & left_path)
{
    if(right_path.points.empty() && !left_path.points.empty())
    {
        valid_path.type = ValidPath::LEFT;
        return valid_path;
    } 
    else if(!right_path.points.empty() && left_path.points.empty())
    {
        valid_path.type = ValidPath::RIGHT;
        return valid_path;
    } 
    else if(!right_path.points.empty() && !left_path.points.empty())
    {
        valid_path.type = ValidPath::LEFTANDRIGHT;
        return valid_path;
    } 
    else if(right_path.points.empty() && left_path.points.empty())
    {
        valid_path.type = ValidPath::NONE;
        return valid_path;
    } 
    else
    {
        valid_path.type = ValidPath::NONE;
        return valid_path;
    }

    valid_path.type = ValidPath::NONE;
    return valid_path;
}

ValidPath ValidPathChecker::filterPathWithBlindspot(ValidPath & valid_path, const DetectedObjects & objects_msg)
{
    if(valid_path.type == ValidPath::NONE)
    {
        return valid_path; //WIP
    }

    checkBlindSpot(objects_msg);

    if(valid_path.type == ValidPath::LEFT)
    {
        if(!lane_status_.left_occupied)
        {
            return valid_path;
        }
        else
        {
            valid_path.type = ValidPath::NONE;
            return valid_path;
        }
    }
    else if(valid_path.type == ValidPath::RIGHT)
    {
        if(!lane_status_.right_occupied)
        {
            return valid_path;
        }
        else
        {
            valid_path.type = ValidPath::NONE;
            return valid_path;
        }
    }
    else if(valid_path.type == ValidPath::LEFTANDRIGHT)
    {
        if(!lane_status_.left_occupied && ego_status_.is_ego_cruising)
        {
            valid_path.type = ValidPath::LEFT;
            return valid_path;
        }
        else if(!lane_status_.right_occupied && ego_status_.is_ego_cruising)
        {
            valid_path.type = ValidPath::RIGHT;
            return valid_path;
        }
        else
        {
            valid_path.type = ValidPath::NONE;
            return valid_path;
        }
    }

    return valid_path; //something wrong
}
} //namespace perception_component
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(perception_component::ValidPathChecker)