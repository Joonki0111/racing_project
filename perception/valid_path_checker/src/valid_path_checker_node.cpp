#include "valid_path_checker/valid_path_checker_node.hpp"

namespace perception_component
{
  
ValidPathChecker::ValidPathChecker(const rclcpp::NodeOptions & node_options) 
  : Node("path_object_checker_node", node_options)
{
    ego_status_.is_ego_cruising = false;

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
    filterPathWithBlindspot(valid_path, *pose_msg_ptr_, *objects_msg_ptr_);
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

std::pair<bool, bool> ValidPathChecker::checkBlindSpot(const Odometry & pose_msg, const DetectedObjects & objects_msg)
{    
    std::pair<bool, bool> current_side_object(false, false);

    for(DetectedObject object : objects_msg.objects)
    {
        const float object_x = object.kinematics.pose_with_covariance.pose.position.x;
        const float object_y = object.kinematics.pose_with_covariance.pose.position.y;

        const float object_diff = std::sqrt(std::pow(object_x, 2) + std::pow(object_y, 2));
    
        if(object_diff < 30.0f)
        {

            if(object_y > 1.5f && object_y < 4.5f) //PARAM
            {
                current_side_object.first = true; //object is on left
            }
            else if(object_y < -1.5f && object_y > -4.5f)
            {
                current_side_object.second = true; //object is on right
            }
        }
    }

    return current_side_object;
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

ValidPath ValidPathChecker::filterPathWithBlindspot(ValidPath & valid_path, const Odometry & pose_msg, const DetectedObjects & objects_msg)
{
    if(valid_path.type == ValidPath::NONE)
    {
        return valid_path; //WIP
    }

    std::pair<bool, bool> current_side_object = checkBlindSpot(pose_msg, objects_msg);
    if(valid_path.type == ValidPath::LEFT)
    {
        if(!current_side_object.first)
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
        if(!current_side_object.second)
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
        if(!current_side_object.first && ego_status_.is_ego_cruising)
        {
            valid_path.type = ValidPath::LEFT;
            return valid_path;
        }
        else if(!current_side_object.second && ego_status_.is_ego_cruising)
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

void ValidPathChecker::pubMarker(const std::vector<Point> & point_vec)
{
    visualization_msgs::msg::MarkerArray marker_vec;
    int id = 0;

    // for(Point point : object.lateral_safe_boundary)
    for(int i = 0; i < point_vec.size(); i++)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();
        marker.ns = "point";
        marker.id = id++;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action =  visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = point_vec[i].x;
        marker.pose.position.y = point_vec[i].y;
        marker.pose.position.z = point_vec[i].z;
        marker.scale.x = 0.5;
        marker.scale.y = 0.5;
        marker.scale.z = 0.5;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker_vec.markers.push_back(marker);
    }
    marker_pub_->publish(marker_vec);
}
} //namespace perception_component
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(perception_component::ValidPathChecker)