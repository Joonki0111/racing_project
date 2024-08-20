#include "valid_path_checker/valid_path_checker_node.hpp"

namespace perception_component
{
  
ValidPathChecker::ValidPathChecker(const rclcpp::NodeOptions & node_options) 
  : Node("path_object_checker_node", node_options)
{
    sub_main_path_ = this->create_subscription<Path>("/planning/scenario_planning/lane_driving/behavior_planning/path", 
        rclcpp::QoS(1), std::bind(&ValidPathChecker::mainpathCallback, this, std::placeholders::_1));
    sub_right_path_ = this->create_subscription<Path>("/planning/path_candidate/external_request_lane_change_right", 
        rclcpp::QoS(1), std::bind(&ValidPathChecker::rightpathCallback, this, std::placeholders::_1));
    sub_left_path_ = this->create_subscription<Path>("/planning/path_candidate/external_request_lane_change_left", 
        rclcpp::QoS(1), std::bind(&ValidPathChecker::leftpathCallback, this, std::placeholders::_1));
    sub_objects_ = this->create_subscription<PredictedObjects>("/perception/object_recognition/objects", 
        rclcpp::QoS(1), std::bind(&ValidPathChecker::objectsCallback, this, std::placeholders::_1));

    pub_valid_path_ = this->create_publisher<ValidPath>("/racing/valid_path", rclcpp::QoS(1));

    main_path_msg_ptr_ = nullptr;
    right_path_msg_ptr_ = nullptr;
    left_path_msg_ptr_ = nullptr;
    objects_msg_ptr_ = nullptr;

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

void ValidPathChecker::objectsCallback(const PredictedObjects::SharedPtr objects_msg)
{
    objects_msg_ptr_ = objects_msg;
}

void ValidPathChecker::run()
{
    if(!checkSubscription())
    {
        return;
    }

    bool object_on_main_path = isObjectOnMainPath(*main_path_msg_ptr_, *objects_msg_ptr_);

    ValidPath valid_path;

    if(object_on_main_path)
    {
        getLeftRightPathExistence(valid_path, *right_path_msg_ptr_, *left_path_msg_ptr_);
        filterPathWithObject(valid_path, *right_path_msg_ptr_, *left_path_msg_ptr_);
        pub_valid_path_->publish(valid_path);
    } 
    else
    {
        valid_path.type = ValidPath::NOOVERTAKE;
        pub_valid_path_->publish(valid_path);
    }
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
    return true;
}

bool ValidPathChecker::isObjectOnMainPath(const Path & main_path_msg, const PredictedObjects & objects_msg)
{
    for(PredictedObject object : objects_msg.objects)
    {
        float object_x = object.kinematics.initial_pose_with_covariance.pose.position.x;
        float object_y = object.kinematics.initial_pose_with_covariance.pose.position.y;

      for(PathPoint point : main_path_msg.points)
      {
          float path_point_x = point.pose.position.x;
          float path_point_y = point.pose.position.y;

          float point_diff = std::sqrt(std::pow((path_point_x - object_x), 2) + 
              std::pow((path_point_y - object_y), 2));

        if(point_diff < 1.f) //[PARAM] object_on_path_threshhold
        {
            return true;
        }
      }
    }
    return false;
}

bool ValidPathChecker::isObjectOnLaneChangingPath(const Path & path_msg, const PredictedObjects & objects_msg)
{
    for(PredictedObject object : objects_msg.objects)
    {
        float object_x = object.kinematics.initial_pose_with_covariance.pose.position.x;
        float object_y = object.kinematics.initial_pose_with_covariance.pose.position.y;

        for(int i = 0; i < path_msg.points.size(); i++)
        {
            float path_point_x_1 = path_msg.points[i].pose.position.x;
            float path_point_y_1 = path_msg.points[i].pose.position.y;

            float path_point_x_2 = path_msg.points[i+1].pose.position.x;
            float path_point_y_2 = path_msg.points[i+1].pose.position.y;

            float pathpoint_diff = std::sqrt(std::pow((path_point_x_1 - path_point_x_2), 2) + 
                std::pow((path_point_y_1 - path_point_y_2), 2));

            if(pathpoint_diff > 10.f || pathpoint_diff < 3.5f)
            {
                continue;
            }

            float dx = (path_point_x_2 - path_point_x_1) / pathpoint_diff;
            float dy = (path_point_y_2 - path_point_y_1) / pathpoint_diff;

            std::vector<Point> points_within_path_points;

            for (int j = 0; j < 7; j++) 
            {
              Point p;
              p.x = path_point_x_1 + j * 0.8f * dx;
              p.y = path_point_y_1 + j * 0.8f * dy;
              points_within_path_points.push_back(p);
            }

            for(Point p : points_within_path_points)
            {
                float pathpoint_object_diff = std::sqrt(std::pow((p.x - object_x), 2) + 
                    std::pow((p.y - object_y), 2)); 

              if(pathpoint_object_diff < 1.f)
              {
                return true;
              }
            }
        }
    }
    return false;
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

ValidPath ValidPathChecker::filterPathWithObject(
  ValidPath & valid_path, const Path & right_path, const Path & left_path)
{
    switch(valid_path.type)
    {
        case ValidPath::LEFT:
            if(!isObjectOnLaneChangingPath(left_path, *objects_msg_ptr_))
            {
                return valid_path;
            } 
            else
            {
                valid_path.type = ValidPath::NONE;
                return valid_path;
            }
        case ValidPath::RIGHT:
            if(!isObjectOnLaneChangingPath(left_path, *objects_msg_ptr_))
            {
                return valid_path;
            } 
            else
            {
                valid_path.type = ValidPath::NONE;
                return valid_path;
            }
        case ValidPath::LEFTANDRIGHT:
        {
            bool is_left_valid = isObjectOnLaneChangingPath(left_path, *objects_msg_ptr_);
            bool is_right_valid = isObjectOnLaneChangingPath(right_path, *objects_msg_ptr_);
            if(is_left_valid && is_right_valid)
            {
                valid_path.type = ValidPath::NONE;
                return valid_path;
            } 
            else if(is_left_valid && !is_right_valid)
            {
                valid_path.type = ValidPath::RIGHT;
                return valid_path;
            } 
            else if(!is_left_valid && is_right_valid)
            {
                valid_path.type = ValidPath::LEFT;
                return valid_path;
            } 
            else
            {
                return valid_path;
            }
        }
        default:
            return valid_path;
    }
    return valid_path;
}
} //namespace perception_component
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(perception_component::ValidPathChecker)