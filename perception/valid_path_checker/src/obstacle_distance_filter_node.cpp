#include "valid_path_checker/obstacle_distance_filter_node.hpp"

namespace perception_component
{

ObstacleDistFilter::ObstacleDistFilter(const rclcpp::NodeOptions & node_options) 
  : Node("path_object_checker_node", node_options)
{
    count_ = 0;
    true_flag_ = 0;

    sub_obstacle_dist_raw = this->create_subscription<DistToObject>("/racing/dist_to_obstacle_raw", 
        rclcpp::QoS(1), std::bind(&ObstacleDistFilter::obstacledistCallback, this, std::placeholders::_1));

    pub_obstacle_dist_ = this->create_publisher<DistToObject>("/racing/dist_to_obstacle", rclcpp::QoS(1));

    obstacle_dist_raw_msg_ptr_ = nullptr;

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), std::bind(&ObstacleDistFilter::run, this));
}

void ObstacleDistFilter::obstacledistCallback(const DistToObject::SharedPtr obstacle_dist_raw_msg)
{
    obstacle_dist_raw_msg_ptr_ = obstacle_dist_raw_msg;
}

void ObstacleDistFilter::run()
{
    if(!checkSubscription())
    {
        return;
    }

    DistToObject dist_to_object_msg;

    bool is_obstacle_detected = checkObstacleDistExistence(*obstacle_dist_raw_msg_ptr_);

    if(is_obstacle_detected)
    {
        dist_to_object_msg.dist_to_object = obstacle_dist_raw_msg_ptr_->dist_to_object;
        pub_obstacle_dist_->publish(dist_to_object_msg);
    }
    else
    {
        dist_to_object_msg.dist_to_object = 0.0;
        pub_obstacle_dist_->publish(dist_to_object_msg);
    }
}

bool ObstacleDistFilter::checkSubscription()
{
    if(obstacle_dist_raw_msg_ptr_ == nullptr)
    {
        return false;
    }
    return true;
}

bool ObstacleDistFilter::checkObstacleDistExistence(const DistToObject & obstacle_dist_raw)
{
    if(count_ != obstacle_dist_raw.count)
    {
        true_flag_++;

        if(true_flag_ > 2)
        {
            true_flag_ = 2;
        }
    }
    else
    {
        true_flag_ = 0;

    }

    count_ = obstacle_dist_raw.count;
    return true_flag_ == 2;
}
} //namespace perception_component
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(perception_component::ObstacleDistFilter)