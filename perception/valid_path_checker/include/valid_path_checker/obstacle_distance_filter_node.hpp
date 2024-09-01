#ifndef VALID_PATH_CHECKER__OBSTACLE_DISTANCE_FILTER_NODE_HPP
#define VALID_PATH_CHECKER__OBSTACLE_DISTANCE_FILTER_NODE_HPP

#include <rclcpp/rclcpp.hpp> 
#include "perception_msgs/msg/dist_to_object.hpp"

namespace perception_component
{
using perception_msgs::msg::DistToObject;

class ObstacleDistFilter : public rclcpp::Node{
    public:
        explicit ObstacleDistFilter(const rclcpp::NodeOptions & node_options);

    private:
        rclcpp::Subscription<DistToObject>::SharedPtr sub_obstacle_dist_raw;

        rclcpp::Publisher<DistToObject>::SharedPtr pub_obstacle_dist_;

        rclcpp::TimerBase::SharedPtr timer_;

        int count_;
        int true_flag_;

        DistToObject::SharedPtr obstacle_dist_raw_msg_ptr_;

        void obstacledistCallback(const DistToObject::SharedPtr obstacle_dist_raw_msg);
        void run();
        bool checkSubscription();
        bool checkObstacleDistExistence(const DistToObject & obstacle_dist_raw);
};
} //namespace perception_component
#endif //VALID_PATH_CHECKER__OBSTACLE_DISTANCE_FILTER_NODE_HPP