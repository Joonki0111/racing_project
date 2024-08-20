#ifndef VALID_PATH_CHECKER__VALID_PATH_CHECKER_NODE_HPP
#define VALID_PATH_CHECKER__VALID_PATH_CHECKER_NODE_HPP

#include <rclcpp/rclcpp.hpp> 
#include "autoware_auto_planning_msgs/msg/path.hpp"
#include "autoware_auto_planning_msgs/msg/path_point.hpp"
#include "autoware_auto_perception_msgs/msg/predicted_objects.hpp"
#include "autoware_auto_perception_msgs/msg/predicted_object.hpp"
#include "perception_msgs/msg/valid_path.hpp"
#include "geometry_msgs/msg/point.hpp"

namespace perception_component
{
using autoware_auto_planning_msgs::msg::Path;
using autoware_auto_planning_msgs::msg::PathPoint;
using autoware_auto_perception_msgs::msg::PredictedObjects;
using autoware_auto_perception_msgs::msg::PredictedObject;
using perception_msgs::msg::ValidPath;
using geometry_msgs::msg::Point;

class ValidPathChecker : public rclcpp::Node{
    public:
        explicit ValidPathChecker(const rclcpp::NodeOptions & node_options);

    private:
        rclcpp::Subscription<Path>::SharedPtr sub_main_path_;
        rclcpp::Subscription<Path>::SharedPtr sub_right_path_;
        rclcpp::Subscription<Path>::SharedPtr sub_left_path_;
        rclcpp::Subscription<PredictedObjects>::SharedPtr sub_objects_;

        rclcpp::Publisher<ValidPath>::SharedPtr pub_valid_path_;

        rclcpp::TimerBase::SharedPtr timer_;

        Path::SharedPtr main_path_msg_ptr_;
        Path::SharedPtr right_path_msg_ptr_;
        Path::SharedPtr left_path_msg_ptr_;
        PredictedObjects::SharedPtr objects_msg_ptr_;

        void mainpathCallback(const Path::SharedPtr main_path_msg);
        void rightpathCallback(const Path::SharedPtr right_path_msg);
        void leftpathCallback(const Path::SharedPtr left_path_msg);
        void objectsCallback(const PredictedObjects::SharedPtr objects_msg);
        void run();
        bool checkSubscription();
        bool isObjectOnMainPath(const Path & main_path_msg, const PredictedObjects & objects_msg);
        bool isObjectOnLaneChangingPath(const Path & path_msg, const PredictedObjects & objects_msg);
        ValidPath getLeftRightPathExistence(ValidPath & valid_path, const Path & right_path, const Path & left_path);
        ValidPath filterPathWithObject(ValidPath & valid_path, const Path & right_path, const Path & left_path);
};
} //namespace perception_component
#endif //VALID_PATH_CHECKER__VALID_PATH_CHECKER_NODE_HPP