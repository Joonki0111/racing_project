#ifndef VALID_PATH_CHECKER__VALID_PATH_CHECKER_NODE_HPP
#define VALID_PATH_CHECKER__VALID_PATH_CHECKER_NODE_HPP

#include <rclcpp/rclcpp.hpp> 
#include "autoware_auto_planning_msgs/msg/path.hpp"
#include "autoware_auto_planning_msgs/msg/path_point.hpp"
#include "autoware_auto_perception_msgs/msg/detected_objects.hpp"
#include "autoware_auto_perception_msgs/msg/detected_object.hpp"
#include "perception_msgs/msg/valid_path.hpp"
#include "perception_msgs/msg/dist_to_object.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace perception_component
{
using autoware_auto_planning_msgs::msg::Path;
using autoware_auto_planning_msgs::msg::PathPoint;
using autoware_auto_perception_msgs::msg::DetectedObjects;
using autoware_auto_perception_msgs::msg::DetectedObject;
using perception_msgs::msg::ValidPath;
using geometry_msgs::msg::Point;
using perception_msgs::msg::DistToObject;
using nav_msgs::msg::Odometry;

class ValidPathChecker : public rclcpp::Node{
    public:
        explicit ValidPathChecker(const rclcpp::NodeOptions & node_options);

    private:
        struct Status
        {
            bool is_ego_cruising;
        };

        struct LaneStatus
        {
            bool left_occupied;
            bool right_occupied;
            ValidPath furthest_lane;
        };

        rclcpp::Subscription<Path>::SharedPtr sub_main_path_;
        rclcpp::Subscription<Path>::SharedPtr sub_right_path_;
        rclcpp::Subscription<Path>::SharedPtr sub_left_path_;
        rclcpp::Subscription<DetectedObjects>::SharedPtr sub_objects_;
        rclcpp::Subscription<Odometry>::SharedPtr sub_pose_;
        rclcpp::Subscription<DistToObject>::SharedPtr sub_obstacle_dist;

        rclcpp::Publisher<ValidPath>::SharedPtr pub_valid_path_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

        rclcpp::TimerBase::SharedPtr timer_;

        Status ego_status_;
        LaneStatus lane_status_;

        Path::SharedPtr main_path_msg_ptr_;
        Path::SharedPtr right_path_msg_ptr_;
        Path::SharedPtr left_path_msg_ptr_;
        DetectedObjects::SharedPtr objects_msg_ptr_;
        DistToObject::SharedPtr obstacle_dist_msg_ptr_;
        Odometry::SharedPtr pose_msg_ptr_;

        void mainpathCallback(const Path::SharedPtr main_path_msg);
        void rightpathCallback(const Path::SharedPtr right_path_msg);
        void leftpathCallback(const Path::SharedPtr left_path_msg);
        void objectsCallback(const DetectedObjects::SharedPtr objects_msg);
        void obstacledistCallback(const DistToObject::SharedPtr obstacle_dist_msg);
        void poseCallback(const Odometry::SharedPtr blindspot_msg);
        void run();
        bool checkSubscription();
        bool checkEgoCruiseState(const DistToObject & dist_to_object_msg);
        void checkBlindSpot(const DetectedObjects & objects_msg);
        ValidPath getLeftRightPathExistence(ValidPath & valid_path, const Path & right_path, const Path & left_path);
        ValidPath filterPathWithBlindspot(ValidPath & valid_path, const DetectedObjects & objects_msg);
};
} //namespace perception_component
#endif //VALID_PATH_CHECKER__VALID_PATH_CHECKER_NODE_HPP