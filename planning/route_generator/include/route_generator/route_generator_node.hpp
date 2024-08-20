#ifndef ROUTE_GENERATOR__ROUTE_NODE_HPP_
#define ROUTE_GENERATOR__ROUTE_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <fstream>
#include <sstream>

#include "std_msgs/msg/int32.hpp"
#include "autoware_planning_msgs/msg/lanelet_route.hpp"
#include "autoware_planning_msgs/msg/lanelet_segment.hpp"
#include "autoware_planning_msgs/msg/lanelet_primitive.hpp"
#include "autoware_adapi_v1_msgs/srv/set_route_points.hpp"   
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace planning_component
{
using autoware_planning_msgs::msg::LaneletRoute;
using autoware_planning_msgs::msg::LaneletSegment;
using autoware_planning_msgs::msg::LaneletPrimitive;
using autoware_adapi_v1_msgs::srv::SetRoutePoints;  
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::Point;  
using geometry_msgs::msg::Quaternion;  

class RouteGenerator : public rclcpp::Node
{
    private:
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_pose_;
        rclcpp::Publisher<LaneletRoute>::SharedPtr pub_route_;

        rclcpp::Client<SetRoutePoints>::SharedPtr client_;  

        rclcpp::TimerBase::SharedPtr timer_;

        nav_msgs::msg::Odometry pose_msg_;
        std::vector<Point> checkpoint_vec_;
        std::vector<Pose> goalpose_vec_;

        nav_msgs::msg::Odometry::SharedPtr pose_msg_ptr_;
        float dist_threshold_;
        int current_lane_;
        int forward_lane_;
        int checkpoint_;
        bool generate_route_executed_;
        
        void sendRequest();
        void run();
        bool checkSubscription();
        void poseCallback(const nav_msgs::msg::Odometry::SharedPtr pose_msg);
        int getCheckpointEx();
        int calcClosestCheckpoint(const geometry_msgs::msg::Point & pose);
        LaneletRoute createRoute(const nav_msgs::msg::Odometry & ego_pose, 
            const int & checkpoint, const int & checkpoint_ex);
        std::vector<Pose> generateGoalpoint();
        std::vector<Point> generateCheckpoint();

    public:
        explicit RouteGenerator(const rclcpp::NodeOptions & node_options);
};
}
#endif //ROUTE_GENERATOR__ROUTE_NODE_HPP_
