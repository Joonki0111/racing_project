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
#include "planning_msgs/msg/lanechange_status.hpp"

namespace planning_component
{
using autoware_planning_msgs::msg::LaneletRoute;
using autoware_planning_msgs::msg::LaneletSegment;
using autoware_planning_msgs::msg::LaneletPrimitive;
using autoware_adapi_v1_msgs::srv::SetRoutePoints;  
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::Point;  
using geometry_msgs::msg::Quaternion;  
using planning_msgs::msg::LanechangeStatus;  

class RouteGenerator : public rclcpp::Node
{
    public:
        explicit RouteGenerator(const rclcpp::NodeOptions & node_options);
        
    private:
        enum class CurrentLane : uint8_t 
        {
            LEFT = 0,
            CENTER,
            RIGHT
        };

        struct Status
        {
            int current_lane_id;
            int forward_lane_id;
            int checkpoint;
            int checkpoint_ex;
            bool is_lanechanging;
            CurrentLane current_lane;
        };

        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_pose_;
        rclcpp::Subscription<LanechangeStatus>::SharedPtr sub_lanechange_status_;

        rclcpp::Publisher<LaneletRoute>::SharedPtr pub_route_;

        rclcpp::Client<SetRoutePoints>::SharedPtr client_;  

        rclcpp::TimerBase::SharedPtr timer_;

        int reserved_checkpoint_;
        bool checkpoint_reserved_;
        LanechangeStatus::SharedPtr lanechange_status_msg_ptr_;
        nav_msgs::msg::Odometry::SharedPtr pose_msg_ptr_;

        std::vector<Point> checkpoint_vec_;
        std::vector<Pose> goalpose_vec_;
        Status ego_status_;
        float dist_threshold_;

        void sendRequest();
        void poseCallback(const nav_msgs::msg::Odometry::SharedPtr pose_msg);
        void lanechangestatusCallback(const LanechangeStatus::SharedPtr lanechange_status_msg);
        void run();
        bool checkSubscription();
        void calcClosestCheckpoint(const geometry_msgs::msg::Point & pose);
        void updateCurrentLane(const LanechangeStatus & lanechange_status_msg);
        void createFirstRoute(const nav_msgs::msg::Odometry & ego_pose);
        void createRoute(const nav_msgs::msg::Odometry & ego_pose);
        LaneletRoute createStartpose(LaneletRoute & route_msg, const nav_msgs::msg::Odometry & ego_pose);
        LaneletRoute createSegment(LaneletRoute & route_msg);
        LaneletRoute checkCheckpointTrigger(LaneletRoute & route_msg);
        bool validateSegment(const LaneletRoute & route_msg);
        LaneletRoute createGoalpose(LaneletRoute & route_msg);
        std::vector<Pose> generateGoalpose();
        std::vector<Point> generateCheckpoint();
};
}
#endif //ROUTE_GENERATOR__ROUTE_NODE_HPP_
