#ifndef LANECHANGE_TRIGGER_NODE__LANECHANGE_TRIGGER_NODE_HPP_
#define LANECHANGE_TRIGGER_NODE__LANECHANGE_TRIGGER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include "tier4_rtc_msgs/msg/cooperate_status_array.hpp"
#include "tier4_rtc_msgs/msg/cooperate_status.hpp"
#include "example_interfaces/srv/trigger.hpp"
#include "example_interfaces/srv/trigger.hpp"
#include "perception_msgs/msg/valid_path.hpp"
#include "planning_msgs/msg/lanechange_status.hpp"

#include <algorithm>

namespace planning_component
{
using tier4_rtc_msgs::msg::CooperateStatusArray;
using tier4_rtc_msgs::msg::CooperateStatus;
using tier4_rtc_msgs::msg::Module;
using tier4_rtc_msgs::msg::Command;
using perception_msgs::msg::ValidPath;
using planning_msgs::msg::LanechangeStatus;  

class LanechangeTrigger : public rclcpp::Node{
    public:
        explicit LanechangeTrigger(const rclcpp::NodeOptions & node_options);

    private:
        rclcpp::Subscription<ValidPath>::SharedPtr sub_valid_path_;
        rclcpp::Subscription<CooperateStatusArray>::SharedPtr sub_rtc_status_;
        rclcpp::Subscription<LanechangeStatus>::SharedPtr sub_lanechange_status_;

        rclcpp::Publisher<CooperateStatusArray>::SharedPtr pub_rtc_status_;

        rclcpp::Client<example_interfaces::srv::Trigger>::SharedPtr cli_trigger_;

        rclcpp::TimerBase::SharedPtr timer_;

        ValidPath::SharedPtr valid_path_msg_ptr_;
        CooperateStatusArray::SharedPtr rtc_status_msg_ptr_;
        LanechangeStatus::SharedPtr lanechange_status_msg_ptr_;

        void validpathCallback(const ValidPath::SharedPtr valid_path_msg);
        void rtcStatusCallback(const CooperateStatusArray::SharedPtr rtc_status_msg);
        void lanechangestatusCallback(const LanechangeStatus::SharedPtr lanechange_status_msg);
        bool checkSubscription();
        void run();
        CooperateStatusArray triggerWithDirection(CooperateStatusArray & statuses_vec, const ValidPath & valid_path);
};
}
#endif  //LANECHANGE_TRIGGER_NODE__LANECHANGE_TRIGGER_NODE_HPP_
