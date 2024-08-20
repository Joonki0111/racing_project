#include "lanechange_trigger_node/lanechange_trigger_node.hpp"

namespace planning_component
{

LanechangeTrigger::LanechangeTrigger(const rclcpp::NodeOptions & node_options) 
  : Node("lanechange_trigger_node", node_options)
  {
    sub_valid_path_ = this->create_subscription<ValidPath>("/racing/valid_path",
        rclcpp::QoS(1), std::bind(&LanechangeTrigger::validpathCallback, this, std::placeholders::_1));
    sub_rtc_status_ = this->create_subscription<CooperateStatusArray>("/api/external/get/rtc_status_",
        rclcpp::QoS(1), std::bind(&LanechangeTrigger::rtcStatusCallback, this, std::placeholders::_1));

    pub_rtc_status_ = this->create_publisher<CooperateStatusArray>("/api/external/get/rtc_status", rclcpp::QoS(1));

    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&LanechangeTrigger::run, this));

    valid_path_msg_ptr_ = nullptr;
    rtc_status_msg_ptr_ = nullptr;
}

void LanechangeTrigger::validpathCallback(const ValidPath::SharedPtr valid_path_msg)
{
    valid_path_msg_ptr_ = valid_path_msg;
}

void LanechangeTrigger::rtcStatusCallback(const CooperateStatusArray::SharedPtr rtc_status_msg)
{
    rtc_status_msg_ptr_ = rtc_status_msg;
}

void LanechangeTrigger::run()
{
    if(!checkSubscription())
    {
        return;
    }
    CooperateStatusArray statuses_msg = triggerWithDirection(*rtc_status_msg_ptr_, *valid_path_msg_ptr_);
    pub_rtc_status_->publish(statuses_msg);
}

bool LanechangeTrigger::checkSubscription()
{
    if(valid_path_msg_ptr_ == nullptr)
    {
        return false;
    }
    if(rtc_status_msg_ptr_ == nullptr)
    {
        return false;
    }
    return true;
}

CooperateStatusArray LanechangeTrigger::triggerWithDirection(CooperateStatusArray & statuses_vec, const ValidPath & valid_path)
{
    CooperateStatusArray filtered_statuses_vec;

    for(int i = 0; i < statuses_vec.statuses.size(); i++)
    {
        if(valid_path.type == ValidPath::LEFT)
        {
            if(statuses_vec.statuses[i].module.type == Module::EXT_REQUEST_LANE_CHANGE_LEFT)
            {
                filtered_statuses_vec.statuses.push_back(statuses_vec.statuses[i]);
                return filtered_statuses_vec;
            }
        } 
        else if(valid_path.type == ValidPath::RIGHT)
        {
            if(statuses_vec.statuses[i].module.type == Module::EXT_REQUEST_LANE_CHANGE_RIGHT)
            {
                filtered_statuses_vec.statuses.push_back(statuses_vec.statuses[i]);
                return filtered_statuses_vec;
            }      
        } 
        else if(valid_path.type == ValidPath::LEFTANDRIGHT)
        {
            if(statuses_vec.statuses[i].module.type == Module::EXT_REQUEST_LANE_CHANGE_LEFT)
            {
                filtered_statuses_vec.statuses.push_back(statuses_vec.statuses[i]);
                return filtered_statuses_vec;
            }
        }
    }
    return filtered_statuses_vec;
}
} // namespace planning_component
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(planning_component::LanechangeTrigger)