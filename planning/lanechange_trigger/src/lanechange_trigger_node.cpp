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
    sub_lanechange_status_ = this->create_subscription<LanechangeStatus>(
        "/racing/lanechange_status", rclcpp::QoS(1),
            std::bind(&LanechangeTrigger::lanechangestatusCallback, this, std::placeholders::_1));

    pub_rtc_status_ = this->create_publisher<CooperateStatusArray>("/api/external/get/rtc_status", rclcpp::QoS(1));

    cli_trigger_ = this->create_client<example_interfaces::srv::Trigger>("/racing/lc_trigger");
    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&LanechangeTrigger::run, this));

    valid_path_msg_ptr_ = nullptr;
    rtc_status_msg_ptr_ = nullptr;
    lanechange_status_msg_ptr_ = nullptr;
}

void LanechangeTrigger::validpathCallback(const ValidPath::SharedPtr valid_path_msg)
{
    valid_path_msg_ptr_ = valid_path_msg;
}

void LanechangeTrigger::rtcStatusCallback(const CooperateStatusArray::SharedPtr rtc_status_msg)
{
    rtc_status_msg_ptr_ = rtc_status_msg;
}

void LanechangeTrigger::lanechangestatusCallback(const LanechangeStatus::SharedPtr lanechange_status_msg)
{
    lanechange_status_msg_ptr_ = lanechange_status_msg;
}

void LanechangeTrigger::run()
{
    if(!checkSubscription())
    {
        return;
    }

    CooperateStatusArray statuses_msg = createStatusesWithDirection(*rtc_status_msg_ptr_, *valid_path_msg_ptr_);
    pub_rtc_status_->publish(statuses_msg);

    requestLanechange(*lanechange_status_msg_ptr_, statuses_msg);
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
    if(lanechange_status_msg_ptr_ == nullptr)
    {
        return false;
    }
    return true;
}

CooperateStatusArray LanechangeTrigger::createStatusesWithDirection(CooperateStatusArray & statuses_vec, const ValidPath & valid_path)
{
    CooperateStatusArray filtered_statuses_vec;

    for(int i = 0; i < statuses_vec.statuses.size(); i++)
    {
        if(valid_path.type == ValidPath::LEFT)
        {
            if(statuses_vec.statuses[i].module.type == Module::EXT_REQUEST_LANE_CHANGE_LEFT)
            {
                statuses_vec.statuses[i].safe = true;
                filtered_statuses_vec.statuses.push_back(statuses_vec.statuses[i]);
                return filtered_statuses_vec;
            }
        } 
        else if(valid_path.type == ValidPath::RIGHT)
        {
            if(statuses_vec.statuses[i].module.type == Module::EXT_REQUEST_LANE_CHANGE_RIGHT)
            {
                statuses_vec.statuses[i].safe = true;
                filtered_statuses_vec.statuses.push_back(statuses_vec.statuses[i]);
                return filtered_statuses_vec;
            }      
        } 
        else if(valid_path.type == ValidPath::LEFTANDRIGHT)
        {
            if(statuses_vec.statuses[i].module.type == Module::EXT_REQUEST_LANE_CHANGE_LEFT)
            {
                statuses_vec.statuses[i].safe = true;
                filtered_statuses_vec.statuses.push_back(statuses_vec.statuses[i]);
                return filtered_statuses_vec;
            }
        }
        else
        {
            return filtered_statuses_vec;
        }
    }
    return filtered_statuses_vec;
}

void LanechangeTrigger::requestLanechange(const LanechangeStatus & lanechange_status_msg, const CooperateStatusArray & statuses_msg)
{
    example_interfaces::srv::Trigger::Request::SharedPtr request = std::make_shared<example_interfaces::srv::Trigger::Request>();

    if(statuses_msg.statuses.empty())
    {
        return;
    }
    else if(lanechange_status_msg.type == LanechangeStatus::CHANGINGLEFT || 
        lanechange_status_msg.type == LanechangeStatus::CHANGINGRIGHT)
    {
        return;
    }

    if(lanechange_status_msg.type == LanechangeStatus::NONE)
    {
        cli_trigger_->async_send_request(request);
    }
}
} // namespace planning_component
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(planning_component::LanechangeTrigger)