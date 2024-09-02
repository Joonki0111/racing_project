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
    sub_main_path_ = this->create_subscription<Path>("/planning/scenario_planning/lane_driving/behavior_planning/path", 
        rclcpp::QoS(1), std::bind(&LanechangeTrigger::mainpathCallback, this, std::placeholders::_1));

    pub_rtc_status_ = this->create_publisher<CooperateStatusArray>("/api/external/get/rtc_status", rclcpp::QoS(1));
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/HYD_racing/perception/objects_with_boundary/debug_marker", rclcpp::QoS(1)); //debug

    cli_trigger_ = this->create_client<example_interfaces::srv::Trigger>("/racing/lc_trigger");
    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&LanechangeTrigger::run, this));

    valid_path_msg_ptr_ = nullptr;
    rtc_status_msg_ptr_ = nullptr;
    lanechange_status_msg_ptr_ = nullptr;
    main_path_msg_ptr_ = nullptr;
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

void LanechangeTrigger::mainpathCallback(const Path::SharedPtr main_path_msg)
{
    main_path_msg_ptr_ = main_path_msg;
}

void LanechangeTrigger::run()
{
    if(!checkSubscription())
    {
        return;
    }

    CooperateStatusArray statuses_msg = createStatusesWithDirection(*rtc_status_msg_ptr_, *valid_path_msg_ptr_);
    pub_rtc_status_->publish(statuses_msg);

    const float curvature = calcCurvature(*main_path_msg_ptr_);
    if(curvature < 0.01f)
    {
        requestLanechange(*lanechange_status_msg_ptr_, statuses_msg);
    }
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

float LanechangeTrigger::calcCurvature(const Path & main_path_msg)
{
    std::vector<Point> lane_bound_vec;

    for(int i = 0; i < 15; i += 5) //PARAM
    {
        lane_bound_vec.push_back(main_path_msg.right_bound[i]);
    }

    const float area_of_triangle = std::fabs((lane_bound_vec[0].x * (lane_bound_vec[1].y - lane_bound_vec[2].y) + 
        lane_bound_vec[1].x * (lane_bound_vec[2].y - lane_bound_vec[0].y) + 
            lane_bound_vec[2].x * (lane_bound_vec[0].y - lane_bound_vec[1].y)) / 2);

    const float d21 = std::sqrt(std::pow((lane_bound_vec[1].x - lane_bound_vec[0].x), 2) + std::pow((lane_bound_vec[1].y - lane_bound_vec[0].y), 2));
    const float d32 = std::sqrt(std::pow((lane_bound_vec[2].x - lane_bound_vec[1].x), 2) + std::pow((lane_bound_vec[2].y - lane_bound_vec[1].y), 2));
    const float d31 = std::sqrt(std::pow((lane_bound_vec[2].x - lane_bound_vec[0].x), 2) + std::pow((lane_bound_vec[2].y - lane_bound_vec[0].y), 2)); 

    const float curvature = 4 * area_of_triangle / (d21 * d32 * d31);  

    pubMarker(lane_bound_vec);
    return curvature;
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

void LanechangeTrigger::pubMarker(const std::vector<Point> points_vec)
{
  visualization_msgs::msg::MarkerArray marker_vec;
  int id = 0;

  for(int i = 0; i < points_vec.size(); i++)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = this->now();
    marker.ns = "point";
    marker.id = id++;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action =  visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = points_vec[i].x;
    marker.pose.position.y = points_vec[i].y;
    marker.pose.position.z = points_vec[i].z;
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker_vec.markers.push_back(marker);
  }
  marker_pub_->publish(marker_vec);
}
} // namespace planning_component
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(planning_component::LanechangeTrigger)