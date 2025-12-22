#include "AutoaimPriority.hpp"

#include <memory>
#include <rclcpp/parameter_client.hpp>

namespace helios_cv
{

using namespace std::chrono_literals;

AutoaimPriority::AutoaimPriority(const rclcpp::NodeOptions& options) : rclcpp::Node("autoaim_priority", options)
{
  priority_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/priority_sequence", rclcpp::SystemDefaultsQoS(),
      std::bind(&AutoaimPriority::priority_callback, this, std::placeholders::_1));
  armor_predictor_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "/armor_predictor");
   timer = this->create_wall_timer(100ms, std::bind(&AutoaimPriority::set_param_callback, this));
}

void AutoaimPriority::priority_callback(std_msgs::msg::String::SharedPtr priority_msg)
{
  const static std::map<int, std::string> number_map{ { 1, "1" }, { 2, "2" },       { 3, "3" },     { 4, "4" },
                                                      { 5, "5" }, { 6, "outpost" }, { 7, "guard" }, { 8, "base" } };
  if (last_priority_sequence_ != priority_msg->data)
  {
    new_priority_.clear();
    for (auto charactors : priority_msg->data)
    {
      int number = charactors - '0';
      new_priority_.emplace_back(number_map.find(number)->second);
    }
    set_new_flag_ = true;
  }
  last_priority_sequence_ = priority_msg->data;
}

void AutoaimPriority::set_param_callback()
{
  if (armor_predictor_param_client_->wait_for_service(10ms) )
  {
    if (set_new_flag_)
    {
      RCLCPP_INFO(logger_, "Change predictor's priority");
      auto param = rclcpp::Parameter{ "priority_sequence", new_priority_ };
      if (!set_armor_predictor_param_future_.valid() ||
          set_armor_predictor_param_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
      {
        RCLCPP_INFO(logger_, "armor predictor is setting priority sequence");
        set_armor_predictor_param_future_ = armor_predictor_param_client_->set_parameters(
            { param }, [this, param](const ResultFuturePtr& results) {
              for (const auto& result : results.get())
              {
                if (!result.successful)
                {
                  RCLCPP_ERROR(logger_, "armor predictor has failed to set priority sequence");
                  return;
                }
              }
              set_new_flag_ = false;
              RCLCPP_INFO(logger_, "armor predictor has successfully set priority sequence!");
            });
      }
    }

  }
  else
  {
    RCLCPP_ERROR(logger_, "Predictor service not ready");
  }
}

}  // namespace helios_cv

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(helios_cv::AutoaimPriority)