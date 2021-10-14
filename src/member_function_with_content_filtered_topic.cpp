// Copyright 2021 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter_value.hpp"
#include "rcl_interfaces/msg/parameter_event.hpp"
#include "rcl_interfaces/msg/parameter.hpp"


using std::placeholders::_1;

class MinimalSubscriberWithContentFilteredTopic : public rclcpp::Node
{
public:
  MinimalSubscriberWithContentFilteredTopic()
  : Node("minimal_subscriber_with_content_filtered_topic",
    rclcpp::NodeOptions().allow_undeclared_parameters(true).
      automatically_declare_parameters_from_overrides(true))
  {
    auto options = rclcpp::SubscriptionOptions();
    options.content_filter_options.filter_expression = "node=%0";
    std::ostringstream expression_parameter;
    expression_parameter << "'" << this->get_fully_qualified_name() << "'";
    options.content_filter_options.expression_parameters = {expression_parameter.str()};

    subscription_ = this->create_subscription<rcl_interfaces::msg::ParameterEvent>(
      "/parameter_events", 10,
      std::bind(&MinimalSubscriberWithContentFilteredTopic::topic_callback, this, _1),
      options);
    if (!subscription_->is_cft_enabled()) {
      RCLCPP_WARN(this->get_logger(),
        "Content filtered topic is not enabled for the subscription.");
    }
  }

private:
  std::string to_string(
    const rcl_interfaces::msg::ParameterEvent * event) const
  {
    std::ostringstream ss;
    ss << "\nParameter event:";

    auto print_parameters = [&ss](
      const std::string & tip,
      const std::vector<rcl_interfaces::msg::Parameter> & pds
      ) {
        if (!pds.empty()) {
          ss << "\n " << tip << ":";
          for (auto & pd : pds) {
            ss << "\n  " << pd.name << "="
              << rclcpp::to_string(rclcpp::ParameterValue(pd.value));
          }
        }
      };

    print_parameters("new parameters", event->new_parameters);
    print_parameters("changed parameters", event->changed_parameters);
    print_parameters("deleted parameters", event->deleted_parameters);

    ss << "\n";
    return ss.str();
  }

  void topic_callback(const rcl_interfaces::msg::ParameterEvent::ConstSharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "I heard a ParameterEvent for Node '%s', %s",
      msg->node.c_str(), to_string(msg.get()).c_str());
  }

  rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriberWithContentFilteredTopic>());
  rclcpp::shutdown();
  return 0;
}
