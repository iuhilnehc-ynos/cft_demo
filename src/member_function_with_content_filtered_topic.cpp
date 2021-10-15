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

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter_value.hpp"
#include "rcl_interfaces/msg/parameter_event.hpp"
#include "rcl_interfaces/msg/parameter.hpp"


using namespace std::chrono_literals;
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
    // Must use 'space' between '=', (e.g. node=%0 not worked)
    options.content_filter_options.filter_expression = "node = %0";
    std::ostringstream expression_parameter;
    // expression_parameter << "'" << this->get_fully_qualified_name() << "'";
    expression_parameter << this->get_fully_qualified_name();
    options.content_filter_options.expression_parameters = {expression_parameter.str()};

    subscription_ = this->create_subscription<rcl_interfaces::msg::ParameterEvent>(
      "/parameter_events", 10,
      std::bind(&MinimalSubscriberWithContentFilteredTopic::topic_callback, this, _1),
      options);
    if (!subscription_->is_cft_enabled()) {
      RCLCPP_WARN(this->get_logger(),
        "Content filtered topic is not enabled for the subscription.");
    } else {
      print_cft_options();
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

  std::string to_string(
    const std::vector<std::string> & expression_parameters) const
  {
    std::ostringstream oss;
    for(auto &expression_parameter: expression_parameters) {
      oss << "[" << expression_parameter << "]";
    }
    return oss.str();
  }

  void focus_on_parameter_updated(const std::string & param_name) const {
    // To set content filtered topic options
    try {
      // Focus on if the parameter ('test_param') is updated,
      // not clear whether DDS supports `changed_parameters[].name` without a index,
      // test with
      // `ros2 param set /minimal_subscriber_with_content_filtered_topic test_param True`
      std::string filter_expression =
        "node = %0 AND changed_parameters[0].name = %1";
      std::vector<std::string> expression_parameters = {
        this->get_fully_qualified_name(),
        param_name
      };
      RCLCPP_INFO(this->get_logger(),
        "set_cft_expression_parameters filter_expression: [%s]", filter_expression.c_str());
      RCLCPP_INFO(this->get_logger(),
        "set_cft_expression_parameters expression_parameters: [%s]",
          to_string(expression_parameters).c_str());
      subscription_->set_cft_expression_parameters(filter_expression, expression_parameters);
    } catch (const std::exception& e) {
      RCLCPP_WARN(this->get_logger(),
        "Catch an exception: %s", e.what());
    }

    print_cft_options();
    // TODO(iuhilnehc-ynos): remove the warning log after it works.
    RCLCPP_WARN(this->get_logger(),
      "Though set cft expression parameters successfully and get the expected options, "
      "but the behavior is not expected.");
  }

  void topic_callback(const rcl_interfaces::msg::ParameterEvent::ConstSharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "I heard a ParameterEvent for Node '%s', %s",
      msg->node.c_str(), to_string(msg.get()).c_str());

    // check if there is a new parameter named 'test_param', and then set the filter to
    // only focus on if the parameter is updated
    for (auto & new_parameter : msg->new_parameters) {
      if (new_parameter.name == "test_param") {
        focus_on_parameter_updated(new_parameter.name);
      }
    }
  }

  void print_cft_options() const {
    // To get content filtered topic options
    try {
      std::string filter_expression;
      std::vector<std::string> expression_parameters;
      subscription_->get_cft_expression_parameters(filter_expression, expression_parameters);
      RCLCPP_INFO(this->get_logger(),
        "get_cft_expression_parameters filter_expression: [%s]", filter_expression.c_str());
      RCLCPP_INFO(this->get_logger(),
        "get_cft_expression_parameters expression_parameters: [%s]",
          to_string(expression_parameters).c_str());
    } catch (const std::exception& e) {
      RCLCPP_WARN(this->get_logger(),
        "Catch an exception: %s", e.what());
    }
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
