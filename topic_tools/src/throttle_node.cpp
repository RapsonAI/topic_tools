// Copyright 2021 Mateusz Lichota
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

#include <deque>
#include <memory>
#include <optional> // NOLINT : https://github.com/ament/ament_lint/pull/324
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "topic_tools/throttle_node.hpp"

namespace topic_tools
{
  ThrottleNode::ThrottleNode(const rclcpp::NodeOptions& options)
  : ToolBaseNode("throttle", options)
  {
    auto input_topics_ = declare_parameter<std::vector<std::string>>("input_topics");
    auto output_topics_ = declare_parameter<std::vector<std::string>>("output_topics");
    auto lazy_ = declare_parameter<bool>("lazy", false);

    const std::string throttle_type_str = declare_parameter<std::string>("throttle_type");

    // TODO: quick solution for messages type
    auto msg_per_sec_param = declare_parameter<std::vector<double>>("msgs_per_sec");

    use_wall_clock_ = declare_parameter("use_wall_clock", false);

    topic_instances_.clear();

    for (size_t i = 0; i < input_topics_.size(); ++i)
    {
      TopicInstance instance;
      instance.input_topic_ = input_topics_[i];
      instance.output_topic_ = output_topics_[i];
      instance.lazy_ = lazy_;

      if (throttle_type_str == "messages")
      {
        throttle_type_ = ThrottleType::MESSAGES;
        instance.msgs_per_sec_ = msg_per_sec_param[i];
        instance.period_ = rclcpp::Rate(instance.msgs_per_sec_).period();
      }
      else if (throttle_type_str == "bytes")
      {
        throttle_type_ = ThrottleType::BYTES;
        instance.bytes_per_sec_ = declare_parameter<int>("bytes_per_sec");
        instance.window_ = declare_parameter<double>("window");
      }
      else
      {
        RCLCPP_ERROR(get_logger(), "Unknown throttle type");
        return;
      }

      instance.last_time_ = use_wall_clock_ ? rclcpp::Clock{}.now() : this->now();

      topic_instances_.push_back(std::move(instance));
    }

    discovery_timer_ = this->create_wall_timer(
      discovery_period_,
      std::bind(&ThrottleNode::make_subscribe_unsubscribe_decisions, this));

    make_subscribe_unsubscribe_decisions();
  }

  void ThrottleNode::process_message(
    TopicInstance& topic,
    std::shared_ptr<rclcpp::SerializedMessage>msg)
  {
    std::scoped_lock lock(pub_mutex_);

    if (!topic.pub_)
    {
      return;
    }

    const auto& now = use_wall_clock_ ? rclcpp::Clock{}.now() : this->now();

    if (throttle_type_ == ThrottleType::MESSAGES)
    {
      if (topic.last_time_ > now)
      {
        RCLCPP_WARN(
          get_logger(), "Detected jump back in time, resetting throttle period to now for.");
        topic.last_time_ = now;
      }

      if ((now - topic.last_time_).nanoseconds() >= topic.period_.count())
      {
        try
        {
          topic.pub_->publish(*msg);
          topic.last_time_ = now;
        }
        catch (const rclcpp::exceptions::RCLError&)
        {
        }
      }
    }
    else if (throttle_type_ == ThrottleType::BYTES)
    {
      while (!sent_deque_.empty() && sent_deque_.front().first < now.seconds() - topic.window_)
      {
        sent_deque_.pop_front();
      }

      // sum up how many bytes are in the window
      const int64_t bytes = std::accumulate(
        sent_deque_.begin(),
        sent_deque_.end(),
        int64_t{ 0 },
        [](int64_t a, const auto& b) {
          return a + b.second;
        });

      if (bytes < topic.bytes_per_sec_)
      {
        try
        {
          topic.pub_->publish(*msg);
          sent_deque_.emplace_back(now.seconds(), msg->size());
        }
        catch (const rclcpp::exceptions::RCLError&)
        {
        }
      }
    }
  }
} // namespace topic_tools

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(topic_tools::ThrottleNode)
