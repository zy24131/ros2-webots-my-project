// 从节点参数构建 rclcpp::QoS，供各 pub/sub 统一配置。
//
// 参数前缀示例：qos.steering_input_sub
//   profile: default | sensor_data | reliable | best_effort | system_default
//   depth: 队列深度（默认 10）
//   reliability: 空=沿用 profile；best_effort | reliable
//   durability: 空=沿用 profile；volatile | transient_local
//   history: 空=沿用 profile；keep_last | keep_all

#pragma once

#include <cstdint>
#include <string>

#include "rclcpp/rclcpp.hpp"

namespace fsm {

inline rclcpp::QoS BaselineQoS(const std::string & profile, size_t depth)
{
  if (profile == "sensor_data") {
    return rclcpp::SensorDataQoS().keep_last(depth);
  }
  if (profile == "services_default") {
    return rclcpp::ServicesQoS();
  }
  if (profile == "parameters") {
    return rclcpp::ParametersQoS();
  }
  if (profile == "system_default") {
    return rclcpp::SystemDefaultsQoS().keep_last(depth);
  }
  if (profile == "best_effort") {
    return rclcpp::QoS(depth).best_effort();
  }
  if (profile == "reliable") {
    return rclcpp::QoS(depth).reliable();
  }
  return rclcpp::QoS(depth);
}

inline void DeclareQoSParams(
  rclcpp::Node * node,
  const std::string & prefix,
  const std::string & default_profile,
  size_t default_depth)
{
  if (!node->has_parameter(prefix + ".profile")) {
    node->declare_parameter(prefix + ".profile", default_profile);
  }
  if (!node->has_parameter(prefix + ".depth")) {
    node->declare_parameter(prefix + ".depth", static_cast<int64_t>(default_depth));
  }
  if (!node->has_parameter(prefix + ".reliability")) {
    node->declare_parameter(prefix + ".reliability", std::string(""));
  }
  if (!node->has_parameter(prefix + ".durability")) {
    node->declare_parameter(prefix + ".durability", std::string(""));
  }
  if (!node->has_parameter(prefix + ".history")) {
    node->declare_parameter(prefix + ".history", std::string(""));
  }
}

inline rclcpp::QoS QoSFromParams(
  rclcpp::Node * node,
  const std::string & prefix,
  const std::string & default_profile,
  size_t default_depth = 10)
{
  DeclareQoSParams(node, prefix, default_profile, default_depth);

  const std::string profile = node->get_parameter(prefix + ".profile").as_string();
  int64_t depth = node->get_parameter(prefix + ".depth").as_int();
  if (depth < 1) {
    depth = 1;
  }

  rclcpp::QoS qos = BaselineQoS(profile, static_cast<size_t>(depth));

  const std::string reliability = node->get_parameter(prefix + ".reliability").as_string();
  if (reliability == "best_effort") {
    qos.best_effort();
  } else if (reliability == "reliable") {
    qos.reliable();
  }

  const std::string durability = node->get_parameter(prefix + ".durability").as_string();
  if (durability == "volatile") {
    qos.durability_volatile();
  } else if (durability == "transient_local") {
    qos.transient_local();
  }

  const std::string history = node->get_parameter(prefix + ".history").as_string();
  if (history == "keep_all") {
    qos.keep_all();
  } else if (history == "keep_last") {
    qos.keep_last(static_cast<size_t>(depth));
  }

  return qos;
}

}  // namespace fsm
