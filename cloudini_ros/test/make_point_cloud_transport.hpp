// Copyright 2025 Davide Faconti
// Licensed under the Apache License, Version 2.0

#pragma once

#include <memory>
#include <point_cloud_transport/point_cloud_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <type_traits>

// point_cloud_transport 6 (Rolling) builds PointCloudTransport from rclcpp::NodeInterfaces, which takes a
// Node&; Humble, Jazzy and Kilted only have the constructor that takes a Node::SharedPtr.
// A template, so that the branch the installed version lacks is never compiled.
template <typename PCT = point_cloud_transport::PointCloudTransport>
std::shared_ptr<PCT> makePointCloudTransport(const rclcpp::Node::SharedPtr& node) {
  if constexpr (std::is_constructible_v<PCT, rclcpp::Node&>) {
    return std::make_shared<PCT>(*node);
  } else {
    return std::make_shared<PCT>(node);
  }
}
