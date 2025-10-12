/*
 * Copyright 2025 Davide Faconti
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <cloudini_plugin/cloudini_publisher_plugin.hpp>
#include <cloudini_plugin/cloudini_subscriber_plugin.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <point_cloud_transport/publisher_plugin.hpp>
#include <point_cloud_transport/subscriber_plugin.hpp>

PLUGINLIB_EXPORT_CLASS(cloudini_point_cloud_transport::CloudiniPublisher, point_cloud_transport::PublisherPlugin)

PLUGINLIB_EXPORT_CLASS(cloudini_point_cloud_transport::CloudiniSubscriber, point_cloud_transport::SubscriberPlugin)
