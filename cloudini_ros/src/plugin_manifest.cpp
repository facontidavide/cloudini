#include <cloudini_plugin/cloudini_publisher.hpp>
#include <cloudini_plugin/cloudini_subscriber.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <point_cloud_transport/publisher_plugin.hpp>
#include <point_cloud_transport/subscriber_plugin.hpp>

PLUGINLIB_EXPORT_CLASS(cloudini_point_cloud_transport::CloudiniPublisher, point_cloud_transport::PublisherPlugin)

PLUGINLIB_EXPORT_CLASS(cloudini_point_cloud_transport::CloudiniSubscriber, point_cloud_transport::SubscriberPlugin)
