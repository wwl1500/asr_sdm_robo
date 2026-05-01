#include <rclcpp/rclcpp.hpp>

#include "asr_sdm_esdf_map/esdf_map.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("esdf_map");
  SDFMap esdf_map;
  esdf_map.initMap(node);

  RCLCPP_INFO(node->get_logger(), "esdf_map node started");
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
