#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "ximea_ros2_cam/ximea_ros2_cam.hpp"

int main(int argc, char ** argv) {
  setvbuf(stdout, nullptr, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.use_intra_process_comms(true); 

  auto node = std::make_shared<ximea_ros2_cam::XimeaRosCam>(options);

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}