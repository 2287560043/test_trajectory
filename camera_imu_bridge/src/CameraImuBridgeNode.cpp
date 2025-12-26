#include <rclcpp/rclcpp.hpp>
namespace helios_cv {
class CameraImuBridgeNode: public rclcpp::Node {
public:
    explicit CameraImuBridgeNode(const rclcpp::NodeOptions& options):
        Node("camera_imu_bridge_node", options) {
    }

private:
};
} // namespace helios_cv
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(helios_cv::CameraImuBridgeNode)
