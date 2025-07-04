// vehicle_node.cpp
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "vehicle_v2x_node.hpp"

class VehicleNode : public rclcpp::Node {
public:
    VehicleNode() : Node("vehicle_node") {
        // Create the VehicleV2XNode instance (assumes already spinning elsewhere)
        vehicle_v2x_node_ = std::make_shared<VehicleV2XNode>();

        // Subscribe to GNSS sensor topic
        gnss_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/sensing/gnss/nav_sat_fix", 10,
            std::bind(&VehicleNode::gnssCallback, this, std::placeholders::_1)
        );
    }

    std::shared_ptr<VehicleV2XNode> getV2XNode() const {
        return vehicle_v2x_node_;
    }

private:
    void gnssCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        double lat = msg->latitude;
        double lon = msg->longitude;
        vehicle_v2x_node_->checkClosestLane(lat, lon);
    }
    

    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_sub_;
    std::shared_ptr<VehicleV2XNode> vehicle_v2x_node_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto vehicle_node = std::make_shared<VehicleNode>();
    auto v2x_node = vehicle_node->getV2XNode();  // expose the internal node

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(vehicle_node);
    executor.add_node(v2x_node);  // ensures mapCallback is active too

    executor.spin();
    rclcpp::shutdown();
    return 0;
}