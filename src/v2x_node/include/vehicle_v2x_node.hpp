#ifndef VEHICLE_V2X_NODE_HPP
#define VEHICLE_V2X_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"
#include <geometry_msgs/msg/point.hpp>
#include <v2x_msg/msg/map.hpp>
#include <v2x_msg/msg/spat.hpp>
#include <v2x_msg/msg/bsm.hpp>
#include <string>
#include <vector>
#include <unordered_map>
#include "autoware_auto_perception_msgs/msg/bounding_box.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "autoware_auto_geometry_msgs/msg/quaternion32.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>

class VehicleV2XNode : public rclcpp::Node {
public:
    VehicleV2XNode();

    // Some helper functions
    void checkClosestLane(double vehicle_lat, double vehicle_lon);
    double haversine(double lat1, double lon1, double lat2, double lon2);
    std::string phaseStateToString(int code);                          // <-- Needed
    
    // V2X Messages Callback
    void mapCallback(const v2x_msg::msg::MAP::SharedPtr msg);
    void spatCallback(const v2x_msg::msg::SPAT::SharedPtr msg);
    void bsmCallback(const v2x_msg::msg::BSM::SharedPtr msg);


private:
    rclcpp::Subscription<v2x_msg::msg::MAP>::SharedPtr map_sub_;
    rclcpp::Subscription<v2x_msg::msg::SPAT>::SharedPtr spat_sub_;     
    rclcpp::Subscription<v2x_msg::msg::BSM>::SharedPtr bsm_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr trafficsignal_pub_;
    rclcpp::Publisher<autoware_auto_perception_msgs::msg::BoundingBox>::SharedPtr bbox_pub_;

    int64_t signal_group_id_ = -1;                                     // <-- Used in SPaT filtering

    struct LaneData {
        int64_t lane_id;
        std::string approach_type;   // Ingress, Egress, etc.
        std::string lane_type;       // Vehicle, Crosswalk, etc.
        std::vector<int64_t> signal_ids;
        std::vector<std::pair<double, double>> node_coords;
    };

    std::vector<LaneData> lanes_;
};

#endif  // VEHICLE_V2X_NODE_HPP
