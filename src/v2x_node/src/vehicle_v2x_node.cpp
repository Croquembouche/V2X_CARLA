// vehicle_v2x_node.cpp
#include "vehicle_v2x_node.hpp"
#include <cmath>
#include <limits>

VehicleV2XNode::VehicleV2XNode() : Node("vehicle_v2x_node") {
    map_sub_ = this->create_subscription<v2x_msg::msg::MAP>(
        "/MAP_Inspiration_1743", 10,
        std::bind(&VehicleV2XNode::mapCallback, this, std::placeholders::_1)
    );
    bsm_sub_ = this->create_subscription<v2x_msg::msg::BSM>(
        "/BSM", 10,
        std::bind(&VehicleV2XNode::bsmCallback, this, std::placeholders::_1)
    );
    // spat_subscription_ = this->create_subscription<v2x_msg::msg::SPAT>("/SPAT_TL_04", 10, std::bind(&SPaTListener::spatCallback, this, _1));

    trafficsignal_pub_ = this->create_publisher<std_msgs::msg::String>("/v2x_trafficlightstate", rclcpp::QoS(10));
    bbox_pub_ = this->create_publisher<autoware_auto_perception_msgs::msg::BoundingBox>("/v2x_bsm", rclcpp::QoS(10));


}


// -------------------------------------------------- Helper Functions Start -----------------------------------------------------------------------------------
std::string VehicleV2XNode::phaseStateToString(int code) {
    static const std::unordered_map<int, std::string> phase_map = {
        {0, "unknown"},
        {1, "dark"},
        {2, "stop-Then-Proceed"},
        {3, "stop-And-Remain"},           // Red
        {4, "pre-Movement"},
        {5, "permissive-Movement-Allowed"},
        {6, "protected-Movement-Allowed"},// Green
        {7, "permissive-Clearance"},
        {8, "protected-Clearance"},       // Yellow
        {9, "caution-Conflicting-Traffic"}
    };
    auto it = phase_map.find(code);
    return it != phase_map.end() ? it->second : "invalid";
}

double VehicleV2XNode::haversine(double lat1, double lon1, double lat2, double lon2) {
    constexpr double DEG_TO_RAD = M_PI / 180.0;
    constexpr double EARTH_RADIUS_M = 6371000.0;

    lat1 *= DEG_TO_RAD;
    lon1 *= DEG_TO_RAD;
    lat2 *= DEG_TO_RAD;
    lon2 *= DEG_TO_RAD;

    double dlat = lat2 - lat1;
    double dlon = lon2 - lon1;

    double a = sin(dlat / 2.0) * sin(dlat / 2.0) +
               cos(lat1) * cos(lat2) * sin(dlon / 2.0) * sin(dlon / 2.0);

    return EARTH_RADIUS_M * 2 * atan2(sqrt(a), sqrt(1 - a));
}

void VehicleV2XNode::checkClosestLane(double vehicle_lat, double vehicle_lon) {
    double min_dist = std::numeric_limits<double>::max();
    int64_t closest_lane_id = -1;
    std::vector<int64_t> closest_signal_ids;

    for (const auto& lane : lanes_) {
        for (const auto& pt : lane.node_coords) {
            double dist = haversine(vehicle_lat, vehicle_lon, pt.first, pt.second);
            if (dist < min_dist) {
                min_dist = dist;
                closest_lane_id = lane.lane_id;
                closest_signal_ids = lane.signal_ids;
            }
        }
    }

    // RCLCPP_INFO(this->get_logger(), "Closest lane ID: %ld", closest_lane_id);
    // RCLCPP_INFO(this->get_logger(), "Distance: %.2f meters", min_dist);
    std::set<int64_t> unique_signal_ids(closest_signal_ids.begin(), closest_signal_ids.end());

    std::ostringstream signal_info;
    for (auto id : unique_signal_ids) {
        signal_info << id << " ";
    }

    // RCLCPP_INFO(this->get_logger(), "Controlling signal group(s): %s", signal_info.str().c_str());
    signal_group_id_ = *unique_signal_ids.begin();  // assuming unique_signal_ids is non-empty

    std::string topic_name = std::string("/SPAT_TL_") + (signal_group_id_ < 10 ? "0" : "") + std::to_string(signal_group_id_);
    // RCLCPP_INFO(this->get_logger(), "Controlling signal group(s): %s", topic_name.c_str());
    spat_sub_ = this->create_subscription<v2x_msg::msg::SPAT>(topic_name, 10, std::bind(&VehicleV2XNode::spatCallback, this, std::placeholders::_1));
}

// -------------------------------------------------- Helper Functions End -----------------------------------------------------------------------------------

// -------------------------------------------------- V2X Functions Start -----------------------------------------------------------------------------------

void VehicleV2XNode::spatCallback(const v2x_msg::msg::SPAT::SharedPtr msg) {
    for (const auto &intersection : msg->intersections) {
        for (const auto &state : intersection.states) {
            if (state.signalgroupid == signal_group_id_) {
                if (!state.statetimespeed.empty()) {
                    int phase_code = state.statetimespeed[0].movementphasestate;
                    std::string readable_state = phaseStateToString(phase_code);
                    RCLCPP_INFO(this->get_logger(),
                                "Signal Group %d: %s (code = %d)",
                                signal_group_id_, readable_state.c_str(), phase_code);
                    auto message = std_msgs::msg::String();
                    message.data = std::to_string(phase_code);
                    trafficsignal_pub_->publish(message);
                    return;
                }
            }
        }
    }
    RCLCPP_WARN(this->get_logger(), "Signal Group %d not found in latest SPaT", signal_group_id_);
}

void VehicleV2XNode::mapCallback(const v2x_msg::msg::MAP::SharedPtr msg) {
    lanes_.clear();
    // RCLCPP_INFO(this->get_logger(), "Received MAP Message");

    for (const auto& intersection : msg->intersections) {
        for (const auto& lane : intersection.laneset) {
            LaneData lane_data;
            lane_data.lane_id = lane.laneid;

            for (const auto& conn : lane.connectsto) {
                if (conn.connectinglane.laneid != 0) {
                    lane_data.signal_ids.push_back(conn.signalgroup);
                }
            }

            for (const auto& node_list : lane.nodelist) {
                for (const auto& node : node_list.nodes) {
                    if (node.delta.nodelatlon.longitude != 0 && node.delta.nodelatlon.latitude != 0) {
                        lane_data.node_coords.emplace_back(
                            node.delta.nodelatlon.latitude / 1e7,
                            node.delta.nodelatlon.longitude / 1e7
                        );
                        // RCLCPP_INFO(this->get_logger(), "Parsed node: lat = %.7f, lon = %.7f", node.delta.nodelatlon.latitude / 1e7, node.delta.nodelatlon.longitude / 1e7);

                        
                    }
                }
            }

            if (!lane_data.node_coords.empty()) {
                lanes_.push_back(lane_data);
            }
        }
    }
}

void VehicleV2XNode::bsmCallback(const v2x_msg::msg::BSM::SharedPtr msg){
    autoware_auto_perception_msgs::msg::BoundingBox box;

    // Convert lat/lon/elev to centroid (optionally convert to meters)
    box.centroid.x = msg->coredata.longitude / 1e7;  // WARNING: still in degrees, not meters
    box.centroid.y = msg->coredata.lat / 1e7;
    box.centroid.z = msg->coredata.elev / 10.0;

    // Set size (convert cm to meters)
    box.size.x = msg->coredata.size.length / 100.0;
    box.size.y = msg->coredata.size.width / 100.0;
    box.size.z = 1.5;  // Estimated height, not available in BSM

    // Convert heading (0-28800 = 0-360 degrees) to quaternion
    float heading_deg = msg->coredata.heading * 0.0125f;
    float heading_rad = heading_deg * M_PI / 180.0f;

    tf2::Quaternion q;
    q.setRPY(0, 0, heading_rad);
    box.orientation.x = q.x();
    box.orientation.y = q.y();
    box.orientation.z = q.z();
    box.orientation.w = q.w();

    // Speed (0.02 m/s per unit)
    box.velocity = msg->coredata.speed * 0.02f;

    // Heading (degrees)
    box.heading = heading_deg;

    // No yaw_rate in BSM, so set heading_rate = 0
    box.heading_rate = 0.0;

    // Corners (optional): leave default (zeros)
    for (int i = 0; i < 4; ++i) {
        box.corners[i].x = 0.0f;
        box.corners[i].y = 0.0f;
        box.corners[i].z = 0.0f;
    }

    // Variance
    for (int i = 0; i < 8; ++i) {
        box.variance[i] = 0.0f;
    }

    // Value
    box.value = 1.0f;

    // Vehicle label — assume CAR unless extended info available
    box.vehicle_label = autoware_auto_perception_msgs::msg::BoundingBox::CAR;

    // Signal label — if brake is active (non-zero), use BRAKE
    if (msg->coredata.brakes.brakeboost != 0 ||
        msg->coredata.brakes.auxbrakes != 0) {
        box.signal_label = autoware_auto_perception_msgs::msg::BoundingBox::BRAKE;
    } else {
        box.signal_label = autoware_auto_perception_msgs::msg::BoundingBox::NO_SIGNAL;
    }

    // Class likelihood — default to 1.0
    box.class_likelihood = 1.0f;

    bbox_pub_->publish(box);
}

// -------------------------------------------------- V2X Functions End -----------------------------------------------------------------------------------
