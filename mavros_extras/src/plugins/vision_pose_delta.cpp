#include <string>

#include "dvl_msgs/msg/dvl_odom_with_confidence.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "mavros/mavros_uas.hpp"
#include "mavros/plugin.hpp"
#include "mavros/plugin_filter.hpp"
#include "mavros/utils.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rcpputils/asserts.hpp"
#include "tf2_eigen/tf2_eigen.hpp"

namespace mavros {
namespace extra_plugins {
using namespace std::placeholders;

/**
 * @brief Vision position delta plugin
 * @plugin vision_position_delta
 *
 * Send position delta from various vision/odometry estimators
 * to FCU position estimators using VISION_POSITION_DELTA messages.
 *
 */
class VisionPositionDeltaPlugin : public plugin::Plugin {
   public:
    explicit VisionPositionDeltaPlugin(plugin::UASPtr uas_)
        : Plugin(uas_, "vision_position_delta") {
        enable_node_watch_parameters();
        RCLCPP_INFO(get_logger(), "VisionPositionDeltaPlugin Node initialized");

        odom_delta_sub = node->create_subscription<dvl_msgs::msg::DVLOdomWithConfidence>(
            "/dvl/confidence_odom", 10, std::bind(&VisionPositionDeltaPlugin::odom_delta_cb, this, _1));

        // Parameters for time delta handling
        node_declare_and_watch_parameter(
            "time_delta_usec", 0, [&](const rclcpp::Parameter& p) {
                time_delta_usec = p.as_int();
            });
    }

    Subscriptions get_subscriptions() override {
        return {/* Rx disabled */};
    }

   private:
    rclcpp::Subscription<dvl_msgs::msg::DVLOdomWithConfidence>::SharedPtr odom_delta_sub;

    int64_t time_delta_usec;
    rclcpp::Time last_delta_stamp{0, 0, RCL_ROS_TIME};

    /* -*- low-level send -*- */
    /**
     * @brief Send vision position delta to FCU
     */
    void send_vision_position_delta(
        const rclcpp::Time& stamp,
        const Eigen::Vector3d& position_delta,
        const Eigen::Vector3d& angle_delta,
        const double confidence,
        // const geometry_msgs::msg::PoseWithCovariance::_covariance_type& cov,
        uint64_t time_delta_us = 0) {
        if (last_delta_stamp == stamp) {
            RCLCPP_DEBUG_THROTTLE(
                get_logger(),
                *get_clock(), 10, "Vision Delta: Same delta as last one, dropped.");
            return;
        }
        last_delta_stamp = stamp;

        // Transform deltas to NED frame (MAVLink standard)
        auto position_ned = position_delta;
        auto angle_ned = angle_delta;
        
        // //Covariance is not used for now
        // // Transform covariance to NED
        // auto cov_ned = cov;
        // ftf::EigenMapConstCovariance6d cov_map(cov_ned.data());

        mavlink::ardupilotmega::msg::VISION_POSITION_DELTA vd{};

        vd.time_usec = stamp.nanoseconds() / 1000;
        vd.time_delta_usec = time_delta_us > 0 ? time_delta_us : time_delta_usec;

        // Position deltas
        vd.position_delta[0] = position_ned.x();
        vd.position_delta[1] = position_ned.y();
        vd.position_delta[2] = position_ned.z();

        // Angle deltas (roll, pitch, yaw)
        vd.angle_delta[0] = angle_ned.x();
        vd.angle_delta[1] = angle_ned.y();
        vd.angle_delta[2] = angle_ned.z();
        vd.confidence = confidence;
        uas->send_message(vd);
    }

    void odom_delta_cb(const dvl_msgs::msg::DVLOdomWithConfidence::SharedPtr req) {
        // Extract position delta
        Eigen::Vector3d pos_delta(
            req->odom.pose.pose.position.x,
            req->odom.pose.pose.position.y,
            req->odom.pose.pose.position.z);

        // Extract angle delta from quaternion
        auto q = req->odom.pose.pose.orientation;
        Eigen::Vector3d angle_delta = ftf::quaternion_to_rpy(
            Eigen::Quaterniond(q.w, q.x, q.y, q.z));
        double confidence = req->confidence;

        send_vision_position_delta(req->odom.header.stamp, pos_delta, angle_delta, confidence);
    }
};
}  // namespace extra_plugins
}  // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>  // NOLINT
MAVROS_PLUGIN_REGISTER(mavros::extra_plugins::VisionPositionDeltaPlugin)