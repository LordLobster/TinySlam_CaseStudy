// ROS 2 wrapper around the tinySLAM (CoreSLAM) core.
//
// Pipeline per accepted scan:
//   1. odom->base_footprint delta since the last scan, applied to the SLAM pose (motion prior)
//   2. ts_monte_carlo_search() matches the scan against the map around that prior
//   3. ts_map_update() integrates the scan at the corrected pose
//   4. map->odom is published so that map->base_footprint equals the SLAM pose
//
// tinySLAM conventions: positions in millimetres, angles in degrees,
// map pixels at TS_MAP_SCALE pixels/mm, TS_NO_OBSTACLE = free, TS_OBSTACLE = occupied.

#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

#include "CoreSLAM.h"

namespace
{
constexpr double kMmPerM = 1000.0;
constexpr double kMapPixelM = 1.0 / (TS_MAP_SCALE * kMmPerM);       // metres per map pixel
constexpr double kMapHalfMm = TS_MAP_SIZE / 2.0 / TS_MAP_SCALE;     // map centre in mm
constexpr ts_map_pixel_t kUnknown = (TS_OBSTACLE + TS_NO_OBSTACLE) / 2;

double rad2deg(double r) { return r * 180.0 / M_PI; }
double deg2rad(double d) { return d * M_PI / 180.0; }

tf2::Transform make_tf(double x, double y, double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  return tf2::Transform(q, tf2::Vector3(x, y, 0));
}
}  // namespace

class TinySlamNode : public rclcpp::Node
{
public:
  TinySlamNode()
  : Node("tinyslam")
  {
    map_frame_ = declare_parameter("map_frame", "map");
    odom_frame_ = declare_parameter("odom_frame", "odom");
    base_frame_ = declare_parameter("base_frame", "base_footprint");
    sigma_xy_mm_ = declare_parameter("sigma_xy_mm", 100.0);
    sigma_theta_deg_ = declare_parameter("sigma_theta_deg", 20.0);
    search_iterations_ = declare_parameter("search_iterations", 1000);
    hole_width_mm_ = declare_parameter("hole_width_mm", 600);
    map_quality_ = declare_parameter("map_quality", 50);
    scan_span_ = declare_parameter("scan_span", 3);
    min_scan_interval_ = declare_parameter("min_scan_interval", 0.1);
    min_travel_m_ = declare_parameter("min_travel_m", 0.0);
    min_rotation_rad_ = declare_parameter("min_rotation_rad", 0.0);
    max_range_m_ = declare_parameter("max_range_m", 0.0);  // 0 -> use scan range_max
    map_downsample_ = std::max<int64_t>(1, declare_parameter("map_downsample", 2));
    map_publish_period_ = declare_parameter("map_publish_period", 1.0);
    transform_tolerance_ = declare_parameter("transform_tolerance", 0.1);
    // Differential-drive odometry tracks the wheel-axle midpoint but publishes it as
    // base_frame. If the axle is not at base_frame's origin, odometry deltas are
    // re-expressed about the real base origin using the wheel frames below.
    left_wheel_frame_ = declare_parameter("left_wheel_frame", "");
    right_wheel_frame_ = declare_parameter("right_wheel_frame", "");

    map_ = std::make_unique<ts_map_t>();
    ts_map_init(map_.get());
    ts_random_init(&randomizer_, 0xdead);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      "map", rclcpp::QoS(1).transient_local().reliable());
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("tinyslam/pose", 10);
    path_pub_ = create_publisher<nav_msgs::msg::Path>("tinyslam/path", 10);

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", rclcpp::SensorDataQoS(),
      std::bind(&TinySlamNode::on_scan, this, std::placeholders::_1));

    save_srv_ = create_service<std_srvs::srv::Trigger>(
      "tinyslam/save_pgm",
      std::bind(&TinySlamNode::on_save, this, std::placeholders::_1, std::placeholders::_2));

    map_timer_ = create_wall_timer(
      std::chrono::duration<double>(map_publish_period_), [this]() {publish_map();});
    tf_timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() {publish_map_to_odom();});

    path_.header.frame_id = map_frame_;
    RCLCPP_INFO(
      get_logger(), "tinySLAM: map %dx%d px @ %.3f m/px (%.1f m square), downsampled x%ld for /map",
      TS_MAP_SIZE, TS_MAP_SIZE, kMapPixelM, TS_MAP_SIZE * kMapPixelM, map_downsample_);
  }

private:
  void on_scan(const sensor_msgs::msg::LaserScan::SharedPtr scan)
  {
    const rclcpp::Time stamp(scan->header.stamp);
    if (initialised_ && (stamp - last_update_stamp_).seconds() < min_scan_interval_) {
      return;
    }

    // Fixed transform base -> laser (looked up once)
    if (!have_laser_tf_) {
      try {
        auto t = tf_buffer_->lookupTransform(base_frame_, scan->header.frame_id, tf2::TimePointZero);
        tf2::Transform base_to_laser;
        tf2::fromMsg(t.transform, base_to_laser);
        // tinySLAM is planar: keep only x, y, yaw of the mount (drop the lidar height)
        base_to_laser_ = make_tf(
          base_to_laser.getOrigin().x(), base_to_laser.getOrigin().y(),
          tf2::getYaw(base_to_laser.getRotation()));
        if (!left_wheel_frame_.empty() && !right_wheel_frame_.empty()) {
          tf2::Transform l, r;
          tf2::fromMsg(
            tf_buffer_->lookupTransform(base_frame_, left_wheel_frame_, tf2::TimePointZero).transform, l);
          tf2::fromMsg(
            tf_buffer_->lookupTransform(base_frame_, right_wheel_frame_, tf2::TimePointZero).transform, r);
          const tf2::Vector3 mid = (l.getOrigin() + r.getOrigin()) / 2.0;
          base_to_axle_ = make_tf(mid.x(), mid.y(), 0.0);
          RCLCPP_INFO(
            get_logger(), "Wheel axle centre at (%.3f, %.3f) in '%s'; correcting odometry deltas",
            mid.x(), mid.y(), base_frame_.c_str());
        }
        have_laser_tf_ = true;  // only once every fixed transform above was found
        RCLCPP_INFO(
          get_logger(), "Laser '%s' at (%.3f, %.3f) yaw %.1f deg in '%s'",
          scan->header.frame_id.c_str(), base_to_laser_.getOrigin().x(),
          base_to_laser_.getOrigin().y(), rad2deg(tf2::getYaw(base_to_laser_.getRotation())),
          base_frame_.c_str());
      } catch (const tf2::TransformException & e) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting for %s -> %s: %s",
          base_frame_.c_str(), scan->header.frame_id.c_str(), e.what());
        return;
      }
    }

    // Odometry pose of the base at the scan time
    tf2::Transform odom_to_base;
    try {
      auto t = tf_buffer_->lookupTransform(
        odom_frame_, base_frame_, stamp, rclcpp::Duration::from_seconds(0.1));
      tf2::fromMsg(t.transform, odom_to_base);
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "No odometry %s -> %s: %s",
        odom_frame_.c_str(), base_frame_.c_str(), e.what());
      return;
    }

    if (initialised_) {
      const tf2::Transform delta = last_odom_to_base_.inverse() * odom_to_base;
      const double travel = delta.getOrigin().length();
      const double rotation = std::fabs(tf2::getYaw(delta.getRotation()));
      if (travel < min_travel_m_ && rotation < min_rotation_rad_) {
        return;
      }
    }

    build_scan(*scan);
    if (scan_.nb_points == 0) {
      return;
    }

    // Planar odometry pose (x, y, yaw)
    odom_to_base = make_tf(
      odom_to_base.getOrigin().x(), odom_to_base.getOrigin().y(),
      tf2::getYaw(odom_to_base.getRotation()));

    if (!initialised_) {
      // Start the map frame on top of the odom frame
      map_to_base_ = odom_to_base;
    } else {
      // Motion prior from odometry (odometry actually moves the axle frame: conjugate into base)
      const tf2::Transform axle_delta = last_odom_to_base_.inverse() * odom_to_base;
      map_to_base_ = map_to_base_ * (base_to_axle_ * axle_delta * base_to_axle_.inverse());
    }

    // tinySLAM tracks the laser pose; rays are then cast from the sensor itself
    ts_position_t laser_pos = to_ts(map_to_base_ * base_to_laser_);
    if (initialised_) {
      int best = 0;
      laser_pos = ts_monte_carlo_search(
        &randomizer_, &scan_, map_.get(), &laser_pos, sigma_xy_mm_, sigma_theta_deg_,
        static_cast<int>(search_iterations_), &best);
      map_to_base_ = from_ts(laser_pos) * base_to_laser_.inverse();
    }

    ts_map_update(&scan_map_, map_.get(), &laser_pos, static_cast<int>(map_quality_),
      static_cast<int>(hole_width_mm_));

    {
      std::lock_guard<std::mutex> lock(tf_mutex_);
      map_to_odom_ = map_to_base_ * odom_to_base.inverse();
      last_scan_stamp_ = stamp;
      have_map_to_odom_ = true;
    }
    last_odom_to_base_ = odom_to_base;
    last_update_stamp_ = stamp;
    initialised_ = true;

    publish_pose(stamp);
    publish_map_to_odom();
  }

  // Convert a LaserScan into the two tinySLAM scans (laser frame, mm):
  // scan_ (1 point per beam) for matching, scan_map_ (scan_span points per beam) for map update.
  void build_scan(const sensor_msgs::msg::LaserScan & msg)
  {
    const double max_range = (max_range_m_ > 0.0) ? std::min<double>(max_range_m_, msg.range_max) :
      msg.range_max;
    const int hole_half = static_cast<int>(hole_width_mm_) / 2;
    const int span = std::max<int64_t>(1, scan_span_);

    scan_.nb_points = 0;
    scan_map_.nb_points = 0;
    for (size_t i = 0; i < msg.ranges.size(); ++i) {
      const float r = msg.ranges[i];
      const bool no_return = !std::isfinite(r) || r >= max_range;
      if (!no_return && (r < msg.range_min || r * kMmPerM <= hole_half)) {
        continue;  // too close / invalid
      }
      const double d_mm = (no_return ? max_range : r) * kMmPerM;
      const int value = no_return ? TS_NO_OBSTACLE : TS_OBSTACLE;
      const double beam = msg.angle_min + msg.angle_increment * i;
      add_point(scan_, d_mm * std::cos(beam), d_mm * std::sin(beam), value);
      for (int j = 0; j < span; ++j) {
        // Sub-rays centred on the beam: a one-sided spread would rotate the map
        // relative to the matching scan and make the heading drift
        const double a = beam + msg.angle_increment * ((j + 0.5) / span - 0.5);
        add_point(scan_map_, d_mm * std::cos(a), d_mm * std::sin(a), value);
      }
    }
  }

  static void add_point(ts_scan_t & s, double x, double y, int value)
  {
    if (s.nb_points >= TS_SCAN_SIZE) {
      return;
    }
    s.x[s.nb_points] = x;
    s.y[s.nb_points] = y;
    s.value[s.nb_points] = value;
    s.nb_points++;
  }

  // ROS map-frame transform <-> tinySLAM pose (mm/deg, origin at the map corner)
  static ts_position_t to_ts(const tf2::Transform & t)
  {
    ts_position_t p;
    p.x = t.getOrigin().x() * kMmPerM + kMapHalfMm;
    p.y = t.getOrigin().y() * kMmPerM + kMapHalfMm;
    p.theta = rad2deg(tf2::getYaw(t.getRotation()));
    return p;
  }

  static tf2::Transform from_ts(const ts_position_t & p)
  {
    return make_tf((p.x - kMapHalfMm) / kMmPerM, (p.y - kMapHalfMm) / kMmPerM, deg2rad(p.theta));
  }

  void publish_map_to_odom()
  {
    std::lock_guard<std::mutex> lock(tf_mutex_);
    if (!have_map_to_odom_) {
      return;
    }
    geometry_msgs::msg::TransformStamped t;
    // Future-date slightly so consumers can transform recent data
    t.header.stamp = last_scan_stamp_ + rclcpp::Duration::from_seconds(transform_tolerance_);
    if (rclcpp::Time(t.header.stamp) < now()) {
      t.header.stamp = now();
    }
    t.header.frame_id = map_frame_;
    t.child_frame_id = odom_frame_;
    t.transform = tf2::toMsg(map_to_odom_);
    tf_broadcaster_->sendTransform(t);
  }

  void publish_pose(const rclcpp::Time & stamp)
  {
    geometry_msgs::msg::PoseStamped p;
    p.header.stamp = stamp;
    p.header.frame_id = map_frame_;
    tf2::toMsg(map_to_base_, p.pose);
    pose_pub_->publish(p);

    // Grow the path only after real motion so it stays small over long runs
    if (!path_.poses.empty()) {
      const auto & last = path_.poses.back().pose;
      const double moved = std::hypot(
        p.pose.position.x - last.position.x, p.pose.position.y - last.position.y);
      const double turned = std::fabs(
        tf2::getYaw(p.pose.orientation) - tf2::getYaw(last.orientation));
      if (moved < 0.05 && std::fabs(std::remainder(turned, 2 * M_PI)) < deg2rad(5.0)) {
        return;
      }
    }
    path_.header.stamp = stamp;
    path_.poses.push_back(p);
    path_pub_->publish(path_);
  }

  void publish_map()
  {
    if (!initialised_) {
      return;
    }
    const int k = static_cast<int>(map_downsample_);
    const int w = TS_MAP_SIZE / k;

    nav_msgs::msg::OccupancyGrid grid;
    grid.header.stamp = now();
    grid.header.frame_id = map_frame_;
    grid.info.resolution = static_cast<float>(kMapPixelM * k);
    grid.info.width = w;
    grid.info.height = w;
    grid.info.origin.position.x = -kMapHalfMm / kMmPerM;
    grid.info.origin.position.y = -kMapHalfMm / kMmPerM;
    grid.info.origin.orientation.w = 1.0;
    grid.data.assign(static_cast<size_t>(w) * w, -1);

    // Average each k x k block of touched pixels; untouched blocks stay unknown (-1).
    // tinySLAM: 0 = obstacle, 65500 = free  ->  ROS: 100 = occupied, 0 = free
    for (int by = 0; by < w; ++by) {
      for (int bx = 0; bx < w; ++bx) {
        long sum = 0;
        int n = 0;
        for (int y = by * k; y < (by + 1) * k; ++y) {
          const ts_map_pixel_t * row = &map_->map[y * TS_MAP_SIZE];
          for (int x = bx * k; x < (bx + 1) * k; ++x) {
            if (row[x] != kUnknown) {
              sum += row[x];
              ++n;
            }
          }
        }
        if (n) {
          const double free_ratio = static_cast<double>(sum) / n / TS_NO_OBSTACLE;
          grid.data[by * w + bx] = static_cast<int8_t>(std::lround(100.0 * (1.0 - free_ratio)));
        }
      }
    }
    map_pub_->publish(grid);
  }

  void on_save(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    // tinySLAM's own PGM export; overlay with a copy of the map draws nothing extra
    auto overlay = std::make_unique<ts_map_t>();
    ts_map_init(overlay.get());
    std::string filename = "tinyslam_map.pgm";
    ts_save_map_pgm(map_.get(), overlay.get(), filename.data(), TS_MAP_SIZE, TS_MAP_SIZE);
    res->success = true;
    res->message = "Saved " + filename + " (in the node's working directory)";
  }

  // Parameters
  std::string map_frame_, odom_frame_, base_frame_;
  double sigma_xy_mm_, sigma_theta_deg_;
  int64_t search_iterations_, hole_width_mm_, map_quality_, scan_span_, map_downsample_;
  double min_scan_interval_, min_travel_m_, min_rotation_rad_, max_range_m_;
  double map_publish_period_, transform_tolerance_;
  std::string left_wheel_frame_, right_wheel_frame_;

  // tinySLAM state
  std::unique_ptr<ts_map_t> map_;   // 8 MB: keep off the stack
  ts_randomizer_t randomizer_;
  ts_scan_t scan_{}, scan_map_{};

  // Poses
  bool initialised_ = false;
  bool have_laser_tf_ = false;
  tf2::Transform base_to_laser_, map_to_base_, last_odom_to_base_;
  tf2::Transform base_to_axle_ = tf2::Transform::getIdentity();
  rclcpp::Time last_update_stamp_;

  std::mutex tf_mutex_;
  bool have_map_to_odom_ = false;
  tf2::Transform map_to_odom_;
  rclcpp::Time last_scan_stamp_;

  nav_msgs::msg::Path path_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_srv_;
  rclcpp::TimerBase::SharedPtr map_timer_, tf_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TinySlamNode>());
  rclcpp::shutdown();
  return 0;
}
