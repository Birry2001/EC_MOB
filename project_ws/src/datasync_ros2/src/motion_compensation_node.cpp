#include <rclcpp/rclcpp.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <dvs_msgs/msg/event_array.hpp>

#include <cmath>
#include <mutex>
#include <vector>
#include <algorithm>

using sll = long long;

class MotionCompensationNode : public rclcpp::Node
{
public:
  MotionCompensationNode() : Node("motion_compensation_node")
  {
    // Parameters (same names as ROS1 for drop-in launch migration)
    width_ = this->declare_parameter<int>("weight_param", 346);
    height_ = this->declare_parameter<int>("height_param", 260);
    focus_ = this->declare_parameter<double>("focus", 6550.0);
    pixel_size_ = this->declare_parameter<double>("pixel_size", 18.5);

    events_topic_ = this->declare_parameter<std::string>("events_topic", "/dvs/events");
    imu_topic_ = this->declare_parameter<std::string>("imu_topic", "/dvs/imu");
    output_topic_ = this->declare_parameter<std::string>("count_image_topic", "/count_image");

    // QoS: event cameras are high-rate -> best effort often makes sense.
    auto qos_events = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    auto qos_imu = rclcpp::QoS(rclcpp::KeepLast(50)).best_effort().durability_volatile();

    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(output_topic_, rclcpp::QoS(1));

    event_sub_ = this->create_subscription<dvs_msgs::msg::EventArray>(
      events_topic_, qos_events,
      std::bind(&MotionCompensationNode::event_cb, this, std::placeholders::_1));

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, qos_imu,
      std::bind(&MotionCompensationNode::imu_cb, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "MotionCompensationNode started. Subscribing to %s and %s",
                events_topic_.c_str(), imu_topic_.c_str());
  }

private:
  // Parameters
  int height_{260};
  int width_{346};
  double focus_{6550.0};
  double pixel_size_{18.5};
  std::string events_topic_{"/dvs/events"};
  std::string imu_topic_{"/dvs/imu"};
  std::string output_topic_{"/count_image"};

  // ROS interfaces
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Subscription<dvs_msgs::msg::EventArray>::SharedPtr event_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  // Buffers
  std::vector<dvs_msgs::msg::Event> event_buffer_;
  std::vector<sensor_msgs::msg::Imu> imu_buffer_;
  std::mutex mtx_;
  bool first_event_received_{true};

  static inline int64_t to_ns(const builtin_interfaces::msg::Time & t)
  {
    return static_cast<int64_t>(t.sec) * 1000000000LL + static_cast<int64_t>(t.nanosec);
  }

  void imu_cb(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    if (!first_event_received_) {
      std::lock_guard<std::mutex> lock(mtx_);
      imu_buffer_.push_back(*msg);
    }
  }

  void event_cb(const dvs_msgs::msg::EventArray::SharedPtr msg)
  {
    if (first_event_received_) {
      // Discard first batch to finish data matching (kept from ROS1 behaviour)
      first_event_received_ = false;
      {
        std::lock_guard<std::mutex> lock(mtx_);
        imu_buffer_.clear();
      }
      RCLCPP_INFO(this->get_logger(), "Data aligned! Start processing data...");
      return;
    }

    // Snapshot IMU buffer then clear it
    std::vector<sensor_msgs::msg::Imu> imu_snapshot;
    {
      std::lock_guard<std::mutex> lock(mtx_);
      imu_snapshot = imu_buffer_;
      imu_buffer_.clear();
    }

    if (imu_snapshot.empty() || msg->events.empty()) {
      return;
    }

    // Append events
    event_buffer_.reserve(event_buffer_.size() + msg->events.size());
    for (const auto & e : msg->events) {
      event_buffer_.push_back(e);
    }

    data_process(event_buffer_, imu_snapshot);

    // Clear event buffer after processing
    event_buffer_.clear();
  }

  void publish_count_image(const std::vector<std::vector<int>> & count_image, int max_count)
  {
    if (max_count <= 0) {
      return;
    }

    cv::Mat image(height_, width_, CV_8UC1);
    const int scale = static_cast<int>(255.0 / static_cast<double>(max_count)) + 1;

    for (int i = 0; i < height_; ++i) {
      for (int j = 0; j < width_; ++j) {
        const int v = count_image[i][j] * scale;
        image.at<uint8_t>(i, j) = static_cast<uint8_t>(std::clamp(v, 0, 255));
      }
    }

    auto ros_img = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", image).toImageMsg();
    ros_img->header.stamp = this->now();
    image_pub_->publish(*ros_img);
  }

  void data_process(std::vector<dvs_msgs::msg::Event> & events,
                    const std::vector<sensor_msgs::msg::Imu> & imu_msgs)
  {
    // Guard conditions similar to ROS1
    const int64_t last_imu_ns = to_ns(imu_msgs.back().header.stamp);
    const int64_t first_event_ns = to_ns(events.front().ts);

    if (last_imu_ns <= first_event_ns) {
      // Not enough IMU coverage for this batch
      return;
    }

    // Average angular rate over a short window [t0-3ms, ...]
    double wx_sum = 0.0, wy_sum = 0.0, wz_sum = 0.0;
    int cnt = 0;
    const int64_t window_start = first_event_ns - 3000000LL; // 3 ms in ns

    for (const auto & imu : imu_msgs) {
      const int64_t t = to_ns(imu.header.stamp);
      if (t >= window_start) {
        wx_sum += imu.angular_velocity.x;
        wy_sum += imu.angular_velocity.y;
        wz_sum += imu.angular_velocity.z;
        cnt++;
      }
    }

    if (cnt == 0) {
      return;
    }

    const double wx = wx_sum / static_cast<double>(cnt);
    const double wy = wy_sum / static_cast<double>(cnt);
    const double wz = wz_sum / static_cast<double>(cnt);

    std::vector<std::vector<int>> count_image(height_, std::vector<int>(width_, 0));
    std::vector<std::vector<double>> time_image(height_, std::vector<double>(width_, 0.0));

    const int64_t t0 = first_event_ns;

    for (auto & e : events) {
      const int64_t te = to_ns(e.ts);
      const double time_diff = static_cast<double>(te - t0) / 1e9; // seconds

      // Rotation offsets
      const double x_ang = time_diff * wx;
      const double y_ang = time_diff * wy;
      const double z_ang = time_diff * wz;

      int x = static_cast<int>(e.x) - width_ / 2;
      int y = static_cast<int>(e.y) - height_ / 2;

      // Incident angles at initial position
      const double pre_x_ang = std::atan(static_cast<double>(y) * pixel_size_ / focus_);
      const double pre_y_ang = std::atan(static_cast<double>(x) * pixel_size_ / focus_);

      // Nonlinear compensation (same formulas as ROS1 reference)
      const double comp_x =
        (static_cast<double>(x) * std::cos(z_ang) - std::sin(z_ang) * static_cast<double>(y))
        - (static_cast<double>(x) - (focus_ * std::tan(pre_y_ang + y_ang) / pixel_size_))
        + static_cast<double>(width_) / 2.0;

      const double comp_y =
        (static_cast<double>(x) * std::sin(z_ang) + std::cos(z_ang) * static_cast<double>(y))
        - (static_cast<double>(y) - (focus_ * std::tan(pre_x_ang - x_ang) / pixel_size_))
        + static_cast<double>(height_) / 2.0;

      const int compen_x = static_cast<int>(comp_x);
      const int compen_y = static_cast<int>(comp_y);

      e.x = static_cast<uint16_t>(std::max(0, std::min(width_ - 1, compen_x)));
      e.y = static_cast<uint16_t>(std::max(0, std::min(height_ - 1, compen_y)));

      if (compen_y >= 0 && compen_y < height_ && compen_x >= 0 && compen_x < width_) {
        if (count_image[compen_y][compen_x] < 20) {
          count_image[compen_y][compen_x]++;
        }
        time_image[compen_y][compen_x] += time_diff;
      }
    }

    int max_count = 0;
    for (int i = 0; i < height_; ++i) {
      for (int j = 0; j < width_; ++j) {
        if (count_image[i][j] != 0) {
          time_image[i][j] /= static_cast<double>(count_image[i][j]);
          max_count = std::max(max_count, count_image[i][j]);
        }
      }
    }

    publish_count_image(count_image, max_count);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MotionCompensationNode>();
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
