#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <algorithm>
#include <cmath>
#include <mutex>
#include <vector>

#include <event_camera_msgs/msg/event_packet.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>

#include <event_camera_codecs/decoder_factory.h>
#include <event_camera_codecs/decoder.h>

class EventVisualizer : public rclcpp::Node {
private:
    struct EventPoint {
        int x;
        int y;
        uint64_t ts;
    };

    std::vector<EventPoint> event_buffer_;
    std::vector<sensor_msgs::msg::Imu> imu_buffer_;
    std::mutex mtx_;
    bool first_event_received_ = true;

    int height_;
    int weight_;
    double focus_;
    double pixel_size_;

    rclcpp::Subscription<event_camera_msgs::msg::EventPacket>::SharedPtr event_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    image_transport::Publisher count_image_pub_;
    image_transport::Publisher time_image_pub_;
    image_transport::Publisher time_image_vis_pub_;
    bool publish_time_image_vis_{true};
    bool enable_compensation_{true};
    bool publish_raw_count_{false};
    bool publish_comp_count_{false};
    std::string raw_count_topic_;
    std::string comp_count_topic_;
    image_transport::Publisher raw_count_pub_;
    image_transport::Publisher comp_count_pub_;
    bool use_event_order_time_{false};
    bool log_time_stats_{false};
    double event_window_sec_{0.001};

    struct EventCollector {
        std::vector<EventPoint> &events;
        bool eventCD(uint64_t ts, uint16_t x, uint16_t y, [[maybe_unused]] bool p) {
            events.push_back(EventPoint{static_cast<int>(x), static_cast<int>(y), ts});
            return true;
        }
        void rawData(const char*, size_t) {}
        bool eventExtTrigger(uint64_t, bool, uint8_t) { return true; }
        void finished() {}
    };

public:
    EventVisualizer() : Node("datasync_node") {
        this->declare_parameter("weight_param", 346);
        this->declare_parameter("height_param", 260);
        this->declare_parameter("focus", 6550.0);
        this->declare_parameter("pixel_size", 18.5);
        this->declare_parameter("events_topic", "/event_camera/events");
        this->declare_parameter("imu_topic", "/event_camera/imu");
        this->declare_parameter("count_image_topic", "/count_image");
        this->declare_parameter("time_image_topic", "/time_image");
        this->declare_parameter("time_image_vis_topic", "/time_image_vis");
        this->declare_parameter("publish_time_image_vis", true);
        this->declare_parameter("use_event_order_time", false);
        this->declare_parameter("event_window_sec", 0.001);
        this->declare_parameter("log_time_stats", false);
        this->declare_parameter("enable_compensation", true);
        this->declare_parameter("publish_raw_count", false);
        this->declare_parameter("publish_comp_count", false);
        this->declare_parameter("raw_count_topic", "/count_image_raw");
        this->declare_parameter("comp_count_topic", "/count_image_comp");

        weight_ = this->get_parameter("weight_param").as_int();
        height_ = this->get_parameter("height_param").as_int();
        focus_ = this->get_parameter("focus").as_double();
        pixel_size_ = this->get_parameter("pixel_size").as_double();

        const auto events_topic = this->get_parameter("events_topic").as_string();
        const auto imu_topic = this->get_parameter("imu_topic").as_string();
        const auto count_image_topic = this->get_parameter("count_image_topic").as_string();
        const auto time_image_topic = this->get_parameter("time_image_topic").as_string();
        const auto time_image_vis_topic = this->get_parameter("time_image_vis_topic").as_string();
        publish_time_image_vis_ = this->get_parameter("publish_time_image_vis").as_bool();
        use_event_order_time_ = this->get_parameter("use_event_order_time").as_bool();
        event_window_sec_ = this->get_parameter("event_window_sec").as_double();
        log_time_stats_ = this->get_parameter("log_time_stats").as_bool();
        enable_compensation_ = this->get_parameter("enable_compensation").as_bool();
        publish_raw_count_ = this->get_parameter("publish_raw_count").as_bool();
        publish_comp_count_ = this->get_parameter("publish_comp_count").as_bool();
        raw_count_topic_ = this->get_parameter("raw_count_topic").as_string();
        comp_count_topic_ = this->get_parameter("comp_count_topic").as_string();

        auto qos = rclcpp::SensorDataQoS();
        event_sub_ = this->create_subscription<event_camera_msgs::msg::EventPacket>(
            events_topic, qos,
            std::bind(&EventVisualizer::event_cb, this, std::placeholders::_1));
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic, qos,
            std::bind(&EventVisualizer::imu_cb, this, std::placeholders::_1));

        count_image_pub_ = image_transport::create_publisher(this, count_image_topic);
        time_image_pub_ = image_transport::create_publisher(this, time_image_topic);
        if (publish_time_image_vis_) {
            time_image_vis_pub_ = image_transport::create_publisher(this, time_image_vis_topic);
        }
        if (publish_raw_count_) {
            raw_count_pub_ = image_transport::create_publisher(this, raw_count_topic_);
        }
        if (publish_comp_count_) {
            comp_count_pub_ = image_transport::create_publisher(this, comp_count_topic_);
        }
    }

    void imu_cb(const sensor_msgs::msg::Imu::SharedPtr imu) {
        if (first_event_received_) {
            return;
        }
        std::lock_guard<std::mutex> lock(mtx_);
        imu_buffer_.push_back(*imu);
    }

    void event_cb(const event_camera_msgs::msg::EventPacket::SharedPtr msg) {
        if (first_event_received_) {
            first_event_received_ = false;
            std::lock_guard<std::mutex> lock(mtx_);
            if (!imu_buffer_.empty()) {
                imu_buffer_.clear();
            }
            RCLCPP_INFO(this->get_logger(), "Data aligned! Start processing data...");
            return;
        }

        std::vector<sensor_msgs::msg::Imu> imu_buffer_copy;
        {
            std::lock_guard<std::mutex> lock(mtx_);
            imu_buffer_copy = imu_buffer_;
            if (!imu_buffer_.empty()) {
                imu_buffer_.clear();
            }
        }
        if (imu_buffer_copy.empty()) {
            return;
        }

        event_buffer_.clear();
        EventCollector collector{event_buffer_};
        event_camera_codecs::DecoderFactory<event_camera_msgs::msg::EventPacket, EventCollector> factory;
        auto decoder = factory.newInstance(*msg);
        if (!decoder) {
            RCLCPP_WARN(this->get_logger(), "No decoder for event packet encoding '%s'", msg->encoding.c_str());
            return;
        }
        decoder->decode(*msg, &collector);
        if (event_buffer_.empty()) {
            return;
        }

        data_process(imu_buffer_copy);
    }

private:
    void show_count_image(
        const std::vector<std::vector<int>> &count_image, int max_count,
        const rclcpp::Time &stamp, image_transport::Publisher &pub) {
        if (max_count <= 0) {
            return;
        }
        cv::Mat image(height_, weight_, CV_8UC1);
        int scale = static_cast<int>(255 / max_count) + 1;
        for (int i = 0; i < height_; ++i) {
            for (int j = 0; j < weight_; ++j) {
                image.at<uchar>(i, j) = static_cast<uchar>(count_image[i][j] * scale);
            }
        }

        std_msgs::msg::Header header;
        header.stamp = stamp;
        sensor_msgs::msg::Image::SharedPtr msg =
            cv_bridge::CvImage(header, "mono8", image).toImageMsg();
        pub.publish(msg);
    }

    void show_time_image(
        const std::vector<std::vector<float>> &time_image, float max_time,
        const rclcpp::Time &stamp) {
        cv::Mat image(height_, weight_, CV_32FC1);
        for (int i = 0; i < height_; ++i) {
            for (int j = 0; j < weight_; ++j) {
                image.at<float>(i, j) = time_image[i][j];
            }
        }

        std_msgs::msg::Header header;
        header.stamp = stamp;
        sensor_msgs::msg::Image::SharedPtr msg =
            cv_bridge::CvImage(header, "32FC1", image).toImageMsg();
        time_image_pub_.publish(msg);

        if (publish_time_image_vis_ && max_time > 0.0f) {
            cv::Mat vis(height_, weight_, CV_8UC1);
            const float scale = 255.0f / max_time;
            for (int i = 0; i < height_; ++i) {
                for (int j = 0; j < weight_; ++j) {
                    const float val = time_image[i][j] * scale;
                    vis.at<uchar>(i, j) = static_cast<uchar>(std::min(255.0f, val));
                }
            }
            sensor_msgs::msg::Image::SharedPtr vis_msg =
                cv_bridge::CvImage(header, "mono8", vis).toImageMsg();
            time_image_vis_pub_.publish(vis_msg);
        }
    }

    void data_process(const std::vector<sensor_msgs::msg::Imu> &imu_buffer_local) {
        const uint64_t first_event_ts = event_buffer_.front().ts;
        const int64_t last_imu_ts = rclcpp::Time(imu_buffer_local.back().header.stamp).nanoseconds();
        if (last_imu_ts <= static_cast<int64_t>(first_event_ts)) {
            event_buffer_.clear();
            return;
        }

        double angular_velocity_x = 0.0;
        double angular_velocity_y = 0.0;
        double angular_velocity_z = 0.0;
        int cnt = 0;

        for (const auto & imu : imu_buffer_local) {
            const int64_t imu_ts = rclcpp::Time(imu.header.stamp).nanoseconds();
            if (imu_ts >= static_cast<int64_t>(first_event_ts) - 3000000) {
                angular_velocity_x += imu.angular_velocity.x;
                angular_velocity_y += imu.angular_velocity.y;
                angular_velocity_z += imu.angular_velocity.z;
                cnt++;
            }
        }
        if (cnt == 0) {
            event_buffer_.clear();
            return;
        }

        const double average_angular_rate_x = angular_velocity_x / static_cast<double>(cnt);
        const double average_angular_rate_y = angular_velocity_y / static_cast<double>(cnt);
        const double average_angular_rate_z = angular_velocity_z / static_cast<double>(cnt);
        const double average_angular_rate = std::sqrt(
            (average_angular_rate_x * average_angular_rate_x) +
            (average_angular_rate_y * average_angular_rate_y) +
            (average_angular_rate_z * average_angular_rate_z));
        if (log_time_stats_) {
            RCLCPP_INFO(
                this->get_logger(),
                "imu stats: cnt=%d wx=%.6f wy=%.6f wz=%.6f w=%.6f",
                cnt, average_angular_rate_x, average_angular_rate_y,
                average_angular_rate_z, average_angular_rate);
        }

        const uint64_t t0 = event_buffer_.front().ts;
        std::vector<std::vector<int>> count_image(height_, std::vector<int>(weight_, 0));
        std::vector<std::vector<int>> raw_count_image;
        if (publish_raw_count_) {
            raw_count_image.assign(height_, std::vector<int>(weight_, 0));
        }
        std::vector<std::vector<float>> time_image(height_, std::vector<float>(weight_, 0.0f));

        uint64_t min_ts = event_buffer_.front().ts;
        uint64_t max_ts = event_buffer_.front().ts;
        for (const auto & ev : event_buffer_) {
            min_ts = std::min(min_ts, ev.ts);
            max_ts = std::max(max_ts, ev.ts);
        }
        const bool timestamps_flat = (max_ts == min_ts);
        if (log_time_stats_) {
            RCLCPP_INFO(
                this->get_logger(),
                "event timestamps: min=%lu max=%lu flat=%s events=%zu",
                static_cast<unsigned long>(min_ts),
                static_cast<unsigned long>(max_ts),
                timestamps_flat ? "true" : "false",
                event_buffer_.size());
        }

        const size_t denom = event_buffer_.size() > 1 ? (event_buffer_.size() - 1) : 1;
        size_t idx = 0;
        for (auto & ev : event_buffer_) {
            double time_diff = static_cast<double>(ev.ts - t0) / 1e9;
            if (use_event_order_time_ && timestamps_flat) {
                time_diff = event_window_sec_ * (static_cast<double>(idx) / denom);
            }
            idx++;

            const double x_angular = time_diff * average_angular_rate_x;
            const double y_angular = time_diff * average_angular_rate_y;
            const double z_angular = time_diff * average_angular_rate_z;

            const int x = ev.x - weight_ / 2;
            const int y = ev.y - height_ / 2;

            const double pre_x_angle = atan(y * pixel_size_ / focus_);
            const double pre_y_angle = atan(x * pixel_size_ / focus_);

            int compen_x = ev.x;
            int compen_y = ev.y;
            if (enable_compensation_) {
                compen_x = static_cast<int>(
                    (x * cos(z_angular) - sin(z_angular) * y) -
                    (x - (focus_ * tan(pre_y_angle + y_angular) / pixel_size_)) +
                    weight_ / 2);
                compen_y = static_cast<int>(
                    (x * sin(z_angular) + cos(z_angular) * y) -
                    (y - (focus_ * tan(pre_x_angle - x_angular) / pixel_size_)) +
                    height_ / 2);
            }

            ev.x = compen_x;
            ev.y = compen_y;

            if (publish_raw_count_) {
                if (ev.y < height_ && ev.y >= 0 && ev.x < weight_ && ev.x >= 0) {
                    if (raw_count_image[ev.y][ev.x] < 20) {
                        raw_count_image[ev.y][ev.x]++;
                    }
                }
            }

            if (compen_y < height_ && compen_y >= 0 && compen_x < weight_ && compen_x >= 0) {
                if (count_image[compen_y][compen_x] < 20) {
                    count_image[compen_y][compen_x]++;
                }
                time_image[compen_y][compen_x] += static_cast<float>(time_diff);
            }
        }

        int max_count = 0;
        float max_time = 0.0f;
        float total_time = 0.0f;
        float average_time = 0.0f;
        int trigger_pixels = 0;
        int nonzero_time_pixels = 0;

        for (int i = 0; i < height_; ++i) {
            for (int j = 0; j < weight_; ++j) {
                if (count_image[i][j] != 0) {
                    time_image[i][j] /= count_image[i][j];
                    max_count = std::max(max_count, count_image[i][j]);
                    max_time = std::max(max_time, time_image[i][j]);
                    total_time += time_image[i][j];
                    trigger_pixels++;
                    if (time_image[i][j] > 0.0f) {
                        nonzero_time_pixels++;
                    }
                }
            }
        }

        if (trigger_pixels > 0) {
            average_time = total_time / trigger_pixels;
        }
        (void)average_time;
        if (log_time_stats_) {
            RCLCPP_INFO(
                this->get_logger(),
                "time_image stats: max_time=%.6f nonzero=%d trigger=%d max_count=%d",
                max_time, nonzero_time_pixels, trigger_pixels, max_count);
        }

        const rclcpp::Time stamp(t0);
        show_count_image(count_image, max_count, stamp, count_image_pub_);
        if (publish_raw_count_) {
            int raw_max = 0;
            for (int i = 0; i < height_; ++i) {
                for (int j = 0; j < weight_; ++j) {
                    raw_max = std::max(raw_max, raw_count_image[i][j]);
                }
            }
            show_count_image(raw_count_image, raw_max, stamp, raw_count_pub_);
        }
        if (publish_comp_count_) {
            show_count_image(count_image, max_count, stamp, comp_count_pub_);
        }
        show_time_image(time_image, max_time, stamp);
        event_buffer_.clear();
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EventVisualizer>();
    rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 3);
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
