#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <algorithm>
#include <cmath>
#include <limits>
#include <mutex>
#include <vector>
#include <deque>

#include <event_camera_msgs/msg/event_packet.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h> 
#include <image_transport/image_transport.hpp>

#include <event_camera_codecs/decoder_factory.h>
#include <event_camera_codecs/decoder.h>

class EventVisualizer : public rclcpp::Node {
private:
    // Utilisation d'un deque pour un nettoyage efficace en tête
    std::deque<sensor_msgs::msg::Imu> imu_buffer_;
    std::mutex mtx_;
    bool first_event_received_ = true;

    int height_, weight_;
    double Focus_, pixel_size_;
    double imu_buffer_seconds_;
    double imu_sync_window_ms_;
    double imu_fallback_max_gap_ms_;
    double imu_time_offset_sec_;
    double imu_sync_window_missing_ms_;
    int sync_warn_throttle_ms_;
    bool use_integrated_gyro_;
    bool auto_imu_time_offset_;
    bool imu_time_offset_initialized_ = false;
    int64_t imu_time_offset_ns_ = 0;
    bool debug_sync_;
    bool enable_translation_comp_;
    double plane_depth_m_;
    double vel_x_mps_;
    double vel_y_mps_;
    double vel_z_mps_;

    rclcpp::Subscription<event_camera_msgs::msg::EventPacket>::SharedPtr event_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    image_transport::Publisher image_pub_;
    rclcpp::CallbackGroup::SharedPtr event_cb_group_;
    rclcpp::CallbackGroup::SharedPtr imu_cb_group_;

    // Structure interne pour le décodage (Motion Handler)
    struct MotionHandler {
        int w, h;
        double foc, pix, ax, ay, az;
        uint64_t t0 = 0;
        bool is_first = true;
        int max_cnt = 0;
        std::vector<std::vector<int>> &img;
        const std::vector<int64_t> * imu_ts = nullptr;
        const std::vector<double> * imu_int_x = nullptr;
        const std::vector<double> * imu_int_y = nullptr;
        const std::vector<double> * imu_int_z = nullptr;
        bool use_integrated = false;
        bool base_int_valid = false;
        double base_int_x = 0.0, base_int_y = 0.0, base_int_z = 0.0;
        size_t imu_idx = 0;
        bool enable_translation = false;
        double plane_depth_m = 1.0;
        double vel_x = 0.0, vel_y = 0.0, vel_z = 0.0;
        double pixel_size_m = 0.0;
        double focal_m = 0.0;

        bool lookup_integral(int64_t t_ns, double &ix, double &iy, double &iz) {
            if (!imu_ts || imu_ts->empty()) return false;
            if (t_ns < (*imu_ts)[0] || t_ns > (*imu_ts)[imu_ts->size() - 1]) return false;
            while (imu_idx + 1 < imu_ts->size() && (*imu_ts)[imu_idx + 1] <= t_ns) {
                ++imu_idx;
            }
            if (imu_idx + 1 >= imu_ts->size()) {
                ix = (*imu_int_x)[imu_idx];
                iy = (*imu_int_y)[imu_idx];
                iz = (*imu_int_z)[imu_idx];
                return true;
            }
            const int64_t t0i = (*imu_ts)[imu_idx];
            const int64_t t1i = (*imu_ts)[imu_idx + 1];
            const double frac = (t1i == t0i) ? 0.0 : (static_cast<double>(t_ns - t0i) / static_cast<double>(t1i - t0i));
            ix = (*imu_int_x)[imu_idx] + ((*imu_int_x)[imu_idx + 1] - (*imu_int_x)[imu_idx]) * frac;
            iy = (*imu_int_y)[imu_idx] + ((*imu_int_y)[imu_idx + 1] - (*imu_int_y)[imu_idx]) * frac;
            iz = (*imu_int_z)[imu_idx] + ((*imu_int_z)[imu_idx + 1] - (*imu_int_z)[imu_idx]) * frac;
            return true;
        }

        bool eventCD(uint64_t ts, uint16_t x, uint16_t y, [[maybe_unused]] bool p) {
            if (is_first) { t0 = ts; is_first = false; }
            const double time_diff = static_cast<double>(ts - t0) / 1e9;

            double x_ang = 0.0;
            double y_ang = 0.0;
            double z_ang = 0.0;
            if (use_integrated && imu_ts && imu_int_x && imu_int_y && imu_int_z) {
                const int64_t t0_ns = static_cast<int64_t>(t0);
                double cur_x = 0.0, cur_y = 0.0, cur_z = 0.0;
                if (!base_int_valid) {
                    double base_x = 0.0, base_y = 0.0, base_z = 0.0;
                    if (lookup_integral(t0_ns, base_x, base_y, base_z)) {
                        base_int_x = base_x;
                        base_int_y = base_y;
                        base_int_z = base_z;
                        base_int_valid = true;
                    }
                }
                if (base_int_valid && lookup_integral(static_cast<int64_t>(ts), cur_x, cur_y, cur_z)) {
                    x_ang = cur_x - base_int_x;
                    y_ang = cur_y - base_int_y;
                    z_ang = cur_z - base_int_z;
                } else {
                    x_ang = time_diff * ax;
                    y_ang = time_diff * ay;
                    z_ang = time_diff * az;
                }
            } else {
                x_ang = time_diff * ax;
                y_ang = time_diff * ay;
                z_ang = time_diff * az;
            }

            double cx = static_cast<double>(x) - w / 2.0; 
            double cy = static_cast<double>(y) - h / 2.0;

            double pre_x_angle = atan(cy * pix / foc);
            double pre_y_angle = atan(cx * pix / foc);

            int compen_x = static_cast<int>((cx * cos(z_ang) - sin(z_ang) * cy) - 
                           (cx - (foc * tan(pre_y_angle + y_ang) / pix)) + w / 2.0);
            int compen_y = static_cast<int>((cx * sin(z_ang) + cos(z_ang) * cy) - 
                           (cy - (foc * tan(pre_x_angle - x_ang) / pix)) + h / 2.0);

            if (enable_translation && plane_depth_m > 0.0 && pixel_size_m > 0.0 && focal_m > 0.0) {
                const double x_m = cx * pixel_size_m;
                const double y_m = cy * pixel_size_m;
                const double u_dot = (-focal_m * vel_x + x_m * vel_z) / plane_depth_m;
                const double v_dot = (-focal_m * vel_y + y_m * vel_z) / plane_depth_m;
                const double dx_pix = (u_dot * time_diff) / pixel_size_m;
                const double dy_pix = (v_dot * time_diff) / pixel_size_m;
                compen_x -= static_cast<int>(dx_pix);
                compen_y -= static_cast<int>(dy_pix);
            }

            if (compen_y < h && compen_y >= 0 && compen_x < w && compen_x >= 0) {
                if (img[compen_y][compen_x] < 20) {
                    img[compen_y][compen_x]++;
                    if (img[compen_y][compen_x] > max_cnt) max_cnt = img[compen_y][compen_x];
                }
            }
            return true; 
        }
        // Overrides requis par le décodeur
        void rawData(const char*, size_t) {}
        bool eventExtTrigger(uint64_t, bool, uint8_t) { return true; }
        void finished() {}
    };

public:
    EventVisualizer() : Node("event_visualizer_node") {
        this->declare_parameter("weight_param", 346);
        this->declare_parameter("height_param", 260);
        this->declare_parameter("focus", 6550.0);
        this->declare_parameter("pixel_size", 18.5);
        this->declare_parameter("imu_buffer_seconds", 2.0);
        this->declare_parameter("imu_sync_window_ms", 3.0);
        this->declare_parameter("imu_fallback_max_gap_ms", 1000.0);
        this->declare_parameter("imu_time_offset_sec", 0.0);
        this->declare_parameter("imu_sync_window_missing_ms", 20.0);
        this->declare_parameter("sync_warn_throttle_ms", 2000);
        this->declare_parameter("use_integrated_gyro", true);
        this->declare_parameter("auto_imu_time_offset", true);
        this->declare_parameter("debug_sync", false);
        this->declare_parameter("enable_translation_comp", false);
        this->declare_parameter("plane_depth_m", 1.0);
        this->declare_parameter("camera_velocity_x_mps", 0.0);
        this->declare_parameter("camera_velocity_y_mps", 0.0);
        this->declare_parameter("camera_velocity_z_mps", 0.0);

        weight_ = this->get_parameter("weight_param").as_int();
        height_ = this->get_parameter("height_param").as_int();
        Focus_ = this->get_parameter("focus").as_double();
        pixel_size_ = this->get_parameter("pixel_size").as_double();
        imu_buffer_seconds_ = this->get_parameter("imu_buffer_seconds").as_double();
        imu_sync_window_ms_ = this->get_parameter("imu_sync_window_ms").as_double();
        imu_fallback_max_gap_ms_ = this->get_parameter("imu_fallback_max_gap_ms").as_double();
        imu_time_offset_sec_ = this->get_parameter("imu_time_offset_sec").as_double();
        imu_sync_window_missing_ms_ = this->get_parameter("imu_sync_window_missing_ms").as_double();
        sync_warn_throttle_ms_ = this->get_parameter("sync_warn_throttle_ms").as_int();
        use_integrated_gyro_ = this->get_parameter("use_integrated_gyro").as_bool();
        auto_imu_time_offset_ = this->get_parameter("auto_imu_time_offset").as_bool();
        enable_translation_comp_ = this->get_parameter("enable_translation_comp").as_bool();
        plane_depth_m_ = this->get_parameter("plane_depth_m").as_double();
        vel_x_mps_ = this->get_parameter("camera_velocity_x_mps").as_double();
        vel_y_mps_ = this->get_parameter("camera_velocity_y_mps").as_double();
        vel_z_mps_ = this->get_parameter("camera_velocity_z_mps").as_double();
        if (imu_time_offset_sec_ != 0.0) {
            imu_time_offset_ns_ = static_cast<int64_t>(imu_time_offset_sec_ * 1e9);
            imu_time_offset_initialized_ = true;
        }
        debug_sync_ = this->get_parameter("debug_sync").as_bool();

        auto qos = rclcpp::SensorDataQoS();

        event_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        imu_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        rclcpp::SubscriptionOptions event_options;
        event_options.callback_group = event_cb_group_;
        event_sub_ = this->create_subscription<event_camera_msgs::msg::EventPacket>(
            "/event_camera/events", qos,
            std::bind(&EventVisualizer::event_cb, this, std::placeholders::_1),
            event_options);

        rclcpp::SubscriptionOptions imu_options;
        imu_options.callback_group = imu_cb_group_;
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/event_camera/imu", qos,
            std::bind(&EventVisualizer::imu_cb, this, std::placeholders::_1),
            imu_options);
        
        image_pub_ = image_transport::create_publisher(this, "count_image");
    }

    void imu_cb(const sensor_msgs::msg::Imu::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mtx_);
        imu_buffer_.push_back(*msg);
        // Garder seulement les dernières secondes d'IMU (évite l'overflow)
        const auto newest_stamp = rclcpp::Time(msg->header.stamp);
        while (!imu_buffer_.empty()) {
            const auto oldest_stamp = rclcpp::Time(imu_buffer_.front().header.stamp);
            if ((newest_stamp - oldest_stamp).seconds() > imu_buffer_seconds_) {
                imu_buffer_.pop_front();
            } else {
                break;
            }
        }
    }

    void event_cb(const event_camera_msgs::msg::EventPacket::SharedPtr msg) {
        if (first_event_received_) {
            first_event_received_ = false;
            return;
        }

        event_camera_codecs::DecoderFactory<event_camera_msgs::msg::EventPacket, MotionHandler> factory;
        auto decoder = factory.newInstance(*msg);
        if (!decoder) {
            RCLCPP_WARN(this->get_logger(), "No decoder for event packet encoding '%s'", msg->encoding.c_str());
            return;
        }

        uint64_t first_event_ts = 0;
        uint64_t last_event_ts = 0;
        size_t num_events[2]{0, 0};
        const bool has_epoch_time = decoder->hasSensorTimeSinceEpoch();
        bool has_first_ts = decoder->findFirstSensorTime(*msg, &first_event_ts);
        if (!has_first_ts || first_event_ts == 0) {
            if (decoder->summarize(*msg, &first_event_ts, &last_event_ts, num_events)) {
                has_first_ts = true;
            }
        }
        int64_t ref_time_ns = rclcpp::Time(msg->header.stamp).nanoseconds();
        const char * ref_source = "header_stamp";
        if (has_epoch_time && has_first_ts && msg->time_base != 0 && first_event_ts != 0) {
            ref_time_ns = static_cast<int64_t>(first_event_ts);
            ref_source = "first_event_ts";
        }

        double av_x = 0, av_y = 0, av_z = 0;
        int cnt = 0;
        const double effective_window_ms = (has_first_ts && first_event_ts != 0) ?
            imu_sync_window_ms_ : imu_sync_window_missing_ms_;
        const int64_t window_ns = static_cast<int64_t>(effective_window_ms * 1e6);
        const int64_t fallback_ns = static_cast<int64_t>(imu_fallback_max_gap_ms_ * 1e6);
        int64_t min_abs_diff = std::numeric_limits<int64_t>::max();
        int64_t closest_imu_ts = 0;
        double closest_x = 0.0, closest_y = 0.0, closest_z = 0.0;
        std::vector<int64_t> imu_ts;
        std::vector<double> imu_int_x;
        std::vector<double> imu_int_y;
        std::vector<double> imu_int_z;

        {
            std::lock_guard<std::mutex> lock(mtx_);
            // Filtrer l'IMU pour correspondre au temps du paquet d'événements
            for (const auto & imu : imu_buffer_) {
                const int64_t imu_ts = rclcpp::Time(imu.header.stamp).nanoseconds();
                const int64_t adj_imu_ts = imu_ts - imu_time_offset_ns_;
                const int64_t diff = std::abs(adj_imu_ts - ref_time_ns);
                if (diff < min_abs_diff) {
                    min_abs_diff = diff;
                    closest_imu_ts = imu_ts;
                    closest_x = imu.angular_velocity.x;
                    closest_y = imu.angular_velocity.y;
                    closest_z = imu.angular_velocity.z;
                }
                // On prend les données IMU dans une fenêtre autour du temps de référence
                if (diff <= window_ns) {
                    av_x += imu.angular_velocity.x;
                    av_y += imu.angular_velocity.y;
                    av_z += imu.angular_velocity.z;
                    cnt++;
                }
            }

            if (debug_sync_ && !imu_buffer_.empty()) {
                const int64_t imu_start = rclcpp::Time(imu_buffer_.front().header.stamp).nanoseconds() -
                                          imu_time_offset_ns_;
                const int64_t imu_end = rclcpp::Time(imu_buffer_.back().header.stamp).nanoseconds() -
                                        imu_time_offset_ns_;
                RCLCPP_INFO_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    2000,
                    "sync debug: encoding=%s time_base=%llu header_ns=%lld first_event_ts=%llu has_epoch=%s ref=%s ref_ns=%lld imu_range=[%lld,%lld] window_ns=%lld min_diff_ns=%lld cnt=%d offset_ns=%lld",
                    msg->encoding.c_str(),
                    static_cast<unsigned long long>(msg->time_base),
                    static_cast<long long>(rclcpp::Time(msg->header.stamp).nanoseconds()),
                    static_cast<unsigned long long>(first_event_ts),
                    has_epoch_time ? "true" : "false",
                    ref_source,
                    static_cast<long long>(ref_time_ns),
                    static_cast<long long>(imu_start),
                    static_cast<long long>(imu_end),
                    static_cast<long long>(window_ns),
                    static_cast<long long>(min_abs_diff == std::numeric_limits<int64_t>::max() ? -1 : min_abs_diff),
                    cnt,
                    static_cast<long long>(imu_time_offset_ns_));
            }
        }

        if (!imu_time_offset_initialized_ && auto_imu_time_offset_ && closest_imu_ts != 0) {
            imu_time_offset_ns_ = closest_imu_ts - ref_time_ns;
            imu_time_offset_initialized_ = true;
            RCLCPP_WARN(
                this->get_logger(),
                "Auto IMU time offset set to %.3f s (imu_ts - ref_time)",
                static_cast<double>(imu_time_offset_ns_) / 1e9);
        }

        if (cnt == 0 && min_abs_diff != std::numeric_limits<int64_t>::max() &&
            (fallback_ns > 0 && min_abs_diff <= fallback_ns)) {
            av_x = closest_x;
            av_y = closest_y;
            av_z = closest_z;
            cnt = 1;
            RCLCPP_WARN(
                this->get_logger(),
                "Using closest IMU sample (diff_ms=%.3f, ref=%s)",
                static_cast<double>(min_abs_diff) / 1e6,
                ref_source);
        }

        if (cnt == 0) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                sync_warn_throttle_ms_,
                "No synchronized IMU samples for packet (ref=%s, window_ms=%.3f)",
                ref_source,
                effective_window_ms);
            return;
        }
        av_x /= cnt; av_y /= cnt; av_z /= cnt;

        if (use_integrated_gyro_) {
            std::lock_guard<std::mutex> lock(mtx_);
            imu_ts.reserve(imu_buffer_.size());
            imu_int_x.reserve(imu_buffer_.size());
            imu_int_y.reserve(imu_buffer_.size());
            imu_int_z.reserve(imu_buffer_.size());
            bool has_prev = false;
            int64_t prev_ts = 0;
            double prev_x = 0.0, prev_y = 0.0, prev_z = 0.0;
            double acc_x = 0.0, acc_y = 0.0, acc_z = 0.0;
            for (const auto & imu : imu_buffer_) {
                const int64_t ts = rclcpp::Time(imu.header.stamp).nanoseconds() - imu_time_offset_ns_;
                if (has_prev && ts <= prev_ts) {
                    continue;
                }
                const double gx = imu.angular_velocity.x;
                const double gy = imu.angular_velocity.y;
                const double gz = imu.angular_velocity.z;
                if (has_prev) {
                    const double dt = static_cast<double>(ts - prev_ts) / 1e9;
                    acc_x += 0.5 * (prev_x + gx) * dt;
                    acc_y += 0.5 * (prev_y + gy) * dt;
                    acc_z += 0.5 * (prev_z + gz) * dt;
                }
                imu_ts.push_back(ts);
                imu_int_x.push_back(acc_x);
                imu_int_y.push_back(acc_y);
                imu_int_z.push_back(acc_z);
                prev_ts = ts;
                prev_x = gx;
                prev_y = gy;
                prev_z = gz;
                has_prev = true;
            }
        }

        std::vector<std::vector<int>> count_img(height_, std::vector<int>(weight_, 0));
        MotionHandler handler{weight_, height_, Focus_, pixel_size_, av_x, av_y, av_z, 0, true, 0, count_img};
        if (use_integrated_gyro_ && imu_ts.size() >= 2) {
            handler.imu_ts = &imu_ts;
            handler.imu_int_x = &imu_int_x;
            handler.imu_int_y = &imu_int_y;
            handler.imu_int_z = &imu_int_z;
            handler.use_integrated = true;
        }
        handler.enable_translation = enable_translation_comp_;
        handler.plane_depth_m = plane_depth_m_;
        handler.vel_x = vel_x_mps_;
        handler.vel_y = vel_y_mps_;
        handler.vel_z = vel_z_mps_;
        handler.pixel_size_m = pixel_size_ * 1e-6;
        handler.focal_m = Focus_ * 1e-6;
        
        decoder->decode(*msg, &handler);
        if (handler.max_cnt > 0) {
            publish_image(count_img, handler.max_cnt, msg->header.stamp);
        }
    }

    void publish_image(const std::vector<std::vector<int>>& img, int max_cnt, rclcpp::Time stamp) {
        cv::Mat mat(height_, weight_, CV_8UC1);
        float scale = 255.0f / max_cnt;

        for (int i = 0; i < height_; ++i) {
            for (int j = 0; j < weight_; ++j) {
                mat.at<uchar>(i, j) = static_cast<uchar>(img[i][j] * scale);
            }
        }

        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", mat).toImageMsg();
        msg->header.stamp = stamp;
        msg->header.frame_id = "event_camera_frame";
        image_pub_.publish(msg);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EventVisualizer>();
    rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 2);
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
