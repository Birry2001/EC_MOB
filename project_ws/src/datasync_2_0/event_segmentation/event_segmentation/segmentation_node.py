import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from message_filters import Subscriber, ApproximateTimeSynchronizer
from scipy import ndimage


class EventSegmentationNode(Node):
    def __init__(self):
        super().__init__("event_segmentation")

        self.declare_parameter("time_image_topic", "/time_image")
        self.declare_parameter("count_image_topic", "/count_image")
        self.declare_parameter("mask_topic", "/event_mask")
        self.declare_parameter("min_count", 1)
        self.declare_parameter("z_threshold", 1.0)
        self.declare_parameter("use_percentile", True)
        self.declare_parameter("percentile", 90.0)
        self.declare_parameter("use_time_image", True)
        self.declare_parameter("invert_time", False)
        self.declare_parameter("min_time", 0.0)
        self.declare_parameter("log_stats", False)
        self.declare_parameter("morph_open", True)
        self.declare_parameter("morph_close", True)
        self.declare_parameter("morph_kernel", 3)

        time_topic = self.get_parameter("time_image_topic").get_parameter_value().string_value
        count_topic = self.get_parameter("count_image_topic").get_parameter_value().string_value
        mask_topic = self.get_parameter("mask_topic").get_parameter_value().string_value

        self._min_count = self.get_parameter("min_count").get_parameter_value().integer_value
        self._z_threshold = self.get_parameter("z_threshold").get_parameter_value().double_value
        self._use_percentile = (
            self.get_parameter("use_percentile").get_parameter_value().bool_value
        )
        self._percentile = (
            self.get_parameter("percentile").get_parameter_value().double_value
        )
        self._use_time_image = (
            self.get_parameter("use_time_image").get_parameter_value().bool_value
        )
        self._invert_time = self.get_parameter("invert_time").get_parameter_value().bool_value
        self._min_time = self.get_parameter("min_time").get_parameter_value().double_value
        self._log_stats = self.get_parameter("log_stats").get_parameter_value().bool_value
        self._morph_open = self.get_parameter("morph_open").get_parameter_value().bool_value
        self._morph_close = self.get_parameter("morph_close").get_parameter_value().bool_value
        self._morph_kernel = int(self.get_parameter("morph_kernel").get_parameter_value().integer_value)

        self._bridge = CvBridge()
        self._mask_pub = self.create_publisher(Image, mask_topic, 10)

        time_sub = Subscriber(self, Image, time_topic)
        count_sub = Subscriber(self, Image, count_topic)
        sync = ApproximateTimeSynchronizer([time_sub, count_sub], queue_size=10, slop=0.05)
        sync.registerCallback(self._on_images)

    def _on_images(self, time_msg: Image, count_msg: Image) -> None:
        time_img = self._bridge.imgmsg_to_cv2(time_msg, desired_encoding="32FC1")
        count_img = self._bridge.imgmsg_to_cv2(count_msg, desired_encoding="mono8")

        if time_img is None or count_img is None:
            return

        count_mask = count_img >= self._min_count
        if not np.any(count_mask):
            empty = np.zeros(count_img.shape, dtype=np.uint8)
            out_msg = self._bridge.cv2_to_imgmsg(empty, encoding="mono8")
            out_msg.header = time_msg.header
            self._mask_pub.publish(out_msg)
            return

        if self._use_time_image:
            times = time_img[count_mask].astype(np.float32)
            if self._use_percentile:
                threshold = float(np.percentile(times, self._percentile))
            else:
                mean = float(np.mean(times))
                std = float(np.std(times))
                threshold = mean + self._z_threshold * std

            if self._invert_time:
                fg_mask = (time_img < threshold) & count_mask & (time_img >= self._min_time)
            else:
                fg_mask = (time_img > threshold) & count_mask & (time_img >= self._min_time)
        else:
            threshold = 0.0
            fg_mask = count_mask.copy()

        if self._log_stats:
            nonzero = times if self._use_time_image else np.array([], dtype=np.float32)
            fg_count = int(np.count_nonzero(fg_mask))
            self.get_logger().info(
                f"mask: count={int(np.count_nonzero(count_mask))} "
                f"fg={fg_count} "
                f"thr={threshold:.6f} "
                f"time_min={float(nonzero.min()) if nonzero.size else 0.0:.6f} "
                f"time_max={float(nonzero.max()) if nonzero.size else 0.0:.6f}"
            )

        if self._morph_open or self._morph_close:
            k = max(1, self._morph_kernel)
            structure = np.ones((k, k), dtype=bool)
            if self._morph_open:
                fg_mask = ndimage.binary_opening(fg_mask, structure=structure)
            if self._morph_close:
                fg_mask = ndimage.binary_closing(fg_mask, structure=structure)

        mask_uint8 = (fg_mask.astype(np.uint8) * 255)
        out_msg = self._bridge.cv2_to_imgmsg(mask_uint8, encoding="mono8")
        out_msg.header = time_msg.header
        self._mask_pub.publish(out_msg)


def main() -> None:
    rclpy.init()
    node = EventSegmentationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
