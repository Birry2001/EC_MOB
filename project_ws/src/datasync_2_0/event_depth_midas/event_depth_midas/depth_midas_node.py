import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class MidasDepthNode(Node):
    def __init__(self):
        super().__init__("event_depth_midas")

        self.declare_parameter("image_topic", "/event_camera/image_raw")
        self.declare_parameter("depth_topic", "/aps_depth")
        self.declare_parameter("model_type", "MiDaS_small")
        self.declare_parameter("device", "cuda")
        self.declare_parameter("invert_depth", True)
        self.declare_parameter("scale", 1.0)
        self.declare_parameter("min_depth", 0.1)
        self.declare_parameter("max_depth", 10.0)
        self.declare_parameter("normalize_depth", True)
        self.declare_parameter("norm_low_pct", 5.0)
        self.declare_parameter("norm_high_pct", 95.0)
        self.declare_parameter("log_stats", False)

        self._image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        self._depth_topic = self.get_parameter("depth_topic").get_parameter_value().string_value
        self._model_type = self.get_parameter("model_type").get_parameter_value().string_value
        self._device = self.get_parameter("device").get_parameter_value().string_value
        self._invert_depth = self.get_parameter("invert_depth").get_parameter_value().bool_value
        self._scale = self.get_parameter("scale").get_parameter_value().double_value
        self._min_depth = self.get_parameter("min_depth").get_parameter_value().double_value
        self._max_depth = self.get_parameter("max_depth").get_parameter_value().double_value
        self._normalize_depth = (
            self.get_parameter("normalize_depth").get_parameter_value().bool_value
        )
        self._norm_low_pct = self.get_parameter("norm_low_pct").get_parameter_value().double_value
        self._norm_high_pct = (
            self.get_parameter("norm_high_pct").get_parameter_value().double_value
        )
        self._log_stats = self.get_parameter("log_stats").get_parameter_value().bool_value

        self._bridge = CvBridge()
        self._depth_pub = self.create_publisher(Image, self._depth_topic, 10)
        self._frame_count = 0

        self._midas = None
        self._transform = None
        self._init_model()

        self.create_subscription(Image, self._image_topic, self._on_image, qos_profile_sensor_data)

    def _init_model(self):
        try:
            import torch
            # lazy import to avoid errors in environments without torch
            self._torch = torch
            self._midas = torch.hub.load("intel-isl/MiDaS", self._model_type, trust_repo=True)
            self._midas.to(self._device)
            self._midas.eval()
            transforms = torch.hub.load("intel-isl/MiDaS", "transforms", trust_repo=True)
            if self._model_type == "MiDaS_small":
                self._transform = transforms.small_transform
            else:
                self._transform = transforms.default_transform
            self.get_logger().info(f"MiDaS model loaded: {self._model_type} on {self._device}")
        except Exception as exc:  # pylint: disable=broad-except
            self.get_logger().error(f"Failed to load MiDaS: {exc}")
            self._midas = None

    def _on_image(self, msg: Image):
        if self._midas is None:
            return

        try:
            if msg.encoding == "mono8":
                mono = self._bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
                img = cv2.cvtColor(mono, cv2.COLOR_GRAY2BGR)
            else:
                img = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            img = img.astype(np.float32) / 255.0
        except Exception as exc:  # pylint: disable=broad-except
            self.get_logger().error(f"MiDaS image decode failed: {exc}")
            return

        input_batch = self._transform(img)
        with self._torch.no_grad():
            prediction = self._midas(input_batch.to(self._device))
            prediction = self._torch.nn.functional.interpolate(
                prediction.unsqueeze(1),
                size=img.shape[:2],
                mode="bilinear",
                align_corners=False,
            ).squeeze()
        depth = prediction.cpu().numpy().astype(np.float32)

        if self._invert_depth:
            depth = 1.0 / (depth + 1e-6)

        if self._normalize_depth:
            finite = depth[np.isfinite(depth)]
            if finite.size > 0:
                low = np.percentile(finite, self._norm_low_pct)
                high = np.percentile(finite, self._norm_high_pct)
                if high - low > 1e-6:
                    depth = (depth - low) / (high - low)
                    depth = depth * (self._max_depth - self._min_depth) + self._min_depth
                else:
                    self.get_logger().warn(
                        "MiDaS depth normalization skipped: low/high too close"
                    )

        depth *= float(self._scale)
        depth = np.clip(depth, self._min_depth, self._max_depth)

        depth_msg = self._bridge.cv2_to_imgmsg(depth, encoding="32FC1")
        depth_msg.header = msg.header
        self._depth_pub.publish(depth_msg)
        self._frame_count += 1
        if self._frame_count % 10 == 0:
            if self._log_stats:
                finite = depth[np.isfinite(depth)]
                if finite.size > 0:
                    self.get_logger().info(
                        f"MiDaS depth frames={self._frame_count} "
                        f"min={float(finite.min()):.4f} "
                        f"max={float(finite.max()):.4f} "
                        f"mean={float(finite.mean()):.4f} "
                        f"std={float(finite.std()):.4f}"
                    )
            else:
                self.get_logger().info(f"MiDaS depth published frames={self._frame_count}")


def main() -> None:
    rclpy.init()
    node = MidasDepthNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
