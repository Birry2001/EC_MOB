import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster


def rpy_to_quaternion(roll: float, pitch: float, yaw: float):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw


class StaticTfNode(Node):
    def __init__(self):
        super().__init__("event_tf_static")

        self.declare_parameter("parent_frame", "base_link")
        self.declare_parameter("child_frame", "event_camera")
        self.declare_parameter("x", 0.0)
        self.declare_parameter("y", 0.0)
        self.declare_parameter("z", 0.30)
        self.declare_parameter("roll_deg", 0.0)
        self.declare_parameter("pitch_deg", 40.0)
        self.declare_parameter("yaw_deg", 0.0)

        parent_frame = self.get_parameter("parent_frame").get_parameter_value().string_value
        child_frame = self.get_parameter("child_frame").get_parameter_value().string_value
        x = self.get_parameter("x").get_parameter_value().double_value
        y = self.get_parameter("y").get_parameter_value().double_value
        z = self.get_parameter("z").get_parameter_value().double_value
        roll = math.radians(self.get_parameter("roll_deg").get_parameter_value().double_value)
        pitch = math.radians(self.get_parameter("pitch_deg").get_parameter_value().double_value)
        yaw = math.radians(self.get_parameter("yaw_deg").get_parameter_value().double_value)

        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = parent_frame
        transform.child_frame_id = child_frame
        transform.transform.translation.x = float(x)
        transform.transform.translation.y = float(y)
        transform.transform.translation.z = float(z)
        qx, qy, qz, qw = rpy_to_quaternion(roll, pitch, yaw)
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw

        self._broadcaster = StaticTransformBroadcaster(self)
        self._broadcaster.sendTransform(transform)
        self.get_logger().info(
            f"Static TF published {parent_frame} -> {child_frame} (z={z}m, pitch={math.degrees(pitch)}deg)"
        )


def main() -> None:
    rclpy.init()
    node = StaticTfNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
