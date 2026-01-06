import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge
from sensor_msgs_py import point_cloud2 as pc2
from scipy.spatial import cKDTree
from tf2_ros import Buffer, TransformListener


def dbscan(points: np.ndarray, eps: float, min_samples: int) -> np.ndarray:
    if points.size == 0:
        return np.array([], dtype=np.int32)

    tree = cKDTree(points)
    labels = -np.ones(points.shape[0], dtype=np.int32)
    visited = np.zeros(points.shape[0], dtype=bool)
    cluster_id = 0

    for idx in range(points.shape[0]):
        if visited[idx]:
            continue
        visited[idx] = True
        neighbors = tree.query_ball_point(points[idx], eps)
        if len(neighbors) < min_samples:
            labels[idx] = -1
            continue
        labels[idx] = cluster_id
        seeds = list(neighbors)
        while seeds:
            current = seeds.pop()
            if not visited[current]:
                visited[current] = True
                current_neighbors = tree.query_ball_point(points[current], eps)
                if len(current_neighbors) >= min_samples:
                    seeds.extend(current_neighbors)
            if labels[current] == -1:
                labels[current] = cluster_id
        cluster_id += 1

    return labels


class EventClusteringNode(Node):
    def __init__(self):
        super().__init__("event_clustering")

        self.declare_parameter("mask_topic", "/event_mask")
        self.declare_parameter("pointcloud_topic", "/dynamic_obstacles")
        self.declare_parameter("marker_topic", "/dynamic_obstacles_markers")
        self.declare_parameter("eps_pixels", 4.0)
        self.declare_parameter("min_samples", 20)
        self.declare_parameter("max_points", 15000)
        self.declare_parameter("pixel_scale_m", 0.01)
        self.declare_parameter("use_ground_plane", True)
        self.declare_parameter("camera_frame", "event_camera")
        self.declare_parameter("output_frame", "base_link")
        self.declare_parameter("fx", 300.0)
        self.declare_parameter("fy", 300.0)
        self.declare_parameter("cx", 173.0)
        self.declare_parameter("cy", 130.0)
        self.declare_parameter("k1", 0.0)
        self.declare_parameter("k2", 0.0)
        self.declare_parameter("p1", 0.0)
        self.declare_parameter("p2", 0.0)
        self.declare_parameter("k3", 0.0)
        self.declare_parameter("undistort", True)
        self.declare_parameter("undistort_iterations", 5)
        self.declare_parameter("plane_z", 0.0)
        self.declare_parameter("use_latest_tf", True)
        self.declare_parameter("publish_centroids_only", True)
        self.declare_parameter("log_stats", False)
        self.declare_parameter("use_depth", False)
        self.declare_parameter("depth_topic", "/aps_depth")
        self.declare_parameter("depth_scale", 1.0)
        self.declare_parameter("depth_min", 0.1)
        self.declare_parameter("depth_max", 10.0)
        self.declare_parameter("depth_use_median", True)

        mask_topic = self.get_parameter("mask_topic").get_parameter_value().string_value
        pc_topic = self.get_parameter("pointcloud_topic").get_parameter_value().string_value
        marker_topic = self.get_parameter("marker_topic").get_parameter_value().string_value

        self._eps_pixels = self.get_parameter("eps_pixels").get_parameter_value().double_value
        self._min_samples = self.get_parameter("min_samples").get_parameter_value().integer_value
        self._max_points = self.get_parameter("max_points").get_parameter_value().integer_value
        self._pixel_scale_m = self.get_parameter("pixel_scale_m").get_parameter_value().double_value
        self._use_ground_plane = (
            self.get_parameter("use_ground_plane").get_parameter_value().bool_value
        )
        self._camera_frame = self.get_parameter("camera_frame").get_parameter_value().string_value
        self._output_frame = self.get_parameter("output_frame").get_parameter_value().string_value
        self._fx = self.get_parameter("fx").get_parameter_value().double_value
        self._fy = self.get_parameter("fy").get_parameter_value().double_value
        self._cx = self.get_parameter("cx").get_parameter_value().double_value
        self._cy = self.get_parameter("cy").get_parameter_value().double_value
        self._k1 = self.get_parameter("k1").get_parameter_value().double_value
        self._k2 = self.get_parameter("k2").get_parameter_value().double_value
        self._p1 = self.get_parameter("p1").get_parameter_value().double_value
        self._p2 = self.get_parameter("p2").get_parameter_value().double_value
        self._k3 = self.get_parameter("k3").get_parameter_value().double_value
        self._undistort = self.get_parameter("undistort").get_parameter_value().bool_value
        self._undistort_iterations = (
            self.get_parameter("undistort_iterations").get_parameter_value().integer_value
        )
        self._plane_z = self.get_parameter("plane_z").get_parameter_value().double_value
        self._use_latest_tf = (
            self.get_parameter("use_latest_tf").get_parameter_value().bool_value
        )
        self._centroids_only = self.get_parameter("publish_centroids_only").get_parameter_value().bool_value
        self._log_stats = self.get_parameter("log_stats").get_parameter_value().bool_value
        self._use_depth = self.get_parameter("use_depth").get_parameter_value().bool_value
        self._depth_topic = self.get_parameter("depth_topic").get_parameter_value().string_value
        self._depth_scale = self.get_parameter("depth_scale").get_parameter_value().double_value
        self._depth_min = self.get_parameter("depth_min").get_parameter_value().double_value
        self._depth_max = self.get_parameter("depth_max").get_parameter_value().double_value
        self._depth_use_median = (
            self.get_parameter("depth_use_median").get_parameter_value().bool_value
        )

        self._bridge = CvBridge()
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._pc_pub = self.create_publisher(PointCloud2, pc_topic, 10)
        self._marker_pub = self.create_publisher(MarkerArray, marker_topic, 10)
        self.create_subscription(Image, mask_topic, self._on_mask, 10)
        self._depth_image = None
        if self._use_depth:
            self.create_subscription(Image, self._depth_topic, self._on_depth, 10)

    def _on_depth(self, msg: Image) -> None:
        depth_img = self._bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
        self._depth_image = depth_img

    def _on_mask(self, msg: Image) -> None:
        mask = self._bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
        if mask is None:
            return

        ys, xs = np.nonzero(mask)
        if ys.size == 0:
            if self._log_stats:
                self.get_logger().info("clusters: mask empty")
            self._publish_empty(msg.header)
            return

        if ys.size > self._max_points:
            idx = np.random.choice(ys.size, self._max_points, replace=False)
            ys = ys[idx]
            xs = xs[idx]

        points_px = np.column_stack((xs, ys)).astype(np.float32)
        labels = dbscan(points_px, self._eps_pixels, self._min_samples)

        if labels.size == 0:
            if self._log_stats:
                self.get_logger().info("clusters: no labels")
            self._publish_empty(msg.header)
            return

        cloud_points = []
        markers = MarkerArray()
        header = msg.header
        header.frame_id = self._output_frame

        unique_labels = [l for l in np.unique(labels) if l >= 0]
        if not unique_labels:
            if self._log_stats:
                self.get_logger().info("clusters: no clusters after filtering")
            self._publish_empty(header)
            return

        for i, label in enumerate(unique_labels):
            idx = labels == label
            cluster_px = points_px[idx]
            if cluster_px.size == 0:
                continue
            centroid_px = np.mean(cluster_px, axis=0)
            centroid_m = self._pixel_to_meters(centroid_px, cluster_px, mask.shape, header)
            if centroid_m is None:
                continue

            if self._centroids_only:
                cloud_points.append(centroid_m)
            else:
                for p in cluster_px:
                    point_m = self._pixel_to_meters(p, cluster_px, mask.shape, header)
                    if point_m is not None:
                        cloud_points.append(point_m)

            marker = Marker()
            marker.header = header
            marker.ns = "event_clusters"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = float(centroid_m[0])
            marker.pose.position.y = float(centroid_m[1])
            marker.pose.position.z = float(centroid_m[2])
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.r = 0.1
            marker.color.g = 0.9
            marker.color.b = 0.1
            marker.color.a = 0.8
            markers.markers.append(marker)

        cloud_msg = pc2.create_cloud_xyz32(header, cloud_points)
        self._pc_pub.publish(cloud_msg)
        self._marker_pub.publish(markers)
        if self._log_stats:
            self.get_logger().info(
                f"clusters: mask_points={ys.size} clusters={len(unique_labels)} "
                f"cloud_points={len(cloud_points)}")

    def _publish_empty(self, header) -> None:
        header.frame_id = self._output_frame
        cloud_msg = pc2.create_cloud_xyz32(header, [])
        self._pc_pub.publish(cloud_msg)
        self._marker_pub.publish(MarkerArray())

    def _pixel_to_meters(self, pixel_xy: np.ndarray, cluster_px, shape, header):
        if self._use_depth and self._depth_image is not None:
            point = self._pixel_to_depth(pixel_xy, cluster_px, shape, header)
            if point is not None:
                return point

        if self._use_ground_plane:
            point = self._pixel_to_ground(pixel_xy, header)
            if point is not None:
                return point

        return self._pixel_to_scaled(pixel_xy, shape)

    def _pixel_to_depth(self, pixel_xy: np.ndarray, cluster_px, shape, header):
        if self._depth_image is None:
            return None
        if self._depth_image.shape != shape:
            return None

        if self._depth_use_median and cluster_px is not None and cluster_px.size > 0:
            xs = cluster_px[:, 0].astype(np.int32)
            ys = cluster_px[:, 1].astype(np.int32)
            depths = self._depth_image[ys, xs]
            depths = depths[np.isfinite(depths)]
            if depths.size == 0:
                return None
            depth = float(np.median(depths))
        else:
            u = int(pixel_xy[0])
            v = int(pixel_xy[1])
            depth = float(self._depth_image[v, u])

        if not np.isfinite(depth):
            return None

        depth *= float(self._depth_scale)
        if depth < self._depth_min or depth > self._depth_max:
            return None

        x = (pixel_xy[0] - self._cx) / self._fx * depth
        y = (pixel_xy[1] - self._cy) / self._fy * depth
        z = depth
        return self._transform_point((x, y, z), header)

    def _transform_point(self, point_xyz, header):
        try:
            if self._use_latest_tf:
                transform = self._tf_buffer.lookup_transform(
                    self._output_frame, self._camera_frame, rclpy.time.Time()
                )
            else:
                transform = self._tf_buffer.lookup_transform(
                    self._output_frame,
                    self._camera_frame,
                    rclpy.time.Time.from_msg(header.stamp),
                )
        except Exception as ex:  # pylint: disable=broad-except
            if self._log_stats:
                self.get_logger().warn(f"TF lookup failed for depth: {ex}")
            return None

        tx = transform.transform.translation.x
        ty = transform.transform.translation.y
        tz = transform.transform.translation.z
        qx = transform.transform.rotation.x
        qy = transform.transform.rotation.y
        qz = transform.transform.rotation.z
        qw = transform.transform.rotation.w
        r = self._quat_to_rot(qx, qy, qz, qw)
        cam_point = np.array(point_xyz, dtype=np.float32)
        out = r.dot(cam_point) + np.array([tx, ty, tz], dtype=np.float32)
        return float(out[0]), float(out[1]), float(out[2])

    def _pixel_to_scaled(self, pixel_xy: np.ndarray, shape) -> tuple:
        height, width = shape
        x = (pixel_xy[0] - (width / 2.0)) * self._pixel_scale_m
        y = (pixel_xy[1] - (height / 2.0)) * self._pixel_scale_m
        return float(x), float(y), 0.0

    def _pixel_to_ground(self, pixel_xy: np.ndarray, header):
        try:
            if self._use_latest_tf:
                transform = self._tf_buffer.lookup_transform(
                    self._output_frame, self._camera_frame, rclpy.time.Time()
                )
            else:
                transform = self._tf_buffer.lookup_transform(
                    self._output_frame,
                    self._camera_frame,
                    rclpy.time.Time.from_msg(header.stamp),
                )
        except Exception as ex:  # pylint: disable=broad-except
            self.get_logger().warn(f"TF lookup failed, using scale fallback: {ex}")
            return None

        tx = transform.transform.translation.x
        ty = transform.transform.translation.y
        tz = transform.transform.translation.z
        qx = transform.transform.rotation.x
        qy = transform.transform.rotation.y
        qz = transform.transform.rotation.z
        qw = transform.transform.rotation.w

        r = self._quat_to_rot(qx, qy, qz, qw)
        ray_cam = np.array(
            [
                (pixel_xy[0] - self._cx) / self._fx,
                (pixel_xy[1] - self._cy) / self._fy,
                1.0,
            ],
            dtype=np.float32,
        )
        if self._undistort:
            xu, yu = self._undistort_normalized(ray_cam[0], ray_cam[1])
            ray_cam[0] = xu
            ray_cam[1] = yu
        ray_out = r.dot(ray_cam)

        if abs(ray_out[2]) < 1e-6 or ray_out[2] >= 0.0:
            return None

        t = (self._plane_z - tz) / ray_out[2]
        if t <= 0.0:
            return None

        x = tx + t * ray_out[0]
        y = ty + t * ray_out[1]
        z = self._plane_z
        return float(x), float(y), float(z)

    @staticmethod
    def _quat_to_rot(qx, qy, qz, qw) -> np.ndarray:
        r00 = 1.0 - 2.0 * (qy * qy + qz * qz)
        r01 = 2.0 * (qx * qy - qz * qw)
        r02 = 2.0 * (qx * qz + qy * qw)
        r10 = 2.0 * (qx * qy + qz * qw)
        r11 = 1.0 - 2.0 * (qx * qx + qz * qz)
        r12 = 2.0 * (qy * qz - qx * qw)
        r20 = 2.0 * (qx * qz - qy * qw)
        r21 = 2.0 * (qy * qz + qx * qw)
        r22 = 1.0 - 2.0 * (qx * qx + qy * qy)
        return np.array([[r00, r01, r02], [r10, r11, r12], [r20, r21, r22]], dtype=np.float32)

    def _undistort_normalized(self, xd: float, yd: float) -> tuple:
        xu = float(xd)
        yu = float(yd)
        for _ in range(max(1, self._undistort_iterations)):
            r2 = xu * xu + yu * yu
            r4 = r2 * r2
            r6 = r4 * r2
            radial = 1.0 + self._k1 * r2 + self._k2 * r4 + self._k3 * r6
            dx = 2.0 * self._p1 * xu * yu + self._p2 * (r2 + 2.0 * xu * xu)
            dy = self._p1 * (r2 + 2.0 * yu * yu) + 2.0 * self._p2 * xu * yu
            if radial == 0.0:
                break
            xu = (xd - dx) / radial
            yu = (yd - dy) / radial
        return xu, yu


def main() -> None:
    rclpy.init()
    node = EventClusteringNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
