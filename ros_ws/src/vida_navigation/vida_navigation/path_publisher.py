"""Publica um caminho (CSV lat/lon) como nav_msgs/Path no frame local do EKF.

Reconverte sempre que o EKF anuncia uma nova origem (boot ou reset). Usado
para visualizacao (RViz) e conferencia; o Stanley e o mode_manager leem o CSV
diretamente.
"""
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from vida_interfaces.msg import EkfStatus

from vida_navigation.path import PathFile
from vida_navigation.ros_common import LATCHED_QOS, origin_from_status


class PathPublisher(Node):
    def __init__(self):
        super().__init__("path_publisher")
        self.path_file = PathFile.load(self.declare_parameter("path_file", "").value)
        self.frame_id = self.declare_parameter("frame_id", "odom").value
        status_topic = self.declare_parameter("status_topic", "/robot_state/status").value
        self.publisher = self.create_publisher(
            Path, self.declare_parameter("path_topic", "/navigation/path").value, LATCHED_QOS)
        self.create_subscription(EkfStatus, status_topic, self.on_status, 10)
        self.origin_id = None
        self.get_logger().info(
            f"Caminho '{self.path_file.meta.get('name', '')}': {len(self.path_file.latitude_deg)} pontos, "
            f"{'fechado' if self.path_file.closed else 'aberto'}; aguardando a origem do EKF")

    def on_status(self, status):
        origin = origin_from_status(status)
        if origin is None or status.origin_id == self.origin_id:
            return
        self.origin_id = status.origin_id
        local = self.path_file.to_local(origin)
        msg = Path()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        xs, ys = list(local.x), list(local.y)
        if local.closed:
            xs.append(xs[0])
            ys.append(ys[0])
        for x, y in zip(xs, ys):
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        self.publisher.publish(msg)
        self.get_logger().info(f"Caminho publicado na origem #{status.origin_id}")


def main():
    rclpy.init()
    node = PathPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
