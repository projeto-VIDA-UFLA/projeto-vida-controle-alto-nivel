"""Grava um caminho dirigindo com o controle (posicao estimada pelo EKF).

    ros2 run vida_navigation path_recorder --ros-args -p output_file:=volta_bruta.csv -p closed:=true

Salva um ponto a cada spacing_m percorrido, so com o EKF rodando com heading
do GPS e (por padrao) RTK FIXED. O arquivo e salvo ao chamar ~/save ou ao
encerrar o no (Ctrl+C); depois trate com:

    ros2 run vida_navigation path_tools from-recording volta_bruta.csv pista.csv
"""
import math

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_srvs.srv import Trigger
from vida_interfaces.msg import EkfStatus, GnssStatus

from vida_navigation.geo import to_geodetic
from vida_navigation.path import PathFile
from vida_navigation.ros_common import ekf_ready, origin_from_status


class PathRecorder(Node):
    def __init__(self):
        super().__init__("path_recorder")
        self.output_file = self.declare_parameter("output_file", "volta_bruta.csv").value
        self.spacing = self.declare_parameter("spacing_m", 0.25).value
        self.closed = self.declare_parameter("closed", True).value
        self.name = self.declare_parameter("name", "").value
        self.require_fixed = self.declare_parameter("require_rtk_fixed", True).value
        self.create_subscription(Odometry, self.declare_parameter("odom_topic", "/robot_state/odom").value,
                                 self.on_odom, 50)
        self.create_subscription(EkfStatus, self.declare_parameter("status_topic", "/robot_state/status").value,
                                 self.on_status, 10)
        self.create_service(Trigger, "~/save", self.on_save)
        self.status = None
        self.origin_id = None
        self.points = []          # (lat, lon)
        self.last_xy = None
        self.skipped_not_fixed = 0
        self.in_gap = False
        self.gaps = []            # tamanho (m) de cada lacuna sem FIXED
        self.saved = False
        self.get_logger().info(f"Gravando caminho em '{self.output_file}' (aguardando o EKF)")

    def on_status(self, status):
        self.status = status

    def on_odom(self, odom):
        status = self.status
        if status is None or not ekf_ready(status):
            return
        if self.origin_id is None:
            self.origin_id = status.origin_id
            self.get_logger().info(f"EKF pronto (origem #{status.origin_id}): gravando")
        elif status.origin_id != self.origin_id:
            self.get_logger().error("A origem do EKF mudou durante a gravacao: gravacao interrompida")
            self.origin_id = -1
            return
        if self.origin_id == -1:
            return
        if self.require_fixed and status.gps_rtk_status != GnssStatus.RTK_FIXED:
            if self.skipped_not_fixed == 0 or not self.in_gap:
                self.get_logger().warn("RTK sem FIXED: trecho nao gravado (o caminho tera uma lacuna; "
                                       "repita a volta se ela passar de 1 m)")
            self.in_gap = True
            self.skipped_not_fixed += 1
            return

        x, y = odom.pose.pose.position.x, odom.pose.pose.position.y
        if self.last_xy is not None and math.hypot(x - self.last_xy[0], y - self.last_xy[1]) < self.spacing:
            return
        if self.in_gap and self.last_xy is not None:
            gap = math.hypot(x - self.last_xy[0], y - self.last_xy[1])
            self.get_logger().warn(f"RTK FIXED de volta: lacuna de {gap:.1f} m na gravacao")
            self.gaps.append(gap)
        self.in_gap = False
        self.last_xy = (x, y)
        lat, lon = to_geodetic(origin_from_status(status), x, y)
        self.points.append((float(lat), float(lon)))
        if len(self.points) % 200 == 0:
            self.get_logger().info(f"{len(self.points)} pontos ({len(self.points) * self.spacing:.0f} m)")

    def save(self):
        if len(self.points) < 4:
            return False, f"apenas {len(self.points)} pontos gravados"
        data = np.array(self.points)
        PathFile(data[:, 0], data[:, 1], self.closed, {
            "name": self.name or "gravacao",
            "source": "ekf_recording_raw",
            "spacing_m": f"{self.spacing}",
            "skipped_not_fixed": f"{self.skipped_not_fixed}",
            "gaps_m": " ".join(f"{g:.1f}" for g in self.gaps) or "nenhuma",
        }).save(self.output_file)
        self.saved = True
        message = f"{len(self.points)} pontos salvos em {self.output_file}"
        if self.gaps:
            message += f" ({len(self.gaps)} lacuna(s) sem FIXED, maior {max(self.gaps):.1f} m)"
        return True, message

    def on_save(self, request, response):
        response.success, response.message = self.save()
        self.get_logger().info(response.message)
        return response


def main():
    rclpy.init()
    node = PathRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if not node.saved:
            ok, message = node.save()
            print(message)
        node.destroy_node()
        rclpy.try_shutdown()
