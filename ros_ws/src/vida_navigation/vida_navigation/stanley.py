"""Seguidor de caminho Stanley.

    ros2 run vida_navigation stanley --ros-args -p path_file:=pista.csv

Lei de controle no eixo dianteiro (frente = traseiro + L (cos th, sin th)):

    delta = psi_e - atan(k e / (k_s + v)) + k_ff atan(L kappa)

psi_e = rumo do caminho - rumo do veiculo; e = erro lateral, positivo com o
veiculo a ESQUERDA do caminho (por isso o sinal negativo: a esquerda, vira a
direita); kappa = curvatura do caminho no ponto projetado. Saida saturada em
+-20 graus e com taxa limitada. Angulos em REP-103 (esquerda positivo).

So calcula com o no habilitado (~/enable, chamado pelo mode_manager). Publica
sugestoes em /navigation/stanley/{steering,drive}; quem publica as referencias
do veiculo e o mode_manager. Uma volta no caminho fechado, ou ate o fim do
aberto, e para.
"""
import math
from dataclasses import dataclass

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_srvs.srv import SetBool
from vida_interfaces.msg import DriveReference, EkfStatus, StanleyStatus, SteeringReference

from vida_navigation.geo import wrap_angle
from vida_navigation.path import PathFile
from vida_navigation.ros_common import origin_from_status, yaw_from_quaternion


@dataclass
class StanleyParams:
    wheelbase_m: float = 1.125
    gain_k: float = 2.0                  # 1/s (simulacao: 2 equilibra erro e ruido)
    softening_m_s: float = 1.0           # k_s: evita ganho infinito parado
    feedforward_gain: float = 1.0
    max_steering_deg: float = 20.0
    steering_rate_deg_s: float = 40.0
    speed_m_s: float = 1.0
    end_decel_m_s2: float = 0.3          # desaceleracao planejada no fim
    min_speed_m_s: float = 0.25          # ate chegar ao fim (abaixo disso o PI nao segue)
    stop_tolerance_m: float = 0.3
    search_back_m: float = 2.0
    search_ahead_m: float = 10.0
    start_search_ahead_m: float = 15.0   # na habilitacao (manobra de ~3 m ja feita)


@dataclass
class StanleyOutput:
    state: int
    path_index: int = 0
    path_s_m: float = 0.0
    progress_m: float = 0.0
    remaining_m: float = 0.0
    cross_track_m: float = 0.0
    heading_error_rad: float = 0.0
    path_curvature_1_m: float = 0.0
    steering_heading_deg: float = 0.0
    steering_cross_track_deg: float = 0.0
    steering_feedforward_deg: float = 0.0
    steering_command_deg: float = 0.0
    steering_saturated: bool = False
    speed_command_m_s: float = 0.0


class StanleyController:
    def __init__(self, path, params=None):
        self.path = path
        self.p = params or StanleyParams()
        self.active = False
        self.finished = False

    def front_axle(self, x, y, yaw):
        L = self.p.wheelbase_m
        return x + L * math.cos(yaw), y + L * math.sin(yaw)

    def start(self, x, y, yaw, steering_deg=0.0):
        fx, fy = self.front_axle(x, y, yaw)
        proj = self.path.project(fx, fy, hint=0, back_m=self.p.search_back_m,
                                 ahead_m=self.p.start_search_ahead_m)
        self.hint = proj.index
        self.last_s = proj.s
        self.progress = 0.0
        # Fechado: uma volta, termina no ponto de partida do caminho. Aberto: no fim.
        self.target = self.path.length - proj.s
        self.command = float(steering_deg)
        self.active = True
        self.finished = False
        return proj

    def stop(self):
        self.active = False

    def update(self, x, y, yaw, speed, dt):
        p = self.p
        fx, fy = self.front_axle(x, y, yaw)
        proj = self.path.project(fx, fy, hint=self.hint, back_m=p.search_back_m, ahead_m=p.search_ahead_m)
        self.hint = proj.index

        ds = proj.s - self.last_s
        if self.path.closed:
            ds = (ds + self.path.length / 2) % self.path.length - self.path.length / 2
        self.last_s = proj.s
        self.progress += ds
        remaining = self.target - self.progress

        heading_error = float(wrap_angle(proj.heading - yaw))
        term_heading = heading_error
        term_cross = -math.atan(p.gain_k * proj.cross_track / (p.softening_m_s + max(speed, 0.0)))
        term_ff = p.feedforward_gain * math.atan(p.wheelbase_m * proj.curvature)
        desired = math.degrees(term_heading + term_cross + term_ff)
        limited = float(np.clip(desired, -p.max_steering_deg, p.max_steering_deg))
        step = p.steering_rate_deg_s * dt
        self.command += float(np.clip(limited - self.command, -step, step))

        if remaining <= p.stop_tolerance_m:
            state, speed_cmd = StanleyStatus.STATE_STOPPING, 0.0
            if speed < 0.05:
                state, self.finished = StanleyStatus.STATE_FINISHED, True
        else:
            braking = math.sqrt(2.0 * p.end_decel_m_s2 * remaining)
            speed_cmd = max(min(p.speed_m_s, braking), p.min_speed_m_s)
            state = StanleyStatus.STATE_TRACKING if braking >= p.speed_m_s else StanleyStatus.STATE_STOPPING
        if self.finished:
            state, speed_cmd = StanleyStatus.STATE_FINISHED, 0.0

        return StanleyOutput(
            state=state, path_index=proj.index, path_s_m=proj.s, progress_m=self.progress,
            remaining_m=max(remaining, 0.0), cross_track_m=proj.cross_track,
            heading_error_rad=heading_error, path_curvature_1_m=proj.curvature,
            steering_heading_deg=math.degrees(term_heading),
            steering_cross_track_deg=math.degrees(term_cross),
            steering_feedforward_deg=math.degrees(term_ff),
            steering_command_deg=self.command, steering_saturated=abs(desired) > p.max_steering_deg,
            speed_command_m_s=speed_cmd)


class StanleyNode(Node):
    def __init__(self):
        super().__init__("stanley")
        dp = self.declare_parameter
        self.path_file = PathFile.load(dp("path_file", "").value)
        self.params = StanleyParams(
            wheelbase_m=dp("wheelbase_m", 1.125).value,
            gain_k=dp("gain_k", 2.0).value,
            softening_m_s=dp("softening_m_s", 1.0).value,
            feedforward_gain=dp("feedforward_gain", 1.0).value,
            max_steering_deg=dp("max_steering_deg", 20.0).value,
            steering_rate_deg_s=dp("steering_rate_deg_s", 40.0).value,
            speed_m_s=dp("speed_m_s", 1.0).value,
            end_decel_m_s2=dp("end_decel_m_s2", 0.3).value)
        self.odom_timeout = dp("odom_timeout_s", 0.2).value
        self.path_name = self.path_file.meta.get("name", "")

        self.create_subscription(Odometry, dp("odom_topic", "/robot_state/odom").value, self.on_odom, 10)
        self.create_subscription(EkfStatus, dp("status_topic", "/robot_state/status").value, self.on_status, 10)
        self.steering_pub = self.create_publisher(SteeringReference, "/navigation/stanley/steering", 10)
        self.drive_pub = self.create_publisher(DriveReference, "/navigation/stanley/drive", 10)
        self.status_pub = self.create_publisher(StanleyStatus, "/navigation/stanley/status", 10)
        self.create_service(SetBool, "~/enable", self.on_enable)
        self.create_timer(0.1, self.on_timer)

        self.controller = None
        self.origin_id = None
        self.odom = None
        self.odom_time = None
        self.last_update = None
        self.last_output = StanleyOutput(state=StanleyStatus.STATE_IDLE)
        self.get_logger().info(f"Stanley: caminho '{self.path_name}', k={self.params.gain_k}, "
                               f"{self.params.speed_m_s} m/s; aguardando habilitacao")

    def _odom_fresh(self):
        return self.odom_time is not None and \
            (self.get_clock().now() - self.odom_time).nanoseconds * 1e-9 < self.odom_timeout

    def on_status(self, status):
        origin = origin_from_status(status)
        if origin is None or status.origin_id == self.origin_id:
            return
        if self.controller is not None and self.controller.active:
            self.get_logger().error("Origem do EKF mudou com o Stanley ativo: desabilitado")
            self.controller.stop()
            self.last_output = StanleyOutput(state=StanleyStatus.STATE_NO_ODOM)
        self.origin_id = status.origin_id
        self.controller = StanleyController(self.path_file.to_local(origin), self.params)
        self.odom = None
        self.odom_time = None

    def on_enable(self, request, response):
        if not request.data:
            if self.controller is not None:
                self.controller.stop()
            self.last_output = StanleyOutput(state=StanleyStatus.STATE_IDLE)
            response.success, response.message = True, "Stanley desabilitado"
        elif self.controller is None or not self._odom_fresh():
            response.success, response.message = False, "sem origem do EKF ou odom atrasada"
        else:
            o = self.odom.pose.pose
            proj = self.controller.start(o.position.x, o.position.y, yaw_from_quaternion(o.orientation))
            self.last_update = self.get_clock().now()
            response.success = True
            response.message = (f"Stanley habilitado em s={proj.s:.1f} m, lateral {proj.cross_track:+.2f} m, "
                                f"{self.controller.target:.1f} m ate o fim")
        self.get_logger().info(response.message)
        return response

    def on_odom(self, odom):
        self.odom = odom
        self.odom_time = self.get_clock().now()
        c = self.controller
        if c is None or not c.active:
            return
        now = self.odom_time
        dt = min((now - self.last_update).nanoseconds * 1e-9, 0.1)
        self.last_update = now
        o = odom.pose.pose
        out = c.update(o.position.x, o.position.y, yaw_from_quaternion(o.orientation),
                       odom.twist.twist.linear.x, dt)
        self.last_output = out

        stamp = now.to_msg()
        steering = SteeringReference()
        steering.timestamp = stamp
        steering.angle_deg = float(out.steering_command_deg)
        self.steering_pub.publish(steering)
        drive = DriveReference()
        drive.timestamp = stamp
        drive.velocity_m_s = float(out.speed_command_m_s)
        self.drive_pub.publish(drive)
        self.publish_status(out, stamp)

    def on_timer(self):
        c = self.controller
        if c is not None and c.active and not self._odom_fresh():
            self.last_output = StanleyOutput(state=StanleyStatus.STATE_NO_ODOM)
            self.publish_status(self.last_output, self.get_clock().now().to_msg())
        elif c is None or not c.active:
            self.publish_status(self.last_output, self.get_clock().now().to_msg())

    def publish_status(self, out, stamp):
        msg = StanleyStatus()
        msg.timestamp = stamp
        msg.state = out.state
        msg.path_name = self.path_name
        msg.path_index = int(out.path_index)
        for name in ("path_s_m", "progress_m", "remaining_m", "cross_track_m", "heading_error_rad",
                     "path_curvature_1_m", "steering_heading_deg", "steering_cross_track_deg",
                     "steering_feedforward_deg", "steering_command_deg", "speed_command_m_s"):
            setattr(msg, name, float(getattr(out, name)))
        msg.steering_saturated = bool(out.steering_saturated)
        self.status_pub.publish(msg)


def main():
    rclpy.init()
    node = StanleyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
