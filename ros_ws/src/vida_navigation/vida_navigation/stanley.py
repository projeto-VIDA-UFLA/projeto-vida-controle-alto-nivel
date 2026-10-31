"""Seguidor de caminho Stanley.

    ros2 run vida_navigation stanley --ros-args -p path_file:=pista.csv

Lei de controle no eixo dianteiro (frente = traseiro + L (cos th, sin th)):

    delta = psi_e - atan(k e / (k_s + v)) + k_ff atan(L_ef kappa) - k_r (r - v kappa)

psi_e = rumo do caminho - rumo do veiculo; e = erro lateral, positivo com o
veiculo a ESQUERDA do caminho (por isso o sinal negativo: a esquerda, vira a
direita); kappa = curvatura do caminho no ponto projetado; L_ef = entre-eixos
efetivo (1,42 m medido no chao: o veiculo vira menos que a geometria preve).
Saida saturada em +-20 graus e com taxa limitada. Angulos em REP-103.

k_r amortece o giro em excesso (r = velocidade de giro medida; Stanley
original): no 1o teste (30/10) o comando oscilava +-10 graus a 0,7 Hz pelo
atraso de ~0,3 s do esterçamento.

Velocidade constante: abaixo de ~2 m/s a controladora de tracao
pulsa, entao nao ha desaceleracao gradual no fim; o Stanley pede freio total
quando a distancia restante chega a distancia de frenagem
(v^2 / 2 a_freio + v * atraso).

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
    wheelbase_m: float = 1.125           # geometrico: posicao do eixo dianteiro
    feedforward_wheelbase_m: float = 1.42  # efetivo, para o feedforward da curvatura
    gain_k: float = 2.0                  # 1/s; padrao para 2,5 m/s (simulador calibrado, 30/10)
    softening_m_s: float = 1.0           # k_s: evita ganho infinito parado
    feedforward_gain: float = 1.0
    yaw_damping_s: float = 0.25          # k_r (s); 0 desliga
    max_steering_deg: float = 20.0
    steering_rate_deg_s: float = 40.0
    speed_m_s: float = 2.5
    brake_decel_m_s2: float = 1.5        # freio total medido no plano (30/10)
    brake_delay_s: float = 0.2           # atraso ate o freio agir
    search_back_m: float = 2.0
    search_ahead_m: float = 10.0
    start_search_ahead_m: float = 25.0   # na habilitacao (manobra a 2 m/s ja feita)


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
    brake_command: float = 0.0


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
        self.stopping = False
        return proj

    def stop(self):
        self.active = False

    def update(self, x, y, yaw, speed, dt, yaw_rate=None):
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
        term_ff = p.feedforward_gain * math.atan(p.feedforward_wheelbase_m * proj.curvature)
        term_damping = 0.0
        if yaw_rate is not None and p.yaw_damping_s > 0.0:
            term_damping = -p.yaw_damping_s * (yaw_rate - max(speed, 0.0) * proj.curvature)
        desired = math.degrees(term_heading + term_cross + term_ff + term_damping)
        limited = float(np.clip(desired, -p.max_steering_deg, p.max_steering_deg))
        step = p.steering_rate_deg_s * dt
        self.command += float(np.clip(limited - self.command, -step, step))

        v = max(speed, 0.0)
        stopping_distance = v * v / (2.0 * p.brake_decel_m_s2) + v * p.brake_delay_s
        brake_cmd = 0.0
        if self.stopping or remaining <= stopping_distance:
            # Uma vez iniciada, a parada nao volta atras (o restante continua caindo).
            self.stopping = True
            state, speed_cmd, brake_cmd = StanleyStatus.STATE_STOPPING, 0.0, 1.0
            if speed < 0.05:
                state, self.finished = StanleyStatus.STATE_FINISHED, True
        else:
            state, speed_cmd = StanleyStatus.STATE_TRACKING, p.speed_m_s
        if self.finished:
            state, speed_cmd, brake_cmd = StanleyStatus.STATE_FINISHED, 0.0, 1.0

        return StanleyOutput(
            state=state, path_index=proj.index, path_s_m=proj.s, progress_m=self.progress,
            remaining_m=max(remaining, 0.0), cross_track_m=proj.cross_track,
            heading_error_rad=heading_error, path_curvature_1_m=proj.curvature,
            steering_heading_deg=math.degrees(term_heading),
            steering_cross_track_deg=math.degrees(term_cross),
            steering_feedforward_deg=math.degrees(term_ff),
            steering_command_deg=self.command, steering_saturated=abs(desired) > p.max_steering_deg,
            speed_command_m_s=speed_cmd, brake_command=brake_cmd)


class StanleyNode(Node):
    def __init__(self):
        super().__init__("stanley")
        dp = self.declare_parameter
        self.path_file = PathFile.load(dp("path_file", "").value)
        self.params = StanleyParams(
            wheelbase_m=dp("wheelbase_m", 1.125).value,
            feedforward_wheelbase_m=dp("feedforward_wheelbase_m", 1.42).value,
            gain_k=dp("gain_k", 2.0).value,
            softening_m_s=dp("softening_m_s", 1.0).value,
            feedforward_gain=dp("feedforward_gain", 1.0).value,
            yaw_damping_s=dp("yaw_damping_s", 0.25).value,
            max_steering_deg=dp("max_steering_deg", 20.0).value,
            steering_rate_deg_s=dp("steering_rate_deg_s", 40.0).value,
            speed_m_s=dp("speed_m_s", 2.5).value,
            brake_decel_m_s2=dp("brake_decel_m_s2", 1.5).value,
            brake_delay_s=dp("brake_delay_s", 0.2).value)
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
                       odom.twist.twist.linear.x, dt, yaw_rate=odom.twist.twist.angular.z)
        self.last_output = out

        stamp = now.to_msg()
        steering = SteeringReference()
        steering.timestamp = stamp
        steering.angle_deg = float(out.steering_command_deg)
        self.steering_pub.publish(steering)
        drive = DriveReference()
        drive.timestamp = stamp
        drive.velocity_m_s = float(out.speed_command_m_s)
        drive.brake_command = float(out.brake_command)
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
